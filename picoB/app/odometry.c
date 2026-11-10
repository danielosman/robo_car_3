#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "imu.h"
#include "encoder.h"
#include "odometry.h"
#include "units.h"

#define WHEEL_DIAMETER_M  0.090f
#define M_PER_COUNT       (PI_F * WHEEL_DIAMETER_M / ENCODER_COUNTS_PER_REV)
#define MAX_DT_S          0.1f   // longer gaps (a stalled loop) are not trusted

#define WHEEL_SPEED_TAU_S 0.03f  // low-pass on wheel speeds from encoder steps
#define TILT_TAU_S        0.5f   // complementary filter: gyro below, gravity above
#define STILL_SPEED_MPS   0.005f
#define STILL_TURN_DPS    3.0f   // excludes being turned by hand with the wheels still
#define STILL_TIME_S      0.5f
#define BIAS_TAU_S        5.0f   // gyro bias tracking while stationary

static odom_t o;
static bool imu_ok, have_tilt;
static float yaw_bias_dps;       // on top of the power-up calibration in imu.c
static float still_s;
static int32_t last_count[ENC_COUNT];
static uint64_t last_us;

// How the ISM330DHCX breakout is mounted (common/mounting.json: Z up, X backward, Y
// right, turned 180° about Z). Verified in the M0 test: nose up raises pitch, left
// side up raises roll, turning left raises yaw.
static int8_t robot_from_sensor[3][3];

// The imu records: the samples summed so far (accel x, y, z, gyro x, y, z) and when
// they were read; the records made, made - taken of them waiting.
static int32_t sum[6];
static int summed;
static uint32_t first_us, offsets_us; // the first sample's time; the others' times after it, added
static imu_record_t made[ODOM_IMU_KEPT];
static uint32_t made_n, taken_n, lost_n;
static uint32_t skipped_n; // partial records dropped after a skipped sample
static uint32_t last_sample_us;
// A sample this long after the one before: one (or more) was skipped (3605 µs, 1.5
// sample times).
#define SAMPLE_GAP_US (3 * 1000000 / 416 / 2)

static void to_robot_frame(const imu_sample_t *s, float accel_g[3], float gyro_dps[3]) {
    const float a[3] = {s->ax, s->ay, s->az}, g[3] = {s->gx, s->gy, s->gz};
    for (int i = 0; i < 3; i++) {
        accel_g[i] = gyro_dps[i] = 0;
        for (int k = 0; k < 3; k++) {
            accel_g[i] += (float)robot_from_sensor[i][k] * a[k];
            gyro_dps[i] += (float)robot_from_sensor[i][k] * g[k];
        }
    }
}

bool odom_init(const mounting_t *m) {
    memcpy(robot_from_sensor, m->imu.robot_from_sensor, sizeof robot_from_sensor);
    imu_ok = imu_init();
    if (imu_ok) imu_calibrate_gyro(400); // ~1 s at 416 Hz
    encoder_init();
    for (int i = 0; i < ENC_COUNT; i++) last_count[i] = encoder_count(i);
    last_us = time_us_64();
    return imu_ok;
}

static void update_wheels(float dt) {
    float step_m[ENC_COUNT];
    float k = dt / (WHEEL_SPEED_TAU_S + dt);
    for (int i = 0; i < ENC_COUNT; i++) {
        int32_t count = encoder_count(i);
        step_m[i] = (float)(count - last_count[i]) * M_PER_COUNT;
        last_count[i] = count;
        o.wheel_mps[i] += k * (step_m[i] / dt - o.wheel_mps[i]);
    }
    // Both wheels of a side roll together (skid steering), so averaging them halves
    // the counting steps and evens out a wheel that slips for a moment.
    float dl = 0.5f * (step_m[ENC_LEFT_FRONT] + step_m[ENC_LEFT_REAR]);
    float dr = 0.5f * (step_m[ENC_RIGHT_FRONT] + step_m[ENC_RIGHT_REAR]);
    o.wheel_left_mps = 0.5f * (o.wheel_mps[ENC_LEFT_FRONT] + o.wheel_mps[ENC_LEFT_REAR]);
    o.wheel_right_mps = 0.5f * (o.wheel_mps[ENC_RIGHT_FRONT] + o.wheel_mps[ENC_RIGHT_REAR]);
    o.v_mps = 0.5f * (o.wheel_left_mps + o.wheel_right_mps);

    // Distance from the encoders, direction from the gyro heading halfway through the step.
    float d = 0.5f * (dl + dr);
    float yaw_mid = o.yaw_rad - 0.5f * o.w_radps * dt;
    o.x_m += d * cosf(yaw_mid);
    o.y_m += d * sinf(yaw_mid);
}

static void update_heading(float yaw_rate_raw_dps, float dt) {
    float rate_dps = yaw_rate_raw_dps - yaw_bias_dps;
    bool still = fabsf(o.wheel_left_mps) < STILL_SPEED_MPS &&
                 fabsf(o.wheel_right_mps) < STILL_SPEED_MPS &&
                 fabsf(rate_dps) < STILL_TURN_DPS;
    still_s = still ? still_s + dt : 0;
    o.stationary = still_s >= STILL_TIME_S;
    if (o.stationary) {
        yaw_bias_dps += dt / BIAS_TAU_S * (yaw_rate_raw_dps - yaw_bias_dps);
        rate_dps = 0;
    }
    // The IMU is mounted Z up, so its z bias is the robot's yaw bias with the same sign.
    o.gyro_bias_radps = (imu_gyro_bias_z() + yaw_bias_dps) * RAD_PER_DEG;
    o.w_radps = rate_dps * RAD_PER_DEG;
    o.yaw_rad += o.w_radps * dt;
}

static void update_tilt(const float accel_g[3], const float gyro_dps[3], float dt) {
    // At rest the accelerometer reads +1 g along whichever axis points up:
    // nose up gives +x, left side up gives +y.
    float accel_pitch = atan2f(accel_g[0], accel_g[2]);
    float accel_roll = atan2f(accel_g[1], accel_g[2]);
    if (!have_tilt) {
        o.pitch_rad = accel_pitch;
        o.roll_rad = accel_roll;
        have_tilt = true;
        return;
    }
    float w = dt / (TILT_TAU_S + dt);
    // Nose up is a negative rotation about the left (y) axis; left side up is positive about x.
    o.pitch_rad = (1 - w) * (o.pitch_rad - gyro_dps[1] * RAD_PER_DEG * dt) + w * accel_pitch;
    o.roll_rad = (1 - w) * (o.roll_rad + gyro_dps[0] * RAD_PER_DEG * dt) + w * accel_roll;
}

// The average of IMU_RECORD_SAMPLES counts that add up to total: the nearest count,
// halves away from zero. Always within int16: the sum of 4 int16 is at most 4 × 32767.
static int16_t average(int32_t total) {
    const int32_t n = IMU_RECORD_SAMPLES;
    return (int16_t)(total >= 0 ? (total + n / 2) / n : (total - n / 2) / n);
}

// One sample's raw counts, read at t_us: each IMU_RECORD_SAMPLES-th makes a record, stamped
// with the middle of its samples' times (the clock's wrap included: uint32 differences).
static void update_record(const imu_counts_t *c, uint32_t t_us) {
    // A sample skipped (a slow loop): the partial record goes, so every record is 4
    // consecutive samples and the skip shows as a gap in the records' times.
    if (summed > 0 && t_us - last_sample_us > SAMPLE_GAP_US) { memset(sum, 0, sizeof sum); summed = 0; skipped_n++; }
    last_sample_us = t_us;
    if (summed == 0) { first_us = t_us; offsets_us = 0; }
    offsets_us += t_us - first_us;
    for (int i = 0; i < 3; i++) {
        sum[i] += c->accel[i];
        sum[3 + i] += c->gyro[i];
    }
    if (++summed < IMU_RECORD_SAMPLES) return;
    imu_record_t *r = &made[made_n++ % ODOM_IMU_KEPT];
    r->t_us = first_us + (offsets_us + IMU_RECORD_SAMPLES / 2) / IMU_RECORD_SAMPLES;
    for (int i = 0; i < 3; i++) {
        r->accel[i] = average(sum[i]);
        r->gyro[i] = average(sum[3 + i]);
    }
    memset(sum, 0, sizeof sum);
    summed = 0;
    if (made_n - taken_n > ODOM_IMU_KEPT) { // the oldest not taken are lost, counted
        lost_n += made_n - taken_n - ODOM_IMU_KEPT;
        taken_n = made_n - ODOM_IMU_KEPT;
    }
}

uint32_t odom_imu_lost(void) { return lost_n; }

uint32_t odom_imu_skipped(void) { return skipped_n; }

bool odom_imu_take(imu_record_t *r) {
    if (taken_n == made_n) return false;
    *r = made[taken_n++ % ODOM_IMU_KEPT];
    return true;
}

void odom_update(void) {
    imu_sample_t s;
    imu_counts_t counts;
    if (!imu_ok || !imu_read(&s, &counts)) return;
    update_record(&counts, time_us_32());
    uint64_t now = time_us_64();
    float dt = (float)(now - last_us) * 1e-6f;
    last_us = now;
    if (dt <= 0) return;
    if (dt > MAX_DT_S) dt = MAX_DT_S;

    float accel_g[3], gyro_dps[3];
    to_robot_frame(&s, accel_g, gyro_dps);
    update_heading(gyro_dps[2], dt); // first: the wheel step uses this step's turn
    update_wheels(dt);
    update_tilt(accel_g, gyro_dps, dt);
}

const odom_t *odom_get(void) { return &o; }
