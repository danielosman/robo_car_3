#include <math.h>
#include "pico/stdlib.h"
#include "imu.h"
#include "encoder.h"
#include "odometry.h"

#define WHEEL_DIAMETER_M  0.090f
#define M_PER_COUNT       (3.14159265f * WHEEL_DIAMETER_M / ENCODER_COUNTS_PER_REV)
#define DEG_TO_RAD        0.017453293f
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
static int32_t last_left, last_right;
static uint64_t last_us;

// How the ISM330DHCX breakout is mounted: Z up, X backward, Y right (turned 180°
// about Z). Verified in the M0 test: nose up raises pitch, left side up raises
// roll, turning left raises yaw.
static void to_robot_frame(const imu_sample_t *s, float accel_g[3], float gyro_dps[3]) {
    accel_g[0] = -s->ax; accel_g[1] = -s->ay; accel_g[2] = s->az;
    gyro_dps[0] = -s->gx; gyro_dps[1] = -s->gy; gyro_dps[2] = s->gz;
}

bool odom_init(void) {
    imu_ok = imu_init();
    if (imu_ok) imu_calibrate_gyro(400); // ~1 s at 416 Hz
    encoder_init();
    last_left = encoder_count(ENC_LEFT_FRONT);
    last_right = encoder_count(ENC_RIGHT_FRONT);
    last_us = time_us_64();
    return imu_ok;
}

static void update_wheels(float dt) {
    int32_t left = encoder_count(ENC_LEFT_FRONT), right = encoder_count(ENC_RIGHT_FRONT);
    float dl = (float)(left - last_left) * M_PER_COUNT;
    float dr = (float)(right - last_right) * M_PER_COUNT;
    last_left = left;
    last_right = right;
    float k = dt / (WHEEL_SPEED_TAU_S + dt);
    o.wheel_left_mps += k * (dl / dt - o.wheel_left_mps);
    o.wheel_right_mps += k * (dr / dt - o.wheel_right_mps);
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
    o.w_radps = rate_dps * DEG_TO_RAD;
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
    o.pitch_rad = (1 - w) * (o.pitch_rad - gyro_dps[1] * DEG_TO_RAD * dt) + w * accel_pitch;
    o.roll_rad = (1 - w) * (o.roll_rad + gyro_dps[0] * DEG_TO_RAD * dt) + w * accel_roll;
}

void odom_update(void) {
    imu_sample_t s;
    if (!imu_ok || !imu_read(&s)) return;
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
