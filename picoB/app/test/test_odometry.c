// Host test for picoB/app/odometry.c: straight driving, turning in place, gyro
// bias while standing still, and tilt, from simulated IMU samples and encoder
// counts at the IMU's 416 Hz; the imu records (4 samples averaged: rounding, sign,
// the int16 limits, every axis its own; 104 a second, stamped with the middle of
// their samples, across the clock's wrap; raw counts; a skipped sample starts a new
// record; the newest 8 kept, the lost counted);
// the IMU's axes turned into the robot's by the mounting
// (common/mounting.json, through build/host/robot_mounting.h). Run from the repo root
// (run_tests.sh does; it makes the header first).
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../odometry.c"
#include "robot_mounting.h"

#define RATE_HZ 416.0f
#define RAD(d)  ((d) * RAD_PER_DEG)
#define DEG(r)  ((r) * DEG_PER_RAD)

static float gyro_bias_dps;      // the real sensor's leftover bias
static double count[ENC_COUNT];  // exact simulated encoder positions
static float rear_left_slip = 1; // the rear-left wheel turns this much faster than its side

// Simulates the robot for `seconds`: wheel speeds in m/s, true turn rate in deg/s.
static void simulate(float seconds, float left_mps, float right_mps, float turn_dps) {
    int n = (int)(seconds * RATE_HZ + 0.5f);
    float wheel_mps[ENC_COUNT] = {
        [ENC_LEFT_FRONT] = left_mps, [ENC_LEFT_REAR] = left_mps * rear_left_slip,
        [ENC_RIGHT_FRONT] = right_mps, [ENC_RIGHT_REAR] = right_mps,
    };
    for (int i = 0; i < n; i++) {
        fake_now_us += (uint64_t)(1e6f / RATE_HZ);
        for (int w = 0; w < ENC_COUNT; w++) {
            count[w] += (double)(wheel_mps[w] / RATE_HZ / M_PER_COUNT);
            fake_counts[w] = (int32_t)count[w];
        }
        fake_imu.gz = turn_dps + gyro_bias_dps;
        fake_imu_fresh = true;
        odom_update();
    }
}

// One IMU sample step_us after the one before, with these raw counts.
static void one_sample(const imu_counts_t *c, uint32_t step_us) {
    fake_now_us += step_us;
    fake_imu_counts = *c;
    fake_imu_fresh = true;
    odom_update();
}

// Takes every record waiting, then feeds samples until one more is made: the next
// sample starts a record.
static void align_records(void) {
    imu_record_t r;
    const imu_counts_t zero = {{0}, {0}};
    while (odom_imu_take(&r)) {}
    while (!odom_imu_take(&r)) one_sample(&zero, 2404);
}

// The record made of four samples whose accel x and gyro z are v[0..3] (the other
// axes fixed); checks the other axes kept their own counts.
static int16_t average_of(int16_t v0, int16_t v1, int16_t v2, int16_t v3, int16_t *gyro_z) {
    const int16_t v[4] = {v0, v1, v2, v3};
    imu_record_t r;
    for (int k = 0; k < 4; k++) {
        imu_counts_t c = {{v[k], 100, -16384}, {-7, 32767, v[k]}};
        assert(!odom_imu_take(&r));
        one_sample(&c, 2404);
    }
    assert(odom_imu_take(&r) && !odom_imu_take(&r));
    assert(r.accel[1] == 100 && r.accel[2] == -16384 && r.gyro[0] == -7 && r.gyro[1] == 32767);
    *gyro_z = r.gyro[2];
    return r.accel[0];
}

static void test_imu_records(void) {
    align_records();
    // The nearest count, halves away from zero, the same for every sign and axis.
    const struct { int16_t v[4], avg; } cases[] = {
        {{1, 2, 3, 4}, 3},          {{-1, -2, -3, -4}, -3},   // 2.5 -> 3, -2.5 -> -3
        {{0, 0, 0, 1}, 0},          {{0, 0, 0, -1}, 0},       // 0.25 -> 0
        {{0, 0, 1, 1}, 1},          {{0, 0, -1, -1}, -1},     // 0.5 -> 1, -0.5 -> -1
        {{0, 1, 1, 1}, 1},          {{0, -1, -1, -1}, -1},    // 0.75 -> 1
        {{5, 5, 5, 5}, 5},          {{-3, 3, -3, 3}, 0},
        {{32767, 32767, 32767, 32767}, 32767},               // the int16 limits, no overflow
        {{-32768, -32768, -32768, -32768}, -32768},
        {{32767, 32767, 32767, 32766}, 32767},               // 32766.75
        {{-32768, -32768, -32768, -32767}, -32768},          // -32767.75
        {{32767, -32768, 32767, -32768}, -1},                // -0.5
        {{16384, 16380, 16390, 16386}, 16385},               // 1 g on an axis, a little noise
    };
    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        int16_t gz, ax = average_of(cases[i].v[0], cases[i].v[1], cases[i].v[2], cases[i].v[3], &gz);
        if (ax != cases[i].avg || gz != cases[i].avg) { printf("case %u: %d, %d not %d\n", i, ax, gz, cases[i].avg); assert(0); }
    }
    printf("imu record: 4 samples averaged, halves away from zero, every axis its own, the int16 limits kept\n");

    // Raw counts: the gyro's bias (taken out of the float sample odometry uses) stays
    // in the record, and what the float sample says doesn't change it.
    fake_imu_bias_z = 0.5f;
    fake_imu.gz = 123;
    int16_t gz;
    assert(average_of(7, 7, 7, 7, &gz) == 7 && gz == 7);
    fake_imu_bias_z = 0;
    fake_imu.gz = 0;

    // 416 samples a second make 104 records, 9.6 ms apart, each stamped with the
    // middle of its four samples' times (2404 µs apart: 1.5 steps after the first).
    imu_record_t r[ODOM_IMU_KEPT];
    const imu_counts_t still = {{0, 0, 16384}, {3, -2, 1}};
    uint32_t first_sample_us = 0;
    int made = 0;
    for (int i = 0; i < 416; i++) {
        one_sample(&still, 2404);
        if (i % 4 == 0) first_sample_us = time_us_32();
        imu_record_t got;
        if (odom_imu_take(&got)) {
            assert(i % 4 == 3 && got.t_us == first_sample_us + 3606); // (0 + 2404 + 4808 + 7212) / 4
            if (made) assert(got.t_us - r[0].t_us == 4 * 2404);
            r[0] = got;
            made++;
        }
    }
    assert(made == 104);
    // Across the clock's wrap: the samples' times go from 2^32 - 3000 µs to past 0.
    fake_now_us = FAKE_CLOCK_WRAP_US - 3000 - 2404;
    first_sample_us = (uint32_t)(fake_now_us + 2404);
    for (int k = 0; k < 4; k++) one_sample(&still, 2404);
    assert(odom_imu_take(&r[0]) && r[0].t_us == first_sample_us + 3606 && r[0].t_us < 1000);
    printf("imu records: 104 a second, stamped with the middle of their samples, across the clock's wrap\n");

    // A skipped sample (a slow loop, 2 sample times without a read): the partial record
    // is dropped and a new one starts, so each record is 4 consecutive samples and
    // the skip shows as a gap between records.
    imu_counts_t one = {{1, 0, 0}, {0, 0, 0}}, nine = {{9, 0, 0}, {0, 0, 0}};
    uint32_t skipped0 = odom_imu_skipped();
    one_sample(&one, 2404);
    one_sample(&one, 2404);
    one_sample(&nine, 2 * 2404);                       // one sample missed before it
    first_sample_us = time_us_32();
    for (int k = 0; k < 3; k++) one_sample(&nine, 2404);
    assert(odom_imu_take(&r[0]) && r[0].accel[0] == 9 && r[0].t_us == first_sample_us + 3606 && !odom_imu_take(&r[0]));
    assert(odom_imu_skipped() == skipped0 + 1); // the dropped partial record is counted
    // Just under 1.5 sample times is still the next sample.
    one_sample(&one, 2404);
    one_sample(&one, 3600);
    one_sample(&nine, 2404);
    one_sample(&nine, 2404);
    assert(odom_imu_take(&r[0]) && r[0].accel[0] == 5 && !odom_imu_take(&r[0]));
    assert(odom_imu_skipped() == skipped0 + 1); // 3600 µs is no skip
    printf("imu records: a skipped sample starts a new record (a gap in the times)\n");

    // Not taken: the newest ODOM_IMU_KEPT wait, oldest first; the older are lost, counted.
    uint32_t lost0 = odom_imu_lost();
    for (int n = 0; n < ODOM_IMU_KEPT + 3; n++) {
        imu_counts_t c = {{(int16_t)n, 0, 0}, {0, 0, 0}};
        for (int k = 0; k < 4; k++) one_sample(&c, 2404);
    }
    for (int n = 3; n < ODOM_IMU_KEPT + 3; n++) assert(odom_imu_take(&r[0]) && r[0].accel[0] == n);
    assert(!odom_imu_take(&r[0]) && odom_imu_lost() == lost0 + 3);
    printf("imu records: the newest %d kept until taken\n", ODOM_IMU_KEPT);
}

int main(void) {
    const odom_t *o = odom_get();
    fake_now_us = FAKE_CLOCK_START_US + 1000000;
    // The mounting's matrix is what turns the axes: here one turned 90° about z
    // (the sensor's x is the robot's y).
    imu_sample_t s = {.ax = 0.1f, .ay = 0.2f, .az = 0.9f, .gx = 1, .gy = 2, .gz = 3};
    float accel_g[3], gyro_dps[3];
    mounting_t turned = ROBOT_MOUNTING;
    const int8_t quarter[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
    memcpy(turned.imu.robot_from_sensor, quarter, sizeof quarter);
    assert(odom_init(&turned));
    to_robot_frame(&s, accel_g, gyro_dps);
    assert(accel_g[0] == -0.2f && accel_g[1] == 0.1f && accel_g[2] == 0.9f);
    assert(gyro_dps[0] == -2 && gyro_dps[1] == 1 && gyro_dps[2] == 3);
    assert(odom_init(&ROBOT_MOUNTING));
    // The breakout as built: turned 180° about z, its x and y point backward and right.
    to_robot_frame(&s, accel_g, gyro_dps);
    assert(accel_g[0] == -0.1f && accel_g[1] == -0.2f && accel_g[2] == 0.9f);
    assert(gyro_dps[0] == -1 && gyro_dps[1] == -2 && gyro_dps[2] == 3);

    simulate(1, 0, 0, 0);
    assert(o->stationary);

    // 50 cm straight at 0.1 m/s.
    simulate(5, 0.1f, 0.1f, 0);
    assert(!o->stationary);
    assert(fabsf(o->x_m - 0.5f) < 0.005f && fabsf(o->y_m) < 0.001f);
    assert(fabsf(o->v_mps - 0.1f) < 0.002f);
    assert(fabsf(o->wheel_left_m - 0.5f) < 0.001f && fabsf(o->wheel_right_m - 0.5f) < 0.001f);
    assert(fabsf(o->wheel_mps[ENC_LEFT_REAR] - 0.1f) < 0.002f && fabsf(o->wheel_mps[ENC_RIGHT_REAR] - 0.1f) < 0.002f);
    printf("straight 50 cm: x %.1f cm, y %.1f cm\n", (double)(o->x_m * 100), (double)(o->y_m * 100));

    // A side's distance is the average of its front and rear wheel: the rear-left
    // wheel spinning 10 % faster (slipping) moves the left side 5 % further.
    simulate(1, 0, 0, 0);
    float left0 = o->wheel_left_m, right0 = o->wheel_right_m, x_before = o->x_m;
    rear_left_slip = 1.1f;
    simulate(1, 0.1f, 0.1f, 0);
    assert(fabsf(o->wheel_left_m - left0 - 0.105f) < 0.001f && fabsf(o->wheel_right_m - right0 - 0.1f) < 0.001f);
    assert(fabsf(o->wheel_left_mps - 0.105f) < 0.002f && fabsf(o->wheel_mps[ENC_LEFT_REAR] - 0.11f) < 0.002f);
    assert(fabsf(o->x_m - x_before - 0.1025f) < 0.001f);
    rear_left_slip = 1;
    printf("rear-left wheel 10 %% faster: left side %.1f cm for 10 cm\n", (double)((o->wheel_left_m - left0) * 100));

    // Turn left in place, 90 deg at 30 deg/s (wheels opposite): position stays put.
    simulate(1, 0, 0, 0);
    float x0 = o->x_m, y0 = o->y_m;
    simulate(3, -0.06f, 0.06f, 30);
    assert(fabsf(DEG(o->yaw_rad) - 90) < 0.5f);
    assert(fabsf(o->x_m - x0) < 0.002f && fabsf(o->y_m - y0) < 0.002f);
    printf("turn left 90 deg: yaw %.2f deg, moved %.2f cm\n", (double)DEG(o->yaw_rad),
           (double)(hypotf(o->x_m - x0, o->y_m - y0) * 100));

    // Then 50 cm forward: now along +y.
    simulate(1, 0, 0, 0);
    simulate(5, 0.1f, 0.1f, 0);
    assert(fabsf(o->y_m - y0 - 0.5f) < 0.01f && fabsf(o->x_m - x0) < 0.01f);

    // A gyro bias appears (temperature): standing still for a minute doesn't drift,
    // and the bias is learned, so the next 90 deg turn still measures 90 deg.
    gyro_bias_dps = 0.5f;
    float yaw0 = o->yaw_rad;
    simulate(60, 0, 0, 0);
    float still_drift = DEG(o->yaw_rad - yaw0);
    assert(fabsf(still_drift) < 0.5f);
    yaw0 = o->yaw_rad;
    simulate(3, -0.06f, 0.06f, 30);
    float turn_error = DEG(o->yaw_rad - yaw0) - 90;
    assert(fabsf(turn_error) < 0.3f);
    // The reported bias is the power-up calibration plus what was learned since.
    assert(fabsf(DEG(o->gyro_bias_radps) - 0.5f) < 0.01f);
    fake_imu_bias_z = 0.2f;
    simulate(0.01f, 0, 0, 0);
    assert(fabsf(DEG(o->gyro_bias_radps) - 0.7f) < 0.01f);
    fake_imu_bias_z = 0;
    printf("gyro bias 0.5 deg/s: drift over 60 s still %.2f deg, next 90 deg turn off by %.2f deg "
           "(without bias tracking: 30 deg and 1.5 deg)\n", (double)still_drift, (double)turn_error);
    gyro_bias_dps = 0;

    // Tilt: nose up 5 deg, then left side up 5 deg. The IMU is mounted turned 180°
    // (X backward, Y right), so nose up reads -X and left side up reads -Y.
    fake_imu.ax = -sinf(RAD(5)); fake_imu.az = cosf(RAD(5));
    simulate(3, 0, 0, 0);
    assert(fabsf(DEG(o->pitch_rad) - 5) < 0.1f && fabsf(DEG(o->roll_rad)) < 0.1f);
    fake_imu.ax = 0; fake_imu.ay = -sinf(RAD(5));
    simulate(3, 0, 0, 0);
    assert(fabsf(DEG(o->roll_rad) - 5) < 0.1f && fabsf(DEG(o->pitch_rad)) < 0.1f);
    printf("tilt: nose up -> pitch +, left side up -> roll +\n");

    test_imu_records();

    printf("OK: odometry\n");
    return 0;
}
