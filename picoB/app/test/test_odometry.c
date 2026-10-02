// Host test for picoB/app/odometry.c: straight driving, turning in place, gyro
// bias while standing still, and tilt, from simulated IMU samples and encoder
// counts at the IMU's 416 Hz. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -IpicoB/app/test/stubs -o build/test_odometry picoB/app/test/test_odometry.c -lm && build/test_odometry
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../odometry.c"

#define RATE_HZ 416.0f
#define RAD(d)  ((d) * 0.017453293f)
#define DEG(r)  ((r) * 57.29578f)

static float gyro_bias_dps;      // the real sensor's leftover bias
static double count_l, count_r;  // exact simulated encoder positions

// Simulates the robot for `seconds`: wheel speeds in m/s, true turn rate in deg/s.
static void simulate(float seconds, float left_mps, float right_mps, float turn_dps) {
    int n = (int)(seconds * RATE_HZ + 0.5f);
    for (int i = 0; i < n; i++) {
        fake_now_us += (uint64_t)(1e6f / RATE_HZ);
        count_l += left_mps / RATE_HZ / M_PER_COUNT;
        count_r += right_mps / RATE_HZ / M_PER_COUNT;
        fake_counts[ENC_LEFT_FRONT] = (int32_t)count_l;
        fake_counts[ENC_RIGHT_FRONT] = (int32_t)count_r;
        fake_imu.gz = turn_dps + gyro_bias_dps;
        fake_imu_fresh = true;
        odom_update();
    }
}

int main(void) {
    const odom_t *o = odom_get();
    fake_now_us = 1000000;
    assert(odom_init());

    simulate(1, 0, 0, 0);
    assert(o->stationary);

    // 50 cm straight at 0.1 m/s.
    simulate(5, 0.1f, 0.1f, 0);
    assert(!o->stationary);
    assert(fabsf(o->x_m - 0.5f) < 0.005f && fabsf(o->y_m) < 0.001f);
    assert(fabsf(o->v_mps - 0.1f) < 0.002f);
    printf("straight 50 cm: x %.1f cm, y %.1f cm\n", (double)(o->x_m * 100), (double)(o->y_m * 100));

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

    printf("OK: odometry\n");
    return 0;
}
