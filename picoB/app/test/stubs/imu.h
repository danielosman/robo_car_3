#pragma once
// Fake IMU: the test sets the next sample and fake_imu_fresh; imu_read() returns
// it once. Using it before imu_init() fails the test.
#include <assert.h>
#include <stdbool.h>
typedef struct { float ax, ay, az; float gx, gy, gz; } imu_sample_t;
static imu_sample_t fake_imu = {0, 0, 1, 0, 0, 0};
static bool fake_imu_fresh, fake_imu_started;
static float fake_imu_bias_z;
static inline bool imu_init(void) { fake_imu_started = true; return true; }
static inline void imu_calibrate_gyro(int samples) { assert(fake_imu_started); (void)samples; }
static inline float imu_gyro_bias_z(void) { assert(fake_imu_started); return fake_imu_bias_z; }
static inline bool imu_read(imu_sample_t *s) {
    assert(fake_imu_started);
    if (!fake_imu_fresh) return false;
    fake_imu_fresh = false;
    *s = fake_imu;
    return true;
}
