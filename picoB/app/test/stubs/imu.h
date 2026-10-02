#pragma once
// Fake IMU: the test sets the next sample; imu_read() returns it once per call to fake_imu_tick().
#include <stdbool.h>
typedef struct { float ax, ay, az; float gx, gy, gz; } imu_sample_t;
static imu_sample_t fake_imu = {0, 0, 1, 0, 0, 0};
static bool fake_imu_fresh;
static inline bool imu_init(void) { return true; }
static inline void imu_calibrate_gyro(int samples) { (void)samples; }
static inline bool imu_read(imu_sample_t *s) { if (!fake_imu_fresh) return false; fake_imu_fresh = false; *s = fake_imu; return true; }
