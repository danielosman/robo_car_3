#pragma once
#include <stdbool.h>
#include <stdint.h>

// One IMU reading (imu_sample): g and deg/s, sensor frame, the gyro's power-up bias taken out.
typedef struct { float ax, ay, az; /* g */ float gx, gy, gz; /* deg/s */ } imu_sample_t;
// The same reading as the sensor gave it: its own int16 counts, nothing taken out.
typedef struct { int16_t acceleration[3], gyro[3]; } imu_counts_t;

// The counts' scale (±2 g, ±250 deg/s; ST DS13012 table 3).
#define IMU_ACCELERATION_G_PER_LSB  0.000061f // 0.061 mg a count
#define IMU_GYRO_DPS_PER_LSB 0.00875f  // 8.75 mdps a count

bool imu_init(void);                 // false if WHO_AM_I is wrong
// The next sample (416 a second): false if no new gyro sample yet. counts (may be
// NULL) gets the same reading as raw counts.
bool imu_read(imu_sample_t *s, imu_counts_t *counts);
void imu_calibrate_gyro(int samples); // keep still; averages gyro bias
float imu_gyro_bias_z(void);         // deg/s, sensor frame: what imu_calibrate_gyro() measured
