#pragma once
#include <stdbool.h>

typedef struct { float ax, ay, az; /* g */ float gx, gy, gz; /* deg/s */ } imu_sample_t;

bool imu_init(void);                 // false if WHO_AM_I is wrong
bool imu_read(imu_sample_t *s);      // false if no new gyro sample yet
void imu_calibrate_gyro(int samples); // keep still; averages gyro bias
float imu_gyro_bias_z(void);         // deg/s, sensor frame: what imu_calibrate_gyro() measured
