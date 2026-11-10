#pragma once
// How the robot has moved since power-up, from the gyro (heading) and the four
// wheel encoders (distance: each side is the average of its front and rear wheel), plus tilt from the accelerometer and gyro, and
// whether the robot is standing still. While still, heading is frozen and the
// gyro's bias is re-measured, so standing still costs no heading drift. Also the
// IMU as it measured, for the PC: each 4 samples (416 a second) averaged into one
// imu_record_t (104 a second; link_messages.h).
// Robot frame: x forward, y left, z up. Angles: + = turned left, nose up, left side up.
#include <stdbool.h>
#include "encoder.h"
#include "link_messages.h"
#include "mounting.h"

typedef struct {
    float x_m, y_m;          // in the frame the robot had at power-up
    float yaw_rad;           // not wrapped: a full left turn adds 2π
    float v_mps, w_radps;    // forward speed, turn rate
    float pitch_rad, roll_rad;
    float wheel_left_mps, wheel_right_mps; // each side, front and rear averaged, + = forward
    float wheel_mps[ENCODER_COUNT];            // each wheel on its own, + = forward
    bool stationary;         // wheels and gyro still for 0.5 s
    float gyro_bias_radps;   // subtracted from the yaw gyro: power-up calibration plus re-measurements while still
} odometry_t;

// Starts the IMU and measures the gyro bias: the robot must stand still for ~1 s.
// Returns false if the IMU doesn't answer; odometry then stays at zero. The IMU's
// axes come from the mounting (main.c: ROBOT_MOUNTING).
bool odometry_init(const mounting_t *m);
// Call as often as possible: it uses every IMU sample (416 per second) and
// returns at once when there's no new one.
void odometry_update(void);
const odometry_t *odometry_get(void);
// The oldest imu record not yet taken (t_us on PicoB's clock, time_us_32()); false if
// none waits. The newest ODOMETRY_IMU_KEPT are kept (take them every loop: one is made
// every 9.6 ms); older ones not taken are lost.
#define ODOMETRY_IMU_KEPT 8
bool odometry_imu_take(imu_record_t *r);
uint32_t odometry_imu_lost(void); // records lost (not taken in time) since power-up
uint32_t odometry_imu_skipped(void); // partial records dropped after a skipped sample (a slow loop) since power-up
