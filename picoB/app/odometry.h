#pragma once
// How the robot has moved since power-up, from the gyro (heading) and the four
// wheel encoders (distance: each side is the average of its front and rear wheel), plus tilt from the accelerometer and gyro, and
// whether the robot is standing still. While still, heading is frozen and the
// gyro's bias is re-measured, so standing still costs no heading drift.
// Robot frame: x forward, y left, z up. Angles: + = turned left, nose up, left side up.
#include <stdbool.h>
#include "encoder.h"

typedef struct {
    float x_m, y_m;          // in the frame the robot had at power-up
    float yaw_rad;           // not wrapped: a full left turn adds 2π
    float v_mps, w_radps;    // forward speed, turn rate
    float pitch_rad, roll_rad;
    float wheel_left_mps, wheel_right_mps; // each side, front and rear averaged, + = forward
    float wheel_left_m, wheel_right_m;     // each side's total distance, + = forward
    float wheel_mps[ENC_COUNT];            // each wheel on its own, + = forward
    bool stationary;         // wheels and gyro still for 0.5 s
    float gyro_bias_radps;   // subtracted from the yaw gyro: power-up calibration plus re-measurements while still
} odom_t;

// Starts the IMU and measures the gyro bias: the robot must stand still for ~1 s.
// Returns false if the IMU doesn't answer; odometry then stays at zero.
bool odom_init(void);
// Call as often as possible: it uses every IMU sample (416 per second) and
// returns at once when there's no new one.
void odom_update(void);
const odom_t *odom_get(void);
