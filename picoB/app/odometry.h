#pragma once
// How the robot has moved since power-up, from the gyro (heading) and the front
// wheel encoders (distance), plus tilt from the accelerometer and gyro, and
// whether the robot is standing still. While still, heading is frozen and the
// gyro's bias is re-measured, so standing still costs no heading drift.
// Robot frame: x forward, y left, z up. Angles: + = turned left, nose up, left side up.
#include <stdbool.h>

typedef struct {
    float x_m, y_m;          // in the frame the robot had at power-up
    float yaw_rad;           // not wrapped: a full left turn adds 2π
    float v_mps, w_radps;    // forward speed, turn rate
    float pitch_rad, roll_rad;
    float wheel_left_mps, wheel_right_mps; // front wheels, + = forward
    bool stationary;         // wheels and gyro still for 0.5 s
} odom_t;

// Starts the IMU and measures the gyro bias: the robot must stand still for ~1 s.
// Returns false if the IMU doesn't answer; odometry then stays at zero.
bool odom_init(void);
// Call as often as possible: it uses every IMU sample (416 per second) and
// returns at once when there's no new one.
void odom_update(void);
const odom_t *odom_get(void);
