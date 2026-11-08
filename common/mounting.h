#pragma once
// Where the sensors sit on the robot. The values come from common/mounting.json: the
// build turns it into robot_mounting.h (ROBOT_MOUNTING, common/mounting.cmake), which
// only main.c includes; it passes the value to the code that needs it. Host tests
// pass their own. Robot frame: x forward from the centre, y left, z up from the floor.
#include <stdint.h>

typedef struct {
    float x_m, y_m, z_m;  // the sensor
    float zone_deg;       // one zone's angle, up-down and across
    int zones_turn_deg;   // the zone grid turned on the PCB, in quarter turns (90 as built)
} mounting_tof_t;

typedef struct {
    float x_m, y_m, z_m;  // the lens
    float f_px;           // focal length in the picture's pixels
    float tilt_deg;       // its own aim, + = up, on top of the robot's pitch
    int width_px, height_px;
} mounting_camera_t;

typedef struct {
    // The IMU's axes turned into the robot's: robot = robot_from_sensor × sensor, for
    // the accelerometer and the gyro alike.
    int8_t robot_from_sensor[3][3];
} mounting_imu_t;

typedef struct {
    mounting_tof_t tof;
    mounting_camera_t camera;
    mounting_imu_t imu;
} mounting_t;
