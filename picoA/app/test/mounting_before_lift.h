#pragma once
// The mounting before the PCB was lifted by 2 cm (Nov 2026): the ToF 7 cm above the
// floor, the camera 8.8 cm, tilted −1.86°. Every recording so far was made with it, so
// the tests and tools that replay recordings pass this one, not ROBOT_MOUNTING.
#include "mounting.h"

static const mounting_t MOUNTING_BEFORE_LIFT = {
    .tof = {.x_m = 0.032f, .y_m = -0.026f, .z_m = 0.07f, .zone_deg = 5.625f, .zones_turn_deg = 90},
    .camera = {.x_m = 0.025f, .y_m = -0.005f, .z_m = 0.088f, .f_px = 188.0f, .tilt_deg = -1.86f, .width_px = 160, .height_px = 120},
    .imu = {.robot_from_sensor = {{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}}},
};
