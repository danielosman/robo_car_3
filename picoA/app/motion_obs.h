#pragma once
// One place where something moved, as a movement detector saw it in one frame
// (the VL53's in tof_motion, the camera's later): what the detectors have in common.
#include <stdint.h>

typedef struct {
    float where[3];  // unit vector from the sensor to it, robot frame (x forward, y left, z up)
    float range_m;   // < 0: unknown (camera; a VL53 blob of zones that lost their target)
    float point[3];  // robot frame, metres; only with a range
    float strength;  // share of the sensor's cells that moved
    int cells;
    uint32_t t_us;   // PicoA's clock
} motion_obs_t;
