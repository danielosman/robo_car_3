#pragma once
// Where the robot was at a given moment: PicoB's odometry reports of the last
// ~1.3 s, interpolated, on PicoA's clock (time_us_32()). Lets a sensor frame be
// placed where the robot was when it was measured, not when it was read.
// Frame: odometry's (where PicoB started), also the map's. Yaw is not wrapped.
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float x_m, y_m;
    float yaw_rad;   // + = left; not wrapped
    float pitch_rad; // + = nose up
} pose_t;

void pose_update(void); // call every loop iteration: takes body's new reports
bool pose_now(pose_t *pose); // the latest report; false before the first one
// false if t_us is before the history or more than 50 ms after the latest report
// (after that, the robot's motion is extrapolated from its last speeds).
bool pose_at(uint32_t t_us, pose_t *pose);
