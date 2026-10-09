#pragma once
// Where the robot was at a given moment: PicoB's odometry reports of the last
// ~1.3 s, interpolated, on PicoA's clock (time_us_32()). Lets a sensor frame be
// placed where the robot was when it was measured, not when it was read.
// Frame: the map's. pose_set_origin() makes the robot's current position and
// heading the origin and x axis (the start-up scan does); until then it is
// odometry's (where PicoB started). Yaw is not wrapped.
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float x_m, y_m;
    float yaw_rad;   // + = left; not wrapped
    float pitch_rad; // + = nose up
} pose_t;

void pose_update(void); // call every loop iteration: takes body's new reports
void pose_set_origin(void); // from now on, poses are relative to where the robot is now
uint32_t pose_origin_changes(void); // how often the origin was set: frames kept in the map's frame are void after a change
bool pose_now(pose_t *pose); // the latest report; false before the first one
// false if t_us is before the history or more than 50 ms after the latest report
// (after that, the robot's motion is extrapolated from its last speeds).
bool pose_at(uint32_t t_us, pose_t *pose);
