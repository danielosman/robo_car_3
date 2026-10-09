#pragma once
// Movement in front of the VL53L8CX, while the robot stands still and while it
// turns in place (TOF_MOTION_PLAN.md §7). Each zone uses the frame's nearest sure
// target, as the map does, or nothing; in a floor zone, readings beyond its floor
// are reflections and tell nothing.
//
// The background is kept per world direction, not per zone: for every zone row, 256
// bins of 1.4° around the robot, each the distance up to which that direction is
// known to be free. A zone reports the nearest surface in its cone, so its reading
// is a lower bound for every bin its cone covered during the whole frame; two
// agreeing readings fill or raise a bin. A zone is predicted from the bins it swept
// over during the frame (the turn from pose, a little wider for heading error): the
// nearest of them. It has moved when it reads clearly closer than that (8 cm, or
// 8 % if more) in 2 of the last 4 frames next to another such zone, or in 3 alone;
// moved zones that touch are reported as observations. So there is no learning after
// a stop: a still view is known after 2 frames, and after a turn (the start-up scan)
// every direction is.
//
// Rules kept from watching while still: a zone that reads nearer briefly without
// moving, then goes back, shows a nearer surface now and then; one that reads closer
// but steady for 1 s has stopped (a box put down). Both lower the bins under it to
// that distance and pin them: farther readings raise pinned bins only after 10 s.
// Farther readings are never movement. While turning, the floor rows tell nothing
// (the robot rolls a few degrees, which moves the floor in them a lot). Once the
// robot has moved 10 cm all bins are forgotten (driving is not watched).
#include <stdbool.h>
#include "rangefinder.h"
#include "motion_obs.h"

// The robot during a frame (the map's frame, from pose_at()): its heading at the
// start and the end of the measurement, and where it was.
typedef struct {
    float yaw_start_rad, yaw_end_rad;
    float x_m, y_m;
} tof_view_t;

void tof_motion_restart(void); // forgets everything (a new map origin)
bool tof_motion_ready(void);   // the view in front is known
// Feeds one frame measured from `view`; fills up to max observations, the most zones
// first, and returns how many.
int tof_motion_add(const range_frame_t *frame, const tof_view_t *view, motion_obs_t *obs, int max);

// For printing, per zone: whether it moved, and its background in the last frame
// (mm, 0 = nothing or not known).
bool tof_motion_moved(int zone);
uint16_t tof_motion_background_mm(int zone);
