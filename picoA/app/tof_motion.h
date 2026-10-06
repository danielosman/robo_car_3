#pragma once
// Movement in front of the VL53L8CX while the robot stands still (ROBOT_PLAN.md
// §6.2). Each zone uses the frame's nearest sure target, as the map does, or
// nothing; in a floor zone, readings beyond its floor are reflections and tell
// nothing. After tof_motion_restart() each zone learns its background over 1 s: the
// nearest distance it gives regularly (nothing, if it had no sure reading). From
// then on a zone has moved when it reads clearly closer than its background (8 cm,
// or 8 % if more) in 2 of the last 4 frames next to another such zone, or in 3 alone;
// moved zones that touch are reported as observations. A zone that reads nearer
// briefly without moving, then goes back, shows a nearer surface now and then: that
// is its background from then on. Farther readings (something taken away, a
// reflection) are never movement: after 10 s in a row they become the background.
// A zone that reads closer but steady for 1 s has stopped moving (a box put down):
// that distance is its background from then on. Movement is change.
// While learning, something passing out of the view (the target leaving while the
// robot learns after a turn): in an outer zone column, 2 zones in one frame read
// clearly nearer than the farthest the zone gives regularly (its 3rd-farthest sure
// distance: it was gone for more than 2 frames).
#include <stdbool.h>
#include "rangefinder.h"
#include "motion_obs.h"

void tof_motion_restart(void); // the view changed (the robot moved): learn the backgrounds again
bool tof_motion_ready(void);   // the backgrounds are learned
// Feeds one frame; fills up to max observations, the most zones first, and returns
// how many (none while learning).
int tof_motion_add(const range_frame_t *frame, motion_obs_t *obs, int max);
// Once learned: whether something passed out of the view on the left (or right)
// while learning, its direction (+ = left) and the last frame it was seen in there.
bool tof_motion_passed(bool left, float *bearing_rad, uint32_t *t_us);

// For printing, per zone: whether it moved, and its background (mm, 0 = nothing).
bool tof_motion_moved(int zone);
uint16_t tof_motion_background_mm(int zone);
