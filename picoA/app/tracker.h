#pragma once
// One target among the VL53's movement observations (ROBOT_PLAN.md §6.6). The
// tracker follows one target: the biggest blob at first; afterwards the biggest blob
// that reaches within 10° of where the target is expected (its last direction moved
// on by its angular speed) and is not more than 0.5 m nearer or farther (a person
// close by is a big blob and pieces). It keeps the target's direction, range and
// angular speed (over the last ~0.8 s). With no such blob for 0.5 s it is lost:
// last seen in the outer zone columns, it left the view on that side; else it
// stopped (it stands still now, or is too small to see). Before it leaves: seen in
// an outer zone column moving outward (at least 10°/s, its speed measured over at
// least 3 frames), it is leaving that way, reported once per target.
#include <stdbool.h>
#include <stdint.h>
#include "motion_obs.h"

typedef struct {
    float bearing_rad; // + = left of straight ahead, from the sensor
    float up_rad;      // + = above level
    float range_m;     // the last known; < 0: not known yet
    float rate_radps;  // angular speed, + = to the left
    uint32_t last_seen_us;
} target_t;

typedef enum {
    TRACK_NOTHING,      // nothing new: following, or nothing to follow
    TRACK_NEW,          // a new target
    TRACK_EXITED_LEFT,  // the target left the view on the left
    TRACK_EXITED_RIGHT,
    TRACK_STOPPED,      // the target was lost inside the view
    TRACK_LEAVING_LEFT, // the target is about to leave the view on the left
    TRACK_LEAVING_RIGHT,
} track_event_t;

void tracker_reset(void); // forgets the target (the robot moved: the view changed)
// One frame's observations (the most cells first) at t_us; returns what happened.
// After TRACK_EXITED_* and TRACK_STOPPED, tracker_target() still gives the lost
// target until the next call.
track_event_t tracker_add(const motion_obs_t *obs, int n, uint32_t t_us);
const target_t *tracker_target(void); // NULL when there is none
