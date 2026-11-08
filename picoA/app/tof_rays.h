#pragma once
// Where the ToF's zones look (doc/ARCHITECTURE.md, the ToF pipeline): the zone
// angles and where the sensor sits (the mounting, given at init), and where a
// zone's cone meets the floor. For a posed frame it places a zone's reading in the
// world: its rays, their end point and heights, and a verdict on what it saw:
// floor, an obstacle, or unsure. What the readings mean for the map is cell_map's,
// for watching watch's. Pure geometry: no sensor, no clock (the PC's helper compiles
// it too).
//
// Directions are unit vectors, x forward, y left, z up. A zone at row index r,
// column index c points (r - 3.5) zone angles below level and (3.5 - c) to the left.
#include <stdbool.h>
#include "mounting.h"
#include "tof_frame.h"

#define TOF_RAYS_N 9 // rays across a zone's cone: 3 x 3, at 1/6, 1/2 and 5/6 of it each way

// A reading ending this low is on the floor (the map's ground layer's top).
#define TOF_FLOOR_TOP_M 0.03f
// A reading closer than this share of where its zone's cone first meets the floor
// is an obstacle (10 % for pitch and noise). Changed only to try other values in replay.
#define TOF_FLOOR_MARGIN 0.9f

typedef enum {
    TOF_VERDICT_FLOOR,    // every ray ends at floor height
    TOF_VERDICT_OBSTACLE, // closer than any floor the cone can see: something stands there
    TOF_VERDICT_UNSURE,   // its rays straddle the floor's height (grazing the floor, or a low thing on it)
} tof_verdict_t;

// A zone's reading placed in the world (the "rays" of doc/DATA_FLOW_PLAN.md).
typedef struct {
    float origin[3];           // the sensor, in the world (z above the floor)
    float dir[TOF_RAYS_N][3];  // the rays, in the world (pitch and yaw applied); k = down third * 3 + left third
    float length_m;            // along each ray
    float point[3];            // where the centre ray ends
    float low_m, high_m;       // the lowest and highest ray end (height above the floor)
    tof_verdict_t verdict;
} tof_rays_t;

// The mounting (main.c: ROBOT_MOUNTING; host tests their own). Call first; copied.
void tof_rays_init(const mounting_t *m);
void tof_rays_set_floor_margin(float margin); // TOF_FLOOR_MARGIN until changed (replay)

// A zone's reading `length_m` long, from the robot where frame `f` was measured.
void tof_rays_of(const posed_frame_t *f, int zone, float length_m, tof_rays_t *out);

// In the robot frame, the robot level (for watching):
void tof_rays_origin(float origin[3]); // the sensor (z above the floor)
void tof_rays_direction(int zone, float dir[3]); // the zone's centre ray
float tof_rays_zone_rad(void); // one zone's angle
// The row index from which down the rows look at the floor (rows 5-8; watch ignores
// them while turning). Not the same as floor_limit_m() > 0: row 5's upper edge is
// level, so it has no limit, but it looks down.
int tof_rays_first_floor_row(void);
// Only for the host tests' simulated sensors (the robot's code uses tof_rays_of()): a
// ray inside the zone, down_frac and left_frac (−0.5…0.5) of it from its centre
// (+ = down, left), with the robot pitched nose up by pitch_rad.
void tof_rays_sub_direction(int zone, float down_frac, float left_frac, float pitch_rad, float dir[3]);
// For a zone that sees the floor: the farthest reading that can still be its floor
// (robot level; farther ones are reflections or a drop, and tell nothing about what
// stands there). 0 for zones with no such limit.
float tof_rays_floor_limit_m(int zone);
