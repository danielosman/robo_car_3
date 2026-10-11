#pragma once
// The map as doc/MAP_DESIGN.md describes it: 10 cm cells in three layers (G ground
// −7…+3 cm, L1 3-13 cm, L2 13-23 cm), 6 x 6 m around the map's origin. Each VL53
// zone shoots 9 rays as long as its reading. Each zone's reading is judged as a whole
// by where its cone can meet the floor: closer than that it is an obstacle (all of it,
// its low rays too, hits in L1 or above), on the floor it is floor, straddling it
// unsure (only its floor-height rays hit). The cells a ray passes are evidence for
// free, but only against what it would have hit (it passed at or below the highest
// point hit there); the cell it ends in for occupied. Each ray is weighted by
// closeness (a sigmoid around 1.5 m) and the VL53's confidence. Per frame a cell gets
// one verdict and a confidence q; it keeps its best measurement and changes only for
// a better one, or for 3 good-enough ones in a row (q ≥ 0.3).
// World frame: the map frame of pose.h.
#include <stdbool.h>
#include "pose.h"
#include "rangefinder.h"

#define CELL_LAYERS 3 // G, L1, L2

typedef enum { CELL_STATE_UNKNOWN, CELL_STATE_FREE, CELL_STATE_OCCUPIED } cell_state3_t;

typedef enum {
    COLUMN_UNKNOWN,
    COLUMN_BLOCKED,  // L1 occupied
    COLUMN_OVERHANG, // L2 occupied, L1 not
    COLUMN_NO_FLOOR, // G free: rays passed where the floor should be (a hole, a drop)
    COLUMN_DRIVABLE, // G occupied (floor seen), L1 and L2 free
    COLUMN_OPEN,     // L1 and L2 free, G unknown
} column_t;

// A reading is an obstacle if it is closer than this share of where its cone first
// meets the floor (10 % for pitch and noise). Changed only to try other values in replay.
#define CELL_MAP_FLOOR_MARGIN 0.9f
void cell_map_set_floor_margin(float margin);

void cell_map_clear(void);
// One frame, measured with the robot at `pose` (its pitch is used).
void cell_map_add(const range_frame_t *frame, const pose_t *pose);
// A cell's state and the confidence of the measurement that set it (0-1); unknown
// outside the window.
cell_state3_t cell_map_cell(float x_m, float y_m, int layer, float *confidence);
column_t cell_map_column(float x_m, float y_m);
// How far from (x, y) along the unit direction (dx, dy) the columns are free (open or
// drivable): up to the first one that isn't, the window's edge, or max_m.
float cell_map_free_distance(float x_m, float y_m, float dx, float dy, float max_m);
// How far along a path from (x, y) in the unit direction (dx, dy), half_width_m to
// each side, the nearest blocked or overhang column starts: the nearest corner of
// such a cell that overlaps the path (exact for a path along the grid, up to a few
// cm early for one across it); max_m if none is closer. Unknown columns don't count.
float cell_map_obstacle_along(float x_m, float y_m, float dx, float dy, float half_width_m, float max_m);
// 4 x 4 m around the robot, x (forward at the scan's start) up, y left; cells set by
// a measurement less confident than min_confidence count as unknown.
void cell_map_print(const pose_t *robot, float min_confidence);
