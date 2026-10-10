#pragma once
// The map as doc/MAP_DESIGN.md describes it: 10 cm cells in three layers (G ground
// −7…+3 cm, L1 3-13 cm, L2 13-23 cm), 6 x 6 m around the map's origin. Each VL53
// zone shoots 9 rays as long as its reading: the cells a ray passes are evidence for
// free, the cell it ends in for occupied, each ray weighted by closeness (a sigmoid
// around 1.5 m) and the VL53's confidence. Per frame a cell gets one verdict and a
// confidence q; it keeps its best measurement and changes only for a better one, or
// for 3 good-enough ones in a row (q ≥ 0.3). Nothing assumes a floor: floor is G
// occupied.
// Two variants (MAP_DESIGN.md §9): VOTES, as above (built 8 Oct), and READINGS (10
// Oct, from the recordings): each zone's reading is judged as a whole by where its
// cone can meet the floor: closer than that it is an obstacle (all of it, its low
// rays too), on the floor it is floor, straddling it unsure (no hit); and a ray
// passing a cell only counts against what it would have hit (it passed at or below
// the highest point hit there). The robot runs VOTES until replay shows READINGS better.
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

typedef enum { CELL_MAP_VOTES, CELL_MAP_READINGS } cell_map_variant_t;
// floor_margin (READINGS): a reading is an obstacle if it is closer than this share of
// where its cone first meets the floor (0.9: 10 % for pitch and noise).
void cell_map_set_variant(cell_map_variant_t variant, float floor_margin);

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
// 4 x 4 m around the robot, x (forward at the scan's start) up, y left; cells set by
// a measurement less confident than min_confidence count as unknown.
void cell_map_print(const pose_t *robot, float min_confidence);
