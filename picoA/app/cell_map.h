#pragma once
// The map (doc/ARCHITECTURE.md): 10 cm cells in three layers (G ground
// −7…+3 cm, L1 3-13 cm, L2 13-23 cm), in a 6 x 6 m window that follows the robot
// by whole 2 x 2 m tiles of the whole-house map (house_map.h): what scrolls out is
// kept there as columns, what scrolls in starts from there. Each VL53 zone shoots 9
// rays as long as its reading; tof_rays places them and judges the reading as a
// whole by where its cone can meet the floor: closer than that it is an obstacle
// (all of it, its low rays too, hits in L1 or above), on the floor it is floor,
// straddling it unsure (only its floor-height rays hit). The cells a ray passes are evidence for
// free, but only against what it would have hit (it passed at or below the highest
// point hit there); the cell it ends in for occupied. Each ray is weighted by
// closeness (a sigmoid around 1.5 m) and the VL53's confidence. Per frame a cell gets
// one verdict and a confidence q; it keeps its best measurement and changes only for
// a better one, or for 3 good-enough ones in a row (q ≥ 0.3).
// World frame: the map frame of pose.h.
#include <stdbool.h>
#include "pose.h"
#include "tof_frame.h"

#define CELL_LAYERS   3  // G, L1, L2
#define CELL_MAP_GRID 60 // cells across the window: 6 m

typedef enum { CELL_STATE_UNKNOWN, CELL_STATE_FREE, CELL_STATE_OCCUPIED } cell_state3_t;

typedef enum {
    COLUMN_UNKNOWN,
    COLUMN_BLOCKED,  // L1 occupied
    COLUMN_OVERHANG, // L2 occupied, L1 not
    COLUMN_NO_FLOOR, // G free: rays passed where the floor should be (a hole, a drop)
    COLUMN_DRIVABLE, // G occupied (floor seen), L1 and L2 free
    COLUMN_OPEN,     // L1 and L2 free, G unknown
} column_t;

void cell_map_clear(void); // the window and the whole-house map; the window stays where it is
// Scrolls the window by whole tiles when the robot at (x, y) is more than 0.5 m out
// of its middle tile (within ~1.5 m of its edge). cell_map_add() does it for each frame.
void cell_map_follow(float x_m, float y_m);
// The window's cells for a snapshot: [layer][ix][iy], cell (ix, iy) is world cell
// (origin_cx + ix, origin_cy + iy); one byte each: state (bits 0-1, cell_state3_t), doubt
// (bits 2-3), confidence 0-15 (bits 4-7).
const uint8_t *cell_map_window(int *origin_cx, int *origin_cy);
// One frame (as unpacked for the map use), measured where the robot was then (its
// pitch is used; the window follows it first). A frame without a pose is left out.
// Each zone's reading as tof_rays places and judges it; unsure zones tell nothing,
// "nothing in range" is free for 1 m, "beyond" free up to the frame's limit.
void cell_map_add(const posed_frame_t *frame);
// A cell's state and the confidence of the measurement that set it (0-1); unknown
// outside the window.
cell_state3_t cell_map_cell(float x_m, float y_m, int layer, float *confidence);
// The window's column; outside it, the whole-house map's.
column_t cell_map_column(float x_m, float y_m);
// How far along a path from (x, y) in the unit direction (dx, dy), half_width_m to
// each side, the nearest blocked or overhang column starts: the nearest corner of
// such a cell that overlaps the path (exact for a path along the grid, up to a few
// cm early for one across it); max_m if none is closer. Unknown columns don't count;
// outside the window, the whole-house map's columns do.
float cell_map_obstacle_along(float x_m, float y_m, float dx, float dy, float half_width_m, float max_m);
// 4 x 4 m around the robot, x (forward at the scan's start) up, y left; cells set by
// a measurement less confident than min_confidence count as unknown.
void cell_map_print(const pose_t *robot, float min_confidence);
