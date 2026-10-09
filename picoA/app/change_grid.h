#pragma once
// Which cells of a sensor's grid show movement (ROBOT_PLAN.md §6.1). A detector (the
// VL53's zones, the camera's blocks) scores every cell every frame by how unlike its
// reference it behaves; the grid decides from that. A cell has moved when its scores
// over the last CHANGE_GRID_WINDOW frames add up to the threshold, or to the lower
// neighbour threshold when a cell next to it does too (people span several cells,
// noise mostly doesn't). Moved cells that touch form blobs.
#include <stdbool.h>
#include <stdint.h>
#include "motion_obs.h"

#define CHANGE_GRID_MAX_CELLS 300 // the camera's 20 x 15 blocks
#define CHANGE_GRID_WINDOW 4

typedef struct { // the fields are the grid's own: use the functions
    int cols, rows;
    float threshold, neighbour_threshold;
    float score[CHANGE_GRID_WINDOW][CHANGE_GRID_MAX_CELLS];
    float sum[CHANGE_GRID_MAX_CELLS];
    bool moved[CHANGE_GRID_MAX_CELLS];
    int next;
} change_grid_t;

void change_grid_init(change_grid_t *g, int cols, int rows, float threshold, float neighbour_threshold);
void change_grid_clear(change_grid_t *g); // forgets all scores: nothing has moved
// One frame's scores, cols x rows, row by row (0 = as the reference, more = less like it).
void change_grid_add(change_grid_t *g, const float *score);
bool change_grid_moved(const change_grid_t *g, int cell);
float change_grid_sum(const change_grid_t *g, int cell);     // its scores over the window
// Numbers moved cells that touch (side by side or one above the other) with the same
// label 1..n, other cells 0; returns n (at most 255).
int change_grid_blobs(const change_grid_t *g, uint8_t *label);
// The blobs as observations: blob b (label b + 1) in all[b], with its direction
// (the mean of its cells' directions; direction() gives a cell's, unit length), its
// leftmost and rightmost cell, cells and strength. range_m is −1, point and t_us 0:
// the detector adds what it knows, using label. Returns the number of blobs.
int change_grid_observations(const change_grid_t *g, void (*direction)(int cell, float dir[3]), uint8_t *label,
                             motion_obs_t *all);
// The biggest of all[0..blobs) (most cells first), at most max of them, into obs;
// returns how many. Reorders all.
int change_grid_biggest(motion_obs_t *all, int blobs, motion_obs_t *obs, int max);
