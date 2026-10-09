#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "geom.h"
#include "change_grid.h"

void change_grid_init(change_grid_t *g, int cols, int rows, float threshold, float neighbour_threshold) {
    g->cols = cols;
    g->rows = rows;
    g->threshold = threshold;
    g->neighbour_threshold = neighbour_threshold;
    change_grid_clear(g);
}

void change_grid_clear(change_grid_t *g) {
    memset(g->score, 0, sizeof g->score);
    memset(g->sum, 0, sizeof g->sum);
    memset(g->moved, 0, sizeof g->moved);
    g->next = 0;
}

static bool neighbour_scores(const change_grid_t *g, int cell) {
    int col = cell % g->cols, row = cell / g->cols;
    return (col > 0 && g->sum[cell - 1] >= g->neighbour_threshold) ||
           (col < g->cols - 1 && g->sum[cell + 1] >= g->neighbour_threshold) ||
           (row > 0 && g->sum[cell - g->cols] >= g->neighbour_threshold) ||
           (row < g->rows - 1 && g->sum[cell + g->cols] >= g->neighbour_threshold);
}

void change_grid_add(change_grid_t *g, const float *score) {
    int n = g->cols * g->rows;
    for (int i = 0; i < n; i++) {
        g->sum[i] += score[i] - g->score[g->next][i];
        if (g->sum[i] < 0) g->sum[i] = 0; // rounding
        g->score[g->next][i] = score[i];
    }
    g->next = (g->next + 1) % CHANGE_GRID_WINDOW;
    for (int i = 0; i < n; i++)
        g->moved[i] = g->sum[i] >= g->threshold || (g->sum[i] >= g->neighbour_threshold && neighbour_scores(g, i));
}

bool change_grid_moved(const change_grid_t *g, int cell) { return g->moved[cell]; }
float change_grid_sum(const change_grid_t *g, int cell) { return g->sum[cell]; }

int change_grid_blobs(const change_grid_t *g, uint8_t *label) {
    int n = g->cols * g->rows, blobs = 0;
    memset(label, 0, (size_t)n);
    static int stack[CHANGE_GRID_MAX_CELLS];
    for (int start = 0; start < n; start++) {
        if (!g->moved[start] || label[start] || blobs == 255) continue;
        label[start] = (uint8_t)++blobs;
        int top = 0;
        stack[top++] = start;
        while (top > 0) { // flood fill: each cell is pushed once, when labelled
            int c = stack[--top], col = c % g->cols;
            int next[4] = {col > 0 ? c - 1 : -1, col < g->cols - 1 ? c + 1 : -1, c - g->cols,
                           c + g->cols < n ? c + g->cols : -1};
            for (int k = 0; k < 4; k++) {
                int m = next[k];
                if (m < 0 || !g->moved[m] || label[m]) continue;
                label[m] = (uint8_t)blobs;
                stack[top++] = m;
            }
        }
    }
    return blobs;
}

int change_grid_observations(const change_grid_t *g, void (*direction)(int cell, float dir[3]), uint8_t *label,
                             motion_obs_t *all) {
    int blobs = change_grid_blobs(g, label), n = g->cols * g->rows;
    memset(all, 0, (size_t)blobs * sizeof all[0]);
    for (int i = 0; i < n; i++) {
        if (!label[i]) continue;
        motion_obs_t *o = &all[label[i] - 1];
        float dir[3];
        direction(i, dir);
        for (int k = 0; k < 3; k++) o->where[k] += dir[k];
        float bearing = bearing_rad(dir);
        if (o->cells == 0 || bearing > o->left_rad) o->left_rad = bearing;
        if (o->cells == 0 || bearing < o->right_rad) o->right_rad = bearing;
        o->cells++;
    }
    for (int b = 0; b < blobs; b++) {
        motion_obs_t *o = &all[b];
        float len = sqrtf(o->where[0] * o->where[0] + o->where[1] * o->where[1] + o->where[2] * o->where[2]);
        for (int k = 0; k < 3; k++) o->where[k] /= len;
        o->range_m = -1.0f;
        o->strength = (float)o->cells / (float)n;
    }
    return blobs;
}

static int compare_cells(const void *a, const void *b) {
    return ((const motion_obs_t *)b)->cells - ((const motion_obs_t *)a)->cells;
}

int change_grid_biggest(motion_obs_t *all, int blobs, motion_obs_t *obs, int max) {
    qsort(all, (size_t)blobs, sizeof all[0], compare_cells);
    int n = blobs < max ? blobs : max;
    memcpy(obs, all, (size_t)n * sizeof all[0]);
    return n;
}
