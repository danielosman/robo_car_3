#include <string.h>
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
