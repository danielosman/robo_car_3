#include <math.h>
#include <string.h>
#include "stats.h"
#include "change_grid.h"
#include "camera_motion.h"

#define BLOCK          8         // pixels per block side
#define MIN_FRAME_US   60000     // frames closer together than this are skipped: ~15 a second at most
#define SETTLED_FRAMES 3         // frames in a row not settling, on target: the exposure is right, hold it
#define LEARN_US       1000000   // 1 s
#define LEARN_MAX      20        // frames learned at most (1 s at ~15 a second)
#define MIN_NOISE      2.0f      // brightness 0-255: a block's noise is never taken as less
#define DIFFER_NOISE   4.0f      // differs: off its background by more than this many times its noise
#define MOVED_FRAMES   2.0f      // differs in this many of the last CHANGE_GRID_WINDOW frames, next to another block...
#define ALONE_FRAMES   3.0f      // ...or this many alone
#define STEADY_US      1000000   // differs but steady for 1 s: it has stopped, the new background
#define DRIFT          0.05f     // a block at its background pulls it this much (dusk, clouds)
#define CALM_US        5000000   // the exposure is adjusted only after this long with nothing moving...
#define MAX_WAIT_US    30000000  // ...or after this long wanting it (something that never stops moving)
#define FOCAL_PX       160.0f    // 160 columns across 53.1° (ROBOT_PLAN.md §3)

static enum { SETTLING, LEARNING, WATCHING } state;
static int settled;                          // frames in a row not settling
static float learning[LEARN_MAX][CAMERA_MOTION_BLOCKS];
static int learned;
static uint32_t learn_start_us, last_us;
static float background[CAMERA_MOTION_BLOCKS], noise[CAMERA_MOTION_BLOCKS];
static bool differs[CAMERA_MOTION_BLOCKS];
// A block that changed: the value it has kept (within its threshold), since when.
static float steady_value[CAMERA_MOTION_BLOCKS];
static uint32_t steady_since_us[CAMERA_MOTION_BLOCKS];
static bool steady[CAMERA_MOTION_BLOCKS];
static uint32_t adjustments;
static uint32_t last_moved_us; // the last frame in which something moved (or watching began)
static bool wanting;           // the driver wants to adjust the exposure, since wants_since_us
static uint32_t wants_since_us;
static change_grid_t grid;
static bool grid_ready;

void camera_motion_restart(void) {
    if (!grid_ready) change_grid_init(&grid, CAMERA_MOTION_COLS, CAMERA_MOTION_ROWS, ALONE_FRAMES, MOVED_FRAMES);
    grid_ready = true;
    change_grid_clear(&grid);
    camera_hold_exposure(false);
    state = SETTLING;
    settled = 0;
    learned = 0;
    memset(differs, 0, sizeof differs);
    memset(steady, 0, sizeof steady);
}

bool camera_motion_ready(void) { return state == WATCHING; }
uint32_t camera_motion_exposure_adjustments(void) { return adjustments; }

static void block_values(const camera_frame_t *f, float *value) {
    for (int by = 0; by < CAMERA_MOTION_ROWS; by++)
        for (int bx = 0; bx < CAMERA_MOTION_COLS; bx++) {
            uint32_t sum = 0;
            for (int y = 0; y < BLOCK; y++) {
                const uint8_t *p = f->pixels + (by * BLOCK + y) * CAMERA_WIDTH + bx * BLOCK;
                for (int x = 0; x < BLOCK; x++) sum += p[x];
            }
            value[by * CAMERA_MOTION_COLS + bx] = (float)sum / (BLOCK * BLOCK);
        }
}

// Each block's background is the middle of what it read while learning, its noise
// the mean distance of its readings from that.
static void learn_backgrounds(void) {
    for (int i = 0; i < CAMERA_MOTION_BLOCKS; i++) {
        float v[LEARN_MAX];
        for (int k = 0; k < learned; k++) v[k] = learning[k][i];
        background[i] = median_value(v, learned);
        float spread = 0.0f;
        for (int k = 0; k < learned; k++) spread += fabsf(v[k] - background[i]);
        noise[i] = fmaxf(MIN_NOISE, spread / (float)learned);
    }
}

static float threshold(int i) { return DIFFER_NOISE * noise[i]; }

// The direction a block's centre looks along, robot frame (x forward, y left, z up).
static void block_direction(int i, float *dir) {
    float x = (float)(i % CAMERA_MOTION_COLS * BLOCK) + (BLOCK - 1) / 2.0f;
    float y = (float)(i / CAMERA_MOTION_COLS * BLOCK) + (BLOCK - 1) / 2.0f;
    float v[3] = {FOCAL_PX, (CAMERA_WIDTH - 1) / 2.0f - x, (CAMERA_HEIGHT - 1) / 2.0f - y};
    float len = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    for (int k = 0; k < 3; k++) dir[k] = v[k] / len;
}

// The blobs, each timed when the camera read its middle row (the rolling shutter).
// No range: the camera gives none.
static int observations(const camera_frame_t *f, motion_obs_t *obs, int max) {
    static uint8_t label[CAMERA_MOTION_BLOCKS];
    static motion_obs_t all[255];
    static float row_sum[255];
    int blobs = change_grid_observations(&grid, block_direction, label, all);
    memset(row_sum, 0, sizeof row_sum);
    for (int i = 0; i < CAMERA_MOTION_BLOCKS; i++)
        if (label[i]) row_sum[label[i] - 1] += (float)(i / CAMERA_MOTION_COLS * BLOCK) + (BLOCK - 1) / 2.0f;
    for (int b = 0; b < blobs; b++) all[b].t_us = camera_row_time(f, (int)(row_sum[b] / (float)all[b].cells));
    return change_grid_biggest(all, blobs, obs, max);
}

int camera_motion_add(const camera_frame_t *f, motion_obs_t *obs, int max) {
    if (!grid_ready) camera_motion_restart();
    if (f->settling) { settled = 0; return -1; }
    if (state == SETTLING) {
        if (f->wants_exposure_change) settled = 0;
        else settled++;
        if (settled < SETTLED_FRAMES) return -1;
        camera_hold_exposure(true);
        state = LEARNING;
        learn_start_us = last_us = f->last_row_us;
        block_values(f, learning[learned++]);
        return -1;
    }
    if (f->last_row_us - last_us < MIN_FRAME_US) return -1;
    last_us = f->last_row_us;
    if (state == LEARNING) {
        block_values(f, learning[learned++]);
        if (learned == LEARN_MAX || f->last_row_us - learn_start_us >= LEARN_US) {
            learn_backgrounds();
            state = WATCHING;
            last_moved_us = f->last_row_us;
            wanting = false;
        }
        return -1;
    }

    // Static: too big for the main core's small stack.
    static float value[CAMERA_MOTION_BLOCKS], score[CAMERA_MOTION_BLOCKS];
    block_values(f, value);
    for (int i = 0; i < CAMERA_MOTION_BLOCKS; i++) differs[i] = fabsf(value[i] - background[i]) > threshold(i);
    for (int i = 0; i < CAMERA_MOTION_BLOCKS; i++) score[i] = differs[i] ? 1.0f : 0.0f;
    change_grid_add(&grid, score);
    // A block that changed and keeps its new value has stopped, also when that value
    // is near the edge of differing and dips under it now and then (it would flicker
    // as movement for a long time otherwise).
    for (int i = 0; i < CAMERA_MOTION_BLOCKS; i++) {
        if (steady[i] && fabsf(value[i] - steady_value[i]) < threshold(i)) {
            if (f->last_row_us - steady_since_us[i] >= STEADY_US) { // it has stopped: the view now
                background[i] = value[i];
                steady[i] = false;
            }
        } else if (differs[i]) {
            steady[i] = true;
            steady_value[i] = value[i];
            steady_since_us[i] = f->last_row_us;
        } else steady[i] = false;
        if (!differs[i]) background[i] += DRIFT * (value[i] - background[i]);
    }
    int n_obs = observations(f, obs, max);
    if (n_obs > 0) last_moved_us = f->last_row_us;
    // The exposure is the driver's: it is adjusted only when calm, without watching.
    if (!f->wants_exposure_change) wanting = false;
    else if (!wanting) { wanting = true; wants_since_us = f->last_row_us; }
    if (wanting && (f->last_row_us - last_moved_us >= CALM_US || f->last_row_us - wants_since_us >= MAX_WAIT_US)) {
        adjustments++;
        camera_motion_restart();
        return -1;
    }
    return n_obs;
}

float camera_motion_background(int block) { return background[block]; }
float camera_motion_noise(int block) { return noise[block]; }
bool camera_motion_differs(int block) { return differs[block]; }
bool camera_motion_moved(int block) { return change_grid_moved(&grid, block); }
