#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "units.h"
#include "change_grid.h"
#include "tof_motion.h"

#define LEARN_FRAMES   (1 * RANGEFINDER_HZ)  // 1 s
#define CLOSER_MM      80.0f  // closer than the background by this...
#define CLOSER_SHARE   0.08f  // ...or this share of it, whichever is more
#define MOVED_FRAMES   2.0f   // closer in this many of the last CHANGE_GRID_WINDOW frames, next to another zone...
#define ALONE_FRAMES   3.0f   // ...or this many alone (a person or a hand always spans several zones)
#define FARTHER_FRAMES (10 * RANGEFINDER_HZ) // 10 s farther in a row: the new background (not sooner: a nearer
                              // surface a zone shows only now and then must not be forgotten)
#define DRIFT          0.05f  // a reading at the background pulls it this much
#define STEADY_FRAMES  (1 * RANGEFINDER_HZ)  // closer but steady for 1 s: it has stopped, the new background
#define NOTHING        1e9f   // the background of a zone with no sure target: far beyond anything
#define EDGE_RAD       (16 * RAD_PER_DEG) // beyond this: the outer zone columns (the view is ±22.5°)
#define PASSING_ZONES  2      // zones of an outer column nearer in one frame: something passing

static float background[RANGEFINDER_RAYS]; // mm, or NOTHING (no sure reading while learning)
static float floor_limit[RANGEFINDER_RAYS]; // mm; 0 = none (rangefinder_floor_limit_m)
// A zone reading nearer without (yet) moving: the nearest such reading, and how many
// frames since it last read nearer. If it goes back without moving, it was a nearer
// surface the zone shows now and then: the background from then on.
static float brief_mm[RANGEFINDER_RAYS];
static uint8_t brief_quiet[RANGEFINDER_RAYS];
// A zone reading closer: the distance it has kept (within the margin), for how many frames.
static float steady_mm[RANGEFINDER_RAYS];
static uint16_t steady_frames[RANGEFINDER_RAYS];
static uint16_t farther[RANGEFINDER_RAYS];  // frames in a row farther than the background
static float learning[LEARN_FRAMES][RANGEFINDER_RAYS]; // the readings while learning
static uint32_t learning_us[LEARN_FRAMES];
static int learned;                        // frames learned since the restart
static uint32_t passed_us[2];              // right, left: the last frame something passed there; 0 = none
static float passed_rad[2];
static change_grid_t grid;
static bool grid_ready;

void tof_motion_restart(void) {
    if (!grid_ready) change_grid_init(&grid, RANGEFINDER_COLS, RANGEFINDER_ROWS, ALONE_FRAMES, MOVED_FRAMES);
    grid_ready = true;
    change_grid_clear(&grid);
    learned = 0;
    memset(passed_us, 0, sizeof passed_us);
    memset(background, 0, sizeof background);
    memset(farther, 0, sizeof farther);
    memset(brief_mm, 0, sizeof brief_mm);
    memset(steady_frames, 0, sizeof steady_frames);
    for (int i = 0; i < RANGEFINDER_RAYS; i++) floor_limit[i] = rangefinder_floor_limit_m(i) * 1000.0f;
}

static int compare_mm(const void *a, const void *b) {
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

// The background: the nearest distance a zone gives regularly while learning (its
// second-nearest sure reading: one odd near frame doesn't count). A zone switching
// between a near and a far surface has the near one: the far one is just farther.
// Nothing only if it had no sure reading at all.
static void learn_backgrounds(void) {
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        float v[LEARN_FRAMES];
        int n = 0;
        for (int k = 0; k < LEARN_FRAMES; k++)
            if (learning[k][i] >= 0 && learning[k][i] < NOTHING) v[n++] = learning[k][i];
        if (n == 0) { background[i] = NOTHING; continue; }
        qsort(v, (size_t)n, sizeof v[0], compare_mm);
        background[i] = v[n > 1 ? 1 : 0];
    }
}

static float margin_of(float mm) { return fmaxf(CLOSER_MM, CLOSER_SHARE * mm); }

// While learning, something passing out of the view: in an outer zone column, 2
// zones in one frame clearly nearer than the farthest the zone gives regularly (its
// 3rd-farthest sure distance).
static void find_passing(void) {
    float far[RANGEFINDER_RAYS], bearing[RANGEFINDER_RAYS];
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        float dir[3];
        rangefinder_ray_direction(i, dir);
        bearing[i] = atan2f(dir[1], dir[0]);
        float v[LEARN_FRAMES];
        int n = 0;
        for (int k = 0; k < LEARN_FRAMES; k++)
            if (learning[k][i] >= 0 && learning[k][i] < NOTHING) v[n++] = learning[k][i];
        far[i] = 0; // none: the zone tells nothing
        if (n < 3) continue;
        qsort(v, (size_t)n, sizeof v[0], compare_mm);
        far[i] = v[n - 3];
    }
    for (int k = 0; k < LEARN_FRAMES; k++) {
        int count[2] = {0, 0};
        float sum_rad[2] = {0, 0};
        for (int i = 0; i < RANGEFINDER_RAYS; i++) {
            if (fabsf(bearing[i]) < EDGE_RAD || far[i] == 0) continue;
            float mm = learning[k][i];
            if (mm < 0 || mm >= far[i] - margin_of(far[i])) continue;
            int side = bearing[i] > 0;
            count[side]++;
            sum_rad[side] += bearing[i];
        }
        for (int side = 0; side < 2; side++)
            if (count[side] >= PASSING_ZONES) {
                passed_us[side] = learning_us[k];
                passed_rad[side] = sum_rad[side] / (float)count[side];
            }
    }
}

bool tof_motion_passed(bool left, float *bearing_rad, uint32_t *t_us) {
    if (!tof_motion_ready() || !passed_us[left]) return false;
    *bearing_rad = passed_rad[left];
    *t_us = passed_us[left];
    return true;
}

bool tof_motion_ready(void) { return grid_ready && learned >= LEARN_FRAMES; }

// A zone's reading as a distance in mm: NOTHING for no target, < 0 when it tells
// nothing (unsure; in a floor zone, nothing or beyond its floor: a reflection).
static float reading(int i, uint16_t mm) {
    if (mm == RANGE_INVALID) return -1.0f;
    if (floor_limit[i] > 0 && (mm == RANGE_NO_TARGET || (float)mm > floor_limit[i])) return -1.0f;
    if (mm == RANGE_NO_TARGET) return NOTHING;
    return (float)mm;
}

static float margin(int i) { return margin_of(background[i]); }

static bool closer(int i, float mm) {
    return mm >= 0 && mm < NOTHING && mm < background[i] - margin(i);
}

// After learning, for zones that didn't read closer: farther for 10 s is the new
// background; at the background, it follows slowly.
static void follow(int i, float mm) {
    if (mm < 0) return;
    bool beyond = mm == NOTHING ? background[i] < NOTHING : mm > background[i] + margin(i);
    if (beyond) {
        if (++farther[i] >= FARTHER_FRAMES) { background[i] = mm; farther[i] = 0; }
        return;
    }
    farther[i] = 0;
    if (mm < NOTHING && background[i] < NOTHING) background[i] += DRIFT * (mm - background[i]);
}

static int compare_cells(const void *a, const void *b) {
    return ((const motion_obs_t *)b)->cells - ((const motion_obs_t *)a)->cells;
}

static int observations(const range_frame_t *f, motion_obs_t *obs, int max) {
    static uint8_t label[RANGEFINDER_RAYS];
    int blobs = change_grid_blobs(&grid, label);
    motion_obs_t all[RANGEFINDER_RAYS];
    memset(all, 0, sizeof all);
    for (int b = 0; b < blobs; b++) all[b].range_m = -1.0f;
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        if (!label[i]) continue;
        motion_obs_t *o = &all[label[i] - 1];
        float dir[3];
        rangefinder_ray_direction(i, dir);
        for (int k = 0; k < 3; k++) o->where[k] += dir[k];
        float bearing = atan2f(dir[1], dir[0]);
        if (o->cells == 0 || bearing > o->left_rad) o->left_rad = bearing;
        if (o->cells == 0 || bearing < o->right_rad) o->right_rad = bearing;
        o->cells++;
        float mm = reading(i, f->range_mm[i]);
        if (mm >= 0 && mm < NOTHING && (o->range_m < 0 || mm / 1000.0f < o->range_m)) o->range_m = mm / 1000.0f;
    }
    float origin[3];
    rangefinder_origin(origin);
    for (int b = 0; b < blobs; b++) {
        motion_obs_t *o = &all[b];
        float len = sqrtf(o->where[0] * o->where[0] + o->where[1] * o->where[1] + o->where[2] * o->where[2]);
        for (int k = 0; k < 3; k++) {
            o->where[k] /= len;
            o->point[k] = o->range_m >= 0 ? origin[k] + o->where[k] * o->range_m : 0.0f;
        }
        o->strength = (float)o->cells / RANGEFINDER_RAYS;
        o->t_us = f->t_us;
    }
    qsort(all, (size_t)blobs, sizeof all[0], compare_cells);
    int n = blobs < max ? blobs : max;
    memcpy(obs, all, (size_t)n * sizeof all[0]);
    return n;
}

int tof_motion_add(const range_frame_t *f, motion_obs_t *obs, int max) {
    if (!grid_ready) tof_motion_restart();
    if (learned < LEARN_FRAMES) {
        for (int i = 0; i < RANGEFINDER_RAYS; i++) learning[learned][i] = reading(i, f->range_mm[i]);
        learning_us[learned] = f->t_us;
        if (++learned == LEARN_FRAMES) { learn_backgrounds(); find_passing(); }
        return 0;
    }
    float score[RANGEFINDER_RAYS];
    for (int i = 0; i < RANGEFINDER_RAYS; i++) score[i] = closer(i, reading(i, f->range_mm[i])) ? 1.0f : 0.0f;
    change_grid_add(&grid, score);
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        float mm = reading(i, f->range_mm[i]);
        if (change_grid_moved(&grid, i)) brief_mm[i] = 0; // movement, not a surface
        else if (score[i]) {
            if (brief_mm[i] == 0 || mm < brief_mm[i]) brief_mm[i] = mm;
            brief_quiet[i] = 0;
        } else if (brief_mm[i] > 0 && ++brief_quiet[i] >= CHANGE_GRID_WINDOW) {
            background[i] = brief_mm[i];
            brief_mm[i] = 0;
        }
        if (score[i]) { // closer: kept still for 1 s, it has stopped moving and is the view now
            if (steady_frames[i] && fabsf(mm - steady_mm[i]) < fmaxf(CLOSER_MM, CLOSER_SHARE * mm)) steady_frames[i]++;
            else { steady_mm[i] = mm; steady_frames[i] = 1; }
            if (steady_frames[i] >= STEADY_FRAMES) { background[i] = mm; steady_frames[i] = 0; brief_mm[i] = 0; }
        } else {
            steady_frames[i] = 0;
            follow(i, mm);
        }
    }
    return observations(f, obs, max);
}

bool tof_motion_moved(int zone) { return change_grid_moved(&grid, zone); }

uint16_t tof_motion_background_mm(int zone) {
    float b = background[zone];
    return b <= 0 || b >= NOTHING ? 0 : (uint16_t)b;
}
