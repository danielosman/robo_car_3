#include <math.h>
#include <string.h>
#include "units.h"
#include "geom.h"
#include "change_grid.h"
#include "tof_motion.h"

#define BINS           256                      // per row, around the robot
#define BIN_RAD        (2 * PI_F / BINS)        // 1.4°: a quarter zone
#define HALF_ZONE_RAD  (5.625f / 2 * RAD_PER_DEG)
#define SLACK_RAD      (1 * RAD_PER_DEG)        // heading error between filling a bin and using it
#define TIME_SLACK     0.15f                    // of the frame's turn: a 10 ms error in the frame's time
#define TURNING_RAD    (1 * RAD_PER_DEG)        // turned more than this during a frame: turning
#define CLOSER_MM      80.0f  // closer than the background by this...
#define CLOSER_SHARE   0.08f  // ...or this share of it, whichever is more
#define MOVED_FRAMES   2.0f   // closer in this many of the last CHANGE_GRID_WINDOW frames, next to another zone...
#define ALONE_FRAMES   3.0f   // ...or this many alone (a person or a hand always spans several zones)
#define FARTHER_FRAMES (10 * RANGEFINDER_HZ) // a bin seen farther this many times in a row: the new background
                              // (not sooner: a nearer surface a zone shows only now and then must not be forgotten)
#define DRIFT          0.05f  // a reading at the background pulls it this much
#define STEADY_FRAMES  (1 * RANGEFINDER_HZ)  // closer but steady for 1 s: it has stopped, the new background
#define NOTHING        1e9f   // a reading or background with no sure target: far beyond anything
#define NOTHING_MM     0xFFFF // ...as a bin stores it
#define MOVED_M        0.1f   // the robot moved this far since the bins were filled: forget them

typedef struct {
    uint16_t bg_mm;   // free up to here (NOTHING_MM: nothing in range); valid once known
    uint16_t cand_mm; // not known yet: the first reading; 0 = none
    bool known;
    uint8_t farther;  // readings in a row farther than bg_mm
} bin_t;

static bin_t bins[RANGEFINDER_ROWS][BINS];
static float left_rad[RANGEFINDER_RAYS];      // each zone's direction from the robot's heading (+ = left)
static float floor_limit[RANGEFINDER_RAYS];   // mm; 0 = none (rangefinder_floor_limit_m)
static float predicted[RANGEFINDER_RAYS];     // the last frame's background per zone, mm; < 0 = not known
// A zone reading nearer without (yet) moving: the nearest such reading, and how many
// frames since it last read nearer. If it goes back without moving, it was a nearer
// surface the zone shows now and then: the background from then on.
static float brief_mm[RANGEFINDER_RAYS];
static uint8_t brief_quiet[RANGEFINDER_RAYS];
// A zone reading closer: the distance it has kept (within the margin), for how many frames.
static float steady_mm[RANGEFINDER_RAYS];
static uint16_t steady_frames[RANGEFINDER_RAYS];
static float anchor_x_m, anchor_y_m;  // where the robot was when the bins were last forgotten
static int frames;                    // since then
static change_grid_t grid;
static bool grid_ready;

static void forget_bins(const tof_view_t *view) {
    memset(bins, 0, sizeof bins);
    memset(brief_mm, 0, sizeof brief_mm);
    memset(steady_frames, 0, sizeof steady_frames);
    change_grid_clear(&grid);
    anchor_x_m = view ? view->x_m : 0;
    anchor_y_m = view ? view->y_m : 0;
    frames = 0;
}

void tof_motion_restart(void) {
    if (!grid_ready) change_grid_init(&grid, RANGEFINDER_COLS, RANGEFINDER_ROWS, ALONE_FRAMES, MOVED_FRAMES);
    grid_ready = true;
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        float dir[3];
        rangefinder_ray_direction(i, dir);
        left_rad[i] = bearing_rad(dir);
        floor_limit[i] = rangefinder_floor_limit_m(i) * 1000.0f;
    }
    forget_bins(0);
}

bool tof_motion_ready(void) { return grid_ready && frames >= 2; }

// A zone's reading as a distance in mm: NOTHING for no target, < 0 when it tells
// nothing (unsure; in a floor zone, nothing or beyond its floor: a reflection).
static float reading(int i, uint16_t mm) {
    if (mm == RANGE_INVALID) return -1.0f;
    if (floor_limit[i] > 0 && (mm == RANGE_NO_TARGET || (float)mm > floor_limit[i])) return -1.0f;
    if (mm == RANGE_NO_TARGET) return NOTHING;
    return (float)mm;
}

static float margin_of(float mm) { return fmaxf(CLOSER_MM, CLOSER_SHARE * mm); }
static float bin_mm(uint16_t v) { return v == NOTHING_MM ? NOTHING : (float)v; }
static uint16_t to_bin(float mm) { return mm >= NOTHING ? NOTHING_MM : (uint16_t)fminf(mm, NOTHING_MM - 1); }
static bin_t *bin_at(int row, int k) { return &bins[row][k & (BINS - 1)]; }

// The background a zone predicts: the nearest known bin among those it swept,
// lo..hi (world bearings). < 0 if more than a third of them are not known.
static float predict(int row, float lo_rad, float hi_rad) {
    int first = (int)floorf(lo_rad / BIN_RAD), last = (int)floorf(hi_rad / BIN_RAD), unknown = 0;
    float p = NOTHING;
    for (int k = first; k <= last; k++) {
        const bin_t *b = bin_at(row, k);
        if (!b->known) unknown++;
        else p = fminf(p, bin_mm(b->bg_mm));
    }
    return unknown * 3 > last - first + 1 ? -1.0f : p;
}

// A reading not closer than the zone's prediction, for a bin its cone covered: every direction there is free at least up to it. Fills a new bin
// (the nearer of its first two readings: a surface seen only now and then counts),
// follows one at its background, and raises one only after 10 s of farther readings.
static void learn(bin_t *b, float mm) {
    if (!b->known) {
        if (!b->cand_mm) { b->cand_mm = to_bin(mm); return; }
        b->bg_mm = to_bin(fminf(mm, bin_mm(b->cand_mm)));
        b->known = true;
        b->cand_mm = 0;
        b->farther = 0;
        return;
    }
    float bg = bin_mm(b->bg_mm);
    bool beyond = mm >= NOTHING ? bg < NOTHING : mm > bg + margin_of(bg);
    if (beyond) {
        if (++b->farther >= FARTHER_FRAMES) { b->bg_mm = to_bin(mm); b->farther = 0; }
        return;
    }
    if (mm < bg - margin_of(bg)) return; // nearer than this bin: the zone's nearest surface is elsewhere in it
    b->farther = 0;
    if (mm < NOTHING && bg < NOTHING) b->bg_mm = to_bin(bg + DRIFT * (mm - bg));
}

// The bins a zone's cone covered all frame (lo..hi): those whose centre is in it (a
// bin across two zones' edge belongs to the one with more of it).
static float centre_rad(int k) { return ((float)k + 0.5f) * BIN_RAD; }
static int first_bin(float lo_rad) { return (int)ceilf(lo_rad / BIN_RAD - 0.5f); }

// A nearer surface that stays (or comes back now and then): the background there now.
static void set_bins(int row, float lo_rad, float hi_rad, float mm) {
    for (int k = first_bin(lo_rad); centre_rad(k) <= hi_rad; k++) {
        bin_t *b = bin_at(row, k);
        b->bg_mm = to_bin(mm);
        b->known = true;
        b->cand_mm = 0;
        b->farther = 0;
    }
}

static void learn_bins(int row, float lo_rad, float hi_rad, float mm) {
    for (int k = first_bin(lo_rad); centre_rad(k) <= hi_rad; k++) learn(bin_at(row, k), mm);
}

// The blobs, with the nearest sure reading in each as its range.
static int observations(const range_frame_t *f, motion_obs_t *obs, int max) {
    static uint8_t label[RANGEFINDER_RAYS];
    motion_obs_t all[RANGEFINDER_RAYS];
    int blobs = change_grid_observations(&grid, rangefinder_ray_direction, label, all);
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        if (!label[i]) continue;
        motion_obs_t *o = &all[label[i] - 1];
        float mm = reading(i, f->range_mm[i]);
        if (mm >= 0 && mm < NOTHING && (o->range_m < 0 || mm / 1000.0f < o->range_m)) o->range_m = mm / 1000.0f;
    }
    float origin[3];
    rangefinder_origin(origin);
    for (int b = 0; b < blobs; b++) {
        motion_obs_t *o = &all[b];
        for (int k = 0; k < 3; k++) o->point[k] = o->range_m >= 0 ? origin[k] + o->where[k] * o->range_m : 0.0f;
        o->t_us = f->t_us;
    }
    return change_grid_biggest(all, blobs, obs, max);
}

int tof_motion_add(const range_frame_t *f, const tof_view_t *view, motion_obs_t *obs, int max) {
    if (!grid_ready) tof_motion_restart();
    if (hypotf(view->x_m - anchor_x_m, view->y_m - anchor_y_m) > MOVED_M) forget_bins(view);
    frames++;
    float y0 = fminf(view->yaw_start_rad, view->yaw_end_rad), y1 = fmaxf(view->yaw_start_rad, view->yaw_end_rad);
    bool turning = y1 - y0 > TURNING_RAD;
    float slack = SLACK_RAD + TIME_SLACK * (y1 - y0);
    // Per zone: the world bearings it swept (widened for heading error) and the ones
    // its cone covered all frame long (none when turning faster than a zone a frame).
    float core_lo[RANGEFINDER_RAYS], core_hi[RANGEFINDER_RAYS], mm[RANGEFINDER_RAYS], score[RANGEFINDER_RAYS];
    bool used[RANGEFINDER_RAYS];
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        int row = i / RANGEFINDER_COLS;
        mm[i] = reading(i, f->range_mm[i]);
        core_lo[i] = y1 + left_rad[i] - HALF_ZONE_RAD;
        core_hi[i] = y0 + left_rad[i] + HALF_ZONE_RAD;
        // Turning, the robot rolls a few degrees: the floor rows' floor moves too much.
        used[i] = !(turning && row >= rangefinder_first_floor_row());
        predicted[i] = used[i] ? predict(row, y0 + left_rad[i] - HALF_ZONE_RAD - slack,
                                         y1 + left_rad[i] + HALF_ZONE_RAD + slack) : -1.0f;
        float p = predicted[i];
        score[i] = p >= 0 && mm[i] >= 0 && mm[i] < NOTHING && mm[i] < p - margin_of(p) ? 1.0f : 0.0f;
    }
    change_grid_add(&grid, score);
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        int row = i / RANGEFINDER_COLS;
        if (!used[i]) continue;
        if (turning) { brief_mm[i] = 0; steady_frames[i] = 0; } // a zone looks somewhere new every frame
        else if (change_grid_moved(&grid, i)) brief_mm[i] = 0; // movement, not a surface
        else if (score[i]) {
            if (brief_mm[i] == 0 || mm[i] < brief_mm[i]) brief_mm[i] = mm[i];
            brief_quiet[i] = 0;
        } else if (brief_mm[i] > 0 && ++brief_quiet[i] >= CHANGE_GRID_WINDOW) {
            set_bins(row, core_lo[i], core_hi[i], brief_mm[i]);
            brief_mm[i] = 0;
        }
        if (score[i]) { // closer: kept still for 1 s, it has stopped moving and is the view now
            if (turning) continue;
            if (steady_frames[i] && fabsf(mm[i] - steady_mm[i]) < margin_of(mm[i])) steady_frames[i]++;
            else { steady_mm[i] = mm[i]; steady_frames[i] = 1; }
            if (steady_frames[i] >= STEADY_FRAMES) {
                set_bins(row, core_lo[i], core_hi[i], mm[i]);
                steady_frames[i] = 0;
                brief_mm[i] = 0;
            }
        } else if (mm[i] >= 0) { // an unsure reading tells nothing: it doesn't end "steady" either
            steady_frames[i] = 0;
            learn_bins(row, core_lo[i], core_hi[i], mm[i]);
        }
    }
    return observations(f, obs, max);
}

bool tof_motion_moved(int zone) { return change_grid_moved(&grid, zone); }

uint16_t tof_motion_background_mm(int zone) {
    float b = predicted[zone];
    return b <= 0 || b >= NOTHING ? 0 : (uint16_t)b;
}
