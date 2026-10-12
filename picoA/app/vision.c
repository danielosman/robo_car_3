#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "camera.h"
#include "stats.h"
#include "vision.h"

#define W            160
#define H            120
#define HW           (W / 2)  // the half-size image: 2 x 2 averages
#define HH           (H / 2)
#define T            8        // templates: 8 x 8 px, at full and at half size
#define CELL         20       // finding: cells of 20 x 20 px, as the regions
#define PER_CELL     2
#define EDGE         8        // features stay this far inside the image (the half-size template)
#define SPACING      6.0f     // a new feature this far from any other at least
#define REACH        4        // the coarse search: ±4 px on the half-size image (±8 full size)
#define CLIMB        3        // full size: at most this many 1 px steps from the coarse place
#define RETAKE_PX    1.0f     // a template is taken again once the feature is this far from where it was taken...
#define RETAKE_MATCH 0.9f     // ...or matches less than this (kept otherwise: no drift on a still scene)
#define LK_STEPS     4        // then Lucas-Kanade steps for the sub-pixel part...
#define LK_DONE      0.01f    // ...until a step is this small (px)
#define ACCEPT       0.8f     // ZNCC for a match...
#define UNIQUE       0.05f    // ...and this much better than the second-best place
#define MAX_MISSES   2        // missed this many frames in a row; the next miss loses it
#define ST_WINDOW    3        // Shi-Tomasi over 7 x 7 px
#define EVERY        3        // on the robot: every 3rd camera frame
#define SAMPLE_STEP  4        // brightness from every 4th pixel of every 4th row
#define SATURATED    250
#define TRIES        6        // FAST thresholds tried per cell, from the highest
static const int fast_thresholds[TRIES] = {48, 34, 24, 17, 12, 8};

// The pixel noise (grey levels, σ) at a gain: sets FAST's lowest threshold (3 σ) and
// the weakest corner taken. A guess until measured on the robot (still scenes at
// each gain, V1 robot test).
static float noise_sigma(float gain) { return 1.0f + 0.15f * gain; }

typedef struct {
    bool live;            // false: a free slot
    bool lost;            // reported LOST in the last record: freed at the next frame
    uint16_t id, age;
    uint8_t misses;       // in a row
    float x, y;           // its last measured place
    float strength;
    float vx, vy;         // its last measured move per frame (0 until measured): where the search starts
    float ax, ay;         // where it was when its templates were taken
    int tx0, ty0, hx0, hy0; // the templates' top-left corners when they were taken
    uint8_t tpl[T * T], tplh[T * T];
    int32_t sum, sum2, sumh, sumh2;
} track_t;

static track_t tracks[VISION_MAX_FEATURES];
static uint8_t half[HH * HW];
static vision_frame_t rec;
static bool have_rec;
static uint16_t next_id = 1;
static uint32_t prev_t_us;
static vision_pose_t prev_pose;

// --- matching ---

static void sums(const uint8_t *t, int32_t *s, int32_t *s2) {
    int32_t a = 0, b = 0;
    for (int i = 0; i < T * T; i++) { a += t[i]; b += t[i] * t[i]; }
    *s = a;
    *s2 = b;
}

// ZNCC of the template against the 8 x 8 window at (x0, y0); -1 if outside the image.
static float zncc(const uint8_t *img, int w, int h, int x0, int y0, const uint8_t *t, int32_t st, int32_t st2) {
    if (x0 < 0 || y0 < 0 || x0 + T > w || y0 + T > h) return -1;
    int32_t si = 0, si2 = 0, sti = 0;
    for (int r = 0; r < T; r++) {
        const uint8_t *row = img + (y0 + r) * w + x0, *tr = t + r * T;
        for (int c = 0; c < T; c++) { int32_t v = row[c]; si += v; si2 += v * v; sti += v * tr[c]; }
    }
    const float n = T * T;
    float vt = (float)st2 - (float)st * (float)st / n, vi = (float)si2 - (float)si * (float)si / n;
    if (vt < 1 || vi < 1) return 0;
    return ((float)sti - (float)st * (float)si / n) / sqrtf(vt * vi);
}

static bool take_templates(track_t *t, const uint8_t *img, float x, float y) {
    int tx0 = (int)lroundf(x) - T / 2, ty0 = (int)lroundf(y) - T / 2;
    int hx0 = (int)lroundf((x - 0.5f) / 2) - T / 2, hy0 = (int)lroundf((y - 0.5f) / 2) - T / 2;
    if (tx0 < 0 || ty0 < 0 || tx0 + T > W || ty0 + T > H || hx0 < 0 || hy0 < 0 || hx0 + T > HW || hy0 + T > HH) return false;
    for (int r = 0; r < T; r++) {
        memcpy(t->tpl + r * T, img + (ty0 + r) * W + tx0, T);
        memcpy(t->tplh + r * T, half + (hy0 + r) * HW + hx0, T);
    }
    sums(t->tpl, &t->sum, &t->sum2);
    sums(t->tplh, &t->sumh, &t->sumh2);
    t->tx0 = tx0; t->ty0 = ty0; t->hx0 = hx0; t->hy0 = hy0;
    t->x = t->ax = x;
    t->y = t->ay = y;
    return true;
}

// The vertex of a parabola through three scores, as an offset from the middle (±0.5).
static float vertex(float a, float b, float c) {
    float d = a - 2 * b + c;
    if (d >= 0) return 0;
    float v = 0.5f * (a - c) / d;
    return v < -0.5f ? -0.5f : v > 0.5f ? 0.5f : v;
}

static float bilinear(const uint8_t *img, float x, float y) {
    int x0 = (int)x, y0 = (int)y;
    float fx = x - (float)x0, fy = y - (float)y0;
    const uint8_t *p = img + y0 * W + x0;
    return (1 - fy) * ((1 - fx) * p[0] + fx * p[1]) + fy * ((1 - fx) * p[W] + fx * p[W + 1]);
}

// The sub-pixel part by Lucas-Kanade: the template's best place near (x0 + u, y0 + v),
// both patches made zero-mean and unit-variance (blind to gain and offset, as the
// ZNCC). False if it doesn't settle within a pixel (the parabola's estimate stays).
static bool refine(const uint8_t *img, int x0, int y0, const track_t *t, float *u, float *v) {
    if (x0 < 3 || y0 < 3 || x0 + T + 3 > W || y0 + T + 3 > H) return false;
    const float n = T * T, tm = (float)t->sum / n, tv = (float)t->sum2 / n - tm * tm;
    if (tv < 1) return false;
    const float ts = sqrtf(tv);
    float uu = *u, vv = *v;
    for (int step = 0; step < LK_STEPS; step++) {
        float val[T * T], gx[T * T], gy[T * T], si = 0, si2 = 0;
        for (int j = 0; j < T; j++)
            for (int i = 0; i < T; i++) {
                float x = (float)(x0 + i) + uu, y = (float)(y0 + j) + vv;
                int k = j * T + i;
                val[k] = bilinear(img, x, y);
                gx[k] = 0.5f * (bilinear(img, x + 1, y) - bilinear(img, x - 1, y));
                gy[k] = 0.5f * (bilinear(img, x, y + 1) - bilinear(img, x, y - 1));
                si += val[k];
                si2 += val[k] * val[k];
            }
        float im = si / n, iv = si2 / n - im * im;
        if (iv < 1) return false;
        float is = sqrtf(iv), hxx = 0, hyy = 0, hxy = 0, bx = 0, by = 0;
        for (int k = 0; k < T * T; k++) {
            float e = (val[k] - im) / is - ((float)t->tpl[k] - tm) / ts, ax = gx[k] / is, ay = gy[k] / is;
            hxx += ax * ax; hyy += ay * ay; hxy += ax * ay; bx += ax * e; by += ay * e;
        }
        float det = hxx * hyy - hxy * hxy;
        if (det < 1e-6f) return false;
        float du = -(hyy * bx - hxy * by) / det, dv = -(hxx * by - hxy * bx) / det;
        uu += du;
        vv += dv;
        if (fabsf(uu) > 1.5f || fabsf(vv) > 1.5f) return false;
        if (fabsf(du) < LK_DONE && fabsf(dv) < LK_DONE) break;
    }
    *u = uu;
    *v = vv;
    return true;
}

typedef enum { FOUND, NOT_FOUND, OUTSIDE } search_t;

// Searches the feature's templates: FOUND, with how far they moved since they were
// taken (sub-pixel) and the match's score. A best place on the edge of the search window may have a
// better one beyond it: NOT_FOUND, or OUTSIDE if beyond it is outside the image.
static search_t search(const track_t *t, const uint8_t *img, float *mx, float *my, float *score) {
    float s[2 * REACH + 1][2 * REACH + 1];
    float best = -2;
    int bi = 0, bj = 0;
    // The window's centre: its last place moved by its own last move, as many times as
    // frames since it was measured; from where the templates were taken, in half pixels.
    int ox = (int)lroundf((t->x - t->ax + t->vx * (float)(t->misses + 1)) / 2);
    int oy = (int)lroundf((t->y - t->ay + t->vy * (float)(t->misses + 1)) / 2);
    for (int j = -REACH; j <= REACH; j++)
        for (int i = -REACH; i <= REACH; i++) {
            float v = zncc(half, HW, HH, t->hx0 + ox + i, t->hy0 + oy + j, t->tplh, t->sumh, t->sumh2);
            s[j + REACH][i + REACH] = v;
            if (v > best) { best = v; bi = i; bj = j; }
        }
    if (best < ACCEPT - 0.2f) return NOT_FOUND; // the half-size image is smoother: some slack
    if (abs(bi) == REACH || abs(bj) == REACH) return NOT_FOUND;
    // The place next to the best, in each direction, outside the image: it may be there.
    for (int d = 0; d < 4; d++) {
        int i = bi + (d == 0) - (d == 1), j = bj + (d == 2) - (d == 3);
        if (s[j + REACH][i + REACH] < -1.5f) return OUTSIDE;
    }
    float second = -2;
    for (int j = -REACH; j <= REACH; j++)
        for (int i = -REACH; i <= REACH; i++)
            if ((abs(i - bi) > 1 || abs(j - bj) > 1) && s[j + REACH][i + REACH] > second) second = s[j + REACH][i + REACH];
    if (best - second < UNIQUE) return NOT_FOUND; // two places alike: a repeated pattern
    // Full size: climb from the coarse place to the best of its 3 x 3 neighbours.
    int x0 = t->tx0 + 2 * (ox + bi), y0 = t->ty0 + 2 * (oy + bj);
    float c[3][3];
    for (int step = 0;; step++) {
        int ci = 0, cj = 0;
        float top = -2;
        for (int j = -1; j <= 1; j++)
            for (int i = -1; i <= 1; i++) {
                float v = zncc(img, W, H, x0 + i, y0 + j, t->tpl, t->sum, t->sum2);
                c[j + 1][i + 1] = v;
                if (v > top) { top = v; ci = i; cj = j; }
            }
        if ((ci == 0 && cj == 0) || step == CLIMB) break;
        x0 += ci;
        y0 += cj;
    }
    if (c[1][1] < ACCEPT) return NOT_FOUND;
    float fx = c[1][0] < -1.5f || c[1][2] < -1.5f ? 0 : vertex(c[1][0], c[1][1], c[1][2]);
    float fy = c[0][1] < -1.5f || c[2][1] < -1.5f ? 0 : vertex(c[0][1], c[1][1], c[2][1]);
    refine(img, x0, y0, t, &fx, &fy);
    *mx = (float)(x0 - t->tx0) + fx;
    *my = (float)(y0 - t->ty0) + fy;
    *score = c[1][1];
    return FOUND;
}

// --- finding ---

static const int8_t ring[16][2] = {{0, -3}, {1, -3}, {2, -2}, {3, -1}, {3, 0}, {3, 1}, {2, 2}, {1, 3},
                                    {0, 3}, {-1, 3}, {-2, 2}, {-3, 1}, {-3, 0}, {-3, -1}, {-2, -2}, {-1, -3}};

// FAST-9: 9 pixels in a row on the ring all brighter than the centre by t, or all darker.
static bool fast9(const uint8_t *img, int x, int y, int t) {
    const uint8_t *p = img + y * W + x;
    int hi = p[0] + t, lo = p[0] - t, bright = 0, dark = 0;
    for (int k = 0; k < 16; k += 4) { int v = p[ring[k][1] * W + ring[k][0]]; bright += v > hi; dark += v < lo; }
    if (bright < 2 && dark < 2) return false; // 9 in a row hold at least 2 of these 4
    uint32_t mb = 0, md = 0;
    for (int k = 0; k < 16; k++) {
        int v = p[ring[k][1] * W + ring[k][0]];
        if (v > hi) mb |= 1u << k;
        if (v < lo) md |= 1u << k;
    }
    mb |= mb << 16;
    md |= md << 16;
    for (int k = 0; k < 16; k++)
        if (((mb >> k) & 0x1ff) == 0x1ff || ((md >> k) & 0x1ff) == 0x1ff) return true;
    return false;
}

// The smaller eigenvalue of the gradient matrix over 7 x 7 px, and it over the
// patch's gradient energy.
static float shi_tomasi(const uint8_t *img, int x, int y, float *strength) {
    float sxx = 0, syy = 0, sxy = 0;
    for (int j = -ST_WINDOW; j <= ST_WINDOW; j++)
        for (int i = -ST_WINDOW; i <= ST_WINDOW; i++) {
            const uint8_t *p = img + (y + j) * W + x + i;
            float gx = 0.5f * (float)(p[1] - p[-1]), gy = 0.5f * (float)(p[W] - p[-W]);
            sxx += gx * gx; syy += gy * gy; sxy += gx * gy;
        }
    float m = 0.5f * (sxx + syy), d = sqrtf(0.25f * (sxx - syy) * (sxx - syy) + sxy * sxy), lmin = m - d;
    *strength = m > 0 ? lmin / m : 0;
    return lmin;
}

static int live_near(float x, float y, float r) {
    int n = 0;
    for (int k = 0; k < VISION_MAX_FEATURES; k++)
        if (tracks[k].live && !tracks[k].lost && fabsf(tracks[k].x - x) < r && fabsf(tracks[k].y - y) < r &&
            hypotf(tracks[k].x - x, tracks[k].y - y) < r) n++;
    return n;
}

static int n_live(void) {
    int n = 0;
    for (int k = 0; k < VISION_MAX_FEATURES; k++) n += tracks[k].live && !tracks[k].lost;
    return n;
}

static track_t *free_slot(void) {
    for (int k = 0; k < VISION_MAX_FEATURES; k++) if (!tracks[k].live) return &tracks[k];
    return NULL;
}

typedef struct { int x, y; float lmin, strength; } candidate_t;

// New features in the cells that have fewer than 2.
static void find_new(const uint8_t *img, float gain) {
    float noise = noise_sigma(gain);
    int floor_t = (int)ceilf(3 * noise);
    float lmin_floor = 2.0f * (2 * ST_WINDOW + 1) * (2 * ST_WINDOW + 1) * noise * noise;
    for (int cy = 0; cy < H / CELL; cy++)
        for (int cx = 0; cx < W / CELL; cx++) {
            int have = 0;
            for (int k = 0; k < VISION_MAX_FEATURES; k++) {
                const track_t *t = &tracks[k];
                have += t->live && !t->lost && (int)t->x / CELL == cx && (int)t->y / CELL == cy;
            }
            if (have >= PER_CELL || n_live() >= VISION_MAX_FEATURES) continue;
            int x_lo = cx * CELL < EDGE ? EDGE : cx * CELL, x_hi = (cx + 1) * CELL > W - EDGE ? W - EDGE : (cx + 1) * CELL;
            int y_lo = cy * CELL < EDGE ? EDGE : cy * CELL, y_hi = (cy + 1) * CELL > H - EDGE ? H - EDGE : (cy + 1) * CELL;
            candidate_t cand[24];
            int n = 0;
            for (int k = 0; k < TRIES && n < 4; k++) {
                if (fast_thresholds[k] < floor_t && k > 0 && fast_thresholds[k - 1] <= floor_t) break;
                int t = fast_thresholds[k] < floor_t ? floor_t : fast_thresholds[k];
                n = 0;
                for (int y = y_lo; y < y_hi && n < 24; y++)
                    for (int x = x_lo; x < x_hi && n < 24; x++)
                        if (fast9(img, x, y, t)) cand[n++] = (candidate_t){x, y, 0, 0};
            }
            for (int i = 0; i < n; i++) cand[i].lmin = shi_tomasi(img, cand[i].x, cand[i].y, &cand[i].strength);
            for (; have < PER_CELL; have++) { // the best that is far enough from the others
                int best = -1;
                for (int i = 0; i < n; i++)
                    if (cand[i].lmin >= lmin_floor && (best < 0 || cand[i].lmin > cand[best].lmin) &&
                        live_near((float)cand[i].x, (float)cand[i].y, SPACING) == 0) best = i;
                if (best < 0) break;
                track_t *t = free_slot();
                if (!t) return;
                memset(t, 0, sizeof *t);
                if (!take_templates(t, img, (float)cand[best].x, (float)cand[best].y)) { cand[best].lmin = -1; have--; continue; }
                t->live = true;
                t->id = next_id++;
                if (next_id == 0) next_id = 1;
                t->strength = cand[best].strength;
                cand[best].lmin = -1;
            }
        }
}

// --- the record ---

static void half_image(const uint8_t *img) {
    for (int y = 0; y < HH; y++)
        for (int x = 0; x < HW; x++) {
            const uint8_t *p = img + 2 * y * W + 2 * x;
            half[y * HW + x] = (uint8_t)((p[0] + p[1] + p[W] + p[W + 1] + 2) / 4);
        }
}

static void brightness(const uint8_t *img) {
    uint32_t sum = 0, n = 0, sat = 0, all = 0;
    for (int y = 0; y < H; y += SAMPLE_STEP)
        for (int x = 0; x < W; x += SAMPLE_STEP) {
            int v = img[y * W + x];
            all++;
            if (v >= SATURATED) sat++;
            else { sum += (uint32_t)v; n++; }
        }
    rec.brightness = n ? (float)sum / (float)n : 255;
    rec.saturated = (float)sat / (float)all;
}

static void regions(void) {
    memset(rec.regions, 0, sizeof rec.regions);
    const int r = CELL / 2; // a region in the half-size image
    for (int ry = 0; ry < VISION_REGION_ROWS; ry++)
        for (int rx = 0; rx < VISION_REGION_COLS; rx++) {
            vision_region_t *g = &rec.regions[ry][rx];
            uint32_t grad = 0, sum = 0, n = 0;
            for (int y = ry * r; y < (ry + 1) * r; y++)
                for (int x = rx * r; x < (rx + 1) * r; x++) {
                    const uint8_t *p = half + y * HW + x;
                    sum += *p;
                    if (x == 0 || y == 0 || x == HW - 1 || y == HH - 1) continue;
                    grad += (uint32_t)(abs(p[1] - p[-1]) + abs(p[HW] - p[-HW]));
                    n++;
                }
            g->brightness = (float)sum / (float)(r * r);
            g->texture = n ? (float)grad / (float)n / 4 : 0; // per full-size px: two differences over 2 half px
            float dxs[VISION_MAX_FEATURES], dys[VISION_MAX_FEATURES];
            int m = 0;
            for (int k = 0; k < rec.n_features; k++) {
                const vision_feature_t *f = &rec.features[k];
                if ((int)f->x / CELL != rx || (int)f->y / CELL != ry) continue;
                if (f->flags & FEATURE_LOST) { g->lost++; continue; }
                g->features++;
                if (f->flags & FEATURE_NEW) g->born++;
                else if (f->flags & FEATURE_MISSED) g->missed++;
                else { dxs[m] = f->dx; dys[m++] = f->dy; }
            }
            if (m) { g->dx = median_value(dxs, m); g->dy = median_value(dys, m); }
        }
}

void vision_reset(void) {
    memset(tracks, 0, sizeof tracks);
    have_rec = false;
}

const vision_frame_t *vision_process(const vision_image_t *im, const vision_pose_t *pose) {
    const uint8_t *img = im->pixels;
    for (int k = 0; k < VISION_MAX_FEATURES; k++) if (tracks[k].lost) tracks[k].live = tracks[k].lost = false;
    half_image(img);
    uint32_t seq = have_rec ? rec.seq + 1 : 1;
    memset(&rec, 0, sizeof rec);
    rec.seq = seq;
    rec.camera_number = im->number;
    rec.row0_us = im->row0_us;
    rec.line_us = im->line_us;
    rec.t_us = im->row0_us + (uint32_t)(59.5f * im->line_us);
    rec.dt_s = have_rec ? (float)(rec.t_us - prev_t_us) * 1e-6f : 0;
    rec.exposure_us = im->exposure_us;
    rec.gain = im->gain;
    if (pose) rec.pose = *pose;
    rec.prev_pose = prev_pose;
    brightness(img);

    // Follow every live feature.
    for (int k = 0; k < VISION_MAX_FEATURES; k++) {
        track_t *t = &tracks[k];
        if (!t->live) continue;
        vision_feature_t *f = &rec.features[rec.n_features++];
        f->id = t->id;
        f->age = ++t->age;
        f->strength = t->strength;
        float mx, my, score;
        // Where its own move would take it outside the part of the image it can be followed in: leaving.
        float px = t->x + t->vx * (float)(t->misses + 1), py = t->y + t->vy * (float)(t->misses + 1);
        bool leaving = px < EDGE || py < EDGE || px > W - EDGE - 1 || py > H - EDGE - 1;
        search_t found = leaving ? OUTSIDE : search(t, img, &mx, &my, &score);
        if (found == FOUND) {
            float nx = t->ax + mx, ny = t->ay + my;
            f->x = nx; f->y = ny; f->dx = nx - t->x; f->dy = ny - t->y; f->match = score;
            t->vx = f->dx / (float)(t->misses + 1);
            t->vy = f->dy / (float)(t->misses + 1);
            t->misses = 0;
            t->x = nx;
            t->y = ny;
            bool retake = hypotf(mx, my) >= RETAKE_PX || score < RETAKE_MATCH;
            if (retake && !take_templates(t, img, nx, ny)) { f->flags = FEATURE_LOST | FEATURE_LEFT_IMAGE; t->lost = true; }
            else rec.n_found++;
        } else {
            f->x = t->x; f->y = t->y;
            if (found == OUTSIDE) { f->flags = FEATURE_LOST | FEATURE_LEFT_IMAGE; t->lost = true; }
            else if (++t->misses > MAX_MISSES) { f->flags = FEATURE_LOST; t->lost = true; }
            else f->flags = FEATURE_MISSED;
        }
    }
    // New ones where there are too few.
    uint16_t first_new = next_id;
    find_new(img, im->gain);
    for (int k = 0; k < VISION_MAX_FEATURES && rec.n_features < VISION_MAX_FEATURES; k++) {
        const track_t *t = &tracks[k];
        if (!t->live || t->age != 0 || (uint16_t)(t->id - first_new) >= (uint16_t)(next_id - first_new)) continue;
        vision_feature_t *f = &rec.features[rec.n_features++];
        *f = (vision_feature_t){.id = t->id, .flags = FEATURE_NEW, .x = t->x, .y = t->y, .strength = t->strength, .match = 1};
    }
    for (int k = 0; k < rec.n_features; k++)
        rec.features[k].t_us = rec.row0_us + (uint32_t)(rec.features[k].y * rec.line_us);
    regions();
    prev_t_us = rec.t_us;
    prev_pose = rec.pose;
    have_rec = true;
    return &rec;
}

const vision_frame_t *vision_latest(void) { return have_rec ? &rec : NULL; }

// --- on the robot ---

static uint32_t last_number, processed_number, frames;
static bool requested;
static uint32_t window_start_us, window_frames, window_us, window_max_us, window_born, window_lost;
static vision_status_t status;

void vision_request(void) { requested = true; }

void vision_update(void) {
    camera_frame_t f;
    if (!camera_last_frame(&f) || f.number == last_number) return;
    last_number = f.number;
    if (!requested && have_rec && f.number - processed_number < EVERY) return;
    requested = false;
    processed_number = f.number;
    uint32_t start = time_us_32();
    vision_image_t im = {.pixels = f.pixels, .number = f.number, .row0_us = camera_row_time(&f, 0),
                         .line_us = f.line_us, .exposure_us = f.exposure_us, .gain = f.gain};
    vision_pose_t pose = {0};
    pose.ok = pose_at(im.row0_us, &pose.first_row) &&
              pose_at(im.row0_us + (uint32_t)((float)(H - 1) * f.line_us), &pose.last_row);
    const vision_frame_t *r = vision_process(&im, &pose);
    uint32_t took = time_us_32() - start;
    frames++;
    window_frames++;
    window_us += took;
    if (took > window_max_us) window_max_us = took;
    float s = 0;
    int n = 0;
    for (int k = 0; k < r->n_features; k++) {
        const vision_feature_t *v = &r->features[k];
        window_born += (v->flags & FEATURE_NEW) != 0;
        window_lost += (v->flags & FEATURE_LOST) != 0;
        if (v->flags) continue;
        s += v->dx * v->dx + v->dy * v->dy;
        n += 2;
    }
    status.jitter_px = n ? sqrtf(s / (float)n) : 0;
    if (start - window_start_us >= 1000000) { // a second's numbers
        status.per_s = (float)window_frames * 1e6f / (float)(start - window_start_us);
        status.ms_mean = window_frames ? (float)window_us / (float)window_frames / 1000 : 0;
        status.ms_max = (float)window_max_us / 1000;
        status.born_1s = window_born;
        status.lost_1s = window_lost;
        window_start_us = start;
        window_frames = window_us = window_max_us = window_born = window_lost = 0;
    }
}

void vision_status(vision_status_t *s) { *s = status; s->frames = frames; }

void vision_print_status(void) {
    const vision_frame_t *r = vision_latest();
    if (!r) { printf("Vision: no frame processed yet (camera working?)\n"); return; }
    vision_status_t s;
    vision_status(&s);
    printf("Vision: %.1f frames/s, %.1f ms per frame (max %.1f), %u features (%lu new, %lu lost in the last second),"
           " jitter %.2f px; frame %lu, exposure %.1f ms x %.1f, brightness %.0f\n",
           (double)s.per_s, (double)s.ms_mean, (double)s.ms_max, (unsigned)r->n_features, (unsigned long)s.born_1s,
           (unsigned long)s.lost_1s, (double)s.jitter_px, (unsigned long)r->camera_number,
           (double)(r->exposure_us / 1000), (double)r->gain, (double)r->brightness);
    printf("Features per region (texture in brackets), as the robot sees it:\n");
    for (int y = 0; y < VISION_REGION_ROWS; y++) {
        for (int x = 0; x < VISION_REGION_COLS; x++)
            printf(" %2u (%4.1f)", (unsigned)r->regions[y][x].features, (double)r->regions[y][x].texture);
        printf("\n");
    }
    printf("Median move per region, px (dx,dy):\n");
    for (int y = 0; y < VISION_REGION_ROWS; y++) {
        for (int x = 0; x < VISION_REGION_COLS; x++)
            printf(" %+4.1f,%+4.1f", (double)r->regions[y][x].dx, (double)r->regions[y][x].dy);
        printf("\n");
    }
}
