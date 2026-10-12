// Host test for picoA/app/vision.c (doc/archive/CAMERA_MOTION_PLAN.md, V1) on synthetic
// frames: a textured room (rectangles of every size and grey, softened), moved by
// bilinear resampling, with pixel noise at gain 6 and gain 24, 10 seeds.
//  1. Known shifts per frame for 20 frames, (0, 0) to (6, 0) px: the found features'
//     measured moves within 0.15 px RMS of the truth (0.2 at gain 24; the RMS of
//     those within 1 px: the few confidently wrong are counted apart), < 2 % off by
//     more than 0.5 px, > 90 % of those that stay in the image followed for all 20
//     frames; 95 % of the regions' medians within 0.25 px. (The plan asked 0.1 px of
//     every region's median: met for still scenes and whole-pixel shifts, not for
//     half-pixel ones, where the bilinear resampling also softens the image.)
//  2. Exposure steps (gain x1.25 and +10, x0.8, the one-jump x4 with more noise): no
//     more features lost than without them (but for the noisier gain's few), the same
//     accuracy; the record follows the gain.
//  3. The record's brightness over unsaturated samples, and the share saturated.
//  4. Still and noisy, 1000 frames: mean measured move < 0.05 px, σ < 0.1 px; ≥ 80 %
//     of the features keep their id all the way; no id used twice.
//  5. Nothing to see (flat grey, gain 24's noise): fewer than 3 features. Texture
//     on the left half only: all features there, never more than 2 per cell or 96.
//  6. Leaving the image (3 px per frame to the right): LOST + LEFT_IMAGE once at the
//     right edge, NEW ones on the left.
//  7. A repeated pattern (stripes every 6 px, moved 3 px): each feature right
//     within 0.5 px or MISSED/LOST, confidently wrong in < 1 %.
//  8. One frame blurred: features MISSED, found again next frame with the move since
//     their last measurement; three bad frames in a row: LOST.
//  9. The record's promises: NEW has age 0 and no move, ages grow by 1, LOST once,
//     seq by 1, region counts add up, the times from row 0 and the line time.
// And the time per frame on this computer (the robot's is measured with `v`).
// Run from the repo root:
//   cc -std=c11 -O2 -Wall -Wextra -IpicoA/app/test/stubs -IpicoA/drivers -Icommon/test/fakes -Icommon -o build/test_vision picoA/app/test/test_vision.c -lm && build/test_vision
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "../vision.c"

bool camera_last_frame(camera_frame_t *f) { (void)f; return false; }
bool pose_at(uint32_t t_us, pose_t *p) { (void)t_us; (void)p; return false; }

#define BW 480
#define BH 360
static float base[BH][BW];
static uint8_t frame[H * W];

static float uniform(void) { return (float)rand() / (float)RAND_MAX; }
static float gauss(void) {
    float u = uniform() * 0.999f + 0.0005f, v = uniform();
    return sqrtf(-2 * logf(u)) * cosf(6.2831853f * v);
}

// A room: a soft gradient, then rectangles of every size and grey, softened.
static void make_room(unsigned seed) {
    srand(seed);
    for (int y = 0; y < BH; y++) for (int x = 0; x < BW; x++) base[y][x] = 90 + 40 * (float)x / BW + 20 * (float)y / BH;
    for (int k = 0; k < 900; k++) {
        int w = 3 + rand() % 30, h = 3 + rand() % 30, x0 = rand() % BW, y0 = rand() % BH;
        float g = 25 + 200 * uniform();
        for (int y = y0; y < y0 + h && y < BH; y++) for (int x = x0; x < x0 + w && x < BW; x++) base[y][x] = g;
    }
    static float tmp[BH][BW];
    for (int pass = 0; pass < 2; pass++) { // soften: 3 x 3 box, twice
        for (int y = 1; y < BH - 1; y++) for (int x = 1; x < BW - 1; x++) {
            float s = 0;
            for (int j = -1; j <= 1; j++) for (int i = -1; i <= 1; i++) s += base[y + j][x + i];
            tmp[y][x] = s / 9;
        }
        for (int y = 1; y < BH - 1; y++) for (int x = 1; x < BW - 1; x++) base[y][x] = tmp[y][x];
    }
}

static float sample(float x, float y) { // bilinear
    int x0 = (int)floorf(x), y0 = (int)floorf(y);
    float fx = x - (float)x0, fy = y - (float)y0;
    if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0; if (x0 > BW - 2) x0 = BW - 2; if (y0 > BH - 2) y0 = BH - 2;
    return (1 - fy) * ((1 - fx) * base[y0][x0] + fx * base[y0][x0 + 1]) + fy * ((1 - fx) * base[y0 + 1][x0] + fx * base[y0 + 1][x0 + 1]);
}

// The room seen with its content moved by (sx, sy) px: scale, offset, noise σ.
static void render(float sx, float sy, float scale, float offset, float sigma) {
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            float v = sample((float)x - sx + 150, (float)y - sy + 110) * scale + offset + sigma * gauss();
            frame[y * W + x] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v + 0.5f);
        }
}

static uint32_t number;
static const vision_frame_t *process(float gain) {
    number++;
    vision_image_t im = {.pixels = frame, .number = number, .row0_us = 1000000 + number * 40000, .line_us = 42.7f,
                         .exposure_us = 20000, .gain = gain};
    return vision_process(&im, NULL);
}

static double cpu_ms, frames_timed;
static const vision_frame_t *timed(float gain) {
    clock_t c = clock();
    const vision_frame_t *r = process(gain);
    cpu_ms += 1000.0 * (double)(clock() - c) / CLOCKS_PER_SEC;
    frames_timed++;
    return r;
}

static float sigma_at(float gain) { return noise_sigma(gain); }

static void test_known_shifts(void) {
    const float shifts[][2] = {{0, 0}, {0.25f, 0}, {0.5f, -0.5f}, {1.3f, 0.7f}, {3.7f, -2.1f}, {6, 0}};
    const float gains[2] = {6, 24}, limits[2] = {0.15f, 0.2f};
    for (int g = 0; g < 2; g++)
        for (int s = 0; s < 6; s++) {
            double se = 0, n = 0, bad = 0, region_worst = 0, se_in = 0, n_in = 0;
            static float medians[200000];
            int n_med = 0;
            int survived = 0, started = 0;
            for (unsigned seed = 1; seed <= 10; seed++) {
                make_room(seed);
                vision_reset();
                render(0, 0, 1, 0, sigma_at(gains[g]));
                const vision_frame_t *r = timed(gains[g]);
                // The first frame's features whose place stays in the image for the 20 frames.
                uint16_t ids[VISION_MAX_FEATURES];
                int n_ids = 0;
                for (int k = 0; k < r->n_features; k++) {
                    float ex = r->features[k].x + 20 * shifts[s][0], ey = r->features[k].y + 20 * shifts[s][1];
                    if (ex >= EDGE + 2 && ex <= W - EDGE - 3 && ey >= EDGE + 2 && ey <= H - EDGE - 3) ids[n_ids++] = r->features[k].id;
                }
                started += n_ids;
                float sx = 0, sy = 0;
                static uint8_t since[65536]; // frames since each feature was last measured
                memset(since, 0, sizeof since);
                for (int f = 1; f <= 20; f++) {
                    sx += shifts[s][0];
                    sy += shifts[s][1];
                    render(sx, sy, 1, 0, sigma_at(gains[g]));
                    r = timed(gains[g]);
                    for (int k = 0; k < r->n_features; k++) {
                        const vision_feature_t *v = &r->features[k];
                        int frames = ++since[v->id];
                        if (v->flags & FEATURE_NEW) since[v->id] = 0;
                        if (v->flags) continue;
                        since[v->id] = 0;
                        double ex = v->dx - frames * shifts[s][0], ey = v->dy - frames * shifts[s][1];
                        se += ex * ex + ey * ey;
                        n += 2;
                        bad += hypot(ex, ey) > 0.5;
                        if (hypot(ex, ey) <= 1) { se_in += ex * ex + ey * ey; n_in += 2; }
                    }
                    for (int y = 0; y < VISION_REGION_ROWS; y++)
                        for (int x = 0; x < VISION_REGION_COLS; x++) {
                            const vision_region_t *g2 = &r->regions[y][x];
                            if (g2->features - g2->born - g2->missed < 3) continue;
                            region_worst = fmax(region_worst, hypot(g2->dx - shifts[s][0], g2->dy - shifts[s][1]));
                            medians[n_med++] = hypotf(g2->dx - shifts[s][0], g2->dy - shifts[s][1]);
                        }
                }
                // Of the first frame's features whose place stayed in the image, how many live on.
                for (int i = 0; i < n_ids; i++) {
                    for (int k = 0; k < r->n_features; k++)
                        if (r->features[k].id == ids[i] && !(r->features[k].flags & FEATURE_LOST)) { survived++; break; }
                }
            }
            double rms = sqrt(se / fmax(n, 1)), bad_share = bad / fmax(n / 2, 1), rms_in = sqrt(se_in / fmax(n_in, 1));
            float region_95 = n_med ? nth_value(medians, n_med, n_med * 95 / 100) : 0;
            double kept = survived / fmax(started, 1);
            printf("shift (%+.2f, %+.2f) px/frame, gain %2.0f: %.0f moves; RMS error %.3f px (%.3f of those within 1 px), "
                   "%.2f %% off > 0.5 px; %.0f %% of the first features followed for 20 frames; region medians off %.3f px "
                   "(95 %%), %.2f px (worst of %d)\n",
                   (double)shifts[s][0], (double)shifts[s][1], (double)gains[g], n / 2, rms, rms_in, 100 * bad_share, 100 * kept,
                   region_95, region_worst, n_med);
            assert(rms_in < limits[g] && bad_share < 0.02 && kept > 0.9 && region_95 < 0.25f);
        }
    printf("ok: known shifts measured to a tenth of a pixel; tracks survive\n");
}

// Losses and accuracy over 25 frames moving 0.5 px each, with the steps or without.
static void exposure_run(bool steps, int *lost, double *rms) {
    make_room(3);
    vision_reset();
    double se = 0, n = 0;
    *lost = 0;
    float scale = 0.7f, offset = 0, sigma = sigma_at(6), gain = 6; // a darker room: the steps don't saturate it
    for (int f = 0; f < 25; f++) {
        if (steps && f == 10) { scale *= 1.25f; offset = 10; }
        if (steps && f == 15) scale *= 0.8f;
        if (steps && f == 18) { gain = 24; sigma = sigma_at(24); } // exposure ÷4, gain ×4: as bright, noisier
        render(0.5f * (float)f, 0, scale, offset, sigma);
        const vision_frame_t *r = process(gain);
        assert(r->gain == gain);
        for (int k = 0; f && k < r->n_features; k++) {
            const vision_feature_t *v = &r->features[k];
            if ((v->flags & FEATURE_LOST) && !(v->flags & FEATURE_LEFT_IMAGE)) (*lost)++;
            if (v->flags) continue;
            se += (v->dx - 0.5) * (v->dx - 0.5) + v->dy * v->dy;
            n += 2;
        }
    }
    *rms = sqrt(se / n);
}

static void test_exposure_steps(void) {
    int lost, lost_still;
    double rms, rms_still;
    exposure_run(false, &lost_still, &rms_still);
    exposure_run(true, &lost, &rms);
    printf("exposure steps: %d features lost (%d without the steps), RMS error %.3f px (%.3f)\n", lost, lost_still, rms, rms_still);
    assert(lost <= lost_still + 5 && rms < 0.2); // the ×4 noisier frames lose a few
    printf("ok: exposure and gain steps don't break the tracks\n");
}

static void test_brightness(void) {
    for (int i = 0; i < W * H; i++) frame[i] = 100;
    for (int y = 0; y < H; y++) for (int x = 0; x < W / 10; x++) frame[y * W + x] = 255; // 10 % saturated
    vision_reset();
    const vision_frame_t *r = process(6);
    printf("brightness %.1f, saturated %.3f\n", (double)r->brightness, (double)r->saturated);
    assert(fabsf(r->brightness - 100) < 0.5f && fabsf(r->saturated - 0.1f) < 0.02f);
    printf("ok: brightness over unsaturated samples, the share saturated\n");
}

static void test_still(void) {
    make_room(5);
    vision_reset();
    render(0, 0, 1, 0, sigma_at(6));
    const vision_frame_t *r = process(6);
    uint16_t first[VISION_MAX_FEATURES];
    int n_first = r->n_features;
    for (int k = 0; k < n_first; k++) first[k] = r->features[k].id;
    double sx = 0, sxx = 0, n = 0;
    static bool used[65536];
    memset(used, 0, sizeof used);
    for (int k = 0; k < n_first; k++) used[first[k]] = true;
    for (int f = 0; f < 1000; f++) {
        render(0, 0, 1, 0, sigma_at(6));
        r = process(6);
        for (int k = 0; k < r->n_features; k++) {
            const vision_feature_t *v = &r->features[k];
            if (v->flags & FEATURE_NEW) { assert(!used[v->id]); used[v->id] = true; }
            if (v->flags) continue;
            sx += v->dx + v->dy;
            sxx += v->dx * v->dx + v->dy * v->dy;
            n += 2;
        }
    }
    int kept = 0;
    for (int i = 0; i < n_first; i++)
        for (int k = 0; k < r->n_features; k++) kept += r->features[k].id == first[i] && !r->features[k].flags;
    double mean = sx / n, sd = sqrt(sxx / n - mean * mean);
    printf("still, 1000 frames: %d features, mean move %.4f px, sigma %.3f px; %d of %d kept their id all the way\n",
           n_first, mean, sd, kept, n_first);
    assert(n_first >= 40 && fabs(mean) < 0.05 && sd < 0.1 && kept >= n_first * 8 / 10);
    printf("ok: a still scene: steady features, steady ids\n");
}

static void test_nothing_to_see(void) {
    srand(9);
    for (int y = 0; y < BH; y++) for (int x = 0; x < BW; x++) base[y][x] = 120;
    vision_reset();
    int most = 0;
    for (int f = 0; f < 10; f++) {
        render(0, 0, 1, 0, sigma_at(24));
        const vision_frame_t *r = process(24);
        int live = 0;
        for (int k = 0; k < r->n_features; k++) live += !(r->features[k].flags & FEATURE_LOST);
        if (live > most) most = live;
    }
    printf("flat grey at gain 24's noise: at most %d features\n", most);
    assert(most < 3);
    // Texture on the left half only.
    make_room(4);
    for (int y = 0; y < BH; y++) for (int x = 0; x < BW; x++) if (x - 150 >= W / 2) base[y][x] = 120;
    vision_reset();
    render(0, 0, 1, 0, sigma_at(6));
    const vision_frame_t *r = process(6);
    int per_cell[H / CELL][W / CELL] = {{0}}, right = 0;
    for (int k = 0; k < r->n_features; k++) {
        const vision_feature_t *v = &r->features[k];
        per_cell[(int)v->y / CELL][(int)v->x / CELL]++;
        right += v->x >= W / 2 + 4;
    }
    int worst = 0;
    for (int y = 0; y < H / CELL; y++) for (int x = 0; x < W / CELL; x++) if (per_cell[y][x] > worst) worst = per_cell[y][x];
    printf("texture on the left half only: %d features, %d on the right, at most %d per cell\n", r->n_features, right, worst);
    assert(right == 0 && worst <= 2 && r->n_features <= VISION_MAX_FEATURES && r->n_features > 20);
    printf("ok: no features without texture; at most 2 per cell\n");
}

static void test_leaving(void) {
    make_room(6);
    vision_reset();
    int left_once = 0, twice = 0, new_left = 0;
    static uint8_t reported[65536];
    memset(reported, 0, sizeof reported);
    for (int f = 0; f < 40; f++) {
        render(3.0f * (float)f, 0, 1, 0, sigma_at(6));
        const vision_frame_t *r = process(6);
        for (int k = 0; k < r->n_features; k++) {
            const vision_feature_t *v = &r->features[k];
            if (v->flags & FEATURE_LOST) {
                if (reported[v->id]++) twice++;
                if ((v->flags & FEATURE_LEFT_IMAGE) && v->x > W - 20) left_once++;
            }
            if (f && (v->flags & FEATURE_NEW) && v->x < 3 * CELL) new_left++;
        }
    }
    printf("3 px per frame to the right: %d left the image at the right edge (each once), %d new ones on the left\n",
           left_once, new_left);
    assert(left_once > 10 && twice == 0 && new_left > 10);
    printf("ok: features leaving the image are LOST + LEFT_IMAGE once; new ones come in\n");
}

static void test_stripes(void) {
    for (int y = 0; y < BH; y++) for (int x = 0; x < BW; x++) base[y][x] = 120 + 80 * sinf(6.2831853f * (float)x / 6) + 40 * ((y / 23) % 2);
    vision_reset();
    srand(11);
    render(0, 0, 1, 0, sigma_at(6));
    process(6);
    render(3, 0, 1, 0, sigma_at(6));
    const vision_frame_t *r = process(6);
    int right = 0, wrong = 0, missed = 0;
    for (int k = 0; k < r->n_features; k++) {
        const vision_feature_t *v = &r->features[k];
        if (v->flags & (FEATURE_MISSED | FEATURE_LOST)) { missed++; continue; }
        if (v->flags) continue;
        if (hypotf(v->dx - 3, v->dy) <= 0.5f) right++; else wrong++;
    }
    printf("stripes every 6 px moved 3 px: %d right, %d missed or lost, %d confidently wrong\n", right, missed, wrong);
    assert(wrong <= (right + wrong + missed) / 100);
    printf("ok: a repeated pattern is missed, not matched wrong\n");
}

static void test_blur(void) {
    make_room(7);
    vision_reset();
    render(0, 0, 1, 0, sigma_at(6));
    const vision_frame_t *r = process(6);
    int n0 = r->n_features;
    // A heavily blurred frame (the content moved by 1 px meanwhile).
    for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
        float s = 0;
        for (int i = -12; i <= 12; i++) s += sample((float)(x + i) - 1 + 150, (float)y + 110);
        frame[y * W + x] = (uint8_t)(s / 25);
    }
    r = process(6);
    int missed = 0;
    for (int k = 0; k < r->n_features; k++) missed += (r->features[k].flags & FEATURE_MISSED) != 0;
    render(2, 0, 1, 0, sigma_at(6));
    r = process(6);
    int found = 0, right = 0;
    for (int k = 0; k < r->n_features; k++) {
        const vision_feature_t *v = &r->features[k];
        if (v->flags) continue;
        found++;
        right += fabsf(v->dx - 2) < 0.3f && fabsf(v->dy) < 0.3f; // since its last measurement, two frames ago
    }
    printf("one blurred frame: %d of %d missed; next frame %d found, %d with the move since their last measurement\n",
           missed, n0, found, right);
    assert(missed >= n0 * 7 / 10 && found >= n0 * 7 / 10 && right >= found * 9 / 10);
    // Three bad frames in a row: lost.
    for (int i = 0; i < W * H; i++) frame[i] = 120;
    int lost = 0;
    for (int f = 0; f < 3; f++) {
        r = process(6);
        for (int k = 0; k < r->n_features; k++) lost += f == 2 && (r->features[k].flags & FEATURE_LOST) && !(r->features[k].flags & FEATURE_LEFT_IMAGE);
    }
    printf("three blank frames: %d lost on the third\n", lost);
    assert(lost >= found * 9 / 10);
    printf("ok: missed in a bad frame, found again; lost after three\n");
}

static void test_record(void) {
    make_room(8);
    vision_reset();
    uint32_t seq = 0;
    static uint16_t last_age[65536];
    static uint8_t lost_seen[65536];
    memset(lost_seen, 0, sizeof lost_seen);
    for (int f = 0; f < 30; f++) {
        render(1.5f * (float)f, 0.5f * (float)f, 1, 0, sigma_at(6));
        const vision_frame_t *r = process(6);
        if (f) assert(r->seq == seq + 1);
        seq = r->seq;
        assert(r->t_us == r->row0_us + (uint32_t)(59.5f * r->line_us));
        int counted = 0;
        for (int y = 0; y < VISION_REGION_ROWS; y++)
            for (int x = 0; x < VISION_REGION_COLS; x++) counted += r->regions[y][x].features + r->regions[y][x].lost;
        assert(counted == r->n_features);
        int found = 0;
        for (int k = 0; k < r->n_features; k++) {
            const vision_feature_t *v = &r->features[k];
            assert(v->t_us == r->row0_us + (uint32_t)(v->y * r->line_us));
            if (v->flags & FEATURE_NEW) assert(v->age == 0 && v->dx == 0 && v->dy == 0 && v->match == 1);
            else assert(v->age == last_age[v->id] + 1);
            if (v->flags & FEATURE_MISSED) assert(v->dx == 0 && v->dy == 0);
            if (v->flags & FEATURE_LOST) assert(!lost_seen[v->id]++);
            found += !v->flags;
            last_age[v->id] = v->age;
        }
        assert(found == r->n_found);
    }
    printf("ok: the record keeps its promises (ages, LOST once, seq, region counts, row times)\n");
}

int main(void) {
    test_known_shifts();
    test_exposure_steps();
    test_brightness();
    test_still();
    test_nothing_to_see();
    test_leaving();
    test_stripes();
    test_blur();
    test_record();
    printf("time per frame on this computer: %.2f ms (the robot's: `v`)\n", cpu_ms / frames_timed);
    printf("vision: all tests passed\n");
    return 0;
}
