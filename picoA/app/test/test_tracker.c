// Host test for picoA/app/tracker.c: people walking through the VL53's view (15
// frames a second, ±2° of jitter, a frame now and then without them) are followed
// with the right angular speed, reported about to leave at the edge (once) and
// leaving on the correct side; someone
// who stops inside the view is reported stopped; the target is kept over a bigger
// newcomer elsewhere and through short gaps; the first target is the biggest blob.
// Run from the repo root (run_tests.sh does):
//   cc -std=c11 -Wall -Wextra -Icommon -o build/test_tracker picoA/app/test/test_tracker.c -lm && build/test_tracker
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../tracker.c"

#define FRAME_US 66667 // 15 Hz
#define VIEW_DEG 22.5f

static uint32_t now_us = 1000000;
static motion_obs_t obs[4];

static float jitter_deg(void) { return 2.0f * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f); }

// A blob of `cells` cells from `right_deg` to `left_deg`.
static motion_obs_t wide_blob(float right_deg, float left_deg, float range_m, int cells) {
    float b = (left_deg + right_deg) / 2 * RAD_PER_DEG;
    motion_obs_t o = {.where = {cosf(b), sinf(b), 0.0f}, .left_rad = left_deg * RAD_PER_DEG,
                      .right_rad = right_deg * RAD_PER_DEG, .range_m = range_m, .cells = cells};
    return o;
}
static motion_obs_t blob(float bearing_deg, float range_m, int cells) {
    return wide_blob(bearing_deg, bearing_deg, range_m, cells);
}

static track_event_t frame(int n) {
    now_us += FRAME_US;
    return tracker_add(obs, n, now_us);
}

static float deg(float rad) { return rad * DEG_PER_RAD; }

// Someone walking from `from` to `to` degrees at `speed` deg/s, 1 m away, missing
// from 1 frame in 8; returns the first event other than "nothing" after they are
// out of view or have stopped (stops: they stand at `to`, then the VL53 takes them
// in after ~1 s and reports nothing). "Leaving" reports are counted in `leaving`,
// the last one's event in `leaving_e`, its direction and speed in `leaving_deg`, `leaving_rate_deg`.
static int leaving;
static track_event_t leaving_e;
static float leaving_deg, leaving_rate_deg;
static track_event_t walk(float from, float to, float speed, bool stops, float *rate_mid_deg) {
    leaving = 0;
    float b = from, step = (to > from ? speed : -speed) * FRAME_US * 1e-6f;
    bool started = false;
    for (int k = 0; k < 400; k++) {
        bool arrived = to > from ? b >= to : b <= to;
        bool seen = fabsf(b) < VIEW_DEG && !(stops && arrived) && rand() % 8 != 0; // still: taken in, no blob
        if (seen) obs[0] = blob(b + jitter_deg(), 1.0f, 6);
        track_event_t e = frame(seen ? 1 : 0);
        if (e == TRACK_NEW) { assert(!started); started = true; continue; }
        if (e == TRACK_LEAVING_LEFT || e == TRACK_LEAVING_RIGHT) {
            leaving++;
            leaving_e = e;
            leaving_deg = deg(tracker_target()->bearing_rad);
            leaving_rate_deg = deg(tracker_target()->rate_radps);
        } else if (e != TRACK_NOTHING) return e;
        if (rate_mid_deg && fabsf(b - (from + to) / 2) < fabsf(step)) *rate_mid_deg = deg(tracker_target()->rate_radps);
        if (!arrived) b += step;
    }
    return TRACK_NOTHING;
}

int main(void) {
    srand(1);
    for (int k = 0; k < 30; k++) assert(frame(0) == TRACK_NOTHING && !tracker_target());
    printf("ok: nothing moving: no target\n");

    for (int seed = 1; seed <= 20; seed++) {
        srand((unsigned)seed);
        float rate;
        assert(walk(30.0f, -30.0f, 20.0f, false, &rate) == TRACK_EXITED_RIGHT);
        assert(fabsf(rate + 20.0f) < 5.0f);
        assert(leaving == 1 && leaving_e == TRACK_LEAVING_RIGHT && leaving_deg < -14.0f && leaving_rate_deg < -12.0f);
        assert(walk(-30.0f, 30.0f, 40.0f, false, &rate) == TRACK_EXITED_LEFT);
        assert(fabsf(rate - 40.0f) < 8.0f);
        assert(leaving == 1 && leaving_e == TRACK_LEAVING_LEFT && leaving_deg > 14.0f && leaving_rate_deg > 25.0f);
        assert(walk(30.0f, 5.0f, 20.0f, true, 0) == TRACK_STOPPED);
        assert(leaving == 0); // came in at the edge, going inward
        assert(fabsf(deg(tracker_target()->bearing_rad) - 5.0f) < 4.0f);
        assert(fabsf(tracker_target()->range_m - 1.0f) < 0.01f);
        assert(frame(0) == TRACK_NOTHING && !tracker_target());
    }
    printf("ok: walking right and left (20, 40 deg/s): angular speed, leaving once at the edge, exit side;\n"
           "    stopping inside: stopped, not leaving (20 seeds)\n");

    // The first target is the biggest blob; it is kept over a bigger newcomer elsewhere.
    obs[0] = blob(-10.0f, 0.8f, 9);
    obs[1] = blob(12.0f, 1.5f, 3);
    assert(frame(2) == TRACK_NEW && fabsf(deg(tracker_target()->bearing_rad) + 10.0f) < 0.1f);
    for (int k = 0; k < 10; k++) {
        obs[0] = blob(15.0f, 0.5f, 12); // bigger, elsewhere
        obs[1] = blob(-10.0f, 0.8f, 9);
        assert(frame(2) == TRACK_NOTHING);
        assert(fabsf(deg(tracker_target()->bearing_rad) + 10.0f) < 0.1f);
    }
    printf("ok: the first target is the biggest blob, and it is kept over a bigger newcomer\n");

    // Gaps shorter than 0.5 s don't lose it; a blob without a range keeps the last one.
    for (int k = 0; k < 7; k++) assert(frame(0) == TRACK_NOTHING && tracker_target());
    obs[0] = blob(-10.0f, -1.0f, 4);
    assert(frame(1) == TRACK_NOTHING && fabsf(tracker_target()->range_m - 0.8f) < 0.01f);
    printf("ok: through a gap of 0.47 s; a blob without a range keeps the last range\n");

    tracker_reset();
    assert(!tracker_target());

    // As on the robot: someone close comes in at the right edge, then is a big blob
    // reaching from -15 to +20 deg (centre +3) plus a 2-zone piece at -17 deg.
    obs[0] = blob(-20.0f, 0.53f, 1);
    assert(frame(1) == TRACK_NEW);
    obs[0] = wide_blob(-15.0f, 20.0f, 0.45f, 24);
    obs[1] = blob(-17.0f, 0.55f, 2);
    assert(frame(2) == TRACK_NOTHING && tracker_target()->last_seen_us == now_us);
    assert(fabsf(deg(tracker_target()->bearing_rad) - 2.5f) < 0.1f);
    obs[0] = wide_blob(5.0f, 22.0f, 0.5f, 12);
    assert(frame(1) == TRACK_NOTHING && deg(tracker_target()->bearing_rad) > 10.0f);
    printf("ok: someone close: the big blob reaching where they are, not a small piece nearer its centre\n");

    // ...leaving at the left edge (moving outward there: about to leave), a far zone
    // in the same direction is not them.
    obs[0] = wide_blob(17.0f, 20.0f, 0.5f, 4);
    assert(frame(1) == TRACK_LEAVING_LEFT);
    obs[0] = blob(20.0f, 2.8f, 1);
    track_event_t e = TRACK_NOTHING;
    for (int k = 0; k < 10 && e == TRACK_NOTHING; k++) e = frame(1);
    assert(e == TRACK_EXITED_LEFT && fabsf(tracker_target()->range_m - 0.5f) < 0.01f);
    printf("ok: a blob 2.3 m farther in the same direction is not the target: it left on the left\n");
    printf("tracker: all tests passed\n");
    return 0;
}
