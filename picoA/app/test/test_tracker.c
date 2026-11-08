// Host test for picoA/app/tracker.c, at 15 and at 10 frames a second (the same
// times must hold at both): people walking through the VL53's view (±2° of jitter, a frame now and then without them) are followed
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

static uint32_t frame_us; // 66667 (15 Hz) or 100000 (10 Hz)
#define VIEW_DEG 22.5f

static uint32_t now_us = 0u - 30000000u; // 30 s before the clock wraps
static watch_obs_t obs[4];

static float jitter_deg(void) { return 2.0f * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f); }

// A blob of `cells` cells from `right_deg` to `left_deg`.
static watch_obs_t wide_blob(float right_deg, float left_deg, float range_m, int cells) {
    float b = (left_deg + right_deg) / 2 * RAD_PER_DEG;
    watch_obs_t o = {.where = {cosf(b), sinf(b), 0.0f}, .left_rad = left_deg * RAD_PER_DEG,
                      .right_rad = right_deg * RAD_PER_DEG, .range_m = range_m, .cells = cells};
    return o;
}
static watch_obs_t blob(float bearing_deg, float range_m, int cells) {
    return wide_blob(bearing_deg, bearing_deg, range_m, cells);
}

static track_event_t frame(int n) {
    now_us += frame_us;
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
static float last_seen_deg; // where the walker really was when last seen
static track_event_t walk(float from, float to, float speed, bool stops, float *rate_mid_deg) {
    leaving = 0;
    float b = from, step = (to > from ? speed : -speed) * (float)frame_us * 1e-6f;
    bool started = false;
    for (int k = 0; k < 400; k++) {
        bool arrived = to > from ? b >= to : b <= to;
        bool seen = fabsf(b) < VIEW_DEG && !(stops && arrived) && rand() % 8 != 0; // still: taken in, no blob
        if (seen) { obs[0] = blob(b + jitter_deg(), 1.0f, 6); last_seen_deg = b; }
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

static void run(uint32_t period_us) {
    frame_us = period_us;
    printf("--- a frame every %.1f ms\n", (double)frame_us / 1000);
    tracker_reset();
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
        // Halfway it has been in view 0.56 s: 8 frames at 15 Hz, 5-6 at 10, so a
        // noisier speed (seen 31-40 deg/s at 10 Hz, 35-41 at 15).
        assert(fabsf(rate - 40.0f) < (frame_us > 70000 ? 10.0f : 8.0f));
        assert(leaving == 1 && leaving_e == TRACK_LEAVING_LEFT && leaving_deg > 14.0f && leaving_rate_deg > 25.0f);
        assert(walk(30.0f, 5.0f, 20.0f, true, 0) == TRACK_STOPPED);
        assert(leaving == 0); // came in at the edge, going inward
        // Where they were last seen (±2° jitter): a few frames before 5° (a frame in 8 is missed).
        assert(fabsf(deg(tracker_target()->bearing_rad) - last_seen_deg) <= 2.01f && last_seen_deg < 12.0f);
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
    int gap = (int)(470000 / frame_us); // 7 frames at 15 Hz, 4 at 10
    for (int k = 0; k < gap; k++) assert(frame(0) == TRACK_NOTHING && tracker_target());
    obs[0] = blob(-10.0f, -1.0f, 4);
    assert(frame(1) == TRACK_NOTHING && fabsf(tracker_target()->range_m - 0.8f) < 0.01f);
    printf("ok: through a gap of %.2f s; a blob without a range keeps the last range\n", (double)(gap * frame_us) * 1e-6);

    tracker_reset();
    assert(!tracker_target());

    // The angular speed by time: 0 until 0.15 s of history; then a line through the
    // last 0.75 s only. Someone walking left at 20 deg/s for 1 s, unseen for 0.3 s,
    // then right at 20 deg/s: 0.8 s later only the walk to the right counts.
    float b = -10.0f;
    uint32_t first_us = now_us + frame_us;
    for (int k = 0; (float)k * (float)frame_us < 1e6f; k++, b += 20.0f * (float)frame_us * 1e-6f) {
        obs[0] = blob(b, 1.0f, 4);
        track_event_t te = frame(1);
        assert(te == (k == 0 ? TRACK_NEW : TRACK_NOTHING));
        bool measured = stamp_us(now_us, first_us) >= RATE_MIN_US;
        assert(measured ? fabsf(deg(tracker_target()->rate_radps) - 20.0f) < 0.5f : tracker_target()->rate_radps == 0.0f);
    }
    for (int k = 0; (float)k * (float)frame_us < 3e5f; k++) assert(frame(0) == TRACK_NOTHING);
    uint32_t turned_us = now_us;
    while (stamp_us(now_us, turned_us) < 800000) {
        b -= 20.0f * (float)frame_us * 1e-6f;
        obs[0] = blob(b, 1.0f, 4);
        assert(frame(1) == TRACK_NOTHING);
    }
    assert(fabsf(deg(tracker_target()->rate_radps) + 20.0f) < 0.5f);
    printf("ok: the angular speed: 0 until 0.15 s of history, then from the last 0.75 s only (after a gap and a turn back)\n");
    tracker_reset();

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
    // in the same direction is not them. (To 21°: at 10 Hz the speed is measured one
    // frame sooner, from the blob's jumps above, and puts them at 30.2°: 20° + 10° missed it.)
    obs[0] = wide_blob(17.0f, 21.0f, 0.5f, 4);
    assert(frame(1) == TRACK_LEAVING_LEFT);
    obs[0] = blob(20.0f, 2.8f, 1);
    track_event_t e = TRACK_NOTHING;
    for (int k = 0; k < 10 && e == TRACK_NOTHING; k++) e = frame(1);
    assert(e == TRACK_EXITED_LEFT && fabsf(tracker_target()->range_m - 0.5f) < 0.01f);
    printf("ok: a blob 2.3 m farther in the same direction is not the target: it left on the left\n");
}

int main(void) {
    run(66667);
    run(100000);
    printf("tracker: all tests passed at 15 and 10 Hz\n");
    return 0;
}
