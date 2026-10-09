// Host test for picoA/app/tof_motion.c: a simulated room as on the robot (range
// noise, unsure frames, a floor zone reading its reflection farther away, far zones
// jumping farther or to nothing, a far zone that is sometimes sure and sometimes
// empty, floor zones reading the wall behind, a door frame's edge now and then) must show no movement however long it is watched; a person stepping in on
// the left, a thin pole in one zone, a hand waving for 3 s and a target where
// nothing was must show; a box
// put down is still after ~1 s, and taking it away is not movement. Turning in a
// 360° room (walls at 1-3 m, a door edge at 1 m with the wall 3 m behind it), after
// a start-up turn: no movement at 0.5, 1 and 1.5 rad/s, also with the heading 1.5°
// off; someone walking past while it turns is found, and when it stops, at once.
// Run from the repo root (run_tests.sh does):
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_tof_motion picoA/app/test/test_tof_motion.c -lm && build/test_tof_motion
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../rangefinder.c"
#include "../change_grid.c"
#include "../tof_motion.c"

static tof_view_t view; // the robot: still at the origin unless a test turns it

static range_frame_t f = {.t_us = 0u - 30000000u}; // 30 s before the clock wraps
static motion_obs_t obs[8];

static float jitter(float mm) { return mm * 0.015f * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f); }
static int chance(int one_in) { return rand() % one_in == 0; }

static void set(int row, int col, float mm) {
    f.range_mm[row * RANGEFINDER_COLS + col] = (uint16_t)(mm + jitter(mm));
}

// The room: the top row sees nothing in range except a far door frame (col 7, sure
// or empty), a wall at 2 m, the floor in the lower rows.
static void room(void) {
    f.t_us += 66667;
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        int row = i / RANGEFINDER_COLS;
        float mm = row == 7 ? 210.0f : row == 6 ? 290.0f : row >= 4 ? 480.0f : 2000.0f;
        if (row == 0) f.range_mm[i] = RANGE_NO_TARGET;
        else f.range_mm[i] = (uint16_t)(mm + jitter(mm));
        if (chance(30)) f.range_mm[i] = RANGE_INVALID; // the odd unsure frame anywhere
    }
    if (chance(2)) f.range_mm[7] = (uint16_t)(3000.0f + jitter(3000.0f));
    // As on the robot: 6th-row zones grazing the floor read the wall behind half the
    // time, an upper zone at a door frame's edge gives the frame now and then.
    if (chance(2)) f.range_mm[5 * RANGEFINDER_COLS + 3] = (uint16_t)(1620.0f + jitter(1620.0f));
    if (chance(3)) f.range_mm[5 * RANGEFINDER_COLS + 5] = RANGE_NO_TARGET;
    if (chance(20)) f.range_mm[2 * RANGEFINDER_COLS + 6] = (uint16_t)(1740.0f + jitter(1740.0f));
    else if (f.range_mm[2 * RANGEFINDER_COLS + 6] != RANGE_INVALID) f.range_mm[2 * RANGEFINDER_COLS + 6] = 2500;
    // For 1-2 frames every ~10 s: the floor zone's reflection, or a far zone's farther surface or nothing.
    static int glitch_frames, glitch_zone;
    static uint16_t glitch_mm;
    if (glitch_frames == 0 && chance(150)) {
        glitch_frames = 1 + rand() % 2;
        bool floor = chance(2);
        glitch_zone = floor ? 5 * RANGEFINDER_COLS : RANGEFINDER_COLS + rand() % 24;
        glitch_mm = floor ? 580 : chance(2) ? RANGE_NO_TARGET : (uint16_t)(2300 + rand() % 1000);
    }
    if (glitch_frames > 0) { glitch_frames--; f.range_mm[glitch_zone] = glitch_mm; }
}

static int frame(void) { return tof_motion_add(&f, &view, obs, 8); }

// Watches the room with `scene` in it for n frames; returns how many frames showed movement.
static int watch(int n, void (*scene)(void)) {
    int moving = 0;
    for (int k = 0; k < n; k++) {
        room();
        if (scene) scene();
        moving += frame() > 0;
    }
    return moving;
}

static void person(void) { // columns 1-2 (left of centre), rows 1-3 (above the floor rows), at 80 cm
    for (int row = 1; row <= 3; row++)
        for (int col = 1; col <= 2; col++) set(row, col, 800.0f);
}
static void pole(void) { set(2, 4, 1200.0f); }      // one zone
static int wave_frame;
static void hand(void) { // a hand at 40 cm waving across columns 2-5, one column every 2 frames
    int col = 2 + (wave_frame++ / 2) % 4;
    for (int row = 2; row <= 3; row++) { set(row, col, 400.0f); set(row, col + 1, 400.0f); }
}
static void appeared(void) { set(0, 4, 1500.0f); set(0, 5, 1500.0f); } // where nothing was in range
static void unsure_patch(void) {
    for (int row = 2; row <= 4; row++)
        for (int col = 4; col <= 6; col++) f.range_mm[row * RANGEFINDER_COLS + col] = RANGE_INVALID;
}

static void quiet_again(void) {
    watch(CHANGE_GRID_WINDOW, 0);
    assert(watch(30, 0) == 0);
}

// --- Turning in a 360° room ---
// The walls around the robot by world bearing: 1-3 m, and a door edge (a cupboard
// 1 m away from 40° to 70°, the wall 3 m behind it on both sides).
static float wall_mm(float b) {
    float deg = wrap_pi(b) * DEG_PER_RAD;
    if (deg > 40.0f && deg < 70.0f) return 1000.0f;
    if (deg > 25.0f && deg < 85.0f) return 3000.0f;
    return 2000.0f + 1000.0f * sinf(2.0f * b);
}

static float walker_rad = 99.0f; // world bearing of someone at 1 m (99: nobody)

// One frame turning from yaw0 to yaw1 (world), the robot believing it is `err` off:
// each upper zone reads the nearest surface its cone swept; the floor rows as room().
static int turning_frame(float yaw0, float yaw1, float err) {
    room();
    for (int i = 0; i < 4 * RANGEFINDER_COLS; i++) {
        float nearest = 1e9f;
        for (int s = 0; s <= 8; s++)
            for (float u = -HALF_ZONE_RAD; u <= HALF_ZONE_RAD; u += 0.5f * RAD_PER_DEG) {
                float b = yaw0 + (yaw1 - yaw0) * (float)s / 8 + left_rad[i] + u;
                nearest = fminf(nearest, wall_mm(b));
                if (fabsf(wrap_pi(b - walker_rad)) < 4 * RAD_PER_DEG) nearest = fminf(nearest, 1000.0f);
            }
        if (f.range_mm[i] != RANGE_INVALID) f.range_mm[i] = (uint16_t)(nearest + jitter(nearest));
    }
    view = (tof_view_t){.yaw_start_rad = yaw0 + err, .yaw_end_rad = yaw1 + err};
    return frame();
}

// Turns `turn_rad` at `radps` from yaw; returns the frames that showed movement.
static int turn(float *yaw, float turn_rad, float radps, float err) {
    float step = radps / RANGEFINDER_HZ * (turn_rad < 0 ? -1.0f : 1.0f);
    int moving = 0, n = (int)(fabsf(turn_rad) / fabsf(step));
    for (int k = 0; k < n; k++, *yaw += step) moving += turning_frame(*yaw, *yaw + step, err) > 0;
    return moving;
}

static void turning_tests(void) {
    tof_motion_restart();
    float yaw = 0;
    assert(turning_frame(0, 0, 0) == 0 && turning_frame(0, 0, 0) == 0);
    assert(turn(&yaw, 390 * RAD_PER_DEG, 0.5f, 0) == 0); // the start-up turn
    for (int k = 0; k < 3; k++) {
        float radps = k == 0 ? 0.5f : k == 1 ? 1.0f : 1.5f;
        int moving = turn(&yaw, 5 * 2 * PI_F, radps, 0) + turn(&yaw, -5 * 2 * PI_F, radps, 0);
        printf("    %.1f rad/s, 5 turns each way: %d frames with movement\n", (double)radps, moving);
        assert(moving == 0);
    }
    printf("ok: turning at 0.5, 1 and 1.5 rad/s past a door edge: no movement\n");
    // The heading off by up to 1.5° (gyro scale, the frame's time): still quiet.
    for (float err_deg = 0.5f; err_deg <= 3.01f; err_deg += 0.5f) {
        int moving = turn(&yaw, 2 * PI_F, 1.0f, err_deg * RAD_PER_DEG) + turn(&yaw, -2 * PI_F, 1.0f, -err_deg * RAD_PER_DEG);
        printf("    heading %.1f deg off: %d frames with movement\n", (double)err_deg, moving);
        if (err_deg <= 1.5f) assert(moving == 0);
    }
    watch(2, 0); // still again at the true heading
    view = (tof_view_t){.yaw_start_rad = yaw, .yaw_end_rad = yaw};
    printf("ok: quiet with the heading up to 1.5 deg off\n");

    // Someone standing 1 m away (world bearing -135°: the wall 3 m behind) while the
    // robot turns towards them at 1 rad/s: found within 3 frames of being in view.
    yaw = 0;
    turn(&yaw, -2 * PI_F, 1.0f, 0); // back where it started, the background all known
    walker_rad = -135 * RAD_PER_DEG;
    int in_view = -1, found = -1;
    for (int k = 0; k < 40 && found < 0; k++, yaw -= 1.0f / RANGEFINDER_HZ) {
        int n = turning_frame(yaw, yaw - 1.0f / RANGEFINDER_HZ, 0);
        if (in_view < 0 && fabsf(wrap_pi(walker_rad - yaw)) < (22.5f + 4) * RAD_PER_DEG) in_view = k; // its edge
        if (n > 0 && found < 0) found = k;
    }
    printf("    someone at 1 m turning towards them: in view at frame %d, found at %d\n", in_view, found);
    assert(found >= in_view && found - in_view <= 3);
    printf("ok: someone in the room found while turning, within 3 frames\n");
    walker_rad = 99.0f;
    turn(&yaw, 2 * PI_F, 1.0f, 0); // a full turn without them: the background there is the wall again

    // Stopped after a turn with someone in view (the wall 3 m behind them): found at
    // once (no learning after a stop).
    walker_rad = -135 * RAD_PER_DEG;
    int first = -1;
    for (int k = 0; k < 10 && first < 0; k++) if (turning_frame(yaw, yaw, 0) > 0) first = k;
    printf("    stopped with someone in view: found in still frame %d\n", first);
    assert(first >= 0 && first <= 2);
    walker_rad = 99.0f;
    printf("ok: stopping with someone in view: found at once\n");

    // Someone who has stopped 15 deg to the right while the robot turns towards them
    // (0.5 rad/s): movement in every turning frame (only standing still makes a
    // nearer thing the view); then still, the view again after ~1 s.
    walker_rad = -135 * RAD_PER_DEG;
    yaw = walker_rad + 15 * RAD_PER_DEG;
    watch(2, 0);
    view = (tof_view_t){.yaw_start_rad = yaw, .yaw_end_rad = yaw};
    for (int k = 0; k < 3; k++) turning_frame(yaw, yaw, 0); // seen, standing
    int turning_moving = 0, turning_frames = 0;
    for (; yaw > walker_rad; yaw -= 0.5f / RANGEFINDER_HZ, turning_frames++)
        turning_moving += turning_frame(yaw, yaw - 0.5f / RANGEFINDER_HZ, 0) > 0;
    int still_moving = 0, quiet_from = -1;
    for (int k = 0; k < 3 * RANGEFINDER_HZ; k++) {
        bool m = turning_frame(yaw, yaw, 0) > 0;
        if (m) still_moving = k + 1;
        else if (quiet_from < 0 && k > 0) quiet_from = k;
    }
    printf("    someone stopped, the robot turning towards them: movement in %d of %d turning frames; "
           "still, the view again after %.1f s\n", turning_moving, turning_frames, (double)still_moving / RANGEFINDER_HZ);
    assert(turning_moving == turning_frames);
    assert(still_moving >= STEADY_FRAMES - 2 && still_moving <= STEADY_FRAMES + CHANGE_GRID_WINDOW + 2);
    walker_rad = 99.0f;
    printf("ok: something that stopped stays movement while the robot turns to it, then becomes the view\n");

    // Driving 20 cm: the bins are forgotten, nothing is movement, and still they fill in 2 frames.
    view = (tof_view_t){.yaw_start_rad = yaw, .yaw_end_rad = yaw, .x_m = 0.2f};
    for (int k = 0; k < 30; k++) { room(); assert(frame() == 0); }
    assert(tof_motion_ready());
    printf("ok: after driving 20 cm the background is forgotten and refilled, no movement\n");
}

int main(void) {
    srand(1);
    tof_motion_restart();
    assert(watch(2, 0) == 0 && tof_motion_ready()); // the background is known after 2 frames
    assert(watch(15 * 300, 0) == 0);
    printf("ok: a still room with noise, unsure frames, reflections and farther surfaces: no movement in 5 min\n");

    // Someone standing there when the robot starts watching is the view.
    tof_motion_restart();
    assert(watch(2, person) == 0);
    assert(watch(30, person) == 0);
    printf("ok: what stands still when watching starts is part of the view\n");
    // ...and when they walk off, the wall behind is farther: not movement.
    assert(watch(2 * FARTHER_FRAMES, 0) == 0); // and after 10 s (of sure readings) the wall is the background
    printf("ok: walking off (farther) is not movement\n");

    // A person steps in on the left: found in the 2nd frame, left of centre, ~80 cm.
    room(); person(); assert(frame() == 0);
    room(); person(); assert(frame() > 0);
    assert(obs[0].cells >= 5); // a zone may be in its odd unsure frame
    assert(obs[0].where[1] > 0.1f && obs[0].where[0] > 0.9f); // left, ahead
    assert(fabsf(obs[0].range_m - 0.8f) < 0.05f);
    assert(fabsf(obs[0].point[0] - 0.8f) < 0.1f && obs[0].point[1] > 0.1f);
    printf("ok: a person steps in on the left: %d zones, where (%.2f %.2f %.2f), %.2f m, in 2 frames\n",
           obs[0].cells, (double)obs[0].where[0], (double)obs[0].where[1], (double)obs[0].where[2],
           (double)obs[0].range_m);
    quiet_again();

    room(); pole(); assert(frame() == 0);
    room(); pole(); assert(frame() == 0);
    room(); pole(); assert(frame() == 1 && obs[0].cells == 1);
    quiet_again();
    printf("ok: a thin pole in one zone: found in 3 frames\n");

    // A hand waving for 3 s: it keeps changing, so it counts the whole time.
    assert(watch(3 * RANGEFINDER_HZ, hand) >= 3 * RANGEFINDER_HZ - 2);
    quiet_again();
    printf("ok: a hand waving for 3 s is movement the whole time\n");

    assert(watch(3, appeared) > 0);
    quiet_again();
    printf("ok: a target where nothing was in range\n");

    assert(watch(15 * 10, unsure_patch) == 0);
    printf("ok: zones turning unsure tell nothing\n");

    // A box put down and left: movement until it has been still for 1 s, then it is the view.
    int moving = 0, quiet_from = -1;
    for (int k = 0; k < 15 * 10; k++) {
        room(); person();
        bool m = frame() > 0;
        moving += m;
        if (!m && quiet_from < 0 && moving) quiet_from = k;
        if (quiet_from >= 0) assert(!m);
    }
    assert(quiet_from >= STEADY_FRAMES && quiet_from <= STEADY_FRAMES + CHANGE_GRID_WINDOW + 2);
    printf("ok: a box put down is still after %.1f s\n", (double)quiet_from / RANGEFINDER_HZ);
    // Taken away: the wall behind is farther, not movement; after 10 s it is the
    // background again, so someone stepping there is found.
    assert(watch(2 * FARTHER_FRAMES, 0) == 0);
    room(); person(); frame();
    room(); person(); assert(frame() > 0);
    printf("ok: taking the box away is not movement; the wall is the background again after 10 s\n");

    turning_tests();
    printf("tof_motion: all tests passed\n");
    return 0;
}
