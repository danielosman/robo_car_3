// Host test for picoA/app/watch_tof.c: a simulated room as on the robot (range
// noise, unsure frames, a floor zone reading its reflection farther away, far zones
// jumping farther or to nothing, a far zone that is sometimes sure and sometimes
// empty, floor zones reading the wall behind, a door frame's edge now and then) must show no movement however long it is watched; a person stepping in on
// the left, a thin pole in one zone, a hand waving for 3 s and a target where
// nothing was must show; a box
// put down is still after ~1 s, and taking it away is not movement. Turning in a
// 360° room (walls at 1-3 m, a door edge at 1 m with the wall 3 m behind it), after
// a start-up turn: no movement at 0.5, 1 and 1.5 rad/s, also with the heading 1.5°
// off; someone walking past while it turns is found, and when it stops, at once.
// Every test runs at 15 and at 10 Hz: the times (1 s, 10 s, 3 s) hold at both,
// each frame carrying its own period (no global rate). Each frame reaches watch_tof
// as on the robot: one target per zone, packed and unpacked for the watch use (whole
// cm; the walls at 3 m read "beyond 2.54 m").
// Run from the repo root (run_tests.sh does):
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_watch_tof picoA/app/test/test_watch_tof.c -lm && build/test_watch_tof
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../tof_rays.c"
#include "../tof_format.c"
#include "../change_grid.c"
#include "../watch_tof.c"
#include "robot_mounting.h"

static tof_view_t view; // the robot: still at the origin unless a test turns it
static int hz;          // the rate the tests run at (10 and 15): times, not frame counts, must hold at both

// The sensor's frame as the scenes write it: per zone mm, RANGE_NO_TARGET for nothing
// in range, RANGE_INVALID for an unsure reading.
#define RANGE_NO_TARGET 0
#define RANGE_INVALID   0xFFFF
static struct { uint32_t t_us; uint16_t range_mm[TOF_ZONES]; } f = {.t_us = 0u - 30000000u}; // 30 s before the clock wraps
static uint32_t period_us(void) { return 1000000u / (uint32_t)hz; }
static watch_obs_t obs[8];

static float jitter(float mm) { return mm * 0.015f * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f); }
static int chance(int one_in) { return rand() % one_in == 0; }

static void set(int row, int col, float mm) { // indexes: row 1 is 0
    f.range_mm[row * TOF_COLS + col] = (uint16_t)(mm + jitter(mm));
}

// The room: the top row sees nothing in range except a far door frame (column 8, sure
// or empty), a wall at 2 m, the floor in the lower rows.
static void room(void) {
    f.t_us += 1000000u / (uint32_t)hz;
    for (int i = 0; i < TOF_ZONES; i++) {
        int row = i / TOF_COLS;
        float mm = row == 7 ? 210.0f : row == 6 ? 290.0f : row >= 4 ? 480.0f : 2000.0f;
        if (row == 0) f.range_mm[i] = RANGE_NO_TARGET;
        else f.range_mm[i] = (uint16_t)(mm + jitter(mm));
        if (chance(30)) f.range_mm[i] = RANGE_INVALID; // the odd unsure frame anywhere
    }
    // The far door frame, sure half the time; sure in the first frame after a restart
    // (empty in both of the first two, a later reading would be a target where
    // nothing was: movement, rightly; at 10 Hz the random draws hit that).
    if (chance(2) || frames == 0) f.range_mm[7] = (uint16_t)(3000.0f + jitter(3000.0f));
    // As on the robot: 6th-row zones grazing the floor read the wall behind half the
    // time, an upper zone at a door frame's edge gives the frame now and then.
    if (chance(2)) f.range_mm[5 * TOF_COLS + 3] = (uint16_t)(1620.0f + jitter(1620.0f));
    if (chance(3)) f.range_mm[5 * TOF_COLS + 5] = RANGE_NO_TARGET;
    if (chance(20)) f.range_mm[2 * TOF_COLS + 6] = (uint16_t)(1740.0f + jitter(1740.0f));
    else if (f.range_mm[2 * TOF_COLS + 6] != RANGE_INVALID) f.range_mm[2 * TOF_COLS + 6] = 2500;
    // For 1-2 frames every ~10 s: the floor zone's reflection, or a far zone's farther surface or nothing.
    static int glitch_frames, glitch_zone;
    static uint16_t glitch_mm;
    if (glitch_frames == 0 && chance(150)) {
        glitch_frames = 1 + rand() % 2;
        bool floor = chance(2);
        glitch_zone = floor ? 5 * TOF_COLS : TOF_COLS + rand() % 24;
        glitch_mm = floor ? 580 : chance(2) ? RANGE_NO_TARGET : (uint16_t)(2300 + rand() % 1000);
    }
    if (glitch_frames > 0) { glitch_frames--; f.range_mm[glitch_zone] = glitch_mm; }
}

// The frame as a tof_frame at the rate, packed and unpacked for the watch use.
static int frame(void) {
    posed_frame_t p = {.frame = {.t_us = f.t_us, .period_us = period_us(), .beyond_mm = TOF_EXACT}};
    for (int i = 0; i < TOF_ZONES; i++) {
        uint16_t mm = f.range_mm[i];
        p.frame.zone[i] = mm == RANGE_NO_TARGET ? (tof_zone_t){.kind = TOF_NOTHING}
                        : mm == RANGE_INVALID ? (tof_zone_t){.distance_mm = 300, .kind = TOF_UNSURE, .targets = 1}
                        : (tof_zone_t){.distance_mm = mm, .kind = TOF_SURE, .targets = 1};
    }
    static uint8_t bytes[TOF_FORMAT_MAX];
    static posed_frame_t used;
    uint32_t n_bytes = tof_pack(TOF_USE_WATCH, &p, bytes);
    assert(tof_unpack(tof_format_of(TOF_USE_WATCH), bytes, n_bytes, &used));
    return watch_tof_add(&used.frame, &view, obs, 8);
}

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

static void person(void) { // columns 2-3 (left of centre), rows 2-4 (above the floor rows), at 80 cm
    for (int row = 1; row <= 3; row++)
        for (int col = 1; col <= 2; col++) set(row, col, 800.0f);
}
static void pole(void) { set(2, 4, 1200.0f); }      // one zone
static int wave_frame;
// A hand at 40 cm waving across columns 2-5, one column every 2 frames (133 ms at 15 Hz,
// 200 ms at 10). In frames, not time: at other speeds (e.g. a column every 100 ms at
// 15 Hz, 133 ms at 10) the edge zones read it in single frames, the "nearer now and
// then" rule makes it the background and most frames show nothing (doc/TODO.md).
static void hand(void) {
    int col = 2 + (wave_frame++ / 2) % 4;
    for (int row = 2; row <= 3; row++) { set(row, col, 400.0f); set(row, col + 1, 400.0f); }
}
static void appeared(void) { set(0, 4, 1500.0f); set(0, 5, 1500.0f); } // where nothing was in range
static void unsure_patch(void) {
    for (int row = 2; row <= 4; row++)
        for (int col = 4; col <= 6; col++) f.range_mm[row * TOF_COLS + col] = RANGE_INVALID;
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
    for (int i = 0; i < 4 * TOF_COLS; i++) {
        float nearest = 1e9f;
        for (int s = 0; s <= 8; s++)
            for (float u = -half_zone_rad; u <= half_zone_rad; u += 0.5f * RAD_PER_DEG) {
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
    float step = radps / hz * (turn_rad < 0 ? -1.0f : 1.0f);
    int moving = 0, n = (int)(fabsf(turn_rad) / fabsf(step));
    for (int k = 0; k < n; k++, *yaw += step) moving += turning_frame(*yaw, *yaw + step, err) > 0;
    return moving;
}

static void turning_tests(void) {
    watch_tof_restart();
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
    // The frame's time 10 ms off (the turn of 10 ms on top, the way it turns), at 1.5 rad/s: quiet.
    int late = turn(&yaw, 2 * PI_F, 1.5f, 1.5f * 0.01f) + turn(&yaw, -2 * PI_F, 1.5f, -1.5f * 0.01f);
    printf("    the frame's time 10 ms off at 1.5 rad/s: %d frames with movement\n", late);
    assert(late == 0);
    watch(2, 0); // still again at the true heading
    view = (tof_view_t){.yaw_start_rad = yaw, .yaw_end_rad = yaw};
    printf("ok: quiet with the heading up to 1.5 deg off\n");

    // Someone standing 1 m away (world bearing -135°: the wall 3 m behind) while the
    // robot turns towards them at 1 rad/s: found within 3 frames of being in view.
    yaw = 0;
    turn(&yaw, -2 * PI_F, 1.0f, 0); // back where it started, the background all known
    walker_rad = -135 * RAD_PER_DEG;
    int in_view = -1, found = -1;
    for (int k = 0; k < 40 && found < 0; k++, yaw -= 1.0f / hz) {
        int n = turning_frame(yaw, yaw - 1.0f / hz, 0);
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
    for (; yaw > walker_rad; yaw -= 0.5f / hz, turning_frames++)
        turning_moving += turning_frame(yaw, yaw - 0.5f / hz, 0) > 0;
    int still_moving = 0, quiet_from = -1;
    for (int k = 0; k < 3 * hz; k++) {
        bool m = turning_frame(yaw, yaw, 0) > 0;
        if (m) still_moving = k + 1;
        else if (quiet_from < 0 && k > 0) quiet_from = k;
    }
    printf("    someone stopped, the robot turning towards them: movement in %d of %d turning frames; "
           "still, the view again after %.1f s\n", turning_moving, turning_frames, (double)still_moving / hz);
    assert(turning_moving == turning_frames);
    assert(still_moving >= hz - 2 && still_moving <= hz + CHANGE_GRID_WINDOW + 2);
    walker_rad = 99.0f;
    printf("ok: something that stopped stays movement while the robot turns to it, then becomes the view\n");

    // Driving 20 cm: the bins are forgotten, nothing is movement, and still they fill in 2 frames.
    view = (tof_view_t){.yaw_start_rad = yaw, .yaw_end_rad = yaw, .x_m = 0.2f};
    for (int k = 0; k < 30; k++) { room(); assert(frame() == 0); }
    assert(watch_tof_ready());
    printf("ok: after driving 20 cm the background is forgotten and refilled, no movement\n");
}

// A box put down and left: movement until it has been still for 1 s, then it is
// the view. Taken away: the wall behind is farther, not movement; the box's
// distance stays the background for 10 s, then the wall is, so someone stepping
// there is found.
static void box_test(void) {
    int moving = 0, quiet_from = -1;
    for (int k = 0; k < 10 * hz; k++) {
        room(); person();
        bool m = frame() > 0;
        moving += m;
        if (!m && quiet_from < 0 && moving) quiet_from = k;
        if (quiet_from >= 0) assert(!m);
    }
    assert(quiet_from >= hz && quiet_from <= hz + CHANGE_GRID_WINDOW + 2); // 1 s (hz frames), plus the window
    printf("ok: a box put down is still after %.1f s\n", (double)quiet_from / hz);
    const int zone = 2 * TOF_COLS + 1; // in the box
    assert(watch(9 * hz, 0) == 0 && watch_tof_background_mm(zone) < 1000);  // 9 s: still the box's
    assert(watch(2 * hz, 0) == 0 && watch_tof_background_mm(zone) > 1500);  // 11 s: the wall's
    room(); person(); frame();
    room(); person(); assert(frame() > 0);
    quiet_again();
    printf("ok: taking the box away is not movement; the wall is the background again after 10 s\n");
}

// Turning means faster than 15 deg/s during a frame, at any rate: then the floor
// rows predict nothing.
static bool turning_at(float degps) {
    watch_tof_restart();
    view = (tof_view_t){0};
    watch(3, 0);
    room();
    view.yaw_end_rad = degps * RAD_PER_DEG * (float)period_us() * 1e-6f;
    frame();
    bool turning = predicted[7 * TOF_COLS + 3] < 0;
    view = (tof_view_t){0};
    watch_tof_restart();
    return turning;
}

static void run(int rate) {
    hz = rate;
    wave_frame = 0;
    printf("--- at %d Hz\n", hz);
    srand(1);
    watch_tof_restart();
    assert(watch(2, 0) == 0 && watch_tof_ready()); // the background is known after 2 frames
    assert(watch(300 * hz, 0) == 0);
    printf("ok: a still room with noise, unsure frames, reflections and farther surfaces: no movement in 5 min\n");
    assert(!turning_at(14) && turning_at(16));
    printf("ok: 14 deg/s is not turning, 16 deg/s is\n");

    // Someone standing there when the robot starts watching is the view.
    watch_tof_restart();
    assert(watch(2, person) == 0);
    assert(watch(30, person) == 0);
    printf("ok: what stands still when watching starts is part of the view\n");
    // ...and when they walk off, the wall behind is farther: not movement.
    assert(watch(20 * hz, 0) == 0); // and after 10 s (of sure readings) the wall is the background
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
    assert(watch(3 * hz, hand) >= 3 * hz - 2);
    quiet_again();
    printf("ok: a hand waving for 3 s is movement the whole time\n");

    assert(watch(3, appeared) > 0);
    quiet_again();
    printf("ok: a target where nothing was in range\n");

    assert(watch(10 * hz, unsure_patch) == 0);
    printf("ok: zones turning unsure tell nothing\n");

    box_test();
    turning_tests();
}

// The rate changed while watching (15 to 10 Hz, no restart): no movement, and 1 s
// and 10 s are still 1 s and 10 s.
static void rate_change(void) {
    printf("--- 15 Hz, then 10 Hz while watching\n");
    hz = 15;
    srand(2);
    view = (tof_view_t){0};
    watch_tof_restart();
    assert(watch(10 * hz, 0) == 0);
    hz = 10;
    assert(watch(60 * hz, 0) == 0);
    printf("ok: a minute after the change: no movement\n");
    box_test();
}

// The watch use's 8-bit distances end at 2.54 m (accepted, 7 Nov): with the wall at
// 3.5 m, someone at 2.7 m reads "beyond" like the wall and isn't seen; at 2.3 m they
// are (until they have stood still 1 s). A floor zone reading "beyond" (a
// reflection) is ignored.
static float far_person_mm;
static void far_wall(void) { // rows 2-4 see a wall 3.5 m away
    for (int row = 1; row <= 3; row++)
        for (int col = 0; col < TOF_COLS; col++) f.range_mm[row * TOF_COLS + col] = 3500;
}
static void far_person(void) {
    far_wall();
    for (int row = 1; row <= 3; row++)
        for (int col = 1; col <= 2; col++) set(row, col, far_person_mm);
}
static void floor_beyond(void) { if (chance(2)) f.range_mm[6 * TOF_COLS + 4] = 3000; } // row 7: its floor's reflection

static void far_test(void) {
    printf("--- beyond 2.54 m, at 10 Hz\n");
    hz = 10;
    srand(5);
    view = (tof_view_t){0};
    watch_tof_restart();
    assert(watch(30, far_wall) == 0);
    far_person_mm = 2700;
    int at_2700 = watch(20, far_person);
    assert(watch(30, far_wall) == 0);
    far_person_mm = 2300;
    int at_2300 = watch(20, far_person);
    printf("someone walking in at 2.7 m (the wall at 3.5 m): moving in %d of 20 frames; at 2.3 m: %d of 20\n",
           at_2700, at_2300);
    assert(at_2700 == 0 && at_2300 >= 8); // seen until it has stood still 1 s, then it is the view
    assert(watch(10 * hz, far_wall) == 0); // gone again: farther is never movement
    watch_tof_restart();
    assert(watch(30 * hz, floor_beyond) == 0);
    const int zone = 6 * TOF_COLS + 4;
    assert(watch_tof_background_mm(zone) > 250 && watch_tof_background_mm(zone) < 350);
    printf("ok: movement beyond 2.54 m isn't seen (2.7 m), nearer is (2.3 m); a floor zone's \"beyond\" is ignored\n");
}

int main(void) {
    tof_rays_init(&ROBOT_MOUNTING);
    run(15);
    run(10);
    rate_change();
    far_test();
    printf("watch_tof: all tests passed at 15 and 10 Hz, and across a change\n");
    return 0;
}
