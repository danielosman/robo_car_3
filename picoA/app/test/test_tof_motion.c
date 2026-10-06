// Host test for picoA/app/tof_motion.c: a simulated room as on the robot (range
// noise, unsure frames, a floor zone reading its reflection farther away, far zones
// jumping farther or to nothing, a far zone that is sometimes sure and sometimes
// empty, floor zones reading the wall behind, a door frame's edge now and then) must show no movement however long it is watched; a person stepping in on
// the left, a thin pole in one zone, a hand waving for 3 s and a target where
// nothing was must show; a box
// put down is still after ~1 s, and taking it away is not movement.
// Run from the repo root (run_tests.sh does):
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_tof_motion picoA/app/test/test_tof_motion.c -lm && build/test_tof_motion
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../rangefinder.c"
#include "../change_grid.c"
#include "../tof_motion.c"

static range_frame_t f;
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

static int frame(void) { return tof_motion_add(&f, obs, 8); }

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

int main(void) {
    srand(1);
    tof_motion_restart();
    assert(watch(LEARN_FRAMES, 0) == 0 && tof_motion_ready());
    assert(watch(15 * 300, 0) == 0);
    printf("ok: a still room with noise, unsure frames, reflections and farther surfaces: no movement in 5 min\n");

    // Someone standing there while the view is learned is the view.
    tof_motion_restart();
    assert(watch(LEARN_FRAMES, person) == 0);
    assert(watch(30, person) == 0);
    printf("ok: what stands still while the view is learned is part of it\n");
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
    printf("tof_motion: all tests passed\n");
    return 0;
}
