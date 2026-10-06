// Host test for picoA/app/camera_motion.c: a simulated camera (a textured room,
// pixel noise, a slight wobble of the whole image, a driver that wants a new
// exposure when the picture is 25 % off and adjusts it in steps when not held)
// must show no movement in a still room however long it is watched, nor at dusk
// (the exposure adjusted when calm); a light switched off or on is movement, then
// still, and the exposure is adjusted once calm, not while something moves (but
// after 30 s of waiting); a person stepping in on the left, someone walking up
// until they fill the view, a small object in one block and a hand waving for 3 s
// must show; a box put down is still after ~1 s, and so is the place it is taken
// from. The same at 100 frames/s.
// Run from the repo root (run_tests.sh does):
//   cc -std=c11 -Wall -Wextra -IpicoA/drivers -o build/test_camera_motion picoA/app/test/test_camera_motion.c -lm && build/test_camera_motion
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../change_grid.c"
#include "../camera_motion.c"

// The camera driver as camera_motion sees it: the picture's mean brightness off
// its target by 25 % for a while (5 s, less the further off, 0.5 s at least): it
// wants a new exposure; not held, it steps exposure x gain by at most x1.25, 3
// settling frames each, until on target.
static bool held;
static float light = 1.0f, exposure = 1.0f; // the room's light, the camera's exposure x gain
static float target;                         // the room's mean brightness at light 1, exposure 1
static int settle_left;
static bool off_target, wants;
static uint32_t off_since_us;
void camera_hold_exposure(bool hold) { held = hold; }

static uint8_t room_px[CAMERA_WIDTH * CAMERA_HEIGHT], scene_px[CAMERA_WIDTH * CAMERA_HEIGHT];
static uint8_t px[CAMERA_WIDTH * CAMERA_HEIGHT];
static camera_frame_t f = {.pixels = px, .line_us = 42.7f, .exposure_us = 40000.0f, .gain = 1.0f};
static uint32_t frame_us = 40000; // 25 frames/s
static motion_obs_t obs[8];

static float random_noise(float amplitude) { return amplitude * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f); }

// The room: walls, furniture and a floor of different brightness, each with its texture.
static void make_room(void) {
    for (int y = 0; y < CAMERA_HEIGHT; y++)
        for (int x = 0; x < CAMERA_WIDTH; x++) {
            float v = y > 80 ? 70.0f : 120.0f + 40.0f * sinf((float)x / 13.0f);
            if (x > 100 && x < 130 && y > 40) v = 30.0f; // a dark cupboard
            if (x > 20 && x < 50 && y < 30) v = 200.0f;  // a bright window
            room_px[y * CAMERA_WIDTH + x] = (uint8_t)(v + random_noise(20.0f));
        }
}

// Draws something into the scene, pixels x0..x1, y0..y1, with a texture of its own
// (the same every frame, as a real surface's).
static void draw(int x0, int y0, int x1, int y1, float v) {
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            uint32_t h = (uint32_t)(x * 7919 + y * 104729) * 2654435761u;
            scene_px[y * CAMERA_WIDTH + x] = (uint8_t)(v + (float)(h >> 24) / 255.0f * 30.0f - 15.0f);
        }
}

// One camera frame of the room with `scene` drawn into it; returns what camera_motion
// reports, and whether it used the frame.
static int frame(void (*scene)(void), bool *used) {
    memcpy(scene_px, room_px, sizeof scene_px);
    if (scene) scene();
    float wobble = random_noise(1.5f), sum = 0.0f;
    for (int i = 0; i < CAMERA_WIDTH * CAMERA_HEIGHT; i++) {
        float v = (float)scene_px[i] * light * exposure + wobble + random_noise(8.0f);
        px[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
        sum += (float)px[i];
    }
    float change = target / (sum / (CAMERA_WIDTH * CAMERA_HEIGHT));
    f.number++;
    f.last_row_us += frame_us;
    f.settling = settle_left > 0;
    if (change > 0.8f && change < 1.25f) off_target = wants = false;
    else {
        if (!off_target) { off_target = true; off_since_us = f.last_row_us; }
        float factor = change > 1.0f ? change : 1.0f / change;
        float wait_us = fmaxf(5e6f * 0.25f / (factor - 1.0f), 5e5f);
        if ((float)(f.last_row_us - off_since_us) >= wait_us) wants = true;
    }
    f.wants_exposure_change = wants;
    if (settle_left > 0) settle_left--;
    else if (!held && f.wants_exposure_change) {
        exposure *= change < 0.8f ? 0.8f : change > 1.25f ? 1.25f : change;
        settle_left = 3;
    }
    uint32_t before = last_us;
    bool watching = state == WATCHING;
    int n = camera_motion_add(&f, obs, 8);
    bool was_used = watching && state == WATCHING && last_us != before; // not an exposure adjustment
    assert(was_used ? n >= 0 : n == -1); // a skipped frame must not read as "nothing moved"
    if (used) *used = was_used;
    return n;
}

// Watches for `seconds` with `scene` in it; returns how many used frames showed movement.
static int watch(float seconds, void (*scene)(void)) {
    int moving = 0;
    for (int k = 0; k < (int)(seconds * 1e6f / (float)frame_us); k++) moving += frame(scene, 0) > 0;
    return moving;
}

// Frames until movement shows (counting only the frames camera_motion used), at most 10.
static int frames_to_find(void (*scene)(void)) {
    for (int used_frames = 0, k = 0; k < 100; k++) {
        bool used;
        int n = frame(scene, &used);
        used_frames += used;
        if (n > 0) return used_frames;
    }
    return 99;
}

static void quiet_again(void) {
    watch(2.0f, 0); // what moved is steady for 1 s
    assert(watch(3.0f, 0) == 0);
}

static void person(void) { draw(24, 16, 55, 104, 50.0f); } // blocks 3-6 (left of centre), rows 2-13
static void small(void) { draw(80, 56, 87, 63, 220.0f); }  // one block, right of centre
// A hand (16 x 16 pixels) waving back and forth across blocks 4-10 of the middle
// rows at 100 px/s (~33 deg/s; 0.6 m/s at 1 m), whatever the frame rate.
static void hand(void) {
    float phase = fmodf((float)f.last_row_us * 100e-6f, 96.0f); // pixels along there and back
    int x = 32 + (int)(phase < 48.0f ? phase : 96.0f - phase);
    draw(x, 48, x + 15, 63, 180.0f);
}
static void box(void) { draw(64, 88, 95, 111, 160.0f); } // on the floor, in the middle
// Someone walking up from the right at ~1 m/s until they fill the view: 2 blocks
// wider every 80 ms.
static uint32_t walk_start_us;
static void walk_up(void) {
    int w = (int)((float)(f.last_row_us - walk_start_us) * 200e-6f); // pixels
    if (w > CAMERA_WIDTH) w = CAMERA_WIDTH;
    if (w > 0) draw(CAMERA_WIDTH - w, 0, CAMERA_WIDTH - 1, CAMERA_HEIGHT - 1, 45.0f);
}

// Frames (in seconds) until the exposure was adjusted while still and the camera
// watches again; asserts no movement meanwhile.
static float until_adjusted(float max_s, void (*scene)(void)) {
    uint32_t before = camera_motion_exposure_adjustments();
    for (int k = 0; k < (int)(max_s * 1e6f / (float)frame_us); k++) {
        assert(frame(scene, 0) <= 0);
        if (camera_motion_exposure_adjustments() > before && camera_motion_ready()) return (float)k * (float)frame_us / 1e6f;
    }
    return 99.0f;
}

static void start(void) {
    camera_motion_restart();
    assert(!held);
    watch(1.5f, 0);
    assert(held && camera_motion_ready());
}

static void run(void) {
    light = exposure = 1.0f;
    start();
    assert(watch(300.0f, 0) == 0);
    printf("ok: a still room with noise: no movement in 5 min\n");

    uint32_t adjusted = camera_motion_exposure_adjustments();
    for (int k = 0; k < 600; k++) { // 0.1 s steps at 100 frames/s, 0.08 s at 25: ~1 min either way
        light = 1.0f - 0.15f * (float)k / 600.0f;
        assert(watch(0.1f, 0) == 0);
    }
    assert(camera_motion_exposure_adjustments() == adjusted);
    printf("ok: dusk (15 %% less light over 1 min): no movement\n");
    uint32_t darker_us = f.last_row_us;
    for (int k = 0; k < 600 && camera_motion_exposure_adjustments() == adjusted; k++) {
        light = 0.85f - 0.15f * (float)k / 600.0f;
        assert(watch(0.1f, 0) == 0);
    }
    float s_ = (float)(f.last_row_us - darker_us) / 1e6f;
    assert(camera_motion_exposure_adjustments() == adjusted + 1);
    uint32_t adjust_us = f.last_row_us;
    while (f.last_row_us - adjust_us < 5000000 && !camera_motion_ready()) assert(frame(0, 0) <= 0);
    float s2 = (float)(f.last_row_us - adjust_us) / 1e6f;
    assert(s2 < 5.0f && held && watch(5.0f, 0) == 0);
    printf("ok: darker still: the exposure adjusted when calm after %.0f s, watching again after %.1f s, no movement\n",
           (double)s_, (double)s2);

    // The light switched off: movement, still after ~1 s; once calm for 5 s the
    // exposure is adjusted.
    light *= 0.5f;
    uint32_t switched_us = f.last_row_us;
    assert(watch(0.5f, 0) > 0);
    int quiet_from = -1;
    for (int k = 0; k < (int)(3e6f / (float)frame_us); k++) {
        bool u, m = frame(0, &u) > 0;
        if (u && !m && quiet_from < 0) quiet_from = k;
        if (quiet_from >= 0) assert(!m);
    }
    float still_s = 0.5f + (float)quiet_from * (float)frame_us / 1e6f;
    assert(quiet_from >= 0 && still_s < 2.0f);
    assert(until_adjusted(8.0f, 0) < 8.0f);
    float adjust_s = (float)(f.last_row_us - switched_us) / 1e6f;
    assert(watch(5.0f, 0) == 0);
    printf("ok: the light switched off: movement, still after %.1f s, watching again with a new exposure after %.1f s\n",
           (double)still_s, (double)adjust_s);
    light *= 2.0f;
    assert(watch(0.5f, 0) > 0);
    watch(2.0f, 0); // still after ~1 s
    assert(until_adjusted(10.0f, 0) < 10.0f && watch(5.0f, 0) == 0);
    printf("ok: and on again: movement, then still, the exposure adjusted when calm\n");

    int n = frames_to_find(person);
    assert(n <= 2);
    assert(obs[0].cells >= 30 && obs[0].range_m < 0);
    assert(obs[0].where[1] > 0.1f && obs[0].where[0] > 0.9f); // left, ahead
    printf("ok: a person steps in on the left: %d blocks, where (%.2f %.2f %.2f), in %d frames\n", obs[0].cells,
           (double)obs[0].where[0], (double)obs[0].where[1], (double)obs[0].where[2], n);
    quiet_again();
    printf("ok: ...standing still, they are the view after ~1 s; walking off is movement, then the view again\n");

    n = frames_to_find(small);
    assert(n == 3 && obs[0].cells == 1 && obs[0].where[1] < 0);
    quiet_again();
    printf("ok: a small object in one block (right): found in %d frames\n", n);

    int moving = 0, used_frames = 0;
    for (int k = 0; k < (int)(3e6f / (float)frame_us); k++) {
        bool u;
        moving += frame(hand, &u) > 0;
        used_frames += u;
    }
    assert(moving >= used_frames - 2);
    quiet_again();
    printf("ok: a hand waving for 3 s: movement in %d of %d frames\n", moving, used_frames);

    quiet_from = -1;
    moving = 0;
    for (int k = 0; k < (int)(4e6f / (float)frame_us); k++) {
        bool u, m = frame(box, &u) > 0;
        moving += m;
        if (u && !m && moving && quiet_from < 0) quiet_from = k;
        if (quiet_from >= 0) assert(!m);
    }
    still_s = (float)quiet_from * (float)frame_us / 1e6f;
    assert(moving && still_s >= 1.0f && still_s < 1.6f);
    printf("ok: a box put down is still after %.1f s\n", (double)still_s);
    assert(watch(0.5f, 0) > 0);
    quiet_again();
    printf("ok: taken away: movement, then still\n");

    // Someone dark walking up until they fill the view: movement all the way, and
    // the darker picture doesn't change the exposure while they move.
    adjusted = camera_motion_exposure_adjustments();
    walk_start_us = f.last_row_us;
    moving = used_frames = 0;
    for (int k = 0; k < (int)(1e6f / (float)frame_us); k++) { // 1 s: they fill the view after 0.8 s
        bool u;
        moving += frame(walk_up, &u) > 0;
        used_frames += u;
    }
    assert(camera_motion_exposure_adjustments() == adjusted && moving >= used_frames - 2);
    printf("ok: someone walking up until they fill the view: movement in %d of %d frames\n", moving, used_frames);
    // Standing there: still after ~1 s, then calm: the exposure follows the dark picture.
    watch(3.0f, walk_up); // still: blocks at the edge of differing take up to ~2.5 s
    assert(until_adjusted(10.0f, walk_up) < 10.0f && watch(3.0f, walk_up) == 0);
    assert(watch(0.5f, 0) > 0); // walking off: movement
    watch(2.0f, 0);
    assert(until_adjusted(10.0f, 0) < 10.0f && watch(3.0f, 0) == 0);
    printf("ok: ...standing there: the exposure adjusted when calm; walking off: movement, then adjusted back\n");

    // Something that never stops moving: the exposure is adjusted after 30 s anyway.
    light *= 0.6f;
    adjusted = camera_motion_exposure_adjustments();
    uint32_t dimmed_us = f.last_row_us;
    while (f.last_row_us - dimmed_us < 40000000 && camera_motion_exposure_adjustments() == adjusted) frame(hand, 0);
    float waited = (float)(f.last_row_us - dimmed_us) / 1e6f; // the driver's wait (~2 s), then 30 s
    assert(waited >= 30.0f && waited < 34.0f);
    watch(3.0f, hand);
    light /= 0.6f;
    watch(3.0f, 0); // the hand gone and the light back: movement, then still
    assert(until_adjusted(40.0f, 0) < 40.0f && watch(3.0f, 0) == 0);
    printf("ok: a hand that never stops while the picture is too dark: the exposure adjusted after %.0f s\n",
           (double)waited);

    camera_motion_restart();
    watch(1.5f, person);
    assert(camera_motion_ready() && watch(5.0f, person) == 0);
    printf("ok: what stands still while the view is learned is part of it\n");
    watch(3.0f, 0); // they walk off
    start();
}

int main(void) {
    srand(1);
    make_room();
    float sum = 0.0f;
    for (int i = 0; i < CAMERA_WIDTH * CAMERA_HEIGHT; i++) sum += (float)room_px[i];
    target = sum / (CAMERA_WIDTH * CAMERA_HEIGHT);
    run();
    printf("At 100 frames/s:\n");
    frame_us = 10000;
    run();
    settle_left = 30;
    assert(watch(0.3f, person) == 0);
    printf("ok: settling frames are not used\n");
    printf("camera_motion: all tests passed\n");
    return 0;
}
