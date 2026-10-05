// Host test for picoA/app/rangefinder.c: the zone order, learning the floor
// during a turn with obstacles around, floor vs obstacle with the robot pitching,
// small bumps ignored, the horizon row, drops (no floor where it should be),
// against a fake VL53L8CX. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_rangefinder picoA/app/test/test_rangefinder.c -lm && build/test_rangefinder
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../rangefinder.c"

#define RAD(d) ((d) * RAD_PER_DEG)

// The fake sensor reports `mm` for the zone the robot sees at (row, col).
static void set_zone(int row, int col, int mm) {
    int zone = (7 - col) * 8 + row, first = zone * (int)VL53L8CX_NB_TARGET_PER_ZONE;
    fake_tof.nb_target_detected[zone] = mm > 0;
    fake_tof.distance_mm[first] = (int16_t)mm;
    fake_tof.target_status[first] = 5;
}

static float noise_m;       // +- this much, random, on every reading
static bool near_edge;      // a zone reports the nearest point of its patch (as the real sensor mostly does)
static bool anywhere;       // a zone reports a random point of its patch (as the real sensor sometimes does)

// Distance along direction d to a flat floor `height_m` below the sensor, and to
// a box in front of the robot (x from box_x_m, any y, box_h_m tall). 4 m = nothing.
static float cast_one(const float d[3], float height_m, float box_x_m, float box_h_m) {
    float best_m = 4.0f;
    if (d[2] < 0) best_m = fminf(best_m, height_m / -d[2]);
    if (box_h_m > 0 && d[0] > 0) {
        float t = (box_x_m - SENSOR_X_M) / d[0];
        if (t > 0 && height_m + t * d[2] < box_h_m) best_m = fminf(best_m, t);
    }
    return best_m;
}

// What zone i reads with the robot pitched by pitch_rad, in mm; 0 = nothing within 4 m.
static int range_to(int i, float pitch_rad, float height_m, float box_x_m, float box_h_m) {
    float d[3];
    float offset_rad = anywhere ? ((float)rand() / (float)RAND_MAX - 0.5f) * ZONE_RAD : 0;
    direction(zone_down_rad[i] + offset_rad, zone_left_rad[i], pitch_rad, d);
    float best_m = cast_one(d, height_m, box_x_m, box_h_m);
    if (near_edge) // the zone's upper and lower edges, 2.8° either way
        for (float k = -1; k <= 1; k += 2) {
            direction(zone_down_rad[i] + k * ZONE_RAD / 2, zone_left_rad[i], pitch_rad, d);
            best_m = fminf(best_m, cast_one(d, height_m, box_x_m, box_h_m));
        }
    if (best_m >= 4.0f) return 0;
    best_m += noise_m * (2 * (float)rand() / (float)RAND_MAX - 1);
    return (int)(best_m * 1000 + 0.5f);
}

static void fake_frame(float pitch_rad, float height_m, float box_x_m, float box_h_m) {
    for (int i = 0; i < RANGEFINDER_RAYS; i++)
        set_zone(i / 8, i % 8, range_to(i, pitch_rad, height_m, box_x_m, box_h_m));
    fake_tof_fresh = true;
}

static int count(const scan_t *s, ray_kind_t kind, int from_row) {
    int n = 0;
    for (int i = from_row * 8; i < RANGEFINDER_RAYS; i++) n += s->ray[i].kind == kind;
    return n;
}

int main(void) {
    range_frame_t f;
    scan_t s;
    int assumed;
    fake_now_us = 1000000;
    assert(rangefinder_init());
    assert(!rangefinder_poll(&f));

    // Zone order: the sensor's zones arrive turned 90°; the frame is as the robot sees it.
    for (int row = 0; row < 8; row++)
        for (int col = 0; col < 8; col++) set_zone(row, col, 100 + row * 10 + col);
    fake_tof_fresh = true;
    assert(rangefinder_poll(&f));
    assert(f.range_mm[0] == 100 && f.range_mm[7] == 107 && f.range_mm[8 * 7] == 170 && f.range_mm[63] == 177);
    assert(f.t_us == 1000000 - 1000000 / 15 / 2 - 2500);
    // Status: unsure readings and no target.
    fake_tof.target_status[0] = 4;
    fake_tof.nb_target_detected[1] = 0;
    fake_tof_fresh = true;
    rangefinder_poll(&f);
    // ULD zone 0 is what the robot sees top right, zone 1 the one below it.
    assert(f.range_mm[0 * 8 + 7] == RANGE_INVALID && f.range_mm[1 * 8 + 7] == RANGE_NO_TARGET);
    assert(f.status[0 * 8 + 7] == 4);
    // The first target unsure, the second sure: the second is used.
    fake_tof.nb_target_detected[0] = 2;
    fake_tof.distance_mm[1] = 555;
    fake_tof.target_status[1] = 5;
    fake_tof_fresh = true;
    rangefinder_poll(&f);
    assert(f.range_mm[0 * 8 + 7] == 555);

    // Before the floor is learned, the floor rows tell nothing; the upper rows still map.
    fake_frame(0, 0.07f, 0, 0);
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    assert(count(&s, RAY_UNUSED, 5) == 24); // the 5th row is used like the rows above until learned
    assert(count(&s, RAY_HIT, 0) == 0);

    // Learning during a turn: in a third of the directions a wall 25 cm away fills
    // the floor rows. The floor still comes out at the sensor's 7 cm.
    for (int k = 0; k < 200; k++) {
        fake_frame(RAD(-0.8f), 0.07f, k % 3 == 0 ? 0.25f : 0, 0.5f);
        rangefinder_poll(&f);
        rangefinder_learn_floor(&f, RAD(-0.8f));
    }
    assert(rangefinder_finish_floor(&assumed) == 32 && assumed == 0);
    // Flat floor, zone centres: 1.43 m, 48, 29, 21 cm (§4.2).
    const float expected_m[4] = {1.434f, 0.479f, 0.287f, 0.208f};
    for (int row = 4; row < 8; row++) assert(fabsf(rangefinder_floor_distance(row) - expected_m[row - 4]) < 0.01f);
    printf("floor learned with a wall in 1/3 of the turn: rows 5-8 see it at %.0f %.0f %.0f %.0f cm\n",
           (double)(rangefinder_floor_distance(4) * 100), (double)(rangefinder_floor_distance(5) * 100),
           (double)(rangefinder_floor_distance(6) * 100), (double)(rangefinder_floor_distance(7) * 100));

    // A flat floor stays floor while the robot nods +-2° (braking, accelerating).
    // (Nose up 2°, the 5th row's floor is beyond 4 m: nothing, so it says nothing.)
    for (float p = -2; p <= 2; p += 1) {
        fake_frame(RAD(p), 0.07f, 0, 0);
        rangefinder_poll(&f);
        rangefinder_scan(&f, RAD(p), &s);
        assert(count(&s, RAY_HIT, 0) == 0 && count(&s, RAY_FLOOR, 5) == 24);
    }

    // A drop: the floor 20 cm lower (a stair). Rows 7-8 read far beyond their floor:
    // no floor where it should be, marked where they expected it. No return: the same.
    // The 6th row's far readings tell nothing (on a shiny floor they're often reflections).
    fake_frame(0, 0.27f, 0, 0);
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    assert(count(&s, RAY_NO_FLOOR, 6) == 16 && count(&s, RAY_NO_FLOOR, 0) == 16 && count(&s, RAY_HIT, 0) == 0);
    for (int col = 0; col < 8; col++) assert(s.ray[5 * 8 + col].kind == RAY_UNUSED);
    for (int i = 6 * 8; i < RANGEFINDER_RAYS; i++) assert(fabsf(s.ray[i].z_m) < 0.005f);
    assert(fabsf(s.ray[7 * 8].x_m - SENSOR_X_M - 0.208f * cosf(RAD(19.7f)) * cosf(RAD(19.7f))) < 0.01f);
    fake_frame(0, 0.07f, 0, 0);
    for (int col = 0; col < 8; col++) set_zone(6, col, 0), set_zone(5, col, 0);
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    for (int col = 0; col < 8; col++) assert(s.ray[6 * 8 + col].kind == RAY_NO_FLOOR);
    for (int col = 0; col < 8; col++) assert(s.ray[5 * 8 + col].kind == RAY_UNUSED);
    // As on the robot: one 6th-row zone reads the wall 1.84 m away (a reflection).
    fake_frame(0, 0.07f, 0, 0);
    set_zone(5, 3, 1840);
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    assert(s.ray[5 * 8 + 3].kind == RAY_UNUSED && count(&s, RAY_NO_FLOOR, 0) == 0);
    // A reading 10 % beyond the floor is still the floor (noise, a slightly lower patch).
    fake_frame(0, 0.077f, 0, 0);
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    assert(count(&s, RAY_FLOOR, 5) == 24);

    // A 4 cm box 25 cm ahead is an obstacle (the 6th row hits its face 3.3 cm up);
    // a 1 cm bump isn't. (A 3 cm box at exactly 25 cm falls between rows: the 6th
    // passes over it, the 7th hits it 0.7 cm up, like the floor. Closer or farther
    // it's seen: 16-20 cm by the 7th row, 27-34 cm by the 6th.)
    fake_frame(0, 0.07f, 0.25f, 0.04f);
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    int box_hits = count(&s, RAY_HIT, 5);
    assert(box_hits >= 8);
    for (int i = 0; i < RANGEFINDER_RAYS; i++)
        if (s.ray[i].kind == RAY_HIT) assert(s.ray[i].z_m >= 0.02f && s.ray[i].z_m <= 0.04f && fabsf(s.ray[i].x_m - 0.25f) < 0.01f);
    fake_frame(0, 0.07f, 0.25f, 0.01f);
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    assert(count(&s, RAY_HIT, 0) == 0);
    printf("4 cm box at 25 cm: %d floor-row hits; 1 cm bump: none\n", box_hits);

    // The horizon row (5th) sees the floor at ~1.4 m: clear. A wall 80 cm away is
    // an obstacle on every row that reaches it above 2 cm.
    fake_frame(0, 0.07f, 0, 0);
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    for (int col = 0; col < 8; col++) assert(s.ray[4 * 8 + col].kind == RAY_CLEAR);
    for (int i = 0; i < 4 * 8; i++) assert(s.ray[i].kind == RAY_CLEAR); // nothing in range: 1 m clear
    for (int i = 0; i < 4 * 8; i++) assert(fabsf(hypotf(s.ray[i].x_m - SENSOR_X_M, s.ray[i].y_m - SENSOR_Y_M) - 1.0f) < 0.1f);
    fake_frame(0, 0.07f, 0.80f, 0.5f);
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    for (int col = 0; col < 8; col++) assert(s.ray[4 * 8 + col].kind == RAY_HIT);
    assert(count(&s, RAY_HIT, 0) == 40); // rows 0-4 reach the wall; rows 5-7 hit the floor first

    // As on the robot: each zone reads the near edge of its floor patch, with
    // +-1.5 cm of noise. Learned like that, the floor stays floor (no ring of false
    // obstacles), and a 10 cm box 40 cm ahead is still an obstacle.
    near_edge = true;
    noise_m = 0.015f;
    srand(2);
    rangefinder_forget_floor();
    for (int k = 0; k < 220; k++) {
        fake_frame(RAD(-0.8f), 0.07f, k % 3 == 0 ? 0.3f : 0, 0.5f);
        rangefinder_poll(&f);
        rangefinder_learn_floor(&f, RAD(-0.8f));
    }
    assert(rangefinder_finish_floor(&assumed) == 32 && assumed == 0);
    int false_hits = 0, false_drops = 0;
    for (int k = 0; k < 200; k++) {
        float p = RAD(-0.8f + (float)(k % 5 - 2) * 0.5f); // nodding +-1°
        fake_frame(p, 0.07f, 0, 0);
        rangefinder_poll(&f);
        rangefinder_scan(&f, p, &s);
        false_hits += count(&s, RAY_HIT, 0);
        false_drops += count(&s, RAY_NO_FLOOR, 0);
    }
    fake_frame(RAD(-0.8f), 0.07f, 0.40f, 0.10f);
    rangefinder_poll(&f);
    rangefinder_scan(&f, RAD(-0.8f), &s);
    int real_hits = count(&s, RAY_HIT, 0);
    printf("near-edge readings with noise: rows 5-8 see the floor at %.0f %.0f %.0f %.0f cm; "
           "%d false hits in 200 frames; 10 cm box at 40 cm: %d hits\n",
           (double)(rangefinder_floor_distance(4) * 100), (double)(rangefinder_floor_distance(5) * 100),
           (double)(rangefinder_floor_distance(6) * 100), (double)(rangefinder_floor_distance(7) * 100),
           false_hits, real_hits);
    assert(false_hits <= 2);
    assert(false_drops == 0);
    // On the robot the 6th row read anywhere in its floor patch (35-71 cm), not
    // only its near edge: still the floor, not a drop.
    near_edge = false;
    anywhere = true;
    for (int k = 0; k < 200; k++) {
        float p = RAD(-0.8f + (float)(k % 5 - 2) * 0.5f);
        fake_frame(p, 0.07f, 0, 0);
        rangefinder_poll(&f);
        rangefinder_scan(&f, p, &s);
        false_drops += count(&s, RAY_NO_FLOOR, 0);
    }
    anywhere = false;
    printf("readings anywhere in the floor patch: %d false drops in 200 frames\n", false_drops);
    assert(false_drops == 0);
    assert(real_hits >= 8);

    // In a room the 5th row mostly sees walls 2-3 m away: that isn't its floor
    // (farther than its centre meets the floor), so it isn't learned, and the real
    // floor at 60-90 cm isn't called an obstacle.
    near_edge = false;
    noise_m = 0;
    rangefinder_forget_floor();
    for (int k = 0; k < 220; k++) {
        fake_frame(0, 0.07f, 2.5f, 1.0f);
        for (int col = 0; col < 8; col++) set_zone(4, col, k % 4 ? 2500 : 700);
        rangefinder_poll(&f);
        rangefinder_learn_floor(&f, 0);
    }
    assert(rangefinder_finish_floor(&assumed) == 24 && assumed == 0 && rangefinder_floor_distance(4) == 0);
    for (int col = 0; col < 8; col++) set_zone(4, col, 700);
    fake_tof_fresh = true;
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    // Unlearned, the 5th row is used like the rows above: 70 cm may be the floor at
    // its lower edge (free up to there); a wall at 2.5 m is free space up to it too.
    for (int col = 0; col < 8; col++) assert(s.ray[4 * 8 + col].kind == RAY_CLEAR);
    for (int col = 0; col < 8; col++) set_zone(4, col, 300);
    fake_tof_fresh = true;
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    for (int col = 0; col < 8; col++) assert(s.ray[4 * 8 + col].kind == RAY_HIT); // something 30 cm away

    // Learning next to a desk's edge: in half the turn the floor rows look over the
    // edge (the room's floor 75 cm lower). The floor still comes out right, and the
    // edge is a drop for the two lowest rows.
    rangefinder_forget_floor();
    for (int k = 0; k < 220; k++) {
        fake_frame(RAD(-0.8f), k % 2 ? 0.82f : 0.07f, 0, 0);
        rangefinder_poll(&f);
        rangefinder_learn_floor(&f, RAD(-0.8f));
    }
    assert(rangefinder_finish_floor(&assumed) == 32 && assumed == 0);
    for (int row = 4; row < 8; row++) assert(fabsf(rangefinder_floor_distance(row) - expected_m[row - 4]) < 0.01f);
    fake_frame(RAD(-0.8f), 0.82f, 0, 0);
    rangefinder_poll(&f);
    rangefinder_scan(&f, RAD(-0.8f), &s);
    assert(count(&s, RAY_NO_FLOOR, 6) == 16);
    // Learning where the floor rows never see the floor (all over the edge): the two
    // lowest rows still tell the drop, from the floor the sensor's height gives.
    rangefinder_forget_floor();
    for (int k = 0; k < 220; k++) {
        fake_frame(0, 0.82f, 0, 0);
        rangefinder_poll(&f);
        rangefinder_learn_floor(&f, 0);
    }
    assert(rangefinder_finish_floor(&assumed) == 0 && assumed == 16);
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    assert(count(&s, RAY_NO_FLOOR, 6) == 16);
    fake_frame(0, 0.07f, 0, 0);
    rangefinder_poll(&f);
    rangefinder_scan(&f, 0, &s);
    assert(count(&s, RAY_FLOOR, 6) == 16 && count(&s, RAY_HIT, 0) == 0);
    printf("next to a desk's edge: the floor learned, the edge a drop; never seeing the floor: rows 7-8 assume it at %.0f %.0f cm, the edge still a drop\n",
           (double)(rangefinder_floor_distance(6) * 100), (double)(rangefinder_floor_distance(7) * 100));

    printf("OK: rangefinder orders zones, picks sure targets, learns the floor, tells floor from obstacles\n");
    return 0;
}
