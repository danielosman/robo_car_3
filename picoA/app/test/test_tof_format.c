// Host test for picoA/app/tof_format.c: the table (every use packs as compact, 277 B,
// its message type); a posed frame packed and unpacked: the distance in whole cm
// (0 nothing, 1 for the shortest, 254 = 2.54 m, 255 = beyond, unpacked as "at least
// 2.55 m"), signal and sigma capped at 255, kind and targets; the header: frame
// number, time, period (0.1 ms), pose (mm, ±32.7 m; yaw wrapped to ±180° in 0.01°,
// pitch, the turn), the flags; a frame unpacked and packed again gives the same
// bytes; a wrong length is refused. The layout byte by byte, as the PC will read it.
// Run from the repo root (run_tests.sh does).
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../tof_format.c"

static uint8_t bytes[TOF_FORMAT_MAX];

static posed_frame_t round_trip(const posed_frame_t *f) {
    posed_frame_t out;
    uint32_t n = tof_pack(TOF_USE_MAP, f, bytes);
    assert(n == TOF_COMPACT_BYTES);
    assert(tof_unpack(TOF_FORMAT_COMPACT, bytes, n, &out));
    return out;
}

static tof_zone_t sure(uint16_t mm) { return (tof_zone_t){.distance_mm = mm, .kind = TOF_SURE, .targets = 1}; }

// One zone through the format.
static tof_zone_t zone_trip(tof_zone_t z, bool *beyond) {
    posed_frame_t f = {.frame = {.beyond_mm = TOF_EXACT}};
    f.frame.zone[5] = z;
    posed_frame_t out = round_trip(&f);
    *beyond = tof_beyond(&out.frame, 5);
    return out.frame.zone[5];
}

static void table(void) {
    for (int u = 0; u < TOF_USES; u++) {
        assert(tof_format_of((tof_use_t)u) == TOF_FORMAT_COMPACT);
        posed_frame_t f = {0};
        assert(tof_pack((tof_use_t)u, &f, bytes) == tof_format_size(TOF_FORMAT_COMPACT));
    }
    assert(tof_format_size(TOF_FORMAT_COMPACT) == 277 && TOF_FORMAT_MAX >= 277);
    assert(tof_format_message_type(TOF_FORMAT_COMPACT) != 0);
    printf("ok: map, watch and snapshot all pack as compact: 277 B (21 header + 64 x 4), 2.8 KB/s at 10 Hz\n");
}

static void distances(void) {
    struct { uint16_t mm, kind, want_mm; bool beyond; } cases[] = {
        {0, TOF_NOTHING, 0, false},      // nothing in range: 0
        {0, TOF_UNSURE, 0, false},       // an unsure target without a distance: 0 too (the kind says which)
        {4, TOF_SURE, 10, false},        // the shortest distance is 1 cm, never 0 ("nothing")
        {5, TOF_SURE, 10, false},
        {14, TOF_SURE, 10, false},
        {15, TOF_SURE, 20, false},       // whole cm, rounded
        {537, TOF_LESS_SURE, 540, false},
        {2540, TOF_SURE, 2540, false},   // 254: the farthest distance
        {2544, TOF_SURE, 2540, false},
        {2545, TOF_SURE, 2550, true},    // 255: beyond 2.54 m...
        {3000, TOF_SURE, 2550, true},    // ...at least 2.55 m
        {32767, TOF_LESS_SURE, 2550, true},
        {3000, TOF_UNSURE, 2550, false}, // unsure: its distance tells nothing, beyond or not
    };
    for (size_t k = 0; k < sizeof cases / sizeof cases[0]; k++) {
        bool beyond;
        tof_zone_t z = sure(cases[k].mm);
        z.kind = (uint8_t)cases[k].kind;
        tof_zone_t out = zone_trip(z, &beyond);
        assert(out.distance_mm == cases[k].want_mm && out.kind == cases[k].kind && beyond == cases[k].beyond);
    }
    // A distance on a "nothing" zone is not sent.
    bool beyond;
    tof_zone_t nothing = {.distance_mm = 500, .kind = TOF_NOTHING};
    assert(zone_trip(nothing, &beyond).distance_mm == 0);
    printf("ok: distances in whole cm: 0 nothing, at least 1, 254 the farthest, 255 beyond 2.54 m\n");
}

static void caps(void) {
    uint16_t in[] = {0, 1, 254, 255, 256, 1000, 65535}, want[] = {0, 1, 254, 255, 255, 255, 255};
    for (int k = 0; k < 7; k++) {
        bool beyond;
        tof_zone_t z = sure(1000);
        z.signal = in[k];
        z.sigma_mm = in[6 - k];
        tof_zone_t out = zone_trip(z, &beyond);
        assert(out.signal == want[k] && out.sigma_mm == want[6 - k]);
    }
    for (int kind = 0; kind < 4; kind++)
        for (int targets = 0; targets <= 4; targets++) {
            bool beyond;
            tof_zone_t z = {.distance_mm = 800, .kind = (uint8_t)kind, .targets = (uint8_t)targets};
            tof_zone_t out = zone_trip(z, &beyond);
            assert(out.kind == kind && out.targets == targets);
        }
    printf("ok: signal and sigma capped at 255; kind and targets kept\n");
}

static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

static void header(void) {
    posed_frame_t f = {.frame = {.number = 0xFFFFFFF0u, .t_us = 0xFFFFFF00u, .period_us = 66666, .beyond_mm = TOF_EXACT},
                       .x_m = -1.2346f, .y_m = 25.0f, .yaw_rad = 1.0f, .pitch_rad = -0.05f, .turn_rad = 0.0523f,
                       .still = true, .pose_known = true};
    posed_frame_t o = round_trip(&f);
    assert(o.frame.number == 0xFFFFFFF0u && o.frame.t_us == 0xFFFFFF00u && o.frame.period_us == 66700);
    assert(o.frame.beyond_mm == TOF_COMPACT_BEYOND_MM);
    assert(near(o.x_m, -1.235f, 1e-6f) && near(o.y_m, 25.0f, 1e-6f));
    assert(near(o.yaw_rad, 1.0f, 0.005f * RAD_PER_DEG) && near(o.pitch_rad, -0.05f, 0.005f * RAD_PER_DEG));
    assert(near(o.turn_rad, 0.0523f, 0.005f * RAD_PER_DEG) && o.still && o.pose_known);
    // The layout: number, time, period, x, y, yaw, pitch, turn, flags, then the zones.
    assert(bytes[0] == 0xF0 && bytes[3] == 0xFF && bytes[8] == (667 & 0xFF) && bytes[9] == 667 >> 8);
    assert((int16_t)(bytes[10] | bytes[11] << 8) == -1235 && (int16_t)(bytes[12] | bytes[13] << 8) == 25000);
    assert(bytes[20] == 3);
    f.still = false;
    assert(!round_trip(&f).still && bytes[20] == 1);
    f.pose_known = false;
    assert(!round_trip(&f).pose_known && bytes[20] == 0);
    f.frame.period_us = 100000;
    assert(round_trip(&f).frame.period_us == 100000);
    f.frame.period_us = 1000000; // 1 Hz
    assert(round_trip(&f).frame.period_us == 1000000);

    // Far: x, y held at ±32.767 m (a flat is far smaller).
    f.x_m = 40.0f, f.y_m = -40.0f;
    o = round_trip(&f);
    assert(near(o.x_m, 32.767f, 1e-6f) && near(o.y_m, -32.767f, 1e-6f));
    // Yaw wrapped to ±180°, in 0.01°; −180° is 180°.
    struct { float in_deg, want_deg; } yaws[] = {{0, 0}, {179.996f, 180}, {180.0f, 180}, {-180.0f, 180},
                                                  {180.01f, -179.99f}, {190, -170}, {-190, 170}, {-540, 180},
                                                  {720.5f, 0.5f}, {-0.004f, 0}, {-45.678f, -45.68f}};
    for (size_t k = 0; k < sizeof yaws / sizeof yaws[0]; k++) {
        f.yaw_rad = yaws[k].in_deg * RAD_PER_DEG;
        float got = round_trip(&f).yaw_rad * DEG_PER_RAD;
        assert(near(got, yaws[k].want_deg, 0.006f));
    }
    // Pitch and turn: ±327 deg at most; a few degrees kept to 0.01°.
    f.pitch_rad = 4.567f * RAD_PER_DEG, f.turn_rad = -400 * RAD_PER_DEG;
    o = round_trip(&f);
    assert(near(o.pitch_rad * DEG_PER_RAD, 4.57f, 0.006f) && near(o.turn_rad * DEG_PER_RAD, -327.67f, 0.006f));
    printf("ok: the header: number, time, period in 0.1 ms, x and y in mm (held at 32.7 m), yaw wrapped"
           " to +-180 in 0.01 deg, pitch, turn, flags\n");
}

static void again_and_refused(void) {
    srand(4);
    posed_frame_t f = {.frame = {.number = 77, .t_us = 1234567, .period_us = 100000, .beyond_mm = TOF_EXACT},
                       .x_m = 0.5f, .y_m = -0.25f, .yaw_rad = -2.0f, .pitch_rad = 0.01f, .pose_known = true};
    for (int i = 0; i < TOF_ZONES; i++)
        f.frame.zone[i] = (tof_zone_t){.distance_mm = (uint16_t)(rand() % 4000), .signal = (uint16_t)(rand() % 400),
                                       .sigma_mm = (uint16_t)(rand() % 300), .kind = (uint8_t)(rand() % 4),
                                       .targets = (uint8_t)(rand() % 5)};
    posed_frame_t once = round_trip(&f);
    uint8_t first[TOF_FORMAT_MAX];
    memcpy(first, bytes, sizeof first);
    posed_frame_t twice = round_trip(&once);
    assert(memcmp(first, bytes, TOF_COMPACT_BYTES) == 0); // unpacked and packed again: the same bytes
    for (int i = 0; i < TOF_ZONES; i++) {
        const tof_zone_t *a = &f.frame.zone[i], *b = &once.frame.zone[i];
        if (a->kind == TOF_NOTHING) assert(b->distance_mm == 0);
        else if (a->distance_mm < 2545) assert(abs((int)a->distance_mm - (int)b->distance_mm) <= 5);
        else assert(b->distance_mm == TOF_COMPACT_BEYOND_MM);
        assert(memcmp(b, &twice.frame.zone[i], sizeof *b) == 0);
    }
    posed_frame_t untouched = {.x_m = 9};
    assert(!tof_unpack(TOF_FORMAT_COMPACT, bytes, TOF_COMPACT_BYTES - 1, &untouched) && untouched.x_m == 9);
    assert(!tof_unpack(TOF_FORMAT_COMPACT, bytes, TOF_COMPACT_BYTES + 1, &untouched) && untouched.x_m == 9);
    printf("ok: within 5 mm below 2.545 m; packing an unpacked frame gives the same bytes; a wrong length refused\n");
}

static float uniform(float lo, float hi) { return lo + (hi - lo) * (float)rand() / (float)RAND_MAX; }
static float angle_diff(float a, float b) { return fabsf(wrapped(a - b)); }

// 1000 random posed frames: every field back within its bound; the bytes of a frame
// packed from an unpacked one are the bytes it came from.
static void random_frames(void) {
    srand(9);
    for (int n = 0; n < 1000; n++) {
        posed_frame_t f = {.frame = {.number = (uint32_t)rand() * 2654435761u, .t_us = (uint32_t)rand() * 40503u,
                                     .period_us = (uint32_t)(20000 + rand() % 980000), .beyond_mm = TOF_EXACT},
                           .x_m = uniform(-30, 30), .y_m = uniform(-30, 30), .yaw_rad = uniform(-10, 10),
                           .pitch_rad = uniform(-0.3f, 0.3f), .turn_rad = uniform(-3, 3),
                           .still = rand() % 2, .pose_known = rand() % 2};
        for (int i = 0; i < TOF_ZONES; i++)
            f.frame.zone[i] = (tof_zone_t){.distance_mm = (uint16_t)(rand() % 5000), .signal = (uint16_t)(rand() % 70000),
                                           .sigma_mm = (uint16_t)(rand() % 600), .kind = (uint8_t)(rand() % 4),
                                           .targets = (uint8_t)(rand() % 5)};
        posed_frame_t o = round_trip(&f);
        uint8_t first[TOF_FORMAT_MAX];
        memcpy(first, bytes, sizeof first);
        const tof_frame_t *a = &f.frame, *b = &o.frame;
        assert(b->number == a->number && b->t_us == a->t_us && abs((int)b->period_us - (int)a->period_us) <= 50);
        assert(fabsf(o.x_m - f.x_m) <= 0.0005f + 1e-5f && fabsf(o.y_m - f.y_m) <= 0.0005f + 1e-5f);
        assert(angle_diff(o.yaw_rad, f.yaw_rad) <= 0.005f * RAD_PER_DEG + 1e-5f && o.yaw_rad > -PI_F && o.yaw_rad <= PI_F + 1e-6f);
        assert(fabsf(o.pitch_rad - f.pitch_rad) <= 0.005f * RAD_PER_DEG + 1e-6f);
        assert(fabsf(o.turn_rad - f.turn_rad) <= 0.005f * RAD_PER_DEG + 1e-6f);
        assert(o.still == f.still && o.pose_known == f.pose_known);
        for (int i = 0; i < TOF_ZONES; i++) {
            const tof_zone_t *za = &a->zone[i], *zb = &b->zone[i];
            assert(zb->kind == za->kind && zb->targets == za->targets);
            assert(zb->signal == (za->signal > 255 ? 255 : za->signal) && zb->sigma_mm == (za->sigma_mm > 255 ? 255 : za->sigma_mm));
            if (za->kind == TOF_NOTHING || za->distance_mm == 0) assert(zb->distance_mm == 0);
            else if (za->distance_mm >= 2545) assert(zb->distance_mm == 2550 && (!tof_valid(zb) || tof_beyond(b, i)));
            else if (za->distance_mm < 5) assert(zb->distance_mm == 10); // at least 1 cm
            else assert(abs((int)zb->distance_mm - (int)za->distance_mm) <= 5 && !tof_beyond(b, i));
        }
        round_trip(&o);
        assert(memcmp(first, bytes, TOF_COMPACT_BYTES) == 0);
    }
    printf("ok: 1000 random frames: every field within its bound, packing an unpacked one gives its bytes\n");
}

int main(void) {
    table();
    distances();
    caps();
    header();
    again_and_refused();
    random_frames();
    printf("tof_format: all tests passed\n");
    return 0;
}
