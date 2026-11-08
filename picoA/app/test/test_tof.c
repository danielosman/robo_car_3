// Host test for picoA/app/tof.c against a fake VL53L8CX (stubs/vl53.h): the rate (a
// setting, 1-15 Hz; refused ones change nothing; the firmware uploaded once); each
// frame's own time and period; the frames a second for status; the zone order (the
// grid turned 0, 90, 180, 270° on the PCB); one target per zone: the first with
// status 5, 6 or 9 and a distance (12, 14 and the rest skipped), its kind (5 sure,
// 6 and 9 less sure), signal (capped) and sigma; no valid target: the first,
// unsure; none: nothing; how many targets the sensor saw.
// Run from the repo root (run_tests.sh does; it makes build/host/robot_mounting.h first).
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "../tof.c"
#include "robot_mounting.h"

#define T VL53L8CX_NB_TARGET_PER_ZONE

// Sensor zone `zone`, target t: distance, status.
static void target(int zone, int t, int mm, int status) {
    int i = zone * (int)T + t;
    fake_tof.distance_mm[i] = (int16_t)mm;
    fake_tof.target_status[i] = (uint8_t)status;
    fake_tof.signal_per_spad[i] = (uint32_t)(100 + 10 * t);
    fake_tof.range_sigma_mm[i] = (uint16_t)(3 + t);
    if (fake_tof.nb_target_detected[zone] < t + 1) fake_tof.nb_target_detected[zone] = (uint8_t)(t + 1);
}

static tof_frame_t poll(void) {
    tof_frame_t f;
    fake_now_us += 100000;
    fake_tof_fresh = true;
    assert(tof_poll(&f));
    return f;
}

static void rate(void) {
    mounting_tof_t m = ROBOT_MOUNTING.tof;
    tof_frame_t f;
    assert(tof_hz() == 0);
    assert(!tof_init(&m, 0) && !tof_init(&m, 16) && !tof_init(&m, -1) && !fake_tof_started);
    mounting_tof_t bad = m;
    bad.zones_turn_deg = 45;
    assert(!tof_init(&bad, 10) && !fake_tof_started);
    assert(tof_init(&m, 15) && fake_tof_hz == 15 && tof_hz() == 15);
    f = poll();
    assert(f.period_us == 66666 && f.t_us == time_us_32() - 66666 / 2 - POLL_US / 2);
    assert(tof_init(&m, TOF_DEFAULT_HZ) && fake_tof_hz == 10);
    f = poll();
    assert(f.period_us == 100000 && f.t_us == time_us_32() - 100000 / 2 - POLL_US / 2);
    assert(!tof_init(&m, 16) && fake_tof_hz == 10 && tof_hz() == 10); // refused: unchanged
    fake_tof_refuse = true;
    assert(!tof_init(&m, 15) && tof_hz() == 10 && poll().period_us == 100000); // the sensor refuses: unchanged
    fake_tof_refuse = false;
    assert(fake_tof_inits == 1); // the firmware is uploaded once; a new rate only restarts it
    for (int k = 0; k < 20; k++) poll();
    assert(fabsf(tof_frames_per_s() - 10.0f) < 0.01f);
    uint32_t n = poll().number;
    assert(poll().number == n + 1 && poll().beyond_mm == TOF_EXACT);
    printf("ok: the rate a setting (10 Hz by default, 1-15); each frame its own period and middle time;"
           " refused: unchanged; firmware once; %.1f frames a second\n", (double)tof_frames_per_s());
}

static void zone_order(void) {
    mounting_tof_t m = ROBOT_MOUNTING.tof;
    assert(m.zones_turn_deg == 90);
    memset(&fake_tof, 0, sizeof fake_tof);
    for (int z = 0; z < 64; z++) target(z, 0, 100 + z, 5);
    // 90 as built: row 1 column 1 is the sensor's zone 56, its zone 0 the robot's top right.
    tof_frame_t f = poll();
    assert(f.zone[0].distance_mm == 156 && f.zone[7].distance_mm == 100 && f.zone[8].distance_mm == 157);
    assert(f.zone[63].distance_mm == 107);
    assert(sensor_zone(0) == 56 && sensor_zone(7) == 0 && sensor_zone(63) == 7);
    m.zones_turn_deg = 0;
    assert(tof_init(&m, 10));
    f = poll();
    assert(f.zone[0].distance_mm == 100 && f.zone[9].distance_mm == 109 && f.zone[63].distance_mm == 163);
    m.zones_turn_deg = 180;
    assert(tof_init(&m, 10));
    assert(sensor_zone(0) == 63 && sensor_zone(9) == 54 && poll().zone[0].distance_mm == 163);
    m.zones_turn_deg = 270; // the robot's (row, col) is the sensor's (col, 7 - row)
    assert(tof_init(&m, 10));
    assert(sensor_zone(0) == 7 && sensor_zone(7) == 63 && sensor_zone(63) == 56 && sensor_zone(9) == 14);
    assert(tof_init(&ROBOT_MOUNTING.tof, 10));
    printf("ok: the zones as the robot sees them, the grid turned 0, 90, 180, 270 deg\n");
}

static void one_target(void) {
    memset(&fake_tof, 0, sizeof fake_tof);
    // Sensor zone z is frame zone `at[z]` (90 as built).
    int at[64];
    for (int i = 0; i < 64; i++) at[sensor_zone(i)] = i;
    target(0, 0, 800, 5);                                      // sure
    target(1, 0, 810, 6);                                      // less sure (6)
    target(2, 0, 820, 9);                                      // less sure (9)
    target(3, 0, 290, 14); target(3, 1, 537, 9); target(3, 2, 1456, 5); // the echo first: 14 skipped, the 9 counts
    target(4, 0, 300, 12); target(4, 1, 700, 5);               // 12 (two surfaces) skipped
    target(5, 0, 310, 4); target(5, 1, 320, 13);               // none valid: unsure, the first
    target(6, 0, 0, 5); target(6, 1, 650, 5);                  // a valid status without a distance: skipped
    target(7, 0, 900, 5);                                      // signal beyond 65535: capped
    fake_tof.signal_per_spad[7 * T] = 100000;
    // zone 8: nothing at all
    for (int t = 0; t < 4; t++) target(9, t, 1000 + 100 * t, 5); // four targets: the first
    target(10, 0, 0, 5);                                       // the only target valid but without a distance: unsure, 0
    target(11, 0, -12, 4);                                     // unsure with a negative distance: 0
    target(12, 0, 950, 5);
    fake_tof.nb_target_detected[12] = 9;                       // the sensor says more than 4: 4
    tof_frame_t f = poll();
    const tof_zone_t *z = f.zone;
    assert(z[at[0]].kind == TOF_SURE && z[at[0]].distance_mm == 800 && z[at[0]].targets == 1);
    assert(z[at[0]].signal == 100 && z[at[0]].sigma_mm == 3);
    assert(z[at[1]].kind == TOF_LESS_SURE && z[at[1]].distance_mm == 810);
    assert(z[at[2]].kind == TOF_LESS_SURE && z[at[2]].distance_mm == 820);
    assert(z[at[3]].kind == TOF_LESS_SURE && z[at[3]].distance_mm == 537 && z[at[3]].targets == 3);
    assert(z[at[3]].signal == 110 && z[at[3]].sigma_mm == 4); // the chosen target's
    assert(z[at[4]].kind == TOF_SURE && z[at[4]].distance_mm == 700 && z[at[4]].targets == 2);
    assert(z[at[5]].kind == TOF_UNSURE && z[at[5]].distance_mm == 310 && z[at[5]].signal == 100 && z[at[5]].targets == 2);
    assert(z[at[6]].kind == TOF_SURE && z[at[6]].distance_mm == 650);
    assert(z[at[7]].signal == 65535);
    assert(z[at[8]].kind == TOF_NOTHING && z[at[8]].distance_mm == 0 && z[at[8]].targets == 0 && z[at[8]].signal == 0);
    assert(z[at[9]].kind == TOF_SURE && z[at[9]].distance_mm == 1000 && z[at[9]].targets == 4);
    assert(z[at[10]].kind == TOF_UNSURE && z[at[10]].distance_mm == 0 && z[at[10]].targets == 1);
    assert(z[at[11]].kind == TOF_UNSURE && z[at[11]].distance_mm == 0);
    assert(z[at[12]].kind == TOF_SURE && z[at[12]].distance_mm == 950 && z[at[12]].targets == 4);
    assert(tof_valid(&z[at[1]]) && !tof_valid(&z[at[5]]) && !tof_valid(&z[at[8]]) && !tof_beyond(&f, at[0]));
    printf("ok: one target per zone: the first with status 5, 6 or 9 (12, 14 skipped); unsure (also a valid status\n"
           "    without a distance); negative distances 0; nothing; targets seen, at most 4\n");
}

int main(void) {
    fake_now_us = FAKE_CLOCK_WRAP_US - 300000; // frames on both sides of the clock's wrap
    rate();
    zone_order();
    one_target();
    printf("tof: all tests passed\n");
    return 0;
}
