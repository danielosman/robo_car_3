// Host test for picoA/app/rangefinder.c: the zone order, the frame's time, unsure
// and missing targets, the floor limit of the floor rows, against a fake VL53L8CX.
// Run from the repo root:
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_rangefinder picoA/app/test/test_rangefinder.c -lm && build/test_rangefinder
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../rangefinder.c"

// The fake sensor reports `mm` for the zone the robot sees at (row, col).
static void set_zone(int row, int col, int mm) {
    int zone = (7 - col) * 8 + row, first = zone * (int)VL53L8CX_NB_TARGET_PER_ZONE;
    fake_tof.nb_target_detected[zone] = mm > 0;
    fake_tof.distance_mm[first] = (int16_t)mm;
    fake_tof.target_status[first] = 5;
}

int main(void) {
    range_frame_t f;
    fake_now_us = FAKE_CLOCK_WRAP_US + 10000; // the frame's time is before the wrap
    assert(rangefinder_init());
    assert(!rangefinder_poll(&f));

    // Zone order: the sensor's zones arrive turned 90°; the frame is as the robot sees it.
    for (int row = 0; row < 8; row++)
        for (int col = 0; col < 8; col++) set_zone(row, col, 100 + row * 10 + col);
    fake_tof_fresh = true;
    assert(rangefinder_poll(&f));
    assert(f.range_mm[0] == 100 && f.range_mm[7] == 107 && f.range_mm[8 * 7] == 170 && f.range_mm[63] == 177);
    assert(f.t_us == (uint32_t)(FAKE_CLOCK_WRAP_US + 10000 - 1000000 / 15 / 2 - 2500));
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

    // The floor rows' limit: beyond where a zone's upper edge meets a flat floor 7 cm
    // below the sensor (+15 %), a reading is a reflection; the rows above have none.
    float row6_m = rangefinder_floor_limit_m(6 * 8 + 3), row4_m = rangefinder_floor_limit_m(4 * 8 + 3);
    printf("floor limits: row 7 %.0f cm, row 5 %.0f cm, row 4 %.0f\n", (double)(row6_m * 100), (double)(row4_m * 100),
           (double)rangefinder_floor_limit_m(3 * 8 + 3));
    assert(fabsf(row6_m - 1.15f * 0.07f / sinf(11.25f * RAD_PER_DEG)) < 0.001f);
    assert(row4_m == 0 && rangefinder_floor_limit_m(3 * 8 + 3) == 0); // row 4's upper edge is level
    printf("rangefinder: all tests passed\n");
    return 0;
}
