#pragma once
// Fake VL53L8CX driver: the test fills fake_tof (in the sensor's own zone order)
// and sets fake_tof_fresh; tof_poll() returns it once. Polling before tof_start()
// fails the test.
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define VL53L8CX_NB_TARGET_PER_ZONE 4U
typedef struct {
    uint8_t nb_target_detected[64];
    int16_t distance_mm[64 * VL53L8CX_NB_TARGET_PER_ZONE];
    uint8_t target_status[64 * VL53L8CX_NB_TARGET_PER_ZONE];
} VL53L8CX_ResultsData;
typedef struct { uint8_t zones, hz, order, mode; uint16_t integration_ms; uint8_t sharpener; } tof_settings_t;
static VL53L8CX_ResultsData fake_tof;
static bool fake_tof_fresh, fake_tof_started;
static inline bool tof_init(void) { return true; }
static inline bool tof_start(const tof_settings_t *s) { assert(s->zones == 64); fake_tof_started = true; return true; }
static inline const VL53L8CX_ResultsData *tof_poll(void) {
    assert(fake_tof_started);
    if (!fake_tof_fresh) return 0;
    fake_tof_fresh = false;
    return &fake_tof;
}
