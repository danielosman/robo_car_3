#pragma once
// Fake VL53L8CX driver: the test fills fake_tof (in the sensor's own zone order)
// and sets fake_tof_fresh; vl53_poll() returns it once. Polling before vl53_start()
// fails the test.
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define VL53L8CX_NB_TARGET_PER_ZONE 4U
typedef struct { // the ULD's outputs the app uses, with the ULD's types
    int8_t silicon_temp_degc;
    uint32_t ambient_per_spad[64];
    uint8_t nb_target_detected[64];
    uint32_t nb_spads_enabled[64];
    uint32_t signal_per_spad[64 * VL53L8CX_NB_TARGET_PER_ZONE];
    uint16_t range_sigma_mm[64 * VL53L8CX_NB_TARGET_PER_ZONE];
    int16_t distance_mm[64 * VL53L8CX_NB_TARGET_PER_ZONE];
    uint8_t reflectance[64 * VL53L8CX_NB_TARGET_PER_ZONE];
    uint8_t target_status[64 * VL53L8CX_NB_TARGET_PER_ZONE];
} VL53L8CX_ResultsData;
typedef struct { uint8_t zones, hz, order, mode; uint16_t integration_ms; uint8_t sharpener; } vl53_settings_t;
static VL53L8CX_ResultsData fake_tof;
static bool fake_tof_fresh, fake_tof_started;
static uint8_t fake_tof_hz; // the rate it was last started at
static int fake_tof_inits;  // vl53_init() calls (each uploads the firmware: ~2 s)
static bool fake_tof_refuse; // vl53_start() fails (the sensor refuses)
static inline bool vl53_init(void) { fake_tof_inits++; return true; }
static inline bool vl53_start(const vl53_settings_t *s) {
    assert(s->zones == 64 && s->hz >= 1 && s->hz <= 15); // the real one refuses others
    if (fake_tof_refuse) return false;
    fake_tof_started = true;
    fake_tof_hz = s->hz;
    return true;
}
static inline const VL53L8CX_ResultsData *vl53_poll(void) {
    assert(fake_tof_started);
    if (!fake_tof_fresh) return 0;
    fake_tof_fresh = false;
    return &fake_tof;
}
