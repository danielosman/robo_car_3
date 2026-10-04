#pragma once
// VL53L8CX on PicoA SPI0 (GP16/17/18/19). No LPn GPIO: the Pololu carrier holds it
// high. All functions run on the main core; only tof_init() takes long (~1-2 s:
// it uploads the sensor firmware). Problems are printed on stdout.
#include <stdbool.h>
#include <stdint.h>
#include "vl53l8cx_api.h"

typedef struct {
    uint8_t zones;          // 16 (4x4) or 64 (8x8)
    uint8_t hz;             // 1-15 at 8x8, 1-60 at 4x4
    uint8_t order;          // 1 = closest target first, 2 = strongest
    uint8_t mode;           // 1 = continuous, 3 = autonomous
    uint16_t integration_ms; // autonomous mode only; must fit the period
    uint8_t sharpener;      // percent
} tof_settings_t;

bool tof_init(void);                      // finds the sensor and loads its firmware; doesn't start ranging
bool tof_start(const tof_settings_t *s);  // (re)configures and starts ranging; false if invalid or refused
bool tof_stop(void);
// The newest measurement if one arrived since the last call, else NULL. Three read
// failures in a row stop ranging: tof_init() and tof_start() again to recover.
const VL53L8CX_ResultsData *tof_poll(void);
const tof_settings_t *tof_settings(void); // active settings, sharpener as the sensor reports it
bool tof_running(void);
