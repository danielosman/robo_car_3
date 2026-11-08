#pragma once
// The VL53L8CX as frames (doc/ARCHITECTURE.md, the ToF pipeline): the only reader
// of the sensor's results (drivers/vl53.h, ST's ULD) and its status numbers. Hides
// the sensor's settings, its zone order (the zone grid is turned on the PCB: the
// mounting says how), which target of a zone counts (one: the first with status 5,
// 6 or 9) and how a frame is timed. Where the zones point is tof_rays'.
#include <stdbool.h>
#include <stdint.h>
#include "mounting.h"
#include "tof_frame.h"

#define TOF_DEFAULT_HZ 10 // the rate main.c starts it at
#define TOF_MAX_HZ     15 // at 8 x 8 zones

// Starts the sensor at hz frames a second (1-TOF_MAX_HZ), the zones turned as the
// mounting says (main.c: ROBOT_MOUNTING.tof). The first call uploads the sensor's
// firmware (~2 s); later ones only restart it, e.g. at a new rate. False if it
// doesn't answer or refuses, or hz or the mounting is wrong: then the active rate
// and mounting stay.
bool tof_init(const mounting_tof_t *m, int hz);
int tof_hz(void); // the active rate; 0 before a start
// Frames read a second over the last 16 (status: the rate the sensor really gives);
// 0 before two.
float tof_frames_per_s(void);
// True when a new frame was read into *frame (each frame carries its own time and
// period).
bool tof_poll(tof_frame_t *frame);
