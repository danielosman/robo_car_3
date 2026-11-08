#pragma once
// The ToF's data as the robot passes it on (doc/DATA_FLOW_PLAN.md, the names):
// a tof_frame (one measurement of the 64 zones, made by `tof`), and a posed_frame
// (a tof_frame and where the robot was while it was measured, made by
// pose_stamp()). Plain data, no code: the robot's map and watch read posed frames,
// `tof_format` packs them, and the PC's helper will compile the same header.
//
// Zones are in row order as the robot sees them: zone = row index * 8 + column
// index, row 1 (the top) is index 0, column 1 (the left) is index 0 (README,
// conventions).
#include <stdbool.h>
#include <stdint.h>

#define TOF_ROWS  8
#define TOF_COLS  8
#define TOF_ZONES (TOF_ROWS * TOF_COLS)

// How sure the sensor is of a zone's target (its status numbers stay in tof.c).
typedef enum {
    TOF_NOTHING,   // no target in range
    TOF_UNSURE,    // targets, none valid (status other than 5, 6, 9): its distance tells nothing
    TOF_LESS_SURE, // valid with less confidence (status 6 or 9)
    TOF_SURE,      // valid (status 5)
} tof_kind_t;

// One zone: the first valid target (status 5, 6 or 9; the sensor gives the closest
// first), or, if none is valid, the first target (TOF_UNSURE). One target per zone:
// the others are dropped.
typedef struct {
    uint16_t distance_mm; // 0 for TOF_NOTHING
    uint16_t signal;      // kcps per SPAD, capped at 65535
    uint16_t sigma_mm;    // the distance's noise, capped at 65535
    uint8_t kind;         // tof_kind_t
    uint8_t targets;      // how many targets the sensor saw in the zone (0-4)
} tof_zone_t;

typedef struct {
    uint32_t number;    // counts every frame the sensor gave since power-up, from 1
    uint32_t t_us;      // PicoA's clock (time_us_32()), the middle of the measurement
    uint32_t period_us; // how long it measured (t_us ± half of it): the frame's own, never a global rate
    // A distance this far or farther is only known to be at least this far (a packed
    // frame's limit: tof_format); TOF_EXACT in a frame straight from the sensor.
    uint16_t beyond_mm;
    tof_zone_t zone[TOF_ZONES];
} tof_frame_t;

#define TOF_EXACT 0xFFFF

static inline bool tof_valid(const tof_zone_t *z) { return z->kind == TOF_SURE || z->kind == TOF_LESS_SURE; }
// A valid zone whose distance is only a lower bound.
static inline bool tof_beyond(const tof_frame_t *f, int zone) {
    return tof_valid(&f->zone[zone]) && f->zone[zone].distance_mm >= f->beyond_mm;
}

// A frame and where the robot was while it was measured, in the map's frame
// (odometry's, from where PicoB started).
typedef struct {
    tof_frame_t frame;
    float x_m, y_m;  // at the middle of the measurement
    float yaw_rad;   // + = left, wrapped to ±π
    float pitch_rad; // + = nose up
    float turn_rad;  // how far it turned during the measurement (+ = left)
    bool still;      // PicoB said it stood still
    bool pose_known; // false: no odometry for that moment (PicoB not connected); the pose is 0
} posed_frame_t;
