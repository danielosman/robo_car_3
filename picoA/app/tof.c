#include "pico/stdlib.h"
#include "vl53.h"
#include "tof.h"

#define POLL_US    5000 // vl53_poll() checks for data this often
#define RATE_FRAMES 16  // tof_frames_per_s() over the last this many

static int turns;       // quarter turns of the zone grid on the PCB (the mounting's)
static bool loaded;     // the sensor's firmware is uploaded (vl53_init(), once)
static int active_hz;
static uint32_t period_us;
static uint32_t frames_read;
static uint32_t read_us[RATE_FRAMES]; // when each was read, by frame number

bool tof_init(const mounting_tof_t *m, int hz) {
    if (m->zones_turn_deg < 0 || m->zones_turn_deg % 90 != 0 || m->zones_turn_deg >= 360) return false;
    if (hz < 1 || hz > TOF_MAX_HZ) return false;
    vl53_settings_t s = {.zones = 64, .hz = (uint8_t)hz, .order = 1, .mode = 1, .integration_ms = 5, .sharpener = 5};
    if (!loaded && !vl53_init()) return false;
    loaded = true;
    if (!vl53_start(&s)) return false; // stops and restarts it at the new rate
    turns = m->zones_turn_deg / 90;
    active_hz = hz;
    period_us = 1000000u / (uint32_t)hz;
    return true;
}

int tof_hz(void) { return active_hz; }

float tof_frames_per_s(void) {
    if (frames_read < 2) return 0;
    uint32_t n = frames_read < RATE_FRAMES ? frames_read : RATE_FRAMES;
    uint32_t span_us = read_us[frames_read % RATE_FRAMES] - read_us[(frames_read - n + 1) % RATE_FRAMES];
    return span_us ? (float)(n - 1) * 1e6f / (float)span_us : 0;
}

// Each quarter turn of the grid on the PCB: the robot's (row, col) is the sensor's
// (7 - col, row). 90 as built: row 1 column 1 is the sensor's zone 56.
static int sensor_zone(int zone) {
    int row = zone / TOF_COLS, col = zone % TOF_COLS;
    for (int q = 0; q < turns; q++) {
        int r = TOF_ROWS - 1 - col;
        col = row;
        row = r;
    }
    return row * TOF_COLS + col;
}

static uint16_t capped16(uint32_t v) { return v > 0xFFFFu ? 0xFFFFu : (uint16_t)v; }

// Statuses the ULD marks as valid (5) or valid with lower confidence (6, 9).
static bool valid_status(uint8_t s) { return s == 5 || s == 6 || s == 9; }

// One zone: the first valid target with a distance (the sensor gives the closest
// first); none valid: the first, unsure; none at all: nothing.
static tof_zone_t read_zone(const VL53L8CX_ResultsData *r, int sz) {
    int n = r->nb_target_detected[sz];
    if (n > (int)VL53L8CX_NB_TARGET_PER_ZONE) n = VL53L8CX_NB_TARGET_PER_ZONE;
    tof_zone_t z = {.targets = (uint8_t)n, .kind = TOF_NOTHING};
    if (n == 0) return z;
    const int first = sz * (int)VL53L8CX_NB_TARGET_PER_ZONE;
    int t = 0;
    while (t < n && !(valid_status(r->target_status[first + t]) && r->distance_mm[first + t] > 0)) t++;
    z.kind = t == n ? TOF_UNSURE : r->target_status[first + t] == 5 ? TOF_SURE : TOF_LESS_SURE;
    if (t == n) t = 0;
    int i = first + t;
    z.distance_mm = r->distance_mm[i] > 0 ? (uint16_t)r->distance_mm[i] : 0;
    z.signal = capped16(r->signal_per_spad[i]);
    z.sigma_mm = r->range_sigma_mm[i];
    return z;
}

bool tof_poll(tof_frame_t *frame) {
    const VL53L8CX_ResultsData *r = vl53_poll();
    if (!r) return false;
    uint32_t now = time_us_32();
    frame->number = ++frames_read;
    read_us[frames_read % RATE_FRAMES] = now;
    // Data is read up to POLL_US after it's ready; the measurement took the whole period.
    frame->t_us = now - period_us / 2 - POLL_US / 2;
    frame->period_us = period_us;
    frame->beyond_mm = TOF_EXACT;
    for (int zone = 0; zone < TOF_ZONES; zone++) frame->zone[zone] = read_zone(r, sensor_zone(zone));
    return true;
}
