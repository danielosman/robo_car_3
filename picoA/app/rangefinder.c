#include <math.h>
#include "pico/stdlib.h"
#include "tof.h"
#include "units.h"
#include "rangefinder.h"

#define POLL_US          5000      // tof_poll() checks for data this often
#define ZONE_RAD         (5.625f * RAD_PER_DEG) // 45° field of view over 8 zones
#define SENSOR_X_M       0.025f    // ahead of the centre
#define SENSOR_Y_M       (-0.03f)  // right of the centre line
#define SENSOR_Z_M       0.07f     // above the floor
#define FIRST_FLOOR_ROW  4         // rows 4-7 see the floor
#define DROP_SHARE       1.15f     // a floor zone's reading this much farther than where its upper
                                   // edge meets a flat floor is a reflection (or a drop)

// Each zone's ray with the robot level: how far below horizontal (rad) and how far left.
static float zone_down_rad[RANGEFINDER_RAYS], zone_left_rad[RANGEFINDER_RAYS];
static bool have_zones;
static const VL53L8CX_ResultsData *raw; // the sensor's data behind the latest frame
static uint32_t frames_read;

static void make_zones(void) {
    for (int row = 0; row < RANGEFINDER_ROWS; row++)
        for (int col = 0; col < RANGEFINDER_COLS; col++) {
            zone_down_rad[row * RANGEFINDER_COLS + col] = ((float)row - 3.5f) * ZONE_RAD;
            zone_left_rad[row * RANGEFINDER_COLS + col] = (3.5f - (float)col) * ZONE_RAD;
        }
    have_zones = true;
}

// Unit direction (x forward, y left, z up) of a ray down_rad below horizontal and
// left_rad to the left, with the robot pitched nose up by pitch_rad.
static void direction(float down_rad, float left_rad, float pitch_rad, float out[3]) {
    float a = down_rad - pitch_rad;
    out[0] = cosf(a) * cosf(left_rad);
    out[1] = cosf(a) * sinf(left_rad);
    out[2] = -sinf(a);
}

bool rangefinder_init(void) {
    if (!have_zones) make_zones();
    tof_settings_t s = {.zones = 64, .hz = RANGEFINDER_HZ, .order = 1, .mode = 1, .integration_ms = 5, .sharpener = 5};
    return tof_init() && tof_start(&s);
}

// Statuses the ULD marks as a valid range (5) or valid with lower confidence (6, 9).
static bool valid_status(uint8_t status) { return status == 5 || status == 6 || status == 9; }

// The closest sure target of a zone (targets come closest first).
static uint16_t closest_sure_mm(const VL53L8CX_ResultsData *r, int zone) {
    int n = r->nb_target_detected[zone];
    if (n == 0) return RANGE_NO_TARGET;
    if (n > (int)VL53L8CX_NB_TARGET_PER_ZONE) n = VL53L8CX_NB_TARGET_PER_ZONE;
    for (int t = 0; t < n; t++) {
        int i = zone * (int)VL53L8CX_NB_TARGET_PER_ZONE + t;
        if (valid_status(r->target_status[i]) && r->distance_mm[i] > 0)
            return (uint16_t)r->distance_mm[i]; // int16: at most 32767, never RANGE_INVALID
    }
    return RANGE_INVALID;
}

// The sensor is turned 90° on the PCB (same as the bring-up viewer's rotation).
int rangefinder_zone(int ray) {
    return (RANGEFINDER_COLS - 1 - ray % RANGEFINDER_COLS) * RANGEFINDER_ROWS + ray / RANGEFINDER_COLS;
}

float rangefinder_zone_rad(void) { return ZONE_RAD; }

// All of the sensor's outputs for the latest frame, in its own zone order, until the
// next rangefinder_poll(); NULL before the first. For recorder.c, which declares it
// itself: the type is tof.h's, which rangefinder.h's users don't include.
const VL53L8CX_ResultsData *rangefinder_raw(void) { return raw; }

bool rangefinder_poll(range_frame_t *frame) {
    const VL53L8CX_ResultsData *r = tof_poll();
    if (!r) return false;
    raw = r;
    frame->number = ++frames_read;
    // Data is read up to POLL_US after it's ready; the measurement took the whole period.
    frame->t_us = time_us_32() - 1000000u / RANGEFINDER_HZ / 2 - POLL_US / 2;
    for (int row = 0; row < RANGEFINDER_ROWS; row++)
        for (int col = 0; col < RANGEFINDER_COLS; col++) {
            int i = row * RANGEFINDER_COLS + col, zone = rangefinder_zone(i);
            frame->range_mm[i] = closest_sure_mm(r, zone);
            frame->status[i] = r->target_status[zone * VL53L8CX_NB_TARGET_PER_ZONE];
        }
    return true;
}

void rangefinder_origin(float origin[3]) {
    origin[0] = SENSOR_X_M;
    origin[1] = SENSOR_Y_M;
    origin[2] = SENSOR_Z_M;
}

void rangefinder_ray_direction(int ray, float dir[3]) {
    if (!have_zones) make_zones();
    direction(zone_down_rad[ray], zone_left_rad[ray], 0, dir);
}

void rangefinder_sub_ray_direction(int ray, float down_frac, float left_frac, float pitch_rad, float dir[3]) {
    if (!have_zones) make_zones();
    direction(zone_down_rad[ray] + down_frac * ZONE_RAD, zone_left_rad[ray] + left_frac * ZONE_RAD, pitch_rad, dir);
}

// A floor seen down_rad below horizontal from the sensor's height: how far along the ray.
static float floor_range_m(float down_rad) { return SENSOR_Z_M / sinf(down_rad); }

int rangefinder_first_floor_row(void) { return FIRST_FLOOR_ROW; }

float rangefinder_floor_limit_m(int ray) {
    if (!have_zones) make_zones();
    float far_down_rad = zone_down_rad[ray] - ZONE_RAD / 2; // where its upper edge meets the floor
    if (ray / RANGEFINDER_COLS < FIRST_FLOOR_ROW || far_down_rad <= 0) return 0;
    return DROP_SHARE * floor_range_m(far_down_rad);
}
