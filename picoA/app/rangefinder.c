#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "tof.h"
#include "units.h"
#include "rangefinder.h"

#define HZ               15
#define POLL_US          5000      // tof_poll() checks for data this often
#define ZONE_RAD         (5.625f * RAD_PER_DEG) // 45° field of view over 8 zones
#define SENSOR_X_M       0.025f    // ahead of the centre
#define SENSOR_Y_M       (-0.03f)  // right of the centre line
#define SENSOR_Z_M       0.07f     // above the floor
#define FIRST_FLOOR_ROW  4         // rows 4-7 see the floor
#define HORIZON_ROW      4         // its floor is far and grazing: 1° of pitch moves it a lot
#define HORIZON_MAX_HIT_M 0.95f    // so only closer hits on it count as obstacles
#define OBSTACLE_MIN_M   0.02f     // lower things are driven over (§4.2)
#define NO_TARGET_CLEAR_M 1.0f     // "saw nothing" is only trusted this far (§4.5)
#define EDGE_FLOOR_SHARE 0.8f      // above the floor rows: a reading this close to where the zone's
                                   // lower edge meets the floor may be the floor, not an obstacle
#define FLOOR_ZONES      ((RANGEFINDER_ROWS - FIRST_FLOOR_ROW) * RANGEFINDER_COLS)
#define FLOOR_SAMPLES    256       // per zone: more than a 390° turn at 15 Hz gives
#define FLOOR_MIN_SAMPLES 20
#define FLOOR_PERCENTILE 0.75f     // the floor wins unless obstacles fill 3/4 of the turn
#define FLOOR_WINDOW     0.1f      // readings within 10 % of that are the floor's scatter
#define SCATTER_MARGIN_M 0.01f     // above the floor's own scatter, before a reading is an obstacle
#define FLOOR_MAX_SHARE  1.25f     // of where the zone's centre meets a flat floor: slack for tilt

// Each zone's ray with the robot level: how far below horizontal (rad) and how far left.
static float zone_down_rad[RANGEFINDER_RAYS], zone_left_rad[RANGEFINDER_RAYS];
static bool have_zones;
// Learned per floor zone. A zone is 5.6° tall and reports the nearest surface in it,
// so where it sees the floor depends on the zone and the floor; the angle at which
// a ray from the sensor's height meets the floor there is the zone's effective
// angle, and obstacles are judged along it. 0 = not learned.
static float floor_down_rad[FLOOR_ZONES];
static float obstacle_min_m[FLOOR_ZONES]; // how far above the floor a reading must end to be an obstacle
static float samples[FLOOR_ZONES][FLOOR_SAMPLES]; // each reading as a floor distance with the robot level, m
static int n_samples[FLOOR_ZONES];

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
    tof_settings_t s = {.zones = 64, .hz = HZ, .order = 1, .mode = 1, .integration_ms = 5, .sharpener = 5};
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

bool rangefinder_poll(range_frame_t *frame) {
    const VL53L8CX_ResultsData *r = tof_poll();
    if (!r) return false;
    // Data is read up to POLL_US after it's ready; the measurement took the whole period.
    frame->t_us = time_us_32() - 1000000u / HZ / 2 - POLL_US / 2;
    for (int row = 0; row < RANGEFINDER_ROWS; row++)
        for (int col = 0; col < RANGEFINDER_COLS; col++) {
            // The sensor is turned 90° on the PCB (same as the bring-up viewer's rotation).
            int zone = (RANGEFINDER_COLS - 1 - col) * RANGEFINDER_ROWS + row;
            frame->range_mm[row * RANGEFINDER_COLS + col] = closest_sure_mm(r, zone);
            frame->status[row * RANGEFINDER_COLS + col] = r->target_status[zone * VL53L8CX_NB_TARGET_PER_ZONE];
        }
    return true;
}

static int floor_index(int ray) { return ray - FIRST_FLOOR_ROW * RANGEFINDER_COLS; }

// Just above the floor rows, a zone's lower edge can still reach the floor far
// away (the robot sits a little nose-down, and nods): such a reading may be the floor.
static bool maybe_floor(int row, float pitch_rad, float range_m) {
    float edge_down_rad = ((float)row - 3.0f) * ZONE_RAD - pitch_rad;
    if (edge_down_rad <= 0) return false;
    return range_m >= EDGE_FLOOR_SHARE * SENSOR_Z_M / sinf(edge_down_rad);
}

void rangefinder_scan(const range_frame_t *frame, float pitch_rad, scan_t *scan) {
    if (!have_zones) make_zones();
    scan->origin_x_m = SENSOR_X_M;
    scan->origin_y_m = SENSOR_Y_M;
    scan->origin_z_m = SENSOR_Z_M;
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        int row = i / RANGEFINDER_COLS;
        // The 5th row's floor is often not learned (in a room it mostly sees walls):
        // then it's used like the rows above it.
        bool floor_row = row >= FIRST_FLOOR_ROW && !(row == HORIZON_ROW && floor_down_rad[floor_index(i)] == 0);
        ray_t *ray = &scan->ray[i];
        uint16_t mm = frame->range_mm[i];
        ray->kind = RAY_UNUSED;
        if (mm == RANGE_INVALID) continue;
        if (floor_row && (mm == RANGE_NO_TARGET || floor_down_rad[floor_index(i)] == 0)) continue;
        if (row == HORIZON_ROW && !floor_row && mm == RANGE_NO_TARGET) continue; // its floor may be near: no "1 m clear"
        float range_m = mm == RANGE_NO_TARGET ? NO_TARGET_CLEAR_M : (float)mm * 0.001f;
        float d[3];
        direction(floor_row ? floor_down_rad[floor_index(i)] : zone_down_rad[i], zone_left_rad[i], pitch_rad, d);
        ray->x_m = SENSOR_X_M + range_m * d[0];
        ray->y_m = SENSOR_Y_M + range_m * d[1];
        ray->z_m = SENSOR_Z_M + range_m * d[2];
        float min_height_m = floor_row ? obstacle_min_m[floor_index(i)] : OBSTACLE_MIN_M;
        bool obstacle = mm != RANGE_NO_TARGET && ray->z_m >= min_height_m &&
                        (row != HORIZON_ROW || range_m < HORIZON_MAX_HIT_M) &&
                        (floor_row || !maybe_floor(row, pitch_rad, range_m));
        ray->kind = obstacle ? RAY_HIT : RAY_CLEAR;
    }
}

void rangefinder_forget_floor(void) {
    for (int z = 0; z < FLOOR_ZONES; z++) floor_down_rad[z] = 0, n_samples[z] = 0;
}

void rangefinder_learn_floor(const range_frame_t *frame, float pitch_rad) {
    if (!have_zones) make_zones();
    for (int z = 0; z < FLOOR_ZONES; z++) {
        int i = z + FIRST_FLOOR_ROW * RANGEFINDER_COLS;
        uint16_t mm = frame->range_mm[i];
        if (mm == RANGE_NO_TARGET || mm == RANGE_INVALID || n_samples[z] == FLOOR_SAMPLES) continue;
        // Where this reading would be with the robot level: the angle at which a ray
        // from the sensor's height meets a floor this far away, plus the pitch.
        float range_m = (float)mm * 0.001f;
        if (range_m <= SENSOR_Z_M) continue; // can't be the floor
        float level_down = asinf(SENSOR_Z_M / range_m) + pitch_rad;
        if (level_down <= 0) continue;
        samples[z][n_samples[z]++] = SENSOR_Z_M / sinf(level_down);
    }
}

static int compare_floats(const void *a, const void *b) {
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

int rangefinder_finish_floor(void) {
    int learned = 0;
    for (int z = 0; z < FLOOR_ZONES; z++) {
        floor_down_rad[z] = 0;
        int n = n_samples[z];
        if (n < FLOOR_MIN_SAMPLES) continue;
        float *v = samples[z];
        qsort(v, (size_t)n, sizeof v[0], compare_floats);
        // The floor's readings: around the long, common distance (obstacles only make
        // readings shorter, so they're below it). Their middle is the floor; how far
        // their short end reaches is the zone's own scatter.
        float r75 = v[(int)(FLOOR_PERCENTILE * (float)(n - 1))];
        int lo = 0, hi = n;
        while (lo < n && v[lo] < r75 * (1 - FLOOR_WINDOW)) lo++;
        while (hi > lo && v[hi - 1] > r75 * (1 + FLOOR_WINDOW)) hi--;
        float floor_m = v[lo + (hi - lo) / 2], short_m = v[lo + (hi - lo) / 10];
        if (floor_m <= SENSOR_Z_M) continue;
        float down = asinf(SENSOR_Z_M / floor_m), nominal = zone_down_rad[z + FIRST_FLOOR_ROW * RANGEFINDER_COLS];
        // A zone reports the nearest surface, so it sees the floor no farther than its
        // centre would (some slack for tilt), and not below its lower edge.
        if (floor_m > FLOOR_MAX_SHARE * SENSOR_Z_M / sinf(nominal) || down > nominal + ZONE_RAD) continue;
        floor_down_rad[z] = down;
        obstacle_min_m[z] = fmaxf(OBSTACLE_MIN_M, (floor_m - short_m) * sinf(down) + SCATTER_MARGIN_M);
        learned++;
    }
    return learned;
}

float rangefinder_floor_distance(int row) {
    float sum_m = 0;
    int n = 0;
    for (int col = 0; col < RANGEFINDER_COLS; col++) {
        int z = floor_index(row * RANGEFINDER_COLS + col);
        if (row < FIRST_FLOOR_ROW || floor_down_rad[z] == 0) continue;
        sum_m += SENSOR_Z_M / sinf(floor_down_rad[z]);
        n++;
    }
    return n ? sum_m / (float)n : 0;
}

bool rangefinder_zone_floor(int ray, float *floor_m, float *obstacle_min_height_m) {
    if (ray < FIRST_FLOOR_ROW * RANGEFINDER_COLS || ray >= RANGEFINDER_RAYS) return false;
    int z = floor_index(ray);
    if (floor_down_rad[z] == 0) return false;
    *floor_m = SENSOR_Z_M / sinf(floor_down_rad[z]);
    *obstacle_min_height_m = obstacle_min_m[z];
    return true;
}

int rangefinder_floor_zones(void) { return FLOOR_ZONES; }

int rangefinder_first_floor_row(void) { return FIRST_FLOOR_ROW; }
