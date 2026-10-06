#include <math.h>
#include <stdlib.h>
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
#define HORIZON_ROW      4         // its floor is far and grazing: 1° of pitch moves it a lot
#define HORIZON_MAX_HIT_M 0.95f    // so only closer hits on it count as obstacles
#define FIRST_DROP_ROW   6         // rows 6-7 (7th, 8th) tell a drop: their floor is near and steady
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
#define FLOOR_SLACK_RAD  (2 * RAD_PER_DEG) // a zone sees the floor between its edges, give or take this
#define DROP_SHARE       1.15f     // rows 5-7: a reading this much farther than the far end of the
                                   // zone's floor patch (where its upper edge meets the floor) may be a drop

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

bool rangefinder_poll(range_frame_t *frame) {
    const VL53L8CX_ResultsData *r = tof_poll();
    if (!r) return false;
    // Data is read up to POLL_US after it's ready; the measurement took the whole period.
    frame->t_us = time_us_32() - 1000000u / RANGEFINDER_HZ / 2 - POLL_US / 2;
    for (int row = 0; row < RANGEFINDER_ROWS; row++)
        for (int col = 0; col < RANGEFINDER_COLS; col++) {
            // The sensor is turned 90° on the PCB (same as the bring-up viewer's rotation).
            int zone = (RANGEFINDER_COLS - 1 - col) * RANGEFINDER_ROWS + row, i = row * RANGEFINDER_COLS + col;
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

static int floor_index(int ray) { return ray - FIRST_FLOOR_ROW * RANGEFINDER_COLS; }
static bool floor_learned(int ray) { return floor_down_rad[floor_index(ray)] != 0; }

// A floor seen down_rad below horizontal from the sensor's height: how far along the ray, and back.
static float floor_range_m(float down_rad) { return SENSOR_Z_M / sinf(down_rad); }
static float floor_angle_rad(float range_m) { return asinf(SENSOR_Z_M / range_m); }

// Just above the floor rows, a zone's lower edge can still reach the floor far
// away (the robot sits a little nose-down, and nods): such a reading may be the floor.
static bool maybe_floor(int ray, float pitch_rad, float range_m) {
    float edge_down_rad = zone_down_rad[ray] + ZONE_RAD / 2 - pitch_rad;
    if (edge_down_rad <= 0) return false;
    return range_m >= EDGE_FLOOR_SHARE * floor_range_m(edge_down_rad);
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
        bool floor_row = row >= FIRST_FLOOR_ROW && !(row == HORIZON_ROW && !floor_learned(i));
        // Below the 5th row the floor is near and always seen, unless it isn't there.
        bool must_see_floor = row > HORIZON_ROW && floor_learned(i);
        // The 6th row's floor patch is long (35-71 cm) and grazing: on a shiny floor
        // it sometimes reflects far away. Only the two lowest rows tell a drop.
        bool tells_drop = must_see_floor && row >= FIRST_DROP_ROW;
        ray_t *ray = &scan->ray[i];
        uint16_t mm = frame->range_mm[i];
        ray->kind = RAY_UNUSED;
        if (mm == RANGE_INVALID) continue;
        // A zone sees a patch of floor, 5.6° tall: any reading up to its far end is the
        // floor. (Nose up so far that the ray doesn't reach the floor: it tells nothing.)
        if (must_see_floor) {
            float down_rad = floor_down_rad[floor_index(i)];
            if (down_rad - pitch_rad <= 0) continue;
            float floor_m = floor_range_m(down_rad - pitch_rad);
            // The 6th row's far end grazes the floor: nose up 3°, it would reach 1.6 m.
            // Its far readings are often reflections, so nose up doesn't stretch it.
            float far_down_rad = zone_down_rad[i] - ZONE_RAD / 2 - (tells_drop ? pitch_rad : fminf(pitch_rad, 0));
            bool beyond_floor = far_down_rad > 0 && (float)mm * 0.001f > DROP_SHARE * floor_range_m(far_down_rad);
            if (mm == RANGE_NO_TARGET || beyond_floor) {
                if (!tells_drop) continue; // the 6th row: tells nothing
                float d[3];
                direction(down_rad, zone_left_rad[i], pitch_rad, d);
                ray->x_m = SENSOR_X_M + floor_m * d[0];
                ray->y_m = SENSOR_Y_M + floor_m * d[1];
                ray->z_m = SENSOR_Z_M + floor_m * d[2];
                ray->kind = RAY_NO_FLOOR;
                continue;
            }
        }
        if (floor_row && (mm == RANGE_NO_TARGET || !floor_learned(i))) continue;
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
                        (floor_row || !maybe_floor(i, pitch_rad, range_m));
        ray->kind = obstacle ? RAY_HIT : must_see_floor ? RAY_FLOOR : RAY_CLEAR;
        if (ray->kind == RAY_FLOOR) {
            float near_down_rad = zone_down_rad[i] + ZONE_RAD / 2 - pitch_rad;
            float near_m = SENSOR_Z_M / tanf(near_down_rad); // along the floor
            ray->floor_from_x_m = SENSOR_X_M + near_m * cosf(zone_left_rad[i]);
            ray->floor_from_y_m = SENSOR_Y_M + near_m * sinf(zone_left_rad[i]);
        }
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
        float level_down = floor_angle_rad(range_m) + pitch_rad;
        if (level_down <= 0) continue;
        // Only readings inside the zone's patch of floor can be the floor: shorter is
        // an obstacle, longer a drop (the robot may learn next to a table's edge).
        if (fabsf(level_down - zone_down_rad[i]) > ZONE_RAD / 2 + FLOOR_SLACK_RAD) continue;
        samples[z][n_samples[z]++] = floor_range_m(level_down);
    }
}

static int compare_floats(const void *a, const void *b) {
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

int rangefinder_finish_floor(int *assumed) {
    int learned = 0;
    *assumed = 0;
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
        float down = floor_angle_rad(floor_m), nominal = zone_down_rad[z + FIRST_FLOOR_ROW * RANGEFINDER_COLS];
        // A zone reports the nearest surface, so it sees the floor no farther than its
        // centre would (some slack for tilt), and not below its lower edge.
        if (floor_m > FLOOR_MAX_SHARE * floor_range_m(nominal) || down > nominal + ZONE_RAD) continue;
        floor_down_rad[z] = down;
        obstacle_min_m[z] = fmaxf(OBSTACLE_MIN_M, (floor_m - short_m) * sinf(down) + SCATTER_MARGIN_M);
        learned++;
    }
    // The two lowest rows tell drops, so they always need a floor: where none was
    // learned, the floor a zone's centre would see from the sensor's height.
    for (int z = (FIRST_DROP_ROW - FIRST_FLOOR_ROW) * RANGEFINDER_COLS; z < FLOOR_ZONES; z++) {
        if (floor_down_rad[z] != 0) continue;
        floor_down_rad[z] = zone_down_rad[z + FIRST_FLOOR_ROW * RANGEFINDER_COLS];
        obstacle_min_m[z] = OBSTACLE_MIN_M + SCATTER_MARGIN_M;
        (*assumed)++;
    }
    return learned;
}

float rangefinder_floor_distance(int row) {
    float sum_m = 0;
    int n = 0;
    for (int col = 0; col < RANGEFINDER_COLS; col++) {
        int z = floor_index(row * RANGEFINDER_COLS + col);
        if (row < FIRST_FLOOR_ROW || floor_down_rad[z] == 0) continue;
        sum_m += floor_range_m(floor_down_rad[z]);
        n++;
    }
    return n ? sum_m / (float)n : 0;
}

bool rangefinder_zone_floor(int ray, float *floor_m, float *obstacle_min_height_m) {
    if (ray < FIRST_FLOOR_ROW * RANGEFINDER_COLS || ray >= RANGEFINDER_RAYS) return false;
    int z = floor_index(ray);
    if (floor_down_rad[z] == 0) return false;
    *floor_m = floor_range_m(floor_down_rad[z]);
    *obstacle_min_height_m = obstacle_min_m[z];
    return true;
}

int rangefinder_floor_zones(void) { return FLOOR_ZONES; }

int rangefinder_first_floor_row(void) { return FIRST_FLOOR_ROW; }

float rangefinder_floor_limit_m(int ray) {
    if (!have_zones) make_zones();
    float far_down_rad = zone_down_rad[ray] - ZONE_RAD / 2; // where its upper edge meets the floor
    if (ray / RANGEFINDER_COLS < FIRST_FLOOR_ROW || far_down_rad <= 0) return 0;
    return DROP_SHARE * floor_range_m(far_down_rad);
}
