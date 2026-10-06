#pragma once
// The VL53L8CX as rays in the robot frame. Hides the zone order and how the sensor
// is mounted (turned 90° on the PCB, 2.5 cm ahead of the centre, 3 cm right, 7 cm
// up), its settings and status codes, and the floor: the lower four rows of zones
// see the floor, and a reading is an obstacle only if it ends at least 2 cm above
// it (more in zones whose floor readings are noisy). The two lowest rows must
// see their floor: no return, or one from beyond the floor, may be a drop (stairs)
// or a floor the sensor can't see, and is reported as such. (The 6th row's floor is
// too long and grazing for that: such readings from it tell nothing.) The floor is learned while
// the robot turns, because floors differ and each zone sees its patch of floor
// differently (a zone reports the near edge of a patch 5.6° tall).
#include <stdbool.h>
#include <stdint.h>

#define RANGEFINDER_ROWS 8
#define RANGEFINDER_COLS 8
#define RANGEFINDER_RAYS (RANGEFINDER_ROWS * RANGEFINDER_COLS)
#define RANGEFINDER_HZ   15 // frames per second; each measures for the whole period

#define RANGE_NO_TARGET 0      // range_mm: nothing within range
#define RANGE_INVALID   0xFFFF // range_mm: the sensor isn't sure

// One measurement as the robot sees it: range_mm[row * 8 + col], row 0 at the top,
// column 0 at the left.
typedef struct {
    uint32_t t_us;                      // PicoA's clock (time_us_32()), middle of the measurement
    uint16_t range_mm[RANGEFINDER_RAYS]; // closest sure target along each ray, or RANGE_NO_TARGET / RANGE_INVALID
    uint8_t status[RANGEFINDER_RAYS];   // VL53 status of the first target, for printing why a zone is unsure
} range_frame_t;

typedef enum {
    RAY_UNUSED,   // tells nothing: an unsure reading, or a floor zone whose floor isn't known
    RAY_CLEAR,    // free from the sensor to the end point (nothing within 1 m, or something far that may be the floor)
    RAY_FLOOR,    // free from the sensor to the end point, where it sees the floor
    RAY_HIT,      // free up to the end point, which is an obstacle 2 cm or more above the floor
    RAY_NO_FLOOR, // the end point is where the floor should be, but it wasn't seen there (a drop? a dark floor?)
} ray_kind_t;

typedef struct {
    float x_m, y_m, z_m; // end point, robot frame (x forward, y left); z above the floor
    ray_kind_t kind;
    // RAY_FLOOR: where the zone's patch of floor begins (its lower edge meets the
    // floor), robot frame. The zone saw the floor from there to the end point.
    float floor_from_x_m, floor_from_y_m;
} ray_t;

typedef struct {
    float origin_x_m, origin_y_m, origin_z_m; // the sensor, robot frame
    ray_t ray[RANGEFINDER_RAYS];
} scan_t;

bool rangefinder_init(void);                 // starts the sensor (~2 s); false if it doesn't answer
bool rangefinder_poll(range_frame_t *frame); // true when a new frame was read
// The unit direction of a ray from the sensor (x forward, y left, z up), robot level,
// and where the sensor is (robot frame, z above the floor).
void rangefinder_ray_direction(int ray, float dir[3]);
void rangefinder_origin(float origin[3]);
// For a zone that sees the floor: the farthest reading that can still be its floor
// (robot level; farther ones are reflections or a drop, and tell nothing about what
// stands there). 0 for zones with no such limit.
float rangefinder_floor_limit_m(int ray);
// Where each reading of `frame` ends and what it means, with the robot pitched by
// pitch_rad (+ = nose up) when it was measured.
void rangefinder_scan(const range_frame_t *frame, float pitch_rad, scan_t *scan);

// Learning the floor: forget it, feed frames from all around the robot (a full
// turn), then finish. Each floor zone takes, of the readings that fit its patch of
// floor, a distance that is both long and common (obstacles only make readings
// shorter), and how much its floor readings scatter. Zones not learned stay unused,
// except in the two lowest rows, which tell drops: they assume the floor the
// sensor's height gives. Returns how many zones were learned, of
// rangefinder_floor_zones(), and how many were assumed.
void rangefinder_forget_floor(void);
void rangefinder_learn_floor(const range_frame_t *frame, float pitch_rad);
int rangefinder_finish_floor(int *assumed);
int rangefinder_floor_zones(void);
int rangefinder_first_floor_row(void);
float rangefinder_floor_distance(int row); // where a floor row sees the floor (robot level), averaged; 0 = not learned
// What floor zone `ray` learned: where it sees the floor (robot level) and how far
// above it a reading must end to be an obstacle. False for other rays and zones not learned.
bool rangefinder_zone_floor(int ray, float *floor_m, float *obstacle_min_height_m);
