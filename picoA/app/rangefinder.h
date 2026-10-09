#pragma once
// The VL53L8CX as rays in the robot frame. Hides the zone order and how the sensor
// is mounted (turned 90° on the PCB, 2.5 cm ahead of the centre, 3 cm right, 7 cm
// up), its settings and status codes. What the readings mean is the map's
// (cell_map) and movement detection's (tof_motion) business.
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

bool rangefinder_init(void);                 // starts the sensor (~2 s); false if it doesn't answer
bool rangefinder_poll(range_frame_t *frame); // true when a new frame was read
// The unit direction of a ray from the sensor (x forward, y left, z up), robot level,
// and where the sensor is (robot frame, z above the floor).
void rangefinder_ray_direction(int ray, float dir[3]);
// The same for a ray inside the zone, down_frac and left_frac (−0.5…0.5) of the zone
// from its centre (+ = down, left), with the robot pitched nose up by pitch_rad.
void rangefinder_sub_ray_direction(int ray, float down_frac, float left_frac, float pitch_rad, float dir[3]);
void rangefinder_origin(float origin[3]);
// For a zone that sees the floor: the farthest reading that can still be its floor
// (robot level; farther ones are reflections or a drop, and tell nothing about what
// stands there). 0 for zones with no such limit.
float rangefinder_floor_limit_m(int ray);
// Rows from this one down see the floor (in movement detection, their readings beyond
// rangefinder_floor_limit_m() tell nothing).
int rangefinder_first_floor_row(void);
