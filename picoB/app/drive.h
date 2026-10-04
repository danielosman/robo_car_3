#pragma once
// Drives the four wheels at a commanded forward speed and turn rate. Each side's
// wheel speed is closed-loop on its two encoders, averaged (both motors on a
// side get the same power), and the turn rate is trimmed with the gyro, so turning is accurate on
// wood or carpet despite skid steering. Commands are limited to safe speeds and
// ramped to limit wheel slip.
#include <stdbool.h>
#include "odometry.h"

#define DRIVE_FOLLOW_TIME_S 1.0f  // a wheel not following its target this long is a fault
#define DRIVE_MAX_TILT_DEG  15.0f // pitch or roll beyond this is a fault; driving stays within ~5°

void drive_init(void);              // motors off (driver standby)
void drive_enable(bool on);         // off stops at once; on starts from standstill
bool drive_enabled(void);
void drive_set(float v_mps, float w_radps); // + = forward / turn left; held until changed
void drive_update(const odom_t *odom);      // call every ~10 ms

typedef enum { DRIVE_OK, DRIVE_LEFT_NOT_FOLLOWING, DRIVE_RIGHT_NOT_FOLLOWING, DRIVE_TILTED } drive_fault_t;
// Why drive_update() switched the motors off by itself; cleared by drive_enable(true).
// Not following: a wheel on that side didn't follow the side's target for
// DRIVE_FOLLOW_TIME_S: stopped (jammed, no encoder signal), far too slow, or
// turning the wrong way (reversed encoder or motor). Tilted: pitch or roll beyond
// DRIVE_MAX_TILT_DEG (lifted, tipping over).
drive_fault_t drive_fault(void);
