#pragma once
// Drives the four wheels at a commanded forward speed and turn rate. Each side's
// wheel speed is closed-loop on its front encoder (the rear motor gets the same
// power), and the turn rate is trimmed with the gyro, so turning is accurate on
// wood or carpet despite skid steering. Commands are limited to safe speeds and
// ramped to limit wheel slip.
#include <stdbool.h>
#include "odometry.h"

void drive_init(void);              // motors off (driver standby)
void drive_enable(bool on);         // off stops at once; on starts from standstill
bool drive_enabled(void);
void drive_set(float v_mps, float w_radps); // + = forward / turn left; held until changed
void drive_update(const odom_t *odom);      // call every ~10 ms

typedef enum { DRIVE_OK, DRIVE_LEFT_NOT_FOLLOWING, DRIVE_RIGHT_NOT_FOLLOWING } drive_fault_t;
// A side's front wheel didn't follow its target for 1 s: stopped (jammed, no
// encoder signal), far too slow, or turning the wrong way (reversed encoder or
// motor). drive_update() switched the motors off. Cleared by drive_enable(true).
drive_fault_t drive_fault(void);
