#pragma once
// PicoB as PicoA sees it: the robot's wheels, odometry and tilt. Hides the link
// protocol: greeting PicoB and checking its version, repeating the drive command
// often enough for PicoB's safety stop, re-sending the motor switch until
// PicoB reports it, and decoding PicoB's reports. PicoB's log lines are printed
// on PicoA's stdout as "B: ...". When PicoB switches the motors off by itself
// (a safety stop, see odom_report_t.stop_reason), they stay off until the next
// body_motors(true).
#include <stdbool.h>
#include "link_msgs.h"

void body_init(void);          // starts the link
void body_update(void);        // call every loop iteration
// PicoB has greeted us with our protocol version and its reports are arriving.
bool body_connected(void);
const odom_report_t *body_odom(void); // latest report; all zero until the first one
void body_motors(bool on);     // on also clears PicoB's safety stop
void body_drive(float v_mps, float w_radps); // + = forward / turn left; held until changed
