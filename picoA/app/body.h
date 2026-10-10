#pragma once
// PicoB as PicoA sees it: the robot's wheels, odometry and tilt. Hides the link
// protocol: greeting PicoB and checking its version, repeating the drive command
// often enough for PicoB's safety stop, re-sending the motor switch until
// PicoB reports it, and decoding PicoB's reports. PicoB's log lines are printed
// on PicoA's stdout as "B: ...". When PicoB switches the motors off by itself
// (a safety stop, see odom_report_t.stop_reason), they stay off until the next
// body_motors(true).
#include <stdbool.h>
#include <stdint.h>
#include "link_msgs.h"

void body_init(void);          // starts the link
void body_update(void);        // call every loop iteration
// PicoB has greeted us with our protocol version and its reports are arriving.
bool body_connected(void);
const odom_report_t *body_odom(void);     // latest report; all zero until the first one
// When the latest report was measured, on PicoA's clock (time_us_32()). PicoB's
// clock is translated from the reports' t_us: within ~0.1 ms once a few seconds of
// reports have arrived, and kept right as the two clocks drift apart.
uint32_t body_odom_time_us(void);
// Every report, numbered from 1 as it arrives; the last BODY_ODOM_KEPT are kept, so
// a loop iteration longer than a report period (20 ms) doesn't lose any. Report n
// and when it was measured (PicoA's clock); false if n isn't kept (any more).
#define BODY_ODOM_KEPT 16
uint32_t body_odom_count(void);
bool body_odom_get(uint32_t n, odom_report_t *report, uint32_t *t_us);
const status_report_t *body_status(void); // latest status (2 Hz); all zero until the first one
void body_motors(bool on);     // on also clears PicoB's safety stop
void body_drive(float v_mps, float w_radps); // + = forward / turn left; held until changed
// The DRIVE commands sent to PicoB so far (repeated every 50 ms), and the latest:
// what and when (PicoA's clock). For recording.
uint32_t body_drive_sent(float *v_mps, float *w_radps, uint32_t *t_us);
