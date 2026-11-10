#pragma once
// PicoB as PicoA sees it: the robot's wheels, odometry and tilt, and its IMU as it
// measured. Hides the link protocol: greeting PicoB and checking its version,
// repeating the drive command often enough for PicoB's safety stop, re-sending the
// motor switch until PicoB reports it, decoding PicoB's reports, and the two clocks. PicoB's log lines are printed
// on PicoA's stdout as "B: ...". When PicoB switches the motors off by itself
// (a safety stop, see odometry_report_t.stop_reason), they stay off until the next
// body_motors(true).
#include <stdbool.h>
#include <stdint.h>
#include "link_messages.h"

void body_init(void);          // starts the link
void body_update(void);        // call every loop iteration
// PicoB has greeted us with our protocol version and its reports are arriving.
bool body_connected(void);
const odometry_report_t *body_odometry(void);     // latest report; all zero until the first one
// When the latest report was measured, on PicoA's clock (time_us_32()). PicoB's
// clock is translated from the reports' t_us: within ~0.1 ms once a few seconds of
// reports have arrived, and kept right as the two clocks drift apart.
uint32_t body_odometry_time_us(void);
// Every report, numbered from 1 as it arrives; the last BODY_ODOMETRY_KEPT are kept, so
// a loop iteration longer than a report period (20 ms) doesn't lose any. Report n
// and when it was measured (PicoA's clock); false if n isn't kept (any more).
#define BODY_ODOMETRY_KEPT 16
uint32_t body_odometry_count(void);
bool body_odometry_get(uint32_t n, odometry_report_t *report, uint32_t *t_us);
const status_report_t *body_status(void); // latest status (2 Hz); all zero until the first one
// PicoB's imu records (104 a second), their t_us turned into PicoA's clock as they
// arrive (as body_odometry_time_us()). Kept only once PicoB has greeted us with our
// version and its clock is known (its first report after a hello); others are
// dropped. The oldest not yet taken; false if none waits. The newest BODY_IMU_KEPT wait (~0.3 s); older ones not taken
// are lost, counted.
#define BODY_IMU_KEPT 32
bool body_imu_take(imu_record_t *r);
uint32_t body_imu_received(void); // records kept since power-up
uint32_t body_imu_lost(void);     // ...and lost: not taken in time
void body_motors(bool on);     // on also clears PicoB's safety stop
void body_drive(float v_mps, float w_radps); // + = forward / turn left; held until changed
