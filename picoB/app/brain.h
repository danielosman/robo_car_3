#pragma once
// PicoA (the brain) as PicoB sees it: the link protocol from PicoB's side.
// Answers PicoA's greeting and keeps the motors off until the protocol versions
// match; applies MOTORS and DRIVE to `drive`; reports odometry and status on
// time. Safety stops: switches the motors off by itself when DRIVE commands stop
// arriving or `drive` reports a fault, and keeps them off until PicoA asks again
// with a new MOTORS request. Each stop is reported once, as a log line.
#include <stdbool.h>

void brain_init(bool imu_ok);  // starts the link; without an IMU the motors stay off
void brain_update(void);       // call every loop iteration, after drive_update()
// A line for PicoB's USB serial and PicoA's debug log (PicoA gets at most
// LINK_MAX_BODY characters). printf-style, no newline.
void brain_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
