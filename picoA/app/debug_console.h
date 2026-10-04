#pragma once
// PicoA's USB serial while developing: single-key commands (h lists them), the
// robot tests' progress and results, the map, PicoB's log lines, and a status line
// (odometry, tilt, gyro bias, motors) every 15 s while nothing else prints. On
// connecting it prints the keys and the last test result; the first time after
// power-up it also starts the start-up scan (behaviour.h). Unplugging the USB
// doesn't stop the robot, so a test can run without the cable.
void debug_console_update(void); // call every loop iteration
