#pragma once
// PicoA's USB serial while developing: single-key commands (h lists them), the
// robot tests' progress and results, PicoB's log lines, and a status line
// (odometry, tilt, gyro bias, motors) every 15 s while no test runs. On
// connecting it prints the keys and the last test result. Unplugging the USB
// doesn't stop the robot, so a test can run without the cable.
void debug_console_update(void); // call every loop iteration
