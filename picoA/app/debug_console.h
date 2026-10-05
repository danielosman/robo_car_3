#pragma once
// PicoA's console while developing, on USB and over WiFi (wifi_console.h):
// single-key commands (h lists them), the robot tests' progress and results, the
// map, PicoB's log lines, and a status line (odometry, tilt, gyro bias, motors)
// every 15 s while nothing else prints. When a serial monitor or the robot server
// connects it prints the keys and the last test result. Losing the monitor doesn't
// stop the robot, so a test can run without the cable.
void debug_console_update(void); // call every loop iteration
