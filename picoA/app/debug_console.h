#pragma once
// PicoA's console, on USB and over WiFi (wifi_console.h), the same keys on both:
// one key, one operation (h lists them): the actions, settings, and prints (status,
// the map, a ToF or camera frame, the movement backgrounds). Nothing is printed by
// itself. When a serial monitor or the robot server connects it prints the keys.
// Losing the monitor doesn't stop the robot.
void debug_console_update(void); // call every loop iteration
