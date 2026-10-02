#pragma once
// PicoA's USB serial while developing: a status line twice a second (link,
// odometry, motors, test) and single-key commands (h lists them). Losing the
// serial monitor (cable pulled, window closed) stops the robot and switches the
// motors off.
void debug_console_update(void); // call every loop iteration
