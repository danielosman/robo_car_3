#pragma once
// M0 test: drive a 50 cm square (forward 50 cm, turn left 90°, four times) on
// PicoB's odometry, then print where the robot thinks it ended up. Compare that
// with where it really is: the gap is the odometry error to calibrate in M1.
#include <stdbool.h>

void drive_test_start(void);  // switches the motors on
void drive_test_stop(void);   // stops the robot; motors stay on
bool drive_test_running(void);
void drive_test_update(void); // call every loop iteration
