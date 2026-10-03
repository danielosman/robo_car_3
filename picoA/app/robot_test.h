#pragma once
// Tests run on the robot to check and calibrate PicoB's odometry (M0, M1). Each
// prints its progress and, at the end, what it measured and what to compare it
// with in reality. They don't need the serial monitor: one can be started, the
// USB cable pulled, and the result read after plugging back in.
//   SQUARE:  50 cm square turning left; compare the end pose with the start.
//   DRIFT:   motors off, stand still 10 min; yaw and gyro bias every 15 s.
//   TURNS:   10 full turns left in place, stopping at 3600° by the gyro;
//            compare with a mark on the floor (gyro scale), plus the
//            effective track width from the wheels.
//   FORWARD, BACK: 2 m straight; compare with a tape measure (encoder distance).
#include <stdbool.h>

typedef enum { ROBOT_TEST_SQUARE, ROBOT_TEST_DRIFT, ROBOT_TEST_TURNS, ROBOT_TEST_FORWARD, ROBOT_TEST_BACK } robot_test_t;

void robot_test_start(robot_test_t test); // stops a running test first; switches the motors on (off for DRIFT)
void robot_test_stop(void);               // stops the robot; the motors stay as they are
bool robot_test_running(void);
void robot_test_update(void);             // call every loop iteration
// How the last test ended and what it measured ("" while none has ended), also
// when it stopped early (and why), so it can be printed again after the serial
// monitor was disconnected.
const char *robot_test_result(void);
