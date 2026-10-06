#pragma once
// Movement around the robot (ROBOT_PLAN.md §6). Watches only while the robot stands
// still (PicoB says so; with no PicoB the robot can't move): after every stop the
// detectors learn what the view looks like (~0.5 s), then report what moves in it.
// M3a so far: the VL53 (tof_motion). With the log on, each movement is printed
// (at most twice a second while it goes on) and its end.
#include <stdbool.h>

void motion_sense_update(void); // call every loop iteration
bool motion_sense_watching(void); // still, and the references are learned
void motion_sense_log(bool on);
bool motion_sense_logging(void);
