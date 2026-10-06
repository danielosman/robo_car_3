#pragma once
// Movement around the robot (ROBOT_PLAN.md §6). Watches only while the robot stands
// still (PicoB says so; with no PicoB the robot can't move): after every stop the
// detectors learn what the view looks like (~1 s), then report what moves in it.
// M3a so far: the VL53 (tof_motion) and the camera (camera_motion), each on its
// own; the tracker follows one target in the VL53's movement (tracker). The camera's exposure follows the light (slowly) while the robot moves; while
// it stands still it is held and adjusted only when calm. With the log on, each
// detector's movement is printed (at most twice a second while it goes on) and its
// end, and each exposure adjustment while still; each line starts with the robot's
// time in seconds since power-up.
#include <stdbool.h>

void motion_sense_update(void); // call every loop iteration
bool motion_sense_watching(void); // still, and the references are learned
bool motion_sense_still(void);    // the robot stands still: the detectors watch or learn
void motion_sense_log(bool on);
bool motion_sense_logging(void);
