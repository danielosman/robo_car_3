#pragma once
// Movement around the robot (ROBOT_PLAN.md §6). The VL53 (tof_motion) watches all
// the time, still and turning in place, against a background kept per world
// direction (no learning after a stop). The camera (camera_motion) watches only while
// the robot stands still (PicoB says so; with no PicoB the robot can't move): after
// every stop it learns the view (~1 s). The tracker follows one target in the VL53's
// movement while the robot stands still (tracker). The camera's exposure follows the light (slowly) while the robot moves; while
// it stands still it is held and adjusted only when calm. With the log on (while
// watching: behaviour switches it), each
// detector's movement is printed (at most twice a second while it goes on) and its
// end, and each exposure adjustment while still; each line starts with the robot's
// time in seconds since power-up. The tracker's target is only logged: behaviour
// turns towards the biggest movement (motion_sense_strongest).
#include <stdbool.h>
#include <stdint.h>

void motion_sense_update(void); // call every loop iteration
// The biggest movement the VL53 sees now (in its latest frame, still or turning):
// its direction from where the robot faced when the frame was measured (+ = left),
// and when that was (PicoA's clock: a new value means a new frame). False if nothing
// moves.
bool motion_sense_strongest(float *bearing_rad, uint32_t *t_us);
bool motion_sense_watching(void); // still, and the references are learned
bool motion_sense_still(void);    // the robot stands still: the detectors watch or learn
void motion_sense_log(bool on);
void motion_sense_stamp(void);  // prints the robot's time in seconds, as the movement log does
