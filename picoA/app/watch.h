#pragma once
// Watching: movement around the robot (doc/ARCHITECTURE.md, watch). The one
// interface for it; inside, watch_tof (the VL53), watch_camera (the camera),
// tracker and change_grid. The VL53 watches all the time, still and turning in
// place, against a background kept per world direction (no learning after a stop):
// it reads the posed frames of the watch use (watch_add()). Their 8-bit distances
// end at 2.54 m: anything farther reads "beyond", so movement past ~2.5 m (someone
// at 2.7 m in front of a wall at 3.5 m) isn't seen (accepted by Daniel, 7 Nov; watch
// may get its own format later). The camera watches only
// while the robot stands still (PicoB says so; with no PicoB the robot can't move):
// after every stop it learns the view (~1 s); watch_update() takes its frames. The
// tracker follows one target in the VL53's movement while the robot stands still.
// The camera's exposure follows the light (slowly) while the robot moves; while it
// stands still it is held and adjusted only when calm. With the log on (while
// watching: behaviour switches it), each detector's movement is printed (at most
// twice a second while it goes on) and its end, and each exposure adjustment while
// still; each line starts with the robot's time in seconds since power-up. The
// tracker's target is only logged: behaviour turns towards the biggest movement
// (watch_strongest).
#include <stdbool.h>
#include <stdint.h>
#include "mounting.h"
#include "tof_frame.h"

// The camera's mounting (main.c: ROBOT_MOUNTING; the VL53's rays are tof_rays'). False
// if it isn't for this picture: then the camera doesn't watch.
bool watch_init(const mounting_t *m);
void watch_add(const posed_frame_t *frame); // a ToF frame, as unpacked for the watch use
void watch_update(void); // call every loop iteration: the camera
// The biggest movement the VL53 sees now (in its latest frame, still or turning):
// its direction from where the robot faced when the frame was measured (+ = left),
// and when that was (PicoA's clock: a new value means a new frame). False if nothing
// moves.
bool watch_strongest(float *bearing_rad, uint32_t *t_us);
bool watch_watching(void); // still, and the references are learned
bool watch_still(void);    // the robot stands still: the detectors watch or learn
void watch_log(bool on);
void watch_stamp(void);  // prints the robot's time in seconds, as the movement log does
