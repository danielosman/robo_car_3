#pragma once
// What the robot does when a key asks for it (doc/COMMANDS_PLAN.md): one action at
// a time, nothing by itself.
//   Scan:  turn 390° left in place (a full turn plus 30° of overlap). The map is fed
//          from every frame anyway; the scan only turns.
//   Move:  straight by distance_m (+ = forward, - = back), slowing down near the end.
//   Turn:  in place by angle_rad (+ = left).
//   Watch: for 1 minute, turn towards the biggest movement the VL53 sees (still or
//          turning), following it at its own angular speed plus a correction, at
//          most ~29°/s, never past where it was last seen; it starts when the
//          movement is 8° off and stops within 3° once it is about still, or when
//          nothing moves. Its movement lines are printed (motion_sense's log) only
//          while watching.
// Each action switches the motors on at its start and off at its end, and says how
// it ended: done (after standing still), stopped, replaced by another action, a
// safety stop on PicoB, or PicoB lost.
#include <stdbool.h>

void behaviour_scan(void);
void behaviour_move(float distance_m);
void behaviour_turn(float angle_rad);
void behaviour_watch(void);
void behaviour_stop(void);   // ends the running action: the robot stands still, motors off
bool behaviour_busy(void);
void behaviour_update(void); // call every loop iteration
