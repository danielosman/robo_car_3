#pragma once
// What the robot does when a command asks for it (doc/ARCHITECTURE.md, actions): one
// action at a time, nothing by itself.
//   Drive:  straight by distance_m (+ = forward, - = back), slowing down near the end.
//   Rotate: in place by angle_rad (+ = left).
//   Watch: for the given seconds, turn towards the biggest movement the VL53 sees
//          (still or turning), following it at its own angular speed plus a
//          correction, at most ~29°/s, never past where it was last seen; it starts
//          when the movement is 8° off and stops within 3° once it is about still,
//          or when nothing moves. Its movement lines are printed (watch's
//          log) only while watching.
//   Go to: a point on the map (its frame): turn in place towards it, then drive
//          straight to it, steering a little to keep it ahead, and stop stop_m
//          before it (measured along the way), or STOP_BEFORE_M before an obstacle
//          on the map as Drive does. At most GOTO_MAX_M at once: a goal farther away
//          is gone towards. The turn and the distance come from the robot's own pose.
// Each action is one move (move.h): the move switches the motors on and off, stands
// still before and after, sends the move's start, end and frames, and ends it
// exactly once, also when it can't start (a move blocked from the start, a goal
// already reached), carrying the number of the command that started it
// (commands.h). This module is how each action drives, inside its move's act phase.
#include <stdbool.h>
#include <stdint.h>

// How far a move goes before an obstacle on the map: room.h. The PC's robot check reads
// these (pc/robot/src/api/firmware.ts): plain numbers only.
#define GOTO_MAX_M       3.0f      // a goal farther away: the robot goes this far towards it
#define GOTO_THERE_M     0.05f     // a goal closer than this: already there

void behaviour_drive(float distance_m, uint32_t command);
void behaviour_rotate(float angle_rad, uint32_t command);
void behaviour_watch(float seconds, uint32_t command);
void behaviour_goto(float x_m, float y_m, float stop_m, uint32_t command);
// Stopping, how it ended and what runs: move.h (move_stop(), move_endings(), move_running()).
