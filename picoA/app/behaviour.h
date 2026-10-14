#pragma once
// What the robot does when a command asks for it (doc/ARCHITECTURE.md, actions): one
// action at a time, nothing by itself.
//   Move:  straight by distance_m (+ = forward, - = back), slowing down near the end.
//   Turn:  in place by angle_rad (+ = left).
//   Watch: for the given seconds, turn towards the biggest movement the VL53 sees
//          (still or turning), following it at its own angular speed plus a
//          correction, at most ~29°/s, never past where it was last seen; it starts
//          when the movement is 8° off and stops within 3° once it is about still,
//          or when nothing moves. Its movement lines are printed (motion_sense's
//          log) only while watching.
//   Go to: a point on the map (its frame): turn in place towards it, then drive
//          straight to it, steering a little to keep it ahead, and stop stop_m
//          before it (measured along the way), or STOP_BEFORE_M before an obstacle
//          on the map as Move does. At most GOTO_MAX_M at once: a goal farther away
//          is gone towards. The turn and the distance come from the robot's own pose.
// Each action switches the motors on at its start and off at its end, and ends
// exactly once, also when it can't start (a move blocked from the start, a goal
// already reached): done (after standing still), an obstacle, stopped, replaced by
// another action, a safety stop on PicoB, PicoB lost, or didn't start. Each action
// carries the number of the command that started it (commands.h), handed back
// when it ends. With recording on, each action is one take (recorder.h), from its
// start to its end.
#include <stdbool.h>
#include <stdint.h>
#include "recorder.h"

// How a move keeps clear of obstacles on the map. The PC's --check reads these
// (pc/robot/src/api/firmware.ts): plain numbers only.
#define STOP_BEFORE_M    0.15f     // a move stops this far before an obstacle on the map...
#define EDGE_M           0.095f    // ...measured from the front (back) edge, this far from the centre
#define HALF_WIDTH_M     0.1175f   // the robot's half width
#define PATH_HALF_M      0.1675f   // HALF_WIDTH_M plus 5 cm: the path a move checks
#define GOTO_MAX_M       3.0f      // a goal farther away: the robot goes this far towards it

void behaviour_move(float distance_m, uint32_t command);
void behaviour_turn(float angle_rad, uint32_t command);
void behaviour_watch(float seconds, uint32_t command);
void behaviour_goto(float x_m, float y_m, float stop_m, uint32_t command);
void behaviour_stop(void);   // ends the running action: the robot stands still, motors off
bool behaviour_busy(void);
void behaviour_update(void); // call every loop iteration

// How an action ended.
typedef struct {
    uint32_t command;     // the number of the command that started it
    const char *action;   // as in recordings: "move", "turn", "watch", "goto"
    rec_end_t reason;
    uint8_t stop_reason;  // REC_END_SAFETY_STOP: PicoB's STOP_* (link_msgs.h)
    float moved_m;        // from where it started to where it ended, straight
    float rotated_rad;    // how far it turned, + = left (not wrapped)
    float goal_m;         // Go to: how far the point was when it started; 0 for the others
} behaviour_end_t;
// Counts the actions that ended; the last one's end (all zero before the first).
uint32_t behaviour_endings(behaviour_end_t *last);
const char *behaviour_action(void); // the running action's name, "" if none
