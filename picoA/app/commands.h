#pragma once
// PicoA's commands (doc/ARCHITECTURE.md, commands): from the robot server's API, one
// command message each with its number (wire.h, link_pc.h); typed on USB (for
// debugging) one per line, "<command> [#<n>]". The robot does nothing by itself.
//   drive <cm>             straight; negative = backwards
//   rotate <deg>           on the spot; + = left
//   goto <x cm> <y cm>     a point on the map (its frame), at most COMMAND_GOTO_MAX_CM
//                          from where the robot is now (farther: refused)
//   watch <s>              turn towards movement for that long
//   snap                   a snapshot to the server (snapshot.h)
//   map                    the 6 x 6 m map window to the server (wire.h, map part)
//   clear_map
//   config pre_ms <ms>     how long each move stands still before it acts (move.h), 0-2000
//   config post_ms <ms>    ...and after; both 200 at power-up
//   correct <x mm> <y mm> <yaw 0.01°>   the PC's correction of the pose, from now on (pose.h,
//                          pose_correct; doc/OUTLINE_MAP_PLAN.md, step F): replaces the one before;
//                          sends "corrected" (wire.h), then the answer; refused while a move runs,
//                          a snapshot is asked for or going out, or the bulk lane holds anything
//                          (control goes first: "corrected" would overtake it; the PC sends it again)
//                          "corrected" goes again on every new connection to the PC (if not 0)
//   mode idle|manual       idle also stops, as stop does
//   stop                   motors off, mode idle
//   status                 PicoA's and PicoB's state, printed (for a serial monitor)
// A move in idle switches to manual. Each command is answered once it has ended, by
// an answer message to the server (wire.h): its number, how it ended, where the
// robot is, and for a move how far it went (from where it started: straight, and
// how far it turned, + = left), for a go to also how far the point was, for a
// snapshot its number (the pose is the snapshot's). How it ended: ok, obstacle,
// stopped, replaced, tilt, wheels, no_drive, picob_lost, no_start, or refused, each
// reason its own end (wire.h): refused (not a command), out_of_range, busy_move,
// busy_snapshot, busy_bulk; the robot's line names the value and the limit too.
// clear_map also sends "map cleared".
// The server's API reads the limits below (pc/robot/src/api/firmware.ts): plain
// numbers only.

#define COMMAND_DRIVE_MAX_CM   300
#define COMMAND_ROTATE_MAX_DEG 720
#define COMMAND_GOTO_MAX_CM    300 // how far a go to's point may be from the robot
#define COMMAND_POINT_MAX_CM   100000 // a go to's x and y on the map: only so the numbers stay plain (1 km)
#define COMMAND_WATCH_MIN_S    1
#define COMMAND_WATCH_MAX_S    600
#define COMMAND_STILL_MAX_MS   2000 // config pre_ms / post_ms (move.h's MOVE_STILL_MAX_MS)
#define COMMAND_CORRECT_MAX_MM 20000 // correct: each of x and y
#define COMMAND_CORRECT_MAX_CDEG 18000 // correct: yaw, 0.01°

#include "wire.h"

void commands_update(void); // call every loop iteration
// The robot's state for the status the PC gets 4 times a second (link_pc.h): pose,
// mode, the running move.
void commands_status(wire_status_t *s);
