#pragma once
// PicoA's commands (doc/ARCHITECTURE.md, commands): one per line, "<command> [#<n>]",
// the same over WiFi (from the robot server's API, which numbers them) and on USB
// (typed, for debugging). The robot does nothing by itself.
//   drive <cm>             straight; negative = backwards
//   rotate <deg>           on the spot; + = left
//   goto <x cm> <y cm>     a point on the map (its frame)
//   watch <s>              turn towards movement for that long
//   snapshot               one to the server (snapshot.h)
//   clear_map
//   recording on|off
//   mode idle|manual       idle also stops, as stop does
//   stop                   motors off, mode idle
//   status                 PicoA's and PicoB's state, printed (for a serial monitor)
// A move in idle switches to manual. Each command is answered once it has ended, by
// a done record to the server: "\001done n=<n> end=<end> x=<cm> y=<cm> yaw=<deg>"
// and, for a move, "moved=<cm>" (from where it started, straight), for a go to also
// "goal=<cm>" (how far the point was), for a snapshot "snapshot=<its number>" (the
// pose is the snapshot's). end: ok, obstacle, stopped, replaced, tilt, wheels,
// no_drive, picob_lost, no_start, refused (not a command, or a number out of range).
// The server's API reads the limits below (pc/robot/src/api/firmware.ts): plain
// numbers only.

#define COMMAND_DRIVE_MAX_CM   300
#define COMMAND_ROTATE_MAX_DEG 720
#define COMMAND_GOTO_MAX_CM    300 // each of x and y
#define COMMAND_WATCH_MIN_S    1
#define COMMAND_WATCH_MAX_S    600

void commands_update(void); // call every loop iteration
