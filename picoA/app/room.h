#pragma once
// How far the robot may drive straight before it must stop for an obstacle on the map
// (doc/ARCHITECTURE.md, actions): the rule behaviour.c drives by. Pure: it reads only
// cell_map, so the PC's helper compiles it too and robot check asks the same rule
// (pc/robot/native/cell_map_helper.c).
#include <stdbool.h>

// The PC reads these (pc/robot/src/api/firmware.ts): plain numbers only.
#define STOP_BEFORE_M    0.15f     // a move stops this far before an obstacle on the map...
#define EDGE_M           0.095f    // ...measured from the front (back) edge, this far from the centre
#define HALF_WIDTH_M     0.1175f   // the robot's half width
#define PATH_HALF_M      0.1675f   // HALF_WIDTH_M plus 5 cm: the path a move checks
#define ROOM_START_M     0.01f     // a drive with less room than this doesn't start

// From (x, y) facing yaw, driving in direction (+1 forward, -1 back): how far it may go
// before it is STOP_BEFORE_M from the nearest obstacle on the map (a blocked or overhang
// column) in its path, measured from its edge on that side. Looks at most look_m far:
// exactly look_m when nothing is closer. Unknown columns count as free.
float room_m(float x_m, float y_m, float yaw_rad, float direction, float look_m);
// The same as a gap: from the edge to that obstacle (room_m + STOP_BEFORE_M); exactly
// STOP_BEFORE_M + look_m when nothing is closer. For the robot's console lines.
float room_gap_m(float x_m, float y_m, float yaw_rad, float direction, float look_m);

// A drive of distance_m (+ forward, - back) on the map as it is now: how far it goes and
// whether an obstacle stops it short; moved 0 with obstacle: it doesn't start. (On the
// robot the map grows while it drives, so it may stop sooner.)
typedef struct {
    float moved_m;
    bool obstacle;
} room_drive_t;
room_drive_t room_drive(float x_m, float y_m, float yaw_rad, float distance_m);
