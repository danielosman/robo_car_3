#pragma once
// Snapshots (doc/ARCHITECTURE.md, snapshots): what the robot knows at one moment, sent
// to the server over the console's TCP connection (wifi_console.h) when a command
// asks for one, and when a server connects: the pose, the action and how it
// ended, the mode, the last ToF frame with all of its targets (and where the robot
// was then), the last camera frame, the colour sensor's last reading, the map
// window with its anchor, and the whole-house map's tiles that changed since they
// were last sent (all of them for a new server). One snapshot is built at a time
// into a static buffer; one asked for while the last is still going out is built
// when it has gone (with what the robot knows then). Without a server nothing is
// built: one is built when a server connects.
//
// Layout (little-endian, packed; the PC reads it in pc/robot/src/snapshot.ts):
// "SNAP", version u8, total length u32, then sections [type u8][length u32][payload],
// then a CRC-32 (as the recordings') of everything before it. The sections are the
// snap_*_t structs in snapshot.c; a section is left out when there is nothing for it.
// Every 0.25 s while a server listens, the pose line, "\001pose <x cm> <y cm>
// <yaw deg> <mode> <action> <WiFi signal dBm> <recording 0/1> <largest jolt x, y,
// z: g> <largest tilt: deg>\n" (the jolt and tilt since the line before, from
// PicoB), says where the robot is now; it is also what tells the server the robot
// is there.
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SNAP_COMMAND = 1, // a command asked for it (0 was "an action ended", until 14 Oct)
    SNAP_SERVER = 2,  // a server connected
} snapshot_why_t;

// Asks for a snapshot. command: the number of the command asking (0: none), handed
// back by snapshot_sent().
void snapshot_request(snapshot_why_t why, uint32_t command);
// A snapshot a command asked for has wholly gone to the connection: whatever is sent
// after it follows it in the stream. Its command, its number and the robot's pose in
// it; true once per such snapshot.
typedef struct {
    uint32_t command, seq;
    float x_m, y_m, yaw_rad;
} snapshot_sent_t;
bool snapshot_sent(snapshot_sent_t *s);
void snapshot_update(void);       // call every loop iteration, after behaviour
void snapshot_print_status(void); // one line for the status key
