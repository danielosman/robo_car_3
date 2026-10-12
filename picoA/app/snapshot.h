#pragma once
// Snapshots (doc/ARCHITECTURE.md, snapshots): what the robot knows at one moment, sent
// to the server over the console's TCP connection (wifi_console.h) at the end of
// every action, on `S`, and when a server connects: the pose, the action and how it
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
// Every 0.25 s while a server listens, "\001pose <x cm> <y cm> <yaw deg> <mode>
// <action>\n" says where the robot is now.
#include <stdint.h>

typedef enum {
    SNAP_ACTION_END, // an action ended (done or not)
    SNAP_KEY,        // S
    SNAP_SERVER,     // a server connected
} snapshot_why_t;

void snapshot_request(snapshot_why_t why);
void snapshot_update(void);       // call every loop iteration, after behaviour
void snapshot_print_status(void); // one line for the status key
