#pragma once
// Snapshots (doc/ARCHITECTURE.md, snapshots): what the robot knows at one moment, sent
// to the server in parts on the bulk lane (link_pc.h; snapshot part messages, wire.h)
// when a command asks for one, and when a server connects: the pose, the last move and how it
// ended, the mode, the last ToF frame as a compact frame (tof_format.h, the snapshot
// use: its pose in it), the last camera frame, the colour sensor's last reading, the map
// window with its anchor, and the whole-house map's tiles that changed since they
// were last sent (all of them for a new server). One snapshot is built at a time
// into a static buffer; one asked for while the last is still going out is built
// when it has gone (with what the robot knows then). Without a server nothing is
// built: one is built when a server connects. A part the lane has no room for
// waits; a snapshot cut by a lost connection goes again whole on the next.
//
// Layout (little-endian, packed; the PC reads it in pc/robot/src/snapshot.ts):
// "SNAP", version u8, total length u32, then sections [type u8][length u32][payload],
// then a CRC-32 (zlib's, crc32.h) of everything before it. The sections are the
// snap_*_t structs in snapshot.c; a section is left out when there is nothing for it.
#include <stdbool.h>
#include <stdint.h>
#include "tof_frame.h"

typedef enum {
    SNAP_COMMAND = 1, // a command asked for it (0 was "an action ended", until 14 Oct)
    SNAP_SERVER = 2,  // a server connected
} snapshot_why_t;

// The boot id and the firmware's build (when it was built) for every snapshot's head;
// build lives for ever.
void snapshot_init(uint32_t boot_id, const char *build);
// Asks for a snapshot. command: the number of the command asking (0: none), handed
// back by snapshot_sent().
void snapshot_request(snapshot_why_t why, uint32_t command);
// A snapshot a command asked for has wholly gone to the connection: every part handed
// to TCP, so its answer, written after this, follows it in the stream. A connection
// lost before this sends it again whole on the next; lost after it (parts in TCP
// but not delivered), the snapshot is gone and its answer arrives on the next
// connection (the PC's runner then says the snapshot wasn't stored). Its command, its number and the robot's pose in
// it; true once per such snapshot.
typedef struct {
    uint32_t command, seq;
    float x_m, y_m, yaw_rad;
} snapshot_sent_t;
bool snapshot_sent(snapshot_sent_t *s);
void snapshot_update(void);       // call every loop iteration, after behaviour
// Each ToF frame as stamped (main.c): the latest one goes into the next snapshot,
// packed for the snapshot use.
void snapshot_tof(const posed_frame_t *frame);
void snapshot_print_status(void); // one line for the status key
