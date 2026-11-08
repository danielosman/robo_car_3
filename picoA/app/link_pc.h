#pragma once
// PicoA's link to the PC (doc/ARCHITECTURE.md, link_pc; doc/DATA_FLOW_PLAN.md, the
// connection): WiFi, finding the server (its UDP announcements, or WIFI_SERVER_IP),
// one TCP connection with framed messages (wire.h) both ways, and two lanes on it.
// Joins the network in wifi_config.h and keeps reconnecting by itself; prints each
// step ("WiFi: ...", "Server: ..."). Losing it doesn't stop the robot. Only this
// module knows lwIP and TCP.
//
// - Everything printed goes to the PC too, one console message per line (USB stays
//   plain text). The first message on each connection is hello (boot id, build, wire
//   version); then, every 0.25 s while connected and control is empty, status
//   (filled by the status function given to link_pc_start(), the WiFi signal added
//   here).
// - Control lane: a 16 KB queue, sent first, never blocks the robot. When full, its
//   oldest messages go to make room (counted). It keeps filling while no server
//   listens (the newest 16 KB kept) and goes out when one does.
//   Decided (Daniel, 8 Nov 2026): any type may go, answers too. A link stalled for
//   16 KB of messages is a deeper problem; protecting answers isn't worth the code.
//   Don't add special cases for it.
// - Bulk lane: a 40 KB queue, sent only while control is empty and with at most
//   BULK_IN_FLIGHT (~5.6 KB) unacknowledged, so a control message waits behind at
//   most that much. A message that doesn't fit is refused (counted); its sender
//   decides (frames and imu batches are dropped, a snapshot part tries again).
//   What is queued when the connection is lost is dropped.
// - From the PC: commands (link_received()) and a heartbeat every 2 s. Nothing heard
//   for 30 s: the connection is closed and the server looked for again. Bytes that
//   aren't messages (wire_take() fails): closed too.
#include <stdbool.h>
#include <stdint.h>
#include "wire.h"

// Fills the status the PC gets 4 times a second (all but the WiFi signal).
typedef void (*link_pc_status_fn)(wire_status_t *s);

// Starts connecting; if already started, prints the state. boot_id and build go in
// hello; build is kept (a string that lives for ever).
void link_pc_start(uint32_t boot_id, const char *build, link_pc_status_fn status);
void link_pc_update(void);       // call every loop iteration
bool link_pc_connected(void);    // a server is listening
bool link_pc_settled(void);      // the first attempt is over: connected, failed, or joined without finding a server for 5 s
void link_pc_print_status(void); // the WiFi, the server, the lanes
// The connection's number (counts up with each connection), 0 while there is none.
// A different number means it was lost in between.
uint32_t link_pc_connection(void);

// A whole control message (made by wire's encoders) to the PC; kept while no server
// listens (joining, looking for it, reconnecting). False if it isn't a control
// message, or WiFi is off or unavailable (no network set, no chip: there will never
// be a server, so nothing is kept).
bool link_send_control(const uint8_t *msg, uint32_t len);
// A whole bulk message to the PC: false (counted) if not connected or no room.
bool link_send_bulk(const uint8_t *msg, uint32_t len);

// The next command from the PC; false if none is whole yet.
bool link_received(wire_command_t *c);
