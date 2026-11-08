#pragma once
// How a posed ToF frame is packed for a use (doc/DATA_FLOW_PLAN.md, data formats):
// the byte layouts, and which layout each use gets. The robot's map and watch read
// a frame after packing and unpacking it, the same data the PC gets. Pure functions,
// no I/O (the PC's helper compiles it too).
//
// One format today, compact (little-endian), for every use:
//   header, 21 B: frame number u32; time u32 (PicoA µs, the middle of the
//     measurement); period u16 (0.1 ms: how long it measured); x, y i16 (mm; ±32.7 m,
//     plenty for a flat); yaw i16 (0.01°, wrapped ±180°); pitch i16 (0.01°); turn
//     i16 (0.01°, during the measurement); flags u8 (bit 0 pose known, bit 1 still).
//   per zone (64, row order), 4 B: distance u8 (cm; 0 nothing or 0 mm, 255 beyond
//     2.54 m); signal u8 (kcps per SPAD, capped at 255); sigma u8 (mm, capped at
//     255); kind (bits 0-1, tof_kind_t) and targets (bits 2-4).
// The plan's 17 B header grew by 4: the period and the turn, so that watch and the
// PC know how long the frame measured and how far the robot turned meanwhile
// without a global rate (a frame at 15 Hz replayed among 10 Hz ones stays right).
#include <stdbool.h>
#include <stdint.h>
#include "tof_frame.h"

typedef enum { TOF_USE_MAP, TOF_USE_WATCH, TOF_USE_SNAPSHOT, TOF_USES } tof_use_t;
typedef enum { TOF_FORMAT_COMPACT, TOF_FORMATS } tof_format_t;

#define TOF_COMPACT_HEADER 21
#define TOF_COMPACT_ZONE   4
#define TOF_COMPACT_BYTES  (TOF_COMPACT_HEADER + TOF_ZONES * TOF_COMPACT_ZONE) // 277
#define TOF_FORMAT_MAX     TOF_COMPACT_BYTES // the largest format: room for any packed frame
#define TOF_COMPACT_BEYOND_MM 2550 // an unpacked distance this far means "beyond 2.54 m"

tof_format_t tof_format_of(tof_use_t use);   // the table use -> format
uint32_t tof_format_size(tof_format_t format); // a packed frame's bytes
// The message type a frame in this format travels in (the wire, step 4a, takes it).
uint8_t tof_format_message_type(tof_format_t format);

// Packs `f` for `use` into bytes (room for TOF_FORMAT_MAX); returns the bytes
// written: tof_format_size(tof_format_of(use)).
uint32_t tof_pack(tof_use_t use, const posed_frame_t *f, uint8_t *bytes);
// Unpacks `len` bytes in `format`; false (and *out untouched) if len is not its size.
bool tof_unpack(tof_format_t format, const uint8_t *bytes, uint32_t len, posed_frame_t *out);
