#pragma once
// The messages between the robot and the PC on their one TCP connection
// (doc/DATA_FLOW_PLAN.md, the connection): the envelope and every payload but the
// ToF frame's (tof_format.h packs that). Pure functions, no I/O: only this module
// knows the bytes. Its twin is pc/robot/src/wire.ts, with the same table; both are
// checked against the same bytes (picoA/app/test/data/wire_golden.hex).
//
// A message: [type u8][payload length u16][payload]. Little-endian. Floats are
// IEEE-754 f32. Text is NUL-padded where its field is fixed. Times (t) are u64 µs
// since PicoA's power-up (clock_since_start_of()): they never wrap.
//
// | type | name          | lane         | payload |
// |------|---------------|--------------|---------|
// | 0x01 | hello         | control      | wire version u8; boot id u32; build char[24] (29 B; any length passes the envelope, so the PC can read the version of any robot) |
// | 0x02 | console       | control      | one printed line without its '\n', 0-240 B (a longer line comes as several) |
// | 0x03 | answer        | control      | command n u32; end u8 (wire_end_t); has u8 (WIRE_HAS_*); x, y f32 m; yaw f32 rad; moved f32 m; rotated f32 rad; goal f32 m; snapshot u32 (34 B) |
// | 0x04 | status        | control      | x, y f32 m; yaw f32 rad (not wrapped); mode char[8]; action char[12] ("" none); WiFi signal i8 dBm (0 unknown) (33 B) |
// | 0x05 | map cleared   | control      | none |
// | 0x06 | move start    | control      | move n u32; command n u32; kind u8 (wire_move_kind_t); parameter f32 (drive m, turn rad, go to: m to the goal, watch s); t u64; pre_ms u16; post_ms u16; flags u8 (bit 0 refused: it ends at once, a running move goes on) (26 B) |
// | 0x07 | move end      | control      | move n u32; end u8 (wire_end_t); t u64; frames sent u32; frames dropped u32; moved f32 m; rotated f32 rad; imu batches sent u32; imu batches dropped u32 (37 B) |
// | 0x10 | snapshot part | bulk         | the next 1-1400 bytes of a snapshot (snapshot.h); a snapshot's parts in order |
// | 0x11 | imu batch     | bulk         | move n u32; first record's t u64; period u16 µs; count u8 (1-10); then per record acceleration x, y, z, gyro x, y, z i16 (raw counts, link_messages.h) (15 + 12 × count B); only during a move (move.h) |
// | 0x12 | map part      | bulk         | part u8 (0-8); origin_cx, origin_cy i16; then 1200 bytes of the robot's window (cell_map.h, 9 parts = 3 × 60 × 60 B in order) (1205 B) |
// | 0x40 | tof compact   | bulk         | move n u32; t u64 (the frame's middle); then a compact tof frame (tof_format.h; its types come from tof_format_message_type()) (12 + 277 B); only during a move (move.h) |
// | 0x20 | command       | to the robot | command n u32; the command's text, 1-46 printable ASCII (commands.h) |
// | 0x21 | heartbeat     | to the robot | none (every 2 s: the robot closes after 30 s of nothing) |
//
// Lanes (link_pc.h): control goes first; bulk is a queue behind it.
//
// Both directions are here, so one file owns the bytes: the robot uses the encoders
// of its messages and the decoder of the command; the rest (the decoders of its own
// messages, the encoders of command and heartbeat) are the PC's side, for the host
// tests. The linker leaves those out of the firmware.
//
// Every tof frame and imu batch names its move: the PC files it by that number,
// however late it comes. An imu batch's records are evenly spaced: record i was
// measured at t + i × period.
#include <stdbool.h>
#include <stdint.h>
#include "link_messages.h"
#include "tof_format.h"

#define WIRE_VERSION 5 // up when any payload or the table changes; the PC refuses another

#define WIRE_HEADER      3
#define WIRE_PAYLOAD_MAX 1400 // the longest payload of any message (a snapshot part)
#define WIRE_MESSAGE_MAX     (WIRE_HEADER + WIRE_PAYLOAD_MAX)

enum {
    WIRE_HELLO = 0x01,
    WIRE_CONSOLE = 0x02,
    WIRE_ANSWER = 0x03,
    WIRE_STATUS = 0x04,
    WIRE_MAP_CLEARED = 0x05,
    WIRE_MOVE_START = 0x06,
    WIRE_MOVE_END = 0x07,
    WIRE_SNAPSHOT_PART = 0x10,
    WIRE_IMU_BATCH = 0x11,
    WIRE_MAP_PART = 0x12,
    WIRE_COMMAND = 0x20,
    WIRE_HEARTBEAT = 0x21,
};

#define WIRE_HELLO_SIZE    29
#define WIRE_BUILD_LEN     24
#define WIRE_CONSOLE_MAX   240
#define WIRE_ANSWER_SIZE   34
#define WIRE_STATUS_SIZE   33
#define WIRE_MOVE_START_SIZE 26
#define WIRE_MOVE_END_SIZE 37
#define WIRE_IMU_BATCH_HEADER 15
#define WIRE_TOF_HEADER    12 // move n, t; the packed frame follows
#define WIRE_IMU_RECORD    12
#define WIRE_IMU_BATCH_MAX 10 // records in a batch (~0.1 s)
#define WIRE_MAP_PARTS     9
#define WIRE_MAP_PART_CELLS 1200
#define WIRE_MAP_PART_SIZE (5 + WIRE_MAP_PART_CELLS)
#define WIRE_MODE_LEN      8
#define WIRE_ACTION_LEN    12
#define WIRE_PART_MAX      1400
#define WIRE_COMMAND_TEXT_MAX 46

typedef enum { WIRE_LANE_CONTROL, WIRE_LANE_BULK, WIRE_LANE_TO_ROBOT } wire_lane_t;

typedef struct {
    wire_lane_t lane;
    uint16_t min, max;  // payload length
} wire_type_info_t;

// What the table says of a type; false for a type it doesn't know.
bool wire_info(uint8_t type, wire_type_info_t *info);
// The table's types are all different (the ToF's from tof_format included).
bool wire_check(void);

// --- the envelope ---

typedef enum { WIRE_UNKNOWN_TYPE = -1, WIRE_TOO_LONG = -2, WIRE_TOO_SHORT = -3 } wire_error_t;
typedef struct {
    uint8_t type;
    const uint8_t *payload; // inside the bytes given to wire_take()
    uint16_t len;
} wire_message_t;
// The first message in bytes[0..len): > 0 its length (header and payload; it is in
// *m), 0 if it isn't whole yet, < 0 a wire_error_t (the stream is broken from here).
int32_t wire_take(const uint8_t *bytes, uint32_t len, wire_message_t *m);
const char *wire_error_text(int32_t error);
// The type of a whole message made by the encoders below (its first byte).
static inline uint8_t wire_type(const uint8_t *message) { return message[0]; }
// Writes the header: type, payload length; returns WIRE_HEADER. The payload follows.
uint32_t wire_header(uint8_t *out, uint8_t type, uint16_t len);

// --- the payloads: each encoder writes a whole message into out (room for
// WIRE_HEADER + its payload) and returns its length; each decoder reads a message
// wire_take() found and returns false if it isn't that type or its length is wrong.

typedef struct {
    uint8_t version;              // WIRE_VERSION
    uint32_t boot_id;
    char build[WIRE_BUILD_LEN + 1]; // when the firmware was built
} wire_hello_t;
uint32_t wire_encode_hello(const wire_hello_t *h, uint8_t *out);
bool wire_decode_hello(const wire_message_t *m, wire_hello_t *h);

// text: n bytes (more than WIRE_CONSOLE_MAX are cut), without '\n'.
uint32_t wire_encode_console(const char *text, uint32_t n, uint8_t *out);
// The line into text (room for WIRE_CONSOLE_MAX + 1), NUL-terminated.
bool wire_decode_console(const wire_message_t *m, char *text);

// How a command ended (the PC's answers name them: pc/robot/src/api/answers.ts).
typedef enum {
    WIRE_END_OK, WIRE_END_OBSTACLE, WIRE_END_STOPPED, WIRE_END_REPLACED, WIRE_END_TILT, WIRE_END_WHEELS,
    WIRE_END_NO_DRIVE, WIRE_END_PICOB_LOST, WIRE_END_NO_START, WIRE_END_REFUSED, WIRE_END_SAFETY, WIRE_END_ENDED,
    WIRE_ENDS
} wire_end_t;
#define WIRE_HAS_MOVE     1 // moved and rotated: a move
#define WIRE_HAS_GOAL     2 // a go to
#define WIRE_HAS_SNAPSHOT 4 // a snapshot
typedef struct {
    uint32_t n;            // the command's number
    uint8_t end;           // wire_end_t
    uint8_t has;           // WIRE_HAS_*
    float x_m, y_m, yaw_rad; // where the robot is (a snapshot's: where it was for it)
    float moved_m, rotated_rad, goal_m;
    uint32_t snapshot;     // its number
} wire_answer_t;
uint32_t wire_encode_answer(const wire_answer_t *a, uint8_t *out);
bool wire_decode_answer(const wire_message_t *m, wire_answer_t *a);

typedef struct {
    float x_m, y_m, yaw_rad;
    char mode[WIRE_MODE_LEN + 1];
    char action[WIRE_ACTION_LEN + 1]; // "" when none runs
    int8_t rssi_dbm;                  // 0: not known
} wire_status_t;
uint32_t wire_encode_status(const wire_status_t *s, uint8_t *out);
bool wire_decode_status(const wire_message_t *m, wire_status_t *s);

uint32_t wire_encode_map_cleared(uint8_t *out);

// A move (move.h): its start and its end.
typedef enum { WIRE_MOVE_DRIVE, WIRE_MOVE_ROTATE, WIRE_MOVE_GOTO, WIRE_MOVE_WATCH, WIRE_MOVE_KINDS } wire_move_kind_t;
typedef struct {
    uint32_t move;         // counts the moves since power-up, from 1
    uint32_t command;      // the number of the command that started it (0: typed on USB without one)
    uint8_t kind;          // wire_move_kind_t
    float parameter;           // drive: m (+ = forward); rotate: rad (+ = left); go to: m to the goal; watch: s
    uint64_t t_us;         // when it started (since power-up)
    uint16_t pre_ms, post_ms; // the still times it stands before and after
    uint8_t flags;         // WIRE_MOVE_REFUSED
} wire_move_start_t;
#define WIRE_MOVE_REFUSED 1 // it can't start: its end follows at once, and a running move goes on (it stays the open one)
uint32_t wire_encode_move_start(const wire_move_start_t *s, uint8_t *out);
bool wire_decode_move_start(const wire_message_t *m, wire_move_start_t *s);

typedef struct {
    uint32_t move;
    uint8_t end;           // wire_end_t, as its command's answer
    uint64_t t_us;         // when it ended (since power-up)
    uint32_t frames_sent;  // tof frames handed to the bulk lane during the move
    uint32_t frames_dropped; // ...and refused by it (full, or no server)
    float moved_m, rotated_rad; // as the answer's
    uint32_t imu_sent, imu_dropped; // imu batches handed to the bulk lane during the move, and refused by it
} wire_move_end_t;
uint32_t wire_encode_move_end(const wire_move_end_t *e, uint8_t *out);
bool wire_decode_move_end(const wire_message_t *m, wire_move_end_t *e);

// Up to WIRE_IMU_BATCH_MAX imu records of a move, oldest first, evenly spaced:
// record i was measured at t_us + i × period_us. The records' own t_us is not on
// the wire (the encoder ignores it, the decoder leaves it 0).
typedef struct {
    uint32_t move;
    uint64_t t_us;         // the first record's time, since power-up
    uint16_t period_us;
    uint8_t n;             // records, 1-WIRE_IMU_BATCH_MAX
    imu_record_t records[WIRE_IMU_BATCH_MAX];
} wire_imu_batch_t;
uint32_t wire_encode_imu_batch(const wire_imu_batch_t *b, uint8_t *out); // 0 if n is out of range
bool wire_decode_imu_batch(const wire_message_t *m, wire_imu_batch_t *b);

// A tof frame of a move, packed for `use` (tof_format.h), in its message (room for
// WIRE_HEADER + WIRE_TOF_HEADER + TOF_FORMAT_MAX). t_us: the frame's middle, since
// power-up.
uint32_t wire_encode_tof(tof_use_t use, uint32_t move, uint64_t t_us, const posed_frame_t *f, uint8_t *out);

// n bytes of a snapshot (1-WIRE_PART_MAX); 0 if n is out of range.
uint32_t wire_encode_snapshot_part(const uint8_t *bytes, uint32_t n, uint8_t *out);

// One ninth of the robot's 3 × 60 × 60 cell window (part 0..WIRE_MAP_PARTS - 1).
typedef struct {
    uint8_t part;
    int16_t origin_cx, origin_cy;
    uint8_t cells[WIRE_MAP_PART_CELLS];
} wire_map_part_t;
uint32_t wire_encode_map_part(uint8_t part, int16_t origin_cx, int16_t origin_cy, const uint8_t *cells, uint8_t *out);
bool wire_decode_map_part(const wire_message_t *m, wire_map_part_t *p);

typedef struct {
    uint32_t n;
    char text[WIRE_COMMAND_TEXT_MAX + 1];
} wire_command_t;
// 0 if text is empty, too long or not printable ASCII.
uint32_t wire_encode_command(const wire_command_t *c, uint8_t *out);
bool wire_decode_command(const wire_message_t *m, wire_command_t *c);

uint32_t wire_encode_heartbeat(uint8_t *out);
