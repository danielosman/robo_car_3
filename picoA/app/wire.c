#include <string.h>
#include "wire.h"

// The table (wire.h); the ToF frames' types come from tof_format.
static const struct { uint8_t type; wire_type_info_t info; } table[] = {
    {WIRE_HELLO, {WIRE_LANE_CONTROL, 1, WIRE_PAYLOAD_MAX, false}}, // any length: another version's hello is read for its version
    {WIRE_CONSOLE, {WIRE_LANE_CONTROL, 0, WIRE_CONSOLE_MAX, true}},
    {WIRE_ANSWER, {WIRE_LANE_CONTROL, WIRE_ANSWER_SIZE, WIRE_ANSWER_SIZE, false}},
    {WIRE_STATUS, {WIRE_LANE_CONTROL, WIRE_STATUS_SIZE, WIRE_STATUS_SIZE, true}},
    {WIRE_MAP_CLEARED, {WIRE_LANE_CONTROL, 0, 0, false}},
    {WIRE_MOVE_START, {WIRE_LANE_CONTROL, WIRE_MOVE_START_SIZE, WIRE_MOVE_START_SIZE, false}},
    {WIRE_MOVE_END, {WIRE_LANE_CONTROL, WIRE_MOVE_END_SIZE, WIRE_MOVE_END_SIZE, false}},
    {WIRE_SNAPSHOT_PART, {WIRE_LANE_BULK, 1, WIRE_PART_MAX, false}},
    {WIRE_IMU_BATCH, {WIRE_LANE_BULK, WIRE_IMU_BATCH_HEADER + WIRE_IMU_RECORD,
                      WIRE_IMU_BATCH_HEADER + WIRE_IMU_BATCH_MAX * WIRE_IMU_RECORD, true}},
    {WIRE_COMMAND, {WIRE_LANE_TO_ROBOT, 5, 4 + WIRE_COMMAND_TEXT_MAX, false}},
    {WIRE_HEARTBEAT, {WIRE_LANE_TO_ROBOT, 0, 0, false}},
};
#define TABLE_N (sizeof table / sizeof table[0])

bool wire_info(uint8_t type, wire_type_info_t *info) {
    for (uint32_t i = 0; i < TABLE_N; i++)
        if (table[i].type == type) { *info = table[i].info; return true; }
    for (int f = 0; f < TOF_FORMATS; f++) {
        if (tof_format_message_type((tof_format_t)f) != type) continue;
        uint16_t size = (uint16_t)tof_format_size((tof_format_t)f);
        *info = (wire_type_info_t){WIRE_LANE_BULK, size, size, true};
        return true;
    }
    return false;
}

bool wire_check(void) {
    uint8_t types[TABLE_N + TOF_FORMATS];
    uint32_t n = 0;
    for (uint32_t i = 0; i < TABLE_N; i++) types[n++] = table[i].type;
    for (int f = 0; f < TOF_FORMATS; f++) {
        types[n++] = tof_format_message_type((tof_format_t)f);
        if (tof_format_size((tof_format_t)f) > WIRE_PAYLOAD_MAX) return false;
    }
    for (uint32_t i = 0; i < n; i++)
        for (uint32_t j = i + 1; j < n; j++)
            if (types[i] == types[j]) return false;
    return true;
}

// --- bytes ---

static void set16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void set32(uint8_t *p, uint32_t v) { set16(p, (uint16_t)v); set16(p + 2, (uint16_t)(v >> 16)); }
static void setf(uint8_t *p, float v) { uint32_t u; memcpy(&u, &v, 4); set32(p, u); }
static uint16_t read16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t read32(const uint8_t *p) { return read16(p) | (uint32_t)read16(p + 2) << 16; }
static float readf(const uint8_t *p) { uint32_t u = read32(p); float v; memcpy(&v, &u, 4); return v; }
static uint32_t text_len(const char *text, uint32_t size) {
    uint32_t n = 0;
    while (n < size && text[n]) n++;
    return n;
}
// A fixed text field: NUL-padded out of a NUL-terminated string (cut to size).
static void set_text(uint8_t *p, const char *text, uint32_t size) {
    uint32_t n = text_len(text, size);
    memcpy(p, text, n);
    memset(p + n, 0, size - n);
}
static void read_text(const uint8_t *p, uint32_t size, char *text) {
    uint32_t n = text_len((const char *)p, size);
    memcpy(text, p, n);
    text[n] = 0;
}

uint32_t wire_header(uint8_t *out, uint8_t type, uint16_t len) {
    out[0] = type;
    set16(out + 1, len);
    return WIRE_HEADER;
}

int32_t wire_take(const uint8_t *bytes, uint32_t len, wire_msg_t *m) {
    if (len < 1) return 0;
    wire_type_info_t info;
    if (!wire_info(bytes[0], &info)) return WIRE_UNKNOWN_TYPE;
    if (len < WIRE_HEADER) return 0;
    uint16_t n = read16(bytes + 1);
    if (n > info.max) return WIRE_TOO_LONG;
    if (n < info.min) return WIRE_TOO_SHORT;
    if (len < WIRE_HEADER + (uint32_t)n) return 0;
    *m = (wire_msg_t){.type = bytes[0], .payload = bytes + WIRE_HEADER, .len = n};
    return WIRE_HEADER + n;
}

const char *wire_error_text(int32_t error) {
    switch (error) {
    case WIRE_UNKNOWN_TYPE: return "unknown message type";
    case WIRE_TOO_LONG: return "message too long for its type";
    case WIRE_TOO_SHORT: return "message too short for its type";
    default: return "no error";
    }
}

static bool is(const wire_msg_t *m, uint8_t type, uint16_t len) { return m->type == type && m->len == len; }

// --- the payloads ---

uint32_t wire_encode_hello(const wire_hello_t *h, uint8_t *out) {
    uint8_t *p = out + wire_header(out, WIRE_HELLO, WIRE_HELLO_SIZE);
    p[0] = h->version;
    set32(p + 1, h->boot_id);
    set_text(p + 5, h->build, WIRE_BUILD_LEN);
    return WIRE_HEADER + WIRE_HELLO_SIZE;
}

bool wire_decode_hello(const wire_msg_t *m, wire_hello_t *h) {
    if (!is(m, WIRE_HELLO, WIRE_HELLO_SIZE)) return false;
    h->version = m->payload[0];
    h->boot_id = read32(m->payload + 1);
    read_text(m->payload + 5, WIRE_BUILD_LEN, h->build);
    return true;
}

uint32_t wire_encode_console(const char *text, uint32_t n, uint8_t *out) {
    if (n > WIRE_CONSOLE_MAX) n = WIRE_CONSOLE_MAX;
    wire_header(out, WIRE_CONSOLE, (uint16_t)n);
    memcpy(out + WIRE_HEADER, text, n);
    return WIRE_HEADER + n;
}

bool wire_decode_console(const wire_msg_t *m, char *text) {
    if (m->type != WIRE_CONSOLE || m->len > WIRE_CONSOLE_MAX) return false;
    memcpy(text, m->payload, m->len);
    text[m->len] = 0;
    return true;
}

uint32_t wire_encode_answer(const wire_answer_t *a, uint8_t *out) {
    uint8_t *p = out + wire_header(out, WIRE_ANSWER, WIRE_ANSWER_SIZE);
    set32(p, a->n);
    p[4] = a->end;
    p[5] = a->has;
    setf(p + 6, a->x_m);
    setf(p + 10, a->y_m);
    setf(p + 14, a->yaw_rad);
    setf(p + 18, a->moved_m);
    setf(p + 22, a->rotated_rad);
    setf(p + 26, a->goal_m);
    set32(p + 30, a->snapshot);
    return WIRE_HEADER + WIRE_ANSWER_SIZE;
}

bool wire_decode_answer(const wire_msg_t *m, wire_answer_t *a) {
    if (!is(m, WIRE_ANSWER, WIRE_ANSWER_SIZE)) return false;
    const uint8_t *p = m->payload;
    *a = (wire_answer_t){.n = read32(p), .end = p[4], .has = p[5], .x_m = readf(p + 6), .y_m = readf(p + 10),
                         .yaw_rad = readf(p + 14), .moved_m = readf(p + 18), .rotated_rad = readf(p + 22),
                         .goal_m = readf(p + 26), .snapshot = read32(p + 30)};
    return true;
}

uint32_t wire_encode_status(const wire_status_t *s, uint8_t *out) {
    uint8_t *p = out + wire_header(out, WIRE_STATUS, WIRE_STATUS_SIZE);
    setf(p, s->x_m);
    setf(p + 4, s->y_m);
    setf(p + 8, s->yaw_rad);
    set_text(p + 12, s->mode, WIRE_MODE_LEN);
    set_text(p + 20, s->action, WIRE_ACTION_LEN);
    p[32] = (uint8_t)s->rssi_dbm;
    return WIRE_HEADER + WIRE_STATUS_SIZE;
}

bool wire_decode_status(const wire_msg_t *m, wire_status_t *s) {
    if (!is(m, WIRE_STATUS, WIRE_STATUS_SIZE)) return false;
    const uint8_t *p = m->payload;
    s->x_m = readf(p);
    s->y_m = readf(p + 4);
    s->yaw_rad = readf(p + 8);
    read_text(p + 12, WIRE_MODE_LEN, s->mode);
    read_text(p + 20, WIRE_ACTION_LEN, s->action);
    s->rssi_dbm = (int8_t)p[32];
    return true;
}

uint32_t wire_encode_map_cleared(uint8_t *out) { return wire_header(out, WIRE_MAP_CLEARED, 0); }

uint32_t wire_encode_move_start(const wire_move_start_t *s, uint8_t *out) {
    uint8_t *p = out + wire_header(out, WIRE_MOVE_START, WIRE_MOVE_START_SIZE);
    set32(p, s->move);
    set32(p + 4, s->command);
    p[8] = s->kind;
    setf(p + 9, s->param);
    set32(p + 13, s->t_us);
    set16(p + 17, s->pre_ms);
    set16(p + 19, s->post_ms);
    p[21] = s->flags;
    return WIRE_HEADER + WIRE_MOVE_START_SIZE;
}

bool wire_decode_move_start(const wire_msg_t *m, wire_move_start_t *s) {
    if (!is(m, WIRE_MOVE_START, WIRE_MOVE_START_SIZE)) return false;
    const uint8_t *p = m->payload;
    *s = (wire_move_start_t){.move = read32(p), .command = read32(p + 4), .kind = p[8], .param = readf(p + 9),
                             .t_us = read32(p + 13), .pre_ms = read16(p + 17), .post_ms = read16(p + 19), .flags = p[21]};
    return true;
}

uint32_t wire_encode_move_end(const wire_move_end_t *e, uint8_t *out) {
    uint8_t *p = out + wire_header(out, WIRE_MOVE_END, WIRE_MOVE_END_SIZE);
    set32(p, e->move);
    p[4] = e->end;
    set32(p + 5, e->t_us);
    set32(p + 9, e->frames_sent);
    set32(p + 13, e->frames_dropped);
    set32(p + 17, e->first_frame);
    set32(p + 21, e->last_frame);
    setf(p + 25, e->moved_m);
    setf(p + 29, e->rotated_rad);
    set32(p + 33, e->imu_sent);
    set32(p + 37, e->imu_dropped);
    return WIRE_HEADER + WIRE_MOVE_END_SIZE;
}

bool wire_decode_move_end(const wire_msg_t *m, wire_move_end_t *e) {
    if (!is(m, WIRE_MOVE_END, WIRE_MOVE_END_SIZE)) return false;
    const uint8_t *p = m->payload;
    *e = (wire_move_end_t){.move = read32(p), .end = p[4], .t_us = read32(p + 5), .frames_sent = read32(p + 9),
                           .frames_dropped = read32(p + 13), .first_frame = read32(p + 17), .last_frame = read32(p + 21),
                           .moved_m = readf(p + 25), .rotated_rad = readf(p + 29), .imu_sent = read32(p + 33),
                           .imu_dropped = read32(p + 37)};
    return true;
}

uint32_t wire_encode_imu_batch(const wire_imu_batch_t *b, uint8_t *out) {
    if (b->n < 1 || b->n > WIRE_IMU_BATCH_MAX) return 0;
    uint16_t len = (uint16_t)(WIRE_IMU_BATCH_HEADER + b->n * WIRE_IMU_RECORD);
    uint8_t *p = out + wire_header(out, WIRE_IMU_BATCH, len);
    uint32_t first = b->records[0].t_us, span = b->records[b->n - 1].t_us - first;
    uint32_t period = b->n > 1 ? (span + (b->n - 1) / 2u) / (b->n - 1u) : IMU_RECORD_PERIOD_US;
    set32(p, b->move);
    set32(p + 4, first);
    set16(p + 8, (uint16_t)(period > 0xffff ? 0xffff : period));
    p[10] = b->n;
    for (int i = 0; i < b->n; i++) {
        uint8_t *r = p + WIRE_IMU_BATCH_HEADER + i * WIRE_IMU_RECORD;
        for (int k = 0; k < 3; k++) {
            set16(r + 2 * k, (uint16_t)b->records[i].accel[k]);
            set16(r + 6 + 2 * k, (uint16_t)b->records[i].gyro[k]);
        }
    }
    return WIRE_HEADER + len;
}

bool wire_decode_imu_batch(const wire_msg_t *m, wire_imu_batch_t *b) {
    if (m->type != WIRE_IMU_BATCH || m->len < WIRE_IMU_BATCH_HEADER) return false;
    const uint8_t *p = m->payload;
    uint8_t n = p[10];
    if (n < 1 || n > WIRE_IMU_BATCH_MAX || m->len != WIRE_IMU_BATCH_HEADER + n * WIRE_IMU_RECORD) return false;
    uint32_t first = read32(p + 4), period = read16(p + 8);
    b->move = read32(p);
    b->n = n;
    for (int i = 0; i < n; i++) {
        const uint8_t *r = p + WIRE_IMU_BATCH_HEADER + i * WIRE_IMU_RECORD;
        b->records[i].t_us = first + (uint32_t)i * period;
        for (int k = 0; k < 3; k++) {
            b->records[i].accel[k] = (int16_t)read16(r + 2 * k);
            b->records[i].gyro[k] = (int16_t)read16(r + 6 + 2 * k);
        }
    }
    return true;
}

uint32_t wire_encode_tof(tof_use_t use, const posed_frame_t *f, uint8_t *out) {
    uint32_t n = tof_pack(use, f, out + WIRE_HEADER);
    wire_header(out, tof_format_message_type(tof_format_of(use)), (uint16_t)n);
    return WIRE_HEADER + n;
}

uint32_t wire_encode_snapshot_part(const uint8_t *bytes, uint32_t n, uint8_t *out) {
    if (n < 1 || n > WIRE_PART_MAX) return 0;
    wire_header(out, WIRE_SNAPSHOT_PART, (uint16_t)n);
    memcpy(out + WIRE_HEADER, bytes, n);
    return WIRE_HEADER + n;
}

static bool command_text_ok(const char *text, uint32_t n) {
    if (n < 1 || n > WIRE_COMMAND_TEXT_MAX) return false;
    for (uint32_t i = 0; i < n; i++)
        if (text[i] < 0x20 || text[i] > 0x7e) return false;
    return true;
}

uint32_t wire_encode_command(const wire_command_t *c, uint8_t *out) {
    uint32_t n = text_len(c->text, sizeof c->text);
    if (!command_text_ok(c->text, n)) return 0;
    uint8_t *p = out + wire_header(out, WIRE_COMMAND, (uint16_t)(4 + n));
    set32(p, c->n);
    memcpy(p + 4, c->text, n);
    return WIRE_HEADER + 4 + n;
}

bool wire_decode_command(const wire_msg_t *m, wire_command_t *c) {
    if (m->type != WIRE_COMMAND || m->len < 5) return false;
    uint32_t n = m->len - 4u;
    if (!command_text_ok((const char *)m->payload + 4, n)) return false;
    c->n = read32(m->payload);
    memcpy(c->text, m->payload + 4, n);
    c->text[n] = 0;
    return true;
}

uint32_t wire_encode_heartbeat(uint8_t *out) { return wire_header(out, WIRE_HEARTBEAT, 0); }
