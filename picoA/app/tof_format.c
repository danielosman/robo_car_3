#include <math.h>
#include <string.h>
#include "units.h"
#include "tof_format.h"

// The table: each use's format. A new format for a use is one new entry here.
static const tof_format_t format_of_use[TOF_USES] = {
    [TOF_USE_MAP] = TOF_FORMAT_COMPACT,
    [TOF_USE_WATCH] = TOF_FORMAT_COMPACT,
    [TOF_USE_SNAPSHOT] = TOF_FORMAT_COMPACT,
};

static const struct { uint32_t size; uint8_t message_type; } formats[TOF_FORMATS] = {
    [TOF_FORMAT_COMPACT] = {TOF_COMPACT_BYTES, 0x40},
};

tof_format_t tof_format_of(tof_use_t use) { return format_of_use[use]; }
uint32_t tof_format_size(tof_format_t format) { return formats[format].size; }
uint8_t tof_format_message_type(tof_format_t format) { return formats[format].message_type; }

#define FLAG_POSE_KNOWN 1
#define FLAG_STILL      2
#define CENTI_DEG_PER_RAD (100.0f * DEG_PER_RAD)

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t get32(const uint8_t *p) { return get16(p) | (uint32_t)get16(p + 2) << 16; }

// Rounded to the nearest whole number, held within int16.
static int16_t to_i16(float v) {
    float r = floorf(v + 0.5f);
    return r >= 32767.0f ? 32767 : r <= -32767.0f ? -32767 : (int16_t)r;
}
static uint8_t cap8(uint16_t v) { return v > 255 ? 255 : (uint8_t)v; }

// −π < a ≤ π.
static float wrapped(float a) {
    a = fmodf(a, 2 * PI_F);
    if (a > PI_F) a -= 2 * PI_F;
    if (a <= -PI_F) a += 2 * PI_F;
    return a;
}

// mm to the distance byte: whole cm, at least 1 for a distance (0 is "nothing"),
// 255 from 2.545 m on.
static uint8_t distance_code(uint16_t mm) {
    if (mm == 0) return 0;
    uint32_t cm = ((uint32_t)mm + 5) / 10;
    return cm < 1 ? 1 : cm > 255 ? 255 : (uint8_t)cm;
}

static uint32_t pack_compact(const posed_frame_t *f, uint8_t *b) {
    const tof_frame_t *t = &f->frame;
    put32(b, t->number);
    put32(b + 4, t->t_us);
    uint32_t period = (t->period_us + 50) / 100;
    put16(b + 8, period > 0xFFFF ? 0xFFFF : (uint16_t)period);
    put16(b + 10, (uint16_t)to_i16(f->x_m * 1000));
    put16(b + 12, (uint16_t)to_i16(f->y_m * 1000));
    int16_t yaw = to_i16(wrapped(f->yaw_rad) * CENTI_DEG_PER_RAD);
    put16(b + 14, (uint16_t)(yaw == -18000 ? 18000 : yaw)); // −180° is 180°
    put16(b + 16, (uint16_t)to_i16(f->pitch_rad * CENTI_DEG_PER_RAD));
    put16(b + 18, (uint16_t)to_i16(f->turn_rad * CENTI_DEG_PER_RAD));
    b[20] = (uint8_t)((f->pose_known ? FLAG_POSE_KNOWN : 0) | (f->still ? FLAG_STILL : 0));
    uint8_t *z = b + TOF_COMPACT_HEADER;
    for (int i = 0; i < TOF_ZONES; i++, z += TOF_COMPACT_ZONE) {
        const tof_zone_t *zone = &t->zone[i];
        uint16_t mm = zone->kind == TOF_NOTHING ? 0 : zone->distance_mm;
        // A frame unpacked before: its "beyond" stays beyond.
        z[0] = mm >= t->beyond_mm ? 255 : distance_code(mm);
        z[1] = cap8(zone->signal);
        z[2] = cap8(zone->sigma_mm);
        z[3] = (uint8_t)((zone->kind & 3) | (zone->targets > 7 ? 7 : zone->targets) << 2);
    }
    return TOF_COMPACT_BYTES;
}

static void unpack_compact(const uint8_t *b, posed_frame_t *f) {
    tof_frame_t *t = &f->frame;
    t->number = get32(b);
    t->t_us = get32(b + 4);
    t->period_us = (uint32_t)get16(b + 8) * 100;
    t->beyond_mm = TOF_COMPACT_BEYOND_MM;
    f->x_m = (float)(int16_t)get16(b + 10) / 1000;
    f->y_m = (float)(int16_t)get16(b + 12) / 1000;
    f->yaw_rad = (float)(int16_t)get16(b + 14) / CENTI_DEG_PER_RAD;
    f->pitch_rad = (float)(int16_t)get16(b + 16) / CENTI_DEG_PER_RAD;
    f->turn_rad = (float)(int16_t)get16(b + 18) / CENTI_DEG_PER_RAD;
    f->pose_known = b[20] & FLAG_POSE_KNOWN;
    f->still = b[20] & FLAG_STILL;
    const uint8_t *z = b + TOF_COMPACT_HEADER;
    for (int i = 0; i < TOF_ZONES; i++, z += TOF_COMPACT_ZONE)
        t->zone[i] = (tof_zone_t){.distance_mm = (uint16_t)(z[0] * 10), .signal = z[1], .sigma_mm = z[2],
                                  .kind = z[3] & 3, .targets = (uint8_t)(z[3] >> 2 & 7)};
}

uint32_t tof_pack(tof_use_t use, const posed_frame_t *f, uint8_t *bytes) {
    switch (tof_format_of(use)) {
    case TOF_FORMAT_COMPACT: return pack_compact(f, bytes);
    default: return 0;
    }
}

bool tof_unpack(tof_format_t format, const uint8_t *bytes, uint32_t len, posed_frame_t *out) {
    if (format >= TOF_FORMATS || len != tof_format_size(format)) return false;
    switch (format) {
    case TOF_FORMAT_COMPACT: unpack_compact(bytes, out); return true;
    default: return false;
    }
}
