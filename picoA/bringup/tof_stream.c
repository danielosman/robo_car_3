// Bring-up ToF streaming on top of drivers/tof.c: settings from the viewer, and
// every frame as one binary packet on USB.
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "tof.h"
#include "tof_stream.h"

// Requested settings, kept across stop/start (the driver reports the sharpener as read back).
static tof_settings_t wanted = {.zones = 64, .hz = 10, .order = 1, .mode = 1, .integration_ms = 5, .sharpener = 5};
static uint32_t sequence;
// Explicit little-endian wire layout, independent of ULD struct padding.
// TOF3 | payload length:u32 | 16-byte metadata | zones * (9 + 4*10 bytes).
static uint8_t packet[8 + 16 + 64 * 49];
static uint8_t *out;
static void u8(uint8_t v) { *out++ = v; }
static void u16(uint16_t v) { u8(v); u8(v >> 8); }
static void u32(uint32_t v) { u16(v); u16(v >> 16); }

static void start(const tof_settings_t *s) {
    if (tof_start(s)) wanted = *s;
}

void tof_stream_init(void) {
    if (tof_init()) start(&wanted);
}

void tof_stream_command(const char *line) {
    while (*line == ' ') ++line;
    if (!strcmp(line, "init")) { tof_stream_init(); return; }
    if (!strcmp(line, "stop")) { tof_stop(); return; }
    if (!strcmp(line, "start")) { start(&wanted); return; }
    unsigned r, f, o, m, t, s; char extra;
    if (sscanf(line, "%u %u %u %u %u %u %c", &r, &f, &o, &m, &t, &s, &extra) == 6) {
        tof_settings_t new_settings = {(uint8_t)r, (uint8_t)f, (uint8_t)o, (uint8_t)m, (uint16_t)t, (uint8_t)s};
        start(&new_settings);
    } else {
        printf("TOF command: T init|start|stop OR T zones hz order mode integration_ms sharpener\n");
    }
}

void tof_stream_poll(void) {
    const VL53L8CX_ResultsData *results = tof_poll();
    if (!results) return;
    const tof_settings_t *s = tof_settings();
    out = packet;
    u8('T'); u8('O'); u8('F'); u8('3'); u32(16 + s->zones * 49);
    u8(1); u8(s->zones); u8(VL53L8CX_NB_TARGET_PER_ZONE); u8(s->hz);
    u8((uint8_t)results->silicon_temp_degc); u8(s->order); u8(s->mode); u8(s->sharpener);
    u16(s->integration_ms); u16(0); u32(sequence++);
    for (unsigned z = 0; z < s->zones; ++z) {
        u8(results->nb_target_detected[z]);
        u32(results->ambient_per_spad[z]);
        u32(results->nb_spads_enabled[z]);
        for (unsigned t = 0; t < VL53L8CX_NB_TARGET_PER_ZONE; ++t) {
            unsigned i = z * VL53L8CX_NB_TARGET_PER_ZONE + t;
            u16((uint16_t)results->distance_mm[i]);
            u16(results->range_sigma_mm[i]);
            u32(results->signal_per_spad[i]);
            u8(results->reflectance[i]); u8(results->target_status[i]);
        }
    }
    fwrite(packet, 1, (size_t)(out - packet), stdout);
    fflush(stdout);
}
