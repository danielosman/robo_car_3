// VL53L8CX on PicoA SPI0. No LPn GPIO: the Pololu carrier holds it high.
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "vl53l8cx_api.h"
#include "tof.h"

static VL53L8CX_Configuration dev;
static VL53L8CX_ResultsData results;
static bool initialized, running;
static uint8_t resolution = 64, frequency = 10, order = 1, mode = 1, sharpener = 5;
static uint16_t integration = 5;
static uint8_t sharpener_actual = 5; // ULD percentage readback is quantized.
static uint32_t sequence;
static absolute_time_t next_poll, last_data;
static unsigned errors;
// Explicit little-endian wire layout, independent of ULD struct padding.
// TOF3 | payload length:u32 | 16-byte metadata | zones * (9 + 4*10 bytes).
static uint8_t packet[8 + 16 + 64 * 49];
static uint8_t *out;
static void u8(uint8_t v) { *out++ = v; }
static void u16(uint16_t v) { u8(v); u8(v >> 8); }
static void u32(uint32_t v) { u16(v); u16(v >> 16); }

static bool check(uint8_t status, const char *operation) {
    if (!status) return true;
    printf("TOF error: %s (status %u). Try reinitialize; a power cycle may be needed.\n",
           operation, status);
    return false;
}
static bool stop(void) {
    if (!running) return true;
    if (!check(vl53l8cx_stop_ranging(&dev), "stop")) return false;
    running = false;
    printf("TOF stopped\n");
    return true;
}
static bool configure(unsigned res, unsigned hz, unsigned ord, unsigned md,
                      unsigned ms, unsigned sharp) {
    if ((res != 16 && res != 64) || hz < 1 || hz > (res == 64 ? 15 : 60) ||
        (ord != 1 && ord != 2) || (md != 1 && md != 3) ||
        ms < 2 || ms > 1000 || sharp > 99 ||
        (md == 3 && ((res == 64 ? 4 : 1) * ms + 1) * hz >= 1000)) {
        printf("TOF invalid settings: 4x4 1-60Hz; 8x8 1-15Hz; autonomous integration must fit period\n");
        return false;
    }
    if (!initialized) { printf("TOF not initialized; use Reinitialize\n"); return false; }
    if (!stop()) return false;
    if (!check(vl53l8cx_set_resolution(&dev, res), "resolution") ||
        !check(vl53l8cx_set_ranging_frequency_hz(&dev, hz), "frequency") ||
        !check(vl53l8cx_set_target_order(&dev, ord), "target order") ||
        !check(vl53l8cx_set_ranging_mode(&dev, md), "ranging mode") ||
        !check(vl53l8cx_set_integration_time_ms(&dev, ms), "integration") ||
        !check(vl53l8cx_set_sharpener_percent(&dev, sharp), "sharpener")) return false;
    // Confirm active settings before advertising them in telemetry.
    uint8_t r, f, o, m, s; uint32_t t;
    if (!check(vl53l8cx_get_resolution(&dev, &r), "read resolution") ||
        !check(vl53l8cx_get_ranging_frequency_hz(&dev, &f), "read frequency") ||
        !check(vl53l8cx_get_target_order(&dev, &o), "read target order") ||
        !check(vl53l8cx_get_ranging_mode(&dev, &m), "read mode") ||
        !check(vl53l8cx_get_integration_time_ms(&dev, &t), "read integration") ||
        !check(vl53l8cx_get_sharpener_percent(&dev, &s), "read sharpener")) return false;
    if (r != res || f != hz || o != ord || m != md || t != ms) {
        printf("TOF configuration readback mismatch; ranging remains stopped\n"); return false;
    }
    resolution = r; frequency = f; order = o; mode = m; integration = t;
    sharpener = sharp; sharpener_actual = s; // Preserve requested value across Stop/Start.
    if (!check(vl53l8cx_start_ranging(&dev), "start")) return false;
    running = true;
    errors = 0;
    last_data = get_absolute_time(); next_poll = last_data;
    printf("TOF running: %ux%u %uHz, %s order, 4 targets/zone, %s mode, integration %ums, sharpener %u%%\n",
           res == 64 ? 8 : 4, res == 64 ? 8 : 4, frequency,
           order == 1 ? "closest" : "strongest", mode == 1 ? "continuous" : "autonomous",
           integration, sharpener_actual);
    return true;
}
void tof_init(void) {
    if (!stop()) return;
    initialized = false;
    printf("TOF initializing: SPI0 GP16/17/18/19, INT GP20 input; LPn not connected\n");
    if (!check(vl53l8cx_pico_init(&dev.platform), "platform")) return;
    sleep_ms(100);
    uint8_t alive = 0;
    if (!check(vl53l8cx_is_alive(&dev, &alive), "ID read") || !alive) {
        printf("TOF not detected. Check carrier power, SPI wiring and SPI_I2C_N/LPn pullups.\n");
        return;
    }
    printf("TOF detected; uploading sensor firmware (~84KB)...\n");
    if (!check(vl53l8cx_init(&dev), "firmware initialization")) return;
    initialized = true;
    configure(resolution, frequency, order, mode, integration, sharpener);
}
void tof_command(const char *line) {
    while (*line == ' ') ++line;
    if (!strcmp(line, "init")) { tof_init(); return; }
    if (!strcmp(line, "stop")) { stop(); return; }
    if (!strcmp(line, "start")) {
        configure(resolution, frequency, order, mode, integration, sharpener); return;
    }
    unsigned r, f, o, m, t, s; char extra;
    if (sscanf(line, "%u %u %u %u %u %u %c", &r, &f, &o, &m, &t, &s, &extra) == 6)
        configure(r, f, o, m, t, s);
    else printf("TOF command: T init|start|stop OR T zones hz order mode integration_ms sharpener\n");
}
void tof_poll(void) {
    if (!running || !time_reached(next_poll)) return;
    next_poll = make_timeout_time_ms(5);
    uint8_t ready = 0;
    uint8_t status = vl53l8cx_check_data_ready(&dev, &ready);
    if (!status && ready) status = vl53l8cx_get_ranging_data(&dev, &results);
    if (status) {
        if (++errors >= 3) {
            check(status, "repeated ranging read failures");
            stop();
            // Stop polling on a fault even if stop itself failed; retry via Init.
            running = false; initialized = false;
        }
        return;
    }
    errors = 0;
    if (!ready) {
        if (absolute_time_diff_us(last_data, get_absolute_time()) > 3000000) {
            printf("TOF no fresh data for 3s; check power/SPI, or use Reinitialize\n");
            last_data = get_absolute_time();
        }
        return;
    }
    last_data = get_absolute_time();
    out = packet;
    u8('T'); u8('O'); u8('F'); u8('3'); u32(16 + resolution * 49);
    u8(1); u8(resolution); u8(VL53L8CX_NB_TARGET_PER_ZONE); u8(frequency);
    u8((uint8_t)results.silicon_temp_degc); u8(order); u8(mode); u8(sharpener_actual);
    u16(integration); u16(0); u32(sequence++);
    for (unsigned z = 0; z < resolution; ++z) {
        u8(results.nb_target_detected[z]);
        u32(results.ambient_per_spad[z]);
        u32(results.nb_spads_enabled[z]);
        for (unsigned t = 0; t < VL53L8CX_NB_TARGET_PER_ZONE; ++t) {
            unsigned i = z * VL53L8CX_NB_TARGET_PER_ZONE + t;
            u16((uint16_t)results.distance_mm[i]);
            u16(results.range_sigma_mm[i]);
            u32(results.signal_per_spad[i]);
            u8(results.reflectance[i]); u8(results.target_status[i]);
        }
    }
    fwrite(packet, 1, (size_t)(out - packet), stdout);
    fflush(stdout);
}
