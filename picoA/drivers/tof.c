// VL53L8CX on PicoA SPI0. No LPn GPIO: the Pololu carrier holds it high.
#include <stdio.h>
#include "pico/stdlib.h"
#include "tof.h"

#define POLL_US       5000    // data-ready checks
#define NO_DATA_US    3000000 // warn when frames stop arriving

static VL53L8CX_Configuration dev;
static VL53L8CX_ResultsData results;
static tof_settings_t active;
static bool initialized, running;
static absolute_time_t next_poll, last_data;
static unsigned errors;

static bool check(uint8_t status, const char *operation) {
    if (!status) return true;
    printf("TOF error: %s (status %u). Try reinitialize; a power cycle may be needed.\n",
           operation, status);
    return false;
}

bool tof_stop(void) {
    if (!running) return true;
    if (!check(vl53l8cx_stop_ranging(&dev), "stop")) return false;
    running = false;
    printf("TOF stopped\n");
    return true;
}

bool tof_start(const tof_settings_t *s) {
    unsigned res = s->zones, hz = s->hz, ms = s->integration_ms;
    if ((res != 16 && res != 64) || hz < 1 || hz > (res == 64 ? 15u : 60u) ||
        (s->order != 1 && s->order != 2) || (s->mode != 1 && s->mode != 3) ||
        ms < 2 || ms > 1000 || s->sharpener > 99 ||
        (s->mode == 3 && ((res == 64 ? 4 : 1) * ms + 1) * hz >= 1000)) {
        printf("TOF invalid settings: 4x4 1-60Hz; 8x8 1-15Hz; autonomous integration must fit period\n");
        return false;
    }
    if (!initialized) { printf("TOF not initialized; use Reinitialize\n"); return false; }
    if (!tof_stop()) return false;
    if (!check(vl53l8cx_set_resolution(&dev, s->zones), "resolution") ||
        !check(vl53l8cx_set_ranging_frequency_hz(&dev, s->hz), "frequency") ||
        !check(vl53l8cx_set_target_order(&dev, s->order), "target order") ||
        !check(vl53l8cx_set_ranging_mode(&dev, s->mode), "ranging mode") ||
        !check(vl53l8cx_set_integration_time_ms(&dev, s->integration_ms), "integration") ||
        !check(vl53l8cx_set_sharpener_percent(&dev, s->sharpener), "sharpener")) return false;
    // Confirm the active settings before reporting them.
    uint8_t r, f, o, m, sharp; uint32_t t;
    if (!check(vl53l8cx_get_resolution(&dev, &r), "read resolution") ||
        !check(vl53l8cx_get_ranging_frequency_hz(&dev, &f), "read frequency") ||
        !check(vl53l8cx_get_target_order(&dev, &o), "read target order") ||
        !check(vl53l8cx_get_ranging_mode(&dev, &m), "read mode") ||
        !check(vl53l8cx_get_integration_time_ms(&dev, &t), "read integration") ||
        !check(vl53l8cx_get_sharpener_percent(&dev, &sharp), "read sharpener")) return false;
    if (r != s->zones || f != s->hz || o != s->order || m != s->mode || t != s->integration_ms) {
        printf("TOF configuration readback mismatch; ranging remains stopped\n");
        return false;
    }
    active = *s;
    active.sharpener = sharp; // the ULD's percentage readback is quantized
    if (!check(vl53l8cx_start_ranging(&dev), "start")) return false;
    running = true;
    errors = 0;
    last_data = next_poll = get_absolute_time();
    printf("TOF running: %ux%u %uHz, %s order, %u targets/zone, %s mode, integration %ums, sharpener %u%%\n",
           res == 64 ? 8 : 4, res == 64 ? 8 : 4, hz, s->order == 1 ? "closest" : "strongest",
           VL53L8CX_NB_TARGET_PER_ZONE, s->mode == 1 ? "continuous" : "autonomous", ms, sharp);
    return true;
}

bool tof_init(void) {
    if (!tof_stop()) return false;
    initialized = false;
    printf("TOF initializing: SPI0 GP16/17/18/19, INT GP20 input; LPn not connected\n");
    if (!check(vl53l8cx_pico_init(&dev.platform), "platform")) return false;
    sleep_ms(100);
    uint8_t alive = 0;
    if (!check(vl53l8cx_is_alive(&dev, &alive), "ID read") || !alive) {
        printf("TOF not detected. Check carrier power, SPI wiring and SPI_I2C_N/LPn pullups.\n");
        return false;
    }
    printf("TOF detected; uploading sensor firmware (~84KB)...\n");
    if (!check(vl53l8cx_init(&dev), "firmware initialization")) return false;
    initialized = true;
    return true;
}

const VL53L8CX_ResultsData *tof_poll(void) {
    if (!running || !time_reached(next_poll)) return NULL;
    next_poll = make_timeout_time_us(POLL_US);
    uint8_t ready = 0;
    uint8_t status = vl53l8cx_check_data_ready(&dev, &ready);
    if (!status && ready) status = vl53l8cx_get_ranging_data(&dev, &results);
    if (status) {
        if (++errors >= 3) {
            check(status, "repeated ranging read failures");
            tof_stop();
            // Stop polling on a fault even if stop itself failed; retry via init.
            running = false; initialized = false;
        }
        return NULL;
    }
    errors = 0;
    if (!ready) {
        if (absolute_time_diff_us(last_data, get_absolute_time()) > NO_DATA_US) {
            printf("TOF no fresh data for 3s; check power/SPI, or use Reinitialize\n");
            last_data = get_absolute_time();
        }
        return NULL;
    }
    last_data = get_absolute_time();
    return &results;
}

const tof_settings_t *tof_settings(void) { return &active; }

bool tof_running(void) { return running; }
