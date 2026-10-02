// PicoA PCB: ADDR GP2 (low -> 0x44), INT GP3 input, I2C0 SDA GP4/SCL GP5.
// TI OPT4048 SBOSA84: 100ms/channel, continuous, auto-range. No general-call
// reset: I2C0 is shared with the future MLX90640. Poll instead of using INT.
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "opt4048.h"
#define ADDRESS 0x44
#define CONFIG 0x3238
static bool ready, have_counter;
static uint8_t last_counter;
static unsigned failures;
static uint32_t sequence;
static absolute_time_t next_poll, last_frame;
static bool read_bytes(uint8_t reg, uint8_t *data, size_t size) {
    return i2c_write_timeout_us(i2c0, ADDRESS, &reg, 1, true, 10000) == 1 &&
        i2c_read_timeout_us(i2c0, ADDRESS, data, size, false, 10000) == (int)size;
}
static bool read16(uint8_t reg, uint16_t *value) {
    uint8_t b[2];
    if (!read_bytes(reg, b, 2)) return false;
    *value = ((uint16_t)b[0] << 8) | b[1]; return true;
}
static bool write16(uint8_t reg, uint16_t value) {
    uint8_t b[] = {reg, value >> 8, value & 255};
    return i2c_write_timeout_us(i2c0, ADDRESS, b, 3, false, 10000) == 3;
}
void opt4048_init(void) {
    ready = false; have_counter = false; failures = 0;
    // Set ADDR before any I2C activity (sampled on every transaction).
    gpio_init(2); gpio_put(2, 0); gpio_set_dir(2, GPIO_OUT);
    gpio_init(3); gpio_set_dir(3, GPIO_IN);
    // Ordinary Fast-mode. >400kHz needs OPT4048's HS entry protocol, not just
    // a faster clock; do not use the PCB plan's proposed 1MHz blindly.
    i2c_init(i2c0, 400000);
    gpio_set_function(4, GPIO_FUNC_I2C); gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4); gpio_pull_up(5);
    sleep_ms(10);
    uint16_t id = 0, config = 0, control = 0;
    if (!read16(0x11, &id) || id != 0x0821) {
        printf("OPT4048 not detected: ID=0x%04x (expected 0821). Check power, GP2 ADDR and I2C0 GP4/5.\n", id);
        return;
    }
    if (!write16(0x0a, 0x3208) || !write16(0x0b, 0x8011) || !write16(0x0a, CONFIG) ||
        !read16(0x0a, &config) || !read16(0x0b, &control) || config != CONFIG || control != 0x8011) {
        printf("OPT4048 configuration failed (config=%04x control=%04x)\n", config, control);
        return;
    }
    ready = true;
    last_frame = get_absolute_time(); next_poll = make_timeout_time_ms(450);
    printf("OPT4048 ready: I2C0 0x44, 400kHz, auto-range, 100ms/channel (~2.5 complete cycles/s)\n");
}
void opt4048_poll(void) {
    if (!ready || !time_reached(next_poll)) return;
    next_poll = make_timeout_time_ms(25);
    uint8_t raw[16], again[16]; uint16_t flags;
    if (!read_bytes(0, raw, sizeof raw) || !read_bytes(0, again, sizeof again) || !read16(0x0c, &flags)) {
        if (++failures >= 3) {
            ready = false;
            printf("OPT4048 repeated I2C failures; check wiring/power and use Reinitialize\n");
        }
        return;
    }
    failures = 0;
    if (absolute_time_diff_us(last_frame, get_absolute_time()) > 3000000) {
        printf("OPT4048 no fresh coherent cycle for 3s; use Reinitialize\n");
        last_frame = get_absolute_time();
    }
    // Channels convert sequentially, not simultaneously. Only publish stable
    // register snapshots with equal channel counters (one complete cycle).
    if (memcmp(raw, again, sizeof raw)) return;
    uint8_t counter = raw[3] >> 4;
    for (unsigned c = 1; c < 4; ++c) if ((raw[4*c + 3] >> 4) != counter) return;
    if (have_counter && counter == last_counter) return;
    have_counter = true; last_counter = counter; last_frame = get_absolute_time();
    // OPT3 | payload length 28 LE | version, overload, ID LE, sequence LE,
    // conversion ms LE, reserved[2], raw 16-byte big-endian register snapshot.
    uint8_t packet[36] = {'O','P','T','3',28,0,0,0,1,(flags & 8) ? 1 : 0,
        0x21,0x08,0,0,0,0,100,0,0,0};
    for (unsigned i = 0; i < 4; ++i) packet[12+i] = sequence >> (8*i);
    ++sequence;
    memcpy(packet + 20, raw, sizeof raw);
    fwrite(packet, 1, sizeof packet, stdout); fflush(stdout);
}
