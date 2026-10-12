#pragma once
// OPT4048 colour sensor on PicoA: I2C0 0x44, continuous, auto-range, 100 ms per
// channel (one complete reading ~2.5 times a second). Only coherent readings (all
// four channels from the same conversion cycle) are handed out; converting them to
// lux and colour is the PC's (pc/bringup/src/opt4048.ts).
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t raw[16];   // registers 0x00-0x07 as read: per channel exponent, mantissa, counter, CRC (big-endian)
    bool overload;
    uint32_t sequence; // counts the readings handed out since init
    uint32_t t_us;     // time_us_32() when it was read
} opt4048_reading_t;

bool opt4048_init(void);  // false if the sensor doesn't answer (printed)
bool opt4048_poll(void);  // call often; true when a new reading arrived
// The newest reading; false before the first one.
bool opt4048_latest(opt4048_reading_t *reading);
