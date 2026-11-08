#pragma once
// Fake USB serial for commands.c: the test says whether a monitor is connected and
// types characters.
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define PICO_ERROR_TIMEOUT (-1)
static bool fake_usb_connected;
static char fake_usb_typed[256];
static uint32_t fake_usb_at, fake_usb_len;
static inline bool stdio_usb_connected(void) { return fake_usb_connected; }
static inline void fake_usb_type(const char *text) {
    uint32_t n = (uint32_t)strlen(text);
    memcpy(fake_usb_typed + fake_usb_len, text, n);
    fake_usb_len += n;
}
static inline int getchar_timeout_us(uint32_t us) {
    (void)us;
    if (fake_usb_at == fake_usb_len) { fake_usb_at = fake_usb_len = 0; return PICO_ERROR_TIMEOUT; }
    return (unsigned char)fake_usb_typed[fake_usb_at++];
}
