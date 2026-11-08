#pragma once
// CRC-32 as zlib computes it (the PC checks snapshots with zlib.crc32). A nibble
// table: small and fast enough for a snapshot (~31 KB) now and then.
#include <stdint.h>

static inline uint32_t crc32_of(const uint8_t *p, uint32_t n) {
    static const uint32_t nibble[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
    };
    uint32_t c = 0xffffffffu;
    for (uint32_t i = 0; i < n; i++) {
        c ^= p[i];
        c = (c >> 4) ^ nibble[c & 15];
        c = (c >> 4) ^ nibble[c & 15];
    }
    return ~c;
}
