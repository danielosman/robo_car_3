#pragma once
#include <stdint.h>
static uint32_t fake_rand = 0x7f3a5c01;
static inline uint32_t get_rand_32(void) { return fake_rand; }
