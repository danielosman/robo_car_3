#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef unsigned int uint;
#define GPIO_FUNC_UART 2
static inline void gpio_set_function(uint p, int f) { (void)p; (void)f; }
static inline void gpio_pull_up(uint p) { (void)p; }
