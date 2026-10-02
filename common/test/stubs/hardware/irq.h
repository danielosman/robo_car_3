#pragma once
#define UART0_IRQ 33
static inline void irq_set_exclusive_handler(int n, void (*h)(void)) { (void)n; (void)h; }
static inline void irq_set_enabled(int n, bool e) { (void)n; (void)e; }
