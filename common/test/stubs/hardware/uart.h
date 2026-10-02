#pragma once
// Fake UART: bytes written to dr are collected in tx_wire[] (committed lazily).
#define SENTINEL 0xFFFFFFFFu
typedef struct { uint32_t dr, imsc; } uart_hw_t;
typedef int uart_inst_t;
static uart_hw_t fake_hw = {SENTINEL, 0};
static uint8_t tx_wire[100000]; static size_t tx_wire_n;
#define uart0 ((uart_inst_t *)0)
#define UART_PARITY_NONE 0
#define UART_UARTIMSC_TXIM_BITS 0x20
static inline uart_hw_t *uart_get_hw(uart_inst_t *u) { (void)u; return &fake_hw; }
static inline void commit_dr(void) { if (fake_hw.dr != SENTINEL) { tx_wire[tx_wire_n++] = (uint8_t)fake_hw.dr; fake_hw.dr = SENTINEL; } }
static inline bool uart_is_writable(uart_inst_t *u) { (void)u; commit_dr(); return true; }
static inline bool uart_is_readable(uart_inst_t *u) { (void)u; return false; }
static inline unsigned uart_init(uart_inst_t *u, unsigned b) { (void)u; return b; }
static inline void uart_set_format(uart_inst_t *u, int a, int b, int c) { (void)u;(void)a;(void)b;(void)c; }
static inline void uart_set_hw_flow(uart_inst_t *u, bool a, bool b) { (void)u;(void)a;(void)b; }
static inline void uart_set_fifo_enabled(uart_inst_t *u, bool e) { (void)u;(void)e; }
static inline void uart_set_irqs_enabled(uart_inst_t *u, bool a, bool b) { (void)u;(void)a;(void)b; }
static inline void hw_set_bits(volatile uint32_t *r, uint32_t m) { *r |= m; }
static inline void hw_clear_bits(volatile uint32_t *r, uint32_t m) { *r &= ~m; }
