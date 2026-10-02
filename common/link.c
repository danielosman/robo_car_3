// Frame on the wire: COBS(type, seq, body, crc16 lo, crc16 hi) followed by 0x00.
// COBS removes every 0x00 from the encoded bytes, so 0x00 only ever ends a frame
// and the receiver resynchronises at the next one after any error.
// CRC is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).
#include <string.h>
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "link.h"

#define UART    uart0
#define PIN_TX  0
#define PIN_RX  1
#define BAUD    1000000

#define RAW_MAX   (2 + LINK_MAX_BODY + 2)
#define FRAME_MAX (RAW_MAX + RAW_MAX / 254 + 2) // COBS overhead and the 0x00

// Ring buffers; indices run freely and are masked on access (sizes are powers of two).
#define RX_SIZE 1024u
#define TX_SIZE 512u
static uint8_t rx_buf[RX_SIZE], tx_buf[TX_SIZE];
static volatile uint32_t rx_head, tx_tail; // advanced by the interrupt
static uint32_t rx_tail, tx_head;

static uint8_t frame[FRAME_MAX];
static size_t frame_len;
static bool frame_too_long;
static uint8_t tx_seq, rx_next_seq;
static bool rx_seq_known;
static link_stats_t stats;

static uint16_t crc16(const uint8_t *p, size_t n) {
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)*p++ << 8;
        for (int i = 0; i < 8; i++) crc = crc & 0x8000 ? (uint16_t)(crc << 1) ^ 0x1021 : (uint16_t)(crc << 1);
    }
    return crc;
}

static size_t cobs_encode(const uint8_t *in, size_t n, uint8_t *out) {
    size_t code_at = 0, o = 1;
    uint8_t code = 1;
    for (size_t i = 0; i < n; i++) {
        if (in[i]) { out[o++] = in[i]; code++; }
        if (!in[i] || code == 0xFF) { out[code_at] = code; code_at = o++; code = 1; }
    }
    out[code_at] = code;
    return o;
}

static bool cobs_decode(const uint8_t *in, size_t n, uint8_t *out, size_t *out_n) {
    size_t i = 0, o = 0;
    while (i < n) {
        uint8_t code = in[i++];
        for (uint8_t k = 1; k < code; k++) {
            if (i >= n) return false;
            out[o++] = in[i++];
        }
        if (code != 0xFF && i < n) out[o++] = 0;
    }
    *out_n = o;
    return true;
}

// Moves queued bytes into the UART FIFO; the TX interrupt stays enabled while
// bytes are waiting. Runs in the interrupt or with interrupts disabled.
static void tx_pump(void) {
    while (tx_tail != tx_head && uart_is_writable(UART))
        uart_get_hw(UART)->dr = tx_buf[tx_tail++ & (TX_SIZE - 1)];
    if (tx_tail != tx_head) hw_set_bits(&uart_get_hw(UART)->imsc, UART_UARTIMSC_TXIM_BITS);
    else hw_clear_bits(&uart_get_hw(UART)->imsc, UART_UARTIMSC_TXIM_BITS);
}

static void on_uart_irq(void) {
    while (uart_is_readable(UART)) {
        uint8_t b = (uint8_t)uart_get_hw(UART)->dr; // a damaged byte fails the frame's CRC
        if (rx_head - rx_tail < RX_SIZE) rx_buf[rx_head++ & (RX_SIZE - 1)] = b;
        // else: overrun; the lost bytes show up as a bad frame and a sequence gap
    }
    tx_pump();
}

void link_init(void) {
    uart_init(UART, BAUD);
    uart_set_format(UART, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(UART, false, false);
    uart_set_fifo_enabled(UART, true);
    gpio_set_function(PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RX, GPIO_FUNC_UART);
    gpio_pull_up(PIN_RX); // idle-high while the other Pico is still starting
    irq_set_exclusive_handler(UART0_IRQ, on_uart_irq);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irqs_enabled(UART, true, false);
}

bool link_send(uint8_t type, const void *body, size_t len) {
    if (len > LINK_MAX_BODY) { stats.tx_dropped++; return false; }
    uint8_t raw[RAW_MAX], out[FRAME_MAX];
    raw[0] = type;
    raw[1] = tx_seq;
    memcpy(raw + 2, body, len);
    uint16_t crc = crc16(raw, len + 2);
    raw[len + 2] = crc & 0xFF;
    raw[len + 3] = crc >> 8;
    size_t n = cobs_encode(raw, len + 4, out);
    out[n++] = 0;

    uint32_t irq_state = save_and_disable_interrupts();
    bool fits = TX_SIZE - (tx_head - tx_tail) >= n;
    if (fits) {
        for (size_t i = 0; i < n; i++) tx_buf[tx_head++ & (TX_SIZE - 1)] = out[i];
        tx_pump();
    }
    restore_interrupts(irq_state);
    if (!fits) { stats.tx_dropped++; return false; }
    tx_seq++;
    stats.tx_msgs++;
    return true;
}

static bool decode_frame(link_msg_t *msg) {
    uint8_t raw[FRAME_MAX];
    size_t n;
    if (frame_too_long || !cobs_decode(frame, frame_len, raw, &n)) return false;
    if (n < 4 || n - 4 > LINK_MAX_BODY) return false;
    if (crc16(raw, n - 2) != (uint16_t)(raw[n - 2] | raw[n - 1] << 8)) return false;
    uint8_t seq = raw[1];
    if (rx_seq_known && seq != rx_next_seq) stats.rx_lost += (uint8_t)(seq - rx_next_seq);
    rx_next_seq = seq + 1;
    rx_seq_known = true;
    msg->type = raw[0];
    msg->len = (uint8_t)(n - 4);
    memcpy(msg->body, raw + 2, msg->len);
    return true;
}

bool link_receive(link_msg_t *msg) {
    while (rx_tail != rx_head) {
        uint8_t b = rx_buf[rx_tail++ & (RX_SIZE - 1)];
        stats.rx_bytes++;
        if (b) {
            if (frame_len < FRAME_MAX) frame[frame_len++] = b;
            else frame_too_long = true;
            continue;
        }
        bool empty = frame_len == 0 && !frame_too_long;
        bool ok = !empty && decode_frame(msg);
        frame_len = 0;
        frame_too_long = false;
        if (ok) { stats.rx_msgs++; return true; }
        if (!empty) stats.rx_bad++;
    }
    return false;
}

link_stats_t link_stats(void) { return stats; }
