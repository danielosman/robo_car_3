// Host test for common/link.c: framing, checksums and resynchronisation, without
// a Pico. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -Icommon/test/stubs -o build/link_test common/test/link_test.c && build/link_test
// The stubs fake just enough of the Pico SDK; the fake UART collects sent bytes in
// tx_wire[], and the test feeds bytes to the receiver through link.c's ring buffer.
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "../link.c"

static void wire_to_rx(const uint8_t *b, size_t n) { // what the other Pico receives
    for (size_t i = 0; i < n; i++) rx_buf[rx_head++ & (RX_SIZE - 1)] = b[i];
}
static size_t send(uint8_t type, const uint8_t *body, size_t len) {
    tx_wire_n = 0; assert(link_send(type, body, len)); commit_dr(); return tx_wire_n;
}
int main(void) {
    link_init();
    srand(1);
    uint8_t body[LINK_MAX_BODY] = {0}; link_message_t m; int checked = 0;
    // 1. Round trips: every length, random bodies full of zeros and 0xFF.
    for (int round = 0; round < 2000; round++) {
        size_t len = (size_t)(rand() % (LINK_MAX_BODY + 1));
        for (size_t i = 0; i < len; i++) { int r = rand() % 4; body[i] = r == 0 ? 0 : r == 1 ? 0xFF : (uint8_t)rand(); }
        uint8_t type = (uint8_t)rand();
        size_t n = send(type, body, len);
        for (size_t i = 0; i + 1 < n; i++) assert(tx_wire[i] != 0);   // only the delimiter is 0
        assert(tx_wire[n - 1] == 0);
        wire_to_rx(tx_wire, n);
        assert(link_receive(&m)); assert(m.type == type && m.len == len && !memcmp(m.body, body, len));
        assert(!link_receive(&m));
        checked++;
    }
    link_statistics_t s = link_statistics();
    assert(s.rx_bad == 0 && s.rx_lost == 0 && s.rx_messages == 2000);
    // 2. A corrupted byte: that frame is dropped, the next one arrives.
    memset(body, 0x55, 10);
    size_t n = send(7, body, 10); tx_wire[3] ^= 0x10; wire_to_rx(tx_wire, n);
    n = send(8, body, 10); wire_to_rx(tx_wire, n);
    assert(link_receive(&m) && m.type == 8); assert(!link_receive(&m));
    s = link_statistics(); assert(s.rx_bad == 1); assert(s.rx_lost == 1);
    // 3. A frame lost entirely: counted as lost, nothing bad.
    send(9, body, 10);                              // never delivered
    n = send(10, body, 10); wire_to_rx(tx_wire, n);
    assert(link_receive(&m) && m.type == 10);
    s = link_statistics(); assert(s.rx_lost == 2 && s.rx_bad == 1);
    // 4. Garbage and a truncated frame in the middle of the stream, then resync.
    uint8_t junk[300]; for (int i = 0; i < 300; i++) junk[i] = (uint8_t)(rand() | 1);
    wire_to_rx(junk, 300); uint8_t z = 0; wire_to_rx(&z, 1);   // too-long frame
    n = send(11, body, 10); wire_to_rx(tx_wire, n - 4); wire_to_rx(&z, 1); // truncated
    n = send(12, body, 10); wire_to_rx(tx_wire, n);
    assert(link_receive(&m) && m.type == 12); assert(!link_receive(&m));
    s = link_statistics(); assert(s.rx_bad == 3);
    // 5. The receive buffer overruns (the loop didn't call link_receive() in time): 30
    // frames of 60 B (~1.9 KB) arrive through the interrupt into the 1 KB buffer. The
    // ones that fit come through, the one cut is bad, the rest count as lost once a
    // frame arrives again.
    s = link_statistics();
    memset(body, 0x33, 50);
    fake_rx_n = fake_rx_at = 0;
    for (int i = 0; i < 30; i++) {
        n = send((uint8_t)(100 + i), body, 50);
        memcpy(fake_rx + fake_rx_n, tx_wire, n);
        fake_rx_n += n;
    }
    size_t frame_n = n;
    on_uart_irq();
    fake_rx_done();
    int got = 0;
    while (link_receive(&m)) { assert(m.type == 100 + got); got++; }
    assert(got == (int)(RX_SIZE / frame_n) && got < 30);
    // The cut frame's start runs into the next frame: that one is bad too; the one after arrives.
    n = send(200, body, 50); wire_to_rx(tx_wire, n);
    n = send(201, body, 50); wire_to_rx(tx_wire, n);
    assert(link_receive(&m) && m.type == 201 && !link_receive(&m));
    link_statistics_t s2 = link_statistics();
    assert(s2.rx_lost - s.rx_lost == (uint32_t)(30 - got + 1) && s2.rx_bad - s.rx_bad == 1);
    printf("overrun: %d of 30 frames fit the buffer, %lu lost, 1 bad\n", got, (unsigned long)(s2.rx_lost - s.rx_lost));
    s = s2;
    // 6. Oversized body is refused.
    assert(!link_send(1, body, LINK_MAX_BODY + 1));
    printf("OK: %d round trips; bad %lu lost %lu dropped %lu\n", checked,
           (unsigned long)s.rx_bad, (unsigned long)s.rx_lost, (unsigned long)link_statistics().tx_dropped);
    return 0;
}
