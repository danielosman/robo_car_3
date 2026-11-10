#pragma once
// Message link between PicoA and PicoB: UART0, GP0 TX / GP1 RX on both boards
// (crossed on the PCB), 1 Mbaud. A message is a type byte plus a body of up to
// LINK_MAX_BODY bytes; framing, checksums and buffering are handled here, and a
// damaged message is dropped rather than delivered. Never blocks: received bytes
// are buffered by an interrupt, and link_send() drops a message (and counts it)
// when the transmit buffer is full. Message types and bodies are in link_messages.h.
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINK_MAX_BODY 64
#define LINK_BAUD     1000000 // 10 bits per byte on the wire (8N1): 10 us per byte

typedef struct {
    uint8_t type;
    uint8_t len;
    uint8_t body[LINK_MAX_BODY];
} link_message_t;

typedef struct {
    uint32_t rx_bytes;   // everything received, including damaged frames
    uint32_t rx_messages;    // messages delivered by link_receive()
    uint32_t rx_bad;     // frames dropped: bad checksum, bad framing or too long
    uint32_t rx_lost;    // messages missing from the other side's sequence (includes its restarts)
    uint32_t tx_messages;
    uint32_t tx_dropped; // transmit buffer full, or body too long
} link_statistics_t;

void link_init(void);
bool link_send(uint8_t type, const void *body, size_t len);
// Next intact message, if one has arrived. Call often: the receive buffer (1 KB) holds
// ~10 ms of a full-speed stream, ~230 ms of PicoB's traffic (~4.5 KB/s); bytes beyond
// it are lost (the damaged frame counts in rx_bad, the missing ones in rx_lost).
bool link_receive(link_message_t *message);
link_statistics_t link_statistics(void);
