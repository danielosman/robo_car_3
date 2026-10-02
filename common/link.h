#pragma once
// Message link between PicoA and PicoB: UART0, GP0 TX / GP1 RX on both boards
// (crossed on the PCB), 1 Mbaud. A message is a type byte plus a body of up to
// LINK_MAX_BODY bytes; framing, checksums and buffering are handled here, and a
// damaged message is dropped rather than delivered. Never blocks: received bytes
// are buffered by an interrupt, and link_send() drops a message (and counts it)
// when the transmit buffer is full. Message types and bodies are in link_msgs.h.
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINK_MAX_BODY 64

typedef struct {
    uint8_t type;
    uint8_t len;
    uint8_t body[LINK_MAX_BODY];
} link_msg_t;

typedef struct {
    uint32_t rx_bytes;   // everything received, including damaged frames
    uint32_t rx_msgs;    // messages delivered by link_receive()
    uint32_t rx_bad;     // frames dropped: bad checksum, bad framing or too long
    uint32_t rx_lost;    // messages missing from the other side's sequence (includes its restarts)
    uint32_t tx_msgs;
    uint32_t tx_dropped; // transmit buffer full, or body too long
} link_stats_t;

void link_init(void);
bool link_send(uint8_t type, const void *body, size_t len);
// Next intact message, if one has arrived. Call often: the receive buffer holds ~10 ms.
bool link_receive(link_msg_t *msg);
link_stats_t link_stats(void);
