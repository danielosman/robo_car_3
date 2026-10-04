#pragma once
// Fake link: records what is sent and delivers what the test queues as the other Pico's
// messages. Using it before link_init() fails the test.
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#define LINK_MAX_BODY 64
#define LINK_BAUD     1000000
typedef struct { uint8_t type; uint8_t len; uint8_t body[LINK_MAX_BODY]; } link_msg_t;

static bool fake_link_started;
static link_msg_t fake_sent[256], fake_inbox[64];
static int fake_sent_n, fake_inbox_head, fake_inbox_tail;

static inline void link_init(void) { fake_link_started = true; }
static inline bool link_send(uint8_t type, const void *body, size_t len) {
    assert(fake_link_started && len <= LINK_MAX_BODY && fake_sent_n < 256);
    link_msg_t *m = &fake_sent[fake_sent_n++];
    m->type = type; m->len = (uint8_t)len; memcpy(m->body, body, len);
    return true;
}
static inline bool link_receive(link_msg_t *msg) {
    assert(fake_link_started);
    if (fake_inbox_head == fake_inbox_tail) return false;
    *msg = fake_inbox[fake_inbox_head++ % 64];
    return true;
}
// The test plays the other Pico.
static inline void fake_link_deliver(uint8_t type, const void *body, size_t len) {
    assert(fake_inbox_tail - fake_inbox_head < 64);
    link_msg_t *m = &fake_inbox[fake_inbox_tail++ % 64];
    m->type = type; m->len = (uint8_t)len; memcpy(m->body, body, len);
}
