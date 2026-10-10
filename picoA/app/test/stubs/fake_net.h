#pragma once
// Fake WiFi chip, lwIP and stdio driver registry for wifi_console.c: the test
// sets the link status, plays the server (announcements, accepting, keys,
// closing) and reads what the robot sent. Callbacks are called the way lwIP
// would, without the lock taken by the caller; lwIP calls from the main loop must
// hold it (checked).
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef uint16_t u16_t;
typedef int8_t err_t;
#define ERR_OK   0
#define ERR_MEM  (-1)
#define ERR_ABRT (-13)
#define ERR_RST  (-14)
#define PICO_ERROR_NO_DATA (-3)

// --- addresses ---
typedef struct { uint32_t addr; } ip_addr_t;
typedef ip_addr_t ip4_addr_t;
#define IPADDR_TYPE_V4 0
static const ip_addr_t fake_ip_any = {0};
#define IP_ANY_TYPE (&fake_ip_any)
static inline ip_addr_t fake_ip(int a, int b, int c, int d) {
    return (ip_addr_t){(uint32_t)(a | b << 8 | c << 16 | d << 24)};
}
static inline char *ipaddr_ntoa(const ip_addr_t *ip) {
    static char text[16];
    snprintf(text, sizeof text, "%u.%u.%u.%u", (unsigned)(ip->addr & 255), (unsigned)(ip->addr >> 8 & 255),
             (unsigned)(ip->addr >> 16 & 255), (unsigned)(ip->addr >> 24));
    return text;
}
#define ip4addr_ntoa ipaddr_ntoa
static inline int ipaddr_aton(const char *text, ip_addr_t *ip) {
    unsigned a, b, c, d;
    if (sscanf(text, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return 0;
    *ip = fake_ip((int)a, (int)b, (int)c, (int)d);
    return 1;
}

// --- the lock ---
static int fake_lock_depth;
static inline void cyw43_arch_lwip_begin(void) { fake_lock_depth++; }
static inline void cyw43_arch_lwip_end(void) { assert(fake_lock_depth > 0); fake_lock_depth--; }
#define FAKE_LOCKED() assert(fake_lock_depth > 0 || fake_in_callback)
static bool fake_in_callback;

// --- the WiFi chip ---
struct netif { ip_addr_t ip; };
typedef struct { struct netif netif[2]; } cyw43_t;
static cyw43_t cyw43_state;
#define CYW43_ITF_STA 0
#define CYW43_COUNTRY_WORLDWIDE 0
#define CYW43_AUTH_WPA2_AES_PSK 0x00400004
#define CYW43_NONE_PM 0xa11140
#define CYW43_LINK_DOWN    0
#define CYW43_LINK_JOIN    1
#define CYW43_LINK_UP      3
#define CYW43_LINK_FAIL    (-1)
#define CYW43_LINK_NONET   (-2)
#define CYW43_LINK_BADAUTH (-3)
static int fake_link = CYW43_LINK_DOWN;
static int32_t fake_rssi = -55;
static int fake_inits, fake_joins, fake_leaves;
static uint32_t fake_pm;
static char fake_ssid[33];
static inline int cyw43_arch_init_with_country(uint32_t country) { (void)country; fake_inits++; return 0; }
static inline void cyw43_arch_enable_sta_mode(void) {}
static inline int cyw43_wifi_pm(cyw43_t *self, uint32_t pm) { (void)self; fake_pm = pm; return 0; }
static inline int cyw43_arch_wifi_connect_async(const char *ssid, const char *pw, uint32_t auth) {
    (void)pw; (void)auth;
    assert(fake_inits == 1);
    snprintf(fake_ssid, sizeof fake_ssid, "%s", ssid);
    fake_joins++;
    fake_link = CYW43_LINK_JOIN;
    return 0;
}
static inline int cyw43_tcpip_link_status(cyw43_t *self, int itf) { (void)self; (void)itf; return fake_link; }
static inline int cyw43_wifi_get_rssi(cyw43_t *self, int32_t *rssi) { (void)self; *rssi = fake_rssi; return 0; }
static inline int cyw43_wifi_leave(cyw43_t *self, int itf) { (void)self; (void)itf; fake_leaves++; fake_link = CYW43_LINK_DOWN; return 0; }
static inline const ip_addr_t *netif_ip4_addr(const struct netif *n) { return &n->ip; }

// --- pbufs ---
struct pbuf { struct pbuf *next; void *payload; u16_t len, tot_len; };
static int fake_pbufs_freed;
static inline u16_t pbuf_copy_partial(const struct pbuf *p, void *buf, u16_t len, u16_t offset) {
    assert(offset == 0);
    u16_t n = p->len < len ? p->len : len;
    memcpy(buf, p->payload, n);
    return n;
}
static int fake_pbufs_alloced;
static bool fake_pbuf_fail;      // pbuf_alloc() runs out of memory
#define PBUF_TRANSPORT 0
#define PBUF_RAM 0
static inline uint8_t pbuf_free(struct pbuf *p) { (void)p; fake_pbufs_freed++; return 1; }
static inline struct pbuf *pbuf_alloc(int layer, u16_t len, int type) { // one at a time
    (void)layer; (void)type;
    static char payload[2048];
    static struct pbuf p;
    if (fake_pbuf_fail || len > sizeof payload) return NULL;
    p = (struct pbuf){.payload = payload, .len = len, .tot_len = len};
    fake_pbufs_alloced++;
    return &p;
}

// --- UDP: the announcements ---
struct udp_pcb;
typedef void (*udp_recv_fn)(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
struct udp_pcb { u16_t port; udp_recv_fn recv; void *arg; };
static struct udp_pcb fake_udp;
static int fake_udps;
static inline struct udp_pcb *udp_new_ip_type(uint8_t type) { (void)type; FAKE_LOCKED(); fake_udps++; return &fake_udp; }
static inline err_t udp_bind(struct udp_pcb *pcb, const ip_addr_t *ip, u16_t port) { (void)ip; FAKE_LOCKED(); pcb->port = port; return ERR_OK; }
static inline void udp_recv(struct udp_pcb *pcb, udp_recv_fn fn, void *arg) { FAKE_LOCKED(); pcb->recv = fn; pcb->arg = arg; }
// The datagrams sent: what, to where.
static char fake_dgram[2048];
static uint32_t fake_dgram_len, fake_dgrams;
static ip_addr_t fake_dgram_ip;
static u16_t fake_dgram_port;
static err_t fake_udp_err = ERR_OK;  // what udp_sendto() returns (ERR_MEM: the WiFi chip is full)
static inline err_t udp_sendto(struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *ip, u16_t port) {
    FAKE_LOCKED();
    assert(pcb == &fake_udp && p->len <= sizeof fake_dgram);
    if (fake_udp_err != ERR_OK) return fake_udp_err;
    memcpy(fake_dgram, p->payload, p->len);
    fake_dgram_len = p->len;
    fake_dgram_ip = *ip;
    fake_dgram_port = port;
    fake_dgrams++;
    return ERR_OK;
}

// The server broadcasts `text` from `from`.
static inline void fake_announce(ip_addr_t from, const char *text) {
    assert(fake_udp.recv && fake_udp.port == 4210);
    struct pbuf p = {.payload = (void *)text, .len = (u16_t)strlen(text), .tot_len = (u16_t)strlen(text)};
    fake_in_callback = true;
    fake_udp.recv(fake_udp.arg, &fake_udp, &p, &from, 4210);
    fake_in_callback = false;
}

// --- TCP: the console's connection to the server ---
struct tcp_pcb;
typedef err_t (*tcp_connected_fn)(void *arg, struct tcp_pcb *pcb, err_t err);
typedef err_t (*tcp_recv_fn)(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
typedef err_t (*tcp_sent_fn)(void *arg, struct tcp_pcb *pcb, u16_t len);
typedef void (*tcp_err_fn)(void *arg, err_t err);
struct tcp_pcb {
    bool open;
    ip_addr_t ip; u16_t port;
    void *arg;
    // lwIP's fields wifi_console_link_status() reads; the test sets them
    uint32_t snd_lbb, lastack, cwnd, snd_wnd;
    uint16_t snd_queuelen;
    uint8_t nrtx;
    int16_t rto;
    tcp_connected_fn connected; tcp_recv_fn recv; tcp_sent_fn sent; tcp_err_fn err;
};
#define TCP_WRITE_FLAG_COPY 1
#define FAKE_SNDBUF 11680
static struct tcp_pcb fake_tcp;
static int fake_tcp_aborts, fake_tcp_connects, fake_outputs, fake_recved;
static uint32_t fake_sndbuf = FAKE_SNDBUF;  // what tcp_sndbuf() reports, used up by writes
static char fake_sent[65536];               // everything written, as the server receives it
static uint32_t fake_sent_len;
static inline struct tcp_pcb *tcp_new_ip_type(uint8_t type) {
    (void)type; FAKE_LOCKED();
    assert(!fake_tcp.open); // one connection at a time
    memset(&fake_tcp, 0, sizeof fake_tcp);
    fake_tcp.open = true;
    return &fake_tcp;
}
static inline void tcp_nagle_disable(struct tcp_pcb *pcb) { (void)pcb; }
static inline void tcp_arg(struct tcp_pcb *pcb, void *arg) { FAKE_LOCKED(); pcb->arg = arg; }
static inline void tcp_recv(struct tcp_pcb *pcb, tcp_recv_fn fn) { FAKE_LOCKED(); pcb->recv = fn; }
static inline void tcp_sent(struct tcp_pcb *pcb, tcp_sent_fn fn) { FAKE_LOCKED(); pcb->sent = fn; }
static inline void tcp_err(struct tcp_pcb *pcb, tcp_err_fn fn) { FAKE_LOCKED(); pcb->err = fn; }
static inline err_t tcp_connect(struct tcp_pcb *pcb, const ip_addr_t *ip, u16_t port, tcp_connected_fn fn) {
    FAKE_LOCKED();
    pcb->ip = *ip; pcb->port = port; pcb->connected = fn;
    fake_tcp_connects++;
    return ERR_OK;
}
static inline u16_t tcp_sndbuf(const struct tcp_pcb *pcb) { (void)pcb; return (u16_t)fake_sndbuf; }
static inline err_t tcp_write(struct tcp_pcb *pcb, const void *data, u16_t len, uint8_t flags) {
    FAKE_LOCKED();
    assert(pcb->open && flags == TCP_WRITE_FLAG_COPY && len <= fake_sndbuf);
    assert(fake_sent_len + len <= sizeof fake_sent);
    memcpy(fake_sent + fake_sent_len, data, len);
    fake_sent_len += len;
    fake_sndbuf -= len;
    return ERR_OK;
}
static inline err_t tcp_output(struct tcp_pcb *pcb) { (void)pcb; FAKE_LOCKED(); fake_outputs++; return ERR_OK; }
static inline void tcp_recved(struct tcp_pcb *pcb, u16_t len) { (void)pcb; fake_recved += len; }
static inline void tcp_abort(struct tcp_pcb *pcb) {
    FAKE_LOCKED();
    assert(pcb->open);
    if (pcb->err) pcb->err(pcb->arg, ERR_ABRT); // as lwIP does
    pcb->open = false;
    fake_tcp_aborts++;
}

static inline void fake_accepts(struct tcp_pcb *pcb) {
    assert(pcb->open && pcb->connected);
    fake_in_callback = true;
    pcb->connected(pcb->arg, pcb, ERR_OK);
    fake_in_callback = false;
}
static inline void fake_server_accepts(void) { fake_accepts(&fake_tcp); }
// The server sends `n` bytes (keys, heartbeats).
static inline void fake_server_sends(const char *bytes, u16_t n) {
    assert(fake_tcp.open);
    struct pbuf p = {.payload = (void *)bytes, .len = n, .tot_len = n};
    fake_in_callback = true;
    fake_tcp.recv(fake_tcp.arg, &fake_tcp, &p, ERR_OK);
    fake_in_callback = false;
}
static inline void fake_closes(struct tcp_pcb *pcb) {
    fake_in_callback = true;
    pcb->recv(pcb->arg, pcb, NULL, ERR_OK);
    fake_in_callback = false;
}
static inline void fake_server_closes(void) { fake_closes(&fake_tcp); }
// The connection fails (refused, reset): lwIP frees it and tells the robot.
static inline void fake_fails(struct tcp_pcb *pcb) {
    assert(pcb->open);
    pcb->open = false;
    fake_in_callback = true;
    pcb->err(pcb->arg, ERR_RST);
    fake_in_callback = false;
}
static inline void fake_tcp_fails(void) { fake_fails(&fake_tcp); }
// The server acknowledged everything: the send buffer is free again.
static inline void fake_acks(struct tcp_pcb *pcb) {
    uint32_t acked = FAKE_SNDBUF - fake_sndbuf;
    fake_sndbuf = FAKE_SNDBUF;
    fake_in_callback = true;
    if (acked) pcb->sent(pcb->arg, pcb, (u16_t)acked);
    fake_in_callback = false;
}
static inline void fake_server_acks(void) { fake_acks(&fake_tcp); }

// --- stdio ---
typedef struct stdio_driver stdio_driver_t;
struct stdio_driver {
    void (*out_chars)(const char *buf, int len);
    void (*out_flush)(void);
    int (*in_chars)(char *buf, int len);
    void (*set_chars_available_callback)(void (*fn)(void *), void *param);
    stdio_driver_t *next;
};
static stdio_driver_t *fake_stdio;
static inline void stdio_set_driver_enabled(stdio_driver_t *d, bool enabled) { assert(enabled); fake_stdio = d; }
