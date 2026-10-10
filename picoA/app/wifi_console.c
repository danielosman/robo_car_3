#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio/driver.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "wifi_console.h"

// The network: wifi_config.h is git-ignored (copy wifi_config.example.h).
#ifndef WIFI_SSID
#if __has_include("wifi_config.h")
#include "wifi_config.h"
#endif
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#endif
#ifndef WIFI_SERVER_IP
#define WIFI_SERVER_IP "" // tried when no announcement arrives
#endif
#ifndef WIFI_COUNTRY
#define WIFI_COUNTRY CYW43_COUNTRY_WORLDWIDE
#endif

#define ANNOUNCE_PORT      4210     // the server's "ROBOCAR-SERVER <port> [<data port>]" broadcasts
#define SERVER_PORT        4211     // for WIFI_SERVER_IP
#define DATA_PORT          4212     // recordings (UDP), when the announcement names no port
#define ANNOUNCE_TEXT      "ROBOCAR-SERVER "
#define JOIN_TIMEOUT_US    15000000
#define JOIN_RETRY_US      30000000
#define FIND_SERVER_US     5000000  // then wifi_console_settled(), and WIFI_SERVER_IP is tried
#define CONNECT_TIMEOUT_US 5000000
#define HEARTBEAT_US       2000000  // "\001rssi N\n" to the server; it sends '\0'
#define SILENCE_US         6000000  // nothing from the server for this long: it's gone
#define OUT_SIZE           16384
#define LWIP_SLOW_MS       500      // lwIP's rto ticks (TCP_SLOW_INTERVAL)
#define IN_SIZE            64

typedef enum {
    WIFI_OFF, WIFI_UNAVAILABLE, WIFI_JOINING, WIFI_JOIN_FAILED,
    WIFI_LOOKING, WIFI_CONNECTING, WIFI_CONNECTED,
} wifi_state_t;

// Only the main loop touches these.
static wifi_state_t state;
static const char *unavailable_why;
static absolute_time_t deadline; // what it means depends on the state
static absolute_time_t next_heartbeat;
static bool settled;
static ip_addr_t server_ip;
static uint16_t server_port, data_port;
static uint32_t console_connections;      // counts the console connections that came up

// A TCP connection to the server and what waits to go out on it.
typedef struct {
    struct tcp_pcb *pcb;                  // NULL if none
    bool up, lost;
    char *out;                            // not yet handed to TCP
    uint32_t size, start, len;
    uint32_t write_errors, output_errors; // tcp_write() / tcp_output() refused, since power-up
} conn_t;

// Shared with lwIP's callbacks, which run in the background: only with the lwIP
// lock held (cyw43_arch_lwip_begin/end; callbacks already hold it).
static struct udp_pcb *udp;
static char console_out[OUT_SIZE];
static conn_t console = {.out = console_out, .size = OUT_SIZE}; // the console's text and keys
static bool announced;
static ip_addr_t announced_ip;
static uint16_t announced_port, announced_data_port;
static absolute_time_t last_heard;
static char in[IN_SIZE];                  // keys from the server
static uint32_t in_start, in_len;

// --- lwIP side (lock held) ---

// Adds to what waits to go out; when it is full, the oldest goes.
static void keep_out(conn_t *c, const char *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        if (c->len == c->size) { c->start = (c->start + 1) % c->size; c->len--; }
        c->out[(c->start + c->len++) % c->size] = buf[i];
    }
}

static void send_out(conn_t *c) {
    if (!c->pcb || !c->up) return;
    bool sent = false;
    while (c->len > 0) {
        uint32_t n = c->len;
        if (n > c->size - c->start) n = c->size - c->start;
        if (n > tcp_sndbuf(c->pcb)) n = tcp_sndbuf(c->pcb);
        if (n == 0) break;
        if (tcp_write(c->pcb, &c->out[c->start], (u16_t)n, TCP_WRITE_FLAG_COPY) != ERR_OK) { c->write_errors++; break; }
        c->start = (c->start + n) % c->size;
        c->len -= n;
        sent = true;
    }
    if (sent && tcp_output(c->pcb) != ERR_OK) c->output_errors++;
}

static void on_announcement(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    (void)arg; (void)pcb; (void)port;
    char text[32] = {0};
    pbuf_copy_partial(p, text, sizeof text - 1, 0);
    pbuf_free(p);
    size_t prefix = strlen(ANNOUNCE_TEXT);
    if (strncmp(text, ANNOUNCE_TEXT, prefix) != 0) return;
    char *end;
    long server = strtol(text + prefix, &end, 10), data_server = strtol(end, NULL, 10);
    if (server <= 0 || server > 65535) return;
    announced = true;
    announced_ip = *addr;
    announced_port = (uint16_t)server;
    announced_data_port = data_server > 0 && data_server <= 65535 ? (uint16_t)data_server : DATA_PORT;
}

static err_t on_connected(void *arg, struct tcp_pcb *pcb, err_t err) {
    (void)pcb; (void)err;
    conn_t *c = arg;
    c->up = true;
    last_heard = get_absolute_time();
    send_out(c);
    return ERR_OK;
}

static err_t on_received(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    (void)err;
    conn_t *c = arg;
    if (!p) { c->lost = true; return ERR_OK; } // the server closed it
    last_heard = get_absolute_time();
    for (struct pbuf *q = p; q; q = q->next) {
        for (u16_t i = 0; i < q->len; i++) {
            char ch = ((const char *)q->payload)[i];
            if (ch && in_len < IN_SIZE) in[(in_start + in_len++) % IN_SIZE] = ch; // '\0' is the heartbeat
        }
    }
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static err_t on_sent(void *arg, struct tcp_pcb *pcb, u16_t len) {
    (void)pcb; (void)len;
    send_out(arg);
    return ERR_OK;
}

static void on_error(void *arg, err_t err) {
    (void)err;
    conn_t *c = arg;
    c->pcb = NULL; // lwIP has freed it
    c->lost = true;
}

// Closes the connection; what waits to go out is kept for the next one.
static void drop(conn_t *c) {
    if (c->pcb) {
        tcp_err(c->pcb, NULL);
        tcp_recv(c->pcb, NULL);
        tcp_sent(c->pcb, NULL);
        tcp_abort(c->pcb);
        c->pcb = NULL;
    }
    c->up = c->lost = false;
}

static bool connect_conn(conn_t *c, const ip_addr_t *ip, uint16_t port) {
    c->pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!c->pcb) return false;
    tcp_nagle_disable(c->pcb); // keys and lines at once
    tcp_arg(c->pcb, c);
    tcp_recv(c->pcb, on_received);
    tcp_sent(c->pcb, on_sent);
    tcp_err(c->pcb, on_error);
    if (tcp_connect(c->pcb, ip, port, on_connected) != ERR_OK) { drop(c); return false; }
    return true;
}

// --- stdio: the second output and input next to USB ---

static void out_chars(const char *buf, int len) {
    cyw43_arch_lwip_begin();
    keep_out(&console, buf, (uint32_t)len);
    send_out(&console);
    cyw43_arch_lwip_end();
}

static int in_chars(char *buf, int len) {
    int n = 0;
    cyw43_arch_lwip_begin();
    while (n < len && in_len > 0) {
        buf[n++] = in[in_start];
        in_start = (in_start + 1) % IN_SIZE;
        in_len--;
    }
    cyw43_arch_lwip_end();
    return n ? n : PICO_ERROR_NO_DATA;
}

static stdio_driver_t driver = {
    .out_chars = out_chars,
    .in_chars = in_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = false, // the server gets plain '\n'
#endif
};

// --- the main loop's side ---

static int32_t signal_dbm(void) {
    int32_t rssi = 0;
    cyw43_wifi_get_rssi(&cyw43_state, &rssi);
    return rssi;
}

static const char *own_ip(void) {
    return ip4addr_ntoa(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA]));
}

static void join_failed(const char *why) {
    printf("WiFi: failed: %s (trying again in %d s)\n", why, JOIN_RETRY_US / 1000000);
    cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    state = WIFI_JOIN_FAILED;
    deadline = make_timeout_time_us(JOIN_RETRY_US);
    settled = true;
}

static void join(void) {
    printf("WiFi: connecting to \"%s\"...\n", WIFI_SSID);
    state = WIFI_JOINING;
    deadline = make_timeout_time_us(JOIN_TIMEOUT_US);
    if (cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK) != 0)
        join_failed("the WiFi chip refused to start joining");
}

static void look_for_server(void) {
    cyw43_arch_lwip_begin();
    drop(&console);
    announced = false;
    cyw43_arch_lwip_end();
    state = WIFI_LOOKING;
    deadline = make_timeout_time_us(FIND_SERVER_US);
}

static void connect_to(const ip_addr_t *ip, uint16_t port, uint16_t data) {
    server_ip = *ip;
    server_port = port;
    data_port = data;
    cyw43_arch_lwip_begin();
    bool ok = connect_conn(&console, ip, port);
    cyw43_arch_lwip_end();
    if (!ok) { printf("Server: out of memory for the connection, looking again...\n"); look_for_server(); return; }
    state = WIFI_CONNECTING;
    deadline = make_timeout_time_us(CONNECT_TIMEOUT_US);
}

static void update_joining(void) {
    int link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    if (link == CYW43_LINK_UP) {
        printf("WiFi: connected, IP %s, signal %ld dBm\n", own_ip(), (long)signal_dbm());
        printf("Server: looking for it...\n");
        look_for_server();
    } else if (link == CYW43_LINK_BADAUTH) join_failed("wrong password");
    else if (link == CYW43_LINK_NONET) join_failed("network not found");
    else if (link == CYW43_LINK_FAIL) join_failed("the network refused");
    else if (time_reached(deadline)) join_failed("no answer after 15 s");
}

static void update_server(void) {
    cyw43_arch_lwip_begin();
    bool found = announced, up = console.up, lost = console.lost;
    ip_addr_t found_ip = announced_ip;
    uint16_t found_port = announced_port, found_data_port = announced_data_port;
    int64_t silent_us = absolute_time_diff_us(last_heard, get_absolute_time());
    cyw43_arch_lwip_end();

    if (state == WIFI_LOOKING) {
        if (found) {
            printf("Server: found at %s:%u, connecting...\n", ipaddr_ntoa(&found_ip), (unsigned)found_port);
            connect_to(&found_ip, found_port, found_data_port);
        } else if (time_reached(deadline)) {
            settled = true;
            ip_addr_t fallback;
            if (*WIFI_SERVER_IP && ipaddr_aton(WIFI_SERVER_IP, &fallback)) {
                printf("Server: no announcement, trying WIFI_SERVER_IP %s:%u...\n", WIFI_SERVER_IP, SERVER_PORT);
                connect_to(&fallback, SERVER_PORT, DATA_PORT);
            } else {
                deadline = make_timeout_time_us(FIND_SERVER_US);
            }
        }
    } else if (state == WIFI_CONNECTING) {
        if (lost) { printf("Server: couldn't connect, looking again...\n"); look_for_server(); }
        else if (up) {
            state = WIFI_CONNECTED;
            settled = true;
            next_heartbeat = get_absolute_time();
            console_connections++;
            printf("Server: connected\n");
        } else if (time_reached(deadline)) { printf("Server: no answer, looking again...\n"); look_for_server(); }
    } else if (state == WIFI_CONNECTED) {
        if (lost) { look_for_server(); printf("Server: lost (connection closed), looking again...\n"); }
        else if (silent_us > SILENCE_US) {
            look_for_server();
            printf("Server: lost (no answer for %d s), looking again...\n", SILENCE_US / 1000000);
        } else if (time_reached(next_heartbeat)) {
            next_heartbeat = make_timeout_time_us(HEARTBEAT_US);
            char beat[24];
            int n = snprintf(beat, sizeof beat, "\001rssi %ld\n", (long)signal_dbm());
            cyw43_arch_lwip_begin();
            keep_out(&console, beat, (uint32_t)n);
            send_out(&console);
            cyw43_arch_lwip_end();
        }
    }
}

void wifi_console_start(void) {
    if (state != WIFI_OFF) { wifi_console_print_status(); return; }
    state = WIFI_UNAVAILABLE;
    settled = true;
    if (!*WIFI_SSID) {
        unavailable_why = "no network set: copy picoA/app/wifi_config.example.h to wifi_config.h, fill it in and rebuild";
    } else if (cyw43_arch_init_with_country(WIFI_COUNTRY) != 0) {
        unavailable_why = "the WiFi chip didn't start";
    } else {
        cyw43_arch_enable_sta_mode();
        cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM); // answers keys at once, doesn't miss broadcasts
        cyw43_arch_lwip_begin();
        udp = udp_new_ip_type(IPADDR_TYPE_V4);
        bool listening = udp && udp_bind(udp, IP_ANY_TYPE, ANNOUNCE_PORT) == ERR_OK;
        if (listening) udp_recv(udp, on_announcement, NULL);
        cyw43_arch_lwip_end();
        if (!listening) {
            unavailable_why = "can't listen for the server's announcements";
        } else {
            settled = false;
            stdio_set_driver_enabled(&driver, true);
            join();
            return;
        }
    }
    printf("WiFi: %s\n", unavailable_why);
}

bool wifi_console_connected(void) { return state == WIFI_CONNECTED; }
bool wifi_console_settled(void) { return settled; }

void wifi_console_print_status(void) {
    switch (state) {
    case WIFI_OFF: printf("WiFi: off (w connects)\n"); return;
    case WIFI_UNAVAILABLE: printf("WiFi: %s\n", unavailable_why); return;
    case WIFI_JOINING: printf("WiFi: connecting to \"%s\"...\n", WIFI_SSID); return;
    case WIFI_JOIN_FAILED:
        printf("WiFi: not connected, trying again in %lld s\n",
               (long long)(absolute_time_diff_us(get_absolute_time(), deadline) / 1000000));
        return;
    default: break;
    }
    printf("WiFi: \"%s\", IP %s, signal %ld dBm; ", WIFI_SSID, own_ip(), (long)signal_dbm());
    if (state == WIFI_LOOKING) printf("looking for the server\n");
    else if (state != WIFI_CONNECTED) printf("server %s:%u connecting...\n", ipaddr_ntoa(&server_ip), (unsigned)server_port);
    else printf("server %s:%u connected, recordings to UDP port %u\n", ipaddr_ntoa(&server_ip),
                (unsigned)server_port, (unsigned)data_port);
}

uint32_t wifi_console_data_connection(void) { return state == WIFI_CONNECTED ? console_connections : 0; }

bool wifi_console_data_send(const void *buf, uint32_t len) {
    if (state != WIFI_CONNECTED || len > 0xffff) return false;
    cyw43_arch_lwip_begin();
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)len, PBUF_RAM);
    bool ok = p != NULL;
    if (ok) {
        memcpy(p->payload, buf, len);
        ok = udp_sendto(udp, p, &server_ip, data_port) == ERR_OK;
        pbuf_free(p);
    }
    cyw43_arch_lwip_end();
    return ok;
}

// What TCP is doing with a connection's data (lock held).
static void conn_status(const conn_t *c, wifi_conn_status_t *s) {
    s->waiting = c->len;
    s->write_errors = c->write_errors;
    s->output_errors = c->output_errors;
    if (!c->pcb || !c->up) return;
    s->unacked = c->pcb->snd_lbb - c->pcb->lastack;
    s->segments = c->pcb->snd_queuelen;
    s->retries = c->pcb->nrtx;
    s->rto_ms = (uint16_t)(c->pcb->rto * LWIP_SLOW_MS);
    s->cwnd = c->pcb->cwnd;
    s->peer_window = c->pcb->snd_wnd;
}

void wifi_console_link_status(wifi_link_status_t *s) {
    memset(s, 0, sizeof *s);
    if (state != WIFI_CONNECTED) return;
    s->rssi_dbm = (int8_t)signal_dbm();
    cyw43_arch_lwip_begin();
    conn_status(&console, &s->console);
    s->console_silent_ms = (uint32_t)(absolute_time_diff_us(last_heard, get_absolute_time()) / 1000);
    cyw43_arch_lwip_end();
}


void wifi_console_update(void) {
    switch (state) {
    case WIFI_JOINING: update_joining(); break;
    case WIFI_JOIN_FAILED: if (time_reached(deadline)) join(); break;
    case WIFI_LOOKING: case WIFI_CONNECTING: case WIFI_CONNECTED:
        if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) {
            look_for_server();
            printf("WiFi: lost, reconnecting...\n");
            cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
            join();
        } else {
            update_server();
        }
        break;
    default: break;
    }
}
