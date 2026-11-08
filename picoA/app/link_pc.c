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
#include "link_pc.h"

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

#define ANNOUNCE_PORT      4210     // the server's "ROBOCAR-SERVER <port>" broadcasts
#define SERVER_PORT        4211     // for WIFI_SERVER_IP
#define ANNOUNCE_TEXT      "ROBOCAR-SERVER "
#define JOIN_TIMEOUT_US    15000000
#define JOIN_RETRY_US      30000000
#define FIND_SERVER_US     5000000  // then WIFI_SERVER_IP is tried
#define CONNECT_TIMEOUT_US 5000000
#define SIGNAL_US          2000000  // the signal asked from the chip this often (~1 ms), for status
#define STATUS_US          250000   // status this often while connected
#define SILENCE_US         30000000 // nothing from the server (it sends a heartbeat every 2 s) for this long: it's gone
#define CONTROL_SIZE       16384    // the control lane (link_pc.h)
#define BULK_SIZE          40960    // the bulk lane: ~10 s of tof frames and imu batches (doc/DATA_FLOW_PLAN.md, budgets)
#define BULK_IN_FLIGHT     (4 * WIRE_MSG_MAX)  // bulk is handed to TCP only while less than this is unacknowledged
#define IN_SIZE            256      // from the server: commands (≤ 53 B) and heartbeats

typedef enum {
    WIFI_OFF, WIFI_UNAVAILABLE, WIFI_JOINING, WIFI_JOIN_FAILED,
    WIFI_LOOKING, WIFI_CONNECTING, WIFI_CONNECTED,
} wifi_state_t;

// Only the main loop touches these.
static wifi_state_t state;
static const char *unavailable_why;
static absolute_time_t deadline; // what it means depends on the state
static absolute_time_t next_signal, next_status;
static int8_t signal_now_dbm;           // asked every SIGNAL_US while connected
static ip_addr_t server_ip;
static uint16_t server_port;
static uint32_t connections;            // counts the connections that came up
static link_pc_status_fn status_fn;

// Shared with lwIP's callbacks, which run in the background: only with the lwIP
// lock held (cyw43_arch_lwip_begin/end; callbacks already hold it).
static struct udp_pcb *udp;
static struct tcp_pcb *pcb;             // the connection to the server, NULL if none
static bool up, lost;
static uint32_t write_errors, output_errors; // tcp_write() / tcp_output() refused, since power-up (status)
static bool announced;
static ip_addr_t announced_ip;
static uint16_t announced_port;
static absolute_time_t last_heard;
static uint8_t hello[WIRE_HEADER + WIRE_HELLO_SIZE]; // this boot's, sent first on each connection
static uint32_t hello_len;
// From the server, not yet read by link_received(). broken: what came isn't messages.
static uint8_t in[IN_SIZE];
static uint32_t in_len;
static int32_t broken;                  // 0, or a wire_error_t (or WIRE_TOO_LONG when in overflowed)
// The control lane: whole messages, oldest first.
static uint8_t control[CONTROL_SIZE];
static uint32_t control_len, control_dropped;
// The bulk lane: a ring of whole messages.
static uint8_t bulk[BULK_SIZE];
static uint32_t bulk_start, bulk_len, bulk_refused;
// The console line being printed (main loop; under the lock with the rest).
static char line[WIRE_CONSOLE_MAX];
static uint32_t line_len;

// --- the lanes (lock held) ---

static uint16_t payload_len(const uint8_t *msg) { return (uint16_t)(msg[1] | msg[2] << 8); }

static void control_pop(uint32_t n) {
    memmove(control, control + n, control_len - n);
    control_len -= n;
}

// Adds a message; the oldest ones go to make room, whatever their type (answers
// too: decided, see link_pc.h).
static void control_add(const uint8_t *msg, uint32_t len) {
    while (control_len + len > CONTROL_SIZE) {
        control_pop(WIRE_HEADER + payload_len(control));
        control_dropped++;
    }
    memcpy(control + control_len, msg, len);
    control_len += len;
}

// The length of the whole messages at the start of control that fit in `room`.
static uint32_t control_fitting(uint32_t room) {
    uint32_t n = 0;
    while (n < control_len) {
        uint32_t m = WIRE_HEADER + payload_len(control + n);
        if (n + m > room) break;
        n += m;
    }
    return n;
}

static uint8_t bulk_at(uint32_t i) { return bulk[(bulk_start + i) % BULK_SIZE]; }
static uint32_t bulk_next_len(void) { return WIRE_HEADER + (uint32_t)(bulk_at(1) | bulk_at(2) << 8); }

static void bulk_pop(uint32_t n) {
    bulk_start = (bulk_start + n) % BULK_SIZE;
    bulk_len -= n;
}

// Hands TCP what fits: control first, as many whole messages as fit; then bulk, one
// whole message at a time, only once control is empty and with at most
// BULK_IN_FLIGHT unacknowledged (a TCP stream is one queue: a control message
// written now waits behind everything handed to TCP before it). Messages go whole,
// so the lanes never cut into each other.
static void send_out(void) {
    if (!pcb || !up || lost) return; // lost: closed by the server, link_pc_update() hasn't dropped it yet
    bool sent = false;
    uint32_t n = control_fitting(tcp_sndbuf(pcb));
    if (n) {
        if (tcp_write(pcb, control, (u16_t)n, TCP_WRITE_FLAG_COPY) == ERR_OK) {
            control_pop(n);
            sent = true;
        } else write_errors++;
    }
    while (control_len == 0 && bulk_len) {
        uint32_t m = bulk_next_len(), room = tcp_sndbuf(pcb);
        if (room < m || TCP_SND_BUF - room + m > BULK_IN_FLIGHT) break;
        static uint8_t whole[WIRE_MSG_MAX]; // a message that wraps round the ring, put together; static: this also runs in lwIP's interrupt
        const uint8_t *msg = bulk + bulk_start;
        if (bulk_start + m > BULK_SIZE) {
            for (uint32_t i = 0; i < m; i++) whole[i] = bulk_at(i);
            msg = whole;
        }
        if (tcp_write(pcb, msg, (u16_t)m, TCP_WRITE_FLAG_COPY) != ERR_OK) { write_errors++; break; }
        bulk_pop(m);
        sent = true;
    }
    if (sent && tcp_output(pcb) != ERR_OK) output_errors++;
}

// --- lwIP side (lock held) ---

static void on_announcement(void *arg, struct udp_pcb *p_udp, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    (void)arg; (void)p_udp; (void)port;
    char text[32] = {0};
    pbuf_copy_partial(p, text, sizeof text - 1, 0);
    pbuf_free(p);
    size_t prefix = strlen(ANNOUNCE_TEXT);
    if (strncmp(text, ANNOUNCE_TEXT, prefix) != 0) return;
    char *end;
    long server = strtol(text + prefix, &end, 10);
    if (server <= 0 || server > 65535) return;
    announced = true;
    announced_ip = *addr;
    announced_port = (uint16_t)server;
}

static err_t on_connected(void *arg, struct tcp_pcb *p, err_t err) {
    (void)arg; (void)err;
    last_heard = get_absolute_time();
    // Hello first, before anything kept. If TCP refuses it, the connection is no use.
    if (tcp_write(p, hello, (u16_t)hello_len, TCP_WRITE_FLAG_COPY) != ERR_OK) { write_errors++; lost = true; return ERR_OK; }
    up = true;
    send_out();
    return ERR_OK;
}

static err_t on_received(void *arg, struct tcp_pcb *p, struct pbuf *b, err_t err) {
    (void)arg; (void)err;
    if (!b) { lost = true; return ERR_OK; } // the server closed it
    last_heard = get_absolute_time();
    for (struct pbuf *q = b; q; q = q->next) {
        if (in_len + q->len > IN_SIZE) { broken = WIRE_TOO_LONG; break; } // the robot reads commands every loop: not a real server
        memcpy(in + in_len, q->payload, q->len);
        in_len += q->len;
    }
    tcp_recved(p, b->tot_len);
    pbuf_free(b);
    return ERR_OK;
}

static err_t on_sent(void *arg, struct tcp_pcb *p, u16_t len) {
    (void)arg; (void)p; (void)len;
    send_out();
    return ERR_OK;
}

static void on_error(void *arg, err_t err) {
    (void)arg; (void)err;
    pcb = NULL; // lwIP has freed it
    lost = true;
}

// Closes the connection. Control keeps what waits; bulk is dropped (its messages
// were for this connection).
static void drop(void) {
    if (pcb) {
        tcp_err(pcb, NULL);
        tcp_recv(pcb, NULL);
        tcp_sent(pcb, NULL);
        tcp_abort(pcb);
        pcb = NULL;
    }
    up = lost = false;
    in_len = 0;
    broken = 0;
    bulk_start = bulk_len = 0;
}

static bool connect_pcb(const ip_addr_t *ip, uint16_t port) {
    pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!pcb) return false;
    tcp_nagle_disable(pcb); // answers at once
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, on_received);
    tcp_sent(pcb, on_sent);
    tcp_err(pcb, on_error);
    if (tcp_connect(pcb, ip, port, on_connected) != ERR_OK) { drop(); return false; }
    return true;
}

// --- stdio: everything printed, one console message per line ---

static void line_out(void) {
    uint8_t msg[WIRE_HEADER + WIRE_CONSOLE_MAX];
    uint32_t n = wire_encode_console(line, line_len, msg);
    control_add(msg, n);
    line_len = 0;
}

static void out_chars(const char *buf, int len) {
    cyw43_arch_lwip_begin();
    for (int i = 0; i < len; i++) {
        if (buf[i] == '\n') line_out();
        else if (buf[i] != '\r') {
            if (line_len == WIRE_CONSOLE_MAX) line_out(); // a longer line comes as several
            line[line_len++] = buf[i];
        }
    }
    send_out();
    cyw43_arch_lwip_end();
}

static stdio_driver_t driver = {
    .out_chars = out_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = false, // lines end in '\n' alone
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
    drop();
    announced = false;
    cyw43_arch_lwip_end();
    state = WIFI_LOOKING;
    deadline = make_timeout_time_us(FIND_SERVER_US);
}

static void connect_to(const ip_addr_t *ip, uint16_t port) {
    server_ip = *ip;
    server_port = port;
    cyw43_arch_lwip_begin();
    bool ok = connect_pcb(ip, port);
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

// Status, if the robot gave a function for it and control is empty (so statuses
// never pile up behind a slow link).
static void send_status(void) {
    if (!status_fn) return;
    wire_status_t s;
    memset(&s, 0, sizeof s);
    status_fn(&s);
    s.rssi_dbm = signal_now_dbm;
    uint8_t msg[WIRE_HEADER + WIRE_STATUS_SIZE];
    uint32_t n = wire_encode_status(&s, msg);
    cyw43_arch_lwip_begin();
    if (control_len == 0) control_add(msg, n);
    send_out();
    cyw43_arch_lwip_end();
}

static void update_server(void) {
    cyw43_arch_lwip_begin();
    bool found = announced, is_up = up, is_lost = lost;
    int32_t why = broken;
    ip_addr_t found_ip = announced_ip;
    uint16_t found_port = announced_port;
    int64_t silent_us = absolute_time_diff_us(last_heard, get_absolute_time());
    cyw43_arch_lwip_end();

    if (state == WIFI_LOOKING) {
        if (found) {
            printf("Server: found at %s:%u, connecting...\n", ipaddr_ntoa(&found_ip), (unsigned)found_port);
            connect_to(&found_ip, found_port);
        } else if (time_reached(deadline)) {
            ip_addr_t fallback;
            if (*WIFI_SERVER_IP && ipaddr_aton(WIFI_SERVER_IP, &fallback)) {
                printf("Server: no announcement, trying WIFI_SERVER_IP %s:%d...\n", WIFI_SERVER_IP, SERVER_PORT);
                connect_to(&fallback, SERVER_PORT);
            } else {
                deadline = make_timeout_time_us(FIND_SERVER_US);
            }
        }
    } else if (state == WIFI_CONNECTING) {
        if (is_lost) { printf("Server: couldn't connect, looking again...\n"); look_for_server(); }
        else if (is_up) {
            state = WIFI_CONNECTED;
            next_signal = next_status = get_absolute_time();
            connections++;
            printf("Server: connected\n");
        } else if (time_reached(deadline)) { printf("Server: no answer, looking again...\n"); look_for_server(); }
    } else if (state == WIFI_CONNECTED) {
        if (is_lost) { look_for_server(); printf("Server: lost (connection closed), looking again...\n"); }
        else if (why) {
            look_for_server();
            printf("Server: sent something that isn't a message (%s): closing, looking again...\n", wire_error_text(why));
        } else if (silent_us > SILENCE_US) {
            look_for_server();
            printf("Server: nothing from it for %d s: closing, looking again...\n", SILENCE_US / 1000000);
        } else {
            if (time_reached(next_signal)) {
                next_signal = make_timeout_time_us(SIGNAL_US);
                signal_now_dbm = (int8_t)signal_dbm();
            }
            if (time_reached(next_status)) {
                next_status = make_timeout_time_us(STATUS_US);
                send_status();
            }
        }
    }
}

void link_pc_start(uint32_t boot_id, const char *build, link_pc_status_fn status) {
    if (state != WIFI_OFF) { link_pc_print_status(); return; }
    if (!wire_check()) printf("Wire: two message types share a number (wire.c, tof_format.c): the PC can't read them\n");
    wire_hello_t h = {.version = WIRE_VERSION, .boot_id = boot_id};
    snprintf(h.build, sizeof h.build, "%s", build);
    hello_len = wire_encode_hello(&h, hello);
    status_fn = status;
    state = WIFI_UNAVAILABLE;
    if (!*WIFI_SSID) {
        unavailable_why = "no network set: copy picoA/app/wifi_config.example.h to wifi_config.h, fill it in and rebuild";
    } else if (cyw43_arch_init_with_country(WIFI_COUNTRY) != 0) {
        unavailable_why = "the WiFi chip didn't start";
    } else {
        cyw43_arch_enable_sta_mode();
        cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM); // answers at once, doesn't miss broadcasts
        cyw43_arch_lwip_begin();
        udp = udp_new_ip_type(IPADDR_TYPE_V4);
        bool listening = udp && udp_bind(udp, IP_ANY_TYPE, ANNOUNCE_PORT) == ERR_OK;
        if (listening) udp_recv(udp, on_announcement, NULL);
        cyw43_arch_lwip_end();
        if (!listening) {
            unavailable_why = "can't listen for the server's announcements";
        } else {
            stdio_set_driver_enabled(&driver, true);
            join();
            return;
        }
    }
    printf("WiFi: %s\n", unavailable_why);
}

bool link_pc_connected(void) { return state == WIFI_CONNECTED; }
uint32_t link_pc_connection(void) { return state == WIFI_CONNECTED ? connections : 0; }

void link_pc_print_status(void) {
    switch (state) {
    case WIFI_OFF: printf("WiFi: off\n"); return;
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
    else printf("server %s:%u connected\n", ipaddr_ntoa(&server_ip), (unsigned)server_port);
    cyw43_arch_lwip_begin();
    uint32_t c = control_len, cd = control_dropped, b = bulk_len, br = bulk_refused, we = write_errors + output_errors;
    cyw43_arch_lwip_end();
    printf("Link: control %lu B waiting (%lu dropped); bulk %lu B waiting (%lu refused); TCP refused %lu writes\n",
           (unsigned long)c, (unsigned long)cd, (unsigned long)b, (unsigned long)br, (unsigned long)we);
}

bool link_send_control(const uint8_t *msg, uint32_t len) {
    wire_type_info_t info;
    if (len < WIRE_HEADER || !wire_info(wire_type(msg), &info) || info.lane != WIRE_LANE_CONTROL) return false;
    if (state == WIFI_OFF || state == WIFI_UNAVAILABLE) return false;
    cyw43_arch_lwip_begin();
    control_add(msg, len);
    send_out();
    cyw43_arch_lwip_end();
    return true;
}

bool link_send_bulk(const uint8_t *msg, uint32_t len) {
    wire_type_info_t info;
    if (len < WIRE_HEADER || len > WIRE_MSG_MAX || !wire_info(wire_type(msg), &info) || info.lane != WIRE_LANE_BULK)
        return false;
    cyw43_arch_lwip_begin();
    bool ok = state == WIFI_CONNECTED && bulk_len + len <= BULK_SIZE;
    if (ok) {
        for (uint32_t i = 0; i < len; i++) bulk[(bulk_start + bulk_len + i) % BULK_SIZE] = msg[i];
        bulk_len += len;
        send_out();
    } else bulk_refused++;
    cyw43_arch_lwip_end();
    return ok;
}

bool link_received(wire_command_t *c) {
    bool got = false;
    cyw43_arch_lwip_begin();
    while (!got && !broken) {
        wire_msg_t m;
        int32_t n = wire_take(in, in_len, &m);
        if (n == 0) break;
        bool heartbeat = n > 0 && m.type == WIRE_HEARTBEAT;
        got = !heartbeat && n > 0 && wire_decode_command(&m, c);
        if (!heartbeat && !got) {
            broken = n < 0 ? n : WIRE_UNKNOWN_TYPE; // not a message the robot takes: link_pc_update() closes it
            break;
        }
        memmove(in, in + n, in_len - (uint32_t)n);
        in_len -= (uint32_t)n;
    }
    cyw43_arch_lwip_end();
    return got;
}

void link_pc_update(void) {
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
