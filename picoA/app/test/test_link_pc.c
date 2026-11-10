// Host test for picoA/app/link_pc.c: joining (and failing to join) the WiFi, finding
// the server by its announcements, hello first on each connection, console lines
// and status as messages, commands and heartbeats from the server, output kept
// while no server listens (the newest 16 KB), the two lanes (control before bulk,
// bulk first in first out and paced, refused and counted when full, bulk dropped
// with a lost connection), console output under load, a server that sends something that
// isn't a message, losing the server or the WiFi (a new connection number each time; an old
// server's announcement with a second port still read),
// against a fake WiFi chip and lwIP with the test playing the server. Every byte
// the server gets is read back with wire_take(): whole messages only. Run from the
// repo root:
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_link_pc picoA/app/test/test_link_pc.c && build/test_link_pc
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define WIFI_SSID     "TestNet"
#define WIFI_PASSWORD "secret"
#define WIFI_SERVER_IP "10.0.0.9"
#include "../tof_format.c"
#include "../wire.c"
// What link_pc prints, kept to check its lines (and printed as before).
#include <stdarg.h>
static char log_text[1 << 16];
static size_t log_len;
static int test_printf(const char *fmt, ...) {
    va_list a, b;
    va_start(a, fmt);
    va_copy(b, a);
    int n = vsnprintf(log_text + log_len, sizeof log_text - log_len, fmt, a);
    if (n > 0 && log_len + (size_t)n < sizeof log_text) log_len += (size_t)n; else log_len = 0;
    vprintf(fmt, b);
    va_end(a);
    va_end(b);
    return n;
}
#define printf test_printf
#include "../link_pc.c"
#undef printf

#define BOOT 0x1234abcd

// Runs the main loop for `ms` milliseconds, 1 ms per iteration.
static void run(int ms) {
    for (int i = 0; i < ms; i++) {
        fake_now_us += 1000;
        link_pc_update();
        assert(fake_lock_depth == 0);
    }
}

static void print(const char *text) { fake_stdio->out_chars(text, (int)strlen(text)); }

// The robot's status, as commands_status() would fill it.
static int status_calls;
static void fill_status(wire_status_t *s) {
    status_calls++;
    s->x_m = 1.5f;
    snprintf(s->mode, sizeof s->mode, "manual");
    snprintf(s->action, sizeof s->action, "turn");
}

// --- the server's side ---

// What the server received since the last call, as messages (all whole).
#define MAX_MESSAGES 4096
static uint8_t got_bytes[sizeof fake_sent];
static wire_message_t got[MAX_MESSAGES];
static uint32_t n_got;
static void receive(void) {
    memcpy(got_bytes, fake_sent, fake_sent_len);
    uint32_t len = fake_sent_len, at = 0;
    fake_sent_len = 0;
    n_got = 0;
    while (at < len) {
        int32_t n = wire_take(got_bytes + at, len - at, &got[n_got]);
        assert(n > 0); // whole messages, nothing but messages
        at += (uint32_t)n;
        n_got++;
        assert(n_got < MAX_MESSAGES);
    }
}
static uint32_t count(uint8_t type) {
    uint32_t n = 0;
    for (uint32_t i = 0; i < n_got; i++) n += got[i].type == type;
    return n;
}
// The console lines received, each with its '\n'.
static const char *console_text(void) {
    static char text[sizeof fake_sent];
    uint32_t n = 0;
    for (uint32_t i = 0; i < n_got; i++) {
        if (got[i].type != WIRE_CONSOLE) continue;
        memcpy(text + n, got[i].payload, got[i].len);
        n += got[i].len;
        text[n++] = '\n';
    }
    text[n] = 0;
    return text;
}
static void server_sends_message(const uint8_t *message, uint32_t len) { fake_server_sends((const char *)message, (u16_t)len); }
static void server_command(uint32_t n, const char *text) {
    wire_command_t c = {.n = n};
    snprintf(c.text, sizeof c.text, "%s", text);
    uint8_t message[64] = {0};
    server_sends_message(message, wire_encode_command(&c, message));
}
static void server_heartbeat(void) { uint8_t message[3] = {0}; server_sends_message(message, wire_encode_heartbeat(message)); }

static void join_wifi(void) {
    fake_link = CYW43_LINK_UP;
    cyw43_state.netif[CYW43_ITF_STA].ip = fake_ip(192, 168, 1, 42);
    run(10);
    assert(state == WIFI_LOOKING);
}

static void server_connects(ip_addr_t from, uint16_t port) {
    char text[32];
    snprintf(text, sizeof text, "ROBOCAR-SERVER %u\n", (unsigned)port);
    fake_announce(from, text);
    run(10);
    assert(state == WIFI_CONNECTING && fake_tcp.open);
    assert(fake_tcp.ip.addr == from.addr && fake_tcp.port == port);
    fake_server_accepts();
    run(10);
    assert(link_pc_connected());
}

// The first message on a connection is this boot's hello.
static void assert_hello_first(void) {
    assert(n_got > 0 && got[0].type == WIRE_HELLO);
    wire_hello_t h;
    assert(wire_decode_hello(&got[0], &h) && h.version == WIRE_VERSION && h.boot_id == BOOT && !strcmp(h.build, "Nov  7 2026"));
    assert(count(WIRE_HELLO) == 1);
}

static uint8_t frame[WIRE_HEADER + WIRE_TOF_HEADER + TOF_COMPACT_BYTES]; // a tof frame message (its payload doesn't matter here)
static bool send_frame(void) { return link_pc_send_bulk(frame, sizeof frame); }
static bool send_part(uint8_t fill, uint32_t n) {
    uint8_t bytes[WIRE_PART_MAX], message[WIRE_MESSAGE_MAX];
    memset(bytes, fill, n);
    uint32_t len = wire_encode_snapshot_part(bytes, n, message);
    return link_pc_send_bulk(message, len);
}
static bool send_answer(uint32_t n) {
    wire_answer_t a = {.n = n, .end = WIRE_END_OK};
    uint8_t message[WIRE_HEADER + WIRE_ANSWER_SIZE] = {0};
    return link_pc_send_control(message, wire_encode_answer(&a, message));
}

// --- the tests ---

static void test_join_fails_then_retries(void) {
    assert(!link_pc_connected());
    assert(!send_answer(1) && control_len == 0); // WiFi off: there will be no server, nothing kept
    link_pc_start(BOOT, "Nov  7 2026", fill_status);
    assert(fake_inits == 1 && fake_joins == 1 && strcmp(fake_ssid, "TestNet") == 0);
    assert(fake_pm == CYW43_NONE_PM);
    assert(fake_udp.port == 4210 && fake_stdio && !fake_stdio->in_chars); // commands come as messages, not through stdin
    run(1000);
    assert(state == WIFI_JOINING);
    fake_link = CYW43_LINK_BADAUTH;
    run(10);
    assert(state == WIFI_JOIN_FAILED && fake_leaves == 1);
    run(29 * 1000);
    assert(fake_joins == 1);
    run(1100);
    assert(fake_joins == 2 && state == WIFI_JOINING);
    // No answer at all: gives up after 15 s, tries again 30 s later.
    run(14 * 1000);
    assert(state == WIFI_JOINING);
    run(1100);
    assert(state == WIFI_JOIN_FAILED && fake_leaves == 2);
    run(30 * 1000);
    assert(fake_joins == 3);
    link_pc_start(BOOT, "x", fill_status); // again: only prints the state
    assert(fake_inits == 1 && fake_joins == 3);
    assert(status_calls == 0 && link_pc_connection() == 0);
    printf("ok: join fails, retries after 30 s, 15 s timeout\n");
}

static void test_finds_server_and_talks(void) {
    print("WiFi: before the server\n"); // kept until the server connects
    print("a line in "); print("two prints\n");
    join_wifi();
    // Unrelated broadcasts are ignored.
    fake_announce(fake_ip(192, 168, 1, 7), "HELLO 4211");
    fake_announce(fake_ip(192, 168, 1, 7), "ROBOCAR-SERVER 0");
    run(10);
    assert(state == WIFI_LOOKING && fake_tcp_connects == 0);
    server_connects(fake_ip(192, 168, 1, 10), 4211);
    assert(link_pc_connection() == 1);
    receive();
    assert_hello_first();
    assert(got[1].type == WIRE_CONSOLE && !strcmp(console_text(), "WiFi: before the server\na line in two prints\n"));
    // Status at once, then every 0.25 s: the robot's fields and the WiFi signal.
    assert(count(WIRE_STATUS) == 1);
    wire_status_t s = {0};
    for (uint32_t i = 0; i < n_got; i++) if (got[i].type == WIRE_STATUS) assert(wire_decode_status(&got[i], &s));
    assert(s.x_m == 1.5f && !strcmp(s.mode, "manual") && !strcmp(s.action, "turn") && s.rssi_dbm == -55);
    run(1000);
    receive();
    assert(count(WIRE_STATUS) == 4 && n_got == 4);

    print("x +1.0 cm\n");
    receive();
    assert(n_got == 1 && !strcmp(console_text(), "x +1.0 cm\n"));
    print("\n"); // an empty line is a line
    receive();
    assert(n_got == 1 && got[0].len == 0);

    // Commands arrive as messages; heartbeats are taken and dropped.
    wire_command_t c;
    assert(!link_pc_received(&c));
    server_heartbeat();
    server_command(7, "drive 50");
    server_heartbeat();
    server_command(8, "status");
    assert(link_pc_received(&c) && c.n == 7 && !strcmp(c.text, "drive 50"));
    assert(link_pc_received(&c) && c.n == 8 && !strcmp(c.text, "status"));
    assert(!link_pc_received(&c));
    // A command cut in two by TCP: whole once its second half is there.
    uint8_t message[64] = {0};
    wire_command_t cut = {.n = 9, .text = "rotate 30"};
    uint32_t n = wire_encode_command(&cut, message);
    server_sends_message(message, 5);
    assert(!link_pc_received(&c));
    server_sends_message(message + 5, n - 5);
    assert(link_pc_received(&c) && c.n == 9 && !strcmp(c.text, "rotate 30"));
    assert(fake_recved > 0);

    // The server's heartbeat every 2 s keeps the connection; nothing is sent back for it.
    for (int i = 0; i < 20; i++) { run(1000); server_heartbeat(); }
    assert(link_pc_received(&c) == false && link_pc_connected());
    receive();
    assert(n_got == count(WIRE_STATUS));
    printf("ok: announcement, connect, hello first, kept output, status, commands, heartbeats\n");
}

static void test_full_send_buffer(void) {
    receive();
    fake_sndbuf = 10;
    print("0123456789abcdef\n"); // 19 B: doesn't fit whole, waits
    assert(fake_sent_len == 0);
    fake_server_acks(); // the server acknowledges: room again
    receive();
    assert(n_got == 1 && !strcmp(console_text(), "0123456789abcdef\n"));
    printf("ok: a full send buffer drains when the server acknowledges; messages go whole\n");
}

static void test_server_silent(void) {
    run(6100); // the server says nothing: a slow link, or it's gone
    assert(link_pc_connected());
    run(24000);
    assert(state == WIFI_LOOKING && !fake_tcp.open);
    assert(!link_pc_connected() && link_pc_connection() == 0);
    // While no server listens, the newest 16 KB are kept, the oldest dropped.
    char line[1001];
    memset(line, 'a', 999);
    line[999] = '\n';
    line[1000] = 0;
    for (int i = 0; i < 20; i++) { line[0] = (char)('A' + i); print(line); }
    assert(send_answer(41));
    assert(control_len <= CONTROL_SIZE && control_dropped > 0);
    receive();
    server_connects(fake_ip(192, 168, 1, 11), 5000);
    fake_server_acks(); // the send buffer holds more than 16 KB: all of it in one go
    receive();
    assert_hello_first();
    const char *text = console_text();
    assert(strlen(text) >= CONTROL_SIZE - 1100);
    assert(strchr(text, 'T') && !strchr(text, 'A') && !strchr(text, 'D')); // oldest dropped
    assert(count(WIRE_ANSWER) == 1);
    wire_answer_t a;
    for (uint32_t i = 0; i < n_got; i++) if (got[i].type == WIRE_ANSWER) assert(wire_decode_answer(&got[i], &a) && a.n == 41);
    assert(link_pc_connection() == 2);
    printf("ok: silent server kept 30 s, then dropped; the newest 16 KB kept\n");
}

static void test_server_closes_or_fails(void) {
    fake_server_closes();
    run(10);
    assert(state == WIFI_LOOKING && !fake_tcp.open);
    // A connection that fails while connecting.
    fake_announce(fake_ip(192, 168, 1, 10), "ROBOCAR-SERVER 4211");
    run(10);
    assert(state == WIFI_CONNECTING);
    fake_tcp_fails();
    run(10);
    assert(state == WIFI_LOOKING);
    // One that never answers.
    fake_announce(fake_ip(192, 168, 1, 10), "ROBOCAR-SERVER 4211");
    run(10);
    run(5100);
    assert(state == WIFI_LOOKING && !fake_tcp.open);
    server_connects(fake_ip(192, 168, 1, 10), 4211);
    receive();
    assert_hello_first();
    printf("ok: server closing, refusing, not answering: looks again\n");
}

// A server that sends what isn't a message for the robot: closed, looked for again.
static void test_server_sends_garbage(void) {
    fake_server_sends("drive 50 #3\n", 12); // an old server's command line
    wire_command_t c;
    assert(!link_pc_received(&c));
    run(10);
    assert(state == WIFI_LOOKING && !fake_tcp.open);
    server_connects(fake_ip(192, 168, 1, 10), 4211);
    uint8_t message[64] = {0};
    wire_answer_t a = {.n = 1};
    server_sends_message(message, wire_encode_answer(&a, message)); // a message, but one the robot doesn't take
    assert(!link_pc_received(&c));
    run(10);
    assert(state == WIFI_LOOKING);
    server_connects(fake_ip(192, 168, 1, 10), 4211);
    uint8_t flood[IN_SIZE + 1] = {0};
    for (uint32_t i = 0; i + 3 <= sizeof flood; i += 3) wire_encode_heartbeat(flood + i);
    server_sends_message(flood, sizeof flood); // more than the robot keeps: not a real server
    run(10);
    assert(state == WIFI_LOOKING);
    server_connects(fake_ip(192, 168, 1, 10), 4211);
    receive();
    printf("ok: a server sending text, a robot's message or too much: closed, looked for again\n");
}

static void test_connection_numbers(void) {
    fake_server_closes();
    run(10);
    assert(link_pc_connection() == 0); // no server
    // An old server's announcement, with the recording port it had: the TCP port is read.
    fake_announce(fake_ip(192, 168, 1, 10), "ROBOCAR-SERVER 4211 4212\n");
    run(10);
    assert(fake_tcp.port == 4211);
    fake_server_accepts();
    run(10);
    uint32_t first = link_pc_connection();
    assert(first > 0);
    receive();
    // Lost and found again: a new number.
    fake_server_closes();
    run(10);
    assert(link_pc_connection() == 0);
    fake_announce(fake_ip(192, 168, 1, 10), "ROBOCAR-SERVER 4211\n");
    run(10);
    fake_server_accepts();
    run(10);
    assert(link_pc_connection() > first);
    receive();
    printf("ok: each connection a new number; an old announcement with a second port read\n");
}

static void test_lanes(void) {
    receive();
    // Not a message of that lane: refused.
    assert(!link_pc_send_bulk((const uint8_t *)"\x03\x00\x00", 3));
    uint8_t part_message[WIRE_MESSAGE_MAX] = {0};
    uint8_t one = 1;
    assert(!link_pc_send_control(part_message, wire_encode_snapshot_part(&one, 1, part_message)));
    receive();

    // Control before bulk: frames queued first, then a line, while TCP has no room;
    // when the server acknowledges, the line goes first.
    fake_sndbuf = 0;
    assert(send_frame() && send_frame());
    print("an answer would not wait\n");
    assert(send_answer(5));
    assert(fake_sent_len == 0);
    fake_server_acks();
    receive();
    assert(n_got == 4 && got[0].type == WIRE_CONSOLE && got[1].type == WIRE_ANSWER);
    assert(got[2].type == frame[0] && got[3].type == frame[0] && bulk_len == 0);

    // Bulk is paced: at most BULK_IN_FLIGHT unacknowledged; first in, first out.
    for (int i = 0; i < 30; i++) assert(send_part((uint8_t)i, 400 + (uint32_t)i));
    assert(TCP_SND_BUF - fake_sndbuf <= BULK_IN_FLIGHT && fake_sent_len > 0 && bulk_len > 0);
    print("between\n"); // control goes at once, past the bulk still queued
    assert(fake_sent_len <= BULK_IN_FLIGHT + 11);
    for (int i = 0; i < 20 && bulk_len; i++) fake_server_acks();
    receive();
    assert(bulk_len == 0 && count(WIRE_SNAPSHOT_PART) == 30 && count(WIRE_CONSOLE) == 1);
    uint32_t k = 0;
    for (uint32_t i = 0; i < n_got; i++) {
        if (got[i].type != WIRE_SNAPSHOT_PART) continue;
        assert(got[i].len == 400 + k && got[i].payload[0] == (uint8_t)k); // in order
        k++;
    }
    assert(got[0].type == WIRE_SNAPSHOT_PART && got[n_got - 1].type == WIRE_SNAPSHOT_PART); // the line went among them
    printf("ok: control before bulk; bulk paced (%d B in flight) and first in, first out\n", BULK_IN_FLIGHT);

    // Status only while control is empty: a slow link doesn't pile them up.
    fake_sndbuf = 0;
    run(1000);
    fake_server_acks();
    receive();
    assert(count(WIRE_STATUS) == 1);
    printf("ok: status only while control is empty\n");

    // Full: refused and counted, whatever the message; once it drains, all that got in goes.
    fake_sndbuf = 0; // nothing leaves
    uint32_t refused = bulk_refused, accepted = 0;
    while (send_frame()) accepted++;
    assert(accepted == BULK_SIZE / sizeof frame && bulk_refused == refused + 1);
    assert(!send_part(0xaa, WIRE_PART_MAX) && bulk_refused == refused + 2);
    for (int i = 0; i < 60 && bulk_len; i++) fake_server_acks();
    receive();
    assert(bulk_len == 0 && count(frame[0]) == accepted);
    printf("ok: bulk full: %u frames in, the rest refused and counted\n", (unsigned)accepted);

    // The ring wraps: messages across its end go whole.
    for (int round = 0; round < 3; round++) {
        for (int i = 0; i < 100; i++) assert(send_frame());
        for (int i = 0; i < 60; i++) fake_server_acks();
        receive();
        assert(count(frame[0]) == 100 && bulk_len == 0);
    }
    printf("ok: messages across the ring's end go whole\n");
}

static void test_reconnect_mid_snapshot(void) {
    receive();
    fake_sndbuf = 2000; // a little goes, the rest waits
    for (int i = 0; i < 20; i++) assert(send_part((uint8_t)i, WIRE_PART_MAX));
    for (int i = 0; i < 5; i++) assert(send_frame());
    uint32_t conn = link_pc_connection();
    fake_tcp_fails();
    run(10);
    // The queue for that connection is dropped; nothing is queued without a server.
    assert(bulk_len == 0 && link_pc_connection() == 0);
    assert(!send_part(1, 10) && !send_frame());
    fake_sndbuf = FAKE_SNDBUF;
    fake_sent_len = 0;
    server_connects(fake_ip(192, 168, 1, 10), 4211);
    assert(link_pc_connection() > conn);
    receive();
    assert_hello_first();
    assert(count(WIRE_SNAPSHOT_PART) == 0);
    // A clean close by the server (FIN), before the main loop has noticed: nothing
    // more is handed to TCP.
    fake_sndbuf = 0;
    assert(send_part(2, 100));
    fake_server_closes();
    fake_sndbuf = FAKE_SNDBUF;
    fake_server_acks();
    assert(fake_sent_len == 0);
    run(10);
    assert(state == WIFI_LOOKING);
    server_connects(fake_ip(192, 168, 1, 10), 4211);
    receive();
    printf("ok: a lost connection drops bulk; nothing goes on a closed one; hello first on the next\n");
}

// Console output under load on a slow server: printing never blocks, the oldest
// messages go to make room, the newest arrive in order.
static void test_console_under_load(void) {
    receive();
    fake_sndbuf = 0;
    uint32_t dropped = control_dropped;
    char line[80];
    for (int i = 0; i < 3000; i++) {
        snprintf(line, sizeof line, "line %04d of a robot printing a lot while the link is slow\n", i);
        print(line);
        if (i % 100 == 0) assert(send_answer(1000 + (uint32_t)i));
        if (i % 10 == 0) run(1);
    }
    assert(control_len <= CONTROL_SIZE && control_dropped > dropped);
    for (int i = 0; i < 10; i++) fake_server_acks();
    receive();
    uint32_t last = 0, answers = 0;
    for (uint32_t i = 0; i < n_got; i++) {
        if (got[i].type != WIRE_ANSWER) continue;
        wire_answer_t a;
        assert(wire_decode_answer(&got[i], &a) && a.n > last);
        last = a.n;
        answers++;
    }
    assert(answers > 0 && last == 3900 && strstr(console_text(), "line 2999 of"));
    printf("ok: console under load: %u messages dropped to make room, the newest arrived in order\n",
           (unsigned)(control_dropped - dropped));
}

// TCP refuses hello: the connection is no use, the server looked for again.
static void test_hello_refused(void) {
    fake_server_closes();
    run(10);
    uint32_t errors = write_errors;
    log_len = 0;
    log_text[0] = 0;
    fake_announce(fake_ip(192, 168, 1, 10), "ROBOCAR-SERVER 4211\n");
    run(10);
    fake_tcp_write_err = ERR_MEM;
    fake_server_accepts();
    fake_tcp_write_err = ERR_OK;
    run(10);
    assert(write_errors == errors + 1 && state == WIFI_LOOKING && !fake_tcp.open && !link_pc_connected());
    assert(strstr(log_text, "couldn't connect"));
    fake_sent_len = 0;
    server_connects(fake_ip(192, 168, 1, 10), 4211);
    receive();
    assert_hello_first();
    printf("ok: hello refused by TCP: closed, connected again, hello first\n");
}

static void test_fallback_ip(void) {
    fake_link = CYW43_LINK_DOWN; // the WiFi goes
    run(10);
    assert(state == WIFI_JOINING && !fake_tcp.open);
    join_wifi();
    run(4900);
    assert(state == WIFI_LOOKING);
    run(200); // no announcement for 5 s: WIFI_SERVER_IP
    assert(state == WIFI_CONNECTING && fake_tcp.ip.addr == fake_ip(10, 0, 0, 9).addr && fake_tcp.port == 4211);
    fake_server_accepts();
    run(10);
    assert(link_pc_connected());
    printf("ok: WiFi lost and rejoined; WIFI_SERVER_IP after 5 s without announcements\n");
}

int main(void) {
    wire_header(frame, tof_format_message_type(TOF_FORMAT_COMPACT), WIRE_TOF_HEADER + TOF_COMPACT_BYTES);
    test_join_fails_then_retries();
    test_finds_server_and_talks();
    test_full_send_buffer();
    test_server_silent();
    test_server_closes_or_fails();
    test_server_sends_garbage();
    test_connection_numbers();
    test_lanes();
    test_reconnect_mid_snapshot();
    test_console_under_load();
    test_hello_refused();
    test_fallback_ip();
    printf("link_pc: all tests passed\n");
    return 0;
}
