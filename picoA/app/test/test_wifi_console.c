// Host test for picoA/app/wifi_console.c: joining (and failing to join) the WiFi,
// finding the server by its announcements, the console text and keys over TCP,
// output kept while no server listens, heartbeats, losing the server or the
// WiFi, and recordings as UDP datagrams (to the announced port, refusals
// reported), against a fake
// WiFi chip and lwIP with the test playing the server. Run
// from the repo root:
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_wifi_console picoA/app/test/test_wifi_console.c && build/test_wifi_console
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define WIFI_SSID     "TestNet"
#define WIFI_PASSWORD "secret"
#define WIFI_SERVER_IP "10.0.0.9"
#include "../wifi_console.c"

#define S_US 1000000

// Runs the main loop for `ms` milliseconds, 1 ms per iteration.
static void run(int ms) {
    for (int i = 0; i < ms; i++) {
        fake_now_us += 1000;
        wifi_console_update();
        assert(fake_lock_depth == 0);
    }
}

static void print(const char *text) { fake_stdio->out_chars(text, (int)strlen(text)); }

static int read_keys(char *buf, int len) {
    int n = fake_stdio->in_chars(buf, len);
    return n == PICO_ERROR_NO_DATA ? 0 : n;
}

// What the server received since the last call, heartbeats included.
static const char *received(void) {
    static char text[sizeof fake_sent + 1];
    memcpy(text, fake_sent, fake_sent_len);
    text[fake_sent_len] = 0;
    fake_sent_len = 0;
    return text;
}

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
    assert(wifi_console_connected());
}

static void test_join_fails_then_retries(void) {
    assert(!wifi_console_connected() && !wifi_console_settled());
    wifi_console_start();
    assert(fake_inits == 1 && fake_joins == 1 && strcmp(fake_ssid, "TestNet") == 0);
    assert(fake_pm == CYW43_NONE_PM);
    assert(fake_udp.port == 4210 && fake_stdio);
    run(1000);
    assert(state == WIFI_JOINING && !wifi_console_settled());
    fake_link = CYW43_LINK_BADAUTH;
    run(10);
    assert(state == WIFI_JOIN_FAILED && wifi_console_settled() && fake_leaves == 1);
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
    wifi_console_start(); // again: only prints the state
    assert(fake_inits == 1 && fake_joins == 3);
    printf("ok: join fails, retries after 30 s, 15 s timeout\n");
}

static void test_finds_server_and_talks(void) {
    print("WiFi: before the server\n"); // kept until the server connects
    join_wifi();
    // Unrelated broadcasts are ignored.
    fake_announce(fake_ip(192, 168, 1, 7), "HELLO 4211");
    fake_announce(fake_ip(192, 168, 1, 7), "ROBOCAR-SERVER 0");
    run(10);
    assert(state == WIFI_LOOKING && fake_tcp_connects == 0);
    server_connects(fake_ip(192, 168, 1, 10), 4211);
    assert(wifi_console_settled());
    const char *got = received();
    assert(strstr(got, "WiFi: before the server\n") == got);
    assert(strstr(got, "\001rssi -55\n")); // first heartbeat right away

    print("x +1.0 cm\n");
    assert(strcmp(received(), "x +1.0 cm\n") == 0);

    // Keys arrive through stdio; the server's '\0' heartbeats don't.
    char keys[8];
    assert(read_keys(keys, sizeof keys) == 0);
    fake_server_sends("p\0m", 3);
    assert(read_keys(keys, sizeof keys) == 2 && keys[0] == 'p' && keys[1] == 'm');
    assert(fake_recved == 3);

    // Heartbeats every 2 s, while the server keeps talking.
    for (int i = 0; i < 5; i++) { run(1000); fake_server_sends("\0", 1); }
    got = received();
    int beats = 0;
    for (const char *s = got; (s = strstr(s, "\001rssi")); s++) beats++;
    assert(beats == 2 || beats == 3);
    assert(wifi_console_connected());
    printf("ok: announcement, connect, kept output sent, keys, heartbeats\n");
}

static void test_full_send_buffer(void) {
    fake_sndbuf = 10;
    print("0123456789abcdef\n");
    assert(fake_sent_len == 10);
    fake_server_acks(); // the rest goes out when the server acknowledges
    assert(strcmp(received(), "0123456789abcdef\n") == 0);
    printf("ok: a full send buffer drains when the server acknowledges\n");
}

static void test_server_silent(void) {
    run(6100); // the server says nothing
    assert(state == WIFI_LOOKING && !fake_tcp.open);
    assert(!wifi_console_connected());
    // While no server listens, the newest 16 KB are kept.
    char line[1001];
    memset(line, 'a', 999);
    line[999] = '\n';
    line[1000] = 0;
    for (int i = 0; i < 20; i++) { line[0] = (char)('A' + i); print(line); }
    received();
    server_connects(fake_ip(192, 168, 1, 11), 5000);
    fake_server_acks(); // the send buffer holds less than 16 KB
    const char *got = received();
    assert(strlen(got) >= OUT_SIZE);
    assert(strchr(got, 'T') && !strchr(got, 'A') && !strchr(got, 'D')); // oldest dropped
    printf("ok: silent server dropped after 6 s; the newest 16 KB kept\n");
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
    printf("ok: server closing, refusing, not answering: looks again\n");
}

static void test_recordings_by_udp(void) {
    fake_server_closes();
    run(10);
    assert(!wifi_console_data_send("x", 1) && wifi_console_data_connection() == 0); // no server
    // The announcement names the recording port.
    fake_announce(fake_ip(192, 168, 1, 10), "ROBOCAR-SERVER 4211 5000\n");
    run(10);
    fake_server_accepts();
    run(10);
    uint32_t first = wifi_console_data_connection();
    assert(first > 0);
    received();
    assert(wifi_console_data_send("take 1", 6) && fake_dgram_len == 6 && memcmp(fake_dgram, "take 1", 6) == 0);
    assert(fake_dgram_ip.addr == fake_ip(192, 168, 1, 10).addr && fake_dgram_port == 5000);
    assert(fake_pbufs_alloced > 0 && fake_pbufs_freed >= fake_pbufs_alloced);
    assert(strcmp(received(), "") == 0); // nothing on the console
    // Refused (the WiFi chip full, out of memory): false, nothing waits.
    fake_udp_err = ERR_MEM;
    assert(!wifi_console_data_send("x", 1));
    fake_udp_err = ERR_OK;
    fake_pbuf_fail = true;
    assert(!wifi_console_data_send("x", 1));
    fake_pbuf_fail = false;
    // The console lost and found again: a new number.
    fake_server_closes();
    run(10);
    assert(wifi_console_data_connection() == 0);
    fake_announce(fake_ip(192, 168, 1, 10), "ROBOCAR-SERVER 4211\n"); // no recording port: 4212
    run(10);
    fake_server_accepts();
    run(10);
    assert(wifi_console_data_connection() > first);
    assert(wifi_console_data_send("y", 1) && fake_dgram_port == 4212);
    // The link's status: the signal, the console's TCP.
    fake_tcp.snd_lbb = 5000;
    fake_tcp.lastack = 1000;
    fake_tcp.nrtx = 3;
    fake_tcp.rto = 6;
    wifi_link_status_t ls;
    wifi_console_link_status(&ls);
    assert(fake_lock_depth == 0 && ls.rssi_dbm == -55 && ls.console.unacked == 4000);
    assert(ls.console.retries == 3 && ls.console.rto_ms == 3000 && ls.console_silent_ms <= 500);
    fake_tcp.snd_lbb = fake_tcp.lastack = 0;
    fake_tcp.nrtx = 0;
    received();
    printf("ok: recordings as UDP datagrams to the announced port (4212 if none), refusals reported\n");
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
    assert(wifi_console_connected());
    assert(wifi_console_data_send("z", 1) && fake_dgram_port == 4212); // no announcement: the default port
    printf("ok: WiFi lost and rejoined; WIFI_SERVER_IP after 5 s without announcements\n");
}

int main(void) {
    test_join_fails_then_retries();
    test_finds_server_and_talks();
    test_full_send_buffer();
    test_server_silent();
    test_server_closes_or_fails();
    test_recordings_by_udp();
    test_fallback_ip();
    printf("wifi_console: all tests passed\n");
    return 0;
}
