#pragma once
// PicoA's console over WiFi (ROBOT_WIFI.md): once started, everything printed also
// goes to the robot server on the PC, and keys typed there arrive through getchar,
// next to USB. Joins the network in wifi_config.h, finds the server by its UDP
// announcements and keeps reconnecting by itself; prints each step ("WiFi: ...",
// "Server: ..."). Output while no server listens is kept (the last 16 KB) and sent
// when one does. Losing the connection doesn't stop the robot.
// Recordings (recorder.c, doc/TELEMETRY_PLAN.md) go to the server as UDP datagrams,
// beside the console: a lost one is lost, and nothing waits behind it.
#include <stdbool.h>
#include <stdint.h>

void wifi_console_start(void);        // starts connecting; if already started, prints the state
bool wifi_console_connected(void);    // a server is listening
bool wifi_console_settled(void);      // the first attempt is over: connected, failed, or joined without finding a server for 5 s
void wifi_console_print_status(void); // one line for the status key
void wifi_console_update(void);       // call every loop iteration
// The console connection's number (counts up with each connection), 0 while there
// is none: recordings go to that server. A different number means it was lost in
// between.
uint32_t wifi_console_data_connection(void);
// One datagram to the server's recording port; false if lwIP or the WiFi chip
// refused it (out of buffers) or there is no server.
bool wifi_console_data_send(const void *buf, uint32_t len);

// How the WiFi and the console connection are doing, for finding where data
// stalls (recorded every 0.5 s during a take). All zero while not connected.
typedef struct {
    uint32_t waiting;        // bytes in our buffer, not yet handed to TCP
    uint32_t unacked;        // handed to TCP, not yet acknowledged by the server
    uint16_t segments;       // TCP's send queue (pbufs)
    uint8_t retries;         // retransmissions of the oldest unacknowledged segment (0 = none pending)
    uint16_t rto_ms;         // TCP's retransmission timeout now (grows with each retry)
    uint32_t cwnd;           // congestion window, bytes
    uint32_t peer_window;    // what the server says it can take, bytes
    uint32_t write_errors, output_errors; // tcp_write() / tcp_output() refused, since power-up
} wifi_conn_status_t;
typedef struct {
    int8_t rssi_dbm;
    uint32_t console_silent_ms; // since the server was last heard on the console
    wifi_conn_status_t console;
} wifi_link_status_t;
void wifi_console_link_status(wifi_link_status_t *s); // asks the WiFi chip for the signal: ~1 ms
