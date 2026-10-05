#pragma once
// PicoA's console over WiFi (ROBOT_WIFI.md): once started, everything printed also
// goes to the robot server on the PC, and keys typed there arrive through getchar,
// next to USB. Joins the network in wifi_config.h, finds the server by its UDP
// announcements and keeps reconnecting by itself; prints each step ("WiFi: ...",
// "Server: ..."). Output while no server listens is kept (the last 16 KB) and sent
// when one does. Losing the connection doesn't stop the robot.
#include <stdbool.h>

void wifi_console_start(void);        // starts connecting; if already started, prints the state
bool wifi_console_connected(void);    // a server is listening
bool wifi_console_settled(void);      // the first attempt is over: connected, failed, or joined without finding a server for 5 s
void wifi_console_print_status(void); // one line for the status key
void wifi_console_update(void);       // call every loop iteration
