# RoboCar — WiFi console

Status: **working on the robot.** PicoA (a Pico 2 W) sends its debug
console over WiFi to a Node server on the PC, and the server shows it in a browser
page with the same keys as the serial monitor. The USB serial monitor stays as it
is. This plan is separate from [ROBOT_PLAN.md](ROBOT_PLAN.md). Like the other docs it
shows only the current state; history is in [CHANGELOG.md](CHANGELOG.md).

## Goal

No USB cable while the robot drives around the apartment: everything PicoA prints
goes to the PC over WiFi, and keys typed on the PC reach the robot. For now this is
the serial monitor over WiFi (text and single keys); robot behaviour and a drawn
map come later.

## Start-up (PicoA)

| At power-up | What PicoA does |
|---|---|
| **USB connected** (a computer enumerated the Pico; a charger doesn't count), checked ~3 s after boot | Nothing: no scan, no WiFi. Waits for the serial monitor and keys. `n` starts the scan, `w` connects to WiFi |
| **No USB** | Connects to WiFi, then does the 390° start-up scan whether that worked or not |

"WiFi worked or not" means the first of: connected to the server; WiFi failed
(wrong password, network not found, no answer after 15 s); or WiFi joined but no
server found within 5 s. Waiting for the server first means the scan's map reaches
the browser. The WiFi attempt also gives the gyro its still second.

The scan no longer starts when a serial monitor opens.

## Connection

1. **WiFi:** joins the network in `picoA/app/wifi_config.h` (git-ignored; copy
   `wifi_config.example.h`). No power saving on the WiFi chip (keys answer at once,
   broadcasts aren't missed).
2. **Finding the server:** the server broadcasts `ROBOCAR-SERVER 4211` on UDP port
   **4210** every second to every network it is on. The robot connects to the
   sender's IP on the port in the message. No IP address in the firmware. Fallback:
   `WIFI_SERVER_IP` in `wifi_config.h`, tried after 5 s without an announcement.
3. **The console:** one TCP connection. Robot → server: the console text, exactly
   what USB shows. Server → robot: keys.
4. **Heartbeats:** the robot sends `\x01rssi -55\n` every 2 s (signal strength;
   the server strips it from the text, the console never prints `\x01`); the
   server sends a `\0` byte every 2 s (the robot ignores it). Either side drops the
   connection after 6 s without hearing anything.
5. **Losing it:** the robot keeps running (as when the USB is unplugged; PicoB's
   safety stops still apply). Server lost → it looks again; WiFi lost or failed →
   it rejoins (after 30 s when the join failed).
6. **Output while not connected:** kept in a 16 KB buffer (oldest dropped first)
   and sent when the server connects, so the browser shows how the WiFi came up.

Statuses, printed on USB and (once connected) over WiFi; `p` adds a WiFi line:

```
WiFi: connecting to "MyNetwork"...
WiFi: connected, IP 192.168.1.42, signal -55 dBm
WiFi: failed: wrong password | network not found | no answer after 15 s (trying again in 30 s)
Server: looking for it...
Server: found at 192.168.1.10:4211, connecting...
Server: connected
Server: lost (no answer for 6 s), looking again...
WiFi: lost, reconnecting...
```

## PC (`pc/robot/`)

Node 24, TypeScript run directly (`npm start` = `node src/main.ts`), like
`pc/bringup/`.

- `src/robot.ts`: the announcements, the robot's TCP connection, heartbeats,
  stripping the status records. One robot at a time; a new connection replaces the
  old one.
- `src/main.ts`: HTTP + WebSocket for the page, log file per server run in
  `pc/robot/logs/` (git-ignored), the last 256 KB of text kept so a reloaded page
  shows the history.
- `public/`: the page at **http://127.0.0.1:8080/**: a top bar ("Waiting for
  robot…" / "Robot connected: IP, signal, since 14:32" / "Robot lost at 14:40"),
  a button per key, and the scrolling log. Typing a key on the page sends it too.

Ports: HTTP 8080 (`PORT` to change), robot TCP 4211, announcements UDP 4210.

**Can block it:** routers that don't pass broadcasts between devices (guest
networks, "client isolation"): use `WIFI_SERVER_IP`. The first time, macOS asks
whether `node` may accept incoming connections: Allow.

## Firmware

- `picoA/app/wifi_console.*`: the WiFi chip, lwIP, the announcements, the TCP
  connection, heartbeats; plugs into the Pico SDK's stdio as a second output and
  input next to USB, so `printf` and `getchar` in `debug_console.c` work unchanged.
  lwIP runs in the background (`pico_cyw43_arch_lwip_threadsafe_background`).
  Host test: `picoA/app/test/test_wifi_console.c`.
- `PICO_BOARD` is `pico2_w` for every firmware (one SDK board per build); PicoB's
  firmware runs the same on its plain Pico 2.

## Test (repeat after changes to the WiFi console or the server)

1. Robot on USB, serial monitor open. Nothing happens at start-up.
2. `npm start` in `pc/robot/`; open http://127.0.0.1:8080/.
3. Press `w` in the serial monitor; watch the statuses.
4. Once connected: the page's buttons (`p`, `m`, …); answers show in both.
5. Unplug the USB (battery on): the page keeps working.
6. Power up without USB: it connects, then scans; the browser shows both.

## Later

- Pose and map as binary messages (same framing as the inter-Pico link) and a drawn
  map on the page.
- New behaviour and commands as the robot grows (ROBOT_PLAN.md).
