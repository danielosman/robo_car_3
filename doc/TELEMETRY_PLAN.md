# Telemetry: raw data to the PC, stored, viewed, replayed

Why (Daniel, 10 Oct 2026): the map's problems (the cup and the box at ~45 cm, MAP_DESIGN
§8) were argued from a simulation that models the VL53 as "the nearest surface", and
the fixes built on it were weak. We need the real sensor's data on the PC: look at
it in 3D, query it, form hypotheses (e.g. "a 5th-row zone with a cup in it returns
more signal than plain floor") and test map variants on the same recordings.
Status: **T1 built and tested; T2 built, host-tested, robot test pending.** Replaces ARCHITECTURE_PLAN §2.6's recording part
(R0) and REWORK_PLAN's open point 19. Keys: [COMMANDS_PLAN.md](COMMANDS_PLAN.md).

## 0. Start here (a new session)

**Where things stand (10 Oct 2026):** T1 (the commands, [COMMANDS_PLAN.md](COMMANDS_PLAN.md))
is built and passed its robot test: the robot is idle with the motors off until a
key, one key one action (space stop, `s` scan 390°, `f` move 50 cm, `t` turn 30°,
`r` direction, `a` watch 1 min, `C` clear the map), the same on USB and WiFi;
movement lines print only while watching. **T2 (recording on the robot) is built**
and worked on the robot: `R`, `5`, takes, the records of §2.1, and on the PC a
listener that saves what arrives and a decoder (`npm run decode`). Three robot
runs over TCP showed 1-6 s stalls while the robot turns (§2 "Measured"), so
recordings now go as **UDP datagrams in a compact format (version 3)**, tested on
the robot 10 Oct: no stalls, 2-3 % of datagrams lost, mostly one at a time (§2
"Measured"). The recordings (three TCP runs and one UDP run: 5 s, a scan, a
minute of watching each) are in `pc/robot/recordings/raw/` and all decode.
**T3 (storage) is built** (§3): every take goes into DuckDB as it ends, `npm run
import` rebuilds it from the raw files, the take list is at
http://127.0.0.1:8080/takes. **T4 (the viewer) is built** (§4): http://127.0.0.1:8080/viewer
shows a take in 3D with both replayed maps and truth boxes with a score. The
robot runs the READINGS map since 10 Oct (MAP_DESIGN §9). **T5 (replay) is built**
with it, and the five scans are a regression test. **T6 is dropped**: the cup
session was done by the analysis of 10 Oct. **This plan is done; the next work is
M4** (REWORK_PLAN "Next"): telemetry stays the tool: record (`R`), look (/viewer),
replay (`tools/replay.sh`).

**Read first:** this file; [REWORK_PLAN.md](REWORK_PLAN.md) (decisions, how to work a
step); [COMMANDS_PLAN.md](COMMANDS_PLAN.md) (the keys, as built). Background for T6:
[MAP_DESIGN.md](MAP_DESIGN.md) §8.1 and the 10 Oct entries in
[REWORK_CHANGELOG.md](REWORK_CHANGELOG.md) (the cup analysis).

**Code T2 built on** (T3 adds to `pc/robot/src/recording.ts`; the record layouts
are in `picoA/app/recorder.c` and §2.1):
- `picoA/drivers/tof.c` / `tof.h`: `tof_poll()` gives the ULD's `VL53L8CX_ResultsData`
  with every output enabled (`picoA/drivers/lib/vl53l8cx/platform.h`), 4 targets
  per zone. `picoA/app/rangefinder.c` `rangefinder_poll()` keeps one distance per
  zone (`closest_sure_mm()`) and maps zones to rows/columns (the sensor is turned
  90°: `zone = (7 - col) * 8 + row`); `TOF_RAW` takes the raw data from there.
- `picoA/app/pose.c`: the pose history (`pose_at(t_us)`); `body.c`: the `ODOM`
  reports (`body_odom()`, `body_odom_time_us()`) and `body_drive()` for `DRIVE`.
- `picoA/app/behaviour.c`: where an action starts (`begin()`), runs and ends
  (`end_early()`, `say_done()`): the hooks for `TAKE_START` / `TAKE_END`.
  `picoA/app/debug_console.c`: the keys (`R` and `5` go here).
- `picoA/app/wifi_console.c`: the console's TCP connection to the server (port
  4211, found by UDP announcements, a 16 KB ring kept while disconnected, lwIP
  raw API); recordings go as UDP datagrams to port 4212 (`wifi_console_data_send()`).
- `pc/robot/` (Node 24, TypeScript run directly, `ws`): `src/robot.ts` (the
  robot's TCP connection, `ROBOT_PORT` 4211), `src/main.ts` (HTTP 8080, WebSocket to
  the page, the log in `logs/`), `public/console.js` (the page's keys). `npm test`.
- Host tests: `./run_tests.sh` (C, stops at the first failure); firmware:
  `cmake --build build` → `build/picoA/picoA_app.uf2`.

**Rules that apply (REWORK_PLAN "How to work a step"):** host tests first; a robot
test only when the robot changes (hand Daniel the steps README-style and wait for
the log); update README / the plans / REWORK_CHANGELOG / REDFLAGS; ask before
behaviour that wasn't agreed; commit only when Daniel asks. **Commits carry no AI
co-author trailer** (no `Co-Authored-By:` line), whatever a tool's default is
(Daniel, 10 Oct). `pc/robot/recordings/` is git-ignored (T2).

## 1. Decisions (Daniel)

- **Raw sensor data and the pose, not the map**, go to the PC. The map is computed
  from them (on the robot as now, and on the PC by replay), so any cell size, layer
  or vote rule can be tried on the same data.
- **Nothing is sent when nothing happens.** `R` turns recording on, but data flows
  only during an action (from its start to its end) or during `5` (record 5 s).
- **Each action is one take**, labelled with the key and action that caused it.
- **DuckDB** for queries on the PC.
- No camera images for now (REWORK_PLAN decision); a record type is kept for later.
- **Limit what is sent** (10 Oct): what the PC can compute isn't sent. `GEOMETRY`
  sends the mounting and the zone angle, not each ray's direction; `TOF_RAW` sends
  only the targets a zone found.
- **The 32-bit PicoA clock** (`time_us_32()`, wraps every 71.6 min) in every
  record; no 64-bit time: runs are shorter (10 Oct). The PC unwraps differences.
- WiFi throughput isn't tested on its own: the T2 robot test measures it (10 Oct).

## 2. Recording on the robot

- `R` switches recording on / off and prints it. Off at power-up. Off during a take
  ends it ("recording off").
- Recording on + an action starts → a **take** starts: `TAKE_START`, then the
  records below while the action runs, then `TAKE_END` when it ends. Recording on,
  idle: nothing is sent. The take's key is the last key pressed.
- `5` (record 5 s) is an action only for recording: the robot stands still with the
  motors off; it needs recording on and the connection up, else it only says why.
  It doesn't need PicoB. Another action replaces it like any other.
- Recording on but no server connected (the console): the action runs anyway, the
  robot prints "Not recorded: no connection to the server". A console connection
  lost during a take cuts it (the robot prints "take N cut", the PC marks it CUT:
  no `TAKE_END`); one that comes up during a take doesn't record the rest of it.
- **Transport: UDP datagrams** (since 10 Oct, record version 3) to the server's
  port 4212, beside the console's TCP connection (4211). The server announces both
  ports (`ROBOCAR-SERVER 4211 4212`); without an announcement (`WIFI_SERVER_IP`) it
  is 4212. Why not TCP (versions 1-2, a second TCP connection): when the signal
  dips while the robot turns (−56 → −68 dBm over about half a turn: the batteries
  under the antenna), a few packets are lost, and TCP holds everything behind them
  until they get through, with lwIP waiting 1.5 s, then 3 s between retries: 1-6 s
  stalls, even ODOM lost once its buffer was full. The console kept working all
  along (it sends little). With UDP a lost datagram is lost and nothing waits:
  no retransmission, no protocol of our own; the PC counts what is missing.
- **Datagrams:** at most 1400 B (one WiFi frame), a header (version, boot id,
  number), then whole records. Small records wait at most 50 ms for a datagram
  to fill. Each ToF frame goes as two records of 32 zones, each starting a
  datagram, so a lost datagram costs half a frame. Each datagram repeats the
  previous one's `ODOM` records (the PC keeps one of each, by PicoB's time), so one
  lost datagram loses no odometry. The datagram with `TAKE_START` and `GEOMETRY` is
  sent twice, the one with `TAKE_END` three times, under one number (the PC keeps
  one).
- **What was lost, said by the data:** datagram numbers count every datagram sent
  (a gap is a lost one); every `TOF_RAW` carries its frame number and how many
  frames weren't sent before it (refused on the robot: lwIP or the WiFi chip out
  of buffers); `TAKE_END` the robot's totals. The import puts them in `takes` and
  `tof_frames`, and the viewer shows the gaps. No buffer on the robot: a datagram
  goes at once or is refused.
- **Every odometry report:** `body` keeps the last 16 reports, numbered, so a loop
  iteration longer than 20 ms loses none (the first robot test lost 5 %: only
  the latest report per iteration was taken). `pose` takes them the same way.
- **The pose before the first frame:** `TAKE_START` and `GEOMETRY` are followed by
  the latest `ODOM`, so the first ToF frames can be placed (the robot is still
  then).
- **Marks:** the console's lines during a take are taken from stdout (a third
  stdio output beside USB and WiFi), PicoB's `B: ...` lines included; keys pressed
  during a take as "key p" ("key space").
- **Framing:** `[type u8][len u16][payload][CRC-32 u32]`, little-endian; the CRC
  (IEEE, as zlib's) covers type, length and payload (UDP's own checksum catches
  the WiFi's errors; ours catches our own bugs).

### Record types

| Type | When | Payload |
|---|---|---|
| `TAKE_START` | action start | boot id (random at power-up), take number (per boot), key, action name, parameters (distance or angle, direction), firmware build, PicoA time |
| `GEOMETRY` | after `TAKE_START` | sensor position (x, y, z), zone angle, rows/cols, the zone order used: so the PC never hard-codes the mounting |
| `TOF_RAW` | every ToF frame (15 Hz), in two halves | frame number (counts every frame the sensor gave, sent or not), frames dropped since the last one sent, frame time (PicoA clock, middle of the measurement); per zone: number of targets, ambient per SPAD, SPADs enabled; per target (up to 4): distance mm, sigma mm, signal per SPAD, reflectance %, status. All of it: what the robot uses today is one distance per zone |
| `ODOM` | every PicoB report (50 Hz) | PicoA time of the measurement, x, y, yaw, pitch, roll, v, w, still/moving, motors, safety stop, gyro bias |
| `DRIVE` | each command sent to PicoB | v, w |
| `MARK` | console lines printed during the take, keys pressed | text |
| `TAKE_END` | action end | how it ended (done / stop / safety stop / PicoB lost / replaced), records sent, records, ToF frames and datagrams refused |
| `WIFI` | every 0.5 s | signal, the console's TCP state, datagrams sent and refused |

**Measured over TCP (three robot runs, 10 Oct):** 15.1 ToF frames/s, ~2 KB a
frame in version 2 (most zones report 2+ targets: 2.2 on average), 30-36 KB/s; a
minute of watching 1.9 MB; no CRC errors. Robot still (`5`): never a stall.
Turning: stalls of 0.9 s (run 1), 6 s in three pieces and 3 s (run 2), 4 s and 3
s (run 3); ODOM lost too once the buffer filled. The `WIFI` records (run 3)
showed why (above): the signal dips by ~10 dB over half a turn, TCP's window
collapses and it waits for retries; no packet was ever refused on the robot, and
the console's data kept arriving during every stall.

**Measured over UDP (version 3, 10 Oct, `2026-10-10-18-08-07-boot-0558a0ee.rec`):**
no stalls. Still: 0 lost. Scan: 214 whole frames, 7 half, 5 missing of 226; 3.0 %
of datagrams lost. Watch (60 s): 877 whole, 28 half, 7 missing of 912; 2.3 %
lost, 9 refused on the robot. Losses mostly one datagram at a time (31 of 39
runs; at most 6 in a row); ODOM gaps 9 in all, 80-221 ms (the repeats cover single
losses). 24-28 KB/s. Losses depend on the heading again (this run 0-135°: 4-9 %,
facing 315-360°: none). Robot and PC sizes differ: the robot counts datagram
headers and repeats, the PC only the records it keeps.

**Version 3 sizes:** the sensor's values fit 16 bits (measured over 3348 frames:
ambient 0-10, SPADs 2816-4096, signal 1-2264 kcps/SPAD; capped at 65535 if ever
more), so a frame is 13 × 2 + 64 × 5 + 8 B per target: ~1.45 KB at 2.2 targets a
zone (0.35 KB with none, 2.4 KB with four everywhere) instead of ~2 KB; with
`ODOM` (60 B at 50 Hz, each sent twice) about 28 KB/s.

### 2.1 Record format (version 3: UDP, compact ToF halves; 10 Oct)

Packed, little-endian; times are PicoA's `time_us_32()`. The layouts are
`rec_*_t` in `picoA/app/recorder.c`; `pc/robot/src/recording.ts` `parseRecord()`
reads them, versions 1-2 too (a host test decodes the firmware's own datagrams).

**Datagram** (≤ 1400 B): version u8 (3), boot id u32, number u32 (counts every
datagram since power-up; a repeat keeps its number), then whole records.

Before version 3 (TCP, one stream per connection): version 1 had no `WIFI`;
version 2's `WIFI` held a second TCP connection's state; both had `TOF_RAW` with
all 64 zones and u32 ambient, SPADs and signal, and a 25-byte `TAKE_END`
without the datagram counts.

| Type | Payload (bytes) |
|---|---|
| 1 `TAKE_START` (52) | version u8 (3), boot id u32, take number u16 (from 1 per boot), t u32, key char, action char[12] (`scan`, `move`, `turn`, `watch`, `record`; NUL-padded), param f32 (scan, turn: rad, + left; move: m, + forward; watch, record: s), build char[24] (firmware build time) |
| 2 `GEOMETRY` (82) | sensor position f32[3] (m: forward, left of the centre, up from the floor), zone angle f32 (rad), rows u8, cols u8, zone of each ray u8[64] (ray = (row − 1) × cols + (col − 1) for rows and columns 1-8, row 1 top, column 1 left; the ray points (row − 4.5) zone angles below horizontal and (4.5 − col) to the left, robot level) |
| 3 `TOF_RAW` (13 + 32 × 5 + 8 per target) | half a frame: frame number u32, t u32 (the middle of the measurement), frames not sent before it u16, sensor temperature i8 (°C), first zone u8 (0 or 32), zones u8 (32); then for each zone in the **sensor's** order: targets u8 (≤ 4), ambient u16 (kcps/SPAD), SPADs enabled u16; then its targets, closest first: distance i16 (mm), sigma u16 (mm), signal u16 (kcps/SPAD), reflectance u8 (%), status u8 (5, 6, 9 valid). Ambient, SPADs and signal are the ULD's u32s capped at 65535 |
| 4 `ODOM` (53) | t u32 (when PicoB measured it, PicoA's clock), PicoB's `odom_report_t` as sent (45: PicoB's t u32, x, y, yaw, v, w, pitch, roll, wheel left, wheel right f32; stationary, motors on, stop reason, motors request, IMU error u8), gyro bias f32 (rad/s) |
| 5 `DRIVE` (12) | t u32 (sent), v f32 (m/s), w f32 (rad/s); one per command sent to PicoB (every 50 ms) |
| 6 `MARK` (4 + text) | t u32, text (UTF-8, no terminator, ≤ 160) |
| 8 `WIFI` (46) | every 0.5 s during a take, and in its first datagram: t u32, RSSI i8 (dBm), ms since the server was last heard on the console u32; the console's TCP (29): bytes waiting in our buffer u32, unacknowledged in TCP u32, TCP send queue u16 (pbufs), retransmissions of the oldest segment u8, retransmission timeout u16 (ms), congestion window u32, the server's window u32, `tcp_write` / `tcp_output` refusals since power-up u32 each; datagrams sent and refused since power-up u32 each |
| 7 `TAKE_END` (33) | t u32, reason u8 (0 done, 1 stopped, 2 replaced, 3 safety stop, 4 PicoB lost, 5 didn't start, 6 recording off), records sent before it u32, records in refused datagrams u32, ToF frames sent u32, ToF frames with a half refused u32, bytes sent before it u32, datagrams sent (repeats included) u32, datagrams refused u32 |

The pose frame is odometry's from power-up (the scan no longer moves the origin),
so all takes of one boot share one frame; takes of different boots don't.

## 3. Storage on the PC (T3, built 10 Oct)

```
pc/robot/recordings/                            (git-ignored)
  raw/2026-10-10-18-08-07-boot-0558a0ee.rec     one robot boot: every datagram as it arrived
  raw/2026-10-10-18-08-07-boot-0558a0ee.arrivals.csv   when the console's text arrived
  raw/2026-10-10-17-21-17.795.rec               before UDP: one TCP connection's stream
  robot.duckdb                                  derived from raw/: rebuilt at any time
```

- The **raw files are the truth**: a file per robot boot (`RCDGRAM3`, then
  [length u16][arrival ms f64][datagram] each, repeats too), written as the
  datagrams arrive and never changed. **Changed from the first plan** (a `.rec`
  per take in day / boot folders): the per-boot files already hold every take
  untouched, and copies per take would be a second truth to keep in step; the
  `takes` table says which file holds each take.
- The server notes each take as it ends (how it ended, frames whole / half /
  missing, datagrams lost, KB/s) and imports it into DuckDB (`store.ts`): "take N
  stored: … frames, … targets". A take whose `TAKE_END` never came is imported
  as "cut". `npm run import` (in `pc/robot/`) imports every take of every raw file
  again (`-- <file>` for some): a take's rows are replaced, its note and the truth
  objects stay. All twelve takes of 10 Oct: 1.4 s.
- `npm run decode -- <file> [--records]` prints a raw file's takes or records.
- The database is opened for each operation and closed after it, so the `duckdb`
  command line (`brew install duckdb`; `duckdb pc/robot/recordings/robot.duckdb`)
  can use it between them; while the command line holds it, the server can't
  store a take ("NOT stored"): `npm run import` then.
- The page **http://127.0.0.1:8080/takes** (link "Takes" on the console) lists
  the takes, newest first: started, take, key, action, seconds, how it ended,
  frames whole / half / missing, datagrams lost, odometry reports, and a note to
  click and change ("cup at 45 cm").
- **Tables** (`store.ts` has the schema with comments). Times: `t_us` PicoA's
  clock, `t_s` seconds since the take started. `row` and `col` are 1-8 (README);
  `zone` is the sensor's own number 0-63.

| Table | One row per | Columns |
|---|---|---|
| `takes` | take | take_id (boot id - take number), boot_id, take_no, started_at (local; before UDP: when the file was opened), day, key, action, param, version, build, duration_s, end_reason ('cut' without `TAKE_END`), frames_whole / half / missing, datagrams, datagrams_lost, odom_reports, file, note |
| `tof_frames` | ToF frame | take_id, frame_no, t_us, t_s, skipped, temp_c, zones (64, or 32 when half was lost), x, y, yaw, pitch of the robot then (NULL if no odometry within 0.1 s) |
| `tof_zones` | zone × frame | take_id, frame_no, zone (the sensor's), row, col (as the robot sees it), targets, ambient, spads |
| `tof_targets` | target × zone × frame | take_id, frame_no, zone, row, col, target (0 = closest), distance_mm, sigma_mm, signal, reflectance, status, x, y, z in the world (m, odometry's frame from power-up; NULL without a pose) |
| `odom` | report | take_id, t_us, t_s, picob_us, x, y, yaw, pitch, roll, v, w, wheel_left, wheel_right, stationary, motors_on, stop_reason, imu_error, gyro_bias |
| `drive`, `marks`, `wifi` | record | take_id, t_us, t_s; v, w / text / rssi_dbm, console_silent_ms, console_unacked, console_retries, datagrams, datagrams_refused |
| `truth` | object placed by hand (T4) | boot_id, take_id, name, x, y, z, size_x, size_y, size_z, note |

- **World positions** as the robot's code places them (`rangefinder.c`,
  `cell_map.c`): the ray at row, col points (row − 3.5) zone angles below
  horizontal, less the robot's pitch, and (3.5 − col) to the left; from the
  sensor's position, along the ray by the distance; turned by yaw and moved by
  x, y (roll ignored, as on the robot). The pose is interpolated between the two
  odometry reports around the frame's time.
- **First look (10 Oct, the four scans; rows 1-8):**
  - **Floor:** rows 7-8 land at z −0.5…−0.3 cm, the floor (the geometry is right);
    row 6 sees the floor at ~42 cm (+0.8 cm), and the box.
  - **Row 5 gives no valid return from bare floor closer than 1 m.** In the UDP
    scan every one came from an object: the box (30-35 cm, left, rows 5-6, signal
    ~430), the cup (40-45 cm, ~30° right, **row 5 only**, 52 returns over ~20
    frames, z ~5 cm), the cupboard behind (45-60 cm, rows 3-5, the strongest
    signal, up to ~1250).
  - **The cup in every scan, by row 5 only:** scans 1-2 at ~48 cm, 19° left (61-63
    frames: a 390° scan passes 0-30° twice; Daniel placed it left in scan 1),
    scans 3-4 at ~45 cm, 30° right (~20 frames). Scan 2's 6 s stall hid 145-321°.
  - **Not found:** a stronger signal from the cup (median 205-231, other row-5
    returns at that distance 125-316); a second target on the cup; the cup in rows
    6-8 (they meet the floor first). The box in scans 1-2 is unclear.
  - So the cue is a valid row-5 return under 1 m at all, seen again and again at
    one world spot; the map losing the cup (MAP_DESIGN §8.1) is the vote's doing,
    for T5/T6. (A first reading, "row 5 sees the floor at +4.7 cm", was wrong:
    those returns were the cupboard.)

The world x, y, z per target means "every 5th-row return within 10 cm of the
cup" is one query. Example questions:
- Signal per SPAD of 5th-row zones by distance: cup vs bare floor.
- How often a 2nd target appears in a zone that sees the cup and the floor.
- The 5th row's reading over bare floor: where it lands, how it scatters.

## 4. The viewer (T4, built 10 Oct)

**http://127.0.0.1:8080/viewer** (`?take=<id>`), served by `pc/robot`
(`viewer_api.ts`); three.js (npm, pinned like every dependency: `npm install`)
comes from the server too, `/vendor/three/`, no internet needed:
- **Take**: a list of all takes; the URL keeps the one shown.
- **Points**: every target of the take where it was in the room (`tof_targets`
  x, y, z), coloured by row, signal, status, height or target number; filters:
  rows 1-8, sure only (status 5, 6, 9), closest target only, frames all / up to
  the slider / the slider's only; point size. Click a point: its row, column,
  target, distance, signal, status, height, frame and time (the slider jumps there).
- **Map**: the take replayed through the robot's own map code (§5) as 10 cm
  cubes: blocked (L1) red, overhang (L2) orange, floor seen / no floor as tiles;
  READINGS (what the robot runs), VOTES (before), or the difference (blue only
  READINGS, purple only VOTES); the floor margin.
- **Time**: a slider through the frames, play at 15 frames/s; the robot (a box)
  and its rays to each zone's closest target.
- **The path**: the odometry, a grey line on the floor.
- **Truth**: boxes where things really stood (name, x, y, size, height; shift-click
  the floor for x, y), kept per boot in the `truth` table (each boot has its own
  odometry frame); drawn as yellow frames.
- **Score**, per variant: each truth box found or not (a blocked or overhang cell
  on it, how many), and the solid cells within 1.5 m of the start on no box ("not
  explained": false, or not placed yet).

Not built (yet): several takes at once, the DuckDB query box.

## 5. Replay: the robot's code on recordings

- A host build of `cell_map.c` (and later `tof_motion.c`, `behaviour` logic) reads
  a take's `.rec` and writes the map as voxels (JSON) for the viewer.
- A **variant** is a build flag or a parameter set; replay runs several on the
  same takes. With `truth`, each gets a score (truth cells found, false blocked
  cells): "is the cup in the map?" becomes a number per variant.
- Recorded takes with truth become regression tests (`run_tests.sh`).
- **Built 10 Oct:** `npm run frames -- <take> <file>` (pc/robot) writes a take's
  frames as the map receives them; `tools/replay.sh <take>` runs `cell_map.c` on
  them in both variants (MAP_DESIGN §9) and lists every column where they
  differ; with `--json` (the viewer, `replay.ts`, which builds the tool with `cc`
  whenever the map code changed) it gives the cells. The replay matches the
  robot's own map (MAP_DESIGN §9). Scores against truth: in the viewer.

## 6. Steps

Each changes something visible; robot tests only where the robot changes.

1. **T1 Commands** (COMMANDS_PLAN): idle start, the key table, scan only 390°,
   move / turn / direction, stop on space, the tests and the 15 s status removed.
   Host tests; robot test: each key once. **Built and tested on the robot 10 Oct.**
2. **T2 Recording on the robot**: records, takes, `R`, `5`, the 4212 connection,
   drop counting; on the PC the listener (raw bytes saved) and the decoder.
   Robot test: a scan and 1 min of watching recorded; bytes, frames, drops.
   **Built 10 Oct; three robot runs over TCP (§2 "Measured"): every ODOM recorded
   after a fix, but 1-6 s stalls while turning; now UDP and a compact format
   (version 3): robot test 10 Oct, no stalls, 2-3 % of datagrams lost. Done.**
3. **T3 Server storage**: DuckDB import, the take list in the page. Check: a
   take's frames, zones, targets and pose in DuckDB. **Built 10 Oct** (§3; the raw
   per-boot files stay the truth instead of per-take copies); all twelve takes
   of 10 Oct imported; the floor lands at z ≈ 0.
4. **T4 Viewer**: the three.js point cloud, pose, time slider, filters, truth
   objects. **Built 10 Oct** (§4).
5. **T5 Replay**: `cell_map` on takes, voxels in the viewer, variants, scores.
   **Built 10 Oct** with T4 (§5): both variants per take; the five scans are a
   regression test (`test_map_replay`, MAP_DESIGN §9), no truth boxes needed: the
   objects' regions come from where Daniel put them.
6. **T6 The cup session**: scans with nothing, the cup at 45 / 80 cm, the 7 cm box;
   truth placed; then the hypotheses (signal, 2nd target, the 5th row's floor
   returns) and map fixes, tested on these takes. **Dropped 10 Oct (Daniel): done
   by the analysis of that day** (§3 "First look", MAP_DESIGN §9): the cup is seen
   by row 5 in every scan, its signal no stronger, no 2nd target; the map's vote
   lost it, and the READINGS map finds it. Truth boxes stay optional in the viewer.

## 7. Open (Daniel)

Decided 10 Oct: watching lasts 1 minute (~3 MB a take); when the WiFi can't keep
up, ToF frames are dropped and the data says so (§2).

1. DuckDB from Node (`@duckdb/node-api`) in the server: imports and the page's
   queries in one place; `npm install` brings DuckDB itself, nothing else to
   install. Analysis also from the command line: the `duckdb` CLI (`brew install
   duckdb`), so Claude can query it directly; only needed from T3 on.
2. The robot's map stays on the robot (needed for driving later); `m`'s text print
   goes once the viewer shows the replayed map?
