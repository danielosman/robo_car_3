# Data flow, the PC's maps and the code's structure — plan

The robot sends what it measures during moves, packed small; the PC stores it and
builds the maps. The robot keeps only what it needs to drive. Two data pipelines (ToF
and IMU), one name per piece of data, the same on PicoB, PicoA and the PC.

This file holds the current state and what is left. History is in CHANGELOG.md; how
the built parts work is in ARCHITECTURE.md.

Contents: 1 Where we are · 2 How to continue · 3 Decisions · 4 The picture · 5 Names ·
6 Data formats · 7 The connection · 8 Modules · 9 Budgets · 10 Old data · 11 Steps left ·
12 Debugging · 13 Known issues · 14 Open questions.

## 1. Where we are

- **Built:** steps 1-6, including the PC's whole-house cell map and PicoA's
  window-only, floor-first map. Step 6 is not flashed. Its implementation and two
  fresh reviews are complete; history is in
  [CHANGELOG.md](CHANGELOG.md), the 10 Nov entry.
- **Current versions:** PicoA–PicoB link 7, robot–PC wire 5, snapshot 3. PicoB reads
  the IMU at 416 Hz; the PC receives and stores averaged records at 104 Hz.
  Odometry, `drive` and PicoA's layout stay as they are; the deferred 4e/4g redesigns
  are not part of the remaining work. Their design ideas have a separate home in
  [ARCHITECTURE_REFACTOR_PLAN.md](ARCHITECTURE_REFACTOR_PLAN.md), starting with
  PicoB's odometry acting as the IMU data distributor.
- **Now: step 7**, the line map, in substeps a-e. Its design and its state are in
  [LINE_MAP_PLAN.md](LINE_MAP_PLAN.md), "Where we are" (confirmed by Daniel, 11 Nov).
  Then 8 map switch, 9 docs, the final flash, the robot test.
- **The robot now has** the firmware of commit `6583e88` (both Picos, flashed 10 Nov).
  PicoA changes since then (step 5's `room.h`, step 6's map, snapshot v3 and wire v5)
  are tested on the robot in step 9 (or flashed earlier if Daniel wants).
- **Validation baseline:** the Step 6 code passed `./run_tests.sh`, `tools/lint.sh`,
  `cmake --build build`, and the PC tests and typecheck. Re-run these after code
  changes; this is not a robot-test result. `npm test` and the server build the
  cell map helper themselves (`cc`, into `build/host/cell_map_helper`).
- **Git baseline:** on `main`; Step 6 code, diagram and handoff docs belong to the
  Step 6 implementation commit, after planning commit `17b3983` and Step 5's
  `553d0dc`. Start with `git status` and `git log`; preserve any local work.
  Review Step 7 against this implementation baseline, not the old planning commit.
  Commit and push only when Daniel says. No `Co-Authored-By` lines.
- **Docs behind the code:** ARCHITECTURE, README and TODO don't show 4d-6 (the
  README's robot tests still speak of `##` maps, link version 6, wire version 3;
  nothing on the PC's cell map, `room.h`, or floor first). Where they disagree with this plan, this
  plan and the code are right. They are brought up to date once, in step 9. The
  CHANGELOG is up to date (history goes there, not here).
- **Architecture diagram:** current through Step 6, independently reviewed:
  37 modules (one planned), 13 flows, 437 checked references. It shows window/tile
  ownership, floor rules, map-free snapshots, and the bulk-window/control-answer
  pose join. `npm run resolve` and `npm run typecheck` pass in
  `architecture-diagram/`. Its separate UI follow-up is in its README, "Next".

## 2. How to continue

- **Read first:** this plan, AGENTS.md (how to answer Daniel; the code review guide),
  the README sections "How we work", "Conventions", "Git conventions".
- **The architecture diagram** is how Daniel and an agent look at the design: `npm
  run dev` in `architecture-diagram/` (http://127.0.0.1:5173/, `#flow=<id>` for one
  flow). An agent reads only `architecture-diagram/data/` (its README first). After
  a code change, `tools/lint.sh` lists stale references.
- **Reviews:** build in the main session. At each substep boundary, use a fresh
  reviewer subagent; use one more fresh reviewer at the end of the whole step.
  Each reads `~/code/home-page/sources/skills/strategic-code-review.md` itself and
  only reports (AGENTS.md). The main session applies agreed fixes; any findings
  left open go in this plan with the reason. Review history goes in CHANGELOG.md.
  Daniel decides whether to commit.
- **Simple first:** a rare case returns an error, is counted or is ignored; it isn't
  handled unless a robot test shows it matters. Tests cover the promised behaviour,
  not every corner.
- **Each step ends** green (the checks above), with this plan's section 1 updated,
  its history in the CHANGELOG (not in this plan: AGENTS.md, "Docs"), and says which
  Picos it changes. The other docs wait for step 9: until then this overrides the
  README's "After a step" and robot-test rules.
- **Robot tests are run by the agent**, not by Daniel by hand: Daniel flashes the
  Picos (`build/picoA/picoA_app.uf2`, `build/picoB/picoB_app.uf2`), runs `npm start`
  in `pc/robot` (or asks the agent to) and switches the robot on. The agent then uses
  only `./robot` in `pc/robot`:
  - `robot status` (connected, boot, pose), `robot report` (the robot's counters),
    `robot log N` (everything the robot printed, PicoB's as "B: ...", the server's
    notes);
  - moves: `robot steer r30 d20 ...` (a snap after each), `robot check` first;
    small moves, `robot stop` at once if anything looks wrong;
  - reads: `robot tof [n]`, `robot map [n]` (the PC's map at snapshot n, raw bytes:
    read them with a small script; `--robot` asks the connected robot for its own
    window right now), `robot map --png` (the whole building; with `--robot`, the
    robot's window) and `robot camera [n]` (PNG files: open and look at them),
    `robot frames <boot> <move>`, `robot imu <boot> <move>`;
  - to look round, turn in 30° steps (12 × `r30`), not one 390° turn: a camera picture
    every 30°, to plan and to check.
- **Step 7:** read [LINE_MAP_PLAN.md](LINE_MAP_PLAN.md) in full; continue at the
  first substep not marked done. Daniel asked for a commit (no push) after each
  substep, once the tests pass and the reviewer agrees.
- **Otherwise commit only when Daniel says** (README).
- Long test runs: cap them with a time limit, and never read large generated data
  into the agent's context.

## 3. Decisions (Daniel)

**Hardware**
- The robot is **13 cm** high (measured 8 Nov); L1 (3-13 cm) is what blocks it.
- The PCB is 2 cm higher: the ToF at **9 cm**, the camera at **10.8 cm**.
- **Camera tilt taken as 0°** (the camera was knocked; calibrated later). Large
  placement errors are accepted for now.
- **The mounting** (where the ToF, the camera and the IMU sit) is one file,
  `common/mounting.json`. The C code takes it as a parameter. It is never sent; the
  PC keeps a copy per boot.

**The ToF**
- **10 Hz**; the rate is a setting.
- **One target per zone**: the first with status 5, 6 or 9. No second targets anywhere.
- **Formats by use:** the robot reads a full frame; a format packs it for a use (map,
  watch, snapshot). One format today, **compact** (8-bit distance). Watch may get
  its own later; it is one new entry in the table.

**The IMU**
- **Raw IMU at 104 Hz is sent and stored** during moves, as training data for a
  later model (inputs: IMU windows; answer: the pose change from the 10 Hz poses).
  No wheel counts. PicoB's 416 Hz is not sent.
- **No peaks** (jolt, tilt): nothing used them. PicoB's tilt stop reads pitch and
  roll and stays. Bump reactions later via the IMU's interrupts, not in this plan.
- The PC gets processed data only when it is smaller than the raw data and the robot
  needs it anyway: the pose in each frame.

**Sending**
- **Only during moves:** drive, turn, go to, watch. Nothing while idle.
- **A move stands still first and last:** `pre_ms` and `post_ms`, 200 ms each
  (`robot config pre_ms N`). `post_ms` is the move's only settle time.
- **One TCP connection**, framed messages both ways. The UDP announcement (how the
  robot finds the server) stays. A WiFi stall freezes the page, then it catches up.
- **No raw files:** DuckDB is the one store; a new data format gets a new file.

**Maps**
- **The robot keeps only its 6 × 6 m cell map**, to stop before obstacles and before
  floor it hasn't identified. `house_map` runs only in the PC helper, through
  `cell_map_keep_tiles`; PicoA has no tile store. Snapshots carry no map.
- **Floor first:** `cell_map_clear(x, y)` centres the window on the given pose and
  seeds a 35 cm circle as drivable (origin at boot, current pose on `clear_map`).
  Ground-layer ray passes count only within 50 cm. Obstacles, overhangs and known
  holes stop a drive in either direction across its full requested distance;
  unidentified floor stops a forward drive only within 25 cm of the current pose
  along the path. The stop margin is 15 cm. These are the code's current limits,
  not the earlier proposed 35 cm floor-check reach.
- **One column rule:** `cell_map_column_of()` in C classifies columns for both
  robot and PC (helper commands `V` and `K`); do not add a TypeScript copy.
- **The PC builds two maps**, in the odometry frame from power-up: the **cell map**
  (the robot's C code in a helper program, whole house; also the free-floor map)
  and the **line map** (TypeScript, [LINE_MAP_PLAN.md](LINE_MAP_PLAN.md)). They
  share nothing else. The page switches between them.
- **`clear_map`** clears only the robot's 6 × 6 m map (and re-seeds the floor around
  it). The PC's maps are not touched; the robot sends "map cleared" for the log.
- **Reads:** `robot map [n]` returns the PC's map at a snapshot; `--png` draws its
  tiles and window together, with the path. `robot check` uses the newest
  snapshot's cell map and pose, with the shared `room.h` stop rule.
  `robot map --robot [--png]` asks for the robot's live window and pose, without
  PC tiles. A snapshot number with `--robot` is an error. The live window covers
  every tile pixel beneath it, even where its cells are unknown.

## 4. The picture

Each box is one module; each arrow carries one named piece of data (section 5).

**ToF** (PicoA → PC)

```
VL53 ─► tof ─(tof_frame)─► pose_stamp ─(posed_frame)─► tof_format ─┬─(map use)───► cell_map ─► move checks (drive, go to)
                                ▲                      pack/unpack  ├─(watch use)─► watch
                        pose ───┘ pose_at(t)                       └─(bytes, during a move)─► link_pc ══TCP══► link ─► wire ─(posed_frame)─┬─► store
                                                                                                                                      ├─► cell_map (helper, read from the store)
                                                                                                                                      └─► line_map (step 7)
```

**IMU and odometry** (PicoB → PicoA → PC)

```
IMU ─(imu_sample, 416 Hz)─┬─► odometry ─(odometry_report, 50 Hz)══UART══► body ─► pose ─► pose_stamp, move
encoders ─(wheel_counts)──┘      │
                                 └─(imu_record, 104 Hz)══UART══► body ─(imu_record, PicoA's clock)─► move ─(imu_batch)─► link_pc ══TCP══► link ─► wire ─► store
```

**Control** (both ways)

```
page, CLI ─(command, heartbeat)─► link ══TCP══► link_pc ─► commands ─► move, behaviour, snapshot, cell_map clear
hello, answer, console line, status, move start / end, map cleared, snapshot part ─► link_pc ══TCP══► link ─► ingest ─► store, page
cell_map window ─(9 map parts, bulk)─► link_pc ══TCP══► link ─► runner (joins answer's pose) ─► map read
```

**Mounting**

```
common/mounting.json ─► CMake (robot_mounting.h) ─► main.c ─► tof_init, tof_rays_init, watch_camera, odometry_init
common/mounting.json ─► PC start ─► boot (a copy per boot) ─► objects; cell_map helper
```

The robot's own map and watch read the frame **after** packing and unpacking: the
same data the PC gets.

## 5. Names

Each name once; the same word in C (`snake_case_t`) and TS (`CamelCase`).

| Name | What it is | Made by | Read by |
|---|---|---|---|
| **mounting** | ToF position, zone angle, zone turn; camera position, f, tilt, size; IMU axes | `mounting.json` | `tof`, `tof_rays`, `watch_camera`, PicoB `odometry`, PC objects |
| **tof_frame** | 64 zones: distance mm, signal, sigma mm, **kind** (sure 5 / less sure 6, 9 / unsure / nothing), targets seen; number, time, its own period | `tof` | `pose_stamp` |
| **pose** | x, y, yaw (wrapped ±180°), pitch, still, at a moment | `pose` (`pose_at`) | `pose_stamp`, `move` |
| **posed_frame** | a tof_frame, the pose it was measured at, the turn during it | `pose_stamp()` in `pose.c` | `tof_format` |
| **use** | what a frame is packed for: map, watch, snapshot | — | `tof_format` |
| **rays** | a zone's rays in the world; its point, height span and **verdict** (floor / obstacle / unsure) | `tof_rays` | `cell_map`; on the PC also `line_map` (through the helper) |
| **cell_map** | 10 cm cells in layers G (ground), L1 (3-13 cm), L2 (13-23 cm) | `cell_map` | `move`, the page |
| **line_map** | lines in bands L1, L2, above (over 23 cm) | `line_map` (PC) | the page |
| **imu_sample** | one IMU reading, 416 Hz, float and raw counts, sensor frame | PicoB `imu` | `odometry` |
| **wheel_counts** | the four encoder counts | PicoB `encoder` | `odometry` |
| **odometry_report** | pose (x, y, yaw, pitch, roll), speeds, still, motors on, stop reason (37 B) | PicoB `odometry` | PicoA `body` → `pose` |
| **imu_record** | 4 consecutive imu_samples averaged: acceleration and gyro, int16 raw counts, sensor frame, time | PicoB `odometry` | PicoA `body` → `move` |
| **imu_batch** | up to 10 imu_records of a move, evenly spaced: move number, first time, period, count | PicoA `move` | the PC: store |
| **move** | one command part with its still time: start → still → act → still → end | `move` | `link_pc` (bulk), the PC |
| **message** | `[type 1 B][length 2 B][payload]` on the TCP connection | `wire` | `wire` |
| **status** | pose, mode, action, WiFi signal; 4 a second | `commands_status()` via `link_pc` | the page |
| **boot** | one power-up: id, the mounting copy, its message count | PC `boot.ts` | `store` |

Watching is called **watch** everywhere.

## 6. Data formats

### Compact tof format (uses: map, watch, snapshot)

On the wire a tof frame comes after its **move number** (u32) and its **time** (u64,
µs since power-up): 12 B, so a frame is filed by the move it names. The format itself
(below) is the same for every use.

Header 21 B: frame number 4, time 4 (PicoA's 32-bit clock, the middle of the
measurement; `robotClockUs` on the PC, which times frames by the 64-bit time), period 2
(0.1 ms), pose 8 (x, y mm; yaw ±180° and pitch in 0.01°), turn 2 (0.01°), flags 1
(still, pose known). Per zone (64, row 1 first), 4 B:

| Field | Size | Values |
|---|---|---|
| distance | 1 B | cm; 0 nothing in range, 255 beyond 2.54 m |
| signal | 1 B | kcps per SPAD, capped at 255 |
| sigma | 1 B | mm, capped at 255 |
| kind + targets | 1 B | kind (2 bits); targets the sensor saw (3 bits) |

- 277 B a frame (289 B with the move and time), 2.9 KB/s at 10 Hz.
- "Beyond" unpacks as 2550 mm with a flag: the map treats it as free to 2.55 m, no hit.
- Dropped: reflectance, ambient, SPAD count, temperature, the 2nd-4th targets.

### IMU

- **imu_record** (PicoB → PicoA, `MESSAGE_IMU`, 16 B): PicoB's time (when it read the
  samples), acceleration x, y, z and gyro x, y, z as int16 raw counts (±2 g, ±250 °/s), gyro
  bias not removed. A skipped sample restarts the record, so every record is 4
  samples in a row and a skip shows as a gap. `body` turns the time into PicoA's.
- **imu_batch** (PicoA → PC, bulk): move number 4, first time 8 (µs since power-up),
  period 2 (µs), count 1, then 12 B a record; up to 135 B. A gap or a step back in
  time starts a new batch; a move's last batch goes at its end, however short.

### Time

PicoA's clock (`time_us_32()`) wraps every 71.6 min (it starts 30 s before the wrap).
Inside the robot it stays 32-bit. Every time sent to the PC is **u64 µs since
power-up** (`clock_since_start_of()` in `common/clock_start.h`): it never wraps, so
the PC only divides by 10^6.

## 7. The connection

| Lane | Robot → PC | PC → robot |
|---|---|---|
| control | hello (boot, build, wire version), console line, answer, status, move start, move end, map cleared | command, heartbeat (every 2 s) |
| bulk | tof frame, imu batch, snapshot part, map part (the map window when asked, 9 parts) | — |

- **Control** (16 KB) never blocks the loop; when full, its oldest messages go
  (counted). Status goes only while control is empty, so it never piles up.
- **Bulk** (40 KB) goes only while control is empty, first in first out. When full, a
  message is refused (counted): frames and imu batches are dropped, a snapshot part
  waits and tries again. A lost connection drops what bulk holds; a snapshot cut by it
  isn't sent again (the new server gets its own).
- **Map on demand:** `map` sends the 10,800 cell bytes in 9 bulk parts, each with
  1,200 cell bytes and a 5 B origin/part header. `CommandRunner.askMap()` serializes
  map requests and waits for both the complete window and the numbered control
  answer's live pose; either may arrive first. A refused transfer or timeout is an
  error, not a usable partial map. No larger wire buffer is needed.
- The PC refuses a robot with another wire version (one note per address and version).
- Move start: move and command number, kind, parameter, time, still times, a
  **refused** flag (it couldn't start: blocked or already there). Move end:
  how it ended, frames sent and dropped, moved, rotated, imu batches sent and dropped.
- Answers: each ended move is answered in order (`move_next_ended()` keeps the last 8).
- The USB console stays plain text.

## 8. Modules

What each hides, and its interface. Built unless marked (step N).

### PicoB

| Module | Hides | Interface |
|---|---|---|
| `drivers/imu`, `drivers/encoder`, `drivers/motor` | registers, PIO | `imu_read(&sample, &counts)`; counts; power |
| `odometry` | gyro and wheel fusion; the 416 → 104 Hz averaging | `odometry_init(&mounting)`, `odometry_update()`; `odometry_report`; `odometry_imu_take()`; queue overflow and skipped-sample drops counted (`imu_lost`, `imu_skipped` in STATUS) |
| `drive` | speed control; tilt and wheel stops | `drive_set(v, w)`; a fault |
| `brain` | the link to PicoA | sends `MESSAGE_ODOMETRY`, `MESSAGE_IMU` (from the start), `MESSAGE_STATUS` |

### PicoA

| Module | Hides | Interface |
|---|---|---|
| `body` | the link to PicoB, the two clocks | odometry reports; `body_imu_take()` on PicoA's clock (kept once greeted and the clock is known); drive commands |
| `pose` | the odometry history | `pose_at(t)`, `pose_stamp(&tof_frame, &posed)` |
| `drivers/vl53` | the ST ULD over SPI | used only by `tof` |
| `tof` | the ULD's structs and statuses, the zone turn | `tof_init(&mounting.tof, hz)`, `tof_poll(&frame)`, rate getters |
| `tof_format` | byte layouts; the use → format table | `tof_format_pack(use, ...)`, `tof_format_unpack(...)`, sizes, message type |
| `tof_rays` | zone angles, the mounting, where a zone meets the floor | `tof_rays_init(&mounting)`, `tof_rays_of(...)` (rays, point, heights, verdict) |
| `cell_map` | grid, layers, window scrolling, floor-first rules, optional tile hook | `cell_map_add(&posed)`, `cell_map_obstacle_along()`, `cell_map_clear(x, y)`, `cell_map_window()`, `cell_map_column_of()`, `cell_map_keep_tiles()` |
| `watch` (parts `watch_tof`, `watch_camera`, `tracker`, `change_grid`) | backgrounds, blobs, the target | `watch_add(&posed)`, `watch_update()`, the target |
| `move` | the phases, counts, frame and imu batch sending | `move_start(kind, parameter, command, actor)`, `move_refused()`, `move_stop()`, `move_update()`, `move_tof()`, `move_imu()`, `move_next_ended()` |
| `behaviour` | how each action drives | three functions per action (begin, step, end) for `move` |
| `room` | the robot's size; where a drive stops before an obstacle on the map (pure; the PC's helper compiles it too) | `room_m()`, `room_gap_m()`, `room_drive()` |
| `wire` | the envelope and payloads (ToF bytes from `tof_format`) | pure encode / decode |
| `link_pc` | WiFi, finding the server, TCP, the two lanes, drop counts | `link_pc_send_control()`, `link_pc_send_bulk()` (true or false), `link_pc_connection()`, `link_pc_received()` |
| `snapshot` | its layout (camera, colour, features, compact tof frame; version 3) | `snapshot_request()`; parts to bulk |
| `commands` | the command words | answers each ended move; `map`, `clear_map`, `config pre_ms|post_ms`; `commands_status()` |

`main.c` shows the pipelines: poll → stamp → pack/unpack per use → map, watch, move;
`imu_pipeline()` hands records from `body` to `move`.

### PC (`pc/robot/src`)

| Module | Hides | Interface |
|---|---|---|
| `link.ts` | the TCP connection, messages | typed events in; commands out |
| `wire.ts`, `tof_format.ts` | the bytes; twins of the C, checked against `picoA/app/test/data/wire_golden.hex` | decode / encode / pack / unpack |
| `mounting.ts` | the file and its checks | `loadMounting()` |
| `boot.ts` | boot id, mounting copy, message numbering, µs → s | one per boot |
| `ingest.ts` | link events into the store, in order | used by `main.ts` and tests |
| `store.ts` | DuckDB `recordings/robot2.duckdb`: boots, moves, frames, zones, imu, status, snapshots (with bytes), objects | writes per message; reads for the page and CLI |
| `api/frames.ts` | `robot frames`, `robot imu` text and JSON (stored of sent, dropped, lost, missing) | |
| `fakerobot.ts` | a robot on the wire; replays a stored boot | |
| `objects/place.ts` | camera placement | takes the boot's camera geometry |
| `maps/cell_map.ts` + `native/cell_map_helper.c` | the helper program (robot `cell_map.c`, `tof_rays.c`, `house_map.c`, `tof_format.c`, `room.c`, built with cc once per server run), one per boot; built from the boot's stored frames when read; the last 16 maps read kept | `atSnapshot(boot, n, (m, pose) => ...)`: `drive` (room.h's rule), `obstacleWithin`, `view`; `viewAtSnapshot(boot, n)`; `rays(boot, frame)`; `columnsOf(cells)` |
| step 7: `maps/line_map/` | lines per height band: sights, pieces, merge, see-through (LINE_MAP_PLAN.md) | `lineMapsAt(boot, n)` |
| `api/map.ts`, `api/check.ts` | the map as a PNG (whole building, or the robot's window); check: a goto's turn and limit (`behaviour.h`'s numbers), a turn's warning; where a drive stops is the helper's (`room.h`) | |

## 9. Budgets

| Robot → PC while moving | KB/s |
|---|---|
| tof frames, 10 Hz | 2.9 |
| imu batches, 104 Hz | 1.4 |
| status, 4 a second | 0.2 |
| **Total** | **~4.5** |

Snapshots on top: ~20 KB each, one after each move. UART PicoB → PicoA: ~4.5 KB/s of
~100. PicoA: ~87 KB RAM free after step 6 (step 6 freed ~36 KB: `house_map` 20 KB,
`SNAPSHOT_MAX` 38912 → 22528 = 16 KB); core 0 stack worst case ~6.4 KB of 8.

## 10. Old data

- `pc/robot/recordings/robot.duckdb` is never opened by the new code.
- Old drives (2D maps, snapshots, the 3D take viewer): `git checkout data-v1` and its
  server on the old file. No converter.
- `picoA/app/test/test_map_replay.c` keeps the old 7 cm recordings as a host test,
  with `MOUNTING_BEFORE_LIFT`.

## 11. Steps left

**7. The line map** (PC) — [LINE_MAP_PLAN.md](LINE_MAP_PLAN.md): the design, the
substeps a-e and which are done. Three maps of lines, one per height band, built
from the stored frames at a snapshot; `robot map --png --lines [--no-cells]`.

Real-data limits to keep in mind:
- Floor rows 8 / 7 / 6 meet the floor at about 27 / 37 / 60 cm with the 9 cm mount.
- Shiny floor gives false hole readings beyond about 50 cm; the cell map ignores
  those ground passes, not all distant readings.
- Low obstacles can have an unsure verdict until close. Rows 1-3 on the right
  often have unsure readings around 31-37 cm; do not assume every close point is
  a real obstacle. Odometry drifts; a commanded 30° turn is about 31°.

**8. The map switch** (PC): cell map / line map on the page; the bands in their own
colours, each can be hidden.

**9. The flash and the robot test** (both Picos): docs final; the README list of robot
tests for steps 1-6 checked; both Picos flashed once (`build/picoA/picoA_app.uf2`,
`build/picoB/picoB_app.uf2`) with the new server. Daniel flashes and powers on;
then the agent runs the tests via `./robot` (section 2).
- Check window scrolling out and back; approaching a drop while held securely;
  `robot map --robot` against the PC's whole-building `robot map --png`.
- Also check watching people, the worst WiFi spot, the tilt stop and camera tilt.

## 12. Debugging without the robot

- `robot frames <boot> <move> [--json]`: a move's posed frames, frames stored of sent,
  dropped, lost; its imu count.
- `robot imu <boot> <move> [--json]`: the imu records; "missing (gaps)" must be 0.
- The page shows any snapshot's compact tof frame.
- `fakerobot` replays a stored boot to a test server (README).
- The PC's cell map rebuilds any boot's map at any snapshot from the store (`robot map
  <n>`, the page's timeline); `robot map --robot` asks the connected robot (or
  `fakerobot`) for its window right now.
- Measure the robot's map code on stored data: a script with `CellMaps` (as in
  `maps/cell_map.test.ts`) on `recordings/robot2.duckdb`, boot `cc0e4e65`.

## 13. Known issues (in TODO.md too)

- **Watch sees nothing beyond 2.54 m** (8-bit distance). Accepted.
- **Low tables:** an underside at ~13-16 cm shows as blocked, not overhang (sensor at
  9 cm). Drive stops the same. Tested as known.
- **Short drives:** a drive of 1 cm or less on open floor is refused as "obstacle".
- **Holes:** in a simulated room a 30 × 40 cm hole shows 2 of 12 columns drivable.
  Floor first (step 6) is meant to catch this; check on the robot.
- **Watch misses waving hands** often (both rates); the door zone after a restart can
  give false movement; the tracker's guessed direction can overshoot out of view.
- **Frames lost on a dropped connection** count as "sent" on the robot; `robot frames`
  shows them as lost.
- **DuckDB is opened per write batch** (~7.5 ms); check on a long watch.
- **PicoA's UART receive buffer** holds ~200 ms of PicoB's traffic; the worst loop is ~29 ms.
- **`place-objects`** re-places boots with their stored mounting; `--from-file` after
  a calibration.

## 14. Open questions for Daniel

1. **Camera:** f 188 and tilt 0° are not measured since the lift (calibrate later).
2. **Step 6 robot validation** is still due in Step 9. Passing host tests and reviews
   does not establish that floor-first driving is safe on the real floor.
