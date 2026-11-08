# Data flow, the PC's maps and the code's structure — plan

The robot sends what it measures during moves, packed small; the PC stores it and
builds the maps. The robot keeps only what it needs to drive. Two data pipelines (ToF
and IMU), one name per piece of data, the same on PicoB, PicoA and the PC.

This file holds the current state and what is left. History is in CHANGELOG.md; how
the built parts work is in ARCHITECTURE.md.

Contents: 1 Where we are · 2 How to continue · 3 Decisions · 4 The picture · 5 Names ·
6 Data formats · 7 The connection · 8 Modules · 9 Budgets · 10 Old data · 11 Steps left ·
12 Debugging · 13 Known issues · 14 Open questions.

## 1. Where we are (8 Nov 2026)

| Step | State |
|---|---|
| 0 Tag `data-v1` | done |
| 1 Mounting file, ToF at 9 cm | done, reviewed |
| 2 ToF at 10 Hz | done, reviewed |
| 3 ToF pipeline on the robot | done, reviewed |
| 4a Framed messages over TCP | done, reviewed |
| 4b Moves, frames, new database | done, reviewed |
| 4c Raw IMU at 104 Hz, peaks gone | done, reviewed, re-review fixes done (8 Nov) |
| 4d Simpler: move number in frames, 64-bit times, simpler link | done (8 Nov), not reviewed |
| 5 PC cell map | to do |
| 6 Robot: window only, floor first | to do |
| 7 Line map | to do |
| 8 Map switch on the page | to do |
| 9 Final docs, one flash, robot test list | to do |

- Steps 1-4c are committed (`3f9f8b7`); step 4d is in the working tree. The tag
  `data-v1` marks the last commit before step 1.
- All checks green on 8 Nov: `./run_tests.sh`, `tools/lint.sh`, `npm test` (73) and
  `npm run typecheck` in `pc/robot`, `cmake --build build` (0 warnings).
- **Docs behind:** ARCHITECTURE, CHANGELOG, README and TODO are not updated for step
  4d; they are brought up to date once, in step 9.
- **Nothing flashed, no robot tests run.** Both Picos are flashed once, in step 9
  (link protocol 6, wire version 4: PicoA, PicoB and the server go together).
- The robot tests for steps 1-4c are collected in the README section "Robot tests
  after the data-flow rework", to be run once after step 9.

## 2. How to continue

- **Each step:** built in the main session from this plan; one review pass after it
  (no second review of the fixes). Then the next step.
- **Simple first:** a rare case returns an error, is counted or is ignored; it isn't
  handled unless a robot test shows it matters. Tests cover the promised behaviour,
  not every corner.
- **Each step ends** green (the four checks above) and says which Picos it changes.
  The docs (ARCHITECTURE, TODO, README robot test list, CHANGELOG) are updated once,
  in step 9; this plan's state table is kept current.
- **No robot tests between steps.** Everything is checked by host tests, PC tests and
  review; the robot is flashed once at the end.
- **To show a reviewer only one step's change:** before the step, save the tree with
  a temporary index (no commit):
  `cp .git/index /tmp/idx && GIT_INDEX_FILE=/tmp/idx git add -A && GIT_INDEX_FILE=/tmp/idx git write-tree`;
  the reviewer runs `git diff <tree>`.
- Workers must not commit, tag, flash or run robot tests, and must not build later
  steps. Long test runs: cap them with a time limit, and never read large generated
  data into the agent's context.

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
  floor it hasn't identified (**floor first**, step 6). No whole-house map on the
  robot; it doesn't send its map (from step 6).
- **The PC builds two maps**, in the odometry frame from power-up: the **cell map**
  (the robot's C code in a helper program, whole house; also the free-floor map)
  and the **line map** (TypeScript, [LINE_MAP_PLAN.md](LINE_MAP_PLAN.md)). They
  share nothing else. The page switches between them.
- **`clear_map`** clears only the robot's 6 × 6 m map (and re-seeds the floor around
  it). The PC's maps are not touched; the robot sends "map cleared" for the log.

## 4. The picture

Each box is one module; each arrow carries one named piece of data (section 5).

**ToF** (PicoA → PC)

```
VL53 ─► tof ─(tof_frame)─► pose_stamp ─(posed_frame)─► tof_format ─┬─(map use)───► cell_map ─► move checks (drive, go to)
                                ▲                      pack/unpack  ├─(watch use)─► watch
                        pose ───┘ pose_at(t)                       └─(bytes, during a move)─► link_pc ══TCP══► link ─► wire ─(posed_frame)─┬─► store
                                                                                                                                      ├─► cell_map (helper, step 5)
                                                                                                                                      └─► line_map (step 7)
```

**IMU and odometry** (PicoB → PicoA → PC)

```
IMU ─(imu_sample, 416 Hz)─┬─► odometry ─(odom_report, 50 Hz)══UART══► body ─► pose ─► pose_stamp, move
encoders ─(wheel_counts)──┘      │
                                 └─(imu_record, 104 Hz)══UART══► body ─(imu_record, PicoA's clock)─► move ─(imu_batch)─► link_pc ══TCP══► link ─► wire ─► store
```

**Control** (both ways)

```
page, CLI ─(command, heartbeat)─► link ══TCP══► link_pc ─► commands ─► move, behaviour, snapshot, cell_map clear
hello, answer, console line, status, move start / end, map cleared, snapshot part ─► link_pc ══TCP══► link ─► ingest ─► store, page
```

**Mounting**

```
common/mounting.json ─► CMake (robot_mounting.h) ─► main.c ─► tof_init, tof_rays_init, watch_camera, odom_init
common/mounting.json ─► PC start ─► boot (a copy per boot) ─► objects; cell_map helper (step 5)
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
| **odom_report** | pose (x, y, yaw, pitch, roll), speeds, still, motors on, stop reason | PicoB `odometry` | PicoA `body` → `pose` |
| **imu_record** | 4 consecutive imu_samples averaged: accel and gyro, int16 raw counts, sensor frame, time | PicoB `odometry` | PicoA `body` → `move` |
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

Header 21 B: frame number 4, time 4 (PicoA µs, middle of the measurement), period 2
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

- **imu_record** (PicoB → PicoA, `MSG_IMU`, 16 B): PicoB's time (when it read the
  samples), accel x, y, z and gyro x, y, z as int16 raw counts (±2 g, ±250 °/s), gyro
  bias not removed. A skipped sample restarts the record, so every record is 4
  samples in a row and a skip shows as a gap. `body` turns the time into PicoA's.
- **imu_batch** (PicoA → PC, bulk): move number 4, first time 8 (µs since power-up),
  period 2 (µs), count 1, then 12 B a record; up to 135 B. A gap or a step back in time starts a new batch;
  a move's last batch goes at its end, however short.

### Time

PicoA's clock (`time_us_32()`) wraps every 71.6 min (it starts 30 s before the wrap).
Inside the robot it stays 32-bit. Every time sent to the PC is **u64 µs since
power-up** (`clock_since_start_of()` in `common/clock_start.h`): it never wraps, so
the PC only divides by 10^6.

## 7. The connection

| Lane | Robot → PC | PC → robot |
|---|---|---|
| control | hello (boot, build, wire version), console line, answer, status, move start, move end, map cleared | command, heartbeat (every 2 s) |
| bulk | tof frame, imu batch, snapshot part | — |

- **Control** (16 KB) never blocks the loop; when full, its oldest messages go
  (counted). Status goes only while control is empty, so it never piles up.
- **Bulk** (40 KB) goes only while control is empty, first in first out. When full, a
  message is refused (counted): frames and imu batches are dropped, a snapshot part
  waits and tries again. A lost connection drops what bulk holds; a snapshot cut by it
  isn't sent again (the new server gets its own).
- The PC refuses a robot with another wire version (one note per address and version).
- Move start: move and command number, kind, parameter, time, still times, a
  **refused** flag (a refused move doesn't close the running one on the PC). Move end:
  how it ended, frames sent and dropped, moved, rotated, imu batches sent and dropped.
- Answers: each ended move is answered in order (`move_next_ended()` keeps the last 8).
- The USB console stays plain text.

## 8. Modules

What each hides, and its interface. Built unless marked (step N).

### PicoB

| Module | Hides | Interface |
|---|---|---|
| `drivers/imu`, `drivers/encoder`, `drivers/motor` | registers, PIO | `imu_read(&sample, &counts)`; counts; power |
| `odometry` | gyro and wheel fusion; the 416 → 104 Hz averaging | `odom_init(&mounting)`, `odom_update()`; `odom_report`; `odom_imu_take()`; queue overflow and skipped-sample drops counted (`imu_lost`, `imu_skipped` in STATUS) |
| `drive` | speed control; tilt and wheel stops | `drive_set(v, w)`; a fault |
| `brain` | the link to PicoA | sends `MSG_ODOM`, `MSG_IMU` (after the greeting), `MSG_STATUS` |

### PicoA

| Module | Hides | Interface |
|---|---|---|
| `body` | the link to PicoB, the two clocks | odom reports; `body_imu_take()` on PicoA's clock; drive commands |
| `pose` | the odom history | `pose_at(t)`, `pose_stamp(&tof_frame, &posed)` |
| `drivers/vl53` | the ST ULD over SPI | used only by `tof` |
| `tof` | the ULD's structs and statuses, the zone turn | `tof_init(&mounting.tof, hz)`, `tof_poll(&frame)`, rate getters |
| `tof_format` | byte layouts; the use → format table | `tof_pack(use, ...)`, `tof_unpack(...)`, sizes, message type |
| `tof_rays` | zone angles, the mounting, where a zone meets the floor | `tof_rays_init(&mounting)`, `tof_rays_of(...)` (rays, point, heights, verdict) |
| `cell_map` | grid, layers, window scrolling | `cell_map_add(&posed)`, `cell_map_obstacle_along()`, `cell_map_clear()`, `cell_map_window()`. Step 6: floor first, tile hook, no `house_map` |
| `watch` (parts `watch_tof`, `watch_camera`, `tracker`, `change_grid`) | backgrounds, blobs, the target | `watch_add(&posed)`, `watch_update()`, the target |
| `move` | the phases, counts, frame and imu batch sending | `move_start(kind, param, command, actor)`, `move_refused()`, `move_stop()`, `move_update()`, `move_tof()`, `move_imu()`, `move_next_ended()` |
| `behaviour` | how each action drives | three functions per action (begin, step, end) for `move` |
| `wire` | the envelope and payloads (ToF bytes from `tof_format`) | pure encode / decode |
| `link_pc` | WiFi, finding the server, TCP, the two lanes, drop counts | `link_send_control()`, `link_send_bulk()` (true or false), `link_pc_connection()`, `link_received()` |
| `snapshot` | its layout (camera, colour, features, compact tof frame, map window and tiles until step 6) | `snapshot_request()`; parts to bulk |
| `commands` | the command words | answers each ended move; `config pre_ms|post_ms`; `commands_status()` |
| `house_map` | the whole-house tiles | goes to the PC helper in step 6 |

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
| step 5: `maps/cell_map.ts` + `native/cell_map_helper.c` | the helper program (robot `cell_map.c`, `tof_rays.c`, `house_map.c`, compiled on the Mac) | `open(boot)`, `add(posed)`, `pathClear()`, `rays(posed)`, `clear()`, tiles |
| step 7: `maps/line_map/` | lines, bands, quadtree | `add(posed, rays)`, lines in view |
| `api/map.ts`, `api/check.ts`, `store.ts` `worldPoint` | TS copies of robot code | replaced by the helper in step 5 |

## 9. Budgets

| Robot → PC while moving | KB/s |
|---|---|
| tof frames, 10 Hz | 2.9 |
| imu batches, 104 Hz | 1.4 |
| status, 4 a second | 0.2 |
| **Total** | **~4.5** |

Snapshots on top: ~20 KB each, one after each move. UART PicoB → PicoA: ~4.9 KB/s of
~100. PicoA: ~51 KB RAM free; core 0 stack worst case ~6.4 KB of 8. Step 6 frees ~31 KB
(whole-house map 20, smaller snapshot ~11).

## 10. Old data

- `pc/robot/recordings/robot.duckdb` is never opened by the new code.
- Old drives (2D maps, snapshots, the 3D take viewer): `git checkout data-v1` and its
  server on the old file. No converter.
- `picoA/app/test/test_map_replay.c` keeps the old 7 cm recordings as a host test,
  with `MOUNTING_BEFORE_LIFT`.

## 11. Steps left

**5. The PC cell map** (PC)
1. `native/cell_map_helper.c`: the robot's `cell_map.c`, `tof_rays.c`, `house_map.c`
   built on the Mac (cc, as the old replay tool was); fed posed frames over a pipe;
   started per boot with that boot's mounting; whole house (window plus tiles).
2. `maps/cell_map.ts`: `open(boot)` rebuilds from the store after a server restart;
   `add`, `pathClear`, `rays` (point, heights, verdict per zone, for the line map),
   `clear`, tiles to draw.
3. The page draws the PC's map; `robot map` and `robot check` ask the helper; the TS
   copies of robot code go (`api/map.ts` lookups, `check.ts` stop rules, `worldPoint`).
4. `robot map --robot`: the robot's own window, read for debugging. The two maps are
   close, not equal (the robot also maps idle frames; Pico and Mac round differently).

**6. The robot's map: window only, floor first** (PicoA, helper)
1. `house_map` leaves the robot (a tile hook in `cell_map`: the robot forgets, the
   helper keeps); the snapshot loses the window and the tiles.
2. Floor first in `cell_map`, so the helper has it too:
   - at a boot's start, floor known ~35 cm around the robot (row 8 first sees the floor
     ~25 cm ahead of the centre, ~15 cm ahead of the front edge);
   - each frame marks the cells under the robot's footprint as floor;
   - drive and go to stop 15 cm before an obstacle **or unidentified floor** (unknown
     or "no floor"). Today unknown counts as free.
3. Robot test (to the list): out of the window and back; towards a table edge (held):
   it stops. Expect some false stops on the waxed floor beyond ~1 m.

**7. The line map** (PC) — [LINE_MAP_PLAN.md](LINE_MAP_PLAN.md): points and verdicts
from the helper's `rays`; one ToF row's zones joined into lines; lines in bands L1,
L2, above, merged through a quadtree; a zone's wedge clears the part of a line in
front of its reading; built and tested on stored drives.

**8. The map switch** (PC): cell map / line map on the page; the bands in their own
colours, each can be hidden.

**9. The flash and the robot test** (both Picos): docs final; the README list of robot
tests for steps 1-6 checked; both Picos flashed once (`build/picoA/picoA_app.uf2`,
`build/picoB/picoB_app.uf2`) with the new server; Daniel runs the list.

## 12. Debugging without the robot

- `robot frames <boot> <move> [--json]`: a move's posed frames, frames stored of sent,
  dropped, lost; its imu count.
- `robot imu <boot> <move> [--json]`: the imu records; "missing (gaps)" must be 0.
- The page shows any snapshot's compact tof frame.
- `fakerobot` replays a stored boot to a test server (README).
- Step 5: the helper rebuilds any boot's map from the store; `robot map --robot`.

## 13. Known issues (in TODO.md too)

- **Watch sees nothing beyond 2.54 m** (8-bit distance). Accepted.
- **Low tables:** an underside at ~13-16 cm shows as blocked, not overhang (sensor at
  9 cm). Drive stops the same. Tested as known.
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

1. **Commit:** step 4d (this simplification) before step 5?
2. **Camera:** f 188 and tilt 0° are not measured since the lift (calibrate later).
