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

- **Done:** steps 1-5: the mounting file, ToF at 10 Hz, the ToF pipeline on the
  robot, one TCP connection with framed messages, moves, the new database, raw IMU at
  104 Hz, link version 7, whole-word names, `robot report`, the measurement run (4i:
  its facts in section 11), and the PC's cell map (step 5: section 11).
- **Dropped 10 Nov (Daniel), to finish the plan sooner:** step 4e's discussions and
  step 4g (interfaces). The IMU stays at 416 Hz on PicoB; the PC keeps its 104 Hz
  records. Odometry, `drive` and PicoA's layout stay as they are.
- **Next: step 6**, the robot's map window only, floor first (section 11, step 6:
  where to start). Then 7 line map, 8 map switch, 9 docs, the final flash, the robot
  test.
- **The robot now has** the firmware of commit `6583e88` (both Picos, flashed 10 Nov).
  A PicoA change from now on (step 5's `room.h` is the first) is tested on the robot
  in step 9 (or flashed earlier if Daniel wants).
- **Checks** (all green with step 5): `./run_tests.sh`, `tools/lint.sh`, `cmake
  --build build` (0 warnings); in `pc/robot`: `npm test`, `npm run typecheck`. `npm
  test` and the server build the cell map helper themselves (`cc`, into
  `build/host/cell_map_helper`).
- **Commits:** on `main`; pushed up to step 5's commit. Commit and push only when
  Daniel says. No `Co-Authored-By` lines (README, "Git conventions").
- **Docs behind the code:** ARCHITECTURE, README and TODO don't show 4d-5 (the
  README's robot tests still speak of `##` maps, link version 6, wire version 3;
  nothing on the PC's cell map or `room.h`). Where they disagree with this plan, this
  plan and the code are right. They are brought up to date once, in step 9. The
  CHANGELOG is up to date (history goes there, not here).
- **8 Nov, the architecture diagram** (`architecture-diagram/`, its README): the
  modules and 11 flows as boxes and ribbons, checked against the code by `npm run
  resolve` (in `tools/lint.sh`); its findings are in data/README.md. Its next change:
  the side bar shows the fields of what each step carries (its README, "Next").

## 2. How to continue

- **Read first:** this plan, AGENTS.md (how to answer Daniel; the code review guide),
  the README sections "How we work", "Conventions", "Git conventions".
- **The architecture diagram** is how Daniel and an agent look at the design: `npm
  run dev` in `architecture-diagram/` (http://127.0.0.1:5173/, `#flow=<id>` for one
  flow). An agent reads only `architecture-diagram/data/` (its README first). After
  a code change, `tools/lint.sh` lists stale references.
- **Each step:** built in the main session from this plan; then a review by a fresh
  subagent that reads the guide itself (`~/code/home-page/sources/skills/
  strategic-code-review.md`; it changes) and only reports (AGENTS.md); the fixes;
  Daniel may ask for a second fresh review; then Daniel says whether to commit.
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
    read them with a small script; `--robot` the robot's own from the snapshot),
    `robot map --png` and `robot camera [n]` (PNG files: open and look at them),
    `robot frames <boot> <move>`, `robot imu <boot> <move>`;
  - to look round, turn in 30° steps (12 × `r30`), not one 390° turn: a camera picture
    every 30°, to plan and to check.
- **Commit only when Daniel says** (README). To review one step, diff against the
  last commit.
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
| bulk | tof frame, imu batch, snapshot part | — |

- **Control** (16 KB) never blocks the loop; when full, its oldest messages go
  (counted). Status goes only while control is empty, so it never piles up.
- **Bulk** (40 KB) goes only while control is empty, first in first out. When full, a
  message is refused (counted): frames and imu batches are dropped, a snapshot part
  waits and tries again. A lost connection drops what bulk holds; a snapshot cut by it
  isn't sent again (the new server gets its own).
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
| `cell_map` | grid, layers, window scrolling | `cell_map_add(&posed)`, `cell_map_obstacle_along()`, `cell_map_clear()`, `cell_map_window()`. Step 6: floor first, tile hook, no `house_map` |
| `watch` (parts `watch_tof`, `watch_camera`, `tracker`, `change_grid`) | backgrounds, blobs, the target | `watch_add(&posed)`, `watch_update()`, the target |
| `move` | the phases, counts, frame and imu batch sending | `move_start(kind, parameter, command, actor)`, `move_refused()`, `move_stop()`, `move_update()`, `move_tof()`, `move_imu()`, `move_next_ended()` |
| `behaviour` | how each action drives | three functions per action (begin, step, end) for `move` |
| `room` | the robot's size; where a drive stops before an obstacle on the map (pure; the PC's helper compiles it too) | `room_m()`, `room_gap_m()`, `room_drive()` |
| `wire` | the envelope and payloads (ToF bytes from `tof_format`) | pure encode / decode |
| `link_pc` | WiFi, finding the server, TCP, the two lanes, drop counts | `link_pc_send_control()`, `link_pc_send_bulk()` (true or false), `link_pc_connection()`, `link_pc_received()` |
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
| `maps/cell_map.ts` + `native/cell_map_helper.c` | the helper program (robot `cell_map.c`, `tof_rays.c`, `house_map.c`, `tof_format.c`, `room.c`, built with cc once per server run), one per boot; built from the boot's stored frames when read; the last 16 maps read kept | `atSnapshot(boot, n, (m, pose) => ...)`: `drive` (room.h's rule), `obstacleWithin`, `view`; `viewAtSnapshot(boot, n)`; `rays(boot, frame)` |
| step 7: `maps/line_map/` | lines, bands, quadtree | `add(posed, rays)`, lines in view |
| `api/map.ts`, `api/check.ts` | the map as a PNG; check: a goto's turn and limit (`behaviour.h`'s numbers), a turn's warning; where a drive stops is the helper's (`room.h`) | |

## 9. Budgets

| Robot → PC while moving | KB/s |
|---|---|
| tof frames, 10 Hz | 2.9 |
| imu batches, 104 Hz | 1.4 |
| status, 4 a second | 0.2 |
| **Total** | **~4.5** |

Snapshots on top: ~20 KB each, one after each move. UART PicoB → PicoA: ~4.5 KB/s of
~100. PicoA: ~51 KB RAM free; core 0 stack worst case ~6.4 KB of 8. Step 6 frees ~31 KB
(whole-house map 20, smaller snapshot ~11).

## 10. Old data

- `pc/robot/recordings/robot.duckdb` is never opened by the new code.
- Old drives (2D maps, snapshots, the 3D take viewer): `git checkout data-v1` and its
  server on the old file. No converter.
- `picoA/app/test/test_map_replay.c` keeps the old 7 cm recordings as a host test,
  with `MOUNTING_BEFORE_LIFT`.

## 11. Steps left

**4e. Dropped 10 Nov** (kept below for later; not part of this plan)

Discussions with Daniel, before any code (no code; each ends in a decision
written into this plan)
1. **Who reads the IMU, and who hands its data on.** Today `odometry` is the IMU's
   only reader, and besides the pose it makes the imu records for the PC, which
   `brain` takes from it: odometry is the IMU's distributor. It has two jobs (the
   pose; recording the IMU for the PC) and every IMU user goes through it. The case
   to discuss: PicoB's main reads each sample once in an IMU pipeline (4g) and hands
   it to odometry (the pose), to an IMU recorder (the records for the PC) and,
   later, to bump detection; odometry then only makes the pose.
2. **PicoA's shape: a world-model flow.** The robot should build an understanding of
   its surroundings (the cell map), keep it up to date, and check each move against
   it. Today the parts are there (frames → pose stamp → `cell_map`; `behaviour`
   asks `cell_map_obstacle_along()`), but neither the code's layout nor the
   diagram shows it. To discuss: PicoA's modules in layers, sense (`tof`, camera,
   `body`) → world (`pose`, `cell_map`, `watch`) → decide (`commands`, `behaviour`,
   `move`) → talk (`link_pc`, `wire`, `snapshot`); the flows each layer may have; the
   missing flow in the diagram (a move checked against the map). Ends in a short
   spec here, then 4g follows it.
3. **PicoB's `drive`: reflexes.** `drive` does wheel control and two safety stops
   (tilt, a wheel not following); `brain` does the third (no drive command for
   250 ms) and turns drive's faults into stop reasons. So "why the motors went off"
   has two owners today. To discuss: a `reflexes` module owning every safety stop
   and its reason, and `drive` (or `wheels`) only following the speed it is given.
4. **The IMU's rate** (PicoB; the PC's stored data changes with it).
   - Today PicoB reads the IMU (ISM330DHCX) 416 times a second: odometry adds up every
     sample (heading, tilt), and each 4 are averaged into a 104 Hz imu record for the
     PC. The chip doesn't add them up into a heading: it gives turn rates and
     accelerations, smoothed by its own filter; heading and tilt are PicoB's code at
     any rate. (It has a small program unit and a buffer, but no heading output.)
   - For 104 Hz: at the robot's speeds the heading's error comes from the gyro's small
     steady error and its size error, not from the rate; drive control runs at 100 Hz;
     the PC already gets 104 Hz records. The 4-sample averaging (`update_record`,
     `IMU_RECORD_SAMPLES`) and the skipped-sample count (`imu_skipped` in STATUS) go.
     The chip smooths before it slows down, so motor shaking is less of a risk than
     reading every 4th sample would be.
   - Bumps: the chip's own bump detection (wake-up, tap) runs on the accelerometer's
     rate, and ST's note for the chip asks for ~417 Hz for taps (to check). The two
     rates are set apart (`CTRL1_XL`, `CTRL2_G`), so one option is: accelerometer at
     416 Hz for the chip's bump detection, gyro at 104 Hz, PicoB reading at 104 Hz.
     The chip's interrupt pins: check how they reach PicoB on the PCB.
   - What else changes at 104 Hz: `imu_calibrate_gyro(400)` counts samples (~1 s now,
     ~4 s then: count by time); `SAMPLE_GAP_US` has 416 in it; the wheels are read at
     each IMU sample, so drive's speeds update at 104 Hz. Stays: the still check, the
     tilt filter and the bias tracking (they work in seconds); `IMU_RECORD_PERIOD_US`
     and `wire.ts`. Keep polling (no chip buffer) unless a slow loop shows it matters.
   - Decided by 4i's data, or by Daniel from the reasons alone. If it changes, it is
     built right after 4i, before step 5, so the PC's maps start from the final
     data: link 7 → 8 (STATUS, the records; 4i flashed 7), wire 4 → 5 (a record is
     one sample, not 4; `boots.wire_version` tells which).

**4g. Dropped 10 Nov** (kept below for later; not part of this plan)

Interfaces take what they use (spec first; then PicoB, PicoA, the PC, one at a
time)
- The rule: a function takes either a named piece of data (section 5) that it reads
  most of, or a small input its own header declares. When it would read only a few
  fields of another module's struct, whether passed in or fetched with a getter
  (`body_odometry()`, `odometry_get()`), a pipeline picks them for it.
- A **pipeline** is the mapping for one data path: a function in the program's
  `main` (a program is one executable: PicoB's firmware, PicoA's, the PC server). It
  gets its own `.h` and `.c` only when `main` grows or it needs a host test of its
  own. PicoA's `tof_pipeline()` and `imu_pipeline()` are pipelines already; on the
  PC, `ingest.ts` is (link events into the boot and the store).
- Not one big `main` with every mapping inside one function, and not a module per
  pair of modules (a pass-through interface each).
- In the architecture diagram: a pick circle on a box other than a pipeline is a
  finding. (A fat ribbon alone is fine: a posed frame into `cell_map` is fat and
  read almost whole.) A pipeline in `main.c` has no header, so the diagram draws it
  once it has its own file; until then its pick shows on the receiver.
- First, the spec: a section in this plan listing each case, from the diagram (pick
  circles on receivers' rows) and a scan of the headers and getters: what it reads,
  the input it would take, the pipeline that maps it; and the cases left as they
  are (a module that reads most of a struct, e.g. `behaviour` of the odometry report).
  Daniel confirms it before any code changes.
- Known cases:
  - `drive_update(const odometry_t *)` reads 9 of `odometry_t`'s 17 values: pitch and roll
    (the tilt stop), the turn rate, the two sides' speeds and the four wheels' (speed
    control, the wheel-following stop). The tilt stop stays in drive: it sets
    drive's fault, which `brain` reports as the stop reason, so one module owns "why
    the motors went off". Drive takes `drive_input_t` (the 9 values, 36 B), declared
    in `drive.h`, which then no longer needs `odometry.h`; PicoB's `main` fills it.
  - `watch` reads only `stationary` of the odometry report (`watch.c`, via `body_odometry()`).

**What the measurement run showed** (4i, boot `cc0e4e65` in `robot2.duckdb`: two
12 × 30° look rounds, a 60 cm drive, a drive stopped by a box; the data for steps 6
and 7)
- Steps 1-4 work on the robot: nothing dropped or lost (frames, imu), link version
  7, loop max 88 ms, RAM free 51 KB, ToF 10.1-10.4 Hz; `gz` ~-40 standing (the
  bias), `az` ~16700.
- Floor rows 8 / 7 / 6 at 27 / 37 / ~60 cm: the ToF's 9 cm is right.
- A box ~8 cm high: seen by the ToF at ~95 cm, on the map only from ~45 cm (far away
  its zone spans the floor too: unsure); the drive stopped ~15 cm before it.
- **The shiny parquet:** much of the room beyond ~50 cm is "no floor" (G free) on the
  map (section 14, 4).
- Each 30° turn turns ~31°.
- Rows 1-3 of the right columns often read ~31-37 cm, unsure, in every direction:
  something of the robot in the ToF's view, or crosstalk (to look at).
- Not checked yet (step 9): watching people, the worst WiFi spot, the tilt stop, the
  camera's tilt.

**5. The PC cell map: done** (how it is now; the code is in section 8)
- The robot's own map code (`cell_map.c`, `tof_rays.c`, `house_map.c`,
  `tof_format.c`, `room.c`) runs in a helper on the Mac, one per boot with its
  mounting. A boot's map is built from its stored frames when read: "the map at
  snapshot n" is the map of the frames that came before snapshot n. It is never
  cleared by the robot's `clear_map`.
- The page draws it (the path since power-up); the page's ToF dots are placed by the
  helper. `robot map [n]` is the PC's map; `--robot` the robot's own from the snapshot.
  `robot check` uses the PC's map at the newest snapshot and its pose.
- The stop rule (where a drive stops before an obstacle) is `picoA/app/room.h`, one
  copy: the robot drives by it, the helper compiles it, `robot check` asks it.
- The PC's map and the robot's agree on ~98 % of known cells (4i, snapshots 13, 16):
  the robot also maps while standing, which it doesn't send.
- Known: a drive of 1 cm or less on open floor is refused as "obstacle".
- Left for step 6: `snapshot.ts` `columnOf` / `windowColumns` copy C `column_of()`.

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

Where to start (state after step 5):
- **The stop rule** is `picoA/app/room.h` (`room_m`, `room_drive`): `behaviour.c`
  drives by it, the PC's helper compiles it, `robot check` asks it. Today it looks up
  `cell_map_obstacle_along()` (blocked and overhang columns only). Floor first changes
  what counts there (unknown and close "no floor" too), so `robot check` follows by
  itself. `test_behaviour.c` and `pc/robot/src/maps/cell_map.test.ts` test it.
- **"No floor" only up close** (section 14, 4): measure the limit on boot `cc0e4e65`
  first (the helper replays it: section 12), e.g. at which distance its "no floor"
  columns turn out drivable later.
- **The seed floor** goes in `cell_map_clear()` (main.c and `clear_map` in
  commands.c call it; the helper calls it at start and to go back in time), so the
  helper has it too. A clear on the robot re-seeds around where it is (section 3).
- **The tile hook:** the helper compiles `house_map.c` and keeps the tiles; the robot
  doesn't (CMakeLists `app/house_map.c` goes; 20 KB). `cell_map.c` hands what scrolls
  out to a hook and starts what scrolls in as unknown on the robot.
- **The snapshot** (`snapshot.c`: `SEC_WINDOW`, `SEC_TILES`, `SNAPSHOT_VERSION` 2 → 3)
  loses the window and the tiles (~11 KB). On the PC they are read in `snapshot.ts`,
  `store.ts` (`map_*` columns, `snapshot_tiles`), `snapshots_api.ts` `snapshotMap`
  (`robot map --robot`), `snapshot.test.ts`, `test_snapshot.c`. `robot map --robot`
  then works only on older snapshots (or goes: ask Daniel). Then `snapshot.ts`
  `columnOf` / `windowColumns` (the last TS copy of `column_of()`) can go: the
  helper's `V` answer can send the column classes.
- **Flash:** PicoA only (link and wire versions unchanged unless a message changes);
  done in step 9 with everything else.

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
- The PC's cell map rebuilds any boot's map at any snapshot from the store (`robot map
  <n>`, the page's timeline); `robot map --robot` the robot's own (snapshots before step 6).
- Measure the robot's map code on stored data: a script with `CellMaps` (as in
  `maps/cell_map.test.ts`) on `recordings/robot2.duckdb`, boot `cc0e4e65`.

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

1. **Camera:** f 188 and tilt 0° are not measured since the lift (calibrate later).
2. **Line map:** LINE_MAP_PLAN.md is a draft; confirm it before step 7.
3. Decided 8 Nov: no abbreviations anywhere (4h, snapshot's `snapshot_*` types too); one
   measurement run (4i), then the final flash (9); PicoB's messages stay separate.
   Decided 10 Nov: 4e and 4g dropped; the IMU stays at 416 Hz.
4. **The shiny floor** (4i): beyond ~50 cm the parquet reads as "no floor". Decided
   10 Nov: step 6 trusts "no floor" only up close (the limit from the stored drives);
   farther it counts as unknown.
