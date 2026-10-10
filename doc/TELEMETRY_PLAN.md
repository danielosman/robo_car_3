# Telemetry: raw data to the PC, stored, viewed, replayed

Why (Daniel, 10 Oct 2026): the map's problems (the cup and the box at ~45 cm, MAP_DESIGN
§8) were argued from a simulation that models the VL53 as "the nearest surface", and
the fixes built on it were weak. We need the real sensor's data on the PC: look at
it in 3D, query it, form hypotheses (e.g. "a 5th-row zone with a cup in it returns
more signal than plain floor") and test map variants on the same recordings.
Status: **planned, not built.** Replaces ARCHITECTURE_PLAN §2.6's recording part
(R0) and REWORK_PLAN's open point 19. Keys: [COMMANDS_PLAN.md](COMMANDS_PLAN.md).

## 0. Start here (a new session)

**Where things stand (10 Oct 2026):** T1 (the commands, [COMMANDS_PLAN.md](COMMANDS_PLAN.md))
is built and passed its robot test: the robot is idle with the motors off until a
key, one key one action (space stop, `s` scan 390°, `f` move 50 cm, `t` turn 30°,
`r` direction, `a` watch 1 min, `C` clear the map), the same on USB and WiFi;
movement lines print only while watching. **Next: T2, recording on the robot**
(§2, §6), then T3-T6.

**Read first:** this file; [REWORK_PLAN.md](REWORK_PLAN.md) (decisions, how to work a
step); [COMMANDS_PLAN.md](COMMANDS_PLAN.md) (the keys, as built). Background for T6:
[MAP_DESIGN.md](MAP_DESIGN.md) §8.1 and the 10 Oct entries in
[REWORK_CHANGELOG.md](REWORK_CHANGELOG.md) (the cup analysis).

**Code T2 starts from:**
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
  raw API); the recording connection (4212) is a second one like it.
- `pc/robot/` (Node 24, TypeScript run directly, `ws`): `src/robot.ts` (the
  robot's TCP connection, `ROBOT_PORT` 4211), `src/main.ts` (HTTP 8080, WebSocket to
  the page, the log in `logs/`), `public/console.js` (the page's keys). `npm test`.
- Host tests: `./run_tests.sh` (C, stops at the first failure); firmware:
  `cmake --build build` → `build/picoA/picoA_app.uf2`.

**Rules that apply (REWORK_PLAN "How to work a step"):** host tests first; a robot
test only when the robot changes (hand Daniel the steps README-style and wait for
the log); update README / the plans / REWORK_CHANGELOG / REDFLAGS; ask before
behaviour that wasn't agreed; commit only when Daniel asks, no AI co-author trailer.
`recordings/` must be added to `.gitignore` (T3).

## 1. Decisions (Daniel)

- **Raw sensor data and the pose, not the map**, go to the PC. The map is computed
  from them (on the robot as now, and on the PC by replay), so any cell size, layer
  or vote rule can be tried on the same data.
- **Nothing is sent when nothing happens.** `R` turns recording on, but data flows
  only during an action (from its start to its end) or during `5` (record 5 s).
- **Each action is one take**, labelled with the key and action that caused it.
- **DuckDB** for queries on the PC.
- No camera images for now (REWORK_PLAN decision); a record type is kept for later.

## 2. Recording on the robot

- `R` switches recording on / off and prints it. Off at power-up.
- Recording on + an action starts → a **take** starts: `TAKE_START`, then the
  records below while the action runs, then `TAKE_END` when it ends. Recording on,
  idle: nothing is sent.
- Recording on but the PC's recording connection isn't there: the action runs
  anyway, the robot prints "not recorded: no recording connection".
- **Transport:** a second TCP connection to the server (port 4212) beside the
  console (4211), so the console's text never waits behind data. Records go into
  a send ring (~16 KB); the loop never waits.
- **When the WiFi can't keep up** (Daniel): whole ToF frames are dropped, `ODOM`,
  `MARK` and the take records are kept. The data says so: every `TOF_RAW` carries
  its frame number and how many frames were dropped before it, and `TAKE_END` the
  totals; the import puts them in `takes` and `tof_frames`, and the viewer shows
  the gaps.
- **Framing:** `[type u8][len u16][payload]` with a CRC per record (TCP already
  keeps order; the CRC catches our own bugs).

### Record types

| Type | When | Payload |
|---|---|---|
| `TAKE_START` | action start | boot id (random at power-up), take number (per boot), key, action name, parameters (distance or angle, direction), firmware build, PicoA time |
| `GEOMETRY` | after `TAKE_START` | sensor position (x, y, z), zone angle, rows/cols, the zone order used: so the PC never hard-codes the mounting |
| `TOF_RAW` | every ToF frame (15 Hz) | frame number (counts every frame the sensor gave, sent or not), frames dropped since the last one sent, frame time (PicoA clock, middle of the measurement); per zone: number of targets, ambient per SPAD, SPADs enabled; per target (up to 4): distance mm, sigma mm, signal per SPAD, reflectance %, status. All of it: what the robot uses today is one distance per zone |
| `ODOM` | every PicoB report (50 Hz) | PicoA time of the measurement, x, y, yaw, pitch, roll, v, w, still/moving, motors, safety stop, gyro bias |
| `DRIVE` | each command sent to PicoB | v, w |
| `MARK` | console lines printed during the take, keys pressed | text |
| `TAKE_END` | action end | how it ended (done / stop / safety stop / PicoB lost / replaced), records sent, records and ToF frames dropped |

Size: `TOF_RAW` ≈ 64 × (9 + 4 × 10) B ≈ 3.1 KB, × 15 Hz ≈ 47 KB/s; `ODOM` ≈ 2
KB/s. A scan (14 s) ≈ 0.7 MB; watching for a minute ≈ 3 MB. The first robot test
measures what the WiFi really carries and how many records get dropped.

The pose frame is odometry's from power-up (the scan no longer moves the origin),
so all takes of one boot share one frame; takes of different boots don't.

## 3. Storage on the PC

```
recordings/                       (git-ignored)
  2026-10-10/
    boot-7f3a/                    one robot boot
      take-001-s-scan.rec         the raw bytes as received, never changed
      take-002-5-record5s.rec
  robot.duckdb                    derived: rebuilt from the .rec files at any time
```

- The **`.rec` files are the truth**: replay reads them; a damaged database is
  just rebuilt.
- The server writes a take's file as it arrives and imports it into DuckDB at
  `TAKE_END` (or when the connection drops: the take is marked "cut").
- **Tables:**

| Table | One row per | Columns (main) |
|---|---|---|
| `takes` | take | take_id, boot_id, take_no, day, key, action, params, start/end time, end reason, dropped, file, note |
| `tof_frames` | ToF frame | take_id, frame_no, t_us, pose at t (interpolated from `ODOM`) |
| `tof_zones` | zone × frame | take_id, frame_no, row, col, nb_targets, ambient, spads |
| `tof_targets` | target × zone × frame | take_id, frame_no, row, col, target, distance_mm, sigma_mm, signal, reflectance, status, plus x, y, z in the world (computed at import from pose + `GEOMETRY`) |
| `odom` | report | take_id, t_us, x, y, yaw, pitch, roll, v, w, flags |
| `marks`, `drive` | record | take_id, t_us, text / v, w |
| `truth` | object you place | boot_id (or take_id), name, x, y, z, size: the real cup, box, wall |

One scan ≈ 210 frames, 13 k zones, up to 54 k targets: small for DuckDB.

The world x, y, z per target at import means "every 5th-row return within 10 cm of
the cup" is one query. Example questions:
- Signal per SPAD of 5th-row zones by distance: cup vs bare floor.
- How often a 2nd target appears in a zone that sees the cup and the floor.
- The 5th row's reading over bare floor: where it lands, how it scatters.

## 4. The viewer (three.js)

A page on the existing Node server (`pc/robot`), beside the console:
- **Take list**: day, boot, key, action, end reason, note; pick one take or
  several from one boot.
- **Point cloud** of `tof_targets` in world coordinates, coloured by signal,
  reflectance, status, row or target number; filters (rows, status, target
  number, signal range, time).
- **Robot pose** and its trail; a **time slider** through the take showing the
  frame's 64 zones as rays.
- **Truth objects**: place a box where the real cup was (x, y, size); saved in
  `truth`.
- **Map voxels** from replay (§5), one variant or two side by side.
- A query box for DuckDB SQL, results as a table (later: plotted).

## 5. Replay: the robot's code on recordings

- A host build of `cell_map.c` (and later `tof_motion.c`, `behaviour` logic) reads
  a take's `.rec` and writes the map as voxels (JSON) for the viewer.
- A **variant** is a build flag or a parameter set; replay runs several on the
  same takes. With `truth`, each gets a score (truth cells found, false blocked
  cells): "is the cup in the map?" becomes a number per variant.
- Recorded takes with truth become regression tests (`run_tests.sh`).

## 6. Steps

Each changes something visible; robot tests only where the robot changes.

1. **T1 Commands** (COMMANDS_PLAN): idle start, the key table, scan only 390°,
   move / turn / direction, stop on space, the tests and the 15 s status removed.
   Host tests; robot test: each key once. **Built and tested on the robot 10 Oct.**
2. **T2 Recording on the robot**: records, takes, `R`, `5`, the 4212 connection,
   drop counting. Robot test: a scan and 1 min of watching recorded; bytes,
   frames, drops.
3. **T3 Server storage**: `.rec` files per take, DuckDB import, the take list in
   the page. Check: a take's frames, zones, targets and pose in DuckDB.
4. **T4 Viewer**: the three.js point cloud, pose, time slider, filters, truth
   objects.
5. **T5 Replay**: `cell_map` on takes, voxels in the viewer, variants, scores.
6. **T6 The cup session**: scans with nothing, the cup at 45 / 80 cm, the 7 cm box;
   truth placed; then the hypotheses (signal, 2nd target, the 5th row's floor
   returns) and map fixes, tested on these takes.

## 7. Open (Daniel)

Decided 10 Oct: watching lasts 1 minute (~3 MB a take); when the WiFi can't keep
up, ToF frames are dropped and the data says so (§2).

1. DuckDB from Node (`@duckdb/node-api`) in the server: imports and the page's
   queries in one place. Analysis also from the command line (`duckdb` CLI), so
   Claude can query it directly.
2. The robot's map stays on the robot (needed for driving later); `m`'s text print
   goes once the viewer shows the replayed map?
