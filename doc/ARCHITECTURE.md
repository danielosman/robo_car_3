# RoboCar — architecture (as built)

How the robot works today, module by module, and where the code is. Written from
the code (11 Oct 2026). Open work is in [TODO.md](TODO.md), history in
[CHANGELOG.md](CHANGELOG.md), words in [GLOSSARY.md](GLOSSARY.md). Old plans and
designs are in [archive/](archive/). How to build, run and test is in the
[README](../README.md).

**Keep this file current.** When a module changes, change its section here. No
history goes here (that goes in the CHANGELOG), and no plans (they go in TODO.md).

## 1. The whole robot

- **PicoA** (Pico 2 W, the brain): the VL53L8CX ToF sensor, the HM0360 camera,
  WiFi. It keeps the map, detects movement, runs the actions the keys ask for, and
  prints its console on USB and over WiFi.
- **PicoB** (Pico 2, the body): four wheels with encoders and the IMU. It drives at
  the speeds PicoA commands, reports odometry and tilt, and stops the motors by
  itself when something is wrong.
- **The PC** (`pc/robot`, Node): the console page, and the recordings: it stores
  each recorded action in DuckDB, shows it in a 3D viewer and replays the robot's
  map code on it.

The two Picos talk over a UART link (1 Mbaud). PicoA talks to the PC over WiFi: a
TCP connection for the console, UDP datagrams for recordings.

**The robot does nothing by itself.** After power-up it is idle with the motors
off, joins the WiFi and waits for a key. Each key does one thing.

### PicoA's main loop (`picoA/app/main.c`)

One superloop on core 0, no RTOS, nothing allocated at run time. Each iteration
runs, in order: `body` (the link to PicoB), `pose`, `recorder` (odometry and drive
commands), `surroundings` (a new ToF frame into the map), `camera`,
`motion_sense` (movement detection), `behaviour` (the running action),
`wifi_console`, `debug_console` (keys). `loop_stats` times every stage. Core 1 is
not used.

### Frames and conventions

- Robot frame: x forward, y left, z up. Angles: + = left, nose up, left side up.
- The map's frame is odometry's from power-up (where PicoB started). It never
  moves: all takes of one power-up share it.
- Times are PicoA's `time_us_32()` (µs, wraps every 71.6 min). Both Picos start
  their clock 30 s before the wrap (`common/clock_start.h`), so every run and every
  host test crosses it; `common/stamp.h` takes differences across it.
- VL53 rows and columns are numbered 1-8 where people read them (row 1 at the top,
  column 1 on the left as the robot sees it); array indexes in the code start at 0.

## 2. PicoB (`picoB/app/`, `picoB/drivers/`)

- **`drive`**: closed-loop wheel speed per side, on both encoders of the side
  (averaged; both motors of a side get the same power). The turn rate is trimmed
  with the gyro, so turns are accurate despite skid steering. Commands are limited
  and ramped against wheel slip. Faults: a wheel not following its side's target
  for 1 s, or pitch or roll beyond 15°.
- **`odometry`**: heading from the gyro, distance from the encoders (each side the
  mean of front and rear), tilt from a complementary filter (gyro short-term,
  gravity long-term). Detects standing still; while still the heading is frozen
  and the gyro bias is measured again, so standing costs no drift.
- **`brain`**: PicoA as PicoB sees it. Answers PicoA's greeting, keeps the motors
  off until the protocol versions match, applies MOTORS and DRIVE, sends ODOM
  (50 Hz) and status (2 Hz). **Safety stops:** no DRIVE for 250 ms, or a fault from
  `drive`, switch the motors off until PicoA asks for them again.
- **Drivers:** `imu` (ISM330DHCX on SPI0, 416 Hz), `motor` (two TB6612FNG, 20 kHz
  PWM), `encoder` (quadrature counted by PIO, 8344 counts per wheel turn).

## 3. Shared between the Picos (`common/`)

- **`link`**: UART0 (GP0/GP1), COBS frames with CRC-16, interrupt-buffered, never
  blocks; a damaged message is dropped. The messages are packed structs in
  `link_msgs.h`, one header for both Picos (protocol version 4).
- **Helpers**: `stamp.h` (time differences across the wrap), `geom.h` (bearings,
  `wrap_pi`), `stats.h` (n-th value, median, sort), `units.h`, `clock_start.h`.

## 4. PicoA (`picoA/app/`, `picoA/drivers/`)

### Sensors and pose

- **`body`**: PicoB as PicoA sees it. Greets it, repeats DRIVE every 50 ms
  (PicoB stops after 250 ms without one), switches the motors, keeps the last 16
  ODOM reports numbered (so a slow loop iteration loses none) and translates
  PicoB's clock into PicoA's. PicoB's log lines are printed as `B: ...`.
- **`pose`**: the odometry reports of the last ~1.3 s, so `pose_at(t)` gives where
  the robot was when a sensor frame was measured (interpolated; pitch included,
  roll not).
- **`rangefinder`**: the VL53L8CX as 64 rays (8 × 8 zones, 5.625° each, 15 Hz). It
  hides the zone order and the mounting: turned 90° on the PCB, 2.5 cm ahead of
  the centre, 3 cm right, 7 cm above the floor. Per zone it keeps the nearest sure
  target (status 5, 6 or 9), or "nothing in range", or "unsure". It gives each
  ray's direction, also for points inside a zone and with the robot's pitch.
- **Drivers:** `tof` (the ST ULD over SPI0, all four targets per zone), `camera`
  (HM0360, 160 × 120 monochrome, captured by PIO and DMA in the background, its
  own exposure control in 10 ms steps against mains flicker, each row's time for
  the rolling shutter), `opt4048` (the colour sensor; used only by the bring-up
  firmware).

### The map (`surroundings`, `cell_map`)

`surroundings` feeds every ToF frame to `cell_map`, placed where the robot was when
the frame was measured. Nothing is learned first; `C` clears the map.

- **Cells** are 10 cm cubes in a fixed 6 × 6 m window around the power-up point,
  in three layers: **G** (ground, −7…+3 cm), **L1** (3-13 cm: what blocks the
  12 cm robot), **L2** (13-23 cm: overhangs). A cell is unknown, free or occupied,
  with the confidence of the measurement that set it and a doubt counter (1 byte).
- **Rays:** each zone shoots 9 rays (at 1/6, 1/2, 5/6 of its width and height), as
  long as its reading. A ray **passes** the cells it crosses and **ends** in one.
  A G cell is passed only by a ray leaving through its bottom (over a hole or a
  drop). A zone with nothing in range passes cells up to 1 m.
- **A reading is judged as a whole** by where its zone's cone first meets the
  floor (row 8 ~18 cm, row 7 ~24, row 6 ~34, row 5 ~63-71 cm, rows 1-4 never):
  - closer than 0.9 × that: an **obstacle**. All 9 rays hit, in L1 or above (rays
    ending below 3 cm are lifted to L1);
  - otherwise all rays ending below 3 cm: **floor** (they hit G);
  - otherwise **unsure** (grazing the floor, or a low thing on it): only its rays
    at floor height hit.
- **A ray clears only what it would have hit:** a pass counts against a cell only
  if the ray went through it at or below the highest point hit there + 1 cm. So
  rays passing over a low thing don't erase it.
- **Weights:** each ray counts `c(d) × v`: closeness, a sigmoid that is ≈1 up to
  0.5 m, 0.5 at 1.5 m and ≈0 at 3 m; and the VL53's confidence, 1 for status 5,
  0.5 for 6 or 9.
- **One frame's verdict on a cell:** H = the weights of the rays hitting it, P = of
  those passing it. Occupied if H > P, else free, with confidence
  q = |H − P| / max(H + P, 9): nine sure, close rays that agree give q = 1.
- **A cell keeps its best measurement:** the same verdict raises its confidence;
  a different verdict replaces it at once if more confident, or after 3 frames in a
  row with q ≥ 0.3 (roughly rays from within 1.3 m); weaker frames are ignored.
  Frames are not added up, so standing still doesn't wear away what was seen from
  closer, and a moved chair or a walker clears within a few frames.
- **Around the robot:** G within 15 cm of the centre is floor and L1 free (it
  stands and turns there); L2 within 25 cm, never seen by any zone, is set free if
  still unknown.
- **Columns:** blocked (L1 occupied), overhang (L2 occupied, L1 not), no floor (G
  free: a hole or a drop), drivable (floor seen, L1 and L2 free), open (L1 and L2
  free, G unknown), unknown.
- **Queries:** `cell_map_column()`, `cell_map_obstacle_along()` (the nearest blocked
  or overhang column on a path of a given width: what `f` stops at),
  `cell_map_print()` (the `m` key: 4 × 4 m as text).
- **Cost:** the worst frame ~27 ms on the robot (a frame comes every 66 ms).

**Known limits:** under a table top ~0.8 m away a few false blocked columns
(driving closer clears them); a small hole next to floor gets some floor hits;
standing still the floor is seen only in rings (~20, 30, 45, 70 cm); far walls
(beyond ~2 m) are weak and patchy; roll is not used (turning, the robot rolls
3-5°); the window doesn't follow the robot; a single echo can make a one-frame
blip that the next rays through it clear; beyond ~1 m, reflections in the waxed
floor give "no floor"; objects under 3 cm are ground by design.

### Movement detection (`motion_sense`, `tof_motion`, `camera_motion`, `change_grid`, `tracker`)

`motion_sense` runs the detectors every loop, logs their movement (only while
watching) and gives behaviour the biggest movement (`motion_sense_strongest()`).
Both detectors score cells in a **`change_grid`**: a cell has moved when it differs
in 2 of the last 4 frames next to another such cell, or in 3 alone; moved cells
that touch form blobs, reported as `motion_obs_t` (direction, extent, range if
known, strength).

- **`tof_motion`** (the VL53; watches still **and while turning in place**): the
  background is kept per world direction, not per zone: for each zone row, 256
  bins of 1.4° around the robot, each "free up to this distance". A zone's reading
  is a lower bound for every bin its cone swept during the frame (the turn comes
  from `pose_at()` at the start and end of the frame), so two agreeing readings
  fill or raise a bin. A zone is predicted from the nearest bin it swept (1° wider
  for heading error); it differs when it reads clearly closer (8 cm, or 8 % if
  more). A still view is known after 2 frames, and after a scan every direction:
  no learning after a stop. Something nearer that stays put for 1 s becomes
  background (pinned: raised again only after 10 s of farther readings). Farther
  readings are never movement. While turning the floor rows tell nothing. All bins
  are forgotten after 10 cm of driving.
- **`camera_motion`** (the camera; watches **only while still**): each 8 × 8-pixel
  block of the image (20 × 15) learns its brightness and noise over 1 s after
  every stop, with the exposure held. A block differs when it is off its
  background by clearly more than its noise; one steady for 1 s has stopped and
  becomes background. The exposure is changed only when calm (nothing moved for
  5 s, or after 30 s), then the view is learned again.
- **`tracker`**: follows one target in the VL53's blobs (its direction, range and
  angular speed; lost, left the view, about to leave). **It is only logged**:
  behaviour uses the biggest movement, not the tracker.

### Actions (`behaviour`)

One action at a time, started by a key; a new one replaces the running one, space
stops it. Each switches the motors on at its start and off at its end, and prints
how it ended: done (after standing still, with what odometry measured), stopped,
replaced, safety stop on PicoB, PicoB lost, or obstacle.

- **Scan** (`s`): turn 390° left in place (~14 s). Only that: the map is fed from
  every frame anyway.
- **Move** (`f`): 50 cm forward, or back with the direction set to back (`r`), at
  ≤ 10 cm/s, slowing near the end. **Stops 15 cm before an obstacle on the map:**
  from the front (back) edge to the nearest blocked or overhang cell on its path
  (the robot's 23.5 cm plus 5 cm each side), checked every loop while driving,
  slowing near the stop point; unknown cells count as free. Within 15 cm it doesn't
  start. Backwards only what the map already has stops it (the VL53 looks forward).
- **Turn** (`t`): 30° in place, left (right with direction back).
- **Watch** (`a`, 1 minute): turns towards the biggest VL53 movement and follows
  it: its angular speed (a line through the last 0.5 s) plus 2 × the angle still
  to go, at most ~29°/s; starts when it is 8° off, stops within 3° once it is
  about still, never turns past where it was last seen. A one-zone movement at the
  edge of the view is inspected too.
- **Record** (`5`): 5 s standing still, motors off, only for a recording.

Speeds come from `motion.h`. The decision "one owner of the wheels" isn't built
yet: `behaviour` drives, and `debug_console` switches the motors off on space.

### Console (`debug_console`, `wifi_console`)

The same keys on USB and over WiFi; nothing is printed by itself.

- Actions: space stop, `s` scan, `f` move, `t` turn, `a` watch, `5` record 5 s.
- Settings: `r` direction forward/back, `R` recording on/off.
- `C` clear the map, `W` connect to WiFi (or show how it is connected).
- Prints: `p` status (pose, PicoB link, WiFi, loop time over 10 s, RAM free,
  direction, recording), `m` the map, `z` one ToF frame, `c` one camera frame as
  blocks, `o` / `k` the ToF / camera movement backgrounds, `h` help.

`wifi_console` joins the network in `wifi_config.h` (git-ignored; copy
`wifi_config.example.h`), finds the server by its UDP announcements (or
`WIFI_SERVER_IP`), sends everything printed over TCP (port 4211) and takes keys;
it keeps the last 16 KB while no server listens and reconnects by itself. Losing
it doesn't stop the robot. lwIP runs in the background interrupt
(`lwipopts.h`).

### Recording (`recorder`)

With recording on (`R`), **each action is a take**: from its start to its end the
raw data goes to the PC as UDP datagrams (port 4212, ≤ 1400 B each, numbered):
`TAKE_START`, `GEOMETRY` (the sensor mounting, so the PC hard-codes nothing),
every ToF frame with all of the sensor's outputs (`TOF_RAW`, in two halves),
every odometry report, every drive command, the console's lines and keys
(`MARK`), the WiFi state every 0.5 s, and `TAKE_END` (how it ended, what was sent
and refused). Idle, nothing is sent. A lost datagram is lost: nothing waits behind
it, the PC counts the gaps by number; each datagram repeats the previous one's
odometry, and the first and last datagrams go more than once. The record layouts
are the `rec_*_t` structs in `recorder.c` (format version 3); the PC reads them in
`recording.ts` `parseRecord()`.

### Budgets

- **RAM:** everything is static, no `malloc`. ~195 KB free of 512 (`p` prints it;
  `.bss` ~310 KB).
- **Loop:** mean ~60 µs, worst ~29 ms (a map frame); a ToF frame every 66 ms, DRIVE
  every 50 ms (PicoB stops after 250 ms), 16 odometry reports kept (320 ms).
- **Stack:** core 0 ~6 KB worst case of the 8 KB it can use (both scratch banks;
  `observations()` in `tof_motion.c` keeps 3.2 KB on the stack).
  `tools/stack_depth.py` computes it from the call graph.

## 5. The PC (`pc/robot/`)

Node 24, TypeScript run directly; `npm start` serves http://127.0.0.1:8080/.

- **`main.ts`, `robot.ts`**: the server. Announces itself on the network, accepts
  PicoA's console connection, passes text to the page and keys back (WebSocket),
  writes a log per run in `logs/`. The page: `public/console.js`.
- **`recording.ts`**: receives the datagrams, saves each robot boot's as they
  arrive in `recordings/raw/<time>-boot-<id>.rec` (never changed: the truth), and
  notes each take as it ends (frames whole / half / missing, datagrams lost).
  `npm run decode -- <file>` prints a file's takes.
- **`store.ts`**: each take goes into DuckDB (`recordings/robot.duckdb`, derived
  from the raw files; `npm run import` rebuilds it): takes, ToF frames, zones,
  targets (also in world x, y, z), odometry, drive, marks, WiFi, and truth boxes.
  The database is opened per operation, so the `duckdb` command line can use it in
  between. The take list: http://127.0.0.1:8080/takes (`public/takes.js`).
- **`viewer_api.ts`, `public/viewer.js`**: http://127.0.0.1:8080/viewer, three.js
  served locally. A take, or a boot's whole drive (every take since a power-up on
  one timeline): the targets as points (coloured by row, signal, status, height),
  the robot's path and rays on a time slider, the replayed map as cubes, truth
  boxes placed by hand and the map's score against them.
- **`replay.ts`, `tools/replay_map.c`**: the robot's own `cell_map.c`, built for
  the Mac, run on a take's frames (`npm run frames` exports them;
  `tools/replay.sh <take>` prints the map). The floor margin can be changed to try
  other values.
- `recordings/` is git-ignored.

`pc/bringup/` is the bring-up viewer (camera, ToF, colour sensor over USB) for the
`picoA_bringup` firmware.

## 6. Tests and tools

- `./run_tests.sh`: host tests of every module above on the Mac, with fakes for
  the hardware and a simulated room (`test_behaviour`, `test_cell_map`), plus the
  map on five recorded scans (`test_map_replay`: the cup, box, chair and cupboard
  must be found; data in `picoA/app/test/data/`).
- `npm test` and `npm run typecheck` in `pc/robot/`.
- `tools/lint.sh`: cppcheck, ESLint, tsc.
- `tools/stack_depth.py`: worst-case stack from GCC's call graph.

## 7. Decisions (Daniel)

Rules the code follows; change them only with Daniel's word.

**The robot:**
- Robot height 12 cm, no extra clearance margin. Footprint 23.5 cm wide × 19 cm
  long, front edge 9.5 cm ahead of the centre.
- The ground band is −7…+3 cm: anything under 3 cm is ground and driven over.
  Everything above is an obstacle. **No floor learning.**
- Drivable = floor seen in the ground cell, 3-13 cm and 13-23 cm free. L2 cells
  within 25 cm of the robot, never seen by any zone, count as free.
- The map: 9 rays per zone, the best measurement wins; a reading judged as a whole.
  Not: adding up evidence over frames, one ray per zone.
- No climbing worries for now: the robot drives slowly in controlled rooms.

**Commands:**
- **The robot does nothing by itself:** idle with the motors off after power-up
  and after a reset, no start-up scan.
- One key, one operation; no repeat counts; no test-only actions.
- Motors off at the end of every action; space stops.
- USB and WiFi behave the same.
- `f` stops 15 cm before an obstacle on the map (blocked and overhang cells),
  unknown cells count as free. Its braking and brief speed-up when a closer reading
  moves the stop point back are fine as they are.

**Movement and watching:**
- Movement is detected while still and while turning, against a background kept
  per world direction; no learning after a stop.
- Watching follows the biggest movement: no predicting where it goes, no turning
  ahead of it; its measured angular speed may be used, never overshoot.
- Movement at the edge of the view is inspected (a one-zone blob turns the robot).

**Data:**
- Raw sensor data and the pose go to the PC, not the map, and only while an action
  runs (one take per action). Hypotheses and map changes are tested on recordings.
- Recordings stay out of git.
- No camera images to the PC for now; the camera's movement detection will be one
  reader of a feature record ([archive/CAMERA_MOTION_PLAN.md](archive/CAMERA_MOTION_PLAN.md)).
- Camera while watching: sharp and dark rather than noisy (exposure ≤ 10 ms, gain
  ≤ 8). For the camera work; the driver doesn't cap it yet.
- Datagram loss of up to 20 % per take is not acceptable (resending: TODO.md).
