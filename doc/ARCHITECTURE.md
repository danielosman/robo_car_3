# RoboCar — architecture (as built)

How the robot works today, module by module, and where the code is. Written from
the code (11 Oct 2026; snapshots, the moving map, modes, go to a point and camera
features added 12 Oct). Open work is in [TODO.md](TODO.md), history in
[CHANGELOG.md](CHANGELOG.md), words in [GLOSSARY.md](GLOSSARY.md). Old plans and
designs are in [archive/](archive/). How to build, run and test is in the
[README](../README.md).

**Keep this file current.** When a module changes, change its section here. No
history goes here (that goes in the CHANGELOG), and no plans (they go in TODO.md).

## 1. The whole robot

- **PicoA** (Pico 2 W, the brain): the VL53L8CX ToF sensor, the HM0360 camera,
  the OPT4048 colour sensor, WiFi. It keeps the map, watches for movement, finds and
  follows camera features, runs the commands the server's API sends (one line
  each, answered when it has ended), prints its console on USB and over WiFi, and
  sends a snapshot of what it knows when a command asks.
- **PicoB** (Pico 2, the body): four wheels with encoders and the IMU. It drives at
  the speeds PicoA commands, reports odometry and tilt, sends the IMU's raw
  readings (104 a second), and stops the motors by itself when something is wrong.
- **The PC** (`pc/robot`, Node): the command API with its one queue, which the
  `robot` CLI (an agent's or a person's) and the viewer and steering page both
  use; the page (the robot's map at any snapshot with the path driven, its camera
  picture with features, ToF, colour, pose; the log; steering); and DuckDB, the one
  store of what the robot sends: its moves, every ToF frame and imu record of a move,
  its statuses and snapshots.

The two Picos talk over a UART link (1 Mbaud). PicoA talks to the PC over WiFi: one
TCP connection with framed messages both ways (console lines, status, answers, moves
with their ToF frames and imu batches, snapshots; commands back). Nothing is sent while no move runs
but the console, the status and snapshots.

**The robot does nothing by itself.** After power-up it is in manual mode with the
motors off, joins the WiFi and waits for a command. Each command does one thing. A
mode says who chooses the actions: idle (nobody) or manual (the operator, by
commands: from the CLI, or the page's buttons and clicks on its map).

### PicoA's main loop (`picoA/app/main.c`)

One superloop on core 0, no RTOS, nothing allocated at run time. Each iteration
runs, in order: `body` (the link to PicoB), `pose`, the IMU pipeline (PicoB's imu
records, on PicoA's clock, to `move`, which batches them for the PC during a move), the ToF pipeline (a new frame
to the PC during a move, into the snapshot, the map and watch, below), `camera`,
`watch` (watching; it takes the camera's frames), `vision` (camera features, every
3rd frame), `move` (the running move's phases; inside it `behaviour`'s action), the colour sensor
(asked every 0.1 s), `snapshot`, `link_pc` (the link to the PC), `commands` (the
server's commands, and their answers). `loop_stats` times every stage. Core 1 is not used.

### Frames and conventions

- Robot frame: x forward, y left, z up. Angles: + = left, nose up, left side up.
- The map's frame is odometry's from power-up (where PicoB started). It never
  moves: all moves of one power-up share it.
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
- **`odometry`**: the IMU's axes turned into the robot's by the mounting
  (`common/mounting.json`, given to `odom_init()`); heading from the gyro, distance from the encoders (each side the
  mean of front and rear), tilt from a complementary filter (gyro short-term,
  gravity long-term). Detects standing still; while still the heading is frozen
  and the gyro bias is measured again, so standing costs no drift. **The imu
  records:** each 4 of the IMU's samples (416 a second) averaged into one
  `imu_record_t` (104 a second; `odom_imu_take()`, the newest 8 kept): accel and gyro
  x, y, z as the sensor's own int16 counts (±2 g, ±250 °/s), its own axes, nothing
  taken out (the gyro's bias neither), rounded to the nearest count (halves away from
  zero), stamped with the middle of the times PicoB read its samples (a small steady
  lag after the sensor measured them, plus a loop's jitter) on PicoB's clock. Its 4
  samples are always consecutive: a sample skipped (more than 1.5 sample times since
  the one before) drops the partial record and starts a new one, so the skip shows
  as a gap. Records not taken in time and partial records dropped after a skipped
  sample are counted (`odom_imu_lost()`, `odom_imu_skipped()`, in STATUS; PicoA's
  `status` prints both). Training
  data for the PC; odometry itself uses the float samples.
- **`brain`**: PicoA as PicoB sees it. Answers PicoA's greeting, keeps the motors
  off until the protocol versions match, applies MOTORS and DRIVE, sends ODOM
  (50 Hz), each imu record as it is made (`MSG_IMU`, once greeted) and status (2 Hz). **Safety stops:** no DRIVE for 250 ms, or a fault from
  `drive`, switch the motors off until PicoA asks for them again.
- **Drivers:** `imu` (ISM330DHCX on SPI0, 416 Hz; `imu_read()` gives the sample in g
  and °/s, the gyro's power-up bias taken out, and the same reading as raw counts), `motor` (two TB6612FNG, 20 kHz
  PWM), `encoder` (quadrature counted by PIO, 8344 counts per wheel turn).

## 3. Shared between the Picos (`common/`)

- **`link`**: UART0 (GP0/GP1), COBS frames with CRC-16, interrupt-buffered, never
  blocks; a damaged message is dropped. The messages are packed structs in
  `link_msgs.h`, one header for both Picos (protocol version 6): HELLO, MOTORS, DRIVE
  (A → B); ODOM (45 B, 50 Hz), IMU (16 B, 104 Hz), LOG, STATUS (B → A: the gyro bias,
  imu records lost and dropped on PicoB). The receive buffer (1 KB) holds ~200 ms of PicoB's
  traffic; bytes beyond it are lost (a bad frame, the rest counted as lost). ~4.9 KB/s from
  PicoB of the ~100 KB/s the UART carries.
- **Helpers**: `stamp.h` (time differences across the wrap), `geom.h` (bearings,
  `wrap_pi`), `stats.h` (n-th value, median, sort), `units.h`, `clock_start.h`, `crc32.h`
  (zlib's CRC-32, for snapshots).
- **The mounting** (`mounting.json`): where the sensors sit, the one place for it.
  The ToF: 3.2 cm ahead of the centre, 2.6 cm right, 9 cm up, zones of 5.625°, its
  zone grid turned 90° on the PCB. The camera: 2.5 cm ahead, 0.5 cm right, 10.8 cm
  up, f 188 px, tilt 0° (assumed), 160 × 120. The IMU: its axes turned into the
  robot's (180° about z). CMake reads it at configure time (`mounting.cmake`, again
  when the file changes) and writes `robot_mounting.h` into the build dir:
  `ROBOT_MOUNTING`, a `mounting_t` (`mounting.h`). Only each Pico's `main.c`
  includes it and passes it on: PicoA to `tof_rays_init()`, `tof_init()` (the zone
  grid's turn) and `watch_init()` (the camera), PicoB to `odom_init()`. No module
  has the numbers. A wrong zone turn makes `tof_init()` fail ("ToF sensor not
  working"), a wrong camera makes the camera not watch (main.c prints both). The
  PC reads the same file (`pc/robot/src/mounting.ts`). Host tests make the header
  with the same script (`build/host/`); the ones on recordings pass the mounting
  from before the PCB was lifted (`picoA/app/test/mounting_before_lift.h`: the ToF
  at 7 cm, the camera at 8.8 cm).

## 4. PicoA (`picoA/app/`, `picoA/drivers/`)

### Sensors and pose

- **`body`**: PicoB as PicoA sees it. Greets it, repeats DRIVE every 50 ms
  (PicoB stops after 250 ms without one), switches the motors, keeps the last 16
  ODOM reports numbered (so a slow loop iteration loses none) and translates
  PicoB's clock into PicoA's (the smallest delay of the ODOM reports). PicoB's imu
  records get their time on PicoA's clock as they arrive and wait to be taken
  (`body_imu_take()`, the newest 32, ~0.3 s; the lost counted); records before the
  clock is known (a hello, then no report yet) are dropped. PicoB's log lines are
  printed as `B: ...`.
- **`pose`**: the odometry reports of the last ~1.3 s, so `pose_at(t)` gives where
  the robot was when a sensor frame was measured (interpolated; pitch included,
  roll not).
- **Drivers:** `vl53` (the ST ULD over SPI0, all four targets per zone; only `tof`
  reads it), `camera`
  (HM0360, 160 × 120 monochrome, subsampled by 4, no binning; its numbers in
  [CAMERA_PARAMS.md](CAMERA_PARAMS.md); captured by PIO and DMA in the background, its
  own exposure control in 10 ms steps against mains flicker, each row's time for
  the rolling shutter; `camera_last_frame()` gives other readers the frame
  `watch` took), `opt4048` (the colour sensor on I2C0: coherent readings of
  its four channels' raw registers, ~2.5 a second; converted to lux and colour on
  the PC).

### The ToF pipeline (`tof`, `pose_stamp()`, `tof_format`, `tof_rays`)

`main.c` shows it: **poll → stamp → to the PC during a move (`move_tof()`, packed for
the map use) → the snapshot (packed for the snapshot use) → pack and unpack per use →
the map, watch.**
Each name is one piece of data ([DATA_FLOW_PLAN.md](DATA_FLOW_PLAN.md), the names;
the types in `tof_frame.h`).

- **`tof`** (`tof_poll()` → a **tof_frame**): the only reader of the sensor's
  results and status numbers (driver `vl53`). 64 zones, 8 × 8, as the robot sees
  them (it turns the sensor's zone order by the mounting's `zones_turn_deg`, given
  to `tof_init()`). **One target per zone:** the first with status 5, 6 or 9 and a
  distance (the sensor gives the closest first; its echo at ~30 cm often comes
  first with status 12 or 14 and is skipped). Per zone: distance (mm), signal
  (kcps per SPAD), sigma (mm), **kind** (sure: status 5; less sure: 6 or 9;
  unsure: targets but none valid, the first one's numbers; nothing), and how many
  targets the sensor saw. Per frame: its number, its middle time and **its own
  period** (how long it measured): readers take the period from the frame, never
  from a global rate. The rate is a setting (`tof_init(&mounting.tof, hz)`, 1-15;
  `main.c` starts it at 10); the sensor's firmware is uploaded once (~2 s), a new
  rate only restarts it. `status` prints the rate set and the frames a second
  read (over the last 16). Its numbers in [TOF_PARAMS.md](TOF_PARAMS.md).
- **stamp** (`pose_stamp()` in `pose`) → a **posed_frame**: the frame and where
  the robot was at its middle (x, y; yaw wrapped to ±180°; pitch), how far it
  turned during the frame's period, whether PicoB said it stood still, and
  whether there was a pose at all (no PicoB: none).
- **`tof_format`**: packs a posed frame for a **use** (map, watch, snapshot) and
  unpacks it; pure functions. A table says which format each use gets; today one,
  **compact**, for all: a 21 B header (frame number, time, period in 0.1 ms, x and
  y in mm (±32.7 m: plenty for a flat), yaw, pitch and the turn in 0.01°, flags
  still and pose known) and 4 B per zone (distance in whole cm, 0 nothing, 255
  "beyond 2.54 m"; signal and sigma capped at 255; kind and targets): 277 B,
  2.8 KB/s at 10 Hz. **The map and watch read the frame after packing and
  unpacking** for their use: the same data the PC will get (a frame that fails to
  unpack is skipped). An unpacked frame says from which distance on it only knows
  "at least that far" (2.55 m); the map takes such a zone as free up to there
  (weighted by its kind), watch as "free up to it" as any reading.
- **`tof_rays`**: where the zones look, from the mounting (`tof_rays_init()`,
  before anything else: the ToF 9 cm above the floor, 3.2 cm ahead of the centre,
  2.6 cm right). For a posed frame it places a zone's reading in the world (its 9
  rays, their end point and height span) and gives its **verdict**: obstacle,
  floor or unsure (below). Also each zone's direction, the sensor's place and the
  floor rows' limits for watch, and rays with the robot's pitch for the tests'
  simulators. Pure geometry, no sensor: the PC's helper will compile it.
- Only `tof` reads the sensor's results: nothing else sees the driver's structs.

### The map (`cell_map`)

`cell_map_add()` takes every posed frame of the map use (one without a pose is
left out), moves the window with the robot first, and maps each zone where the
robot was when the frame was measured. Nothing is learned first; `clear_map`
clears the map. Unsure zones tell nothing.

- **Cells** are 10 cm cubes in a 6 × 6 m window that follows the robot (below),
  in three layers: **G** (ground, −7…+3 cm), **L1** (3-13 cm: what blocks the
  13 cm robot), **L2** (13-23 cm: overhangs). A cell is unknown, free or occupied,
  with the confidence of the measurement that set it and a doubt counter (1 byte).
- **Rays:** each zone shoots 9 rays (at 1/6, 1/2, 5/6 of its width and height), as
  long as its reading (`tof_rays`). A ray **passes** the cells it crosses and **ends** in one.
  A G cell is passed only by a ray leaving through its bottom (over a hole or a
  drop). A zone with nothing in range passes cells up to 1 m; one "beyond 2.54 m"
  passes them up to 2.55 m and ends nowhere.
- **A reading is judged as a whole** (`tof_rays`' verdict) by where its zone's
  cone first meets the floor (along the ray, the ToF at 9 cm: row 8 ~23 cm, row 7
  ~31, row 6 ~46, row 5 ~92 cm, rows 1-4 never; the centre rays meet it at
  ~27 / 37 / 61 / 183 cm):
  - closer than 0.9 × that: an **obstacle**, a **detection**. Which of the cells its
    9 rays end in holds it isn't known, so all of them are marked occupied (in L1
    or above: rays ending below 3 cm are lifted to L1), each as sure as the whole
    reading, however its rays happen to split between cells;
  - otherwise all rays ending below 3 cm: **floor** (they hit G);
  - otherwise **unsure** (grazing the floor, or a low thing on it): only its rays
    at floor height hit.
- **A ray clears only what it would have hit:** a pass counts against a cell only
  if the ray went through it at or below the highest point hit there + 1 cm. So
  rays passing over a low thing don't erase it. The cell keeps that height while it
  is occupied, or doubted, until a pass below it finds nothing.
- **A detection beats the passes beside it:** a cell marked by a detection gets no
  passes in that frame. A thin leg fills 2 cm of a 10 cm cell, and the other zones'
  rays pass it on their way to the wall behind (snapshot 35, 12 Oct: the legs were
  "drivable, 15/15", and go-to drove into them).
- **Weights:** a reading counts `c(d) × v`: closeness, a sigmoid that is ≈1 up to
  0.5 m, 0.5 at 1.5 m and ≈0 at 3 m; and the VL53's confidence of the target used,
  1 for status 5, 0.5 for 6 or 9. In L1 and L2 a hit's d is the reading's length,
  a pass's d where the ray leaves the cell (it shows the cell free only that far);
  in G both are where the ray enters the cell.
- **One frame's verdict on a cell:** in L1 and L2, H = the strongest hit in it, P =
  the strongest pass (rays are not added up: a thin leg's few rays count as its
  whole reading). Occupied if H > P, else free, with confidence
  q = |H − P| / max(H + P, 1): one sure, close reading gives q = 1. In G the rays
  are added up and q = |H − P| / max(H + P, 9): the floor fills a zone, and the few
  steep rays of a reading that end a little below it (one length for all 9) must
  not outweigh the ones that hit it.
- **A cell keeps its best measurement:** the same verdict raises its confidence;
  a different verdict replaces it at once if more confident, or after 3 frames in a
  row with q ≥ 0.3 (a status-5 reading from within ~1.7 m, a status-9 one within
  ~1.4 m); weaker frames are ignored, so a far, wide footprint can't thicken again
  what was seen from closer.
  Frames are not added up, so standing still doesn't wear away what was seen from
  closer, and a moved chair or a walker clears within a few frames.
- **Around the robot:** G within 15 cm of the centre is floor and L1 free (it
  stands and turns there); L2 within 25 cm, never seen by any zone, is set free if
  still unknown.
- **Columns:** blocked (L1 occupied), overhang (L2 occupied, L1 not), no floor (G
  free: a hole or a drop), drivable (floor seen, L1 and L2 free), open (L1 and L2
  free, G unknown), unknown.
- **Queries:** `cell_map_column()`, `cell_map_obstacle_along()` (the nearest blocked
  or overhang column on a path of a given width: what drive and go-to stop at; the
  PC's `robot check` has a copy, tested against it), `cell_map_print()` (4 × 4 m as
  text, for the tests). Outside the window they answer from
  the whole-house map's columns.
- **The window follows the robot** (`cell_map_follow()`, inside `cell_map_add()`): it is
  3 × 3 tiles of 2 × 2 m of the whole-house map, the robot in the middle tile; when
  the robot is more than 0.5 m out of the middle tile (within ~1.5 m of the
  window's edge) it scrolls by a whole tile, so its corner is always on the 10 cm
  grid and nothing is resampled. At power-up it is centred on the power-up point,
  as before. The tiles that scroll out go into the whole-house map as columns; the
  ones that scroll in start from it, each column as cells only 1/15 confident, so
  any measurement overrides them.
- **The whole-house map** (`house_map`): each 10 cm column's class (4 bits) in 2 × 2
  m tiles, only the tiles seen (at most 100: a 20 × 20 m flat, 20 KB). **The window
  is the truth where it knows something:** a known column overwrites the stored
  one (a moved chair shows), an unknown one keeps it (the robot didn't look). Tiles
  changed since they were last sent are dirty and go with the next snapshot; a new
  server gets them all. `C` clears it with the window.
- **Cost:** the worst frame ~27 ms on the robot (measured at 15 Hz; a frame now
  comes every 100 ms, so the map takes less of the loop).

**Known limits:** under a table top ~0.8 m away a few false blocked columns
(driving closer clears them); a small hole next to floor gets some floor hits;
standing still the floor is seen only in rings (~25-33, 33-48, 48-95 cm from the
centre, and row 5's from ~95 cm). The ToF first sees the floor ~25 cm ahead of the
centre (15 cm ahead of the front edge): from the robot's 15 cm floor circle to there the floor isn't
seen standing still. A table top 6 cm above the ToF (15-20 cm) seen from ~0.6 m:
row 4 grazes its underside where row 3 sees its edge, so its front columns show
blocked, not overhang (host test, `test_cell_map`); far walls
(beyond ~2 m) are weak and patchy; roll is not used (turning, the robot rolls
3-5°); a single echo can make a one-frame
blip that the next rays through it clear; beyond ~1 m, reflections in the waxed
floor give "no floor"; objects under 3 cm are ground by design.

### Watch (`watch`; inside it `watch_tof`, `watch_camera`, `change_grid`, `tracker`)

`watch.h` is the one interface for watching: `watch_add()` takes every posed frame
of the watch use, `watch_update()` the camera's frames every loop; it logs the
movement (only while watching) and gives behaviour the biggest movement
(`watch_strongest()`). The rest are its inside parts; nothing else calls them.
Both detectors score cells in a **`change_grid`**: a cell has moved when it differs
in 2 of the last 4 frames next to another such cell, or in 3 alone (frames, not
time: how many readings agree; 0.4 s at the VL53's 10 Hz); moved cells
that touch form blobs, reported as `watch_obs_t` (direction, extent, range if
known, strength).

- **`watch_tof`** (the VL53; watches still **and while turning in place**): the
  background is kept per world direction, not per zone: for each zone row, 256
  bins of 1.4° around the robot, each "free up to this distance". A zone's reading
  is a lower bound for every bin its cone swept during the frame (the frame's yaw
  ± half its turn), so two agreeing readings
  fill or raise a bin. A zone is predicted from the nearest bin it swept (1° wider
  for heading error); it differs when it reads clearly closer (8 cm, or 8 % if
  more). A still view is known after 2 frames, and after a scan every direction:
  no learning after a stop. Something nearer that stays put for 1 s becomes
  background (pinned: raised again only after 10 s of farther readings). Farther
  readings are never movement. Turning means faster than 15°/s during a frame;
  then the floor rows tell nothing. Its times are times: each frame's own period
  turns them into frame counts (1 s is 10 frames at 10 Hz). A frame without a
  pose counts as where the last one was, not turning. **It doesn't see movement
  beyond ~2.54 m:** the watch use's distances end there, so someone at 2.7 m in
  front of a wall at 3.5 m reads "beyond" like the wall (accepted, 7 Nov). All bins
  are forgotten after 10 cm of driving.
- **`watch_camera`** (the camera; watches **only while still**; its f from the
  mounting, `watch_init()`): each 8 × 8-pixel
  block of the image (20 × 15) learns its brightness and noise over 1 s after
  every stop, with the exposure held. A block differs when it is off its
  background by clearly more than its noise; one steady for 1 s has stopped and
  becomes background. The exposure is changed only when calm (nothing moved for
  5 s, or after 30 s), then the view is learned again.
- **`tracker`**: follows one target in the VL53's blobs (its direction, range and
  angular speed from the last 0.75 s, at least 0.15 s of it; lost after 0.5 s,
  left the view, about to leave). **It is only logged**:
  behaviour uses the biggest movement, not the tracker.

### Camera features (`vision`)

Milestone V1 of [archive/CAMERA_MOTION_PLAN.md](archive/CAMERA_MOTION_PLAN.md):
corners found in the camera image and followed from frame to frame, on core 0, on
every 3rd camera frame (~8 a second) and on the frame a snapshot needs. Each
processed frame is one record (`vision_frame_t`): every feature's place (sub-pixel),
its measured move since the previous processed frame, the match's score, its
row's time; per 20 × 20 px region the texture, brightness and median move. Nothing
is interpreted: the robot's own turn is for readers to remove (V2's `ego`). Nothing
reads it yet but snapshots and `v`.

- **Finding:** FAST-9 corners in each 20 × 20 px cell with fewer than 2 features
  (the threshold lowered until a few come up, never below 3 × the pixel noise at
  the gain in use), the best by Shi-Tomasi score (7 × 7) at least 6 px from other
  features; at most 96, none within 8 px of the image's edge.
- **Following:** each feature's 8 × 8 template is searched ±8 px around its last
  place moved by its own last measured move, coarse on the half-size image, then at
  full size; the sub-pixel part by a parabola and Lucas-Kanade steps. Taken when
  the ZNCC (blind to gain and offset: exposure steps don't break tracks) is ≥ 0.8
  and clearly better than the second-best place (≥ 0.05: repeated patterns fail);
  a best place on the search window's edge isn't taken. Not found: MISSED (up to 2
  frames in a row), then LOST; one that would leave the image: LOST and
  LEFT_IMAGE. A template is taken again when the feature has moved 1 px from where
  it was taken or matches less than 0.9, so a still scene's features don't wander.
- **Pixel noise per gain** is a guess (1 + 0.15 × gain grey levels) until measured.
- **Cost:** 0.2 ms a frame on the Mac; on the robot `v` says. ~30 KB of RAM.

### Modes (`mode`)

Who chooses the robot's actions. **Manual** (after power-up): the operator, by
commands. **Idle** (after a stop): nobody, the robot stands with the motors off; a
move in idle switches to manual. `mode idle|manual` sets it. The mode changes
nothing else yet; later modes (sentry, explore) will choose actions themselves.

### Snapshots (`snapshot`)

What the robot knows at one moment, sent to the server when a command asks (the
API asks after every move) and when a server connects: the boot id and build (given
by `main.c`: `snapshot_init()`), the pose, the move that ended and how (its end word,
as the answer's), the running move, the mode, the last ToF frame **as a compact frame
packed for the snapshot use** (its pose in it; 277 B), the last camera frame
(160 × 120) and its features, the colour sensor's last reading, the map window (all
three layers, one byte a cell) with its corner, and the whole-house map's dirty
tiles. No sensor geometry: the PC has the boot's mounting. ~30 KB without tiles. Built into one static 38 KB buffer; one asked for while the last
is going out is built when it has gone; without a server none is built. A snapshot
waits up to 0.3 s for `vision` to process the frame it shows, so the features are
that picture's. It goes to the PC in snapshot parts (≤ 1400 B each) on `link_pc`'s
bulk lane; a part the lane has no room for waits, none is dropped; a snapshot cut
by a lost connection goes again whole on the next (also when every part had
reached TCP but the connection went before the snapshot counted as sent). A
command's snapshot is answered once every part has been handed to TCP on a
connection that is still up, so on one connection its answer always follows it.
The one bad case: the connection lost after that, before the parts arrived; the
snapshot is gone, its answer arrives on the next connection, and the server's
runner says "the snapshot wasn't stored". The layout is the `snap_*_t` structs in `snapshot.c` (sections, CRC-32
from `common/crc32.h`; version 2), read by `pc/robot/src/snapshot.ts`, which refuses another version.

### Moves (`move`)

Each action is a **move**, the unit the PC stores (`move.h`). `move` owns its phases;
the action's own code (`behaviour`) runs inside the act phase, given to
`move_start()` as three functions (begin, step, end), so `move` knows no action:
- **start**: the motors on; the PC gets **move start** (move number, command number,
  kind, its parameter, PicoA's time, the still times) and from now until the end
  every ToF frame (`move_tof()`, the map use's compact format, bulk lane) and every
  imu record (`move_imu()`), 10 to an **imu batch** (bulk lane, under the move's
  number). A gap between two records (one lost on the way from PicoB) or a step back
  starts a new batch, so a batch's records are evenly spaced. The robot must stand
  still within 3 s, else the move ends "no_start".
- **pre**: it stands still `pre_ms` (200 ms; `config pre_ms`), motors on, not driving.
- **act**: the action drives until it says done (or done before an obstacle).
- **post**: it stands still `post_ms` (200 ms; `config post_ms`) and until PicoB says
  still (at most 3 s). This is the only settling after an action.
- **end**: the last imu batch goes, however few records it has; the motors off; the
  PC gets **move end**: how it ended (the answer's end word), the frames sent and
  dropped (the bulk lane full, or no server), the first and last frame number sent,
  how far it went and turned, the imu batches sent and dropped. The console says
  "Drive 12: 41 ToF frames sent, 0 dropped; 43 imu batches sent, 0 dropped".

A stop, PicoB lost or PicoB's safety stop ends a move at once, in any phase; a new
move replaces the running one (the motors stay on). A move that can't start (blocked
from the start, a goal already reached) is numbered and sends its start and end at
once, with no phases, frames or imu batches (`move_refused()`; its start says "refused", so the
PC keeps the running move open); a running move goes on. One
command is one move: a go to (turn, then drive) is one move; `robot steer r30 d50`
is two commands, so two moves, each standing still before and after. A watch is a
move like the others. Nothing goes to the PC while no move runs but the console,
status and snapshots. `move_next_ended()` gives each ended move's record, oldest first (commands answer each:
a refused move and the running one can end in the same loop); `move_endings()` the last
one's (the snapshot's head names it); `move_running()` the running move's kind for
the status.

### Actions (`behaviour`)

How each action drives, inside its move (above): one at a time, started by a command;
a new one replaces the running one, `stop` stops it (`move_stop()`). When its move
ends done or before an obstacle it prints what odometry measured. Kinds in the data:
drive, rotate, goto, watch.

- **Drive** (`drive <cm>`): straight, negative = back, at ≤ 10 cm/s, slowing near
  the end. **Stops 15 cm before an obstacle on the map:**
  from the front (back) edge to the nearest blocked or overhang cell on its path
  (the robot's 23.5 cm plus 5 cm each side), checked every loop while driving,
  slowing near the stop point; unknown cells count as free. Within 15 cm it doesn't
  start. Backwards only what the map already has stops it (the VL53 looks forward).
- **Rotate** (`rotate <deg>`): in place, + = left. A scan is `rotate 390` (the map is
  fed from every frame anyway).
- **Watch** (`watch <s>`): turns towards the biggest VL53 movement and follows
  it: its angular speed (a line through the last 0.5 s) plus 2 × the angle still
  to go, at most ~29°/s; starts when it is 8° off, stops within 3° once it is
  about still, never turns past where it was last seen. A one-zone movement at the
  edge of the view is inspected too.
- **Go to** (`goto <x> <y>`, or a click on the page's map): turn in place towards the
  point, then drive straight to it at ≤ 10 cm/s, steering a little to keep it
  ahead, stopping where it is (measured along the way) or 15 cm before an obstacle
  on the map, as Move. At most 3 m at once (a farther goal is gone towards). The
  goal is a point in the map frame; the robot works out the turn and the distance
  from its own pose.

Speeds come from `motion.h`. The decision "one owner of the wheels" isn't built
yet: `behaviour` drives, and `commands` switches the motors off on stop.

### Commands (`commands`)

One command message from the PC each (its number and its text, `drive 50`; from the
server's API, which numbers them), or one line `"<command> [#<n>]"` typed on USB
(for debugging); nothing is done or printed by itself. `drive <cm>`, `rotate
<deg>`, `goto <x cm> <y cm>`, `watch <s>`, `snap`, `clear_map`, `config pre_ms
<ms>` and `config post_ms <ms>` (the moves' still times, 0-2000, from the next move
on), `mode idle|manual`, `stop` (motors off, mode idle), `status` (pose, PicoB link,
WiFi and the lanes, loop time over 10 s, RAM free, mode, snapshots, moves ended and
the still times, imu records from PicoB and those not taken in time: printed). The limits are in `commands.h` (the API
reads them); a command out of range is refused. Each is answered when it has
ended, by an answer message: its number, how it ended (ok, obstacle, stopped,
replaced, tilt, wheels, no_drive, picob_lost, no_start, refused), the pose, and for a
move how far it went and turned, for a go to the goal's distance, for a snapshot
its number; a move is answered from its record (`move_endings()`), each command's
before the next can end another. `clear_map` also sends "map cleared".
`commands_status()` fills the status `link_pc` sends (pose, mode, running move).

### The link to the PC (`link_pc`, `wire`)

**`wire`** (`wire.h`) is the only code that knows the bytes: the envelope
`[type 1 B][length 2 B][payload]` and every payload but the ToF frame's (that is
`tof_format`'s; the table takes the ToF types from `tof_format_message_type()`,
and `wire_check()` finds a collision). Pure functions. Its twin on the PC is
`wire.ts`; both are checked against the same golden bytes
(`picoA/app/test/data/wire_golden.hex`, a compact tof frame at its limits among
them). `WIRE_VERSION` (3) is in hello.

| Type | Message | Lane | Payload |
|---|---|---|---|
| 0x01 | hello | control | wire version, boot id, build (29 B) |
| 0x02 | console | control | one printed line, ≤ 240 B |
| 0x03 | answer | control | command number, end, which fields, x, y, yaw, moved, rotated, goal, snapshot (34 B) |
| 0x04 | status | control | x, y, yaw, mode, the running move, WiFi dBm (33 B) |
| 0x05 | map cleared | control | none |
| 0x06 | move start | control | move n, command n, kind, parameter, t_us, pre_ms, post_ms, flags (refused) (22 B) |
| 0x07 | move end | control | move n, end, t_us, frames sent, dropped, first and last frame n, moved, rotated, imu batches sent, dropped (41 B) |
| 0x10 | snapshot part | bulk | 1-1400 B of a snapshot |
| 0x11 | imu batch | bulk | move n, the first record's t_us, period µs, count; per record accel and gyro x, y, z, int16 counts (11 + 12 × 1-10 B; only during a move) |
| 0x40 | tof compact | bulk | a compact tof frame, the map use's (277 B; only during a move) |
| 0x20 | command | to the robot | number, text (≤ 46 B) |
| 0x21 | heartbeat | to the robot | none |

**`link_pc`** (`link_pc.h`) is the only code that knows lwIP and TCP. It joins the
network in `wifi_config.h` (git-ignored; copy `wifi_config.example.h`), finds the
server by its UDP announcements (or `WIFI_SERVER_IP`), keeps one TCP connection
(port 4211) and reconnects by itself; losing it doesn't stop the robot. Its
interface: `link_send_control()`, `link_send_bulk()` (whole messages from wire's
encoders), `link_received()` (the next command), `link_pc_connection()` (a number
that changes with each connection), `link_bulk_state()` (a bulk message's ticket:
queued, sent, or lost with its connection), and status, which it sends
itself every 0.25 s while connected (filled by `commands_status()`, the WiFi signal
added).

- **Hello first** on each connection, then what waits.
- **Everything printed** goes as console messages, one per line (a longer line as
  several), next to USB, which stays plain text.
- **Control lane** (16 KB): sent first. Never blocks the robot (a blocked loop would
  starve PicoB's drive commands): when full, the oldest console lines and statuses
  go to make room (counted); answers, hello and map cleared never go (if only they
  filled it, a new one would be refused and counted: 442 answers). It keeps
  filling while no server listens (the newest 16 KB) and goes out on the next
  connection; with WiFi off or unavailable (no network set) nothing is kept. A newer status replaces one still waiting.
- **Bulk lane** (40 KB ring): first in, first out; handed to TCP only while control
  is empty, one whole message at a time, with at most ~5.6 KB unacknowledged, so a
  control message waits behind at most that. Droppable messages (tof frames, imu
  batches) are dropped and counted when it is full and never take its last ~1.5 KB,
  so a snapshot part always gets in next; a part that doesn't fit is refused and
  its sender tries again. A lost connection drops what is queued (frames counted).
- **Hello refused by TCP** (out of memory): the connection is closed and the server
  looked for again at once (counted as a write error).
- **From the PC:** commands and a heartbeat every 2 s. Nothing heard for 30 s: the
  connection is closed and the server looked for again; so are bytes that aren't a
  message the robot takes. After 6 s of nothing the server says "nothing from the
  robot for N s; the connection is open" (in the log and the status pill), and
  "data again" when bytes come.
- lwIP runs in the background interrupt (`lwipopts.h`): TCP's send buffer 16
  segments (~23 KB), `MEM_SIZE` 44000, `TCP_SND_QUEUELEN` 64, `MEMP_NUM_TCP_SEG` 66.
- The UDP socket only hears the server's announcements; nothing goes out on it.
- A move's end travels on control and can arrive before its last frames (bulk): it
  carries the first and last frame number, so the PC files late frames under it. An
  imu batch names its move itself. A batch's records are evenly spaced: record i at
  the first's time + i × the period (the encoder takes it from the first and last
  record's times).

### Budgets

- **RAM:** everything is static, no `malloc`. ~51 KB free of 512 (`status` prints it;
  7 Nov, after the snapshot buffer 38 KB, vision ~30 KB, the whole-house map 20 KB,
  the ToF pipeline's frames 1.8 KB, and step 4a's bulk lane 40 KB, lwIP's bigger send
  buffer 12 KB and the snapshot's part buffers 2.8 KB; the recorder's buffers, ~5 KB,
  freed in step 4b; the imu records kept and the batch ~0.8 KB, step 4c). Step 6
  frees ~31 KB.
- **Loop:** mean ~60 µs, worst ~29 ms (a map frame, measured at 15 Hz); a ToF frame every 100 ms, DRIVE
  every 50 ms (PicoB stops after 250 ms), 16 odometry reports kept (320 ms), 32 imu
  records (~300 ms). The UART's receive buffer (1 KB) holds ~200 ms of PicoB's messages.
- **To the PC while moving:** tof frames ~2.8 KB/s, imu batches ~1.4 KB/s (10 a
  second, 134 B each), status ~0.1 KB/s.
- **Stack:** core 0 ~6.4 KB worst case of the 8 KB it can use (both scratch banks;
  `watch_tof_add()` keeps 4.3 KB on the stack, its blobs included; the pipeline's
  frames are static).
  `tools/stack_depth.py` computes it from the call graph.

## 5. The PC (`pc/robot/`)

Node 24, TypeScript run directly; `npm start` serves http://127.0.0.1:8081/.

- **`main.ts`, `link.ts`, `wire.ts`**: the server. `link.ts` announces it on the
  network (or only to `ANNOUNCE_TO`, for a test server), accepts PicoA's connection
  and turns its messages into events: hello (the boot; a robot of another wire
  version is refused, with a log line saying so, once per robot address and
  version), console lines, status (the pose), answers (done records), map cleared,
  move start and end, tof frames, imu batches, whole snapshots (put together from their parts),
  each in the order it came; it sends command messages and a heartbeat every 2 s.
  `wire.ts` is the twin of the robot's `wire` (the bytes), `tof_format.ts` of
  `tof_format` (unpacks a compact frame, and packs it again to the same bytes for
  the fake robot). `main.ts` passes the log, the robot's status, poses and new
  snapshots to the pages (WebSocket, one way). Settings in `.env` (`.env.example`).
- **`boot.ts`**: one power-up of the robot: its id (from hello), **its own copy of
  the mounting** (`common/mounting.json` when the boot was first seen, kept with it,
  so a later calibration doesn't move what it saw), its messages numbered in the
  order they came, and **its clock**: PicoA's 32-bit µs times (wrapping every 71.6
  min, 30 s after power-up first) turned into seconds since power-up that only grow
  (each time follows the one before; how long the PC waited counts the wraps while
  it was away). A tof frame is filed under the move that ended and sent it (by the
  move end's first and last frame number: frames can come after their move's end)
  or else the open move (a refused move never becomes the open one). An imu batch
  names its move itself, so a late one is filed right; each of its records gets its
  time since power-up.
- **`ingest.ts`**: link.ts' events stamped by the boot and written to the store, in
  the order they came; hello opens the boot (the store keeps where it left it, so a
  restarted server goes on; the same boot's hello on a new connection keeps it as it is); a snapshot's picture goes to `recordings/camera/`.
- **`store.ts`**: DuckDB, the one store, **a file of its own**:
  `recordings/robot2.duckdb` (`ROBOT_DB`, `RECORDINGS` in `.env`); the old
  `robot.duckdb` is never opened (`git checkout data-v1` looks at it). Written as each
  message arrives (what comes while a write runs goes in the next): `boots` (build,
  mounting JSON, where its clock and moves were), `moves` (start and end), `frames`
  (one per tof frame: number, move, time, pose, period, turn, flags), `zones` (one per
  zone: row, col 1-8, distance, kind, signal, sigma, targets), `imu` (one per imu
  record: move, its batch's message and place in it, time, `ax`…`gz` the raw counts in
  the sensor's axes; the boot's mounting turns them), `status` (the path driven: pose,
  mode, action, WiFi), `snapshots` (with their bytes), `snapshot_zones` (each also in the world,
  by the boot's mounting), `snapshot_features`, `snapshot_tiles`, the objects' tables.
  Times: `t_us` PicoA's as sent, `t_s` since power-up, `arrived_at` the PC's;
  `msg` the order they came. Opened per operation, so the `duckdb` command line can
  use it in between. Reads: a move's frames, its imu records, a boot's messages for a
  replay, the path. No migration: `robot2.duckdb` is new with this rework (not yet
  written on the robot); a database from before step 4c would need deleting.
- **`api/`, `cli.ts`, `robot`**: the command API under `/api/robot/` (Hono) and the
  `robot` CLI that calls it ([CLI_API_PLAN.md](CLI_API_PLAN.md): the commands,
  answers and the design, module by module). One queue (`api/queue.ts`) for the
  page's and the CLI's commands (steer, snap, clear_map, mode, config pre_ms|post_ms;
  stop skips it), one at a time; anything that doesn't end ok empties it; a snap after
  every move (auto_snap). Every answer is a JSON line with the same fields.
  `api/runner.ts` numbers each command, sends it and waits for its answer;
  `api/commands.ts` is the command language (`robot steer r30d50`); `api/reads.ts`
  the commands that run at once (status, map, tof, camera, light, features, frames,
  imu, log, config) as JSON (`robot frames <boot> <move>` and `robot imu <boot> <move>`
  as text, `--json` for a program: the move, its frames or its imu records, and the
  imu count in both: records stored, missing (two stored records more than 1.5
  periods apart have round(gap / period) − 1 missing between them), batches; `api/frames.ts` formats them); `api/check.ts` `robot check` on the newest snapshot's map;
  `api/firmware.ts` reads the limits and the move rules from the firmware's headers.
- **`log.ts`, `live.ts`**: the one log everyone sees (the robot's text, the server's
  notes, every answer with who sent it), kept for new pages and `robot log` and
  written to a file per run in `logs/`; the robot now (status, pose, the boot, the
  still times of its moves).
- **`public/app.html`, `app.js`**: the viewer and steering page at
  http://127.0.0.1:8081/. A 2D map (x up, y left, as `robot map`): the snapshot's window
  over the whole-house map's tiles as they were then (where the window is unknown
  and a tile knows the column, dimmed; "hide weak" hides columns under 10 %
  confident), the path driven up to the snapshot (from the pose
  lines) and where each snapshot was taken, the snapshot's ToF
  zones (sure and less sure, by height), the robot then and now; a timeline of the boot's snapshots ("live"
  follows the newest; a new boot's first snapshot, sent when the robot connects,
  always takes over the page and ticks "live"); the camera picture with its features
  (the camera's view from the boot's own mounting), the 8 × 8 ToF grid (~ less sure, ?
  unsure, >254 beyond 2.54 m), the
  colour reading as a swatch, the pose; the log; steering through the API: Idle /
  Manual, Stop (and space), drive, rotate and watch with a box for the number,
  snapshot, clear map, status. On the live view a click on the map
  sends the robot there (`g<x>,<y>`). (The old console page, `/console`, went on
  8 Nov: the viewer shows everything it showed.)
- **`snapshot.ts`, `snapshots_api.ts`**: a snapshot read (version 2; the ToF section a
  compact frame), stored with its bytes, each camera picture as
  `recordings/camera/<folder>/<seq>.png` (map pictures from `robot map --png` in
  `recordings/map/<folder>/`). The API composes the map at any snapshot: the newest
  of each tile up to it since the map was last cleared; `/api/mounting?boot=` gives
  the boot's own mounting.
- **`boot_folders.ts`**: the robot names each boot with a random 8-hex boot id (it
  has no clock); the folders under `recordings/` are named by when the PC first saw
  the boot, `YYYYMMDDHHMM` (local). `recordings/boots.json` maps boot id to folder
  name; read once at start, the last boot's folder remembered, written only when a
  new boot appears (named when its first snapshot arrives; a name already taken is
  an error). The database and the API keep the boot id; the page's Boot dropdown
  shows the folder name (`/api/boots` sends it as `folder`; `robot frames` takes
  either);
  `snapshots.image` holds the picture's path, so the page's `/camera/<boot>/<seq>.png`
  reads it from there.
- **`objects/`**: objects on the map from the camera ([OBJECTS_PLAN.md](OBJECTS_PLAN.md)).
  `robot detect [n]` sends the snapshot's picture to a vision model on an
  OpenAI-compatible server (`MODEL_API`, default http://127.0.0.1:8080, llama.cpp's
  router; `MODEL`, default `gemma-4-26b`; loaded on the first request if it isn't;
  thinking at most `REASONING_BUDGET` tokens, or `--reasoning-budget=N`),
  with a system prompt that asks for the bottom edge exactly where the object meets
  the floor (`session.ts`). One prompt per model, picked by `MODEL`'s name:
  `system_prompt_gemma4.md` (boxes y first on 0-1000) and `system_prompt_qwen.md` (x
  first, shown only by its example, in the picture's 160 × 120 pixels). Qwen's boxes
  are turned into Gemma's form as they arrive, so the database and everything after
  see them y first on 0-1000. The model answers by calling `report_objects` with
  `{label, bbox: [ymin, xmin, ymax, xmax]}` on 0-1000; a fixed list of labels
  (`labels.ts`: chair, table, sofa, door, wall, plant, shelf, cabinet, box, object).
  Wrong calls go back with what to fix (3 corrections), no call gets a reminder (3),
  the first empty list is pushed back once. `place.ts` puts each box's bottom edge
  on the floor through the camera (the boot's own copy of the mounting; its numbers in [CAMERA_PARAMS.md](CAMERA_PARAMS.md))
  and the snapshot's pose and pitch: its front edge, and a
  rectangle of the label's depth behind it (doors and walls a line); camera only.
  `db.ts`: `detect_runs`, `detections`, `map_objects`, `object_detections` in
  DuckDB; a new run on a snapshot replaces the old. `robot detect_save <n>
  --by=<name> '<json>'` saves boxes found elsewhere (an outside agent that looked at
  `robot camera n`), the JSON as `report_objects` takes it. `robot detect` saves
  through the same `detectSave` (`api.ts`): checked, placed, saved; `model` is who
  found them (`--by`), and an outside agent leaves prompt version, thinking, seconds,
  turns and the conversation null (a program may send them in the JSON). The CLI
  passes `--name=value` flags as `?name=value`. `robot objects [n]` reads them
  back (`--transcript`: the conversation). `npm run place-objects` places every
  saved box again with its boot's mounting (`-- --from-file [<boot>]` first gives the
  boot, or every boot, today's `mounting.json`: after a calibration). The page draws the boxes on the camera
  picture and the shapes on the map (`/api/snapshots/<boot>/<seq>/objects`;
  `?snapshot=<n>` opens a snapshot).
- **The page without the robot:** `npm run fakerobot -- <boot>` connects to a
  server as the robot does and speaks the wire: it replays the stored boot's
  messages (moves, frames, imu batches, statuses, snapshots, in the order they came; `--speed=`),
  then answers every command (hello, console lines, status, a move start and end
  for each move, map cleared), drives its pose at the robot's speeds, and sends the
  boot's snapshots, in parts, one after another when asked for one (`FakeRobot`,
  also used by `fakerobot.test.ts`). The 3D viewer of takes is gone with the
  recordings (TODO: one on the stored frames).
- `recordings/` is git-ignored.

`pc/bringup/` is the bring-up viewer (camera, ToF, colour sensor over USB) for the
`picoA_bringup` firmware.

## 6. Tests and tools

- `./run_tests.sh`: host tests of every module above on the Mac, with fakes for
  the hardware and a simulated room (`test_behaviour`, `test_cell_map`; frames go
  through packing and unpacking as on the robot), the ToF pipeline's parts
  (`test_tof` against a fake sensor, `test_tof_rays`, `test_tof_format` at its
  limits, the stamp in `test_pose`, `test_watch` end to end), plus the
  map on five recorded scans (`test_map_replay`: the cup, box, chair and cupboard
  must be found; one target per zone, `frames_file.h`; data in `picoA/app/test/data/`), the window following the robot down
  a simulated hallway, camera features on synthetic frames (`test_vision`), and the
  snapshots as the PC reads them (`test_snapshot`, whose file the PC's test reads,
  and the map's obstacle check on it, which the PC's copy must match; its parts
  waiting for room and sent again whole after a lost connection), the wire
  (`test_wire`: every message, the envelope's errors, the golden bytes
  `picoA/app/test/data/wire_golden.hex` that `wire.test.ts` checks too), the link to
  the PC (`test_link_pc`, against a fake WiFi chip and lwIP: hello first, the lanes'
  order, drops and counts, console under load, reconnecting) and the commands over
  it (`test_commands`: answers, map cleared, config pre_ms / post_ms, status), a move's
  phases and messages (`test_move`: pre and post with the fake clock, frames in start,
  act and post, still before the
  motors drive, post the only settling, stops in pre and post, a watch as a move,
  frames only during a move, dropped and counted; imu batches of 10, the last at the
  end, a gap starting a new one, dropped and counted; `test_behaviour` runs every action
  through the real `move`), PicoB's imu records (`test_odometry`: averaging, rounding,
  the int16 limits, 104 a second, time stamps across the wrap; `test_brain`: sent as
  made, only once greeted) and their way through `body` (`test_body`: PicoA's clock
  within 0.1 ms, across both clocks' wraps); `test_imu_path` runs the whole way
  end to end (PicoB's real odometry and brain in a file of their own, the link
  messages passed over, PicoA's real body, pipeline, move and wire): every record of
  a move once, in order, unchanged, within 0.5 ms on PicoA's clock across its wrap;
  `link_test` also overruns the receive buffer (counted lost). First it
  makes `build/host/robot_mounting.h` from `common/mounting.json` and checks it
  holds the file's numbers (`test_mounting`) and that a missing field stops it;
  the simulated rooms use the robot's mounting, the recorded scans the one from
  before the lift.
- `npm test` and `npm run typecheck` in `pc/robot/`: also the API (the steering
  language, the queue and runner, `robot check`, the reads and the routes end to end),
  the wire and the compact tof frame against the golden bytes, `link.ts` with a robot
  played on localhost (TCP chunks split and merged, moves and frames in order,
  another wire version refused), `boot.ts`' clock across the wrap (from 30 s before
  it), the store (moves, frames, zones, imu records written and read, late frames and
  imu batches under their move, each boot's own mounting, a restarted server, the old
  database never opened), `robot frames` and `robot imu` as text and JSON (dropped and
  lost frames and batches), `ingest.ts` end to end (a refused move during a running
  one, the same boot's second hello, a late imu batch), and the fake robot end to end
  with the link and the runner, and replaying a stored boot (imu batches too) into a
  second database.
- `tools/lint.sh`: cppcheck, ESLint, tsc.
- `tools/stack_depth.py`: worst-case stack from GCC's call graph.

## 7. Decisions (Daniel)

Rules the code follows; change them only with Daniel's word.

**The robot:**
- Robot height 13 cm (measured 8 Nov, after the PCB was raised), no extra clearance margin. Footprint 23.5 cm wide × 19 cm
  long, front edge 9.5 cm ahead of the centre.
- The ground band is −7…+3 cm: anything under 3 cm is ground and driven over.
  Everything above is an obstacle. **No floor learning.**
- Drivable = floor seen in the ground cell, 3-13 cm and 13-23 cm free. L2 cells
  within 25 cm of the robot, never seen by any zone, count as free.
- The map: 9 rays per zone, the best measurement wins; a reading judged as a whole,
  and a detection marks every cell its rays end in, as sure as the whole reading.
  Not: adding up evidence over frames or over rays, one ray per zone.
- No climbing worries for now: the robot drives slowly in controlled rooms.

**Commands:**
- **The robot does nothing by itself:** manual with the motors off after power-up
  and after a reset, no start-up scan.
- Modes: idle and manual; set, not switched. A click on the map steers only on the
  live view.
- The commands and decisions of 14 Oct: [CLI_API_PLAN.md](CLI_API_PLAN.md). One
  command, one operation, each with its number required; no test-only actions.
- Motors off at the end of every action; stop stops (and empties the queue).
- Commands come only from the API (USB typing is for debugging).
- drive stops 15 cm before an obstacle on the map (blocked and overhang cells),
  unknown cells count as free. Its braking and brief speed-up when a closer reading
  moves the stop point back are fine as they are.

**Movement and watching:**
- Movement is detected while still and while turning, against a background kept
  per world direction; no learning after a stop.
- Watching follows the biggest movement: no predicting where it goes, no turning
  ahead of it; its measured angular speed may be used, never overshoot.
- Movement at the edge of the view is inspected (a one-zone blob turns the robot).

**Data:**
- After every move the API asks for a snapshot, which goes to the PC over TCP (14 Oct): pose, camera
  picture and features, one VL53 frame (compact), the colour reading, the map window
  with its anchor and the changed tiles.
- Data flow (7 Nov, [DATA_FLOW_PLAN.md](DATA_FLOW_PLAN.md)): the robot sends what it
  measures only during moves (each still 200 ms before and after), packed small, on
  the one TCP connection; DuckDB is the one store, no raw files; a new data format
  gets a new database file. Hypotheses and map changes are tested on what is stored.
- The map window is the truth over the whole-house map where it knows something;
  unknown keeps the old column. Both the robot and the PC follow this rule.
- Recordings stay out of git.
- Camera pictures go to the server at 160 × 120, never larger (in snapshots); the
  server does all image recognition (not built yet). Camera
  features go to the server too and are not kept on the robot; the camera's movement
  detection will be one reader of a feature record
  ([archive/CAMERA_MOTION_PLAN.md](archive/CAMERA_MOTION_PLAN.md)).
- Camera while watching: sharp and dark rather than noisy (exposure ≤ 10 ms, gain
  ≤ 8). For the camera work; the driver doesn't cap it yet.
- Frames dropped during a move are counted per move (move end); none should be.
