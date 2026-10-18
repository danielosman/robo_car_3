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
  the OPT4048 colour sensor, WiFi. It keeps the map, detects movement, finds and
  follows camera features, runs the commands the server's API sends (one line
  each, answered when it has ended), prints its console on USB and over WiFi, and
  sends a snapshot of what it knows when a command asks.
- **PicoB** (Pico 2, the body): four wheels with encoders and the IMU. It drives at
  the speeds PicoA commands, reports odometry and tilt, and stops the motors by
  itself when something is wrong.
- **The PC** (`pc/robot`, Node): the command API with its one queue, which the
  `robot` CLI (an agent's or a person's) and the viewer and steering page both
  use; the page (the robot's map at any snapshot with the path driven, its camera
  picture with features, ToF, colour, pose; the log; steering); the snapshots, pose
  lines and recordings in DuckDB; and the 3D viewer of recorded takes, which
  replays the robot's map code on them.

The two Picos talk over a UART link (1 Mbaud). PicoA talks to the PC over WiFi: a
TCP connection for the console and the snapshots, UDP datagrams for recordings.

**The robot does nothing by itself.** After power-up it is in manual mode with the
motors off, joins the WiFi and waits for a command. Each command does one thing. A
mode says who chooses the actions: idle (nobody) or manual (the operator, by
commands: from the CLI, or the page's buttons and clicks on its map).

### PicoA's main loop (`picoA/app/main.c`)

One superloop on core 0, no RTOS, nothing allocated at run time. Each iteration
runs, in order: `body` (the link to PicoB), `pose`, `recorder` (odometry and drive
commands), `surroundings` (a new ToF frame into the map), `camera`,
`motion_sense` (movement detection; it takes the camera's frames), `vision` (camera
features, every 3rd frame), `behaviour` (the running action), the colour sensor
(asked every 0.1 s), `snapshot`, `wifi_console`, `commands` (the server's command
lines, and their answers). `loop_stats` times every stage. Core 1 is not used.

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
  and the gyro bias is measured again, so standing costs no drift. Keeps the
  largest jolt (acceleration with gravity taken out, along x, y, z) and tilt since
  the last report, for setting bump and tilt stops from measurements later.
- **`brain`**: PicoA as PicoB sees it. Answers PicoA's greeting, keeps the motors
  off until the protocol versions match, applies MOTORS and DRIVE, sends ODOM
  (50 Hz, with the jolt and tilt peaks) and status (2 Hz). **Safety stops:** no DRIVE for 250 ms, or a fault from
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
- **`rangefinder`**: the VL53L8CX as 64 rays (8 × 8 zones, 5.625° each, 15 Hz;
  its numbers in [TOF_PARAMS.md](TOF_PARAMS.md)). It
  hides the zone order and the mounting: turned 90° on the PCB, 3.2 cm ahead of
  the centre, 2.6 cm right, 7 cm above the floor. Per zone it keeps the nearest sure
  target (status 5, 6 or 9), or "nothing in range", or "unsure"; and the next sure
  target behind it (the wall behind a leg). Each with its own status (the sensor's
  echo at ~30 cm often comes first, with status 12 or 14: it is skipped). It gives each
  ray's direction, also for points inside a zone and with the robot's pitch.
- **Drivers:** `tof` (the ST ULD over SPI0, all four targets per zone), `camera`
  (HM0360, 160 × 120 monochrome, subsampled by 4, no binning; its numbers in
  [CAMERA_PARAMS.md](CAMERA_PARAMS.md); captured by PIO and DMA in the background, its
  own exposure control in 10 ms steps against mains flicker, each row's time for
  the rolling shutter; `camera_last_frame()` gives other readers the frame
  `motion_sense` took), `opt4048` (the colour sensor on I2C0: coherent readings of
  its four channels' raw registers, ~2.5 a second; converted to lux and colour on
  the PC).

### The map (`surroundings`, `cell_map`)

`surroundings` feeds every ToF frame to `cell_map`, placed where the robot was when
the frame was measured. Nothing is learned first; `C` clears the map.

- **Cells** are 10 cm cubes in a 6 × 6 m window that follows the robot (below),
  in three layers: **G** (ground, −7…+3 cm), **L1** (3-13 cm: what blocks the
  12 cm robot), **L2** (13-23 cm: overhangs). A cell is unknown, free or occupied,
  with the confidence of the measurement that set it and a doubt counter (1 byte).
- **Rays:** each zone shoots 9 rays (at 1/6, 1/2, 5/6 of its width and height), as
  long as its reading. A ray **passes** the cells it crosses and **ends** in one.
  A G cell is passed only by a ray leaving through its bottom (over a hole or a
  drop). A zone with nothing in range passes cells up to 1 m.
- **A reading is judged as a whole** by where its zone's cone first meets the
  floor (row 8 ~18 cm, row 7 ~24, row 6 ~34, row 5 ~63-71 cm, rows 1-4 never):
  - closer than 0.9 × that: an **obstacle**, a **detection**. Which of the cells its
    9 rays end in holds it isn't known, so all of them are marked occupied (in L1
    or above: rays ending below 3 cm are lifted to L1), each as sure as the whole
    reading, however its rays happen to split between cells. The zone's second
    target is judged and marked the same way;
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
  text, for the tests and `tools/replay.sh`). Outside the window they answer from
  the whole-house map's columns.
- **The window follows the robot** (`cell_map_follow()`, before every frame): it is
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
- **Cost:** the worst frame ~27 ms on the robot (a frame comes every 66 ms).

**Known limits:** under a table top ~0.8 m away a few false blocked columns
(driving closer clears them); a small hole next to floor gets some floor hits;
standing still the floor is seen only in rings (~20, 30, 45, 70 cm); far walls
(beyond ~2 m) are weak and patchy; roll is not used (turning, the robot rolls
3-5°); a single echo can make a one-frame
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
API asks after every move) and when a server connects: the pose, the action that ended
and how, the running action, the mode, the last ToF frame with all its targets and
where the robot was for it, the sensor's geometry, the last camera frame (160 × 120)
and its features, the colour sensor's last reading, the map window (all three
layers, one byte a cell) with its corner, and the whole-house map's dirty tiles. ~31
KB without tiles. Built into one static 38 KB buffer; one asked for while the last
is going out is built when it has gone; without a server none is built. A snapshot
waits up to 0.3 s for `vision` to process the frame it shows, so the features are
that picture's. A command's snapshot is answered once it has wholly gone to the
connection, so it comes before its answer in the stream. Every 0.25 s while a
server listens, the pose line, `"\001pose x y yaw mode action wifi recording jx jy
jz tilt"`, says where the robot is now, with the largest jolt and tilt PicoB
measured since the line before. The layout is the `snap_*_t` structs in
`snapshot.c` (sections, CRC-32), read by `pc/robot/src/snapshot.ts`.

### Actions (`behaviour`)

One action at a time, started by a command; a new one replaces the running one,
stop stops it. Each switches the motors on at its start and off at its end, prints
how it ended, and ends exactly once, carrying its command's number, also when it
can't start (a move blocked from the start, a goal already reached): done (after
standing still, with what odometry measured), stopped, replaced, safety stop on
PicoB, PicoB lost, didn't start, or obstacle.

- **Move** (`drive <cm>`): straight, negative = back, at ≤ 10 cm/s, slowing near
  the end. **Stops 15 cm before an obstacle on the map:**
  from the front (back) edge to the nearest blocked or overhang cell on its path
  (the robot's 23.5 cm plus 5 cm each side), checked every loop while driving,
  slowing near the stop point; unknown cells count as free. Within 15 cm it doesn't
  start. Backwards only what the map already has stops it (the VL53 looks forward).
- **Turn** (`rotate <deg>`): in place, + = left. A scan is `rotate 390` (the map is
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

### Commands (`commands`) and the console (`wifi_console`)

One command per line, `"<command> [#<n>]"`, the same over WiFi (from the server's
API, which numbers them) and typed on USB (for debugging); nothing is done or
printed by itself. `drive <cm>`, `rotate <deg>`, `goto <x cm> <y cm>`, `watch <s>`,
`snapshot`, `clear_map`, `recording on|off` (full telemetry, off at power-up),
`mode idle|manual`, `stop` (motors off, mode idle), `status` (pose, PicoB link,
WiFi, loop time over 10 s, RAM free, mode, snapshots, recording: printed). The
limits are in `commands.h` (the API reads them); a command out of range is refused.
Each is answered when it has ended, by a done record to the server: `"\001done
n=<n> end=<ok|obstacle|stopped|tilt|wheels|…|refused> x=<cm> y=<cm> yaw=<deg>"`,
for a move also `moved=<cm>` (and `goal=<cm>` for a go to), for a snapshot
`snapshot=<its number>`.

`wifi_console` joins the network in `wifi_config.h` (git-ignored; copy
`wifi_config.example.h`), finds the server by its UDP announcements (or
`WIFI_SERVER_IP`), sends everything printed over TCP (port 4211) and takes commands;
it keeps the last 16 KB while no server listens and reconnects by itself. Losing
it doesn't stop the robot. Snapshots go on the same connection, between the text,
in chunks `[0x02][length u16][bytes]` (the text goes first), at most two chunks
unacknowledged at a time: a TCP stream is one queue, so whatever is written later
waits behind them. A snapshot cut by a lost connection is sent again from its
start on the next. The pose line and the done records (`"\001…\n"`) go to the
server only; the pose line (4 a second) is what tells it the robot is there;
there is no separate heartbeat. The server writes `'\0'` every
2 s. Either side closes the connection only after 30 s with nothing from the
other; after 6 s the server says "nothing from the robot for N s; the connection
is open" (in the log and the status pill), and "data again" when bytes come. lwIP
runs in the background interrupt (`lwipopts.h`).

### Recording (`recorder`)

With recording on (`recording on`), **each action is a take**: from its start to its end the
raw data goes to the PC as UDP datagrams (port 4212, ≤ 1400 B each, numbered):
`TAKE_START`, `GEOMETRY` (the sensor mounting, so the PC hard-codes nothing),
every ToF frame with all of the sensor's outputs (`TOF_RAW`, in two halves),
every odometry report (since link version 5 with PicoB's jolt and tilt peaks; the
PC tells the two sizes apart), every drive command, the console's lines (`MARK`), the WiFi state every 0.5 s, and `TAKE_END` (how it ended, what was sent
and refused). Idle, nothing is sent. A lost datagram is lost: nothing waits behind
it, the PC counts the gaps by number; each datagram repeats the previous one's
odometry, and the first and last datagrams go more than once. The record layouts
are the `rec_*_t` structs in `recorder.c` (format version 3); the PC reads them in
`recording.ts` `parseRecord()`.

### Budgets

- **RAM:** everything is static, no `malloc`. ~103 KB free of 512 (`p` prints it;
  12 Oct, after the snapshot buffer 38 KB, vision ~30 KB, the whole-house map 20 KB).
- **Loop:** mean ~60 µs, worst ~29 ms (a map frame); a ToF frame every 66 ms, DRIVE
  every 50 ms (PicoB stops after 250 ms), 16 odometry reports kept (320 ms).
- **Stack:** core 0 ~6.5 KB worst case of the 8 KB it can use (both scratch banks;
  `observations()` in `tof_motion.c` keeps 3.2 KB on the stack).
  `tools/stack_depth.py` computes it from the call graph.

## 5. The PC (`pc/robot/`)

Node 24, TypeScript run directly; `npm start` serves http://127.0.0.1:8081/.

- **`main.ts`, `robot.ts`**: the server. Announces itself on the network (or only to
  `ANNOUNCE_TO`, for a test server), accepts PicoA's console connection, takes the
  snapshot chunks, pose lines and done records out of its stream, sends command
  lines back, and passes the log, the robot's status, poses and new snapshots to
  the pages (WebSocket, one way).
- **`api/`, `cli.ts`, `robot`**: the command API under `/api/robot/` (Hono) and the
  `robot` CLI that calls it ([CLI_API_PLAN.md](CLI_API_PLAN.md): the commands,
  answers and the design, module by module). One queue (`api/queue.ts`) for the
  page's and the CLI's commands (steer, snap, clear_map, recording, mode; stop
  skips it), one at a time; anything that doesn't end ok empties it; a snap after
  every move (auto_snap). Every answer is a JSON line with the same fields.
  `api/runner.ts` numbers each command, sends it and waits for its done record;
  `api/commands.ts` is the command language (`robot steer r30d50`); `api/reads.ts`
  the commands that run at once (status, map, tof, camera, light, features, log,
  config) as JSON; `api/check.ts` `robot check` on the newest snapshot's map;
  `api/firmware.ts` reads the limits and the move rules from the firmware's headers.
- **`log.ts`, `live.ts`**: the one log everyone sees (the robot's text, the server's
  notes, every answer with who sent it), kept for new pages and `robot log` and
  written to a file per run in `logs/`; the robot now (status, pose, the boot, the
  largest jolt and tilt since power-up), and each pose line into DuckDB (`poses`)
  with its boot: the path driven.
- **`public/app.html`, `app.js`**: the viewer and steering page at
  http://127.0.0.1:8081/. A 2D map (x up, y left, as `robot map`): the snapshot's window
  over the whole-house map's tiles as they were then (where the window is unknown
  and a tile knows the column, dimmed; "hide weak" hides columns under 10 %
  confident, as the 3D viewer), the path driven up to the snapshot (from the pose
  lines) and where each snapshot was taken, the snapshot's ToF
  targets, the robot then and now; a timeline of the boot's snapshots ("live"
  follows the newest; a new boot's first snapshot, sent when the robot connects,
  always takes over the page and ticks "live"); the camera picture with its features, the 8 × 8 ToF grid, the
  colour reading as a swatch, the pose; the log; steering through the API: Idle /
  Manual, Stop (and space), drive, rotate and watch with a box for the number,
  snapshot, clear map, recording, status. On the live view a click on the map
  sends the robot there (`g<x>,<y>`). The old console page, `/console`, still sends
  keys nothing takes (CLI_API_PLAN.md, gaps).
- **`snapshot.ts`, `snapshots_api.ts`**: each boot's snapshots as they arrive in
  `recordings/snapshots/raw/<time>-boot-<id>.snap` (the truth), each camera picture
  as `recordings/camera/<folder>/<seq>.png` (map pictures from `robot map --png` in
  `recordings/map/<folder>/`), and in DuckDB: `snapshots` (one row each),
  `snapshot_targets`, `snapshot_features`, `snapshot_tiles`, the window as a BLOB.
  The API composes the map at any snapshot: the newest of each tile up to it since
  the map was last cleared.
- **`boot_folders.ts`**: the robot names each boot with a random 8-hex boot id (it
  has no clock); the folders under `recordings/` are named by when the PC first saw
  the boot, `YYYYMMDDHHMM` (local). `recordings/boots.json` maps boot id to folder
  name; read once at start, the last boot's folder remembered, written only when a
  new boot appears (named when its first snapshot or recording arrives; a name
  already taken is an error). The database and the API keep the boot id; the page's
  Boot dropdown, the takes page and the 3D viewer show the folder name (`/api/boots`
  and `/api/takes` send it as `folder`; a take as `<folder>-<n>`);
  `snapshots.image` holds the picture's path, so the page's `/camera/<boot>/<seq>.png`
  reads it from there.
- **`recording.ts`**: receives the datagrams, saves each robot boot's as they
  arrive in `recordings/raw/<time>-boot-<id>.rec` (never changed: the truth), and
  notes each take as it ends (frames whole / half / missing, datagrams lost).
  `npm run decode -- <file>` prints a file's takes.
- **`store.ts`**: each take and snapshot goes into DuckDB (`recordings/robot.duckdb`, derived
  from the raw files; `npm run import` rebuilds it): takes, ToF frames, zones,
  targets (also in world x, y, z), odometry, drive, marks, WiFi, and truth boxes.
  The database is opened per operation, so the `duckdb` command line can use it in
  between. The take list: http://127.0.0.1:8081/takes (`public/takes.js`).
- **`viewer_api.ts`, `public/viewer.js`**: http://127.0.0.1:8081/viewer, three.js
  served locally. A take, or a boot's whole drive (every take since a power-up on
  one timeline): the targets as points (coloured by row, signal, status, height),
  the robot's path and rays on a time slider, the replayed map as cubes, truth
  boxes placed by hand and the map's score against them.
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
  on the floor through the camera (its numbers in [CAMERA_PARAMS.md](CAMERA_PARAMS.md))
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
  saved box again with the camera's numbers now (after a calibration). The page draws the boxes on the camera
  picture and the shapes on the map (`/api/snapshots/<boot>/<seq>/objects`;
  `?snapshot=<n>` opens a snapshot).
- **`replay.ts`, `tools/replay_map.c`**: the robot's own `cell_map.c`, built for
  the Mac, run on a take's frames (`npm run frames` exports them;
  `tools/replay.sh <take>` prints the map). The floor margin can be changed to try
  other values.
- **Trying the viewer without the robot:** `npm run snapreplay -- <boot>` replays a
  recorded drive's ToF frames through the robot's own map and snapshot code
  (`tools/replay_snapshots.c`) into snapshots (no pictures); `npm run fakerobot --
  <file.snap>` connects to the server as the robot does, answers every command
  with a done record, drives its pose at the robot's speeds, and sends the file's
  snapshots one after another when asked for one.
- `recordings/` is git-ignored.

`pc/bringup/` is the bring-up viewer (camera, ToF, colour sensor over USB) for the
`picoA_bringup` firmware.

## 6. Tests and tools

- `./run_tests.sh`: host tests of every module above on the Mac, with fakes for
  the hardware and a simulated room (`test_behaviour`, `test_cell_map`), plus the
  map on five recorded scans (`test_map_replay`: the cup, box, chair and cupboard
  must be found; data in `picoA/app/test/data/`), the window following the robot down
  a simulated hallway, camera features on synthetic frames (`test_vision`), and the
  snapshots as the PC reads them (`test_snapshot`, whose file the PC's test reads,
  and the map's obstacle check on it, which the PC's copy must match).
- `npm test` and `npm run typecheck` in `pc/robot/`: also the API (the steering
  language, the queue and runner, `robot check`, the reads and the routes end to end).
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
  picture and features, one full VL53 frame, the colour reading, the map window
  with its anchor and the changed tiles. The full telemetry (raw sensor data and
  the pose while an action runs, one take per action, UDP) is a separate switch,
  off at power-up. Hypotheses and map changes are tested on recordings.
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
- Datagram loss of up to 20 % per take is not acceptable (resending: TODO.md).
