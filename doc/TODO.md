# RoboCar — what is left to do

Open work only (14 Oct 2026). What is built is in [ARCHITECTURE.md](ARCHITECTURE.md);
the old plans with the details are in [archive/](archive/). An idea needs Daniel's
yes before it is built. When something is done, it leaves this file and goes into
the [CHANGELOG](CHANGELOG.md).

This file is in four parts. **Next** is the order of work agreed with Daniel.
**Goals** lists the robot's big modes, each with the tasks it needs. **Tasks**
describes every task once, the ones that most modes need first. **Small** lists
fixes to what is already built. A task keeps the same name everywhere it appears,
so the count shows what to build first.

## Next

The order agreed on 12 Oct was built the same day (host tests, the fake robot and
a replayed drive; [CHANGELOG](CHANGELOG.md)). What is left of it is the robot:

1. **Robot tests** of the new work, in the README's order: "Snapshots and the
   viewer", "Manual mode, go to a point", "The map follows the robot", "Thin legs
   on the map", "A slow link", "Camera features" (its frames/s and ms per frame decide whether
   V3, core 1, comes soon). Until they pass, these stay here:
   - Snapshots and the viewer (with the colour sensor now read by the app)
   - Modes idle and manual (the start of Mode switching)
   - Map follows the robot, and the whole-house map
   - Drive to a point (go to, click on the map)
   - Camera features V1 (on core 0, every 3rd frame)
   - Thin legs on the map (13 Oct: detections marked whole, passes can't erase
     them in the same frame, the second target). Two drives the same evening
     (boots 31bcd26e, 5b0d72ab): small obstacles and legs found, the ground solid
     after the floor fix. Left of the README test: steps 1-2 (`f` at a single leg,
     on and off the middle line) and step 4 (the chair taken away).
   - A slow link (13 Oct: the server closes only after 30 s, snapshots paced).
     Drive 5b0d72ab had no drops and no "nothing from the robot" notes; the
     weak-signal spot (step 2) and PicoA switched off (step 3) not tried yet.
2. Then Daniel picks the next task from the list below.

**Built 17 Oct, waiting for Daniel's look:** objects from the camera, first step
([OBJECTS_PLAN.md](OBJECTS_PLAN.md)): `robot detect` / `robot objects` and the
overlays; the README's "Objects from the camera" test. Camera calibrated (f 180 px,
tilt −1.8°). Next: the plan's "Next phase" (bottom edge from the pixels, ToF
distance and "ToF verified", the tilt calibrating itself). Flash PicoA: the ToF's
position changed (3.2 cm ahead, 2.6 cm right).

**Built 14 Oct, waiting for its robot test:** the command API and the `robot` CLI
([CLI_API_PLAN.md](CLI_API_PLAN.md)). Flash both Picos (the link version went up),
then the README's "Steering through the API" test. Left of the plan (CLI_API_PLAN.md, "To do"):
   - The map as text is hard to read: ideas to discuss. The map PNG: the whole map
     with every row and column labelled in cm.
   - The camera features' direction in degrees (`robot features` gives pixels): the
     camera's field of view isn't measured or in the code yet.
   - Safety step 2: the bump stop and the 10° tilt stop, from the jolt and tilt
     numbers the pose lines now store (`poses` in DuckDB) once a few drives show
     what normal looks like.
   - The old console page (`/console`) sends keys nothing takes: it goes, or moves
     to the API.

## Goals: what the robot is for

Modes 1-3 are Daniel's; 4-6 were proposed and agreed on 11 Oct; 7 on 12 Oct.

### 1. Watch movement: monitor or investigate

Find the biggest movement and either follow it from where the robot stands
(monitor) or drive to it (investigate): turn to its bearing, drive towards it,
stop 50 cm away, and if it left the view, turn past the edge of the view on that
side. Then face open space. A picture of the place goes to the server. Part of
this is built already: the Watch action (`a`) turns towards and follows the
biggest VL53 movement.

- Mode switching
- Single wheel owner
- Movement target
- Camera tracking
- Camera features
- Corridor query
- Server vision
- VL53 detector by sigma
- ToF confidence model
- Moving away is movement (to discuss)
- Slow glance (not answered)

### 2. Explore and patrol

Drive through all the free space and fill in every blank in the map, so that the
robot knows where it can drive. Take pictures at interesting places, such as where
movement was seen and the key places of each room, so that the server can
catalogue them. A patrol is an exploration in which part of the map is marked as
out of date and explored again.

- Mode switching
- Single wheel owner
- Movement target
- Corridor query
- Server vision
- Watchdog
- Battery watch
- Drift correction
- Camera features
- Features on the server
- Full-turn heading check
- Path planning
- Places catalogue
- Server commands
- ToF confidence model
- Frontier search
- Map age and change events

### 3. Run away

When something moves towards the robot, it drives away from it to the most open
side. This mode is for fun. It needs only the map around the robot, to choose which
way to run.

- Mode switching
- Single wheel owner
- Movement target
- Camera tracking
- Camera features
- Corridor query
- Movement while driving

### 4. Go to a place

An operator says "go to the kitchen". The server finds the kitchen in its catalogue
of pictures and sends the robot a goal (a point on the map, or a list of moves) and
a mode to switch to on arrival, such as monitor. The robot sends a picture when it
arrives, so the server can check that it is in the right place. "Go home", back to
the power-up point, is the simplest case and needs no server.

- Mode switching
- Single wheel owner
- Corridor query
- Server vision
- Watchdog
- Battery watch
- Drift correction
- Camera features
- Features on the server
- Full-turn heading check
- Path planning
- Places catalogue
- Server commands

### 5. Follow me

The robot follows a moving person and keeps about 1 m behind them. This is the
investigate mode in which the target keeps moving, so it is cheap once mode 1
drives.

- Mode switching
- Single wheel owner
- Movement target
- Camera tracking
- Camera features
- Corridor query
- Movement while driving

### 6. Sentry (the easiest)

The robot stands still for hours, at the power-up point or at a place it was sent
to. When something moves, it turns to the movement and sends a picture, and the
server decides whether to raise an alarm. Because the robot does not drive, this
mode is mostly built already.

- Mode switching
- Movement target
- Camera tracking
- Camera features
- Server vision
- Watchdog
- Battery watch
- VL53 detector by sigma

### 7. Manual steering

Daniel steers the robot from the viewer: by keys as today, or by clicking a place
on the map, which makes the robot turn towards it and drive there (stopping 15 cm
before an obstacle). He sees after each action where the robot is, its map, the
camera picture and the other data, and can go back to any earlier moment. This is
the first mode to build, together with idle.

- Mode switching
- Single wheel owner
- Camera features

## Tasks, most needed first

| Task | Modes | Needed by |
|---|---|---|
| Mode switching | 7 | 1 2 3 4 5 6 7 |
| Camera features | 7 | 1 2 3 4 5 6 7 |
| Single wheel owner | 6 | 1 2 3 4 5 7 |
| Movement target | 5 | 1 2 3 5 6 |
| Corridor query | 5 | 1 2 3 4 5 |
| Server vision | 4 | 1 2 4 6 |
| Camera tracking | 4 | 1 3 5 6 |
| Watchdog | 3 | 2 4 6 |
| Battery watch | 3 | 2 4 6 |
| Drift correction | 2 | 2 4 |
| Features on the server | 2 | 2 4 |
| Full-turn heading check | 2 | 2 4 |
| Path planning | 2 | 2 4 |
| Places catalogue | 2 | 2 4 |
| Server commands | 2 | 2 4 |
| Movement while driving | 2 | 3 5 |
| VL53 detector by sigma | 2 | 1 6 |
| ToF confidence model | 2 | 1 2 |
| Frontier search | 1 | 2 |
| Map age and change events | 1 | 2 |
| Moving away is movement | 1 | 1 |
| Slow glance | 1 | 1 |

### Mode switching (7)

Idle and manual are built (`mode`, 12 Oct). Left: a mode that chooses actions by
itself until it is stopped, for example scan, then investigate, then face open
space. The order of needs and the Safety guard
are in [archive/ROBOT_PLAN.md](archive/ROBOT_PLAN.md) §7.3. The operator, or later
the server, starts and stops modes, so the rule "the robot does nothing by itself"
still holds after a power-up.

### Camera features (7)

V1 is built (12 Oct): features found and followed on core 0, every 3rd frame, in
snapshots. Left: V2 (`ego`: the robot's turn predicted and removed, the search
hint, calibration) and V3 (core 1, every frame). The goal: features in every
camera frame, on core 1. Two tasks read them. Camera tracking follows them from frame
to frame to detect movers. Drift correction sends them, with the pose, to the
server and matches them against the features the server saw there before. The
snapshot carries the last frame's features. Before core 1 is used, both cores need
explicit 8 KB stacks, because core 1's stack sits where core 0's can grow today
([archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) V3). Plan:
[archive/CAMERA_MOTION_PLAN.md](archive/CAMERA_MOTION_PLAN.md); V1 (found and
tracked while still, on core 0) first.

### Single wheel owner (6)

Only one module drives the wheels, and every command passes a safety check. Today
`behaviour` drives and the console switches the motors off when space is pressed.
Details: [archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) A5.

### Movement target (5)

One `movement` module combines the VL53 and the camera into one target and its
events. With it, the tracker is either removed or becomes that target. Details:
[archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) A6 and
[archive/TOF_MOTION_PLAN.md](archive/TOF_MOTION_PLAN.md) T4. Robot test: 10
walk-pasts × 2 distances × 2 ways, with the misses counted.

### Corridor query (5)

The map answers how far the robot can drive along a given bearing (drivable or
open), and which bearings are open. Investigate, run away and follow use it to
choose a direction, and all modes use it to face open space. Details:
[archive/SURROUNDINGS_PLAN.md](archive/SURROUNDINGS_PLAN.md) S4.

### Server vision (4)

The PC passes each snapshot's picture to an image model, which decides for example
whether it shows a person lying on the floor and should raise an alarm, or which
room it shows. The robot itself does no image recognition.

### Camera tracking (4)

Track the camera features from frame to frame, take the robot's own motion from
the gyro, and detect movers first while still and then while turning, on core 1.
This replaces `camera_motion`, which only watches while still and learns for 1 s
after each stop. Milestones V1-V5:
[archive/CAMERA_MOTION_PLAN.md](archive/CAMERA_MOTION_PLAN.md).

### Watchdog (3)

Modes that run unattended for a long time need printing that never blocks the
loop (printing the map stops the robot today), a watchdog and a crash-loop guard.
Details: [archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) A3.

### Battery watch (3)

Once the BT_SNS wire is fitted, stop below 6.4 V and switch the motors off below
6.0 V. Later this could include "go home" when the battery is low.

### Drift correction (2)

Correct drift in heading and position over minutes. There are two sources: the
ToF against the map ([archive/ROBOT_PLAN.md](archive/ROBOT_PLAN.md) §5.5), and the
camera features against the ones the server saw near the same pose before (see
"Features on the server"). It could later become a Kalman filter (x, y, heading)
inside `pose`, tuned from logs. First measure the drift:
- From snapshots: compare two windows from different times where they overlap; the
  shift that lines up the same wall best is the drift between them. This needs no
  full telemetry.
- In the whole 11 Oct drive (boot 54564d43) in the viewer: how far odometry drifted
  between the first and the last scan (a wall seen twice), and which cells are
  false.

### Features on the server (2)

The robot keeps no features of its own. It sends each frame's camera features with
the pose to the server, which stores them. When the robot wants to correct its
pose, it sends the server its current pose and asks for the past features most
likely to be visible from there. Matching them against what it sees now gives the
correction. Without WiFi, the robot corrects by the ToF and the map only.

### Full-turn heading check (2)

Correct the heading using the overlap of the 390° scan; the gyro reads ~1.5° low per
full turn ([archive/ROBOT_PLAN.md](archive/ROBOT_PLAN.md) §4.6).

### Path planning (2)

Find a path around obstacles (A* on the whole-house map) and drive it as a series of
"drive to a point" steps.

### Places catalogue (2)

The server stores pictures from key places with their poses and names the rooms
("kitchen"), so that a name becomes a point on the map.

### Server commands (2)

The server sends the robot a goal or a list of moves, and a mode to switch to on
arrival. It can also start a patrol. They go over TCP, like the viewer's commands.

### Movement while driving (2)

Detect a mover while the robot drives straight. Today the VL53 forgets its
background after 10 cm of driving, and camera tracking (V1-V5) covers turning but
not driving. This is the hard part of run away and follow. Their first version is
crude (Daniel, 11 Oct): stop, look, and drive again.

### VL53 detector by sigma (2)

Detect a change with a z-test on the reading's sigma instead of a fixed 8 cm / 8 %,
and take edge times and angular rates from neighbouring zones. Details:
[archive/TOF_MOTION_PLAN.md](archive/TOF_MOTION_PLAN.md) T2, T3.

### ToF confidence model (2)

One confidence for each ToF reading, from its status, sigma and signal, used both by
the map (instead of 1 / 0.5 by status) and by the movement detector. Details:
[archive/TOF_MOTION_PLAN.md](archive/TOF_MOTION_PLAN.md) §3.
An example to check it against: snapshot 35 (boot 22a46c41), zone row 4 col 2,
a leg at 537 mm with status 9, sigma 11 mm, signal 32, and the wall behind it at
1456 mm, status 5, sigma 5 mm, signal 44. The map counts the leg at half weight
(status 9), so it needs 3 such frames in a row to overturn a free cell.

### Frontier search (1)

Find the edges between known free space and unknown space, and pick the next one to
drive to. Exploration ends when none can be reached. A patrol first marks part of
the map as out of date, which creates new edges to explore.

### Map age and change events (1)

Know how old each part of the map is, and scan when too much of the surroundings is
unknown or old. Report map changes as events: move an object while the robot isn't
looking, and the robot finds it. Details:
[archive/ROBOT_PLAN.md](archive/ROBOT_PLAN.md) §5.4 and
[archive/SURROUNDINGS_PLAN.md](archive/SURROUNDINGS_PLAN.md) S4.

### Moving away is movement (1)

Daniel raised this for discussion. Today only something nearer than the background
counts as movement, and a farther change never does.

### Slow glance (1)

Until a movement is confirmed (3 sightings over 0.2 s), turn at ~10°/s, then at full
speed, so that a blip causes only a small twitch. (Not answered.)

### Not needed by any mode yet

- **Roll in the pose:** when turning, the robot rolls 3-5°, and the map and the
  ToF floor rows ignore it. This includes the ToF frame time from the INT pin, the
  IMU sample time in ODOM, and DRIVE sent on change
  ([archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) A2). Do it only
  when a feature needs it, for example if door edges show false movement while
  turning.
- **A slip flag:** raised when the encoders and the gyro disagree, with roll as a
  check that the body really turns.
- **The MLX90640 thermal camera** (not implemented). It could help a sentry tell a
  person from a moving curtain.

## Small: fixes and improvements to existing modules

**Recording** (the full telemetry, `R`, off at power-up)
- **Resend lost datagrams from a ring** (planned 11 Oct; since 12 Oct only the full
  telemetry needs it, because snapshots go over TCP): the drive of 11 Oct lost up
  to 20 % of datagrams per take, on the air. The robot keeps ~25-50 KB of sent
  datagrams; the PC reports the gaps every ~0.2 s; the robot resends them, and at
  the action's end until the PC has all (or gives up and says so). Details:
  [archive/TELEMETRY_PLAN.md](archive/TELEMETRY_PLAN.md) §6, T7.
- Measure WiFi's CPU time on core 0 (the driver busy-waits for its DMA; estimated
  ~1 %, stalls up to ~0.5 ms), with and without recording.
- `TAKE_START`'s build time is `recorder.c`'s compile time, not the firmware's: a
  git hash from CMake if it matters.

**Map**
- The replay (`tools/replay_map.c`, the 3D viewer's map) keeps the window fixed
  around the power-up point: a drive beyond 3 m isn't replayed whole. The
  snapshot replay (`npm run snapreplay`) follows the robot.
- The cupboard was missing from the earlier scans of 10 Oct.
- One-frame blips (a single echo makes a cell); revisit with recordings of driving.
- Under a table top, false blocked columns; a small hole gets floor hits; far
  walls weak (ARCHITECTURE.md §4, known limits).
- `cell_map_free_distance()` is no longer used: remove it.
- "No floor" from floor reflections (13 Oct replay: 7 of 109 cells under the
  robot's path end as "no floor"): readings "beyond the floor" in rows 6-8 count
  fully since each target has its own status. An idea: a cell the robot stood on
  can't become "no floor".
- **A wall seen at a shallow angle is cleared by a scan** (13 Oct, boot 5b0d72ab,
  snapshot 25: the diagonal (-7, 262)-(28, 288) cm, seen at ~19°): rays slide
  through the free corners of the wall's cells and end on the wall further along,
  so the cells get passes, almost no hits, and 3 frames in a row clear them (the
  old code too). "Only through a cell's middle" was tried and dropped (it keeps a
  walker on the map). Next idea: a reading that meets its surface at under ~30°
  (from the neighbouring zone's reading) can't clear occupied cells. Also open:
  what is at (36, 304) cm, behind the wall's end, that only that scan saw.
- Second targets paint bands up to ~8 cells deep behind walls, where no ray ever
  clears them (Daniel: fine for now). If it matters: keep only second targets
  within some distance of the first, or let them mark unknown cells only weakly.

**Movement and watching**
- The watch test's 25°/s walker overshoots 1-4° depending on the seed and on where
  the moves before it ended to the mm (13 Oct: 3.6° after the map change moved a
  stop by 2 mm; the limit is now 4°): look at why it overshoots at all.
- A stopped thing becomes background after 1 s: shorter fades a stopped person
  sooner but may absorb someone walking slowly.
- ToF settings to compare: 15 or 10 Hz; heading bins of 1.4° or 2.8° (half the RAM).

**Robot and console**
- After a reconnect, the rest of a cut pose record shows as text ("manual -",
  "se -110.3 …"): the robot should drop a half-sent record when the connection
  changes, or the server ignore text up to the first newline. Rarer since 13 Oct
  (the server closes only after 30 s), not gone.
- The robot's uplink loses packets in places (13 Oct near (166, 61) cm: 11 KB
  unacknowledged for ~2 s, the congestion window halved, the server's `\0` still
  arriving). Watch the "nothing from the robot" notes and `p` there; a recording
  (`R`) of such a stretch has the TCP state every 0.5 s.
- RAM free is ~103 KB (12 Oct) after the snapshot buffer, vision and the
  whole-house map: watch it with V3's stacks and ring.
- `p` right after a power-up, to confirm the link's "lost" messages come from the
  start-up.
- M1 tests not yet done: the encoder distance backwards and the carpet (README
  "Robot tests", M1).
- Should `m` (the map as text) go now that the viewer shows the replayed map? The
  robot keeps its map either way (it drives by it).

**Viewer** (the 3D take viewer)
- Several takes at once; a DuckDB query box.

**Camera features (V1)**
- **Too slow on the robot** (first run, 12 Oct): `p` says vision takes ~60-63 ms a
  frame (the plan's limit: 25 ms), the loop's worst 85-89 ms. Measure with `v`
  where the time goes (finding, the coarse search, Lucas-Kanade), cut it, or
  process less often until V3 moves it to core 1.
- The pixel noise per gain is a guess (1 + 0.15 × gain grey levels): measure it
  (still scenes at each gain) and keep a table; it sets FAST's floor and the
  weakest corner.
- The archived plan's V1 limits that the host test doesn't meet: the RMS of every
  measured move under 0.15 px (met for the moves within 1 px; < 1 % are wrong by
  more), and every region's median within 0.1 px (met for still scenes and
  whole-pixel shifts; 95 % within 0.25 px at half-pixel ones). Test 10 (on
  recordings) waits for camera recordings.
- The search window is centred on each feature's own last move (V2's gyro hint
  replaces it), and templates are kept while they match (≥ 0.9, < 1 px): both
  differ from the plan's V1, which refreshed every frame around the last place.

**Viewer (the snapshots)**
Daniel's notes after the first run (12 Oct), to do together:
- **Pan with the arrow keys**, not the mouse. The right button opens the browser's
  menu ("Save image as…") on the map: no menu on the map, and panning by the
  arrows (the left-button drag can stay or go).
- **`v` is missing on the page**: the keyboard only passes the keys that have
  buttons, and `v` (camera features) has none. Add a button (Print: v features), or
  pass every key the robot knows.
- Explain or rename the header's links "3D takes" and "Takes" (they show the full
  telemetry recordings, `R`; empty unless `R` was on), or move them out of the way.
- Several snapshots compared side by side; the features' tracks over time.

**Camera**
- Binning gives vertical stripes on the PCB (not used: Sub4 without binning):
  [archive/CAMERA_STRIPES_TODO.md](archive/CAMERA_STRIPES_TODO.md).
