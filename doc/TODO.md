# RoboCar — what is left to do

Open work only (11 Oct 2026). What is built is in [ARCHITECTURE.md](ARCHITECTURE.md);
the old plans with the details are in [archive/](archive/). An idea needs Daniel's
yes before it is built. When something is done, it leaves this file and goes into
the [CHANGELOG](CHANGELOG.md).

This file is in three parts. **Goals** lists the robot's big modes, each with the
tasks it needs. **Tasks** describes every task once, the ones that most modes need
first. **Small** lists fixes to what is already built. A task keeps the same name
everywhere it appears, so the count shows what to build first.

## Goals: what the robot is for

Modes 1-3 are Daniel's; 4-6 were proposed and agreed on 11 Oct.

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
- Map follows the robot
- Corridor query
- Drive to a point
- Pictures to the server
- Resend lost datagrams
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
- Map follows the robot
- Corridor query
- Drive to a point
- Pictures to the server
- Resend lost datagrams
- Server vision
- Watchdog
- Battery watch
- Whole-house map
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
- Map follows the robot
- Corridor query
- Drive to a point
- Movement while driving

### 4. Go to a place

An operator says "go to the kitchen". The server finds the kitchen in its catalogue
of pictures and sends the robot a goal (a point on the map, or a list of moves) and
a mode to switch to on arrival, such as monitor. The robot sends a picture when it
arrives, so the server can check that it is in the right place. "Go home", back to
the power-up point, is the simplest case and needs no server.

- Mode switching
- Single wheel owner
- Map follows the robot
- Corridor query
- Drive to a point
- Pictures to the server
- Resend lost datagrams
- Server vision
- Watchdog
- Battery watch
- Whole-house map
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
- Map follows the robot
- Corridor query
- Drive to a point
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
- Pictures to the server
- Resend lost datagrams
- Server vision
- Watchdog
- Battery watch
- VL53 detector by sigma

## Tasks, most needed first

| Task | Modes | Needed by |
|---|---|---|
| Mode switching | 6 | 1 2 3 4 5 6 |
| Camera features | 6 | 1 2 3 4 5 6 |
| Single wheel owner | 5 | 1 2 3 4 5 |
| Movement target | 5 | 1 2 3 5 6 |
| Map follows the robot | 5 | 1 2 3 4 5 |
| Corridor query | 5 | 1 2 3 4 5 |
| Drive to a point | 5 | 1 2 3 4 5 |
| Pictures to the server | 4 | 1 2 4 6 |
| Resend lost datagrams | 4 | 1 2 4 6 |
| Server vision | 4 | 1 2 4 6 |
| Camera tracking | 4 | 1 3 5 6 |
| Watchdog | 3 | 2 4 6 |
| Battery watch | 3 | 2 4 6 |
| Whole-house map | 2 | 2 4 |
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

### Mode switching (6)

Today the robot runs one action at a time, each started by a key. A mode is a
longer-running behaviour that chooses actions by itself until it is stopped, for
example scan, then investigate, then face open space. The order of needs and the
Safety guard are in [archive/ROBOT_PLAN.md](archive/ROBOT_PLAN.md) §7.3. The
operator, or later the server, starts and stops modes, so the rule "the robot does
nothing by itself" still holds after a power-up.

### Camera features (6)

Find features (corners, edges) in every camera frame and give their positions in
the image, on core 1. Two tasks read them. Camera tracking follows them from frame
to frame to detect movers. Drift correction sends them, with the pose, to the
server and matches them against the features the server saw there before. Before
core 1 is used, both cores need explicit 8 KB stacks, because core 1's stack sits
where core 0's can grow today
([archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) V3). Plan:
[archive/CAMERA_MOTION_PLAN.md](archive/CAMERA_MOTION_PLAN.md).

### Single wheel owner (5)

Only one module drives the wheels, and every command passes a safety check. Today
`behaviour` drives and the console switches the motors off when space is pressed.
Details: [archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) A5.

### Movement target (5)

One `movement` module combines the VL53 and the camera into one target and its
events. With it, the tracker is either removed or becomes that target. Details:
[archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) A6 and
[archive/TOF_MOTION_PLAN.md](archive/TOF_MOTION_PLAN.md) T4. Robot test: 10
walk-pasts × 2 distances × 2 ways, with the misses counted.

### Map follows the robot (5)

The detailed 3-layer cell map is a fixed 6 × 6 m window around the power-up point,
and `f` already stops by it. Once the robot is more than 3 m from where it was
powered up, it drives blind. Every mode that drives therefore needs this window to
move with the robot. The window keeps its 3 layers with a confidence per cell
(~11 KB): building the map needs them, because a ray passing low over a thing must
not clear it, an overhang is not a wall, and the best measurement wins. The
4-bit columns of the whole-house map are its summary. What scrolls out of the
window is stored there, and what scrolls back in starts from it.

### Corridor query (5)

The map answers how far the robot can drive along a given bearing (drivable or
open), and which bearings are open. Investigate, run away and follow use it to
choose a direction, and all modes use it to face open space. Details:
[archive/SURROUNDINGS_PLAN.md](archive/SURROUNDINGS_PLAN.md) S4.

### Drive to a point (5)

Turn to a bearing, drive towards it and stop at a given distance from it, or 15 cm
before an obstacle. This is the move that every driving mode uses: investigate
(stop 50 cm from the target), run away, follow (stop about 1 m behind), and the
steps of a planned path.

### Pictures to the server (4)

Send a camera picture with the pose and the reason it was taken, for example
movement, a key place or arrival. The picture is the camera's 160 × 120
monochrome frame (19 KB), which is good enough for the server (Daniel, 11 Oct).

### Resend lost datagrams (4)

The drive of 11 Oct lost up to 20 % of datagrams per take, over the air. Pictures
and server commands need a link that does not lose data. Planned on 11 Oct: the
robot keeps ~25-50 KB of the datagrams it sent, and the PC reports the gaps every
~0.2 s. The robot resends them, and at the end of the action keeps resending until
the PC has them all, or gives up and says so. Details:
[archive/TELEMETRY_PLAN.md](archive/TELEMETRY_PLAN.md) §6, T7.

### Server vision (4)

The PC passes each picture to an image model, which decides for example whether it
shows a person lying on the floor and should raise an alarm, or which room it
shows. The robot itself does no image recognition.

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

### Whole-house map (2)

A map of the whole flat that is kept in RAM. A full-size cell map would not fit, but
a much smaller map would:
- **Columns, not cells:** exploring and planning need only each 10 cm column's
  class (drivable, blocked, overhang, no floor, open, unknown), which fits in
  4 bits. A 20 × 20 m flat then takes 20 KB, compared with ~195 KB free.
- **Sparse:** keep only the 2 × 2 m tiles that have been seen, so that open space
  costs little.
- **Lines for walls:** smaller still, but they make frontier search and path
  planning harder. Use them only if the columns don't fit.
- **On the server:** the PC could keep the full map and send the robot its part.
  The robot then depends on WiFi.

The 4-bit column grid is the first option to try.

### Drift correction (2)

Correct drift in heading and position over minutes. There are two sources: the
ToF against the map ([archive/ROBOT_PLAN.md](archive/ROBOT_PLAN.md) §5.5), and the
camera features against the ones the server saw near the same pose before (see
"Features on the server"). It could later become a Kalman filter (x, y, heading)
inside `pose`, tuned from logs. First measure the drift: in the whole 11 Oct drive
(boot 54564d43) in the viewer, measure how far odometry drifted between the first
and the last scan (a wall seen twice) and find which cells are false.

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
arrival. It can also start a patrol.

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

**Recording**
- **Resend lost datagrams from a ring** (planned 11 Oct): the drive of 11 Oct lost
  up to 20 % of datagrams per take, on the air. The robot keeps ~25-50 KB of sent
  datagrams; the PC reports the gaps every ~0.2 s; the robot resends them, and at
  the action's end until the PC has all (or gives up and says so). Details:
  [archive/TELEMETRY_PLAN.md](archive/TELEMETRY_PLAN.md) §6, T7.
- Measure WiFi's CPU time on core 0 (the driver busy-waits for its DMA; estimated
  ~1 %, stalls up to ~0.5 ms), with and without recording.
- `TAKE_START`'s build time is `recorder.c`'s compile time, not the firmware's: a
  git hash from CMake if it matters.

**Map**
- The cupboard was missing from the earlier scans of 10 Oct.
- One-frame blips (a single echo makes a cell); revisit with recordings of driving.
- Under a table top, false blocked columns; a small hole gets floor hits; far
  walls weak (ARCHITECTURE.md §4, known limits).
- `cell_map_free_distance()` is no longer used: remove it.

**Movement and watching**
- The watch test's overshoot limit (3°) fails for some random seeds (4 and 12):
  loosen it or look at why.
- A stopped thing becomes background after 1 s: shorter fades a stopped person
  sooner but may absorb someone walking slowly.
- ToF settings to compare: 15 or 10 Hz; heading bins of 1.4° or 2.8° (half the RAM).

**Robot and console**
- `p` right after a power-up, to confirm the link's "lost" messages come from the
  start-up.
- M1 tests not yet done: the encoder distance backwards and the carpet (README
  "Robot tests", M1).
- Should `m` (the map as text) go now that the viewer shows the replayed map? The
  robot keeps its map either way (it drives by it).

**Viewer**
- Several takes at once; a DuckDB query box.

**Camera**
- Binning gives vertical stripes on the PCB (not used: Sub4 without binning):
  [archive/CAMERA_STRIPES_TODO.md](archive/CAMERA_STRIPES_TODO.md).
