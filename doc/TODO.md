# RoboCar — what is left to do

Open work only (11 Oct 2026). What is built is in [ARCHITECTURE.md](ARCHITECTURE.md);
the old plans with the details are in [archive/](archive/). An idea needs Daniel's
yes before it is built. When something is done, it leaves this file and goes into
the [CHANGELOG](CHANGELOG.md).

## Big: new features and open ideas

### Next

1. **The map's window follows the robot.** The map is a fixed 6 × 6 m window around
   the power-up point; once the robot drives more than a few moves it leaves it.
   Needed before M4.
2. **Look at the 11 Oct drive in the viewer** (whole drive of boot 54564d43): how
   far the odometry drifted between the first and the last scan (a wall seen
   twice), and which cells are false.

### Driving (M4)

3. **Drive towards movement** ("Investigate"): turn to a movement's bearing, drive
   towards it, stop 50 cm from it; for a target that left the view, turn past the
   edge of the view on that side and watch. Then face open space. The order of
   needs (scan, investigate, face open space) and the Safety guard:
   [archive/ROBOT_PLAN.md](archive/ROBOT_PLAN.md) §7.3.
4. **One owner of the wheels** with a safety pass on every command (today
   `behaviour` drives and the console switches the motors off on space):
   [archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) A5.
5. **Map queries for driving:** a corridor ahead (drivable / open), how old each
   part of the map is, change events:
   [archive/SURROUNDINGS_PLAN.md](archive/SURROUNDINGS_PLAN.md) S4.

### Knowing where it is (M5, M6)

6. **Staleness and the full-turn check:** scan when too much of the surroundings is
   unknown or old; correct the heading with the 390° scan's overlap; map changes
   as events (move an object while the robot isn't looking, and it finds it):
   [archive/ROBOT_PLAN.md](archive/ROBOT_PLAN.md) §5.4, §4.6.
7. **Correction against the map** (heading and position drift over minutes):
   [archive/ROBOT_PLAN.md](archive/ROBOT_PLAN.md) §5.5.

### Movement detection

8. **Camera tracking:** features found and tracked frame to frame, ego-motion from
   the gyro, movers while still and then while turning, on core 1. Replaces
   `camera_motion` (which only watches while still and learns 1 s after each
   stop). Milestones V1-V5:
   [archive/CAMERA_MOTION_PLAN.md](archive/CAMERA_MOTION_PLAN.md).
9. **One `movement` module** fusing the VL53 and the camera into one target and
   events (and with it: remove the tracker or make it that target):
   [archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) A6,
   [archive/TOF_MOTION_PLAN.md](archive/TOF_MOTION_PLAN.md) T4. Robot test: 10
   walk-pasts × 2 distances × 2 ways, misses counted.
10. **"Movement is movement"** (Daniel, for discussion): today only "nearer than the
    background" is movement, farther changes never are.
11. **Slow glance:** until a movement is confirmed (3 sightings over 0.2 s) turn at
    ~10°/s, then at full speed, so a blip is only a small twitch. (Not answered.)
12. **The VL53 detector by sigma** instead of a fixed 8 cm / 8 % (a z-test), and
    edge times and angular rates from neighbouring zones:
    [archive/TOF_MOTION_PLAN.md](archive/TOF_MOTION_PLAN.md) T2, T3.

### Sensor data

13. **One ToF confidence model** from status, sigma and signal, used by the map
    (instead of 1 / 0.5 by status) and the movement detector:
    [archive/TOF_MOTION_PLAN.md](archive/TOF_MOTION_PLAN.md) §3.
14. **Roll in the pose** (turning, the robot rolls 3-5°; the map and the ToF
    floor rows ignore it), with the ToF frame time from the INT pin, the IMU
    sample time in ODOM and DRIVE sent on change:
    [archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) A2. Only when a
    feature needs it (e.g. door edges showing false movement while turning).

### Groundwork, when a feature needs it

15. **Printing that never blocks the loop** (printing the map stops the robot
    today), a watchdog and a crash-loop guard:
    [archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) A3.
16. **Core 1** (for the camera): explicit 8 KB stacks for both cores first, since
    core 1's stack sits where core 0's can grow today:
    [archive/ARCHITECTURE_PLAN.md](archive/ARCHITECTURE_PLAN.md) V3.

### Later

17. Path planning around obstacles (A* on the grid).
18. A Kalman filter (x, y, heading) inside `pose`, tuned from logs.
19. Battery watch once the BT_SNS wire is fitted (stop below 6.4 V, motors off
    below 6.0 V).
20. A slip flag (encoders and gyro disagree), roll as a check that the body really
    turns.
21. The MLX90640 thermal camera (not implemented).

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
