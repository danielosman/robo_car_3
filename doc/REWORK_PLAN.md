# RoboCar — the rework (start here)

The rework of the movement detection, the map and the architecture. **A fresh
session starts here.** This file holds only the current state, the decisions, and
the open suggestions and discussion points. What happened, and what was tried and
dropped, is in [REWORK_CHANGELOG.md](REWORK_CHANGELOG.md). README.md and
ROBOT_PLAN.md describe the robot as it is; ROBOT_PLAN.md stays the plan for
everything outside the rework (M1's open tests, M4-M6).

## The files

| File | What it is | Read it for |
|---|---|---|
| **REWORK_PLAN.md** (this) | Current state, decisions, open points, how to work | always first |
| [REWORK_CHANGELOG.md](REWORK_CHANGELOG.md) | What happened, robot runs, what was tried and dropped | history, before retrying an idea |
| [MAP_DESIGN.md](MAP_DESIGN.md) | **The map as built** (9 rays per zone, best measurement wins), its known limits and open points | anything about the map |
| [ARCHITECTURE_PLAN.md](ARCHITECTURE_PLAN.md) | Budgets (§1.3: RAM, stack, the stack tool), shared types, deep modules, red flags, the groundwork milestones (A2-A6, R0, M-S, V3) | groundwork when a feature needs it |
| [TOF_MOTION_PLAN.md](TOF_MOTION_PLAN.md) | VL53 movement. §7 (heading background) is built in a simpler form (below); the confidence model (§3), the z-test (T2) and edge timing (T3) are open ideas | movement detection |
| [SURROUNDINGS_PLAN.md](SURROUNDINGS_PLAN.md) | The first map design. **§3-§4.3 (cone rule, log-odds) are replaced by MAP_DESIGN.md**; its layers, columns and the L2 rule near the robot (§4.4) are what was built | background |
| [CAMERA_MOTION_PLAN.md](CAMERA_MOTION_PLAN.md) | Camera: features tracked frame to frame, ego-motion from the gyro, movement while turning. Not started | the camera work |
| [cross_plan.md](cross_plan.md) | The reviewer's numbers and shared types across the plans; its step order is outdated | when plans disagree on names or numbers |
| [COMMANDS_PLAN.md](COMMANDS_PLAN.md) | **Next.** Keys and states: idle at start, one key one operation, scan only 390°, move / turn / direction | the robot's keys |
| [TELEMETRY_PLAN.md](TELEMETRY_PLAN.md) | **Next.** Raw ToF + pose per action (a take) to the PC, `.rec` files, DuckDB, a three.js viewer, replay of the map code on recordings | anything about data, recordings, the viewer |
| [GLOSSARY.md](GLOSSARY.md) | The terms; MAP_DESIGN.md adds zone, cell, column, ray | when a word is unclear |
| [RobotMotionTrackingAlgorithms.md](RobotMotionTrackingAlgorithms.md) | The inspiration (a Gemini conversation); not a spec | background only |

## Decisions (Daniel)

How to work:
- **Visible progress first.** Each step should change something the robot does;
  groundwork (timing, telemetry, recording) only when a feature needs it.
- **Robot tests only when a step changes what the robot does or needs the robot's
  numbers.** A pure refactor is checked by the host tests (same output before and
  after) and rides along with the next robot test. No regression runs of code a
  later step replaces.
- **Plan files hold the current truth and the open suggestions, not history.**
  History goes to REWORK_CHANGELOG.md.
- Ask before changing behaviour that wasn't agreed.

The robot:
- Robot height **12 cm**, no extra clearance margin.
- Ground band **−7…+3 cm**: anything under 3 cm is ground and driven over (a 2.5 cm
  object not on the map is fine). Everything above is an obstacle. **No floor
  learning.**
- **Drivable** = floor seen in the ground cell, 3-13 cm and 13-23 cm free. L2 cells
  within 25 cm of the robot, never seen by any zone, count as free.
- **The map: 9 rays per zone, the best measurement wins** (MAP_DESIGN.md).
- **Movement is detected while still and while turning**, against a background
  kept per world direction; no learning after a stop.
- **Watching follows the biggest movement**: no predicting where it will go, no
  jumping ahead; its measured angular speed may be used, but never overshoot.
  **Movement at the edge of the view is inspected** (a one-zone blob turns the
  robot), like an animal would.
- The camera's movement detector will be one reader of a universal feature record;
  no camera images to the PC for now (CAMERA_MOTION_PLAN §11).
- Recordings (for replay on the Mac) are not in git: a git-ignored `recordings/`.
- **The robot does nothing by itself** (10 Oct): idle with the motors off after
  power-up and after a reset (no start-up scan); one key, one operation; no
  test-only actions (COMMANDS_PLAN.md).
- **Raw sensor data and the pose go to the PC, not the map** (10 Oct), only while
  an action runs (one take per action), stored as `.rec` files and in DuckDB;
  hypotheses and map fixes are tested on recordings (TELEMETRY_PLAN.md).
- Camera while watching: sharp and dark rather than noisy (exposure ≤ 10 ms, gain
  ≤ 8).
- No climbing worries for now: the robot drives slowly in controlled rooms.

## Current state

- **Budgets** (ARCHITECTURE_PLAN §1.3): everything is static, no `malloc`; RAM free
  **219 KB**. The core-0 stack's worst case was ~6 KB of the 8 KB it can use (the
  SDK's 2 KB is nominal); `tools/stack_depth.py` recomputes it (not re-run since
  the new map). `p` prints the loop time (mean, max, slowest stage over 10 s) and
  the free RAM.
- **Helpers** in `common/`: `stamp.h` (time differences across the wrap), `geom.h`
  (bearings, `wrap_pi`), `stats.h` (n-th value, median, sort). **Both Picos start
  their clock 30 s before `time_us_32()` wraps**, and so does the host tests' fake
  clock: every run and every test crosses the wrap.
- **VL53 movement** (`tof_motion`): background per world direction (8 rows × 256
  bins of 1.4°), fed every frame with the turn during the frame (from the pose); a
  zone is compared with the nearest bin it swept (1° slack for heading error);
  floor rows tell nothing while turning; bins forgotten after 10 cm of driving. A
  nearer thing that stays put for 1 s (robot still) becomes background. Works on
  the robot while still and turning.
- **The map** (`cell_map`, MAP_DESIGN.md): fed every frame, the scan's too; `m`
  prints it (the scan doesn't); the most open direction after the scan comes from
  it. The old map and the floor learning are gone.
- **Commands** (COMMANDS_PLAN, T1, 10 Oct): idle with the motors off at start-up,
  WiFi connects with or without USB, one key one operation, the same on USB and
  WiFi: space stop, `s` scan (390° only), `f` move 50 cm, `t` turn 30°, `r`
  direction, `a` watch 1 min, `C` clear the map. `robot_test` and the 15 s status
  are gone; the map's frame is odometry's from power-up. Robot test passed (10 Oct).
- **Recording** (TELEMETRY_PLAN T2, 10 Oct): `R` on, then each action (and `5`,
  record 5 s) is a take sent raw as UDP datagrams (`recorder.c`); the PC saves
  them and notes each take (`pc/robot/src/recording.ts`, `npm run decode`).
  Over TCP it stalled 1-6 s while turning; over UDP (10 Oct) no stalls, 2-3 % of
  datagrams lost, mostly single ones at some headings (the batteries under the
  antenna).
- **Watching** (`behaviour`, `a`, 1 minute): turns toward the biggest VL53 movement and follows it
  (world directions, its angular speed from a line through the last 0.5 s, plus 2 ×
  the angle still to go, at most ~29°/s; starts at 8°, stops below 3° when it is
  about still; its speed is dropped once past where it was last seen). Smooth on
  the robot (9 Oct). When the biggest movement is one zone, the log prints that
  zone's reading and background.

## Next

**A session on telemetry starts at [TELEMETRY_PLAN.md](TELEMETRY_PLAN.md) §0.**
T1-T6 in TELEMETRY_PLAN §6. T1 (the commands) is built and tested on the robot;
T2 (recording on the robot) is built and tested on the robot (UDP, 10 Oct); T3
(DuckDB, the take list) is built; next T4, the viewer. Then the viewer, replay, and the cup
session. Map fixes (point 1 below) wait
for its data.

## Open suggestions and discussion points

Not decided; each needs Daniel's yes before it is built.

**Map** (details in MAP_DESIGN.md §8):
1. **Low obstacles (under ~8 cm) at 0.4-1 m can be missed**: a 6 cm box 40 cm in
   front was not on the map after the scan (robot, 9 Oct); in the host test a 7 cm
   box at 50 cm has 1 of 4 front columns blocked. Suggestion: use the robot's own
   height and pitch to recognise a zone whose reading is where its lowest ray meets
   the floor (floor), so other readings are real surfaces that rays passing over
   them don't erase. Two other fixes were tried and failed (REWORK_CHANGELOG, 9 Oct).
   10 Oct: the cup loses the vote in every frame, mostly to rays passing over it
   inside the 3-13 cm layer (MAP_DESIGN §8.1); waits for real data (T6).
2. Under a table top ~0.8 m away: false "blocked" columns (host test: 6).
3. A small hole next to floor gets a few floor hits (a long drop is handled).
4. Standing still, the floor is seen only in rings (~20, 30, 45, 70 cm).
5. Far walls (beyond ~2 m) are weak and patchy (closeness weight).
6. Roll is not used (pitch only); turning, the robot rolls 3-5°.
7. A fixed 6 × 6 m window around the origin: a moving window before driving (M4).
8. Measure the map's cost per frame with `p` (with both maps the worst frame was
   22 ms in `surroundings`; 66 ms between frames).
9. VL53 confidence `v` from the shared confidence model (sigma, signal) instead of
   the status alone (TOF_MOTION_PLAN §3).

**Movement and watching:**
10. **Discussion (Daniel): "movement is movement".** Today only "nearer than the
    background" is movement; farther changes never are. Someone behind something
    nearer (the 6 cm box) can't be seen by the VL53 either way.
11. **Slow glance:** until a movement is confirmed (3 sightings over 0.2 s), turn at
    ~10°/s, then at full speed: a real movement gets a slow look, then following; a
    blip only a 2-3° twitch. (Not answered.)
12. Not starting a turn for a one-zone blob: Daniel wants edge movement inspected
    for now; the diagnostic showed the edge blobs at the start of walks were real.
13. The camera still learns 1 s after each stop and doesn't watch while turning
    (CAMERA_MOTION_PLAN V4-V5).
14. The tracker only logs now: remove it, or use its target for the log only.
15. A stopped thing becomes background after 1 s standing still: shorter fades a
    stopped person sooner but may absorb someone walking slowly.
16. The VL53 detector by sigma instead of a fixed 8 cm / 8 % (TOF_MOTION_PLAN T2),
    and edge times and rates from neighbouring zones (T3).

**Groundwork, when a feature needs it** (ARCHITECTURE_PLAN §8):
17. A2: the ToF frame time from the INT pin (if door edges show false movement while
    turning), DRIVE sent on change (turn latency), the IMU sample time in ODOM
    (protocol v5), roll in the pose, `camera_next()` for several readers (V1).
18. A3: printing that never blocks the loop (printing the map switches the motors
    off today), the watchdog and the crash-loop guard.
19. R0 and M-S: replaced by TELEMETRY_PLAN.md (recording per take, no images yet).
20. V3 (core 1 for the camera): explicit stacks first; core 1's stack sits where
    core 0's overflows today. `observations()` in `tof_motion.c` keeps 3.2 KB on the
    stack.
21. A5: one owner of the wheels before M4.

## Decided against

- Floor learning; a single ray per zone; adding up evidence over frames (standing
  still would erase what was explored).
- Predicting where a movement goes and turning ahead of it.
- A 75-minute soak test (the clock starts near the wrap instead).

## How to work a step

1. Read the code it changes, and the plan part if there is one. If the plan is wrong
   against the code, say so and fix the plan.
2. Host tests first where they make sense; `./run_tests.sh` stays green.
3. Build. A robot test only if the step changes what the robot does (above); hand
   Daniel the steps in the README's "Robot tests" style and wait for the log.
4. Update README / ROBOT_PLAN / this file (current state), REWORK_CHANGELOG.md
   (what happened), REDFLAGS.md (the review pass).
5. Commit only when Daniel asks; no AI co-author trailer (README "Git conventions").
