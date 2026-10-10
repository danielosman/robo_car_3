# Rework changelog

What happened in the rework (doc/REWORK_PLAN.md), newest first: robot runs, results,
what was tried, decided, kept or dropped. The plan files hold only the current state
and the open suggestions; the history is here. Before the rework: ../CHANGELOG.md.

## 10 Oct 2026: T1, the commands (COMMANDS_PLAN)

Daniel's answers: stop on space; one turn key (`t`, right in back direction);
motors off as much as possible; `C` clears the map; watching lasts a minute; when
the WiFi can't keep up, ToF frames are dropped and the data must say so; USB and
WiFi behave the same.

Built: the robot is idle with the motors off at start-up and connects to WiFi with
or without USB (no start-up scan, no scan after WiFi). `behaviour` has four actions
(scan 390° only, move, turn, watch 1 min), each switching the motors on at its
start and off at its end, printing "done" with what odometry measured or "stopped"
and why; a new action replaces the running one. The console: space stop, `s`, `f`,
`t`, `r` (direction), `a`, `C`, `p` (with link counters and the direction), `W`; gone:
`g`, `n`, `l`, `q`, `d`, `b`, `w`, the last test result and the 15 s status line.
`robot_test` and its test are removed (moves + telemetry replace them), and with
them `pose_set_origin()` (the map's frame is odometry's from power-up); the most
open direction is gone. The page's buttons follow the keys. Host tests:
`test_behaviour` checks idle, scan, turn ±30°, move ±50 cm, stop, replacing,
clearing and the 1-minute watch.

Found: the behaviour test's 8 cm box at 40 cm was found with 3 of 6 random seeds;
the old test passed by luck of its seed. It is the cup problem (MAP_DESIGN §8.1),
so the test reports it as KNOWN instead of asserting it.

Robot test (Daniel): every key as expected. Scan 14.9 s, +390.8°; moves +50.2-50.3
cm, 0.0-0.1° off; turns +30.5-31.1° (back direction −30.5°); keys pressed while an
action runs replace it (the new one counts from where the robot is then).

Then `v` removed (Daniel): the movement lines are printed only while watching;
idle, nothing is printed (the camera's lines came while idle). Telemetry will record
them as `MARK`s during a take.

## 10 Oct 2026: the cup and the box; plans for commands and telemetry

Daniel's runs: a glossy 6.5 cm cup ~45 cm ahead, a bit left, was on the map once
(one cell, 40 cm ahead, 20 cm **right**); a second scan with the cup and the 7 cm box
to the right showed no cup, and the only new obstacle was 50 cm to the **left**,
beside the robot. The room itself was right in both (not a mirror: the zone order,
the yaw sign, pose timing and the print all check out). The placements are not
explained. Between the runs the robot was turned back by hand (yaw 464.6° → 358.9°),
so both maps share one "up".

Simulation (scratch, not committed), the cup's 3-13 cm cell over a 390° scan: the
cup loses the vote in every frame at 25-60 cm (hits 13-23 % of the passes); ~80 %
of the passes go over the cup inside the layer (rows 3-4 from 1, at 7-13 cm).
Making hits count 4× kept the cup but brought back the ring of false obstacles at
~0.7-0.8 m (as on 9 Oct). Beyond ~71 cm the 5th row sees floor and cup in one zone;
what the real VL53 reads there is unknown (the simulation's nearest-surface model
says floor).

Daniel: the fixes argued from the simulation are weak; get the real data to the PC.
New plans: COMMANDS_PLAN.md (idle at start, one key one operation, scan = 390°
only, move / turn / direction, no test-only actions, no 15 s status) and
TELEMETRY_PLAN.md (raw ToF with all targets + pose per action as a take, `.rec`
files and DuckDB, a three.js viewer, replay). Replaces R0 / M-S and the start-up
scan after a watchdog reset. No code changed.

## 9 Oct 2026 (evening): robot test of smooth following, the old map gone

Daniel's run (scan, `m`, then `a` with `v`, walking across both ways, stopping):
**no more jitter**: following was smooth and stopped facing him. The scan printed no
map; `m` printed the new one. A 6 cm box ~40 cm in front was **not on the map** after
the scan, and later seemed to disturb the tracking when he walked behind it (the
VL53 can't see someone behind something nearer: movement is only "nearer than the
background"). No fixes asked for: written down as open points in the plans.

The one-zone diagnostic answered the earlier question: the one-zone blobs at the
start of each walk were real (e.g. row 4 col 7 read 1.40 m against a 3.27 m
background: Daniel at the edge of the view); a few read against a background of 0
(directions never seen) or just under the margin.

**Tried and dropped on 9 Oct** (kept here so they aren't tried again blindly):
- *A2 started, rolled back* (Daniel: no visible progress): the ToF frame time from
  the INT pin. The current timing error (~5 ms) is 0.3° at 1 rad/s, inside the 1°
  slack of the heading background.
- *A 75-minute soak for the 32-bit clock wrap*: replaced by starting both Picos'
  clocks 30 s before the wrap.
- *Map, after the first robot run (obstacles on the right shown too far)*: (1) an L1
  cell passed only by rays through its bottom 2 cm, (2) the verdict "more than half
  of max(rays, 9)" with closeness per cell. Both made a ring of false obstacles at
  ~0.7 m (the 5th row's rays grazing the floor end 3-6 cm up) and lost far walls;
  reverted. The obstacles turned out to be a chair on wheels (thin parts, gaps).
- *Tracker: a target lost while moving fast toward the edge counts as leaving*
  (predicted direction): Daniel didn't want the robot to try to get ahead of the
  movement. Replaced by following the biggest movement.
- *Following, first version*: turn toward the biggest blob, 5° dead band, minimum
  speed 8.6°/s: stop-and-go every 0.2-0.5 s on the robot. Replaced by the follow
  controller (world directions, fitted speed, 8°/3° start/stop, no overshoot).

## 9 Oct 2026: following movement smoothly; the old map removed

Robot test of watching while turning and the new map (Daniel): detection while
turning works; the new map looks better than the old one. Turning towards the
biggest movement replaced the tracker's leave-and-jump (Daniel: don't predict, start
earlier, turn slowly); then, from the next run's stop-and-go every 0.2-0.5 s: the
direction is kept in world terms (corrects for the robot's own turn since the
frame), the movement's angular speed is fitted over 0.5 s and used as feed-forward,
start at 8° / stop at 3°, no minimum speed, the speed dropped once past where it was
last seen (no overshoot). Host test: a steady walker gets 1 start, 0.5-4.8° lag, at
most 2.9° overshoot when they stop. One-zone blobs at the edges still turn the robot
(Daniel: an animal inspects edge movement); the log now prints the zone's reading
and background to find out what they are. An unsure ToF reading no longer resets a
zone's "steady for 1 s". The old map (`world_map`, floor learning, the scan's kept
frames, the auto print, the motors-off pause after the scan) is removed: `m` prints
the new map, the most open direction comes from it. RAM free 219 KB (was 74).

## 9 Oct 2026: watching while turning, the new map

Reordered with Daniel: visible progress first, groundwork only when a feature needs
it (A2's ToF INT timing was started and rolled back: 5 ms of timing error is 0.3°
at 1 rad/s, inside the 1° slack).

**Watching while turning** (T-R, TOF_MOTION_PLAN §7): `tof_motion` keeps the
background per world direction (8 rows × 256 bins of 1.4°), fed every frame with
the turn from `pose_at` at both ends of the frame. No learning after a stop;
`find_passing()` is gone; floor rows tell nothing while turning; the bins are
forgotten after 10 cm of driving. Host tests: the still-room cases as before;
turning 5 times each way at 0.5/1/1.5 rad/s past a door edge and with the heading
3° off: no movement; someone found in the frame they enter the view while turning,
and in the 2nd still frame after a stop (was ~1.5 s).

**The new map** (`cell_map.c`, design in doc/MAP_DESIGN.md, Daniel's): 9 rays per
zone as long as the reading, passes for free, the end cell for occupied, each ray
weighted by closeness (sigmoid at 1.5 m) and VL53 status; per frame one verdict and
confidence per cell; a cell keeps its best measurement and changes for a better one
or 3 good-enough ones in a row. No floor learning, no floor assumed. Runs beside the
old map (`m` old, `M` new). Host tests: no false obstacles in an empty room or a
hallway, a box blocked, a table top an overhang, nothing beyond stairs down
drivable, a chair leg erased from 1 m, a box kept while standing 10 min 2 m away,
a walker gone after 1 frame. Found: the ground layer may only be "passed" by a ray
going through its bottom (crossing the air above the floor says nothing about it);
standing still the floor is seen in rings at each row's distance; known cases:
under a table top at 15-20 cm ~0.8 m away, 6 false blocked columns (the 4th row's
rays stop at the underside); a small hole beside floor can get a few floor hits.
RAM free 74 KB.

## 9 Oct 2026: rework step A1 (helpers, clock wrap, loop timer)

Shared helpers in `common/`: `stamp.h` (time differences), `geom.h` (`bearing_rad`,
`wrap_pi`), `stats.h` (`nth_value`, `median_value`, `sort_values`), with
`test_helpers`. They replace five `atan2f` bearings, four hand-written time
differences, `pose`'s `after()`, `behaviour`'s `wrap_angle()` and three
`compare_*` + `qsort` copies. `change_grid_observations()` /
`change_grid_biggest()` turn blobs into observations once, for both the VL53 and
the camera detector. Every host test's output was the same before and after.
The fake clock now starts 30 s before `time_us_32()` wraps (the short tests start
nearer), and every test crosses the wrap (checked by printing each test's end
time): no wrap bug found, only test artifacts (a hard-coded frame time, PicoB's
simulated report on multiples of 20 ms of the absolute time). New `loop_stats`:
`p` prints "Loop: N per s, mean, max (slowest: stage)" over the last 10 s and the
free RAM (136 KB). No robot test of its own (Daniel: no regression runs of code
that later steps replace): the first `p` check rides with A2's robot test. No 75
min soak for the wrap: both Picos now start their clock 30 s before
`time_us_32()` wraps (`common/clock_start.h`), so every run crosses it 30 s after
power-up; the movement log still prints seconds since power-up. Rule added to REWORK_PLAN: robot
tests only when a step changes what the robot does or needs the robot's numbers.

## 9 Oct 2026: rework step A0 (Measure) done by analysis

Daniel questioned A0's robot session: RAM and stack follow from the data
structures, and the old loop's stalls go away in A3 anyway. So A0 was done from the
build instead. RAM: everything is static, no `malloc` is linked, 385 of 512 KB
used. Stack: `tools/stack_depth.py` (new) sums GCC's per-function stack sizes along
the call graph, with the calls through function pointers added by hand (printf's
output, lwIP). Core 0 needs ~6 KB at worst: the main loop alone 4.2 KB, 3.2 KB of
it in `observations()` (`tof_motion.c`). That is over the SDK's nominal 2 KB, which
nothing enforces: the stack grows into the two empty scratch banks (8 KB). It works
now, but core 1's stack sits there, so V3 must set both stacks before starting core
1 (already planned). The loop timer and "RAM free" in `p` moved to A1.
ARCHITECTURE_PLAN §1.3 and §8, cross_plan and REWORK_PLAN updated.

## 8 Oct 2026: rework plans (architecture, movement, map)

No code changed. From Daniel's brain dump and an inspiration doc
(`doc/RobotMotionTrackingAlgorithms.md`), four plans were written, reviewed
together by a reviewer and revised: the architecture (today's functions as a
spec, deep modules, red flags), VL53 movement (per-reading confidence, edge times
across neighbouring zones, watching while turning with a heading background),
the map (ground as an ordinary cell layer, the cone-slice rule, confidence-weighted
log-odds, no floor learning) and the camera (features tracked frame to frame into
a record of measurements only; movement detection is one reader and removes the
robot's own turn itself). Decided with Daniel: ground band −7…+3 cm; drivable =
ground + both layers above free; camera dark rather than noisy (≤ 10 ms, gain
≤ 8); no camera images to the PC; recordings not in git; scan after a watchdog
reset. Start: [doc/REWORK_PLAN.md](doc/REWORK_PLAN.md) (order of 21 steps), terms
in [doc/GLOSSARY.md](doc/GLOSSARY.md). It replaces the "fix the misses" next step
of 7 Oct (`find_passing()` goes in T-R).

