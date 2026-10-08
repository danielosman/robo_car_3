# RoboCar — the rework (start here)

The main plan for the rework of the architecture, movement detection and map
(written 8 Oct 2026, from Daniel's brain dump; four plans written, cross-reviewed
and revised). **A fresh session starts here**, not in one of the sub-plans: this
file says what each plan is, what was decided and in which order to build.

It supersedes, once each step lands: ROBOT_PLAN.md §4 (the map, floor learning),
§6.2-§6.6 (movement detection, learning after a stop, `find_passing()`) and the
"Next: the misses" item of ROBOT_PLAN's handover. Don't fix `find_passing()`: T-R
removes it. ROBOT_PLAN.md stays the plan for everything else (M1's open tests,
M4-M6 after this rework) and keeps describing the robot **as it is**; update it,
the README and CHANGELOG as each step lands (README "Docs").

## The files

| File | What it is | Read it for |
|---|---|---|
| **REWORK_PLAN.md** (this) | Entry point: the files, the decisions, the order, how to work | always first |
| [ARCHITECTURE_PLAN.md](ARCHITECTURE_PLAN.md) | Today's functions as a spec, a first-principles design, shared data types (§2.3), deep modules (§5), red flags (§6), every improvement idea with an opinion (§7); milestones A0-A6, R0, M-S, V3 | the A, R0 and M-S steps; the shared types; how modules fit |
| [cross_plan.md](cross_plan.md) | The reviewer's binding decisions: numbers, shared types, the one ToF confidence model, ownership, RAM budget, merged order | when two plans seem to disagree: this file wins on names and numbers, the sub-plan on detail |
| [TOF_MOTION_PLAN.md](TOF_MOTION_PLAN.md) | VL53 movement: per-reading confidence (§3), a zone over time and edge timing across neighbouring zones (§4-§6), the heading background for watching while turning (§7) | T1-T4 |
| [SURROUNDINGS_PLAN.md](SURROUNDINGS_PLAN.md) | The map: ground as an ordinary cell layer, the cone-slice rule (§3, with a figure), log-odds evidence weighted by confidence (§4), drivable = ground + both layers above free | S1-S4 |
| [CAMERA_MOTION_PLAN.md](CAMERA_MOTION_PLAN.md) | Camera: corners tracked frame to frame (`vision`), the universal per-frame record `vision_frame_t` (§5), ego-motion from the gyro (§6), movement detection as one reader (`camera_movers`, §8) | V1-V5 |
| [GLOSSARY.md](GLOSSARY.md) | The terms all plans use, one word per thing | when a word is unclear; add new terms there |
| [RobotMotionTrackingAlgorithms.md](RobotMotionTrackingAlgorithms.md) | The inspiration (a Gemini conversation); not a spec | background only |

## Decisions (Daniel, 8 Oct 2026)

- Robot height: **12 cm** (really a bit less); **no extra clearance margin**.
- Ground band **−7…+3 cm**: anything under 3 cm counts as ground and is driven
  over; everything above is an obstacle. No floor learning.
- **Drivable** = ground seen and L1 (3-13 cm) and L2 (13-23 cm) both free. The
  never-seen L2 cells within 25 cm of the robot are set free once after the scan.
- **ToF and camera both detect while the robot turns**, using the known rotation;
  no learning after a stop.
- The camera's movement detector is **one reader** of a universal feature record.
- **No camera images to the PC** for now: no image recording, no drawing of
  features (CAMERA_MOTION_PLAN §11). Vision is tested with synthetic frames on
  the host and printed numbers on the robot.
- **Recordings** (VL53 frames, odometry, keys; for replay on the Mac) are
  **not in git**: a git-ignored folder (`recordings/`). Host tests in git use
  synthetic data; replay checks run when the folder is there.
- **After a watchdog reset without USB: the start-up scan**, as at power-up
  (Daniel prefers a scan after any start without USB). Guard: after 3 resets
  within a minute, stay idle and say why (a crash loop must not keep driving).
- **Camera while watching: sharp and dark rather than noisy.** Exposure ≤ 10 ms
  (1.6 px blur at 1 rad/s), gain ≤ 8; evening images ~3× darker than the target,
  accepted (CAMERA_MOTION_PLAN §6.3).
- **No climbing worries:** under 3 cm is usually the floor; the robot drives
  slowly in controlled rooms for now; tilt/shock detection comes later.

Still open, not blocking: grid size 6 × 6 m (decide before M4,
SURROUNDINGS_PLAN §11); the measured values the M-S session decides (status
weights, 2 or 3 targets, 15 or 10 Hz).

## The order

One step at a time; each ends with a working robot, host tests green and its robot
tests done by Daniel. "Plan" says where the step is written out.

| # | Step | Plan | What it gives |
|---|---|---|---|
| 1 | A0 — Measure | ARCHITECTURE §8 | loop time, stack, RAM numbers to decide on |
| 2 | A1 — Helpers, clock wrap | ARCHITECTURE §8 | shared bearing/time/median code; tests across the 71.6 min wrap |
| 3 | A2 — Time at the source, published frames, DRIVE on change | ARCHITECTURE §8 | every measurement stamped when measured; readers don't steal frames (protocol v5) |
| 4 | A3 — Non-blocking telemetry, watchdog | ARCHITECTURE §8 | printing never blocks the loop; hang → reset → scan |
| 5 | R0 — Recording and replay | ARCHITECTURE §8 | key `R`, WiFi recording, `replay` on the Mac |
| 6 | M-S — Measurement session | ARCHITECTURE §8 (+ the M-S parts of TOF §12, SURROUNDINGS §9, CAMERA §13) | recordings and numbers that settle the unverified claims |
| 7 | A4 + T1 — `tof_quality`, `tof_frame_t`, `view` | ARCHITECTURE §8, TOF §12 | one confidence model; no behaviour change |
| 8 | S1 — Classify readings | SURROUNDINGS §9 | the cone-slice rule as a pure function |
| 9 | T2 — z-test, weighted evidence | TOF §12 | "nearer" judged by sigma, not fixed cm |
| 10 | T-R — Heading background | TOF §12 (§7) | ToF watches while turning; no learning after a stop |
| 11 | S2 — New map alongside the old | SURROUNDINGS §9 | both maps compared on the robot |
| 12 | S3 — Switch the map, floor learning gone | SURROUNDINGS §9 | ~135 KB RAM freed (needed for the camera) |
| 13 | V1 — `vision` on core 0, measured | CAMERA §13 | features found and tracked, timed |
| 14 | V2 — Ego-motion, calibration | CAMERA §13 | gyro prediction, rolling shutter, focal length from the scan |
| 15 | V3 — Core 1 | CAMERA §13, ARCHITECTURE §8 | vision runs beside the main loop |
| 16 | A5 — One activity at a time, safety pass | ARCHITECTURE §8 | one owner of the wheels (before M4) |
| 17 | V4 — Camera movers while still | CAMERA §13 | old `camera_motion` removed |
| 18 | T3 — ToF edge times and rates | TOF §12 | direction and speed from neighbouring zones |
| 19 | A6 + T4 + V6 — `movement` | ARCHITECTURE §8, TOF §12, CAMERA §13 | both sensors into one tracker and behaviour |
| 20 | V5 — Camera movers while turning | CAMERA §13 | |
| 21 | S4 — Map queries for M4 | SURROUNDINGS §9 | drivable corridor, changes, staleness |

Then back to ROBOT_PLAN.md: M4 onwards. Steps 8-12 (map) and 9-10 (ToF) touch
different modules and may swap, but S3 must come before V1 (RAM, cross_plan §5).

## How to work a step

1. Read the step in its plan, the parts of cross_plan.md it names, and the code
   it changes. If the plan is wrong against the code, say so and fix the plan
   first.
2. Host tests first where the plan lists them; `./run_tests.sh` stays green.
3. Build, then hand Daniel the robot test steps (as written in the plan, in the
   README's "Robot tests" style) and wait for the pasted log.
4. Update README / ROBOT_PLAN (current state), CHANGELOG (what happened),
   REDFLAGS (the review pass), and tick the step here.
5. Commit only when Daniel asks; no AI co-author trailer (README "Git
   conventions").

## Progress

- [ ] 1 A0 · [ ] 2 A1 · [ ] 3 A2 · [ ] 4 A3 · [ ] 5 R0 · [ ] 6 M-S · [ ] 7 A4+T1
- [ ] 8 S1 · [ ] 9 T2 · [ ] 10 T-R · [ ] 11 S2 · [ ] 12 S3 · [ ] 13 V1 · [ ] 14 V2
- [ ] 15 V3 · [ ] 16 A5 · [ ] 17 V4 · [ ] 18 T3 · [ ] 19 A6+T4+V6 · [ ] 20 V5 · [ ] 21 S4
