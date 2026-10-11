# Refactoring proposals: robot firmware M0 + M1

Code review of everything since tag `pcb-bringup-v1`, including the uncommitted M1
work, run with `doc/skills/code-review.md` (3 Oct 2026). Two separate reviews:

- **Standards:** does the code follow this repo's rules? The rules are ROBOT_PLAN.md
  §10 (who owns which knowledge), §11 (APOSD red flags, comments first, host tests)
  and "How we work" (stubs fail before init, float32 only, units in names), plus
  Fowler's code smells as judgement calls.
- **Spec:** does the code do what ROBOT_PLAN.md asks for M0 and M1?

The two axes are kept apart on purpose, so a clean result on one can't hide a
problem on the other. Each finding has a proposed fix.

**Status: all applied on 3 Oct 2026** (Daniel: "fix everything"). The outcomes are
logged in REDFLAGS.md, "M1 review pass". Where the fix differs from the proposal
below:
- **R3:** the new PicoB module is called `brain` (mirroring PicoA's `body`), not
  `brain_link`.
- **S1:** fixed with numbered MOTORS requests. PicoB echoes the last one it acted
  on in ODOM (protocol v4), instead of tracking agreement on PicoA alone, which
  can't tell a stale report from a new stop. PicoB ignores repeats, so a late
  repeat can't undo a safety stop either.
- **S6** changed the plan text; **A6** and **F3** were accepted.
- **T1** found a stale-ODOM case: motors already on, then refused. It isn't
  reachable on the robot, because PicoB only refuses without an IMU. The test now
  starts from motors off.
- **Found while fixing:** `run_tests.sh` passed when a test failed to compile
  (`set -e` and `&&`). Fixed.

S3 still needs a check on the robot.

Marks: ✔ = verified against the code while writing this doc, ? = plausible, not
proven.

---

## Spec (does the code do what the plan says?)

### Wrong or fragile

| # | Finding | Where | Proposed fix |
|---|---|---|---|
| S1 ✔ | **A lost MOTORS-on after a safety stop is never retried.** The plan says "re-sent until PicoB agrees". But `body` gives up as soon as a report more than 100 ms after the send still shows "off + stop reason", which is before the 200 ms retry. One dropped frame ends a test with "motors didn't switch on". | `picoA/app/body.c:84-89` | Only give up on a stop reason that is **new**, i.e. PicoB reported it after it had agreed to "on". Track `b_agreed_since_send`, or compare against a stop counter. Add a test to `test_body.c`: drop the first MOTORS after a safety stop. |
| S2 ✔ | **Pressing `s` after a safety stop raises the stop again.** `drive_enable(false)` keeps `fault`, and `on_motors(off)` clears `stop_reason`. On the next 10 ms tick `main` sees the old fault and logs a second "Motors off: …". The log then gets confusing. | `picoB/app/drive.c:43-50`, `picoB/app/main.c:58-66, 131-137` | `main` reports a fault once per occurrence: compare against the last reported fault, or let `drive_fault()` mean "new since the last enable". Covered by the PicoB `main` test (T3 below). |
| S3 ? | **USB printing can stall the main loop past the 250 ms DRIVE timeout.** If the Mac keeps the port open but isn't reading, the SDK's `printf` waits up to 500 ms (`PICO_STDIO_USB_STDOUT_TIMEOUT_US`). PicoA sends DRIVE from the same loop, so PicoB would stop with "drive commands stopped arriving". It could also explain the unexplained silence after `f` / `g`. | `picoA/app/main.c` loop; every `printf` | Set `PICO_STDIO_USB_STDOUT_TIMEOUT_US` to a few ms for `picoA_app` (in CMake), so a slow monitor loses text, never drive commands. Check on the robot with `l` (link counters) next session. |
| S4 ✔ | **A test that ends early leaves no result.** Only `finish()` stores text. A stop reason ("PicoB switched the motors off"), the progress ("turn 7 of 10") and PicoB's `B: Motors off: …` are printed once. If they happen while unplugged, they're gone. This is part of why the 3 Oct forward result was lost. | `picoA/app/robot_test.c:73-77, 205-210` | `stop(why)` also goes through `say()`, with the progress so far ("Turn test stopped after 7 turns: …"), and includes PicoB's stop reason (`stop_reason_text(o->stop_reason)`). |
| S5 ✔ | **The welcome text and last result are printed in the same loop pass the USB connects,** and the Mac drops them. Known: already planned in ROBOT_PLAN "Next session" 2. | `picoA/app/debug_console.c:48-53` | Print ~1 s after connecting; add key `t` to reprint the last result (as planned). |

### Missing or partial

| # | Finding | Proposed fix |
|---|---|---|
| S6 ✔ | §5.1 says the bias is re-measured when "motors commanded to zero, encoders not moving". The code uses wheel speed plus gyro < 3 °/s instead: odometry doesn't know the command. A slow turn below 3 °/s with the wheels still (being pushed by hand) is learned as bias, with a 5 s time constant. | **Keep the code, change the plan:** the gyro-rate check covers the same case without coupling odometry to `drive`. Reword §5.1. |

### Not asked for (all small, keep)

The "(robot moved)" flag in the drift test, "The centre moved" in the turn result,
"PicoB restarted", and `body` not re-asking for motors while PicoB reports
`imu_error`.

### Tests that pass for easier reasons than the robot

| # | Finding | Proposed fix |
|---|---|---|
| T1 ✔ | `test_robot_test.c` makes `motors_on` follow `body_motors()` instantly and the robot follow commands with no ramp. Stale ODOM from before the MOTORS message, and the motors never switching on, are never exercised. The test comment claims the second one. | Delay the simulated `motors_on` by a few ms. Add the "motors don't switch on → stop after 3 s" case. Ramp the simulated speed. |
| T2 ✔ | In `test_body.c`, the PicoB restart keeps the link alive. On the robot, PicoB's ~1 s gyro calibration drops the reports first. | Add a gap without reports (disconnected), then HELLO and reports again; motors restored. |

---

## Standards (does the code follow the repo's rules?)

### Hard violations of documented rules

| # | Rule | Finding | Proposed fix |
|---|---|---|---|
| R1 ✔ | §11: a header says what the module promises | `picoA/app/debug_console.h:2-5` still promises "a status line twice a second" and "losing the serial monitor stops the robot". Both are false since M1. | Rewrite: status every 15 s (none while a test runs), keys listed by `h`, unplugging doesn't stop the robot. |
| R2 ✔ | How we work: stubs fail if used before init | The fake IMU (`picoB/app/test/stubs/imu.h`) works without `imu_init()`. The fake encoders and motors don't. | `fake_imu_started`, asserted in `imu_read`, `imu_calibrate_gyro`, `imu_gyro_bias_z`. |
| R3 ✔ | How we work: every logic module gets a host test | PicoB `main.c` holds real logic with no test: the HELLO / version lock, `on_motors` refusing and clearing, the drive timeout, fault → stop reason. S2 lives exactly there. | Move the message and safety logic into a module, e.g. `picoB/app/brain_link.c` ("PicoA as PicoB sees it", the mirror of `body`). `main` then only runs the loop. Add `test_brain_link.c` with the fake link, playing PicoA. **(T3)** |
| R4 ✔ | How we work: float32 only, `-Wdouble-promotion` | The host tests compile without `-Wdouble-promotion`, so they allow what the firmware build rejects. | Add it to `CC` in `run_tests.sh`. |
| R5 ✔ | How we work / §11 nonobvious code: names carry units | `drive.c`: `v_target`, `w_target`, `v`, `w`, `left_i`, `right_i`, `turn_trim`. `robot_test.c`: `FULL_TURN`, `bias_start/min/max`, and `step_t.amount`, which is m, rad or s depending on `kind`. | `v_target_mps`, `w_radps`, `left_integral_m`, `turn_trim_mps`, `FULL_TURN_RAD`, `bias_start_radps`. Give `step_t` a union `{distance_m, angle_rad, duration_s}`. |
| R6 ✔ | Doc matches code | ROBOT_PLAN §10 lists `link_poll(&msg)`; the code has `link_receive()`. The plan's build section lists the host tests as "(link, drive, odometry)". | Update the plan. |

### APOSD red flags (judgement calls)

| # | Red flag | Finding | Proposed fix |
|---|---|---|---|
| A1 | Special-general mixture | The generic step runner in `robot_test.c` knows about particular tests: `if (test == ROBOT_TEST_TURNS)` prints turn lines inside `MOVE_TURN`, and `HOLD_STILL` does the drift test's bias tracking and reports. | Per-test hooks in a table (see F1): `progress(o)` and `result(o)`. The runner only moves, turns, holds and pauses. |
| A2 | Nonobvious code | `report_drift` and the turn result time from `leg_time` (start of the *step*) as if it were the start of the *test*. That's correct only because those tests have one step. | `test_start_time` set in `begin()`. |
| A3 | Repetition | Numbers restated as text: "10 min", "50 cm sides", "N/36 %" (only right while `TURNS` = 10), "15 deg" vs `MAX_TILT_RAD`, "1 s" vs `FOLLOW_TIME_S`. | Format the texts from the constants; comments point at the constant. |
| A4 | Repetition | `RAD_TO_DEG` defined in `debug_console.c` and `robot_test.c`, `DEG_TO_RAD` in `odometry.c`, π typed out twice. §11 names exactly this ("angle maths copied around"). | `common/units.h`: `PI_F`, `DEG_PER_RAD`, `RAD_PER_DEG`. The planned `vec2.h` can live next to it later. |
| A5 | Vague / colliding names | `DRIVE_PERIOD_US` is the 10 ms control loop on PicoB but the DRIVE resend period on PicoA. PicoB's `report()` prints a log line while `next_report` times ODOM. In `robot_test.c`, `leg` means "step" and `moved_time` means "step ended". | `CONTROL_PERIOD_US`, `log_to_a()`, `step_start`, `step_start_time`, `step_end_time`. |
| A6 | Information leakage (minor) | `debug_console` reads `link_stats()` directly, although §10 says `body` hides the link. | **Accept** and log in REDFLAGS: link counters are diagnostics about the wire itself, and wrapping them in `body` would be a pass-through method. |

### Code smells (Fowler; judgement calls)

| # | Smell | Finding | Proposed fix |
|---|---|---|---|
| F1 | Repeated Switches | `switch (test)` three times in `robot_test.c` (`name`, `robot_test_start`, `finish`), and "needs motors" computed twice. | One table of tests: `{name, intro, steps[], needs_motors, progress(), result()}`. Adding a test becomes one entry. This also fixes A1. |
| F2 | Repeated Switches / Shotgun Surgery | `drive_fault_t` mirrors `STOP_*`, joined by a ternary cascade in `main.c`. The tilt stop needed edits in `drive.h`, `drive.c`, `link_msgs.h` and `main.c`. | One fault → stop reason table in the new `brain_link` (R3). Or let `drive` report `STOP_*` directly, but that would leak the wire format into the controller, so the table is preferred. |
| F3 | Data Clumps | `wheel_left_*` / `wheel_right_*` pairs in `odom_t`, `odom_report_t` and `robot_test`. | **Leave** until a third use appears. |
| — | (REDFLAGS acceptances rechecked) | The duplicated HELLO exchange and `odom_t` vs `odom_report_t` still hold. `send_odom()` copying every field by hand is a growing cost; revisit if ODOM grows again. | — |

---

## Order of work (as proposed; all done)

Ordered by what unblocks the M1 robot tests first. This is not a severity ranking
across the two axes.

1. **Before the next robot session** (small, affects the M1 tests):
   - S3: shorter USB print timeout.
   - S1: MOTORS retry after a safety stop.
   - S4: keep early-stop results.
   - S5: delayed welcome text and `t` key.
   - R1: `debug_console.h` comment.
2. **PicoB `brain_link` module** with its host test: R3, S2, F2, A5 (PicoB names).
3. **`robot_test` table:** F1, A1, A2, A3, R5 (`robot_test` names), plus T1.
4. **Housekeeping:**
   - R2 (IMU stub), R4 (`-Wdouble-promotion`), A4 (`units.h`), R5 (`drive` names), T2.
   - R6 and S6 (plan text), A6 (REDFLAGS entry).

Steps 2–4 change no behaviour on the robot apart from S2. Rerun `./run_tests.sh`
after each step. The square test on the robot confirms nothing regressed.
