# Red-flag log

Red flags from *A Philosophy of Software Design*, found while writing the robot
firmware or in the review pass at the end of each milestone (ROBOT_PLAN.md §11).
Each entry says how it was fixed, or why it was accepted.

## M0 — link, drive, odometry

| Module | Red flag | What | Resolution |
|---|---|---|---|
| `link_msgs.h`, PicoA `body`, PicoB `main` | Information leakage | The 250 ms safety timeout, the 50 Hz report rate and the 1 s HELLO period were hard-coded on both Picos; changing one side would silently break the other | **Fixed:** defined once in `link_msgs.h` (`LINK_DRIVE_TIMEOUT_US`, `LINK_ODOM_PERIOD_US`, `LINK_HELLO_PERIOD_US`); PicoA derives its periods from them |
| PicoA `body`, PicoB `main` | Vague name | Both had a `handle()` for incoming messages | **Fixed:** renamed `on_message()`, next to `on_hello()`, `on_drive()` etc. |
| PicoB `odometry` | Nonobvious code | The wheel step relies on the heading step running first (it uses this step's turn); nothing said so | **Fixed:** comment at the call site |
| PicoA `body`, PicoB `main` | Repetition | The HELLO exchange (send, reply unless it's a reply, version check) is written on both sides | **Accepted:** ~10 lines each; the two sides react differently to a mismatch (PicoB locks the motors, PicoA tells you to reflash). Revisit if the exchange grows |
| PicoB `odometry`, `link_msgs.h` | Repetition | `odom_t` (PicoB's estimate) and `odom_report_t` (what PicoB sends) share most fields | **Accepted:** they're different abstractions. `odom_t` has wheel speeds that only `drive` needs; the report adds motor and safety state owned by `main` and `drive`. One struct for both would leak the wire format into the controller |
| PicoB `odometry` | Nonobvious code | The IMU's mounting (which axis is forward) was an assumption | **Fixed:** the M0 test showed X backward, Y right (turned 180° about Z); `to_robot_frame()` and the odometry test now use that |
| PicoA `body` | (test gap, not an APOSD flag) | The greeting, motor re-send and connection logic has no host test yet; PicoB's side and the link do | **Fixed in M1:** `picoA/app/test/test_body.c` with a fake link (which fails if used before `link_init()`) and the test playing PicoB |
| PicoB `odometry` | (bug found on the robot, test gap) | `encoder_init()` was never called, so the wheels read 0 and the square test drove on at full power. The host tests' fake encoders worked without being started | **Fixed:** odometry starts the encoders; the fake encoders and motors now fail a test if used before their init. Added a stall stop in `drive` (a wheel driven but not turning for 1 s switches the motors off) so a missing encoder or a jammed wheel can't run away again |
| PicoB `drivers/encoder.c`, `drive` | (bug found on the robot) | Both encoder signs were reversed (set to match the motor signs in bring-up, never checked with RPM going forward), so the speed loop ran away at full power. The stall stop didn't catch a wheel turning the wrong way | **Fixed:** both encoder signs flipped; the stall stop became "wheel not following its target for 1 s" (stopped, far too slow or wrong way), with a host test for reversed encoders. Stop texts shortened to fit a LOG message |

## M1 — calibration tests

| Module | Red flag | What | Resolution |
|---|---|---|---|
| PicoA `drive_test` → `robot_test` | Vague name / hard to pick name | With the drift test (motors off) the module no longer only drives | **Fixed:** renamed `robot_test`: tests run on the robot. One interface, `robot_test_start(test)`; each test is a short list of steps (forward, turn, hold still) run by one state machine, so the square, turns, straight and drift tests share the driving and pausing code |
| PicoA `robot_test` | Information leakage (avoided) | The turn test needs the effective track width, and the track width is PicoB's knowledge (§10) | **Avoided:** PicoA computes it from the wheel distances and the gyro angle alone, (right − left) / angle; no PicoB constant is copied to PicoA |
| PicoB `odometry`, `drivers/imu` | Nonobvious code | The reported gyro bias is the IMU driver's power-up calibration plus odometry's tracked part, and the two are in different frames in principle | **Fixed:** `imu_gyro_bias_z()` exposes the driver's part; a comment at the sum says why sensor z equals robot z (mounted Z up) |
| PicoB `drive` | Special-general mixture (considered) | The tilt stop is safety, not wheel control | **Accepted:** `drive` already owns "switch the motors off by itself" (the wheel stops) and reports it as `drive_fault()`; a second place switching motors off would split that knowledge. `brain` only maps faults to stop reasons |
| `run_tests.sh` | (bug found in review) | Piping the robot test through `grep` to hide drift progress lines made the pipeline take grep's exit code, hiding a failing test | **Fixed:** the test writes to a log; a failure prints the log and exits 1 (checked by breaking an assertion) |

### M1 review pass (REFACTORING.md, 3 Oct 2026)

A two-axis code review (standards and spec) of everything since `pcb-bringup-v1`;
all items applied. Item numbers refer to REFACTORING.md.

| Module | Red flag | What | Resolution |
|---|---|---|---|
| PicoB `main` | (test gap) / Divergent change | `main.c` held the protocol, the safety stops and the loop, with no host test (R3) | **Fixed:** new module `brain` (PicoA as PicoB sees it, the mirror of PicoA's `body`) with `test_brain.c`; `main` is only the loop |
| PicoB `brain` | (bug found in review) | `s` after a safety stop raised the same stop again: `drive` keeps its fault until switched on, and MOTORS off cleared the stop reason (S2) | **Fixed:** each fault is reported once (`reported_fault`); tested |
| PicoA `body`, `link_msgs.h` | (bug found in review) | A lost MOTORS-on after a safety stop was never re-sent: `body` read the stale "off + stop reason" reports as a new stop (S1) | **Fixed:** MOTORS requests are numbered and PicoB echoes the last one it acted on in ODOM (protocol v4). `body` re-sends until echoed and only respects a stop that came after; PicoB ignores repeats, so a late repeat can't undo a stop. Tested on both sides |
| PicoA build | (bug found in review) | USB `printf` could block the loop up to 500 ms when the monitor isn't reading, longer than PicoB's 250 ms DRIVE timeout (S3) | **Fixed:** `PICO_STDIO_USB_STDOUT_TIMEOUT_US=5000` for `picoA_app`; to confirm on the robot |
| PicoA `robot_test` | Repeated switches / special-general mixture | Three `switch (test)` and test-specific code inside the generic step runner (F1, A1) | **Fixed:** one table of tests (steps, intro, begin, progress, result); the runner only moves, turns, holds and pauses |
| PicoA `robot_test` | (spec gap) | A test stopped early left no result to reprint (S4) | **Fixed:** the result says how the test ended, why (with PicoB's stop reason) and what was measured so far |
| PicoA `robot_test` | Nonobvious code | Turn and drift timing used the step's start as the test's start (A2) | **Fixed:** `test_start_time` |
| PicoA `robot_test`, `debug_console`, `link_msgs.h`, `drive.h` | Repetition | Numbers restated in texts ("10 min", "50 cm", "N/36", "15 deg", "1 s") (A3) | **Fixed:** texts formatted from the constants; the drive limits are public in `drive.h` (`DRIVE_FOLLOW_TIME_S`, `DRIVE_MAX_TILT_DEG`) and the stop texts no longer carry numbers |
| all | Repetition | Angle constants and π defined in four places (A4) | **Fixed:** `common/units.h` |
| PicoB `drive`, `main`; PicoA `robot_test` | Vague / colliding names, nonobvious units | `v`, `w`, `left_i`, `turn_trim`, `FULL_TURN`, `bias_start`, `step.amount`; `DRIVE_PERIOD_US` meaning two things; `report()` vs `next_report` (R5, A5) | **Fixed:** units in names (`v_mps`, `left_integral_m`, `turn_trim_mps`, `FULL_TURN_RAD`, `bias_start_radps`, a step union `distance_m` / `angle_rad` / `duration_s`), `CONTROL_PERIOD_US`, `brain_log()` |
| PicoB `brain` | Repeated switches / shotgun surgery | Fault → stop reason as a ternary cascade (F2) | **Fixed:** one table `stop_for_fault[]` |
| PicoA `debug_console` | Information leakage (considered) | Reads `link_stats()` directly although `body` hides the link (A6) | **Accepted:** the counters are diagnostics about the wire itself; wrapping them in `body` would be a pass-through method |
| PicoA `debug_console` | Interface comment | The header promised M0 behaviour (status twice a second, unplugging stops) (R1) | **Fixed** |
| test stubs | (test gap) | The fake IMU worked without `imu_init()` (R2); the host tests allowed float → double (R4); `robot_test`'s fake robot followed commands instantly (T1); the PicoB restart test kept the link alive (T2) | **Fixed:** init asserts; `-Wdouble-promotion` in `run_tests.sh`; fake robot with ODOM lag and ramps, plus "motors don't switch on"; restart with a report gap. Fakes shared in `common/test/fakes` |
| `run_tests.sh` | (bug found while fixing) | `set -e` doesn't stop at a failing command left of `&&`, or inside a function called left of `||`: a test that failed to compile still let the script pass | **Fixed:** compile and run as separate commands; checked by breaking a compile and an assertion |
| Wheel pairs | Data clumps | `wheel_left_*` / `wheel_right_*` travel together (F3) | **Accepted** until a third use appears |

