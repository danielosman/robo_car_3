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


### M1 close: motor change (4 Oct 2026)

| Module | Red flag | What | Resolution |
|---|---|---|---|
| PicoB `odometry.h` | Information leakage | `odom_t.wheel_mps[ENC_COUNT]` exposes the encoder driver's wheel enum to every user of odometry | **Accepted:** only `drive` reads it, and the per-wheel stop needs each wheel; a second wheel enum in odometry would be repetition |
| PicoB `drivers/encoder` | Nonobvious code | `encoder_count()` drains the PIO FIFO and waits for one fresh count (~50 ns) | **Fixed:** comment at `raw_count()` says why |
| PicoB `drivers/encoder` | (bug avoided) | A PIO state machine started with an unknown previous pin state counts a false step | **Avoided:** the current pins are loaded as the previous state before starting |

### M2 while coding (4 Oct 2026)

| Module | Red flag | What | Resolution |
|---|---|---|---|
| PicoA `robot_test`, `behaviour` | Repetition | Both turn to a target angle with the same slow-down profile | **Fixed:** `motion.h` (`motion_turn_rate`, `motion_speed`), used by both |
| PicoA `drivers/tof` | Special-general mixture | The driver built the bring-up's USB packets and parsed its commands | **Fixed:** the driver only starts the sensor and returns frames; the streaming is `bringup/tof_stream.c` |
| PicoA `surroundings` | Pass-through module (considered) | Between rangefinder, pose and world_map | **Accepted:** it owns real knowledge: which pose a frame belongs to, and keeping the start-up turn's frames until the floor is known |
| PicoA `rangefinder` | Conjoined methods (considered) | `forget_floor` / `learn_floor` / `finish_floor` only make sense in order | **Accepted:** learning spans a whole turn of frames; one call can't do it. Documented as one sequence in the header |
| PicoA `world_map` | Special-general mixture (considered) | `map_print()` is console output inside the map | **Accepted:** printing needs the cell size and window, which only the map knows (§10) |
| PicoA `debug_console` | (design) | The start-up scan is triggered by the console (first monitor connection) | **Accepted for M2** (Daniel: start when the monitor opens); moves to `main`/`behaviour` when the robot starts on its own |
| PicoA `body` | Information leakage (avoided) | The clock offset could have been worked out in `pose` | **Avoided:** PicoB's clock is part of "PicoB as PicoA sees it"; `body_odom_time_us()` hides it |

### M2 review pass (5 Oct 2026)

Checklist §11 over `rangefinder`, `world_map`, `pose`, `surroundings`, `behaviour`
and the M2 parts of `debug_console`. No behaviour changed; host tests pass.

| Module | Red flag | What | Resolution |
|---|---|---|---|
| PicoA `rangefinder` | Repetition / nonobvious code | The floor geometry (sensor height ↔ angle ↔ distance along the ray) written out as `SENSOR_Z_M / sinf(…)` and `asinf(SENSOR_Z_M / …)` in six places; "is this floor zone learned" as `floor_down_rad[floor_index(i)] == 0` in three; a zone's lower edge as `(row - 3.0f) * ZONE_RAD`, which only works because zone centres are `(row - 3.5f)` | **Fixed:** `floor_range_m()`, `floor_angle_rad()`, `floor_learned()`; the lower edge is `zone_down_rad[ray] + ZONE_RAD / 2` |
| PicoA `world_map` | Repetition | "One update per cell per scan" written out in the hit loop and in `clear_along()` | **Fixed:** `first_this_scan()` |
| PicoA `behaviour` | Information leakage (considered) | `FREE_MAX_M` (2 m) restates the map's reach | **Accepted:** `map_free_distance()` already stops at the window's edge; the cap is behaviour's own choice, so every heading is scored over the same distance (the window's edge is 1.5-2.5 m away depending on where it was last centred) |
| PicoA `surroundings`, `debug_console` | Information leakage (considered) | They know which rows are floor rows (`rangefinder_first_floor_row()`, `rangefinder_floor_zones()`) to print the learned floor | **Accepted:** diagnostics only, like `link_stats()` in M1; the decisions stay in `rangefinder` |
| PicoA `rangefinder` | Nonobvious code (considered) | `HORIZON_ROW` and `FIRST_FLOOR_ROW` are both row 4 | **Accepted:** two facts about the same row (it learns the floor; its floor is too far for obstacles beyond 0.95 m); one name for both would hide one of them |
| PicoA `world_map` | (to watch, M5) | "Map changes so far: 1584" after one scan and a square: ~2 per frame, so change detection as it stands is noisy (cells at edges and far walls flipping between free and occupied) | **Open:** nothing uses changes until M5, which has to filter them (e.g. a change only counts if it persists, or clusters) |
| PicoA `world_map`, `behaviour` | (found on the robot) | Isolated unknown cells inside the free area (often in radial lines) stop `map_free_distance()`, so the robot undercounts open directions | **Open:** cause to find and fix before M4 (ROBOT_PLAN §14, next session) |

## Rework

### A0 — Measure, by analysis (9 Oct 2026)

| Module | Red flag | What | Resolution |
|---|---|---|---|
| PicoA `tof_motion`, the core-0 stack | Nonobvious code | The `static` locals assumed a 2 KB stack, but the SDK never enforces it, and `observations()` keeps a 3.2 KB `all[]` on the stack anyway: the main loop needs 4.2 KB, ~6 KB with interrupts. It works only because the stack can grow through both empty scratch banks (8 KB) | **Open until V3:** explicit 8 KB stacks before core 1 starts (core 1's stack is in that area); `tools/stack_depth.py` re-run then |

### A1 — helpers, clock wrap (9 Oct 2026)

| Module | Red flag | What | Resolution |
|---|---|---|---|
| `tracker`, `tof_motion`, `camera_motion`, `motion_sense` | Repetition | `atan2f(v[1], v[0])` for a bearing, five times | **Fixed:** `geom.h` `bearing_rad()` |
| `pose`, `tracker`, `behaviour` | Repetition | Time differences as `(float)(a - b) * 1e-6f` and `pose`'s `after()` | **Fixed:** `stamp.h` `stamp_us()`, `stamp_s()`. `robot_test`'s `seconds_between()` stays: 64-bit `absolute_time_t`, no wrap |
| `behaviour` | Repetition (ahead of need) | `wrap_angle()` private, while the map and ToF work need it too | **Fixed:** `geom.h` `wrap_pi()` |
| `tof_motion`, `camera_motion`, `rangefinder` | Repetition | Three `compare_*` functions with `qsort` for a median or an n-th value | **Fixed:** `stats.h`; `nth_value()` where one value is needed, `sort_values()` where `rangefinder` scans the sorted samples |
| `tof_motion`, `camera_motion` | Repetition | Blob → observation (sum directions, left / right, normalise, strength, sort by cells) written twice | **Fixed:** `change_grid_observations()` and `change_grid_biggest()`; each detector adds only its own (range; rolling-shutter time) |
| test fake clock | (test gap) | Started at 1: no test crossed the 32-bit wrap | **Fixed:** starts 30 s before; each test checked to cross it. No wrap bug found |
| `tof_motion` | Nonobvious code (considered) | `passed_us` uses 0 for "none", a real time once per 71.6 min | **Accepted:** misfires only for a frame stamped at exactly 0 µs; T-R removes `find_passing()` |
| `behaviour`, `robot_test` | Repetition | Driving to a distance or an angle (remaining, slowing near the end, settling) was written in both | **Fixed (T1):** `robot_test` removed; `behaviour`'s move and turn are the one copy |
| `behaviour` | Shallow module (considered) | Four actions share one small state machine (starting, running, settling) with a `switch` per action in `step()` | **Accepted:** one place says how every action starts and ends (motors, "done", "stopped: why"); a table of action structs would be more code for four cases |
| `pose`, `motion_sense` | Dead code | `pose_set_origin()` had no caller once the scan stopped moving the origin | **Fixed (T1):** removed, with the origin check in `motion_sense` |
| `test_behaviour` | (test gap) | The low box was asserted "seen" though it is found with about half the random seeds: the test passed by its seed | **Fixed (T1):** reported as KNOWN (MAP_DESIGN §8.1) until the map fix is tested on recordings |

### T2 — recording (10 Oct 2026)

| Module | Red flag | What | Resolution |
|---|---|---|---|
| `wifi_console` | Repetition | A second connection would have copied the console's connect, send, callbacks and drop | **Fixed:** `conn_t` for both; the callbacks get it via `tcp_arg()`, only receiving differs (keys vs nothing) |
| `recorder`, `rangefinder` | Nonobvious code | `rangefinder_raw()` returns tof.h's type, so it is declared in `recorder.c`, not in `rangefinder.h` (whose users, and their host tests, don't include the ULD) | **Accepted:** commented at both ends |
| `recorder` | Nonobvious code | The marks are taken inside `printf` (a stdio output), so nothing the record path calls may print: a lost connection only sets `cut`, printed by `recorder_update()` | **Accepted:** commented on `put()` |
| `recorder` | Information leakage (considered) | The record layouts live in `recorder.c` and again in `recording.ts` | **Accepted:** TELEMETRY_PLAN §2.1 is the contract, and the Node test decodes the firmware's own bytes from `test_recorder`, so a mismatch fails a test |
| `recorder` | (limit) | `TAKE_START`'s build time is `recorder.c`'s compile time, not the whole firmware's | **Open:** a git hash from CMake if it matters |
| `recorder`, `pose` | (limit) | A new ODOM is noticed by its time changing once per loop: two reports in one loop iteration keep only the second | **Fixed (after the robot test):** accepted at first on the belief that the loop runs far faster than 50 Hz; the recording showed 5 % of reports lost (loop iterations of 10-20 ms, sometimes more). `body` now keeps the last 16 reports numbered; `pose` and `recorder` take every new one |
| `recorder`, `wifi_console` | (design) | Over TCP, a few packets lost while the robot turned stalled the recording 1-6 s (TCP delivers in order and lwIP waits 1.5-3 s between retries) | **Fixed:** UDP datagrams, nothing retransmitted; losses counted by datagram numbers. The 48 KB buffer and the second TCP connection are gone |
| `recorder` | Nonobvious code | A frame's two halves may share a datagram or not, and `frames_failed` is counted per refused datagram holding a half, then capped at one per frame | **Accepted:** commented in `recorder_tof()`; `test_recorder` covers a refused frame |
| `recording.ts` | (limit) | Duplicate datagrams are found by (boot, number) in a set cleared at 100 000 entries | **Accepted:** ~50 datagrams/s, so 30 min of takes; repeats arrive within milliseconds |

### T3 — storage (10 Oct 2026)

| Module | Red flag | What | Resolution |
|---|---|---|---|
| `store` | (design) | DuckDB allows one writer per file: the server holding it open would lock out the `duckdb` command line | **Accepted:** opened per operation, one at a time; a take that can't be stored is noted, `npm run import` stores it later |
| `store`, `rangefinder` | Repetition | The world position of a target is computed on the PC as `rangefinder.c` and `cell_map.c` do on the robot | **Accepted:** the PC must not need the firmware; the zone angle and mounting come from `GEOMETRY`, and the floor landing at z ≈ 0 checks it. T5 (replay) runs the robot's own code |
| `store` | (limit) | Takes from before UDP have no arrival times: `started_at` is when their file was opened, the same for all takes in it | **Accepted:** three runs, all on 10 Oct; their order is kept by take number |

