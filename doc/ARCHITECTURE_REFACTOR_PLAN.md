# Architecture refactors — ideas and plan

A place for design improvements that are not part of the current feature work.
These are proposals, not approved implementation tasks. Daniel chooses an idea
and confirms its scope before code changes. Step 7 of
[DATA_FLOW_PLAN.md](DATA_FLOW_PLAN.md) stays separate.

## How to use this plan

- Each idea says what is wrong today, who should own the work, what stays the
  same, and how to check it.
- Prefer a small, useful boundary over extra modules or general frameworks.
- Keep only current facts and open work here. Completed work and review history
  go in [CHANGELOG.md](CHANGELOG.md).
- When implementing: a fresh reviewer after each substep and another at the end.
  Each reads Daniel's code review guide itself (AGENTS.md). Apply agreed fixes;
  record findings left open here with the reason.

## 1. PicoB: odometry should not distribute IMU data

**Status:** first idea to discuss; not scheduled or approved for implementation.

### What is wrong today

- `picoB/app/odometry.c` reads the IMU, calculates the robot's pose and tilt, and
  makes the raw IMU records sent to the PC. Those are two different jobs: working
  out motion and recording sensor data.
- `odometry_update()` is the only caller of `imu_read()`. It sends raw counts to
  its own `update_record()` before updating motion.
- `brain.c` takes those records through `odometry_imu_take()` and reads their
  lost/skipped counters through odometry too. A new IMU consumer would have to go
  through odometry or add another reader of the sensor.
- The name and interface hide that odometry is also the IMU's data distributor.
  Changing recording should not require changing the module that estimates pose.

### Proposed ownership

- PicoB's `main.c` reads each new IMU sample once and hands it to its consumers.
  Start with a small `imu_pipeline()` there, not a new dispatcher module.
- Odometry receives the measured sample and time. It owns heading, tilt, stillness
  and wheel-based motion, not the raw-record queue.
- An IMU recorder owns averaging raw counts, making records, the queue, and the
  lost/skipped counters. A focused `imu_recorder.h/.c` is a possible boundary;
  agree the interface before adding it.
- `brain` takes records and counters from the recorder and sends them as today.
- Future bump detection can receive the same sample if it needs it. Do not build
  that consumer or a subscription system now.

```
Now:
IMU → odometry ─┬→ pose and tilt → drive / brain
               └→ raw records → brain → PicoA → PC

Proposed:
IMU → main's imu_pipeline ─┬→ odometry → pose and tilt → drive / brain
                          └→ IMU recorder → brain → PicoA → PC
```

### Keep the behaviour unchanged

- Read at 416 Hz; average four consecutive samples into each 104 Hz record.
- Keep raw counts in the sensor's axes, without gyro-bias removal; preserve record
  timestamps, rounding, skipped-sample handling and queue overflow rules.
- Keep the mounting transform, heading, tilt, stillness and bias tracking unchanged.
- Keep wheel sampling tied to the same motion updates and drive control at 100 Hz.
- Keep the startup gyro calibration and IMU-failure motor lockout.
- Keep the wire payloads, sending rules and PC data unchanged. A change in sample
  rate or a new stop reaction would be a separate decision, not this refactor.

### Decisions before code

- Agree the odometry input: measured values, sample time and any sensor calibration
  information it still needs. Do not pass raw counts it does not use.
- Decide where IMU startup/calibration lives once odometry no longer reads it.
  Trace `odometry_init()`, `imu_init()`, `imu_calibrate_gyro()` and bias access first.
- Decide whether encoder reads stay inside odometry. Moving them is not required
  to separate IMU recording; do not widen the work without a clear benefit.
- Agree the recorder interface and queue ownership. Do not expose sensor-driver
  details to `brain` merely to remove the odometry dependency.

### Suggested substeps and checks

1. Capture current behaviour in host tests: motion from a fixed sample sequence;
   four-sample recording, timestamps, gaps and overflow. Check the existing driver
   and test seams before deciding how to feed measured samples.
2. Separate recording from odometry, keeping one sensor read and the existing
   output. Review the new boundary before changing the main loop.
3. Put the read-and-hand-over flow in `main.c`; remove the recording functions and
   counters from `odometry.h`. Review startup, failure and timing paths.
4. Run the full checks and firmware build, then the final fresh review. Update the
   architecture diagram's IMU and odometry flows. Decide with Daniel whether a
   robot run is needed; unchanged behaviour is first checked on the host.

**Files to start with:** `picoB/app/main.c`, `odometry.h/.c`, `brain.c`,
`picoB/drivers/imu.h/.c`, `common/link_messages.h`, and their host tests.
PicoB firmware changes; PicoA and PC should not need changes if the contract holds.

## 2. PicoA: make the world-model flow clear

**Status:** idea to discuss, not a reason to reorganize files now.

- The current flow is sensors → pose and cell map → behaviour and move checks →
  commands and link. The architecture diagram already shows frame-to-map and
  drive-start checks; it does not yet trace every running-stop path.
- Consider whether sense (`tof`, camera, `body`), world (`pose`, `cell_map`,
  `watch`), decide (`behaviour`, `move`, `commands`) and talk (`link_pc`, `wire`,
  `snapshot`) make useful responsibility boundaries.
- Trace real dependencies first. Only move responsibilities where that makes the
  flow easier to understand; avoid a folder-only rewrite or pass-through layers.

## 3. PicoB: clarify ownership of motor-stop reasons

**Status:** idea to discuss; do not move safety logic without tracing it.

- `drive` handles wheel control, tilt and wheel-following faults. `brain` handles
  the command timeout and reports stop reasons to PicoA.
- Check whether that split makes stop priority and reporting hard to follow.
  Compare keeping local faults in `drive` with a single owner for safety stops.
- Preserve motor-off timing, fault priority, command timeout and reported reasons.
  A new `reflexes` module is only justified if it makes those rules simpler.

## 4. Interfaces should take what they use

**Status:** apply to concrete problems, not as a whole-repo rewrite.

- A function takes a named piece of data it reads mostly whole, or a small input
  owned by its own header. A pipeline picks fields when the receiver needs only a
  small part of another module's data.
- Candidates to inspect: `drive_update()` receives odometry's pose as well as the
  wheel speeds, turn rate and tilt it uses; `watch` reads the stationary flag
  through `body_odometry()`.
- A focused `drive_input_t` may remove a dependency on `odometry.h`. Do not create
  wrappers for every hand-over or force a full, useful input apart.
- Agree each changed interface and its caller before implementation; test the
  promised behaviour rather than only the new adapter.
