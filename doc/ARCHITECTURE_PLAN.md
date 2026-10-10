# RoboCar — architecture plan

Status: **plan, not agreed yet; nothing here is implemented.** It takes everything
the robot does today as a spec, designs the firmware from scratch for it, compares
that with the code, and turns the differences into small milestones, each leaving
a working robot. It is the **master plan for the order of work** (§8.1) of four
plans written together:
- [CAMERA_MOTION_PLAN.md](CAMERA_MOTION_PLAN.md): `vision`, features tracked from
  frame to frame, milestones V*;
- [TOF_MOTION_PLAN.md](TOF_MOTION_PLAN.md): per-zone series, neighbours,
  `tof_quality`, milestones T*;
- [SURROUNDINGS_PLAN.md](SURROUNDINGS_PLAN.md): 10 cm voxels, ground as a layer,
  weighted evidence, milestones S*;
- this plan, milestones A* and the shared ones (R0, M-S).

Shared names, numbers, types and ownership were settled in a review of all four
plans. This plan states them (§1.2, §2.3, §5) and the others point here.
[RobotMotionTrackingAlgorithms.md](RobotMotionTrackingAlgorithms.md) was used as
inspiration, not as a spec.

## 0. The decisions in short

1. **Stay with C, the Pico SDK, a superloop and interrupts.** No RTOS, no ROS, no
   C++ (§7). The firmware is small, and the problems are elsewhere.
2. **A fixed pipeline: sense → estimate → world model → decide → act.** Plain
   structs flow one way between stages. A stage never calls a later one. `main.c`
   runs the stages in that order and says so.
3. **Every measurement carries its time and a confidence.** The time is a
   `stamp_t` on PicoA's clock, at the moment the measurement was taken. Confidence
   is a float from 0 to 1, except inside `tof_target_t`, where it is stored as
   0-255. Every pose-dependent value names its frame.
4. **Sensors publish frames; they don't hand them to one reader.** Any number of
   readers take the newest frame by sequence number. Today the console steals
   camera frames from movement detection, and movement detection gets ToF frames
   through the map module.
5. **Drivers touch hardware; everything above them is pure C** (no `pico/`
   includes). It runs unchanged in host tests and in a replay of recorded robot
   runs on the Mac.
6. **`view` owns all sensor geometry:**
   - the VL53 zones and the floor patch each zone sees;
   - the camera intrinsics;
   - where each sensor is mounted.

   Ego-motion is predicted from `pose` and `view`. Its compensation is an explicit
   step with its own fields in the data, not a module of its own.
7. **One owner of the wheels (`activity`)**, with a safety pass on every command.
   Today `behaviour` and `robot_test` both drive, and the console keeps them apart.
8. **Nothing blocks the core-0 loop.** Text and binary records go into rings that
   USB and WiFi drain.
9. **Core 1 runs `vision`, decided.** It moves there in V3, after V1 has measured it
   on core 0. The reason is CPU: one vision step takes 8-25 ms, which alone breaks
   the core-0 budget. Core 1 is fed through mailboxes of plain structs and shares
   no mutable state with core 0.
10. **Recording comes early (R0, right after A3), and there is only one recording
    path.** Records go over a second TCP connection, with one `replay` tool on the
    Mac. One measurement session (M-S) follows it and settles the numbers all three
    companion plans depend on.
11. **RAM is the binding constraint.** The end state is ~380 KB of 512 KB. S3 (the
    new map, −135 KB) must land before V1. Every milestone's robot test checks
    "RAM free ≥ 40 KB".

---

## 1. The spec

### 1.1 What the robot does (today's functions as requirements)

| # | Function | Requirement (rate, latency, behaviour) |
|---|---|---|
| F1 | **Link** | UART 1 Mbaud, COBS + CRC, versioned messages; never blocks; counters for diagnosis |
| F2 | **Wheel control** (PicoB) | 100 Hz speed loop per side, gyro-trimmed turn rate, ramps, limits 0.2 m/s / 1.5 rad/s |
| F3 | **Odometry, tilt** (PicoB) | IMU at 416 Hz, encoders on 4 wheels; heading from the gyro with bias re-measured when still; ODOM at 50 Hz |
| F4 | **Safety stops** (PicoB) | No DRIVE for 250 ms, wheel not following for 1 s, tilt > 15°: motors off until PicoA switches them on |
| F5 | **Pose at any moment** (PicoA) | `pose_at(t)` for any t in the last ~1.3 s, on PicoA's clock, in the map frame, with pitch and roll |
| F6 | **Surroundings** | VL53 8 × 8 at 15 Hz → placed rays → a 4 × 4 m map of 10 cm cells; a place is drivable when its ground is seen and both 10 cm cells above it are free (robot **12 cm** tall); cells change only when measured |
| F7 | **Start-up scan** | 390° turn, map printed, turn to the most open heading |
| F8 | **Watching, turning to look** | Watch for a target, motors on; target leaving → turn to where it will be (≤ 90°, 1 rad/s), on again if it keeps going |
| F9 | **Camera movement** | 160 × 120 at ~25 fps, own exposure (10 ms steps), row times; features tracked from frame to frame; movement while still and while turning |
| F10 | **ToF movement** | Movement against a background kept per heading, so it **works while the robot turns**; no "learning the view" after a stop |
| F11 | **Tracker** | One target, direction, range, angular speed in the world frame; leaving / exited / stopped |
| F12 | **Console** | USB and WiFi, single keys, status, map, frames; losing it doesn't stop the robot |
| F13 | **Robot tests** | Square, drift, turns, straight; results kept after early stops |
| F14 | **Bring-up firmwares** | Share the drivers; keep building; viewer on the PC |
| F15 | **Host tests and replay** | Every logic module has a host test; stubs fail before init; `./run_tests.sh` stops at the first failure; recorded clips replay through the same code |

Coming next (they shape the architecture now): M4 driving to a target with a
Safety guard; M5 staleness, full-turn check, map changes; M6 map correction.

### 1.2 Numbers every plan uses

| What | Value | Note |
|---|---|---|
| Robot | **12 cm tall**, 23.5 cm wide × 19 cm long, front edge 9.5 cm ahead of the centre | confirmed by Daniel; ROBOT_PLAN §3 says 10 cm high: fixed there in S3 |
| VL53 | 7 cm up, 2.5 cm ahead of the centre, 3 cm right, level; zones 5.625° | `rangefinder.c` |
| VL53 rows | **0-based, row 0 at the top, as in the code.** Row r's centre is (r − 3.5) × 5.625° below level; rows 0-3 look up, 4-7 down. Floor along the centre ray: row 4 ≈ 1.43 m, row 5 ≈ 48 cm, row 6 ≈ 29 cm, row 7 ≈ 21 cm | README's "rows 6/7/8" (1-based) get rewritten |
| Camera | 8.5 cm up, 2.5 cm ahead of the centre, f ≈ 160 px, HFOV 53.1°, VFOV 41.1° | ROBOT_PLAN §3 |
| A walker | Both sensors see only up to ~46-49 cm at 1 m and ~85-90 cm at 2 m: **legs**, not a torso. All simulations model two legs with a gait | 7 cm + 1 m · tan 22.5°; 8.5 cm + 1 m · tan 20.5° |
| Height bands | ground G = −7…+3 cm, L1 = 3-13 cm, L2 = 13-23 cm | SURROUNDINGS_PLAN; the ground's top at 3 cm is for Daniel to confirm (§9) |
| Drivable | G floor **and L1 free and L2 free** | Daniel |
| ToF frame time | `t_us` = middle of the integration = INT-pin interrupt − 33.3 ms − `TOF_PROC_US`; the integration runs from `t_us − 33.3 ms` to `t_us + 33.3 ms` | `TOF_PROC_US` starts at 0 and is measured in M-S (door-edge turns) |
| Camera row time | `row0_us` = middle of row 0's exposure; row r at `row0_us + r · line_us` | |

### 1.3 Budgets (Pico 2: RP2350, 2 × Cortex-M33 at 150 MHz with a single-precision FPU, 520 KB SRAM)

**Memory, PicoA** (from the build: `arm-none-eabi-size`, `nm`). .bss **292 KB** of
the 512 KB main SRAM, **219 KB free** since the old map and its floor learning were
removed (the table below was written with them: subtract their 135 KB from its
totals). No `malloc` is linked, so that is the whole RAM use; lwIP has its own
static pools. Flash: 488 KB of 4 MB.
PicoB uses 4.5 KB.

**Stack, PicoA core 0** (A0, `tools/stack_depth.py` on GCC's call graphs). The SDK
default of **2 KB** is only nominal: nothing enforces it, and the stack may grow
down through both 4 KB scratch banks to `__StackLimit` (8 KB) while core 1 is
unused. The worst case today is **~6 KB**:
- main loop 4.2 KB, of which `observations()` (`tof_motion.c`) is 3.2 KB: its local
  `all[RANGEFINDER_RAYS]`;
- lwIP in the lowest-priority interrupt 0.75-1.3 KB, USB 0.25 KB (handlers of one
  priority don't nest), two exception frames 0.2 KB.

So the main loop overflows the nominal 2 KB today and works only because the
scratch banks are empty. That holds until core 1 starts: its stack is in scratch
X, exactly where core 0 overflows to. **V3 must set both stacks before launching
core 1** (as planned: 8 KB each, ~2 KB margin over today's 6 KB); re-run the tool
then, with the `static` locals back on the stack.

| Change | KB | When |
|---|---|---|
| Today | 385 | |
| Telemetry ring (text 16 KB replaces the WiFi `out` 16 KB; records 16-32 KB) | +16…32 | A3 |
| R0: one image staging copy (19.2), record ring, the second TCP connection's lwIP memory | +30…40 | R0 |
| ToF: `tof_frame_t` producer + 2 reader copies, per-zone state, chains, the heading background | +20 | T1-T-R |
| Surroundings: −kept frames 63.6, −cells 51.2, −floor samples 32.8, +13 | −135 | S3 |
| Camera: half images 9.6, features 19, a ring of 4 × 6.6, a reader copy 6.6, attitude history 1.5 | +63 (net +33 once V4 removes `camera_motion`: learning 24, grid ~6) | V1-V4 |
| Explicit 8 KB stacks for both cores, out of the scratch banks into main SRAM | +16 | V3 (when core 1 runs) |
| **Total after everything** | **~380** | ~130 KB left |

**Order constraint:** without S3's −135 KB, A3 + R0 + the ToF work + V1 come to
~500 KB, leaving 12 KB for the heap. V3's ring and stacks would then take it to
~545 KB. So **S3 lands before V1.** Every milestone's robot test includes "`p`:
RAM free ≥ 40 KB"; less fails the milestone. Rule: everything is static, sizes are
known at build time, no `malloc` outside lwIP's own pools.

**Time** (targets; A1's loop timer shows today's numbers):

| Path | Today | Target |
|---|---|---|
| ToF: measurement → frame available | middle of the integration + ~33 ms + processing + ≤ 5 ms polling; the time is estimated from the read | stamped from the INT pin (§1.2) |
| ToF frame → map + movement updated | not measured | < 5 ms |
| Camera frame → `vision_frame_t` | — | ≤ 25 ms per frame with 96 features (V1's pass line), on core 1 from V3 |
| Decision → DRIVE on the wire | 0-50 ms (DRIVE goes out every 50 ms, not on change) | ≤ 1 loop iteration |
| Core-0 loop iteration | not measured; building and printing the map ≫ 250 ms (hence motors off) | typical < 2 ms, max < 20 ms, never > 50 ms |
| ODOM interpolation error | 50 Hz, linear | fine: at 3 rad/s² the yaw error between samples is ~0.01° |

Compute for `vision`, from CAMERA_MOTION_PLAN (FAST at candidate points + ZNCC
matching):
- the plan's estimate is 1.2-1.8 M cycles per frame;
- the review's worst case is 2.5-3.5 M cycles (17-23 ms), because without SIMD a
  ZNCC product costs ~3 cycles.

At 25 frames/s that is up to ~60 % of a core, so it can't share core 0 with the
loop: hence core 1 (§2.4).

**Link and WiFi bandwidth:** the UART carries < 5 % today. Over WiFi, records are
about:
- `TOF_RAW` ~3.1 KB × 15 Hz ≈ 47 KB/s;
- ODOM ~2.3 KB/s;
- `vision_frame_t` a few KB × 25 Hz;
- full images 19.2 KB each, every Nth frame.

R0's first robot test measures what the WiFi really carries before anything relies
on it.

### 1.4 Quality requirements

- **Fully autonomous:** no PC needed to run. The PC only shows, records and replays.
- **Judged by the log:** Daniel tests on the robot and pastes the log, so every
  decision must be visible in text, and reproducible from a recording.
- **Safe on its own:** PicoB stops by itself (F4). A hung PicoA must never leave the
  robot driving.
- **Keep it simple** (ROBOT_PLAN "How we work"): an algorithm fits in a few plain
  sentences, written before the code.
- **Deep modules** (APOSD), host tests for every logic module, red-flag reviews.

---

## 2. First-principles design

### 2.1 Rules

1. **Five stages, one direction.** Sense (drivers → typed frames), estimate (pose,
   `vision`, ToF series), world model (surroundings, movement), decide
   (activities), act (body). A stage reads what earlier stages published; reading
   `pose_at()` from any later stage is allowed. Back-channels are explicit requests
   only.
2. **Time and confidence on every measurement.** The time is a `stamp_t`, taken
   when the measurement was made, not when it was read. Confidence is a float from
   0 to 1, except inside `tof_target_t`.
3. **Frames of reference in names** (`_robot`, `_map`, sensor frames). Only `view`
   and `pose` convert between them; `movement` is the only module that turns
   observations into the map frame.
4. **Drivers are the only code that touches hardware.** Everything above them is
   pure C over plain structs, so it compiles for the Mac without fakes, apart
   from the edges.
5. **Readers don't consume.** A producer publishes its newest value with a sequence
   number. Each reader keeps the last number it took and gets newer values only.
6. **Nothing blocks core 0 for more than a few ms.** Printing writes into a ring.
   Long jobs are split up, deleted (S3 removes the scan's batch map build) or run
   on core 1 (`vision`).
7. **One owner per actuator.** Only `activity` calls `body_drive` / `body_motors`,
   after the safety pass.
8. **Singletons for hardware owners; state structs for algorithms** that could run
   twice (two sensors, replay A/B), as `change_grid_t` already does.

### 2.2 The pipeline

```
 SENSE                      ESTIMATE                         WORLD MODEL              DECIDE              ACT
 ─────                      ────────                         ───────────              ──────              ───
 VL53 ─► tof_sensor ─tof_frame_t─► tof_motion ──motion_obs_t────┐
  (INT time, tof_quality: │      (ToF plan: zone series,         │
   2 credible targets,    │       heading background, works      ▼
   weak_mm, ambient)      │       while turning; pose_at)    movement ─movement_event_t─► activity ─cmd─► safety ─► body ═link═► PicoB
                          │                                (tracker, fusion,  ▲           (scan, watch,  ▲                   │
 HM0360 ─► camera ─image_t─► vision (core 1) ─vision_frame_t─► camera_movers ─┘           investigate,  │                   │
  (row0_us, line_us,      │  (camera plan; ego part from          (motion_obs_t)           tests)       │                   │
   exposure)              │   attitude + view)                                               ▲          │                   │
                          │                                                                  │ queries  │                   │
                          └─► view_place_tof ─ray_meas_t[]─► surroundings (voxels, G/L1/L2, evidence)    │                   │
                                                    drivable? free distance? stale? changes ─────────────┘                   │
 PicoB ODOM ════════════════════════► body ─► pose ◄── pose_at(t) for every stamped frame ◄──────────────────────────────────┘
                                                 └──► attitude queue ─► core 1 (vision's own history, pose_interp.c)

 every stage ─► telemetry: text ring ─► USB / WiFi console;  record ring ─► second TCP connection ─► pc/robot saves ─► replay
```

`main.c` becomes the pipeline written out: `sense()`, `estimate()`, `model()`,
`decide()`, `act()`, `report()`, each a few calls in a fixed, commented order.

### 2.3 Shared data types

These are the interfaces between stages; the companion plans use them as written
here. Each one is a value: no pointers into another module's buffers, apart from
the image, which is lent and given back.

**Time.** Every time in every struct is a `stamp_t`; no raw `uint32_t t_us` in new
code.

```c
// common/stamp.h — PicoA's clock, µs (time_us_32()); wraps every 71.6 min.
typedef uint32_t stamp_t;
static inline int32_t stamp_us(stamp_t later, stamp_t earlier) { return (int32_t)(later - earlier); }
static inline float   stamp_s(stamp_t later, stamp_t earlier)  { return (float)stamp_us(later, earlier) * 1e-6f; }
```

**Pose.** `pose_t` gets its time, roll and turn rate in A2. Core 1 keeps its own
attitude history, fed by a queue, using the same interpolation code
(`pose_interp.c`, pure, shared).

```c
typedef struct {
    stamp_t t_us;
    float x_m, y_m, yaw_rad;      // map frame; yaw unwrapped, + = left
    float pitch_rad, roll_rad;
    float v_mps, w_radps;
} pose_t;
bool pose_at(stamp_t t, pose_t *p);
bool pose_rotation(stamp_t from, stamp_t to, float *yaw_rad, float *pitch_rad, float *roll_rad);
```

**ToF confidence: one model** (`tof_quality.c`, pure, owned by TOF_MOTION_PLAN, used
only by `tof_sensor`; the map and movement read the numbers, never the status):

```
confidence   = status_weight(status) × faint_factor(signal, range, row)   // 0-1
status_weight: 5 → 1.0; 6 → 0.5; 9 → 0.5; 10 → 0.5 (tentative); 12 → 0; others → 0.
               M-S's data changes the table once, recorded in the CHANGELOG.
faint_factor:  1, except the echo guard (only if M-S shows the echo with a good status):
               15-35 cm, signal < 15 kcps/SPAD, a credible target behind → 0.1
sigma_eff_mm = max(sigma_mm, 3 + 0.005 × range_mm) × (status == 9 ? 2 : 1)   // its own field
CREDIBLE     = confidence ≥ 0.25        // the same threshold in tof_motion, view_place_tof, the map
```

**Sigma is not folded into the confidence.** It travels as its own number, and each
consumer uses it in its own way:
- the ToF plan: a z-test;
- the map: a margin where free space stops, and the weight × 0.5 when
  `sigma_eff` > 50 mm.

```c
// tof_sensor.h — one VL53 frame, robot order (row 0 top, column 0 left).
typedef struct { uint16_t range_mm, sigma_mm, signal_kcps; uint8_t status, confidence; } tof_target_t; // 8 B; confidence 0-255
typedef struct {
    uint32_t seq;                  // counts frames: a gap means one was missed
    stamp_t t_us;                  // middle of the integration (§1.2)
    uint16_t ambient_kcps[64];     // for tof_quality_clear_m (sunlight)
    uint8_t  n_targets[64];        // credible targets kept, 0-2
    tof_target_t target[64][2];    // the two nearest CREDIBLE targets, nearest first
    uint16_t weak_mm[64];          // nearest non-credible target (echo, faint floor), 0 = none
} tof_frame_t;                     // ~1.4 KB
static inline float tof_conf(const tof_target_t *t) { return (float)t->confidence / 255.0f; }
bool tof_sensor_next(tof_frame_t *f, uint32_t *seen_seq); // newest frame after *seen_seq
```

`weak_mm` exists because the ToF plan drops non-credible targets, while the map
needs "a faint target inside the floor patch" to decide that a missing floor isn't
a drop. Raw frames with all 4 targets exist only as the `TOF_RAW` record.

**Camera.** `image_t` is the driver's lent image. `vision_frame_t` and the module
name `vision` (read by `camera_movers`) are defined in CAMERA_MOTION_PLAN; this
plan asks only that the frame:
- is made of values, with no pointers into the image;
- is stamped with `stamp_t`;
- keeps the robot's own motion apart from the measured motion;
- carries a summary per region that any reader can use (movement now; edges,
  landmarks, visual odometry later).

```c
typedef struct {
    uint32_t seq;
    const uint8_t *pixels;    // 160 × 120, top row first, as the robot sees it; valid until camera_release()
    stamp_t row0_us;          // middle of row 0's exposure; row r at row0_us + r * line_us
    float line_us, exposure_us, gain;
    bool settling;            // brightness not comparable with earlier frames
} image_t;
```

**Placed measurements** (what SURROUNDINGS_PLAN consumes; general on purpose, so
another range sensor could feed the map later):

```c
typedef struct {
    stamp_t t_us;
    float origin_map[3], dir_map[3]; // dir includes pitch and roll
    float half_rad;                  // cone half-angle (VL53 2.8125°); 0 = a true ray
    float range_m;                   // < 0: nothing up to max_m
    float max_m;                     // 1.0 m (tof_quality_clear_m: less in sunlight)
    float sigma_m;                   // sigma_eff
    float confidence;                // 0-1; "nothing" rays: 0.5
    float weak_range_m;              // from weak_mm, < 0 none
} ray_meas_t;
int  view_place_tof(const tof_frame_t *f, const pose_t *p, ray_meas_t *rays, int max);
void surroundings_add(const ray_meas_t *rays, int n, const pose_t *robot);
```

**Observations:** one struct for both sensors, with flags instead of sentinels or
NAN. Both detectors produce observations **while the robot turns**. Only
`movement` converts them to the map frame.

```c
typedef struct {
    stamp_t t_us;
    uint8_t sensor;                  // OBS_TOF, OBS_CAMERA
    float where_robot[3];            // unit, robot frame AT t_us
    float left_rad, right_rad;       // robot frame, + = left
    bool has_range;  float range_m, range_sigma_m; float point_robot[3];
    bool rate_known; float rate_radps, rate_sigma_radps; // WORLD frame (the robot's own turn removed), + = left
    bool range_rate_known; float range_rate_mps;         // + = going away
    float strength; uint16_t cells;
    float confidence;                // 0-1
} motion_obs_t;
```

**Events.** Detectors may keep internal events of their own (`tof_event_t`, stamped
with `stamp_t`); only `movement` emits `movement_event_t`. These are by value and
in the map frame, so a turn doesn't invalidate them:

```c
typedef enum { MOVE_NEW, MOVE_SEEN, MOVE_LEAVING, MOVE_EXITED, MOVE_STOPPED, MOVE_REMOVED,
               MOVE_MAP_CHANGE,               // only from surroundings_next_change()
               MOVE_PASSED_WHILE_LEARNING     // deleted in T-R, when the ToF stops "learning"
             } movement_kind_t;
typedef struct {
    movement_kind_t kind;
    uint8_t sources;          // MOVE_FROM_TOF | MOVE_FROM_CAMERA | MOVE_FROM_MAP
    stamp_t t_us;             // when last seen
    float bearing_map_rad, up_rad;
    bool has_range;  float range_m;
    bool rate_known; float rate_radps; // world frame, + = to the left
    int8_t side;              // +1 left, -1 right, 0 inside
    float confidence;         // 0-1
} movement_event_t;

// activity.h — one command per loop, from the one activity that runs.
typedef struct { float v_mps, w_radps; bool motors_on; } drive_cmd_t;
```

### 2.4 Cores, interrupts, timing

| Where | What | Notes |
|---|---|---|
| Interrupts, core 0 | UART0 RX → link ring; **GPIO IRQ on the ToF INT pin (new: timestamp only)**; cyw43 / lwIP background; USB | Each one only stores data and a time. No logic, no printing |
| Interrupts, core 1 (from V3) | DMA_IRQ_1, the camera's frame done | The camera driver is initialised on core 1, so its handler runs there; check that cyw43 never enables DMA_IRQ_1 on core 0 |
| Core-0 loop | body (link in/out), tof_sensor (SPI read, a few ms per 66 ms), pose, tof_motion, view placing, surroundings, movement, activity, safety, telemetry draining, console | Typical < 2 ms per iteration, max < 20 ms |
| Core 1 (V3) | `vision`: image → `vision_frame_t`, with its own attitude history | Images and attitude arrive through single-producer / single-consumer mailboxes, and frames go back the same way. **Core 1 never calls lwIP, cyw43 or `printf`**: it writes records into its own ring |

- **The camera driver stays core-agnostic.** The bring-up firmware keeps it on
  core 0, and the robot firmware moves it to core 1 in V3.
- **The DRIVE keep-alive is not moved to a timer; it stays in the loop.** PicoB's
  250 ms timeout exists to catch a hung brain, and resending DRIVE from an
  interrupt would hide one.

### 2.5 PicoB

PicoB's structure (`drive`, `odometry`, `brain`, `main` as the loop) is already
what a fresh design would give. Two small changes: ODOM carries the IMU sample's
time, not the send time (`brain.c:101`), and the hardware watchdog (§7, I24).

### 2.6 Telemetry, console, recording

**Recording (the Records and host tool parts below) is replaced by
[TELEMETRY_PLAN.md](TELEMETRY_PLAN.md) (10 Oct 2026):** one take per action, no
images yet, `.rec` files and DuckDB on the PC. The text ring and the console stay.

- **Text:** one ring that `printf` writes into (via an stdio driver). USB and the
  WiFi console connection drain at most N bytes per loop. When the ring is full,
  whole lines are dropped and counted; nothing waits.
- **Records** (R0):
  - format: `[type u8][len u16][payload]`, COBS + CRC (`frame.c`, split out of
    `link.c`);
  - transport: a **second TCP connection** (port 4212), so a 19 KB image never
    queues ahead of the console's text; the USB fallback is the same byte stream
    on the CDC port in a recording mode;
  - key **`R`** switches recording on and off;
  - types: `ODOM`, `TOF_RAW` (all 4 targets, the bring-up `TOF3` layout),
    `TOF_FRAME`, `IMAGE` (every Nth frame), `MARK` (keys and log lines), and later
    `VISION`, `OBS`, `EVENT`.
- **One host tool:** `build/replay file.rec [--stats|--tof-stats|--vision|--map|--pgm dir|--csv]`.
  The companion plans' `vision_replay`, `tof_stats` and dump scripts become modes
  of it. As `tof_motion` and `movement` become pure, `replay` runs them too and
  prints the same log lines as the robot.
- **Console:** keys become activity requests and printouts. It only reads
  published values: it never takes frames from other readers and never waits for
  them.

### 2.7 Testing

- Host tests stay one module per test, `#include "../x.c"`, with fakes only for
  drivers and the clock. Pure modules need no clock fake at all: time comes in
  the data.
- **The fake clock starts just before the 32-bit wrap,** so every test crosses it.
  Today it starts at 1, and `time_us_32()` wraps after 71.6 minutes on the robot.
- **Recorded clips** from R0 on: a known case and the log lines it should give
  become a regression test (S5's golden recordings run continuously from R0 on).
- **Simulations model a walker as two legs with a gait** (§1.2).
- **Bring-up firmwares build in every milestone** that touches a driver.

---

## 3. The architecture as it is

```
 Interrupts: UART0 RX → link ring   DMA_IRQ_1 → camera buffers   cyw43/lwIP background   USB
 One core. main.c loop, every iteration, in this order:

 start_up ─(no USB)─► wifi_console_start, then behaviour_scan
 body_update ◄══link══► PicoB (DRIVE every 50 ms, MOTORS until echoed, ODOM, STATUS, LOG)
 pose_update ◄── body_odom (history 1.3 s, clock offset)
 surroundings_update ─► rangefinder_poll ─► tof driver (ULD)      [owns the ToF frames]
                     ─► rangefinder_scan (floor/obstacle) ─► world_map_add_scan(pose_at(t))
 camera_update (exposure)
 motion_sense_update ─► surroundings_last_frame ─► tof_motion ─► change_grid ─► tracker ─► leaving_t
                     ─► camera_frame (takes it) ─► camera_motion ─► change_grid  (+ camera_hold_exposure)
                     ─► log lines (printf)
 behaviour_update ─► body_drive/body_motors, pose, world_map, surroundings, motion_sense_leaving
 robot_test_update ─► body_drive/body_motors
 wifi_console_update (stdout driver, 16 KB buffer)
 debug_console_update ─► keys → behaviour / robot_test / body; prints map, frames, camera (takes a frame, waits ≤ 500 ms)
```

**What is good and stays:**
- `link` (deep: framing, CRC, buffering, counters behind three calls) and the
  versioned messages in one header.
- `body` (PicoB as PicoA sees it, including PicoB's clock) and PicoB's `brain`, its
  mirror.
- PicoB as a whole: an independent safety layer, simple and tested.
- The camera driver: own exposure control, flicker steps, row times.
- `rangefinder` hides the zone order, the mounting and the floor well. Its parts
  move (§5), but what it hides stays hidden.
- `change_grid` as a state struct with a small interface; `robot_test` as a table.
- The habits: units in names, stubs that fail before init, `-Wdouble-promotion`,
  REDFLAGS reviews, headers that promise.

---

## 4. Gap analysis

| Topic | First-principles design | Today | Gap |
|---|---|---|---|
| Who gets sensor data | Published frames, any number of readers | Camera: one reader (`camera_frame` hands a frame out once), and the console takes frames. ToF: `surroundings` polls and others read `surroundings_last_frame()` | Multi-reader publishing (A2) |
| Timestamps | Measurement time at the source | Camera rows ✓. ToF estimated from the read time (`rangefinder.c:88`). ODOM stamped when sent (`brain.c:101`). The map stamps cells with the time of insertion (`world_map.c:175`) | INT pin, sample time, scan time (A2) |
| Confidence | On every measurement | ToF reduced to "sure / unsure" in the adapter; sigma, signal, ambient and the second target are dropped | `tof_quality`, `tof_frame_t` (A4 + T1) |
| Geometry | One `view` module | ToF geometry in `rangefinder`, camera geometry (`FOCAL_PX`) inside `camera_motion` | `view` (A4 + T1; camera intrinsics in V2) |
| ToF while turning | Background per heading | Detection only while still; "learning the view" after each stop misses targets leaving meanwhile | T-R (needs A2's time stamps) |
| Ego-motion | Explicit step from attitude + `view` | None: the detectors run only while still; the tracker resets on any movement | V2, T-R |
| World model | Voxels, ground layer, evidence weights | 4 layers above 2 cm, floor learned in `rangefinder`, miss counters | S1-S3; architecture: `ray_meas_t` in, queries out |
| Movement → decisions | Events by value, map frame | `leaving_t` in the sensor frame with a sentinel; `tracker_target()` pointer valid until the next call | `movement` (A6 + T4 + V6) |
| Actuation | One owner + safety pass | `behaviour`, `robot_test`, `debug_console` all call `body_*`; exclusivity by convention | `activity` (A5) |
| Blocking | Nothing blocks core 0 | Map build + print ≫ 250 ms (behaviour switches motors off for it); `c` waits up to 500 ms; `printf` up to 5 ms per call | Telemetry ring (A3); the batch map build goes in S3 |
| Diagnosis | Records + replay | Text log only; host tests are synthetic | R0, M-S |
| Cores | `vision` on core 1 | Core 1 unused | V3 |
| Budgets | Known and printed | RAM and stack computed from the build (A0): 385 of 512 KB used; the core-0 stack ~6 KB worst case against a nominal 2 KB; loop time not printed | A1's loop timer; explicit stacks in V3; the order constraint in §1.3 |
| Long runs | Tested across the clock wrap | Done in A1: the host tests' fake clock and both Picos start 30 s before the wrap | A1 |
| Restart | Watchdog, safe start | No watchdog; a reset without USB starts the scan (the robot moves by itself) | A3 |

---

## 5. Deep modules

A deep module has a small interface and hides a lot behind it. Proposed module set,
PicoA (PicoB unchanged):

| Module | Interface (sketch) | Hides | From today |
|---|---|---|---|
| `link` (common) | `link_init`, `link_send`, `link_receive`, `link_stats` | UART, IRQ ring, COBS, CRC, sequence numbers | Keep. Framing split into `frame.c` for the records |
| `body` | `body_update`, `body_connected`, `body_odom`, `body_motors`, `body_drive` | Protocol, greeting, keep-alive, MOTORS retries, PicoB's clock | Keep. DRIVE sent on change. Called only by `activity` |
| `tof_sensor` | `tof_sensor_init`, `tof_sensor_next(&frame, &seq)` | ULD, SPI, zone order and the 90° rotation, INT timestamp, confidence via `tof_quality` | The poll half of `rangefinder` plus `drivers/tof` |
| `tof_quality` (ToF plan) | `tof_quality_target(...)`, `tof_quality_clear_m(ambient)` | The status table, the echo guard, `sigma_eff` | New, pure |
| `camera` | `camera_init`, `camera_next(&image, &seq)`, `camera_release` | PIO/DMA, registers, exposure policy, row times, buffer lending | Keep. `camera_frame` → multi-reader `camera_next` |
| `view` | `view_tof_dir(zone, dir)`, `view_floor_patch(zone, pitch, roll, &d1, &d2)`, `view_pixel_dir(u, v, dir)`, `view_project(dir, &u, &v)`, `view_origin(sensor, xyz)`, `view_place_tof(frame, pose, rays, max)` | Mounting of each sensor, zone angles, camera intrinsics (f, cx, cy, k1, time offset; filled in by `vision_calibrate`), robot ↔ sensor frames | New: from `rangefinder` and `camera_motion`. `view_floor_patch` replaces `rangefinder_floor_limit_m`. The camera plan's `vision_direction()` / `vision_pixel()` become `view_pixel_dir()` / `view_project()` |
| `pose` | `pose_update`, `pose_at`, `pose_now`, `pose_rotation`, `pose_set_origin` | Odometry history, interpolation (`pose_interp.c`, shared with core 1), origin; later full-turn check, map correction, EKF | Keep. Add `t_us`, roll, rates and `pose_rotation` |
| `vision` (camera plan) | see CAMERA_MOTION_PLAN | Pyramid, corners, matching, IDs, the ego part | New |
| `camera_movers` (camera plan) | `vision_frame_t` → `motion_obs_t` | Which features move against the world | Replaces `camera_motion` in V4 |
| `tof_motion` (ToF plan) | `tof_motion_add(&frame) → motion_obs_t` | Per-zone series, the heading background, neighbour correlation, edge times | Rewritten T1-T4 |
| `surroundings` (surroundings plan) | `surroundings_add(rays, n, robot)`, `surroundings_drivable(x, y)`, `surroundings_free_distance(...)`, `surroundings_cell(x, y, z)`, `surroundings_staleness(...)`, `surroundings_next_change(&ev)`, `surroundings_print()` | Voxel size, ground band, evidence, window, ray walking, change detection | Merge of `surroundings` + `world_map` + `rangefinder`'s floor classification; floor learning goes completely (S3) |
| `movement` | `movement_add(obs, n)`, `movement_next(&ev)`, `movement_target(&copy)` | Tracker, leaving rules, fusion of camera and ToF, react-or-log, the conversion to the map frame | From `motion_sense` + `tracker`; the log wording goes to the console |
| `activity` | `activity_start(kind)`, `activity_stop(why)`, `activity_update()`, `activity_running()` | One activity at a time, switching motors, settling, the safety pass, stop reasons | New. Scan / watch (from `behaviour`) and the tests (from `robot_test`) become activities, each a table entry like today's tests |
| `telemetry` | `printf` (stdio driver), `telemetry_record(type, body, len)`, `telemetry_drain()`, `telemetry_stats()` | Rings, transports, drop policy, framing, the second connection | New: from the WiFi console's buffer and the USB `printf` |
| `console` | `console_update()` | Keys, printouts | `debug_console` without the policies (it no longer stops motors to print) |

**Shallow or muddled today, and what happens to them:**
- `surroundings` + `world_map`: two modules for one idea ("what is around the
  robot"); `surroundings` also owns the ToF frames for everyone. They merge, and
  the frames go to `tof_sensor`.
- `motion_sense`: gating, routing, event conversion and ~17 `printf`s. It is split
  into `movement` (decisions) and the console (words). Its gating ("watch only
  while still") goes with T-R and V5.
- `motion.h` (turn and speed profiles) vs `motion_sense` / `tof_motion` /
  `camera_motion`: "motion" means both the robot moving and the scene moving.
  Rename it `move_profile.h`, and use "movement" for the scene.
- `motion_obs_t` becomes the one observation struct of §2.3.
- `change_grid` stays while the camera's block detector and the old ToF detector
  exist; it goes with them (V4, T1-T2).

Ego-motion is deliberately **not** a module: the attitude history plus
`view_project()` already hide all it needs. A separate `ego_motion` module would be
a pass-through, so the step is explicit in the data instead.

---

## 6. Red flags found

New flags only; REDFLAGS.md's resolved ones are not repeated. File references are
to the current code.

| Module | Red flag | What | Proposed fix (milestone) |
|---|---|---|---|
| `debug_console.c:107` `print_camera` | Information leakage / temporal coupling | `camera_frame()` hands out each frame once, so `c` takes a frame from movement detection. It also waits up to 500 ms in the loop: without a camera that is longer than PicoB's 250 ms DRIVE timeout | Multi-reader `camera_next`; console prints the last published image, never waits (A2) |
| `surroundings.c:65`, `motion_sense.c:89` | Pass-through / temporal decomposition (came back) | Movement detection gets ToF frames through the map module because that module polls first. REDFLAGS M2 accepted `surroundings` as a pass-through; this second job is new | `tof_sensor` publishes frames (A2) |
| `motion_sense.c:90` | Nonobvious code | "New frame?" is `f->t_us == last_frame_us` | Sequence numbers (A2) |
| `behaviour.c:31-35, 216-222` | Information leakage / temporal decomposition | Behaviour has MAPPING / RESTARTING states only because building and printing the map blocks the loop longer than PicoB's DRIVE timeout: it knows the loop's timing and the link's timeout | Printing goes into the ring (A3); the batch build and both states go in S3 |
| `debug_console.c:184-186` `print_map` | Special-general mixture | Printing the map stops behaviour, tests and the motors | Ring (A3) |
| `debug_console.c:194, 236-238`, `behaviour.c`, `robot_test.c` | Information leakage / conjoined | Two modules command the wheels; the console keeps them apart by calling `behaviour_stop()` before a test and `robot_test_stop()` before a scan | `activity` (A5) |
| `motion_sense.c` | Special-general mixture | Gating (still / learning), routing to the tracker, turning tracker events into `leaving_t`, and the log's wording in one file | `movement` + console (A6) |
| `motion_sense.h` `leaving_t`, `behaviour.c:168` | Nonobvious code | `rate_radps == 0` means "seen only while learning" | `rate_known` and the event kind (A6) |
| `tracker.h` `tracker_target()` | Conjoined methods | After EXITED / STOPPED the pointer still gives the lost target "until the next call" | Events carry a copy (A6) |
| `motion_sense.h` `leaving_t`, `behaviour.c:170-182` | Information leakage | Bearings are sensor-relative; behaviour rebuilds the world direction with `pose_at(l->t_us)` | `movement` reports map-frame bearings (A6) |
| `behaviour.c:114, 182` | Information leakage | Headings come from `pose` (map frame) but turns are measured on `body_odom()->yaw_rad` (odometry frame): two yaw frames in one module | Behaviour uses `pose` only (A5) |
| `camera_motion.c:44, 145`, `drivers/camera.c:273` | Information leakage / split ownership | A consumer holds and frees the driver's exposure and restarts itself when the driver wants a change; a second camera consumer would fight over it | `vision`'s ZNCC is gain-invariant, so the exposure is never held; `camera_hold_exposure` goes with `camera_motion` (V4) |
| `rangefinder.c:88` | Nonobvious code | The frame's time is the read time minus half a period minus half the poll interval, though the INT pin (GP20) is wired | INT time − 33.3 ms − `TOF_PROC_US` (A2) |
| `brain.c:101` (PicoB) | Nonobvious code | ODOM's `t_us` is when it is sent, not when the IMU sample was taken | Sample time in `odom_t` (A2, protocol v5) |
| `world_map.c:39, 175` | Nonobvious code | Cell age uses the clock at insertion; the start-up scan's 300 kept frames all get the time of the map build | The scan's own time (A2) |
| `rangefinder.c` `closest_sure_mm` | Information hiding gone too far | The adapter throws away sigma, signal, ambient and the second target: the right decision for the map then, but now the ToF and surroundings plans need them | `tof_frame_t` keeps them (A4 + T1) |
| `camera_motion.c:165`, `change_grid.c:45`, `tof_motion.c:155` | Nonobvious code | `static` locals because of the 2 KB stack: hidden non-reentrancy, a trap once core 1 runs code | Explicit stack sizes; locals back on the stack (V3). A0 found the 2 KB was never enforced: `tof_motion.c`'s `observations()` alone uses 3.2 KB |
| `tracker.c:22`, `tof_motion.c:84, 166`, `camera_motion.c:118`, `motion_sense.c:51` | Repetition | `atan2f(where[1], where[0])` for a bearing, five times | `geom.h`: `bearing_rad(v)` (A1) |
| `pose.c`, `tracker.c:33, 65`, `behaviour.c:174`, `robot_test.c:54` | Repetition | `(float)(a - b) * 1e-6f` and `int32_t after()` for time differences | `stamp.h` (A1) |
| `tof_motion.c:69, 91`, `camera_motion.c:61`, `rangefinder.c:223` | Repetition | Three `compare_*` functions + `qsort` for a median or n-th value | `stats.h`: `nth_value()` (A1) |
| `tof_motion.c` `observations()`, `camera_motion.c` `observations()` | Repetition | Blob → observation (sum directions, left / right, normalise, strength, sort by cells) written twice | One helper next to `change_grid` (A1), gone with both detectors later |
| `motion.h` vs `motion_sense` / `tof_motion` | Vague name | "Motion" means the robot's and the scene's | `move_profile.h`; "movement" for the scene (A6) |
| `body.c:8, 123` | Nonobvious behaviour | DRIVE goes out every 50 ms whatever changed: a new command waits up to 50 ms | Send on change (at most every 10 ms) plus the keep-alive (A2) |
| test fakes `common/test/fakes/pico/stdlib.h` | (test gap) | The fake clock starts at 1: no test crosses the 32-bit wrap the robot reaches after 71.6 min | Start at 2³² − 30 s (A1) |
| `main.c` | Nonobvious code | The loop order matters (`pose_update` before `surroundings_update`, `camera_update` before `motion_sense_update`), and nothing says so | The pipeline written out with comments (A2) |

Considered and **not** flagged: `rangefinder`'s floor accessors for printing
(accepted in M2; they go in S3 anyway), `link_stats()` read by the console
(accepted in M1), `map_print` inside the map (accepted in M2: it stays, writing
into the ring).

---

## 7. Improvement ideas, each with an opinion

| # | Idea | Verdict | Why | Cost | Risk |
|---|---|---|---|---|---|
| I1 | Pipeline written out in `main.c` (sense, estimate, model, decide, act, report) | **Worth it** | Order already matters; makes the data flow reviewable | S | Low |
| I2 | Published frames with sequence numbers, readers keep a cursor | **Worth it** | Fixes the stolen camera frames and the ToF frames via the map; `vision` adds a second camera reader | S-M | Low |
| I3 | Pure modules (no `pico/` includes) vs platform edge, enforced by the host build | **Worth it** | Replay, simpler tests, and core 1 can run pure code safely | M (mostly moving clock reads into the data) | Low |
| I4 | Timestamps at the source: ToF INT pin, ODOM sample time, map uses scan time | **Worth it; a hard dependency of T-R** | The heading background and ego-motion place frames by time; a few ms is 0.3° at 1 rad/s | S; protocol v5 | Low |
| I5 | One confidence model for the ToF (`tof_quality`), sigma as its own field | **Worth it** | The ToF and surroundings plans need it; same behaviour at first (CREDIBLE ≈ today's "sure") | M | Low |
| I6 | `view`: all sensor geometry, camera intrinsics included | **Worth it** | `vision`, `camera_movers`, `tof_motion` and the map all need geometry; ROBOT_PLAN §10's "only `camera_motion` knows the camera geometry" stops working | M | Low |
| I7 | Ego-motion as an explicit step (fields in the vision frame, attitude history), not a module | **Worth it** (V2, T-R) | Turning while watching needs it; a module of its own would be a pass-through | S once `view` exists | Medium (row-time and sign errors: host tests with a synthetic turn) |
| I8 | `activity`: one owner of the wheels + safety pass | **Worth it, before M4** | Driving needs the Safety guard on every command; today's exclusivity is by convention | M | Medium (behaviour's state machine moves) |
| I9 | `movement`: one observation struct, events by value, map-frame bearings | **Worth it** | Fixes four flags in §6 and is where camera and ToF meet. After the open `find_passing()` fixes, so live M3b work isn't mixed with refactoring | M | Medium |
| I10 | Non-blocking telemetry ring for text | **Worth it** | Removes the 5 ms `printf` stalls and the map print's motor stop | S-M | Low |
| I11 | Binary records + replay (R0) | **Worth it, early (right after A3)** | Every companion plan's first step is a recording; tuning by replaying the same walk-by instead of walking again; golden tests from real misses | L (firmware, server, replay tool) | Medium (WiFi throughput, lwIP memory: measured first in R0) |
| I12 | One framing (COBS + CRC) for the UART link and the records | **Worth it, with I11** | Reuse of a tested deep module | S | Low |
| I13 | Loop, stage, stack and RAM statistics on `p` | **Worth it, first** | Every later check (RAM ≥ 40 KB, loop max) needs the numbers | S | None |
| I14 | Core 1 for `vision` | **Decided: yes, in V3**, after V1 measured it on core 0 | 17-23 ms per frame worst case can't share the core-0 loop (max < 20 ms). The camera plan's second reason (core 0 stalling while the map prints) disappears with A3 and S3, so it is not a reason | M | Medium-high (races; mitigated by mailboxes and no shared mutable state) |
| I15 | RTOS (FreeRTOS SMP) | **Not worth it** | Superloop + IRQs + one core-1 worker covers the needs; an RTOS adds a stack per task (RAM is the tight resource), priorities, lwIP reconfiguration, and harder host tests. Revisit if several blocking activities appear | L | High |
| I16 | ROS / micro-ROS | **Not worth it** | No Linux computer in the loop; XRCE-DDS costs RAM; ties messages to ROS. `replay --csv` can feed PlotJuggler if wanted | — | — |
| I17 | C++ | **Not worth it** | Deep modules work fine in C; mixing styles in a small codebase is worse than either; tests and fakes are C | — | — |
| I18 | Rust | **Not worth it** | Rewrite cost; the SDK, the ULD and cyw43 are C | — | — |
| I19 | A generic publish / subscribe bus | **Not worth it** | Speculative generality: a handful of producers, each with a `*_next(&value, &seq)` call, need no framework | — | — |
| I20 | Algorithm state as structs passed in (instances) | **For new and rewritten modules** (`vision`, ToF series, surroundings) | Replay A/B and two cores want it; rewriting stable modules only for it isn't worth it | S each | Low |
| I21 | EKF for the pose | **Later** (as ROBOT_PLAN §12) | No noise model yet; `pose_at` stays the interface | M | Medium |
| I22 | Runtime tunables over the console | **Later, maybe** | Replay with rebuilt constants covers most tuning, and the constants stay in the code | S-M | Low |
| I23 | CMSIS-DSP | **Not now** | V1 measures the ZNCC cost first; the fallback is fewer features or a smaller search window | S | Low |
| I24 | Hardware watchdog (PicoA 1 s, PicoB 0.5 s); after a watchdog reset PicoA starts as at power-up (scan without USB), IDLE after 3 resets in a minute | **Worth it** | A hung PicoA is already stopped by PicoB, but stays dead; Daniel prefers the scan after any start without USB, and the reset count stops a crash loop from driving | S | Low (policy question in §9) |
| I25 | DRIVE sent on change | **Worth it** | Up to 50 ms less latency on every turn decision | S | Low |
| I26 | Fake clock near the wrap; the robot's clock started 30 s before it (no soak) | **Done (A1)** | `time_us_32()` wraps after 71.6 min; no test covers it | S | None |
| I27 | Folders under `picoA/app/` per stage | **Later, optional** | Helps once the file count grows; churn in includes and tests now | S | Low |
| I28 | 64-bit timestamps everywhere | **Not worth it** | u32 with wrap-safe helpers is enough for differences under 35 min; the wrap test covers it | — | — |
| I29 | Move odometry to PicoA / one Pico for everything | **Not worth it** | PicoB as an independent safety layer is the best part of the design | — | — |
| I30 | Faster ODOM or raw IMU to PicoA | **Not worth it** | 50 Hz interpolated is ~0.01° off at the drive's 3 rad/s² | — | — |
| I31 | PC page draws the map, features and events from records | **Later, after R0** | Cheap once records exist; `replay --map` / `--pgm` cover it on the Mac first | M | Low |
| I32 | Fixed-point maths | **Not worth it** | The FPU does float32 in one cycle; `-Wdouble-promotion` already guards against double | — | — |

---

## 8. Milestones

### 8.1 The master order (all four plans)

| # | Milestone | From | Needs | Notes |
|---|---|---|---|---|
| 1 | Measure (by analysis, done) | A0 | | RAM and stack computed, no robot test; the loop timer moved to A1 |
| 2 | Helpers, clock wrap, loop timer (done) | A1 | | robot check with A2; no soak: the clock starts 30 s before the wrap |
| 3 | Time at the source, published frames, DRIVE on change, pose roll + `t_us` | A2 | | ToF INT time; `camera_next`; protocol v5 |
| 4 | Non-blocking telemetry, watchdog | A3 | | the old scan map build is not chunked (S3 deletes it); MAPPING stays until S3 |
| 5 | Recording + replay, WiFi throughput measured | **R0** (A7's core, V0, T0's record type, S0's dump) | A3 | |
| 6 | Measurement session | **M-S** (T0 + S0 + V0 step 3) | R0 | settles §2.3's status table, the echo guard, the drop veto, `TOF_PROC_US` |
| 7 | `tof_quality`, `tof_frame_t`, `view` | **A4 + T1** | M-S | no behaviour change |
| 8 | Classify readings | S1 | 7 | |
| 9 | z-test, weighted evidence | T2 | 7 | |
| 10 | Rotation-aware ToF background (per heading), detection while turning, no learning after a stop | **T-R** | 9, A2 | Daniel's directive |
| 11 | New map alongside the old one | S2 | 8 | |
| 12 | Switch the map; floor learning gone (−135 KB) | S3 | 11 | MAPPING / RESTARTING go here |
| 13 | `vision` on core 0, measured | V1 | R0, **S3 (RAM)** | algorithm work on recordings (host only) may start any time after M-S |
| 14 | Ego-motion, calibration | V2 | 13 | f calibrated independently of the gyro |
| 15 | Core 1 for `vision` | **V3 (= old A8)** | 12, 14 | RAM order §1.3 |
| 16 | One activity, safety pass | A5 | | before M4; may move earlier |
| 17 | Camera movers while still, final `motion_obs_t` | V4 | 15 | old `camera_motion` removed |
| 18 | ToF edge times and rates (still and turning) | T3 | 10 | |
| 19 | `movement` (fusion, events by value) + ToF events + camera into the tracker | **A6 + T4 + V6** | 17, 18, the `find_passing` fixes | one milestone, one robot test |
| 20 | Camera movers while turning | V5 | 19 | |
| 21 | Map queries for M4 | S4 | 12, 16 | |
| later | ToF fill fraction (T5), golden recordings (S5: continuous from R0 on), visual yaw, landmarks | | | |

**Why this order:**
- **Cheap foundations first.** A0-A3 change little and unblock everything after
  them.
- **Recording before tuning.** R0 and M-S come before any companion plan changes
  behaviour, so every threshold is set from recorded data.
- **A2 before T-R.** A2's ODOM sample time and INT-pin time are a **hard
  dependency** of T-R: the heading background needs `pose_at` at the start and the
  end of each frame's integration, accurate to a few ms.
- **S3 before V1.** S3 frees 135 KB, and the camera work doesn't fit in RAM
  without it (§1.3).
- **Everything in one place.** Milestones A4/T1, A6/T4/V6 and A8/V3 are merged, so
  each part has one robot test.

**Every milestone:**
- `./run_tests.sh` passes;
- the robot tests below are done;
- `p` shows **RAM free ≥ 40 KB**, and the loop max within its target (from A3 on);
- a red-flag review goes into REDFLAGS.md and an entry into the CHANGELOG.

Robot tests follow REWORK_PLAN's rule (only when a step changes what the robot does
or needs the robot's numbers; refactors ride along with the next one), and a
regression set only covers behaviour that no later step replaces. "Regression set" means: README **M0** step 4 (the square), **M2** steps 2-4 (scan
and map), **Movement, VL53** steps 1-2, and **Watching** steps 1-2. The companion
milestones (T*, S*, V*) have their test steps in their own plans; the A, R0 and
merged milestones are below.

### A0 — Measure

**Done, by analysis** (no robot session: RAM and stack follow from the build).
- RAM from the build (`arm-none-eabi-size`, `nm`): everything is static, no
  `malloc` linked (current numbers in §1.3).
- Stack from GCC's call graphs (`-fcallgraph-info=su`) with `tools/stack_depth.py`,
  which adds the calls through function pointers (printf's output, lwIP's
  callbacks): core 0 ~6 KB worst case in 8 KB of room. The results and what they
  mean for V3 are in §1.3.
- Loop timing can't be computed (I2C, USB and WiFi waits); it moved to A1 as a
  small timer printed by `p`, read during the robot tests that come anyway.

Pass / fail held: RAM free ≥ 40 KB; the stack needs no change before V3.

### A1 — Shared helpers, the clock wrap

- New helpers:
  - `common/stamp.h`: time differences;
  - `common/geom.h`: `bearing_rad`, `wrap_pi`;
  - `common/stats.h`: `nth_value`;
  - one helper that turns a blob into an observation.
- The fake clock starts at 2³² − 30 s.
- The loop timer (from A0): loop iterations per s, mean and max over the last 10 s,
  and which stage was slowest, printed by `p` and over WiFi. `p` also prints "RAM
  free N KB": the linker's free space (`__HeapLimit` − `end`), a constant, so every
  milestone's "RAM free ≥ 40 KB" check is one key.
- Otherwise a pure refactor (A0: the stacks wait for V3).

Host tests:
- `test_helpers`:
  - `stamp_us` across the wrap (2³² − 1 → 5 gives +6);
  - `wrap_pi` at ±π and ±3π;
  - `nth_value` against a sorted copy, on random arrays;
  - `bearing_rad` on the four axes.
- `test_loop_stats`: the mean and the max over a window; the window rolling over;
  the slowest stage named.
- Every existing test passes with the clock crossing the wrap during the test
  (check by printing the start time once). A test that fails here is a real wrap
  bug: fix it, then log it in REDFLAGS.

**Done.** No robot test of its own (the host tests' output was the same before and
after) and no soak: both Picos start their clock 30 s before `time_us_32()` wraps
(`common/clock_start.h`, the timer's TIMELW/TIMEHW written first in `main()`), so
every robot run crosses the wrap 30 s after power-up. `p` prints the loop time and
the free RAM.

### A2 — Time at the source, published frames, DRIVE on change

- **`tof_sensor`** (the poll half of `rangefinder`):
  - ToF frames are stamped from a GPIO interrupt on INT (GP20):
    `t_us` = INT − 33.3 ms − `TOF_PROC_US` (0 for now);
  - frames are published with a sequence number;
  - `surroundings`, `tof_motion` and the console all read them through
    `tof_sensor_next()`.
- **`camera_next(&image, &seq)`** for any number of readers, with `row0_us`. `c`
  prints the last image and never waits.
- **PicoB:** ODOM carries the IMU sample time; protocol v5.
- **`pose_t`** gets `t_us`, `roll_rad` and `w_radps`.
- **The map** stamps cells with the scan's time.
- **`body`** sends DRIVE when the command changes (at most every 10 ms), and still
  every 50 ms.
- **`main.c`** is written as the pipeline.
- **Behaviour** prints "turn started N ms after the decision": the time from its
  `body_drive()` call to the first ODOM with |ω| > 0.05 rad/s.

Host tests:
- `test_tof_sensor`:
  - two readers each get every frame exactly once;
  - a slow reader that misses a frame sees the gap in the sequence;
  - the time comes from the INT, not the read: a fake INT at t, read at t + 4 ms,
    gives a frame at t − 33.3 ms.
- `test_body`:
  - a new `body_drive()` goes out on the next update;
  - unchanged commands still go out every 50 ms;
  - commands changing at 1 kHz go out at most every 10 ms.
- `test_brain` / `test_odometry`: ODOM time = the sample's time (fake IMU sample at
  t, sent at t + 3 ms).
- `test_pose`: roll is interpolated like pitch; `pose_rotation` over a simulated
  90° turn.
- `test_world_map`: a scan measured 20 s ago and added now has age 20 s.
- `test_camera_motion` with a second reader taking frames: same results.
- `test_behaviour`: the "turn started" line is printed, with the fake robot's lag.

Robot:
1. Reflash **both** Picos (protocol v5).
   - `l`: bad 0, lost ~0.
   - Optional: reflash only PicoA. Expected: "PicoB protocol v4, mine v5: reflash
     both Picos", and the motors stay off.
2. `z` a few times: a new line "read N ms after INT". It should be steady, about
   4-5 ms (the SPI read).
3. `v` on, walk across, press `c` every second while walking. Expected:
   - "Movement (camera)" lines without gaps;
   - each `c` answers at once;
   - "N since the last c" ≈ 25 × seconds.
4. Watching step 2. Expected: "turn started N ms after the decision" with N < 40.
   **N > 60 fails** (DRIVE not sent on change, or ODOM late).
5. Regression set; `p`: RAM free ≥ 40 KB.
6. Bring-up: flash `picoA_bringup`, the viewer shows camera and ToF as before.

### A3 — Non-blocking telemetry, watchdog

- `telemetry`: `printf` goes into a ring. USB and the WiFi console drain it with a
  byte budget per loop. Drops are whole lines and counted (`p` shows them).
- **Printing the map** goes through the ring, and `m` no longer stops anything.
- **Building the scan's map is not split up:** S3 deletes it, and with it
  behaviour's MAPPING / RESTARTING states. Until then they stay.
- **Watchdog** on both Picos.
  - After a watchdog reset, PicoA prints "PicoA restarted by the watchdog" and
    starts as at power-up (Daniel, 8 Oct): without USB, WiFi then the scan. After
    3 watchdog resets within a minute (counted in the watchdog scratch registers,
    which survive the reset) it stays IDLE and says why: a crash loop must not
    keep driving.
  - A debug key `!` hangs the loop on purpose.

Host tests:
- `test_telemetry`:
  - lines come out in order;
  - when the ring is full, whole lines are dropped and counted, and the writer
    never waits;
  - two transports drain at different speeds.
- `test_behaviour`: `m` while watching leaves the watch running with the motors on.
- Start-up policy:
  - after a watchdog reset without USB: the scan, as at power-up;
  - the 3rd watchdog reset within a minute: IDLE, no scan; a 4th after more than
    a minute: the scan again;
  - after power-up without USB: WiFi, then the scan (as today).

Robot:
1. `a`, then `m` while watching. Expected:
   - the map prints;
   - the status still says watching;
   - walk past: it turns;
   - no `B: Motors off` line.
2. Serial monitor and the WiFi page both open: press `m` five times fast. Expected:
   - no "B: Motors off: drive commands stopped arriving";
   - `p` may show dropped lines (paste the count) and a loop max < 20 ms.
3. `!` while watching (motors on). Expected:
   - "B: Motors off: drive commands stopped arriving" within ~0.25 s;
   - PicoA restarts within ~1 s and prints the watchdog line; with USB it waits
     for keys as at power-up.

   Repeat without USB, on the WiFi page: the watchdog line, then the scan. `!`
   three times within a minute: after the third, "staying idle: 3 watchdog
   resets in a minute" and no scan.
4. `n`: the scan works as before (still with motors off while the map is built).
5. Regression set; `p`: RAM free ≥ 40 KB.

### R0 — Recording and replay (shared: replaces V0, T0's record type and `x`, S0's `Y`, A7)

- **Records:** typed records (§2.6) in COBS + CRC frames (`frame.c`, split out of
  `link.c`), on a second TCP connection (port 4212); `MEM_SIZE` raised for it.
  First types: `ODOM`, `TOF_RAW`, `IMAGE` (every Nth frame), `MARK`.
- **USB fallback:** the same stream on the CDC port in a recording mode.
- **Key `R`** switches recording on and off.
- **pc/robot** saves `.rec` files.
- **`build/replay`** with `--stats`, `--tof-stats`, `--pgm`, `--csv`.
- **Clips** for host tests go in `picoA/app/test/recordings/`, gzipped, ≤ 1 MB each
  (whether in git: §9).
- **A debug mode** sends records of dummy bytes, for the throughput test.

Host tests:
- The `frame` codec, shared with the link: round trips, and a damaged frame is
  dropped. The link's tests still pass.
- When the record ring is full, records are dropped and counted, and the loop
  never waits.
- `replay --stats` on a synthetic file gives the counts per type and the gaps in
  the sequence numbers.
- `npm test` in `pc/robot/`: the server writes records to a file and keeps the
  console text separate.

Robot:
1. **Throughput first.** `npm start` in `pc/robot/`, robot on WiFi (no USB).
   - Record dummy bytes at 50, 100, 200 and 400 KB/s, 10 s each.
   - For each rate, note what arrived, what was dropped, and `l`.
   - The highest rate with no drops decides the `IMAGE` rate (fill it into §1.3).
2. `R`, walk past three times (left to right, right to left, stop in the middle),
   press a key as a mark, `R`. Expected: "Recording off, N records, 0 dropped".
3. On the Mac: `build/replay <file>.rec --stats`. Expected:
   - ODOM at ~50/s, `TOF_RAW` at ~15/s, `IMAGE` at the chosen rate;
   - no sequence gaps;
   - the mark at the right time.
4. `--pgm out/`: the images open and show the walk.
5. While recording, the robot watches and turns as usual; the console text is not
   delayed (keys answer at once).
6. Regression set; `p`: RAM free ≥ 40 KB.

### M-S — One measurement session (shared: T0 + S0 + V0 step 3)

One session gives one set of recordings and one `--tof-stats` table. Record:
- a quiet room for 2 min;
- walks at 1 m and 2 m, both ways, one of them in dark clothes;
- a hand wave at 30 cm;
- a box at 60, 45 and 30 cm; a box put down and taken away;
- standing still for 10 s, then walking away;
- a sofa or door edge (status 12);
- a sunny window;
- open floor, the floor turned by hand, the carpet;
- a table edge at 15 cm;
- a glossy cupboard at 40 cm and a chair at 50 cm;
- `n` (scan) and `r` (turns);
- the lights off, then on;
- **slow turns and 1 rad/s turns past a door edge, left and right.** The edge's map
  bearing must agree in both directions; the disagreement / 2ω is `TOF_PROC_US`.

It settles:
- the status table and the echo guard (§2.3);
- the drop veto;
- sigma against the measured scatter;
- the leg widths of a walker and the jitter of an edge.

The results go into the CHANGELOG and into the plans' constants.

### A4 + T1 — `tof_quality`, `tof_frame_t`, `view` (no behaviour change)

- `tof_quality` (§2.3, the table from M-S) and `tof_frame_t`: the two nearest
  credible targets per zone, `weak_mm`, `ambient_kcps`.
- The geometry moves to `view` (`view_tof_dir`, `view_floor_patch`, `view_origin`,
  `view_place_tof`). `rangefinder` keeps only the floor and obstacle classification
  until S3.
- The map and `tof_motion` use CREDIBLE where they used "sure".

The detailed tests are in TOF_MOTION_PLAN, T1.

Host tests:
- `test_tof_quality`:
  - status 5 → 1.0; 6 / 9 / 10 → 0.5; 12 and others → 0;
  - sigma does **not** change the confidence;
  - `sigma_eff` is at least 3 mm + 0.5 % of the range, doubled for status 9.
- The echo case: a faint target at 29 cm and a credible one at 2 m give
  `target = [2000]`, `weak_mm = 290`.
- Equivalence: for the synthetic rooms of `test_rangefinder` and `test_behaviour`,
  the nearest CREDIBLE target equals today's `closest_sure_mm`, zone for zone, and
  the printed map is identical. Where M-S's table differs from today's statuses,
  the CHANGELOG says which zones change.
- `test_view`:
  - zone directions as before (the 90° rotation, the 3 cm offset);
  - `view_floor_patch` equals `rangefinder_floor_limit_m` for level rows;
  - a ray placed with the robot turned 90° rotates with it;
  - roll tilts it.

Robot:
1. `z`: a new block with each zone's credible targets (cm) and their confidence
   (0-9), and `weak_mm`. Robot facing a wall at ~1 m:
   - wall zones at 9 (status 5);
   - floor rows 5-7 credible;
   - in the upper zones that look into the open room, the floor echo at ~30 cm
     shows in `weak_mm`, not as a target.
2. A dark object and a white one at ~1 m in front of the left half: both credible;
   the dark one's sigma is higher. Paste both `z`.
3. Regression set; the map after `n` looks as before; `p`: RAM free ≥ 40 KB.

### A5 — One activity at a time, safety pass

- `activity` runs one activity at a time: the scan, the watch, the five tests, and
  later investigate.
- Only `activity` calls `body_drive` / `body_motors`, through `safety`:
  - now: PicoB not connected, or an IMU error → stop;
  - M4 adds the obstacle guard from S4's queries.
- The console asks for activities.
- Behaviour turns with `pose` only.

Host tests:
- `test_activity`:
  - starting a test while watching stops the watch first: it gets its stop, and
    its commands stop reaching the body;
  - the test's commands reach the body;
  - `s` stops any activity;
  - a finished activity leaves the robot IDLE;
  - PicoB lost → the command is zero and the activity ends with the reason.
- `test_behaviour` and `test_robot_test` pass unchanged through `activity`.
- `test_safety`: a command passes unchanged when all is well, and is zero when
  PicoB isn't connected.

Robot:
1. `a`, then `q`. Expected: "Watching stopped: test started", the square runs, IDLE
   at the end; `t` shows the result.
2. `q`, then `n` during the second leg. Expected: "Square test stopped: scan
   started", `t` keeps the partial result, the scan runs.
3. `s` during the scan, during watching and during a test: each stops, motors off.
4. While watching, switch PicoB off (battery and USB). Expected: "Watching stopped:
   PicoB not connected", and PicoA keeps answering keys.
5. Regression set (the M0 square through the new path); `p`: RAM free ≥ 40 KB.

### A6 + T4 + V6 — `movement`

`motion_sense` and `tracker` become `movement`:
- it takes `motion_obs_t` from both detectors and keeps one target;
- it emits `movement_event_t` by value, with map-frame bearings and
  `rate_known`, including the ToF plan's events (T4) and the camera in the
  tracker (V6);
- the console turns events into log lines;
- `motion.h` is renamed `move_profile.h`.

It comes after the `find_passing()` fixes and T-R, so `MOVE_PASSED_WHILE_LEARNING`
is gone by then.

Host tests:
- The scenarios of `test_tracker`, rewritten as events, give the same numbers.
- `test_behaviour` with events gives the same turns (−71°, −85°, +43°, at most
  90°).
- New events:
  - a lost target's event carries its last bearing, with no pointer to read
    afterwards;
  - with the robot turned 90° between the event and its use, the map bearing
    still points at the target;
  - a target seen by both sensors is one target, with `sources` showing both;
  - a camera-only observation (no range) joins the ToF target at the same bearing.
- The event queue full: the oldest event is dropped and counted.

Robot:
1. **Target, VL53** steps 1-4 and **Watching** steps 1-5. The log lines must keep
   the same meaning; the wording may change, as noted in the CHANGELOG.
2. **10 walk-pasts × 2 distances (1 m and 2 m) × 2 ways**, with recording on.
   - Count the misses: no turn, or a turn the wrong way.
   - Expected: fewer than today's (CHANGELOG, 7 Oct).
   - Paste the counts; the recordings of the misses become golden host tests.
3. `p`: RAM free ≥ 40 KB, loop max < 20 ms.

### V3 (= old A8) — Core 1 for `vision`

This is decided, not conditional (§0.9, I14). Its other steps are in
CAMERA_MOTION_PLAN, V3. The architecture's part:
- the mailboxes (images, attitude in; `vision_frame_t` out);
- explicit 8 KB stacks for both cores, with the `static` locals back on the stack;
- the camera driver initialised on core 1;
- no `printf`, lwIP or cyw43 on core 1.

Host tests:
- `test_mailbox`:
  - ordering, full and empty, wrap;
  - a threaded host test (pthreads) passing 10⁶ items between two threads with a
    checksum.

  This catches logic errors, not memory-ordering errors: the barriers need a review
  by hand.
- `vision` gives the same frames called directly as through the mailbox.
- `pose_interp.c` on core 1's attitude history gives the same answer as `pose_at`
  on core 0.

Robot:
1. `p`: core-0 loop max < 20 ms (typical < 2 ms); core 1's load in % at ~25
   frames/s; images dropped 0 per s while still.
2. The camera plan's V1-V2 robot tests: the same results as on core 0.
3. Soak for 30 min, watching on WiFi with recording on, someone walking past every
   few minutes. Expected: no reset, `l` clean, stack high-water on both cores
   < 75 % (and `tools/stack_depth.py` under the new sizes before flashing).
4. Regression set; `p`: RAM free ≥ 40 KB.

---

## 9. Open questions

**For Daniel:** **Answered (Daniel, 8 Oct): see [REWORK_PLAN.md](REWORK_PLAN.md) "Decisions".** Questions 1-5 below are settled there (no margin; 3 cm is fine; clips not in git; exposure cap 10 ms with gain ≤ 8, dark rather than noisy; scan after a watchdog reset, idle after 3 in a minute).
1. **Clearance margin:** the robot is 12 cm tall (confirmed). Is any margin wanted
   above it (L2 reaches 23 cm, so up to 11 cm is covered already)?
2. **The ground band:** is "under 3 cm is ground" (anything lower is driven over)
   all right, instead of "the ground cell's centre 5 cm below the floor"? The cone
   geometry can't support a top at 0.
3. **Recording clips in git** (≤ 1 MB gzipped each), or kept outside the repo?
4. **Exposure cap while watching:** 10 ms is sharp but needs gain ~24 in the
   evening, which is noisy; 20 ms is softer. Which one?
5. **After a watchdog reset without USB:** IDLE (proposed: an unexplained reset
   must not drive) or the scan, as at power-up?

**Answered by the companion plans (recorded here):**
- Floor learning goes completely in S3. The ground is an evidence layer, and with
  it go the 64 KB of start-up frames and the 33 KB of floor samples.
- The camera's block detector (`camera_motion`) retires in V4, replaced by
  `camera_movers` on `vision_frame_t`.
- Exposure: `vision` matches with ZNCC, which is gain-invariant, so the exposure is
  never held. What remains is the cap (question 4).
- Core 1: decided (§0.9).

**Still open in this plan:**
- Folder layout: keep `picoA/app/` flat or split it per stage once the companion
  modules land (I27)?
