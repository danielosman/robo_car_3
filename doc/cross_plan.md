# Cross-plan decisions (binding for all four plans)

Reviewer's decisions. Every plan adopts these names, numbers and the order below;
where a plan says otherwise, it changes. Items marked **(user)** need Daniel's word
and are repeated at the end.

All four plans were revised to these decisions; where a revised plan is more
detailed (e.g. T-R in TOF_MOTION_PLAN.md), the plan is the reference. The master
milestone order also lives in ARCHITECTURE_PLAN.md §8.1.

## 1. Numbers every plan uses

| What | Value | Source / note |
|---|---|---|
| Robot height | **12 cm** (confirmed by Daniel) | ROBOT_PLAN §3 says 10 cm (23.5 × 19 × 10); fix there in S3 |
| Footprint | 23.5 wide × 19 long; front edge 9.5 cm ahead of centre | ROBOT_PLAN §3 |
| VL53 mounting | 7 cm above the floor, 2.5 cm ahead of centre, 3 cm right, level; zones 5.625° | `rangefinder.c` 10-12 |
| VL53 rows | **1-8, row 1 at the top** (README; changed 10 Oct from 0-based; the code's indexes are one less). Row r's centre is (r − 4.5) × 5.625° below level: rows 1-4 look up, 5-8 down. Floor along the centre ray: row 5 ≈ 1.43 m, row 6 ≈ 48 cm, row 7 ≈ 29 cm (README measured 31), row 8 ≈ 21 cm | |
| Camera | 8.5 cm up, 2.5 cm ahead of centre, f ≈ 160 px, HFOV 53.1°, VFOV 41.1° | ROBOT_PLAN §3 |
| What the sensors see of a walker | both sensors see **only up to ~46-49 cm at 1 m** (top zone edge / top image row), ~85-90 cm at 2 m: **legs**, not a 45 cm torso. All sims (tof_sim, sim_room, camera sprites) model two legs with a gait (stance leg still ~60 % of the time, swing leg ~2× walking speed) | geometry: 7 cm + 1 m · tan 22.5°; 8.5 cm + 1 m · tan 20.5° |
| Ground band | G = −7…+3 cm, L1 = 3-13 cm, L2 = 13-23 cm (surroundings plan; deviation from Daniel's "ground centre 5 cm below the floor" explained there) **(user: OK with "under 3 cm is ground"?)** | |
| Drivable | G floor **and L1 free and L2 free** (Daniel's rule; the surroundings plan's "L2 not occupied" is dropped, see surroundings.md M1) | |
| ToF frame time | `t_us` = middle of the integration = INT-pin interrupt time − 33.3 ms − `TOF_PROC_US` (on-chip processing; start at 0, measured in T-R, see §4). Integration runs from `t_us − 33.3 ms` to `t_us + 33.3 ms` (continuous mode) | |
| Camera row time | image carries `row0_us` = middle of row 0's exposure, and `line_us`; row r at `row0_us + r · line_us` | replaces arch `bottom_row_us` and camera `top_row_us` / `t_us` ambiguity |

## 2. Shared types

**Time.** `stamp_t` (uint32 µs, `time_us_32()`, wrap-safe helpers in `common/stamp.h`)
for every time in every struct (`tof_event_t.t_us`, `vision_frame_t.*_us`,
`motion_obs_t.t_us` …). No raw `uint32_t t_us` in new code.

**Confidence scale.** Stored as `uint8_t` 0-255 only inside `tof_target_t`
(255 = 1.0, helper `tof_conf(const tof_target_t *)` → float). Everywhere else
(`ray_meas_t`, `motion_obs_t`, `movement_event_t`, `vision_*`) a float 0-1.

**One ToF confidence model** (`tof_quality.c`, pure, owned by TOF_MOTION_PLAN, used
by `tof_sensor` only; the map and movement read the numbers, never the status):

```
confidence = status_weight(status) × faint_factor(signal, range, row)   // 0-1
status_weight: 5 → 1.0; 6 → 0.5; 9 → 0.5; 10 → 0.5 (tentative, see tof.md S3); 12 → 0;
               everything else → 0. T0/S0 data changes the table once, recorded in CHANGELOG.
faint_factor:  1, except the echo guard (only if the measurement session shows the echo
               with a good status): 15-35 cm, signal < 15 kcps/SPAD, credible target behind → 0.1
sigma_eff_mm = max(sigma_mm, 3 + 0.005 × range_mm) × (status == 9 ? 2 : 1)   // separate field
CREDIBLE     = confidence ≥ 0.25        // same threshold in tof_motion, view_place_tof, the map
```

Sigma is **not** folded into the confidence (the surroundings plan's "sigma factor"
and "signal factor" go): sigma travels as its own number and each consumer uses it
its own way (ToF: z-test; map: free-space stop margin, and weight × 0.5 when
`sigma_eff` > 50 mm). Arch A4's host test ("large sigma or faint signal lower it")
is rewritten to this.

**`tof_frame_t`** (arch §2.3, extended):

```c
typedef struct { uint16_t range_mm, sigma_mm, signal_kcps; uint8_t status, confidence; } tof_target_t; // 8 B
typedef struct {
    uint32_t seq;
    stamp_t t_us;                  // middle of the integration (§1)
    uint16_t ambient_kcps[64];     // for tof_quality_clear_m (sunlight)
    uint8_t  n_targets[64];        // credible targets kept, 0-2
    tof_target_t target[64][2];    // the two nearest CREDIBLE targets, nearest first
    uint16_t weak_mm[64];          // nearest non-credible target (echo, faint floor), 0 = none
} tof_frame_t;                     // ~1.4 KB
```

`weak_mm` exists because the ToF plan drops confidence-0 targets while the map's
drop veto needs "a faint target inside the floor patch". Raw 4-target frames exist
only as the `TOF_RAW` record.

**`ray_meas_t`** (arch §2.3 + the surroundings plan's two fields, accepted):

```c
typedef struct {
    stamp_t t_us;
    float origin_map[3], dir_map[3]; // dir includes pitch AND roll (pose_t gets roll in A2)
    float half_rad;                  // cone half-angle (VL53 2.8125°); 0 = a true ray
    float range_m;                   // < 0: nothing up to max_m
    float max_m;                     // 1.0 m (tof_quality_clear_m: less in sunlight)
    float sigma_m;                   // sigma_eff
    float confidence;                // 0-1; "nothing" rays: 0.5
    float weak_range_m;              // from weak_mm, < 0 none
} ray_meas_t;
void surroundings_add(const ray_meas_t *rays, int n, const pose_t *robot);
```

**`pose_t`** gets `t_us`, `roll_rad`, `w_radps` (A2), as arch §2.3. Core 1 keeps its
own attitude history fed by a queue (camera §6.2) using the same interpolation code
(`pose_interp.c`, pure, shared).

**`motion_obs_t`** (one struct for both sensors; flags instead of sentinels and NAN):

```c
typedef struct {
    stamp_t t_us;
    uint8_t sensor;                  // OBS_TOF, OBS_CAMERA
    float where_robot[3];            // unit, robot frame AT t_us (movement converts with pose_at)
    float left_rad, right_rad;       // robot frame, + = left
    bool has_range;  float range_m, range_sigma_m; float point_robot[3];
    bool rate_known; float rate_radps, rate_sigma_radps; // WORLD frame (robot's own turn removed), + = left
    bool range_rate_known; float range_rate_mps;         // + = going away
    float strength; uint16_t cells;
    float confidence;                // 0-1
} motion_obs_t;
```

Both detectors produce observations **while the robot turns** (Daniel's directive
for the ToF; the camera already does). Only `movement` converts to the map frame.

**Events.** Detectors may emit their own internal events (`tof_event_t`, stamped
`stamp_t`); only `movement` emits `movement_event_t`. `movement_kind_t` gets
`MOVE_REMOVED` (ToF REMOVED); `MOVE_MAP_CHANGE` comes only from
`surroundings_next_change()`. `MOVE_PASSED_WHILE_LEARNING` is deleted once the
rotation-aware ToF background (T-R) removes "learning".

**Camera frame.** The camera plan's `vision_frame_t` and module name `vision`
(reader `camera_movers`) win; arch's `feature_t` / `feature_frame_t` sketch and
`features` module are deleted from §2.3/§5 and replaced by a reference.

**Geometry owner.** `view` owns all sensor geometry: VL53 zones, camera intrinsics
(f, cx, cy, k1, time offset, filled by `vision_calibrate`), mountings.
`vision_direction()` / `vision_pixel()` become `view_pixel_dir()` / `view_project()`.
`view` exports the floor patch of a zone (`view_floor_patch(zone, pitch, roll, &d1, &d2)`),
used by `tof_motion` (replaces `rangefinder_floor_limit_m`) and the map.

## 3. One recording path (replaces V0, T0's record type + `x` fallback, S0's `Y` dump, A7)

**Decided (Daniel, 8 Oct): no `IMAGE` records and no `--pgm` for now** (CAMERA_MOTION_PLAN §11); the throughput test still runs, for the ToF and odometry records.

- **R0 milestone, right after A3**: records `[type u8][len u16][payload]`, COBS + CRC
  (`frame.c` split from `link.c`), on a **second TCP connection** (port 4212) so a
  19 KB image never queues ahead of console text. Key **`R`** on/off. Types:
  `ODOM` (50 Hz), `TOF_RAW` (all 4 targets, the bring-up `TOF3` layout, ~3.1 KB),
  `TOF_FRAME` (later), `IMAGE` (every Nth, header + 19 200 px), `MARK` (keys and
  log lines), later `VISION`, `OBS`, `EVENT`.
- USB fallback: the same byte stream on the CDC port in a "recording" mode.
- **One host tool** `build/replay file.rec [--stats|--tof-stats|--vision|--map|--pgm dir|--csv]`.
  `vision_replay`, `tof_stats`, `tools/tof_dump_to_c.py` become modes of it.
- Clips for host tests in `picoA/app/test/recordings/` (gzipped, ≤ 1 MB each)
  **(user: in git or not)**.
- R0's first robot test measures WiFi throughput (records of dummy bytes at 50, 100,
  200, 400 KB/s for 10 s each, drops and `l` printed); it decides the image rate
  before anyone relies on 240 KB/s. lwIP today: `MEM_SIZE` 32 000, `TCP_SND_BUF`
  8 × MSS (~11.7 KB) per connection; a second connection likely needs `MEM_SIZE`
  raised.

## 4. One measurement session (M-S, right after R0: T0 + S0 + V0 step 3 together)

One session, one set of recordings, one `--tof-stats` table: quiet room 2 min; walks
at 1 / 2 m both ways (one in dark clothes); hand wave at 30 cm; box at 60 / 45 / 30
cm; box put down and taken away; stand 10 s then walk away; sofa/door edge (status
12); sunny window; open floor; floor turned by hand; table edge 15 cm; glossy
cupboard at 40 cm; chair at 50 cm; carpet; `n` scan; `r` turns; lights off/on;
**and slow + 1 rad/s turns past a door edge, left and right** (for T-R and the ToF
time offset: the edge's map bearing must agree for both directions; the
disagreement / 2ω is `TOF_PROC_US`). Decides: the status table, the echo guard,
the drop veto, sigma vs scatter, walker leg widths and edge jitter.

## 5. RAM budget (static, KB; today .bss 377 + .data 8 = 385 of 512 main SRAM)

| Change | KB | When |
|---|---|---|
| Today | 385 | |
| Telemetry ring (text 16 replaces WiFi `out` 16; records 16-32) | +16…32 | A3 |
| Explicit 8 KB stacks for both cores move out of the 4 KB scratch banks into main SRAM | +16 | A8/V3 (only when core 1 runs) |
| R0: one image staging copy (19.2) + record ring + second TCP connection's lwIP memory | +30…40 | R0 |
| ToF: `tof_frame_t` producer + 2 reader copies, per-zone state, chains, heading background (T-R) | +20 | T1-T-R |
| Surroundings: −kept 63.6, −cells 51.2, −samples 32.8, +13 | −135 | S3 |
| Camera: half images 9.6, features 19, **ring of 4** × 6.6, reader copy 6.6, attitude 1.5; −`camera_motion` (learning 24, grid ≈ 6) | +63 (net +33 once V4 removes `camera_motion`) | V1-V4 |
| **Total after everything** | **~380** | ~130 KB left for heap |

Arch's "stays under ~330 KB" is wrong (camera counted at 30 KB instead of ~63,
stacks, recorder and reader copies missing). **Ordering constraint:** without S3's
−135 KB, A3 + R0 + the ToF work + V1 come to ~500 KB (12 KB left for heap), and V3's
ring and 8 KB stacks push it to ~545 KB: **S3 must land before V1** (V1 and later
camera milestones start once the old map and floor learning are gone). Every milestone's robot test prints `p`'s RAM
free; a milestone that leaves < 40 KB free fails.

## 6. Cores

**Decided: `vision` runs on core 1 (camera plan V3 = arch A8), unconditionally**, after
V1 has measured it on core 0. Reason: CPU, not stalls. One vision step is ~8-25 ms
(camera estimate, likely 2× optimistic), which alone breaks the core-0 loop budget
(typical < 2 ms, max < 20 ms). The camera plan's other reason ("core 0 stalls for
seconds while the map prints") disappears with A3 and S3 and must not be used.
The camera driver is initialised on core 1 (its DMA_IRQ_1 handler then runs there;
check that cyw43 on core 0 never enables DMA_IRQ_1 on core 0). Core 1: no printf,
records only. Everything else stays on core 0.

## 7. Merged milestone order

| # | Milestone | From | Needs | Notes |
|---|---|---|---|---|
| 1 | Measure (done by analysis: ARCHITECTURE §1.3) | A0 | | |
| 2 | Helpers, clock wrap (done) | A1 | | no soak: both Picos start their clock 30 s before the wrap |
| 3 | Time at source, published frames, DRIVE on change, pose roll + `t_us` | A2 | | ToF INT time; `camera_next`; protocol v5 |
| 4 | Non-blocking telemetry, watchdog | A3 | | **skip chunking the old scan map build** (S3 deletes it); keep MAPPING until S3 |
| 5 | Recording + replay, throughput measured | **R0** = A7 core + V0 + T0 record + S0 dump | A3 | |
| 6 | Measurement session | **M-S** = T0 + S0 + V0 step 3 | R0 | decides §2's table |
| 7 | `tof_quality`, `tof_frame_t`, `view` | A4 + T1 | M-S | no behaviour change |
| 8 | Classify readings | S1 | 7 | |
| 9 | z-test, weighted evidence | T2 | 7 | |
| 10 | **Rotation-aware ToF background** (heading-indexed), detection while turning, no learning after a stop | **T-R** (old T6, moved up and widened) | 9, A2 | Daniel's directive |
| 11 | New map alongside | S2 | 8 | |
| 12 | Switch the map, floor learning gone (−135 KB) | S3 | 11 | |
| 13 | `vision` on core 0, measured | V1 | R0, **S3 (RAM)** | algorithm work on recordings (host only) may start any time after M-S |
| 14 | Ego-motion, calibration | V2 | 13 | f calibrated independently of the gyro (camera.md M5) |
| 15 | Core 1 | V3 = A8 | 12, 14 | RAM ordering §5 |
| 16 | One activity, safety pass | A5 | | before M4; may move earlier |
| 17 | Camera movers while still, final `motion_obs_t` | V4 | 15 | old `camera_motion` removed |
| 18 | ToF edge times and rates (still and turning) | T3 | 10 | |
| 19 | `movement` (fusion, events by value) + ToF events + camera into tracker | A6 + T4 + V6 | 17, 18, `find_passing` fixes | one milestone, one robot test (10 walk-pasts × 2 distances × 2 ways, misses counted) |
| 20 | Camera movers while turning | V5 | 19 | |
| 21 | Map queries for M4 | S4 | 12, 16 | |
| later | ToF fill fraction (T5), golden recordings (S5: continuous from R0 on), visual yaw, landmarks | | | |

## 8. Questions only Daniel can answer

**Answered (Daniel, 8 Oct): see [REWORK_PLAN.md](REWORK_PLAN.md) "Decisions".** All five are settled there; 5: the scan, idle after 3 resets in a minute.

1. Clearance margin above the 12 cm robot (confirmed height)?
2. Ground band top at 3 cm (anything lower is driven at) instead of your
   "ground centre 5 cm below the floor" (top at 0, which the cone geometry can't
   support)? (L2 must be *free*: your rule, adopted; the still-unknown L2 cells
   within 25 cm are set free once after the scan, SURROUNDINGS_PLAN.md.)
3. Recording clips in git (≤ 1 MB gzipped each)?
4. Exposure cap while watching (10 ms: sharp, gain ~24 in the evening, noisy; or 20 ms)?
5. After a watchdog reset without USB: IDLE (proposed) or scan?
