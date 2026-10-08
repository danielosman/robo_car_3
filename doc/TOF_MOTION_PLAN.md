# RoboCar — ToF movement plan (VL53L8CX)

Planning only, no code. How the VL53L8CX finds movement, standing still **and
while the robot turns**, from first principles: each zone's distance as a signal
over time, neighbours compared in time and space, the robot's own rotation
predicted from the gyro, and the sensor's quality numbers turned into a
confidence. Companions: [ARCHITECTURE_PLAN.md](ARCHITECTURE_PLAN.md) (stages,
`tof_frame_t`, the R0 recording path, the merged milestone order),
[CAMERA_MOTION_PLAN.md](CAMERA_MOTION_PLAN.md) (`motion_obs_t`),
SURROUNDINGS_PLAN.md (the map; points at §3 for the confidence model). Current
design: [ROBOT_PLAN.md](../ROBOT_PLAN.md) §6.2, §6.6.

Rows and columns are **0-based, as in the code**: row 0 at the top, column 0 at
the left (as the robot sees it). README's "rows 6 / 7 / 8" are rows 5 / 6 / 7 here.

## 0. The decisions in short

1. **One confidence model, shared** (`tof_quality`, §3; this plan owns it, the map
   reads its numbers): confidence from the status (5 → 1.0, 6 / 9 / 10 → 0.5,
   others → 0), `CREDIBLE` = confidence ≥ 0.25; the effective sigma travels as its
   own number. `tof_frame_t` keeps the two nearest credible targets per zone and
   the nearest weak one (`weak_mm`, for the map).
2. **A zone's signal is "nearer than its background or not"**, judged by a z-score
   (≥ 4 combined sigmas and ≥ 30 mm), not by 8 cm / 8 %. Each frame adds the
   reading's confidence to `change_grid` under today's rule (2 of 4 frames with a
   neighbour, 3 alone), so one sure reading counts as two status-9 readings.
3. **Detection never switches off while turning.** The background is stored by
   **world heading** (per row, 256 bins of 1.4°), not per zone. For each frame the
   gyro yaw at the start and the end of the integration gives the cone each zone
   swept; its prediction is the nearest background over that cone, widened by the
   heading error. Only readings clearly nearer than that count. Chosen over
   ray-casting into the map (§7.4).
4. **No learning after a stop.** Bins are refreshed whenever a zone's reading is
   consistent with them, still or turning; the 390° start-up scan fills every
   heading. After the robot drives > 10 cm the bins are invalid and refill in ~2
   frames.
5. **Direction and rate from edge times, in world bearings.** Each zone timestamps
   when it becomes nearer (onset) and free again (offset); onsets in neighbouring
   columns, at similar range, in order, are one edge moving. Fitting world bearing
   against time removes the robot's own turn by construction. This is the
   cross-correlation / Reichardt idea in its cheap form for step signals.
6. **A walker is two legs.** The VL53 sits 7 cm up and sees at most ~48 cm high at
   1 m, ~90 cm at 2 m. Legs stand still ~60 % of the time and swing at ~2.5× the
   walking speed, so the edges move in jumps: the sign comes in 2-3 frames, but a
   walking rate needs about one step (~0.55 s) of edges. The simulator has a gait.
7. **Events from the same patterns:** appeared (inside, or entered from a side),
   gone (inside, or left the view on a side), stopped (now background), removed
   (the old background back). Kept inside `tof_motion` as `tof_event_t`; only
   `movement` emits `movement_event_t`.
8. **No spatial filtering** (no blur, no median): a zone is already a 5.6° average,
   and a leg at 2 m is narrower than a zone.
9. **Measured before built:** the shared recording path (R0) and one measurement
   session (M-S) come first; every milestone is checked on synthetic frames and on
   recordings, then on the robot.

## 1. Where we are and what goes wrong

`tof_motion.c` today: the nearest **sure** target per zone (5, 6, 9) or nothing; a
background per zone learned over 1 s after each stop (its nearest regular reading);
a zone moved when **8 cm or 8 %** nearer in 2 of 4 frames next to another, or 3
alone; brief nearer surfaces, 10 s farther and 1 s steady become the background;
touching moved zones form blobs; the tracker fits a line through blob bearings
(~0.8 s) for the angular speed. It works (CHANGELOG 6-7 Oct), with these limits:

- **Blind while turning and ~1.5 s after every stop** (0.5 s settle + 1 s
  learning): the main cause of the M3b misses (CHANGELOG 7 Oct), and against
  Daniel's directive that detection must not switch off while turning.
- **Rate is late and noisy.** A blob's centre jumps as zones join and leave;
  "left the view" comes 0.5 s after the last blob (`LOST_US`).
- **Sensor quality is thrown away** in `closest_sure_mm`: sigma, signal, the
  second target. A fixed 8 cm is coarse for a strong return at 1 m (σ ~1 cm) and
  near the noise of a weak one at 3 m.

A lesson to keep (CHANGELOG 6 Oct): the first detector scored "bits of surprise"
from per-zone histograms and was rewritten as too complicated. This plan keeps
today's structure: one number per reading (its confidence, from the sensor, not
learned), one test per reading (z-score), the same grid rule; the background moves
from "per zone" to "per heading", with today's rules for what becomes background.

## 2. What the sensor says, and what it sees of a person

Per frame (15 Hz at 8 × 8, continuous mode: the whole 66.7 ms integrates), per zone:
`nb_target_detected`, `ambient_per_spad` (kcps/SPAD), `nb_spads_enabled`; per target
(up to 4, nearest first): `distance_mm`, `range_sigma_mm` (the ULD divides by 128:
whole mm), `signal_per_spad` (kcps/SPAD, divided by 2048), `reflectance`,
`target_status`.

Status (UM3109 results table, as UM2884 for the VL53L5CX; ST forum answers in
§10): **5** range valid ("100 %"); **6** wrap-around check not done (usually a
first frame); **9** valid with a large pulse, probably two surfaces merged;
**10** valid, but no target at the previous range: exactly what a zone reports in
the frame something appears; **12** target blurred by another one (sharpener);
1-4, 7, 8, 11, 13 failed checks; 255 no target. ST: 5 = 100 %, 6 and 9 = 50 %, the
rest less. **Not verified here:** ST's forum says status 9 appears when two
surfaces are within ~60 cm, and that two surfaces farther apart are reported as
two targets; I could not extract UM3109's text to cite the line. M-S measures it
(a person 30 / 60 / 90 cm in front of a wall) before `tof_sim` relies on it.

Measured on this robot (README): most zones above the floor rows report a faint
**unsure** target at ~30 cm (signal 3-10 kcps/SPAD against the floor's 150-730),
the floor's echo inside the sensor; the real target is the next one. Status 12
zones come and go. Row 5 sometimes reads the wall behind the waxed floor; rows 6-7
can read "beyond the floor" in front of glossy furniture. Turning in place the
robot rolls −3…−5°.

Geometry (cross-plan numbers): the VL53 is 7 cm above the floor (robot 12 cm
tall), 2.5 cm ahead of centre, 3 cm right; zones 5.625°. Column centres ±2.8°,
±8.4°, ±14.1°, ±19.7° (+ = left); row r's centre is (r − 3.5) × 5.625° below level:
rows 0-3 look up, 4-7 down. Floor along the centre ray: row 4 ≈ 1.43 m, row 5 ≈
48 cm, row 6 ≈ 29 cm (README measured 31), row 7 ≈ 21 cm. The top zone edge is
22.5° up.

**What it sees of a walker** (top of view = 7 cm + d · tan 22.5°; leg ~12 cm wide;
1 m/s walking, cadence ~1.8 steps/s, step ~0.55 m, a gait cycle ~1.1 s, each foot
in stance ~60 % = ~0.67 s, then swinging ~1.1 m in ~0.44 s, ~2.5 m/s):

| Distance | Sees up to | Zone width | One leg spans | One step spans | 1 m/s is | Rows on the legs |
|---|---|---|---|---|---|---|
| 0.3 m | 19 cm | 2.9 cm | (a hand, 8 cm: 2.7 columns) | — | 191°/s | — |
| 1 m | 48 cm (knees) | 9.8 cm | 6.9° = 1.2 columns | 31° = 5.5 columns | 57°/s | 0-4 (5 rows) |
| 2 m | 90 cm (hips, hands) | 19.7 cm | 3.4° = 0.6 column | 16° = 2.8 columns | 29°/s | 0-3 (row 4 sees the floor at 1.43 m first) |
| 3 m | 1.31 m | 29.5 cm | 2.3° = 0.4 column | 10.5° = 1.9 columns | 19°/s | 0-3 |

Consequences: at 1 m a walker is two blobs of ~6-8 zones (a gap between the legs
half the time), not one 22-zone body. The leading edge jumps a step (5.5 columns at
1 m) in ~0.3 s and then waits; the stance foot stays put for ~0.67 s. At 2 m and
beyond a leg covers only part of a zone: the zone reports it if its share of the
signal is enough (a second target if the wall is far enough behind). At 1 m the
whole view is crossed in ~1.4 steps.

## 3. The confidence model (one, shared)

`tof_quality.c`: pure functions, owned by this plan, called only by `tof_sensor`
when it fills `tof_frame_t`. The map and movement read the numbers, never the
status. SURROUNDINGS_PLAN points here.

```c
// tof_sensor.h (ARCHITECTURE_PLAN §2.3, as extended by the cross-plan review)
typedef struct { uint16_t range_mm, sigma_mm, signal_kcps; uint8_t status, confidence; } tof_target_t; // 8 B
typedef struct {
    uint32_t seq;
    stamp_t t_us;                  // middle of the integration (below)
    uint16_t ambient_kcps[64];
    uint8_t  n_targets[64];        // credible targets kept, 0-2
    tof_target_t target[64][2];    // the two nearest CREDIBLE targets, nearest first
    uint16_t weak_mm[64];          // nearest non-credible target (echo, faint floor), 0 = none
} tof_frame_t;                     // ~1.4 KB
static inline float tof_conf(const tof_target_t *t) { return t->confidence / 255.0f; }

// tof_quality.h
float tof_quality_confidence(uint8_t status, uint16_t range_mm, uint16_t signal_kcps,
                             bool credible_behind);          // 0-1
float tof_quality_sigma_mm(uint8_t status, uint16_t range_mm, uint16_t sigma_mm); // sigma_eff
float tof_quality_clear_m(uint16_t ambient_kcps);            // how far "nothing" is trusted
#define TOF_CREDIBLE 0.25f
```

In simple sentences:

1. **Confidence = status weight × faint factor.** Status weight: 5 → 1.0; 6 → 0.5;
   9 → 0.5; 10 → 0.5 (tentative: it marks onsets, and 0 would delay every onset by
   a frame); 12 → 0; everything else → 0. M-S changes the table once (counting
   statuses in onset frames separately), recorded in the CHANGELOG.
2. **Faint factor** = 1, except the **echo guard**, only if M-S shows the echo with
   a good status: a target at 15-35 cm with signal < 15 kcps/SPAD and a credible
   target behind it → × 0.1. A hand at 30 cm returns hundreds of kcps/SPAD.
3. **Sigma is not folded into the confidence.** `sigma_eff = max(sigma_mm, 3 mm +
   0.5 % of range) × (status 9 ? 2 : 1)`: the sensor's sigma is shot noise only
   (offset errors are a floor), and a merged pulse lies somewhere between two
   surfaces. Each consumer uses it its own way (here: the z-test; the map: its
   stop margin and weight).
4. **Credible = confidence ≥ 0.25**, the same threshold in `tof_motion`,
   `view_place_tof` and the map.
5. **Kept per zone:** the two nearest credible targets, and in `weak_mm` the
   nearest non-credible one (the echo, a faint floor): the map's drop veto needs
   it; movement ignores it.
6. **"Nothing" is trusted up to `clear_m`:** 1.0 m indoors (today's
   `NO_TARGET_CLEAR_M`), less in strong ambient light (M-S measures by a window).

Floor geometry is not in `tof_quality`: whether a reading lies beyond a zone's
floor patch (a reflection) comes from `view_floor_patch(zone, pitch, roll, &d1,
&d2)` (owned by `view`; replaces `rangefinder_floor_limit_m`).

**Frame time:** `t_us` = INT-pin interrupt time − 33.3 ms − `TOF_PROC_US` (on-chip
processing; 0 until T-R measures it). The integration runs from `t_us − 33.3 ms`
to `t_us + 33.3 ms`; the swept cone (§7) uses both ends.

## 4. First principles: a zone over time

Let zone *i* read `d_i(t)` with `sigma_eff` `s_i(t)` and confidence `p_i(t)`, and
let `P_i(t)` be its predicted background with spread `σb_i(t)` (§7: from the
heading bins, so it moves when the robot turns). Things only appear *in front of*
what was there, so the useful signal is

    o_i(t) = 1 if p_i ≥ 0.25 and (P_i − d_i) / sqrt(s_i² + σb_i²) ≥ 4 and P_i − d_i ≥ 30 mm
             0 otherwise (not nearer, or tells nothing)

`o_i` is a step function: 0 → 1 when something covers the zone (onset), 1 → 0
when it leaves (offset). The raw derivative `d'_i` is useless by comparison: a
single spike at the step (2.5 m → 1.0 m in one frame), and noise and status
flicker make spikes of their own. The steps of `o_i` are the difference signal.

Something moving sideways at angular speed ω gives the same step in the next
column shifted by τ = 5.625° / ω: `o_{i+1}(t) ≈ o_i(t − τ)`. Cross-correlating the
two series peaks at τ (Basu & Rowe use a similar lag analysis on an 8 × 8 thermal
array). For step signals the peak is the difference of the step times, so it is
enough to timestamp the steps: the answer is there the frame the step happens, for
O(1) work per zone. A Reichardt detector multiplies one signal by the other delayed
by a fixed τ and subtracts the mirror; it answers "which way" for one tuned speed
and cancels flicker (both change at once). Edge timing gives the speed directly;
the flicker case is lag 0, read as "appeared", not movement.

**World bearings.** An onset's bearing is the column's bearing plus the yaw at the
frame (`pose_at(t_us)`). Fitting world bearing against time gives the world rate:
a stance foot (still in the world) sweeping across the columns while the robot
turns has a constant world bearing and fits 0°/s; static background never makes
onsets at all, because it is the prediction.

| Pattern of steps | Meaning |
|---|---|
| Onset in one zone, nothing near it | something small appeared, or noise (3 frames alone) |
| Onsets in several zones in the same frame (lag 0) | appeared there (from behind, put down, stood up, came straight at the robot) |
| Onsets in neighbouring columns, similar range, lag > 0, in order | an edge moving: direction from the order, speed from the slope |
| The chain started in an outer column (or the leading column of a turn, §7) | it entered the view from that side |
| Offsets running out to an outer column, then nothing there | it left the view on that side |
| Offsets in place, no chain | gone inside the view (behind something, beyond range) |
| Its whole blob steady for 1 s | stopped: becomes background |
| Range falling or rising steadily | coming toward / going away |
| Chains flipping direction | waving, oscillating |

Rows give the same for up and down (a box lowered).

**How good is the rate?** Two parts.

- *Timing noise of a smooth edge* (a box carried, a hand, a slide): an onset's
  time is quantised to the frame (66.7 ms / √12 = 19 ms) plus trigger jitter
  (~20 ms): σ_t ≈ 28 ms. A least-squares line through n columns 5.625° apart has a
  slope sigma of 7.0 / 3.5 / 2.2 / 1.6 ms per degree for n = 2 / 3 / 4 / 4 with
  both edges.
- *The gait* (a walker): within a step the leading edge moves at 0 or ~2.5× the
  walking speed, so a fit shorter than a step measures the swing, not the walk;
  over a whole step it measures the walk to ~step-to-step variation (~10 %,
  assumed).

| | Smooth edge, n = 3 | Smooth edge, n = 4 | Legs, < 1 step of edges | Legs, ≥ 1 step (~0.55 s) |
|---|---|---|---|---|
| at 1 m (57°/s) | ±20 % (±11°/s) | ±13 % (±7°/s) | sign only (0-140°/s) | ±10-15 % (±6-9°/s) |
| at 2 m (29°/s) | ±10 % (±3°/s) | ±6 % (±2°/s) | sign only | ±10 % (±3°/s) |

So: the **sign** after 2-3 columns (2-3 frames at 1 m), a coarse rate with σ ≥ 40 %
until the edges span one step, the walking rate after ~0.55 s, the same as
today's 0.8 s bearing fit, but without centre jitter and with the robot's turn
removed. The leg numbers are assumptions until M-S's walks give leg widths, stance
times and edge jitter; T3's tolerances come from those walks.

## 5. The detector in simple sentences (`tof_motion`, rewritten)

1. **A zone's reading** is its nearest credible target in `tof_frame_t` (within the
   floor patch's far end in floor rows), with `sigma_eff` and confidence; or
   "nothing" (no target, trusted to `clear_m`); or "tells nothing".
2. **Its prediction** `P_i, σb_i` comes from the heading bins over the cone it
   swept during the frame (§7). No known bin under the cone: tells nothing.
3. **Nearer** (`o_i = 1`): z ≥ 4 and ≥ 30 mm nearer than `P_i`; any credible target
   where the bins say "nothing seen there".
4. **Score:** each frame a nearer zone adds its confidence to `change_grid`
   (status 5: 1.0; 6 / 9 / 10: 0.5); moved = 2.0 over the last 4 frames next to
   another such zone, or 3.0 alone, as today. A second target agreeing with `P_i`
   adds 0.25 (capped at 1 per frame): the old view is still there behind it.
   `change_grid` stays per zone in the robot frame: during a 4-frame window at 1
   rad/s the view turns 15°, so a mover stays in the same or a neighbouring zone.
5. **Steps:** a zone's onset is the first frame of a run of nearer frames that
   ends up moved; it is known in hindsight, when the zone is moved (1-2 frames
   later). Its offset is the first of 2 frames not nearer. Each step keeps its time,
   world bearing, range and confidence.
6. **Chains:** an onset joins the chain of an onset in a neighbouring column (same
   row or ±1), 1 frame to 1 s earlier, within max(25 cm, 3 combined sigma) in
   range. Lag 0 joins as "simultaneous". Offsets chain the same way. A chain keeps
   its (world bearing, time) points of the last 1.2 s (two steps).
7. **Rate:** world bearing against time, least squares, over a chain's points;
   with ≥ 3 columns a coarse rate (σ ≥ 40 %) and its sign; with points spanning ≥
   0.55 s the walking rate and its sigma (§4). Leading and trailing chains of one
   observation are averaged by their sigmas. A mover that keeps still in the robot
   frame (the robot turning after it) makes no new onsets: its rate comes from its
   blob's world bearing over the last 0.8 s, as the tracker does today.
8. **Radial speed:** the slope of the observation's range over the last 0.5 s.
9. **Observations:** touching moved zones form blobs; blobs within 2 columns and
   25 cm in range are one observation (two legs). Its range is the median of its
   zones' nearest ranges; its confidence as §6.1.
10. **Events** from its chains when it starts and ends (§6.2).
11. **What becomes background** (today's rules, now written into the bins, §7.2):
    a nearer reading that stays steady (within max(3 combined sigma, 3 cm)) for 1 s
    **while no zone of its observation changed in that second** → stopped (a
    stance foot is steady for ~0.67 s, and the other leg keeps changing); a brief
    nearer surface seen now and then (a door frame's edge) → background; farther
    is never movement and becomes background after 10 s. The background a stopped
    thing covered is remembered for 60 s, so reading it again for 1 s is
    **removed**, and someone who stood and then walks away is still nearer than it.

## 6. Output

### 6.1 Observations

`motion_obs_t` as the cross-plan review fixes it (one struct for both sensors,
flags instead of sentinels or NAN):

```c
typedef struct {
    stamp_t t_us;                    // middle of the integration
    uint8_t sensor;                  // OBS_TOF
    float where_robot[3];            // unit, robot frame at t_us; bearing = atan2(y, x), + = left;
                                     // elevation = asin(z)
    float left_rad, right_rad;       // robot frame, + = left
    bool has_range;  float range_m, range_sigma_m; float point_robot[3];
    bool rate_known; float rate_radps, rate_sigma_radps; // WORLD frame (own turn removed), + = left
    bool range_rate_known; float range_rate_mps;         // + = going away
    float strength; uint16_t cells;
    float confidence;                // 0-1
} motion_obs_t;
```

Observations are produced **while turning** as well; only `movement` converts to
the map frame. `rate_sigma_radps` includes the fit and the gyro's share (scale
error 0.42 % of ω: 0.24°/s at 1 rad/s, small against the fit).

**Confidence** = min(1, E / 6), where E is the sum over the observation's zones of
their confidence-weighted scores over the last 4 frames: two sure zones for 3
frames give 1.0, a lone sure zone for 3 frames 0.5. A score, not yet a probability:
T2 calibrates it on the quiet clips (share of false observations per bin).

### 6.2 Events

```c
typedef enum { TOF_APPEARED, TOF_GONE, TOF_STOPPED, TOF_REMOVED } tof_event_kind_t;
typedef struct {
    tof_event_kind_t kind;
    int8_t side;          // APPEARED: +1 entered from the left, -1 from the right, 0 inside;
                          // GONE: +1 left the view on the left, -1 on the right, 0 inside
    float bearing_rad, up_rad;   // robot frame at t_us
    bool has_range; float range_m;
    bool rate_known; float rate_radps;
    float confidence;
    stamp_t t_us;
} tof_event_t;
int tof_motion_events(tof_event_t *e, int max); // since the last call; internal to movement
```

`movement` (ARCHITECTURE_PLAN A6) maps them: APPEARED → `MOVE_NEW` (with `side`);
GONE with a side → `MOVE_EXITED`; GONE inside and STOPPED → `MOVE_STOPPED`;
REMOVED → `MOVE_REMOVED`. `MOVE_MAP_CHANGE` is the map's alone, and
`MOVE_PASSED_WHILE_LEARNING` goes away with learning (T-R). `MOVE_LEAVING` stays
the tracker's, now decided on the first outer-column observation with a known
outward sign and coarse rate ≥ 10°/s; GONE at the edge comes 2 frames (0.13 s)
after the last offset instead of 0.5 s.

Log lines (with `v`), today's plus:

```
   12.3 Movement (ToF): 14 zones, +14 deg (+ = left), +5 deg up, 1.02 m, going right at 55 deg/s (±8), conf 1.0
   12.1 ToF: appeared from the left at +20 deg, 1.0 m
   13.4 ToF: gone on the right at -20 deg, 54 deg/s
   20.0 ToF: stopped at -5 deg, 0.61 m (now background)
   31.2 ToF: removed at -5 deg (the background again)
```

While turning the movement line adds the yaw: `… (turning at -57 deg/s, yaw 132)`.

## 7. Watching while turning: the heading background

### 7.1 The problem in numbers

| Turn rate | Turned per frame | Of a zone | A zone's swept cone | Swept cone + heading error (§7.3) |
|---|---|---|---|---|
| still | 0 | 0 | 5.6° | 6.6° |
| 0.5 rad/s (29°/s) | 1.9° | 0.34 | 7.5° | 8.9° |
| 1 rad/s (57°/s, today's watch turns) | 3.8° | 0.68 | 9.4° | 11.4° |
| 1.5 rad/s (86°/s) | 5.7° | 1.02 | 11.4° | 13.8° |

So the prediction is never "the zone to the left": each frame each zone has swept
a cone wider than itself, and its range is the nearest surface in that cone that
returned enough signal (closest-first order). The prediction is therefore the
**nearest** background over the swept cone ± the heading error.

### 7.2 The bins

Per row, 256 bins of 1.406° (a quarter zone; a uint8 bin index wraps at 360°),
indexed by world bearing. Each bin holds `bg_mm` (u16), `cand_mm` (u16), `spread`
(u8, mm/4) and flags with a coarse age (u8: known, nothing-seen, pinned): 6 bytes,
8 × 256 × 6 = 12.3 KB.

What a reading tells about bins: a zone reports the nearest surface in its cone,
so a reading r means "every bearing in the cone is at least r away, and one is at
r". It is a **lower bound** for every bin it fully covered. Bins therefore keep the
largest lower bound that has been confirmed:

1. **Which bins a reading updates:** those inside the cone it covered for the
   whole integration (the zone minus the turn: 5.6° still, 1.8° at 1 rad/s, none
   at 1.5 rad/s), so a fast turn only predicts, it doesn't update. Turning at 1
   rad/s, each bin passes through all 8 columns' cores: ~8 updates per row per pass.
2. **Raising:** a credible reading not nearer than the prediction proposes `r −
   2·sigma_eff` for its bins. If the bin's `cand_mm` already proposes about the same
   (within 2 sigma), `bg_mm` becomes it; otherwise it becomes the candidate. Two
   readings are needed, as today's learning ignores one odd near frame; a single far
   frame (a reflection, a far zone's farther surface) never raises a bin.
3. **Pinned bins** (stopped things, brief nearer surfaces, §5 rule 11) are lowered
   to that range and are raised again only after 10 s of agreeing farther readings
   (today's rule).
4. **Nothing:** a zone with no target marks its core bins "nothing seen" if they
   have no range yet; a credible target there later is nearer. A zone that sees
   "nothing" where bins hold a range is farther: never movement.
5. **Spread:** the mean |r − bg| of readings that agree with the bin, at least the
   readings' own sigma: `σb`.
6. **Prediction for a zone:** the bins under its swept cone (yaw at `t_us − 33.3 ms`
   and `t_us + 33.3 ms` from `pose_at`, plus the column's ±2.8°), widened by the
   heading error each side; `P` = the smallest known `bg_mm` there, `σb` its
   spread. If more than a third of those bins are unknown, the zone tells nothing.
7. **Translation:** the bins remember the robot's position when filled; once it
   has moved > 10 cm (driving; turning in place slides ~1.5 cm per turn, fine) all
   bins are unknown again and refill (~2 frames still). Driving straight (M4) is
   out of scope; ROBOT_PLAN §6 leaves movement detection off while driving.

### 7.3 What it costs and where it is honest

- **The column entering the view.** Turning, the leading column looks at bearings
  the robot faced earlier. After the 390° start-up scan every heading has bins, so
  this column is predicted like any other: never "unseen" until the robot drives.
  After driving, new headings are unknown: the leading ~11° of the view (the bins
  need 2 readings, i.e. the first two columns to pass over them at 1 rad/s) tells
  nothing.
- **Heading error.** Gyro scale 0.42 % low (~1.5° per full turn), bias re-measured
  when still. The error that matters is between when the bins were filled and now:
  widening = 0.5° + 0.42 % of the yaw turned since the bins were last confirmed +
  |ω| × the frame-time uncertainty (10 ms until `TOF_PROC_US` is measured: 0.6° at
  1 rad/s). Bins are reconfirmed whenever seen, so it stays ~1°.
- **Frame time.** A 10 ms error is 0.6° at 1 rad/s. `TOF_PROC_US` is measured in
  M-S: turning left and right past a door edge, the edge's world bearing must agree
  both ways; the disagreement divided by 2ω is the offset.
- **Depth edges and mixed zones.** A zone over a near-far edge reports the near
  surface, two targets, status 9 / 12, or a range in between; all are ≥ the near
  surface. Door edge at 1 m, wall at 3 m: every zone whose widened swept cone
  touches the door predicts 1.0 m, and every possible reading there is ≥ 1.0 m − a
  few sigma, so z < 4: quiet at 0.5 and at 1 rad/s, as long as the heading error
  stays inside the widening. The host test checks it with a ±1.5° error.
- **The price: masking near depth edges.** Within the widened swept cone of a near
  surface, only things nearer than that surface count. At 1 rad/s the masked strip
  beside each near edge is ~11.4° − 2.8° ≈ 8.6° wide (still: ~3.8°): a walker at
  2 m passing just behind a door edge at 1 m is missed for ~0.3 s at 29°/s. In a
  room with 2-3 such edges in the 45° view, that is ~20-40 % of the view masked for
  targets between the two depths while turning fast, ~10 % while still.
- **Floor rows while turning.** The robot rolls −3…−5° turning, and a floor zone's
  range is steep in its angle (row 5: 7 % per 1°; row 4: 35 % per 1°). Floor bins
  learned level would be wrong, so floor rows predict the floor from
  `view_floor_patch` at the frame's pitch and roll (A2 adds roll), and use bins only
  for things above the floor (bins whose range is nearer than the floor patch).
  Row 4 tells nothing while |roll| > 2°.
- **Rates while turning.** Fits are in world bearings (§4), so a mover's world rate
  comes out directly; seen from the robot it moves at its world rate − ω_robot.

### 7.4 Where the background lives: heading bins or the map

| | (a) Heading bins (chosen) | (b) Ray-cast each zone into the surroundings map |
|---|---|---|
| Vertical reach | every row, any range | the voxel layers are 3-23 cm: rows 0-3 beyond ~40-60 cm look above them, unpredictable |
| Range resolution | the readings' own (mm), σb per bin | 10 cm cells: ±5 cm quantisation against a 30 mm / z ≥ 4 test |
| Movers | never raise a bin (needs 2 agreeing frames, and movers are nearer) | the map keeps a walker's trail ~0.5 s, and nothing fades: false "background" right where the target walks |
| Coupling | `tof_motion` alone | movement would depend on the map's update order |
| Wrong after | the robot drives > 10 cm (cheap to detect) | stays right while driving: the one advantage |
| RAM | 12.3 KB | 0 extra |

(a), with (b) as a later cross-check (once the map exists, compare predictions for
rows 4-7 near the robot). Owner: `tof_motion` (answers the old open question 5).

## 8. Worked examples

Assumed until M-S measures them: σ ≈ 8 mm for a strong target at 1 m, ≈ 20 mm for
a wall at 2.5-3 m, σb ≈ 20 mm. Frame k is k × 66.7 ms after the first onset.

**Walking at 1 m/s, 1 m away, left to right, wall at 2.5 m; robot still.** Legs at
Δ = 1.5 m: z ≈ 1500 / 22 = 68. The front leg enters column 0 (rows 0-4): onsets at
frame 0. With a neighbour row nearer too, the zones are moved at frame 1 (2 of 4):
first observation at ~0.07-0.13 s, `APPEARED side +1` (the onset is known at frame
1 in hindsight). The swinging leg crosses into columns 1-2 within ~2 frames: sign
"going right" at frame ~2-3 (0.13-0.2 s), coarse rate 40-140°/s. The front foot
lands and stays ~0.67 s while the other leg swings past it: the leading edge
jumps 5.5 columns per step. After ~0.55 s (8 frames) the chain spans a step:
57 ± 7°/s. The leading edge reaches column 7 at ~0.8 s; `LEAVING` from the
tracker at the first outer-column observation (it needs only the sign and ≥
10°/s); the last leg's offset in column 7 at ~1.0 s; `GONE side −1` at ~1.15 s.
Today: rate good after ~0.5-0.8 s, exit reported ~1.5 s.

**The same at 2 m, wall at 3 m.** Legs at Δ = 1 m, z ≈ 1000 / 28 = 35, rows 0-3.
Each leg covers ~60 % of a zone: the zone reports the leg if its share of the
signal is enough, as a second target behind it the wall (≥ 60 cm apart, to be
verified). A step spans 2.8 columns, a column every 2.9 frames on average. First
observation ~0.13-0.2 s, sign ~0.4 s, walking rate after ~0.55-0.7 s: 29 ± 3°/s.
In view ~1.6 s (45° + a 13° stride at 29°/s). With dark clothes and status 9 or 10:
0.5 per frame, moved after 4 frames instead of 2 (0.27 s).

**Walking at 1 m while the robot turns after it at 1 rad/s (57°/s).** Robot and
walker turn alike: in the robot frame the walker stays near the middle, its legs
scissoring. The stance foot is still in the world, so it slides across the columns
at −57°/s: onsets and offsets in the robot frame, but a constant world bearing:
it fits 0°/s; the swing leg fits ~140°/s; their blob's world bearing over 0.8 s
gives ~57°/s. The wall at 2.5 m sweeps past at −57°/s and stays predicted: its
zones are never nearer. Observations every frame during the turn.

**Walking the other way during a 1 rad/s turn.** Relative speed 114°/s: the
walker crosses the 45° view in ~0.4 s (6 frames). Moved by frame 1-2, observed
for ~4 frames; world rate from the blob's world bearing over those 4 frames (~0.27
s): a sign and a coarse rate only.

**A door edge at 1 m, wall at 3 m, robot turning at 1 rad/s, nothing moving.**
Zones over the edge read 1.0 m, 1.0 m + 3.0 m (two targets), a merged ~1.6 m
(status 9, σ doubled) or 3.0 m. The prediction for every zone whose widened swept
cone (11.4°) touches the door's bins is 1.0 m. None of those readings is nearer
than 1.0 m − 4σ: quiet. With the heading off by 2.5° (beyond the widening), the
zone just past the edge reads the door at 1.0 m while bins say 3.0 m: one zone,
one frame, z = 70: a false onset; 2 of 4 frames with a neighbour does not follow
from one frame. The host test sets the limit (§12 T-R).

**A hand waved at 30 cm, desk at 1.5 m.** Signal hundreds of kcps/SPAD, σ ~5 mm,
z > 100. The hand spans ~2.7 columns; waved ±10 cm at 1 Hz it reaches ~120°/s, 1.4
columns per frame, so neighbouring columns often get onsets in the same frame.
Expected: one observation the whole time, the sign flipping every ~0.5 s; two flips
in 1.5 s mark it oscillating (logged, no turn; ROBOT_PLAN §6.5's "react or log").
The floor echo at ~30 cm is not the hand: signal 3-10, unsure status.

**A box (30 × 20 cm) put down at 60 cm, then left.** It spans 28° (5 columns) and
rows 2-4 (row 5 sees the floor at 48 cm, in front of it). Hand and box come in
(chains, likely vertical), observations with the hand's rate. The hand's offsets
run out; the box zones stay nearer, steady (±1 cm), and nothing in their
observation changes → after 1 s `STOPPED at −5 deg, 0.61 m`; its bins are pinned
at 0.61 m, the covered background remembered. Taken away: the hand is movement;
the box zones read the remembered background steadily for 1 s → `REMOVED`, the old
bins back. Today: only "ended", and the box zones wait 10 s.

**The floor echo.** Zone row 2, col 3: target 1 at 290 mm, status 4, signal 6,
σ 40 → confidence 0, into `weak_mm` = 290; target 2 at 2000 mm, status 5, σ 15 →
`target = [2000]`, the zone's reading. Nothing nearer than the 2.0 m bins. If M-S
shows the echo sometimes with status 9, the echo guard takes it to 0.05, below
`CREDIBLE`: never even a reading.

**A status-12 zone flickering** (a sofa edge at 1.2 m, wall at 2.6 m). Bins: 1.2 m
(confirmed by the status-5 frames). Status 12 → 0: not credible, no evidence. If
M-S lifts 12 to 0.25 and it reads nearer than the bins every frame: 4 × 0.25 = 1.0
< 2.0, never moved on its own. Readings at the wall are farther: never movement.

**Curtain sway** (±2 cm at 2 m, σ 20 / 20): z = 0.7: nothing. **Someone 6 cm in
front of a wall** at 1.5 m (σ 10 / 10): z = 4.2, 60 mm → nearer; today's rule
needed 12 cm.

## 9. Compute and memory

Per frame: `tof_quality` for 64 × 4 targets (~10 operations each), 64 predictions
(the minimum over ≤ 10 bins), 64 z-tests, bin updates (≤ 4 per zone), a few
hundred chain checks, a least-squares fit per chain: ~8 000 cycles, ~55 µs at 150
MHz, under 0.1 % of the 66.7 ms frame. The SPI read (existing) dominates.

| What | Bytes |
|---|---|
| `tof_frame_t`: producer + 2 reader copies | 3 × 1.4 K = 4.2 K |
| Heading bins: 8 rows × 256 × 6 B | 12.3 K |
| Per zone: steps, steady timers, remembered background | 64 × 16 = 1 K |
| Chains: 16 × 24 points × (bearing, time) | ~3 K |
| Today's learning buffer (15 × 64 floats) | −3.8 K |
| **Net** | **~17 K** (cross-plan budget: +20 K) |
| `TOF_RAW` record (R0 only) | 3.1 K staging |

## 10. What others did

- **ST's on-chip motion indicator** (ULD plugin): per-zone movement within a depth
  window ≥ 40 cm, ≤ 1.5 m wide, 16 aggregates at 8 × 8, its own 16-frame reference.
  Checked and rejected here (CHANGELOG 6 Oct): no hand at 30 cm, no range, useless
  while turning. [ST forum](https://community.st.com/imaging-sensors-49/example-10-motion-indicator-example-23385)
- **ST gesture library** (STSW-IMG035, hand X/Y/Z, taps and swipes near the
  sensor) and **Smart Presence Detection** (STSW-IMG048, a user in front of a
  laptop): closed libraries for a still sensor and one near target. They show that a
  swipe's direction is readable from 8 × 8 zones at 15 Hz; nothing about people at
  1-3 m or a turning sensor.
  [STSW-IMG035](https://www.st.com/en/embedded-software/stsw-img035.html),
  [SPD thread](https://community.st.com/imaging-sensors-49/stm32-vl53l7cx-smart-presence-detection-146877)
- **ST people counting with the VL53L1X** (UM2600): two ROIs, a person is "under"
  a ROI when the distance is below a threshold, and the order in which they trigger
  gives the direction: the two-detector idea with a distance threshold. Their lesson
  on timing (ST: with 50 ms per zone you get one sample per person) is ours at 1 m.
  [UM2600](https://www.st.com/resource/en/user_manual/dm00626942-counting-people-with-the-vl53l1x-longdistance-ranging-timeofflight-sensor-stmicroelectronics.pdf),
  [Roode](https://github.com/gorbunovav/Roode)
- **8 × 8 thermal arrays (Grid-EYE):** Basu & Rowe subtract a per-pixel long-term
  background, cross-correlate pixel series and read direction from the lag pattern
  (1-2 samples between neighbours at 10 Hz): similar to edge times, over windows.
  Simple open code thresholds against the mean (room-assistant); a survey notes that
  optical flow and gradient features don't work at such resolutions.
  [Basu & Rowe, arXiv 1511.08166](https://arxiv.org/abs/1511.08166),
  [room-assistant Grid-EYE](https://www.room-assistant.io/integrations/grid-eye.html),
  [arXiv 1811.05416](https://arxiv.org/abs/1811.05416)
- **PIR direction:** dual-element and interleaved PIR sensors read direction from
  the order and polarity of two time-shifted signals.
  [US 5291020](https://patents.justia.com/patent/5291020),
  [US 2005/0184869](https://patents.justia.com/patent/20050184869),
  [two dual sensors at 90°](https://eejournal.ktu.lt/index.php/elt/article/view/11187/5923)
- **Reichardt / Hassenstein correlator** and Barlow-Levick: delay-and-multiply (or
  delay-and-veto) between neighbours, mirror subtracted to cancel flicker; output
  depends on contrast and speed tuning.
  [Review, PMC8097889](https://pmc.ncbi.nlm.nih.gov/articles/PMC8097889),
  [Burr, motion detectors](https://win.pisavisionlab.org/Downloads/motion_burr.pdf)
- **Background subtraction on depth:** Greff et al. compared methods on Kinect depth
  and found the simplest, "minimum background" (foreground = nearer than the nearest
  seen while training), as good as codebooks and faster; invalid depth is background.
  That is today's rule, kept in the bins. Per-pixel Gaussian mixtures
  (Stauffer-Grimson) are for intensity; depth noise grows with range (Nguyen et al.:
  quadratic for the Kinect), hence a per-reading sigma instead of a fixed threshold.
  [Greff et al. 2012](https://scitepress.org/Papers/2012/38491/pdf/index.html),
  [Stauffer & Grimson 1999](https://www.cse.buffalo.edu/courses/cse725/peter/Stauffer_1999.pdf),
  [Nguyen et al. 2012](https://computer.org/csdl/proceedings-article/3dimpvt/2012/4873a524/12OmNAlvHUP)
- **Probabilistic occupancy:** Elfes' inverse sensor model and log-odds updates; a
  per-reading confidence is the weight of one update (the map's use of §3).
  [Mapping slides, Edinburgh](https://homepages.inf.ed.ac.uk/msridhar/Teaching/MobileRobotics/Slides/S9_Mapping.pdf),
  [Confidence-rich grid mapping, arXiv 2006.15754](https://arxiv.org/abs/2006.15754)
- **VL53 status and sigma:** ST forum answers on status 6, 9, 12 and validity;
  sigma as a quality filter after the status.
  [status 6](https://community.st.com/imaging-sensors-49/vl53l5cx-what-does-status-6-mean-42122),
  [status validity](https://community.st.com/imaging-sensors-49/vl53l5-l7cx-target-status-validity-136986),
  [sensor features](https://community.st.com/imaging-sensors-49/analysis-of-vl53l8cx-sensor-features-166933),
  [UM3109](https://www.st.com/resource/en/user_manual/um3109-a-guide-for-using-the-vl53l8cx-lowpower-highperformance-timeofflight-multizone-ranging-sensor-stmicroelectronics.pdf),
  [datasheet](https://www.st.com/resource/en/datasheet/vl53l8cx.pdf)

What we take: the minimum background (kept, now per heading), a threshold scaled by
the reading's own noise, two-detector order for direction (ST counting, PIR,
Reichardt), the lag between neighbours for speed (Basu & Rowe), done as edge times.

## 11. Rejected alternatives

- **Suspending detection while turning** (today, and this plan's first draft):
  against Daniel's directive, and the 1.5 s blind time after each stop is the main
  cause of the M3b misses.
- **Shifting the last frame's background by the rotation, zone by zone:** a turn
  is a fraction of a zone per frame (0.68 at 1 rad/s), a frame smears the view by
  as much, and fractional shifts mix a near and a far surface into a range nobody
  saw; the column entering has nothing to shift in. The heading bins do the same
  job without these faults.
- **Ray-casting into the map as the background:** §7.4.
- **Gaussian blur or a 3 × 3 median on the grid:** a zone is already an angular
  average; a median erases a 2-column walker at 2 m (a leg is 0.6 column) and the
  edge we time. Noise is per zone in time, handled by sigma and confidence.
- **Frame-to-frame differences:** miss slow movement, flicker with status changes,
  and every zone changes while turning.
- **Windowed cross-correlation of every pair** (Basu & Rowe): needs ~1 s windows
  with both zones active; for step signals it gives the same lag as edge times,
  later and dearer. Kept as an offline check (`replay --xcorr`).
- **A Reichardt bank with fixed delays:** a speed needs several delays, and the
  answer depends on contrast; edge times give the speed directly.
- **Per-zone Gaussian mixtures / Kalman filters:** built for intensity backgrounds
  and smooth states; zones switching between surfaces are handled by the minimum
  background. More state, more tuning, the "bits" lesson again.
- **ST's motion indicator, gesture and presence libraries:** §10.
- **A neural network (ST's people counting):** no data, no need.
- **Interpolating ranges between zones:** mixes surfaces at depth edges.

## 12. Milestones and tests

In the merged order of the cross-plan review: **R0** (5) and **M-S** (6) carry what
the ToF needs from recording and measuring; then **T1** (7, with A4), **T2** (9),
**T-R** (10), **T3** (18), **T4** (19, with A6 and V6); **T5** later. Every
milestone's robot test also prints `p`'s RAM free (fails under 40 KB).

### R0 and M-S — what the ToF needs from them

**R0** (shared recording, ARCHITECTURE_PLAN / cross-plan §3): a `TOF_RAW` record
per frame: seq, `t_us`, INT time, and per zone `nb_target_detected`, ambient,
nb_spads, all 4 targets (distance, sigma, signal, reflectance, status): the
bring-up `TOF3` layout (`tof_stream.c`; `pc/bringup` parses it), ~3.1 KB,
~47 KB/s at 15 Hz; plus `ODOM` at 50 Hz (yaw for the bins) and `MARK`.
`replay --tof-stats file.rec`:
- per status: count and share, overall and **in onset frames** (the first frame a
  zone is nearer than its background) separately;
- per zone over still parts: the measured standard deviation of the nearest
  credible target against its mean reported sigma;
- signal and ambient ranges; every credible target at 15-35 cm with signal < 15 in
  zones above the floor rows (the echo with a good status);
- status-12 ranges against the zone's status-5 median;
- `--csv`: (t, zone, target, range, sigma, signal, status, yaw) for plotting d_i(t).

Host tests: `TOF_RAW` round trip; `--tof-stats` on a synthetic recording with σ =
10 mm: measured σ within 10 %; status shares exact; onset-frame statuses counted
only at the synthetic onsets.

**M-S** (one session for all plans, cross-plan §4). The ToF steps (motors off
except 9-10; tape lines at 1 m and 2 m from the sensor, 1.5 m each side; a phone
stopwatch). `R` before each step, `R` after; after each `R` off expect "Recording
off, N records, 0 dropped", N ≈ 15 × seconds `TOF_RAW` + 50 × seconds `ODOM`, the
file growing ~50 KB/s. A step with drops is repeated over USB.
1. Facing ~2.5 m of room, stand behind the robot, still, 2 min ("quiet").
2. Walk left to right at 1 m at a normal pace (3 m of tape in ~3 s), then right to
   left; the same at 2 m; once at 2 m in dark clothes. Say the times aloud.
3. Wave a hand ~30 cm in front, 5 s.
4. Put a box down at ~60 cm, step away, wait 5 s, take it away, wait 5 s.
5. Stand at 1 m for 10 s, walk straight away to ~3 m, stand, come back.
6. A person (or a board) at 30, 60, 90 cm in front of a wall, 10 s each (two
   targets or status 9: verifies the ~60 cm claim).
7. Facing a sofa or door edge at ~1.2 m with a wall behind, 1 min (status 12).
8. Near a sunny window or a lamp pointing at the sensor, 30 s (ambient).
9. A door edge ~1 m away, wall ~3 m behind: `r` turns (10 × 360° at 1 rad/s), and
   slow turns by hand (~0.3 rad/s) left and right past the edge.
10. `n` (scan).

Expected / paste: `--tof-stats` per file, and from step 9 the door edge's world
bearing turning left and turning right (`replay --tof-edge`). Decides: the status
table (§3), the echo guard, sigma against scatter, leg widths, stance times and
edge jitter from step 2 (T3's tolerances), whether two surfaces 60 cm apart give two
targets, `TOF_PROC_US` = bearing disagreement / 2ω. Commit 2-5 s clips of steps 1-9,
gzipped, to `picoA/app/test/recordings/` (if Daniel agrees to clips in git).

### T1 — `tof_quality`, `tof_frame_t` (no behaviour change; with A4)

`tof_quality` per §3 with M-S's numbers; `tof_sensor` fills `tof_frame_t`; the map
and `tof_motion` use "confidence ≥ 0.5" where they used "sure", so nothing changes
yet (`CREDIBLE` 0.25 takes effect in T2).

Host tests (`test_tof_quality`):
- Status 5 → 1.0; 6, 9, 10 → 0.5 (or M-S's values); 0-4, 7, 8, 11, 12, 13, 255 → 0.
- `sigma_eff`: reported 2 mm at 2 m → 13 mm; status 9 doubles it.
- The README echo (290 mm, status 4, signal 6; then 2000 mm, status 5):
  `target = [2000]`, `weak_mm = 290`; with the echo guard and status 9: confidence
  0.05, still `weak_mm = 290`.
- Targets [echo, person 1000, wall 2500] → `target = [1000, 2500]`.
- `clear_m`: 1.0 m at indoor ambient, less above M-S's window value.
- Equivalence: every M-S clip and `test_tof_motion`'s room replay with the nearest
  target of confidence ≥ 0.5 equal to today's `closest_sure_mm`, zone for zone,
  frame for frame; `test_behaviour`'s printed map unchanged.

Robot: `z` gains a block per zone "confidence 0-9 / σ cm" for both targets, and
`weak` in cm.
1. Facing a wall at ~1 m: wall zones `9 / 1`; zones above the floor rows looking
   into the room: `weak` ~30 (the echo), the first target the real one.
2. A dark and a white object at ~1 m: the dark one higher σ (lower confidence only
   if its status says so). Paste both `z`.
3. **Movement, VL53** steps 1-5 and **Target, VL53** step 1: the same lines as before.

### T2 — z-test and confidence-weighted evidence (still)

§5 rules 1-4 with today's per-zone background (the bins come in T-R): nearer by
z ≥ 4 and ≥ 30 mm, score = confidence, `CREDIBLE` 0.25, observation confidence.

Host tests (`test_tof_motion`, its room now made by `tof_sim.h`: per zone a
background surface with σ from M-S, the status mix, the echo, a status-12 zone, row
5's reflections; objects in bearing / elevation with coverage per zone; two
surfaces ≥ 60 cm apart reported as two targets, closer ones merged as status 9 (or
as M-S found); **walkers as two legs** of 12 cm up to the view's top, gait 1.8
steps/s, step 0.55 m, stance 60 %):
- The quiet room for 5 min, 20 seeds: no observation.
- A status-12 zone flickering every other frame, at 0.25 and reading nearer: never
  moved.
- Someone 6 cm in front of a wall at 1.5 m: moved within 3 frames; a curtain
  swaying ±2 cm at 2 m: never.
- A walker (legs) at 1 m and at 2 m: moved within 2 frames of the first leg
  entering; at 2 m in status 9 only: within 4.
- A walker's stance foot never STOPPED (steady 0.67 s, its other leg changing); a
  slow walker (0.5 m/s, stance 0.86 s): not STOPPED either.
- Today's cases still pass (pole in 3 frames, hand the whole time, box still after
  ~1 s, taking it away not movement).
- Confidence: a walker ≥ 0.9 after 3 frames; with noise doubled, the quiet room's
  rare observations below 0.5.

Replay gate on the quiet clips: **at most 1 false observation per 10 min** in
total, and no more than today's detector; walk clips: first observation no later
than today's (print both per walk).

Robot: **Movement, VL53** steps 1-5 again, plus:
6. Stand 5-10 cm in front of a wall ~1.5 m from the robot, then step sideways
   along it. Expected: "Movement (ToF)" (today often none). Paste the log and `o`.

### T-R — The heading background: watching while turning, no learning

§7 entire; `tof_motion_restart` and the 1 s learning go; `motion_sense` feeds every
frame, still or turning, with `pose_at` for both ends of the integration;
`find_passing` and `MOVE_PASSED_WHILE_LEARNING` are deleted.

Host tests (`tof_sim` with a 360° room: walls at 1-3 m, a door edge at 1 m with a
wall at 3 m behind, furniture, the floor; `pose` from a simulated turn with roll
−4° while turning; heading error up to ±1.5° and `TOF_PROC_US` error ±10 ms):
- 5 full turns at 0.5, 1 and 1.5 rad/s, after a simulated 390° scan: **zero
  observations**, the door edge included.
- The same with the heading error raised until the first false observation:
  print the margin (expected: errors under ~2° stay quiet).
- A walker (legs) at 1 m crossing during a 1 rad/s turn, the same way and the
  opposite way: observed within 3 frames of entering; its world rate within T3's
  limits once T3 exists (here: the sign right).
- Stop after a 45° turn with a walker in view: observed in the first still frame
  (today ~1.5 s later).
- After a simulated 20 cm drive: no observation from unknown bins; the leading
  ~11° of a 1 rad/s turn into new bearings tells nothing; still, the bins refill
  within 2 frames.
- A box put down while the robot faces away, then the robot turns back to it:
  nearer than its bins, movement, then STOPPED after 1 s; turning away and back
  again: quiet (pinned bins).
- Floor rows with roll −4°: no "nearer floor" observations.
- Replay the M-S step 9 recordings (turns past the door edge): no observations;
  `TOF_PROC_US` from the edge's two bearings within ±5 ms of what M-S printed.

Robot (both Picos, battery on, `v` on):
1. `n`, then `a`. Walk past at 1 m and at 2 m so it turns. Expected: "Movement
   (ToF) … (turning at N deg/s, yaw Y)" lines **during** each turn, and no
   "learning the view" in `o` after it; `o` shows "watching" all the time. Paste
   the log.
2. Empty room (stand behind it), `r` (10 turns): no movement lines. Paste any,
   with the yaw.
3. Turn the robot slowly by hand (motors off) past a door edge, both ways: no
   movement lines.
4. The **Watching** test (README) and 10 walks across at 2 m (5 each way): count
   turns and misses; compare with the 7 Oct run (11 turns, "often" missed while
   learning). Paste the log and the counts.
5. `p`: RAM free; the line `TOF_PROC_US` in use.

### T3 — Edge times and rates (still and turning)

§5 rules 5-8; `rate_known`, `rate_radps`, `rate_sigma_radps`, `range_rate_*`. The
tracker still uses its own fit (both printed).

Host tests (`tof_sim`, 20 seeds, legs, onset jitter from M-S):
- Walkers at 1 m/s at 1 m, both ways: the sign right in every run by the 3rd
  column; after one step of edges |rate − 57°/s| within M-S's measured spread (the
  plan's estimate: ±9°/s); `rate_sigma` honest (the true rate within 2 sigma in ≥
  90 % of runs).
- At 2 m: within ±3°/s after a step (to be confirmed by M-S).
- At 3 m (a leg 0.4 column): a sign, and a rate within ±5°/s after two steps.
- A box carried sideways (a smooth edge) at 1 m: within ±11°/s from 3 columns.
- Hand waving at 120°/s peak: the sign flips every 0.5 ± 0.1 s.
- A box lowered from above: a vertical chain, |horizontal rate| < 10°/s.
- Appearing in 4 zones in one frame: no rate, not a chain.
- During a 1 rad/s turn: a walker going the same way and the opposite way, world
  rate within the still limits × 1.5; a stance foot alone fits |rate| < 10°/s.
- Two walkers at 1 m and 2.5 m crossing: chains not joined.
- Walking straight at the robot at 0.5 m/s: range rate −0.5 ± 0.1 m/s; the
  sideways rate limit set from M-S step 5 (legs scissor), not 5°/s by assumption.

Replay: the walk clips print both rates; expected the edge rate's sign ≥ 0.2 s
before the tracker's and the walking rate within the tracker's ±25 %. Paste the
table.

Robot (motors off; tape at 1 m and 2 m; a stopwatch):
1. Walk the 3 m of tape at 1 m in ~3 s, left to right. Expected: "going right"
   from the 2nd or 3rd movement line, "at ~55 deg/s (±N)" from ~0.6 s on.
2. Same at 2 m: ~28 deg/s.
3. Slowly (~0.5 m/s, 6 s for the tape) at 1 m: ~28 deg/s.
4. Walk straight at the robot from 2.5 m to 0.7 m: a small sideways rate, "coming
   at 0.N m/s".
5. Wave a hand at 30 cm: the sign flipping.
6. Motors on, `a`, walk past: during its turn the movement lines show a world rate
   near your walking rate, not the turn's.
Paste the logs with the times you measured.

### T4 — Events and the tracker (with A6 and V6)

§5 rules 9-11, §6.2. `movement` maps the events; the tracker takes `rate_known`
and GONE at the edge.

Host tests:
- `test_tof_motion`: a walker in from the left → APPEARED side +1, GONE side −1
  within 0.2 s of its last frame; stepping out from behind a box in the middle →
  APPEARED side 0; box put down → STOPPED at ~1 s, taken away → REMOVED at ~1 s;
  standing 2 s then walking straight away → movement again (remembered
  background); a status-12 zone and the quiet room → no events; during a turn,
  something entering through the leading column → APPEARED on that side.
- `test_tracker`: LEAVING on the first outer-column observation with an outward
  known sign and ≥ 10°/s; none for a hand oscillating at the edge; EXITED from GONE.
- `test_behaviour`: the same turns as today (−71°, −85°, +43°, at most 90°), and a
  leaving seen 0.3 s sooner starts the turn 0.3 s sooner.

Replay: each walk clip gives one APPEARED and one GONE with the right sides; the
box clip STOPPED then REMOVED; the quiet clip nothing.

Robot (both Picos, `v` on): the merged milestone's test (cross-plan step 19): 10
walk-pasts × 2 distances × 2 ways, misses counted, plus:
1. **Target, VL53** steps 1-3: "ToF: appeared from the left", "gone on the right"
   ~0.1-0.2 s after you are out of view (today ~0.5 s).
2. Box at 60 cm: "stopped … (now background)" ~1 s after you let go; take it:
   "removed" ~1 s later.
Paste the log and the counts.

### T5 — Fill fraction (later, optional)

The near target's signal over its full-coverage signal as a sub-zone edge
position. With legs this may be too noisy; only if T3's replays show rate errors
worth halving. Host: the 1 m smooth-edge error falls below ±6°/s from 3 columns.

## 13. Risks

- **The ULD's sigma may not match the real scatter** (merged targets, multipath).
  M-S measures it; the floor and the status-9 factor absorb the gap; thresholds are
  checked on the quiet clips.
- **Heading error beyond the widening** gives false onsets at depth edges while
  turning. The widening grows with the yaw turned since confirmation; T-R's host
  test prints the margin; `r` on the robot checks it.
- **Masking near depth edges while turning** (§7.3): a walker just behind a near
  edge is missed for a few tenths of a second at 1 rad/s.
- **Floor rows while turning** depend on roll from A2; until then row 4 tells
  nothing while turning and rows 5-7 use a wider margin.
- **Legs are thin at 2-3 m** (0.4-0.6 of a zone): a zone may report the wall and
  not the leg. M-S's 2 m walks show how often; the hips and hands at 2 m help.
- **More sensitive is more false movement** (a door swinging slowly, a pet). The
  30 mm floor and T2's absolute gate limit it; fallback: 50 mm.
- **Rate needs a step:** a walker at 1 m gives only a sign and a coarse rate for
  the first ~0.55 s. The behaviour's leaving rule needs only that.
- **Several people:** chains are kept apart by range; two at the same range
  crossing merge (the tracker's problem, as today).
- **Recording bandwidth:** `TOF_RAW` at ~47 KB/s next to images; R0's throughput
  test decides; record the ToF steps over USB if needed.

## 14. Open questions

1. Status weights for 6, 9, 10, 12, and whether the echo guard is needed (M-S).
2. 15 Hz or 10 Hz (ROBOT_PLAN §6.2): 10 Hz has less noise but 100 ms frames (a
   swept cone of 11.4° at 1 rad/s becomes 11.4° + 1.9°). And the sharpener (5 %
   now): does 0 % reduce status 12?
3. Does `tof_frame_t` need 3 credible targets per zone? M-S's count of zones with
   3 credible targets decides; 2 otherwise.
4. Bin size: 1.4° (a quarter zone) is a guess; 2.8° halves the RAM and may be
   enough given the heading widening. T-R's host test compares both.
5. Calibrating confidence as a probability needs labels: is "quiet clip = all
   observations false, walk clip = the walker's frames true" enough?
6. Driving (M4): keep watching with the bins shifted by the translation (a
   ray-cast per bin), or leave it to the map's changes as ROBOT_PLAN §6 says?
