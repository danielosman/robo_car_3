# Camera: features, their measured moves, and movers (plan)

Status: **plan, nothing implemented.** It replaces the camera half of ROBOT_PLAN.md
§6.3/§6.4 (`camera_motion.c`: 8 × 8-pixel blocks against a learned background).
Written with [TOF_MOTION_PLAN.md](TOF_MOTION_PLAN.md) (the VL53's movement, also
while turning) and [ARCHITECTURE_PLAN.md](ARCHITECTURE_PLAN.md) (module design);
the shared types, numbers, recording path and milestone order are the reviewed
cross-plan decisions (§13 lists where this plan sits). Terms as in
[GLOSSARY.md](GLOSSARY.md): **feature**, **measured move**, **ego-motion** (the
robot's own turn or move), **predicted move**, **leftover move**, **mover**,
**region**. Sources are in §15.

## 1. In short

- The camera image is turned into **features**: small high-contrast patches, each
  with an id, a position, an age and its **measured move** since the last frame.
  FAST finds candidate corners, a Shi-Tomasi score picks the features, and a
  normalised patch search (ZNCC) finds each feature again in the next frame.
- One module, `vision`, does only that, on core 1, and publishes one
  **`vision_frame_t`** per processed frame. **It reports only what the camera
  measured**: per feature its position, measured move, strength, match score, age,
  id, flags and row time; per region its feature count, texture, brightness and
  median measured move; per frame the exposure and gain, and the robot's pose at
  the frame's row times, copied from `pose` as known facts. No predicted move, no
  leftover move, no ego-motion fit is in the record.
- `vision` does use the gyro inside, as a **search hint**: it looks for a feature
  where the robot's turn should have taken it. That only decides where to search;
  the reported move is always the measured one.
- **Removing the ego-motion is the readers' job.** The pure helper module `ego`
  (`ego_predict()`, `ego_fit()`) gives, from a `vision_frame_t`, each feature's
  predicted move (exact rotation, per row for the rolling shutter), a robust fit of
  what the image adds to the gyro, and each feature's leftover move. Any reader
  calls it; nothing of it goes back into the record.
- **The movement detector is one reader** (`camera_movers`): features with a clear
  leftover move are moving; moving features in one place over ~0.3 s are one
  **mover**, reported as the shared `motion_obs_t` (§9.1) with a bearing (+ = left),
  a world-frame angular speed and a confidence, like the VL53's observations, which
  now also come while turning.
- Geometry (camera intrinsics, mounting, time offset) lives in the shared `view`
  module.
- Later readers, without touching `vision`: visual yaw (a gyro check), landmarks
  for the full-turn check (ROBOT_PLAN §5.4), edges, visual odometry. Readers that
  need pixels read the image itself through `camera_next()` (multi-reader,
  ARCHITECTURE_PLAN A2).

Why change: the block detector (CHANGELOG, 6-7 Oct) works only while still, needs
1 s to learn after every stop, must hold the exposure and go blind 1-3 s to change
it, sees every shadow and lamp as movement, misses small things faster than
~50°/s, and its blocks are useless for anything else.

## 2. What the robot needs from the camera

| Need | Now | With this plan |
|---|---|---|
| Movement while standing still | blocks vs background, after 1 s of learning | movers at once, no learning |
| Movement while turning (ROBOT_PLAN §6.4, §7.2) | not done | movers from leftover moves after `ego` removes the turn |
| Angular speed of a target | the tracker, from bearings over ~0.8 s | each observation carries one, with its sigma |
| Exposure changes | hold, adjust only when calm, 1-3 s blind | free: ZNCC ignores gain and offset |
| Heading cross-check (§12 Later) | none | a visual-yaw reader using `ego_fit()`; its scale is the camera's own (f from a closed 360° loop, not from the gyro, §7.3) |
| Landmarks, edges, odometry (later) | none | readers of the same records and images |

Numbers that set the design (cross-plan §1, ROBOT_PLAN §3, `camera.c`):

| | Value |
|---|---|
| Image | 160 × 120, 8 bit, f ≈ 160 px (53.1° across, VFOV 41.1°): 1° ≈ 2.8 px in the middle, 3.5 px at the sides |
| Mounting | 8.5 cm up, 2.5 cm ahead of the centre, level |
| Frame rate | 25 frames/s at 40 ms exposure; the frame length follows the exposure (more frames at 10-20 ms) |
| Turning | 1 rad/s (looking), up to 1.5 rad/s: 160-240 px/s, **6.4-9.6 px per frame at 25 frames/s** |
| Motion blur | ω × f × exposure: 6.4 px at 1 rad/s and 40 ms, 1.6 px at 10 ms |
| Rolling shutter | row r at `row0_us + r · line_us`; 120 rows ≈ 5 ms (host test 42.7 µs per row; `c` prints the measured one): 0.8 px of skew at 1 rad/s |
| Camera off the turning axis | 2.5 cm: ≤ 0.5 px per frame of parallax at 0.3 m and 1 rad/s, ignored |
| **What it sees of a walker** | the top row sees up to ~46 cm at 1 m, ~85 cm at 2 m: **legs**. Each leg stands still ~60 % of the time and swings at ~2× walking speed; trousers have little texture and fold |
| A walker at 1 m/s, 1 m away | body ~1 rad/s, ~6 px per frame; the swing leg ~12 px per frame, the stance leg 0. At 3 m, 0.4 m/s: ~0.8 px per frame |
| CPU | one M33 core at 150 MHz: **6 M cycles per frame at 25 frames/s** |

## 3. How others do it (research summary)

| Approach | Where used | What it gives | Cost at 160 × 120 | Fit here |
|---|---|---|---|---|
| **Block matching, SAD** on a grid of 8 × 8 tiles, histogram of shifts | PX4FLOW (STM32F4, ~250-400 Hz on a 64 × 64 crop; `usada8` SIMD, 5 × 5 tiles, gradient check per tile, half-pixel refinement, gyro subtracted afterwards) | one global flow per frame, plus a quality | ~2 k cycles per tile with SIMD | good idea for the global estimate; no feature identities, SAD not robust to gain |
| **Optical mouse chip** (PMW3901, Bitcraze Flow deck) | Crazyflie, PX4 | one global flow, ~121 frames/s, ≥ 8 cm | extra hardware | global only: can't find movers |
| **Phase correlation** (FFT) | OpenMV `find_displacement` | one global shift + response; power-of-two crops; users report noisy sub-pixel | 3 FFTs of 128 × 128: ~20-30 ms on an M33 (my estimate) | global only, slow |
| **Edge histograms** (EdgeFlow, Edge-FS) | TU Delft pocket drone, STM32F4 168 MHz, 128 × 96, 20-30 Hz; yaw flow from the gyro per column | 1-D flow along x and y, cheap | ~1 ms | ego speed, not movers or features |
| **Insect EMD** (Reichardt) | analog VLSI, FPGA, blimps | local motion signals | cheap per pixel | speed- and contrast-dependent, no identities; hard to tune and explain |
| **FAST + KLT** (pyramidal Lucas-Kanade) | the standard feature tracker; IMU-aided KLT (Hwangbo, CMU) predicts the shift from the gyro | sparse features with ids, sub-pixel | FAST < 7 % of a 2006 CPU on PAL video; KLT ~10 k cycles per feature (my estimate) | the family chosen |
| **Census transform** matching | Stein (DaimlerChrysler), real-time on PCs | illumination-invariant matching | cheap, but the M33 has no popcount instruction | fallback for ZNCC |
| **ORB / BRIEF** descriptors | SLAM | features found again after large motion or time | ~1 k cycles per feature | not needed frame to frame (the gyro hints); later for landmarks |
| **Global model + RANSAC, leftovers = movers** | moving-camera detection (homography from sparse flow, 50 RANSAC iterations; REDBEE adds the IMU because RANSAC fails with many outliers) | movers from a moving camera | small for ~100 features | the method chosen (in `ego`), with the gyro as the model |
| **Gyro rolling-shutter correction** | Karpenko et al., phone video | per-row rotation from the gyro | per feature: one interpolation | used: per-feature row time |

What the MAV and phone work agrees on: **use the gyro to predict, the image to
correct**. Gyro-only prediction (PX4FLOW's compensation, EdgeFlow's per-column yaw
flow) is enough when the motion is mostly rotation, as here. Lerman et al. found
that a gyro-predicted start gives KLT little over a careful image-only start on
handheld video; here the gain is different: it keeps the search small (cheap) and
gives the ego-motion fit a prior that someone filling the view can't overrule.
Note PX4FLOW itself reports the measured flow and subtracts the gyro afterwards,
which is the split this plan makes (`vision` measures, `ego` removes).

## 4. The choice, from first principles

1. **Sparse features, not dense blocks.** A mover is only visible where there is
   texture; blocks on a blank wall carry noise and lights, not information.
   Features have identities, which every later use (landmarks, odometry, gyro
   check) needs.
2. **FAST-9 to find candidate corners, Shi-Tomasi to score them, one or two
   features per cell.** FAST is the cheapest corner test on a scalar CPU (most
   pixels are rejected by 4 comparisons). Shi-Tomasi's score (the smaller
   eigenvalue of the 2 × 2 gradient matrix over the patch) is exactly "can this
   patch be found again in both directions"; computed only at candidates it is
   cheap. A grid of 8 × 6 cells (20 × 20 px), at most 2 features each, spreads them
   over the image: the ego-motion fit and the regions need spread.
3. **ZNCC patch search, coarse to fine, centred on the search hint.** An 8 × 8
   template per feature, searched ±3 px on the half-size image (±6 px full size),
   then ±1 px at full size, then a parabola through the scores for the sub-pixel
   part. Zero-mean normalised correlation ignores gain and offset, so exposure
   steps don't break tracks and the exposure no longer needs holding. The score
   (0-1) and how much better the best match is than the second best are the match
   quality. An exhaustive search has no local minima and costs the same every
   frame. Pyramidal KLT is the accepted alternative (§12).
4. **The record holds measurements; ego-motion is removed by readers.** What the
   camera measured stays true whatever model is later used to explain it; a reader
   that doubts the gyro (calibration, visual yaw) needs the raw measured moves, and
   one that wants movers removes the turn itself with `ego`. The record also stays
   smaller and `vision` simpler.
5. **Ego-motion: the gyro predicts each feature, the image corrects** (in `ego`).
   For every feature: its direction at its row time in the previous frame, rotated
   by the pose change to its row time now, projected back: its **predicted move**.
   Exact for a rotating camera, so the sides of the image move 25 % more than the
   middle and the rolling shutter is included. Then one small correction (dx, dy,
   roll) for the whole image, fitted robustly and held close to the gyro. Measured
   minus predicted-and-corrected is the **leftover move**: the feature's own
   movement.
6. **Movers are features whose leftover move adds up**, per frame (fast) or over
   the last ~0.3 s of their track (slow); moving features in one place over ~0.3 s
   are one mover (legs move by turns, §9). Regions where features are lost add
   evidence for movers with little texture.

## 5. The universal record

One `vision_frame_t` per processed camera frame. Pixel coordinates are as the robot
sees the image: x to the right (column 0 on the left), y down (row 0 at the top).
Bearings: + = left, as everywhere else. Times are `stamp_t` (`common/stamp.h`,
`time_us_32()`, wrap-safe). Everything in it is measured by the camera or copied
from `pose`.

```c
#define VISION_MAX_FEATURES 96
#define VISION_REGION_COLS  8    // regions of 20 x 20 px
#define VISION_REGION_ROWS  6

enum { // vision_feature_t.flags
    FEATURE_NEW        = 1,  // found in this frame: no measured move yet
    FEATURE_MISSED     = 2,  // not found in this frame (blur, covered); x, y are its last
                             // measured place, no measured move; searched again (≤ 2 frames)
    FEATURE_LOST       = 4,  // given up in this frame; reported once, then the slot is free
    FEATURE_LEFT_IMAGE = 8,  // ...given up because it would be outside the image (with LOST)
};

typedef struct {             // ~36 B
    uint16_t id;          // unique while it lives (wraps after 65 535 births)
    uint16_t age;         // frames since it was found; 0 when NEW
    uint8_t flags;
    float x, y;           // where it was measured in this frame, px (sub-pixel)
    float dx, dy;         // measured move since the previous processed frame, px
                          // (0 when NEW or MISSED; after a miss: since its last measurement)
    float strength;       // Shi-Tomasi score, normalised by the patch's contrast
    float match;          // ZNCC of this frame's match, 0-1 (1 when NEW)
    stamp_t t_us;         // its row's time: row0_us + y * line_us
} vision_feature_t;

typedef struct {             // ~20 B
    uint8_t features, lost, born, missed;
    float texture;        // mean gradient magnitude, grey levels per px: low = nothing to find here
    float brightness;     // mean, 0-255
    float dx, dy;         // median measured move of its features, px
} vision_region_t;

typedef struct {             // the robot at the frame's rows, copied from pose
    bool ok;                 // false: no odometry (pose_at failed)
    stamp_t row0_us; float line_us;
    pose_t first_row, last_row; // pose_at(row 0's time), pose_at(row 119's time):
                                // yaw, pitch, roll, w_radps, t_us (pose_t after A2)
} vision_pose_t;

typedef struct {
    uint32_t seq;            // counts published frames (readers detect gaps)
    uint32_t camera_number;  // the driver's frame number
    stamp_t row0_us;         // middle of row 0's exposure (cross-plan §1)
    float line_us;           // row r at row0_us + r * line_us
    stamp_t t_us;            // the middle row's time: row0_us + 59.5 * line_us
    float dt_s;              // since the previous processed frame
    float exposure_us, gain;
    float brightness;        // mean over unsaturated samples, 0-255
    float saturated;         // share of samples >= 250
    vision_pose_t pose;      // the robot during this frame's rows
    vision_pose_t prev_pose; // ...and during the previous processed frame's rows,
                             // so the record alone is enough to predict a move
    uint8_t n_features, n_found; // in the array; of which measured in this frame
    vision_feature_t features[VISION_MAX_FEATURES]; // live ones, NEW, MISSED and LOST included
    vision_region_t regions[VISION_REGION_ROWS][VISION_REGION_COLS];
} vision_frame_t; // ~4.6 KB (features 3.5 KB, regions 1 KB, header ~0.2 KB)
```

The interface (`vision.h`):

```c
bool vision_start(void);  // core 1: camera_init(), then frames forever (main starts it on core 1)
// Each reader keeps its own place: frames come in order, every one, until the reader
// falls behind by more than VISION_RING (4 frames, 0.16 s at 25/s): then it gets the
// oldest still there, and the gap in seq says how many it missed. Copies (§10).
typedef struct { uint32_t next_seq; } vision_reader_t;
bool vision_read(vision_reader_t *reader, vision_frame_t *frame);
void vision_status(vision_status_t *s); // frames/s, ms per frame (mean, max), skipped, for `k` / `p`
```

Geometry is `view`'s (cross-plan §2): `view_pixel_dir(x, y, dir)` (pixel → unit
direction in the robot frame, distortion included) and `view_project(dir, &x, &y)`.
`view` holds f, cx, cy, k₁ and the camera time offset; `replay --calibrate`
(§7.3) produces them.

Design notes:
- **Readers never touch `vision`'s insides**; they get a copy (~4.6 KB, ~6 k
  cycles; each reader on core 0 owns one, counted in §8). Adding a reader is one
  `vision_reader_t`, one copy and one `vision_read()` loop.
- **What is not in it, and why:** predicted moves, leftover moves, the ego-motion
  fit, which features fit it, "the light changed" and the expected blur are
  interpretations; each reader computes what it needs with `ego` (§7) or one line
  of its own. The record says what was seen and what the robot did.
- The pose of both frames is in the record so that `ego_predict()` needs nothing
  else, also on recordings.
- No descriptors yet. Landmarks (later) add an optional 32-byte BRIEF per
  long-lived feature: a field added, not a change for existing readers.

## 6. The extractor (`vision.c`) in plain sentences

Each frame (on core 1):
1. Take the newest image (`camera_next()`); make the half-size image (2 × 2
   averages, 80 × 60); each region's texture and brightness; copy the robot's pose
   at row 0 and row 119 from the attitude history (§10).
2. For every live feature: search its 8 × 8 template ±3 px around the **search
   hint** (§6.2) on the half-size image, then ±1 px at full size, sub-pixel by a
   parabola. Accept the match if its ZNCC ≥ 0.8 and the best is clearly better than
   the second-best peak (≥ 0.05; repeated patterns fail this): the feature's new
   position, and its **measured move** = new position − last measured position.
   Not accepted: MISSED (searched again next frame, at most 2 frames in a row), then
   LOST. A hint outside the image: LOST, LEFT_IMAGE.
3. Refresh each found feature's template from this frame (frame to frame, so slow
   changes of view and light are followed).
4. Each region's median measured move and counts.
5. In each cell with fewer than 2 features: FAST-9 candidate corners (threshold
   adapted per cell so a few come up, never below 3 × the noise at this gain),
   Shi-Tomasi score on a 7 × 7 window, keep the best that are ≥ 6 px from existing
   features as NEW features; never more than 96 in all, never within 4 px of the
   image edge.
6. Publish the record. Images that arrive while one is processed are skipped (the
   newest is taken next); `dt_s` and the row times keep it right.

### 6.1 Brightness, exposure and noise

- ZNCC makes a match independent of gain and offset; exposure steps change nothing
  for the tracks. So **the exposure is never held**.
- The record carries the frame's brightness (over unsaturated samples), the share
  saturated, the exposure and the gain. Whether the **light changed** is the
  readers' call (camera_movers, §9): with B the brightness and E = exposure × gain,
  `light = (B_now / B_prev) / (E_now / E_prev)`; changed when `light > 1.2` or
  `< 1 / 1.2`. The driver's own ×1.25 step gives light ≈ 1.
- FAST's threshold is relative (per-cell adaptive, floor at 3 × noise), so a dark
  room gives fewer, not noisier, features. **The pixel noise per gain is measured**
  (M-S recordings: still scenes at the evening exposure, and at the watching cap,
  gain up to ~24) and kept as a small table in `vision`: it sets the FAST floor and
  the ZNCC acceptance (noisier patches correlate lower).
- Flicker: the driver's 10 ms exposure steps keep 100 Hz banding out.

### 6.2 The search hint (gyro inside `vision`, not in the record)

Where to search for a feature: its last measured position moved by
`ego_predict()` (§7.1) for the pose change between its row times, i.e. where the
robot's turn alone would have taken it. Without odometry (`pose.ok` false), the
hint is the last position.

- **The hint only decides where the search window sits.** The reported position
  and measured move come from the match alone; with the hint off, a feature found
  in the window gives exactly the same numbers (V1 test 10 checks it).
- Why it is needed: at 1 rad/s a feature moves 6-10 px per frame; without the hint
  the window would need ±12 px (4× the positions, 4× the cost) and would find the
  wrong place more often.
- The window (±6 px) is wide enough for the hint's errors (gyro scale, time offset,
  pitch and roll at the ramps, ~1-2 px) and a mover's own move of up to ~6 px per
  frame; a faster mover (a leg swinging close by) is MISSED or LOST, which is
  itself evidence (§9).

### 6.3 Motion blur and the exposure cap

At 1 rad/s and 40 ms exposure a point smears 6.4 px across. Both frames of a pair
are smeared alike while the turn rate is steady, so ZNCC still matches; at the
start and end of a turn they are not. Measures:
- a feature not found in a blurred frame is MISSED, not lost, for up to 2 frames;
- readers scale their thresholds with the blur they compute from the record
  (|ω| × f × exposure);
- **an exposure cap while watching**, not while turning: the driver's steps of
  ≤ ×1.25 after 0.5-5 s waits can't reach 10 ms within a 0.5-1 s turn (from 40 ms ×
  5.9 in the evening that is ~6 steps). So behaviour sets the cap when WATCH starts
  (`camera_max_exposure(us, max_gain)`), long before any turn, and the driver
  moves to it in **one jump**: exposure down to the cap, gain raised only up to
  the gain cap (ZNCC absorbs the step). Outside WATCH (the scan, IDLE) the caps
  are off. **Decided (Daniel, 8 Oct): sharp and dark rather than noisy**:
  exposure ≤ **10 ms** (blur 1.6 px at 1 rad/s) and gain ≤ **8** (about today's
  evening gain). In the evening the image is then ~3× darker than the brightness
  target (mean ~30-40 instead of ~100); that is accepted. FAST and ZNCC
  thresholds follow the measured noise at the gain in use (§6.1), so a dark image
  gives fewer, not false, features. V2 counts the features per region in evening
  light at these caps; the gain cap may go up a step or two, the exposure cap
  stays.

### 6.4 Low texture

A region with low `texture` has no features; the record says so, and readers know
the camera is blind there (a white wall, the dark under a sofa). A person in front
of a blank wall still has texture of their own (shoes, trouser folds, the edges of
the legs against the wall). A mover with little texture in front of texture (a
plain door swinging) shows as `lost` and `missed` features in its regions.

## 7. Ego-motion helper (`ego`): predicted and leftover moves

Pure functions on a `vision_frame_t` and `view`'s constants; no state, no
hardware; used by `vision` (the hint only), `camera_movers`, the calibration and the
later visual-yaw reader. Host-tested on their own.

```c
// Where a world-fixed point seen at (x, y) in the previous frame is now (pose of both
// frames from the record, each at its own row time): the predicted move.
bool ego_predict(const vision_frame_t *f, float x_prev, float y_prev, float *x_now, float *y_now);
typedef struct {
    float dx, dy, roll_rad;  // what the image adds to the gyro's prediction
    float spread_px;         // median distance of fitting features from the fit: the jitter
    uint8_t n_fit, n_used;   // features that fit, of those measured
    bool gyro_only;          // too few features (< 8), or the fit was refused
    uint8_t fits[VISION_MAX_FEATURES / 8]; // bit per feature: fits the ego-motion
} ego_fit_t;
void ego_fit(const vision_frame_t *f, ego_fit_t *fit);
// A feature's leftover move: measured move - (predicted move + the fit's correction).
void ego_leftover(const vision_frame_t *f, const ego_fit_t *fit, int feature, float *lx, float *ly);
float ego_visual_yaw(const vision_frame_t *f, const ego_fit_t *fit); // the turn the image saw, rad
```

### 7.1 Predicted move (derotation)

A feature's previous position (x − dx, y − dy) looks along d = `view_pixel_dir()`
in the robot frame (x forward, y left, z up). The pose at its row time in the
previous frame and in this one (interpolated between the record's first and last
row, ~5 ms apart) gives the rotation R between them. A point fixed in the world now
looks along Rᵀd; `view_project()` gives its pixel. ~60 float operations per feature.

- Turning left moves the scene to the right (x grows): ≈ f × Δyaw in the middle,
  25 % more at the left and right edges (that is why it is not one shift).
- The rolling shutter is in it for free: each feature uses its own row's time, so
  at 1 rad/s the bottom row is predicted 0.8 px further than the top.
- **Pitch and roll** come from PicoB's complementary filter with the accelerometer
  (`odometry.c`), which acceleration at the start and end of turns disturbs (the
  −3…−5° roll in turns may be partly that). So yaw comes from the gyro, pitch and
  roll changes are only a starting guess, and the fit's `dy` and `roll` correct them
  (§7.2). V2 prints the fit's roll next to odometry's to find out which is right;
  if odometry's roll is the artefact, `ego_predict()` uses yaw only.
- The camera's 2.5 cm offset and the robot's sliding in turns (~1.5 cm) are not
  modelled: ≤ 0.5 px per frame at 0.3 m, under the mover threshold.
- Forward driving (later) makes depth-dependent moves; `ego` covers turning in
  place only, and readers that need movers stay quiet while |v| > 2 cm/s.

### 7.2 The ego-motion fit (what the image adds to the gyro)

1. For every feature measured in this frame: r = measured move − predicted move.
2. Start: (dx, dy) = the median of r, roll 0. Fitting features: within max(1 px,
   3 × the spread, 0.3 × blur) of the model. Least squares on them for (dx, dy, roll
   about the image centre); again; done (2 rounds).
3. **Held close to the gyro:** the correction may be at most 1 px + 5 % of the
   predicted move (gyro scale error, the time offset, pitching on the tyres). A
   larger one means most features moved on their own (someone close filling the
   view): the fit is refused, the gyro alone explains the ego-motion (`gyro_only`),
   and the features that disagree keep their leftover moves. Also gyro only with
   fewer than 8 measured features (a blank wall).

Deterministic (no RANSAC sampling): the gyro prior makes the model nearly known,
medians tolerate up to half the features moving, and the host tests give the same
answer every run. ~50 k cycles per frame for ~100 features (prediction 30 k, fit
20 k), in whichever reader calls it.

### 7.3 Calibration, and visual yaw that doesn't lean on the gyro

If f were fitted to the gyro, visual yaw would agree with the gyro by construction
(a 0.5 % error in f hides the gyro's 0.42 % scale error). So, in `replay
--calibrate` on recordings, using the record's measured moves:
- **f, cx and k₁ from the camera alone**: (a) within each frame pair all fitting
  features share one rotation; features at the edges and in the middle must give
  the same angle, which fixes k₁ and cx; (b) over a full turn (the scan's 390°, or
  `r`), the frame where the start's image is found again (best ZNCC of the whole
  half-size start image, kept in RAM during the scan and matched on the robot; no images leave it) is exactly 360° on, plus its
  small measured shift; the summed visual yaw from start to there must be 2π, which
  fixes f. Cross-check: the HM0360's pixel pitch × the Sub4 subsampling against the
  lens's 2.59 mm (ROBOT_PLAN §3), and whether Sub4 covers the whole field of view.
- **The gyro only for the time offset** between the camera's row times and
  odometry (± a few ms): the offset that makes visual and gyro yaw rates line up at
  the start and end of turns.
- `ego_visual_yaw()` = the yaw change that best explains the fitting features under
  `view`'s constants. It is then an independent measurement: bias while still
  (≈ 0), scale over turns. The reader that uses it comes later (§13).

## 8. Compute and memory

Estimates from operation counts (C, data and hot loops in SRAM via
`__not_in_flash_func`, not the XIP flash cache that core 0's code shares). Without
SIMD an M33 needs 2 byte loads and 1 multiply-accumulate per product: ~3 cycles,
so ~200-250 cycles per ZNCC position of 64 products.

**`vision`, core 1:**

| Step | Per | Cycles | Per frame |
|---|---|---|---|
| Half-size image | output px (4 800) | ~4 | 20 k |
| Region texture and brightness (half-size image) | px (4 800) | ~6 | 30 k |
| Search hint (`ego_predict`) | feature | ~300 | 30 k |
| ZNCC: 49 positions coarse, 9 fine, 64 products each; sums from a small local integral image | feature | ~13-15 k (200-250 per position) | 1.3-1.5 M for 96 |
| FAST-9 in cells with room (typically 10-30 % of the image, all of it after a light switch) | px | ~20-30 | 0.1-0.6 M |
| Shi-Tomasi score, 7 × 7 | candidate (≤ 200) | ~500 | 0.1 M |
| Templates (copy 8 × 8, two scales) | feature | ~150 | 15 k |
| Region medians, publish (copy into the ring) | — | — | 15 k |
| **Total** | | | **~2.5-3.5 M ≈ 17-23 ms** |

That fits core 1 at 25 frames/s (40 ms); at short exposures it takes every 2nd
image or so. **V1's pass line: ≤ 25 ms per frame with 96 features**; if not, 64
features, or a ±2 px coarse window (25 positions instead of 49). With the M33's DSP
instructions (SMLAD: 2 products per cycle on unpacked bytes; the RP2350 product
page lists "DSP instructions", and the Armv8-M DSP extension includes SMLAD and
USADA8, which PX4FLOW uses on the M4) the ZNCC part roughly halves; to confirm in
the RP2350 datasheet's Cortex-M33 section before V1 relies on it. Census matching
is the fallback.

**`camera_movers`, core 0:** `ego_predict` + `ego_fit` ~50 k, leftover moves and
grouping ~20 k, the copy ~6 k: **~80 k cycles ≈ 0.5 ms per frame**, within core 0's
loop budget (typical < 2 ms, max < 20 ms).

| Memory (static, KB) | |
|---|---|
| Half-size images, this and the previous | 9.6 |
| Features: state + 8 × 8 templates at two scales | 19 |
| Ring of **4** records (4 × 4.6) | 18.4 |
| One reader copy on core 0 (`camera_movers`; +4.6 per later reader) | 4.6 |
| `camera_movers`: leftover history (96 features × 8 frames × 8 B), groups | 6.5 |
| Attitude history on core 1 | 1.5 |
| **In all** | **~60** |
| Freed by V4: `camera_motion` (learning 24, change grid ≈ 6) | −30 |
| Net after V4 | +30 |
| Not counted here: the two 8 KB core stacks (cross-plan §5, A8/V3), the driver's three buffers (as today) | |

Cross-plan §5: static RAM is 385 KB today; without the map rework's −135 KB (S3),
the earlier milestones plus V1 leave ~12 KB for the heap. **So V1 starts after S3**,
and every V milestone's robot test checks `p`: **RAM free ≥ 40 KB**. A ring of 4 is
enough because core 0 reads every loop and, after A3, its loop never stalls.

## 9. Movement detection: the `camera_movers` reader

In plain sentences (on core 0, replacing `camera_motion.c`):
1. Each record, as `vision_read()` gives it; `ego_fit()` on it, and each measured
   feature's **leftover move** (`ego_leftover()`). Records where the light changed
   (§6.1), or while the robot drives forward (|v| > 2 cm/s), say nothing. Standing
   still and turning in place are the same to it.
2. A feature is **moving** when its leftover move is clearly more than the jitter:
   this frame's > max(0.8 px, 3 × the fit's spread, 0.25 × blur) in 2 of its last 3
   frames (fast: a hand at 60°/s, a swinging leg), or its leftover moves summed over
   the last ~0.3 s > 2.5 px (slow: someone at 3 m). Features younger than 3 frames
   don't count.
3. **Grouping by place over ~0.3 s:** each moving feature marks the bearing span it
   covered in the last ~0.3 s; spans that overlap or lie within ~8° (25 px) of each
   other are one **mover**. Not by agreeing leftover moves: a walker's legs move by
   turns (one still, one at twice the walking speed), and grouping by move would
   split them or drop the standing leg. A mover needs 2 moving features, or 1 plus
   features MISSED or LOST in its region (a leg too fast or plain to be found).
4. **Its rate is the group's, not a leg's:** the slope of a straight line through
   the group's mean bearing in the map frame (robot bearing + yaw at the feature's
   row time) over the last ~0.3-0.5 s; the swing and stance legs average out to the
   body's speed. `rate_sigma_radps` from that fit's scatter. The tolerances in the
   tests come from the M-S walk recordings (V4).
5. Each mover is a `motion_obs_t` (§9.1): direction at its mean row time (mean of
   its features' rays, robot frame), leftmost and rightmost bearing, no range,
   `cells` = its features, `strength` = its share of measured features, the rate, a
   confidence.
6. A region where several features were LOST at once and nothing moving was found
   there (an untextured mover covering texture) is a weak observation (confidence
   ≤ 0.3), so `movement` can wait for the VL53.
7. No learning, no background: movement is seen from the first frame after a stop
   and during turns, and things that stop moving stop being movers at once.

Unlike the old detector, a box put down is not movement once it rests (it was
movement only while carried), a light switched is not movement, and a soft shadow
has few corners, so few features and far fewer movers (V4 measures it). A
hard-edged shadow or a reflection on the waxed floor moving is still movement: the
VL53 tells (ROBOT_PLAN §6.5).

### 9.1 The observation (the shared `motion_obs_t`)

Both detectors fill the same struct (cross-plan §2), **while still and while
turning** (the ToF plan's rotation-aware background, T-R):

```c
typedef struct {
    stamp_t t_us;
    uint8_t sensor;                  // OBS_TOF, OBS_CAMERA
    float where_robot[3];            // unit, robot frame AT t_us
    float left_rad, right_rad;       // robot frame, + = left
    bool has_range;  float range_m, range_sigma_m; float point_robot[3];
    bool rate_known; float rate_radps, rate_sigma_radps; // WORLD frame (ego-motion removed), + = left
    bool range_rate_known; float range_rate_mps;         // + = going away
    float strength; uint16_t cells;
    float confidence;                // 0-1
} motion_obs_t;
```

The camera sets `has_range` false, `range_rate_known` false, `rate_known` true once
the mover has ≥ 3 frames. Confidence: 1 − 0.5^features, scaled down by low
`match`, young tracks, a `gyro_only` fit while turning, and blur.

**Only `movement` converts to the map frame**, with `pose_at(t_us)` for each
observation's own time. Its pairing rule (ROBOT_PLAN §6.5: dot > cos 10°) compares
**map-frame** bearings, each at its own `t_us`: a camera and a ToF observation
50 ms apart at 1 rad/s differ by 2.9° in robot bearings. A pair is one target with
the ToF's range and the camera's bearing and rate (the smaller sigma wins).

### 9.2 How it plugs in

- `camera_movers` hands its observations to `movement` (ARCHITECTURE_PLAN A6),
  which also gets the ToF's, still and turning alike; the old "still" gating in
  `motion_sense` goes for both sensors.
- Log lines in today's format, plus the rate: "Movement (camera): N features,
  +X deg (+ = left), going right at R deg/s".
- The tracker gets the rate at once instead of after ~0.8 s of bearings, and sees
  out to the camera's ±26.5° beyond the VL53's ±22.5°. With both sensors working
  while turning, a target leaving during the turn or right after the stop is seen:
  the gap CHANGELOG 7 Oct blames for the misses.

## 10. Latency and cores

| Step | Time |
|---|---|
| Light to the middle of the exposure | exposure / 2: 5-20 ms |
| Exposure end to the bottom row read | ~5 ms |
| Waiting for core 1 (busy with the previous image) | 0-23 ms |
| `vision` | 17-23 ms |
| Core 0 reads it (main loop, < 20 ms after A3), `camera_movers` | < 20 ms + 0.5 ms |
| Mover confirmed (2 of 3 frames) | + 1 frame: 10-40 ms |
| **Something moves → `motion_obs_t`** | **~60-130 ms** (the VL53: 2 frames of 66 ms + its own processing) |

| Core 0 | Core 1 |
|---|---|
| link, `body`, `pose`, ToF and the map, WiFi, console, recording, `camera_movers` (+ `ego`), `movement`, tracker, behaviour | camera driver (its DMA_IRQ_1 handler and exposure control on I2C1; nothing else uses I2C1), `vision` (+ `ego_predict` for the hint). No printf: status through snapshots, data through records |

- **Why core 1: CPU.** One vision step of 17-23 ms alone breaks core 0's loop
  budget (typical < 2 ms, max < 20 ms, ARCHITECTURE_PLAN). V1 runs on core 0 only to
  measure it, at a reduced rate.
- The camera driver is started on core 1, so its interrupt runs there and
  `camera_frame()`'s interrupt lock is right (it masks its own core only). The
  driver shares DMA_IRQ_1 (`camera.c` `DMA_IRQ_INDEX`, `irq_add_shared_handler`):
  **check that cyw43 on core 0 never enables DMA_IRQ_1 on core 0** (its driver
  waits on its DMA without interrupts today; V3 confirms it in code and on the robot).
- Core 0 → core 1: each ODOM's pose into an SDK `queue_t`; core 1 keeps its own
  1.3 s history and interpolates with `pose_interp.c` (pure, shared with `pose`) to
  fill the record's `vision_pose_t`. A row newer than the latest report is
  extrapolated with the last ω (≤ 20 ms: at 3 rad/s² that is ≤ 0.1 px). Core 1 →
  core 0: the record ring. No other shared state.
- **The ring's protocol** (a sequence lock per slot): the writer makes the slot's
  sequence odd, copies the record in, then makes it even (the record's `seq` × 2),
  with a memory barrier (`__dmb()`) after each step; a reader reads the sequence,
  copies the record out, reads the sequence again, and keeps the copy only if both
  were the same even number, else it retries (or moves on if the slot was
  overwritten by a newer record). Never blocking, no lock held during a 4.6 KB copy.

## 11. Recording and replay (the shared R0)

There is one recording path for all plans (cross-plan §3, milestone R0, right after
A3): records `[type u8][len u16][payload]`, COBS + CRC, on a second TCP connection
(port 4212), key **`R`**; USB fallback on the CDC port. What the camera work needs
from it.

**Decided (Daniel, 8 Oct): no camera images go to the PC for now**: no `IMAGE`
records, no `--pgm`, no drawing of features. Vision is tested on the host with
synthetic (rendered) frames and on the robot with the numbers it prints; image
recording can be added later if those aren't enough. What this means for the
milestones: a camera "recording" is the `VISION` records (computed on the robot by
`vision`) plus `ODOM`; they exist only once V1 runs on the robot, so the camera's
M-S recordings listed below are made in V1/V2's robot tests instead of in M-S, and
the M-S and recording tests in §13 that say "recording" mean these records. So the
camera needs:
- `ODOM` at 50 Hz (yaw, pitch, roll, ω, v, stationary, PicoA time), `MARK` (keys,
  log lines), later `VISION` (the record itself: 4.6 KB, a fraction of an image)
  and `OBS`.
- The host tool `build/replay file.rec`: `--vision` runs `vision`, `ego` and
  `camera_movers` (the same C files; the attitude history fed from the ODOM
  records, `view` with the calibrated constants) on recorded `VISION` records (no
  images: `ego` and `camera_movers` run from the record alone) and prints the same
  log lines; `--csv` writes every feature for plots; `--calibrate` (§7.3).
- Recordings live in a git-ignored folder (`recordings/`), not in git (Daniel).
- The camera's recordings are made in the **joint measurement session (M-S)**:
  quiet room, walks at 1 and 2 m both ways (one in dark trousers), a hand wave,
  a box put down and taken away, lights off/on, the evening lamp and shadows, `r`
  turns, the `n` scan (for the closed loop, §7.3), slow and 1 rad/s turns past a
  door edge, and still scenes at the watching cap's gain (noise, §6.1).

## 12. Rejected alternatives

| Alternative | Why not |
|---|---|
| Keep the 8 × 8 blocks vs a learned background | Only while still; 1 s blind after every stop; exposure held and blind to change; lamps and shadows; nothing reusable. The reason for this plan |
| Ego-motion results in the record (predicted move, leftover move, the fit, a "fits" flag) | Interpretations, not measurements: a reader that checks the gyro or calibrates needs the raw measured moves, and a model change would change the record. In `ego`, called by readers (Daniel's decision) |
| Dense block matching on a fixed grid (PX4FLOW) | Flow per block whether textured or not; SAD breaks on exposure steps; no identities. Its median/histogram global estimate is kept (§7.2) |
| Phase correlation (OpenMV) | One global shift only; FFT ~20-30 ms per frame; power-of-two crops; noisy sub-pixel in practice |
| EdgeFlow / edge histograms | Excellent for ego speed at ~1 ms, but 1-D and global: no movers, no features |
| Insect EMDs | Response depends on speed and contrast, no identities, hard to put in plain sentences |
| Dense optical flow (Horn-Schunck, Farnebäck) | 10-100× the cost for flow on blank walls nobody needs |
| Harris / Shi-Tomasi over every pixel | ~3× FAST's cost for the same corners; used only at FAST's candidates |
| Pyramidal KLT as the tracker | A good alternative at similar cost. Not first: iterative (cost varies, can converge wrong without a good start), needs a photometric model for gain changes, and gives no "second best" for ambiguity. One LK step can refine the ZNCC result later if visual odometry needs better than ~0.1-0.2 px |
| Census transform matching | Illumination-invariant and cheap, but gives Hamming distances (coarse sub-pixel) and the M33 has no popcount. Fallback if ZNCC is too slow |
| ORB/BRIEF descriptors and matching across the whole image | Needed to find features again after big moves; here the gyro hints each frame's move. Later, for landmarks |
| Homography / affine model with RANSAC | 8 or 6 parameters for a motion that is a known rotation; random sampling makes tests flaky; fails when movers dominate (REDBEE adds the IMU for that). The gyro-held 3-parameter fit does the job |
| Grouping movers by agreeing leftover moves | Splits a walker into two legs or drops the standing one (§9 step 3) |
| Calibrating f against the gyro | Makes visual yaw agree with the gyro by construction; f from a closed 360° loop instead (§7.3) |
| A short exposure requested at the start of each turn | The driver can't get there within a turn; the cap is set while watching (§6.3) |
| A flow chip (PMW3901) | More hardware, global flow only |
| Frame differencing after derotation as a cue in the record | An interpretation (needs the ego-motion), only edges of movers and every light and shadow; lost features per region do the job. A reader may do it from `camera_next()` images if ever needed |

## 13. Milestones

Where the V milestones sit in the merged order (cross-plan §7): A0-A3 →
**R0** (recording) → **M-S** (measurement session, includes the camera recordings
of §11) → ToF and map work, **S3** (−135 KB) → **V1** (needs R0, S3) → **V2** →
**V3** (= A8, needs S3, V2) → A5 → **V4** → T3 → **#19: `movement` + ToF events +
camera into the tracker** (A6 + T4 + V6, one milestone) → **V5** → S4. Algorithm
work on recordings (host only) may start any time after M-S. Each ends with host
tests in `./run_tests.sh`, a robot test in the README's style (every one prints
`p`: RAM free ≥ 40 KB), and a red-flag review. The old `camera_motion` keeps running
until V4 replaces it.

| | Milestone | Done when |
|---|---|---|
| V1 | `vision`: features found and tracked, robot still, core 0, no hint yet | features stable on a still scene, measured moves right for known shifts, ≤ 25 ms per frame with 96 features |
| V2 | `ego` (predicted move, fit, leftover move, visual yaw), the record's pose, the search hint, gyro-free calibration | turning scenes: leftover moves of still features < 0.5 px; visual yaw agrees with the gyro without having been fitted to it |
| V3 | `vision` on core 1 (= A8), the ring, the attitude queue | core 0's loop budget holds with vision running |
| V4 | `camera_movers` while still; the final `motion_obs_t` | README "Movement, camera" passes with fewer false movers than the old detector; old detector removed |
| V6 → #19 | Into `movement`, the tracker and behaviour, with the ToF plan | Watching misses fewer walk-pasts (10 × 2 distances × 2 ways, counted) |
| V5 | Movers while turning (logged and used) | no movers from turning in an empty room; a walker found during a turn with the right world rate |
| Later | Visual yaw as gyro check (bias still, scale over turns), landmarks for the full-turn check, edges | readers of `vision_frame_t` (with `ego`) and images; `vision` unchanged |

### V1: features found and tracked (still)

Build: `vision.c/.h` (§6 without the hint: the window centred on the last
position) on core 0, every 2nd-3rd image, from the main loop; `k` prints the vision
status (replaces the blocks print). Host-only work on M-S recordings can be done
before S3.

Host tests (`test_vision.c`, synthetic frames: a textured room image, pixel noise
at the measured σ for gain 6 and gain 24, the scene moved by bilinear resampling;
10 seeds):
1. **Known shifts:** the whole image moved by (dx, dy) of (0, 0), (0.25, 0),
   (0.5, −0.5), (1.3, 0.7), (3.7, −2.1), (6, 0) px per frame for 20 frames: every
   found feature's measured move within 0.15 px RMS of the truth (0.25 px at gain
   24), < 2 % off by more than 0.5 px, > 90 % still found after 20 frames (minus
   those that left the image). Each region's median measured move within 0.1 px.
2. **Exposure steps:** the same with gain ×1.25 and an offset of +10 at frame 10,
   ×0.8 at frame 15, and the one-jump cap (exposure ÷4, gain ×4, noise up) at
   frame 18: no feature lost because of it, the same accuracy; the record's
   exposure, gain and brightness follow.
3. **Brightness in the record:** a lamp (×1.2 brighter) in the same frame as a
   driver step (×1.25): the record's brightness rises ×1.5 and exposure × gain
   ×1.25 (so a reader's light ratio is 1.2); with 10 % of pixels saturated the
   brightness ignores them and `saturated` ≈ 0.10.
4. **Still and noisy:** 1 000 frames of the same scene: mean |dx, dy| < 0.05 px,
   σ < 0.1 px (gain 6); ≥ 80 % of features keep their id all the way; no id used twice.
5. **Nothing to see:** a flat grey image with noise at gain 24: fewer than 3
   features. A texture on the left half only: all features there, never more than 2
   per cell, never more than 96.
6. **Leaving the image:** 3 px per frame to the right: features reaching the right
   edge are LOST + LEFT_IMAGE once, then gone; NEW ones appear on the left.
7. **Repeated pattern:** vertical stripes every 6 px, moved 3 px: each feature
   either right within 0.5 px or MISSED/LOST (the second-best check), never
   confidently wrong in > 1 % of matches.
8. **Missed, then found:** one frame blurred heavily: features MISSED (no measured
   move, last position kept), found again in the next frame with the measured move
   since their last measurement; three bad frames in a row: LOST.
9. **The record's promises:** NEW has age 0 and no measured move; ages grow by 1;
   LOST is reported once; `seq` increases by 1; region counts add up to the
   features; `t_us` = `row0_us` + 59.5 × `line_us`; each feature's `t_us` =
   `row0_us` + y × `line_us`; the record has no field that depends on the pose
   other than `pose` and `prev_pose` (a compile-time list in the test).
10. **Recordings (M-S):** the still recording: 40-96 features, jitter (σ of dx)
    < 0.15 px, ≥ 70 % of ids alive after 10 s. The lights recording: tracks survive
    the driver's exposure steps (count drops < 20 % per step). The thresholds are
    set from these numbers once and then kept.

On the robot (only PicoA; motors off):
1. Flash `picoA_app`. Robot facing ~2 m of the room, lights on. `k`: "Vision:
   N frames/s, X ms per frame (max Y), F features (B new, L lost in the last
   second), jitter Z px", then the 8 × 6 regions as feature counts and texture.
   Expected: **X ≤ 25 ms with F near 96** (else the fallback of §8), F 40-96, L near
   0, Z < 0.15. `p`: RAM free ≥ 40 KB. Paste both.
2. Hold a hand in the left half, still: feature counts go up in the left regions.
   Move it slowly: `k` shows the left regions' median measured move ≠ 0, the others
   ~0.
3. Switch the light off/on, wait ~10 s, `k`: the exposure changed (as `c`), F back
   to roughly what it was, no period without features longer than ~1 s.
4. Cover the lens: F → 0, texture ~0 everywhere; uncover: features back within
   ~0.5 s.
5. `p` for core 0's loop time while vision runs on it at the reduced rate: paste it
   (it shows why V3 exists; nothing to pass here).

### V2: ego-motion in `ego`, the search hint, calibration

Build: `ego.c/.h` (§7), the record's `pose` / `prev_pose` (`pose_t` with roll and
`t_us`, from A2), the search hint in `vision` (§6.2), `view`'s camera constants;
`replay --calibrate` (§7.3); `k` prints `ego`'s view of the last frame (computed on
core 0 from the record, like any reader).

Host tests, `ego` alone (`test_ego.c`: records built by hand from a known
rotation, no images; exact numbers):
1. **Predicted move:** a pure yaw of 1° between two frames: a feature at x = 80
   moves +f × tan-geometry ≈ 2.79 px, at x = 5 ≈ 3.5 px (25 % more); pitch +1°: y
   grows ~2.8 px; the rolling shutter: same yaw rate, rows 0 and 119 differ by
   ω × 119 × `line_us` × f.
2. **Fit:** measured moves = predicted + (0.4, −0.2) px + noise 0.1 px, and 30 % of
   features with an extra 3 px: the fit returns (0.4, −0.2) ± 0.05, those 30 % don't
   fit and their leftover moves are 3 ± 0.1 px, the others' < 0.2 px.
3. **Refused fit:** 70 % of features with an extra 4 px while the robot is still:
   `gyro_only`, the 70 % keep 4 px leftover moves, the rest ~0.
4. **False roll from odometry:** odometry's −4° of roll that the camera never had,
   and the reverse: the fit's roll takes it out, leftover moves < 0.5 px.
5. **Few features:** 5 measured: `gyro_only`, no failure.

Host tests, `vision` + `ego` on rendered turns (a 360° textured panorama, the
camera rendered exactly for each row's yaw, so the rolling shutter is real; blur =
the average of 8 renders over the exposure; ODOM at 50 Hz with 2 ms of jitter and a
gyro 0.42 % low, as the robot's; 10 seeds):
6. **Steady turns** at 0.5, 1 and 1.5 rad/s, 10 ms exposure (noise at gain 24):
   leftover moves of features on the panorama RMS < 0.3 px; > 80 % of features found
   frame to frame (with the hint; < 20 % without it at 1 rad/s: shows the hint is
   wired); visual yaw per frame within 0.02° of the truth; summed over 360°
   within 0.1 % of the **truth** (so the gyro's 0.42 % shows as a difference).
7. **The hint changes nothing reported:** at 0.5 rad/s (≤ 3 px per frame, inside
   the window either way) the measured moves with and without the hint are equal to
   within 0.01 px.
8. **40 ms exposure** at 1 rad/s (6.4 px blur, gain 6 noise), and 20 ms (3.2 px,
   gain 12): leftover RMS < 0.6 px, > 50 % found; no feature MISSED longer than 2
   frames.
9. **Turn ramps** at 3 rad/s² from still to 1 rad/s and back: no more than 2 frames
   in a row with a `gyro_only` fit; leftover RMS < 1 px in the ramp frames.
10. **The rolling shutter matters:** `ego_predict` with the row times ignored (all
    rows at the middle): a top-to-bottom leftover slope of ~0.8 px appears; with
    them: < 0.1 px.
11. **Someone filling the view:** two textured legs covering 70 % of the image
    moving at 0.5 rad/s while the robot is still: the fit is refused, the legs'
    features get the leftover moves, the others stay at ~0.
12. **Gyro-free calibration:** a synthetic 390° scan rendered with f = 155,
    k₁ = −0.08, cx = 82, a camera time offset of 6 ms and a gyro 0.42 % low:
    `--calibrate` recovers f within 0.2 %, k₁ within 0.02, cx within 1 px from the
    images alone, and the offset within 1 ms; then the summed visual yaw over the
    closed loop differs from the gyro's by 0.42 % ± 0.15 %. (With f fitted to the
    gyro instead, the difference would be 0: the test shows the calibration doesn't
    lean on the gyro.)
13. **Recordings (M-S):** the `n` scan recording: calibrate, write the constants to
    `view`, replay the `r` recording: leftover RMS < 0.5 px in steady turning;
    per-turn visual yaw vs gyro (paste the table).

On the robot (both Picos, battery on):
1. `r` (10 turns at ~30°/s) with `v` on: each turn's line gets "camera saw
   +N deg" next to the gyro's 360. Expected: all 10 within ~0.5 % of each other
   (the camera is consistent); their mean differs from 360 by the gyro's scale error
   (M1 measured 0.42 % low: expect the camera ~1.5° more or less than the gyro per
   turn; paste all 10). Agreement is not by construction: f came from the scan's
   closed loop.
2. Standing still for a minute: `k` shows the visual yaw rate ~0 (< 0.05°/s) next
   to the gyro's; paste.
3. During `r`, `k` every few seconds: the fit's roll next to odometry's roll (the
   −3…−5° in turns). Paste: if the fit says ~0 while odometry says −4°, odometry's
   roll is the artefact, and `ego_predict` switches to yaw only (§7.1).
4. `a` (watching) and walk past so it turns at 1 rad/s: `k` after the turn shows
   "turn: gyro −44.1 deg, camera −43.8 deg, fit ok in N of M frames, features found
   F %". Expected: **N/M ≥ 90 %**, F ≥ 60 %; paste.
5. `p`: RAM free ≥ 40 KB.

### V3: core 1 (= A8)

Build: `vision_start()` on core 1 with the camera driver, the ring with its
sequence lock (§10), readers, the attitude queue, explicit 8 KB stacks; `k` and `c`
through core 1's snapshots.

Host tests:
1. `test_vision_ring.c` (single thread): readers in step get every record; a reader
   5 records behind gets the oldest kept and the right gap in `seq`; two readers
   don't affect each other; a slot overwritten during a read is retried, not
   returned half-copied (simulated by a writer step between the reader's two
   sequence reads).
2. The same with two POSIX threads (writer at 100 records/s, reader with random
   pauses up to 0.5 s), 100 000 records: every record read is whole (a checksum per
   record matches), `seq` only increases. (This checks the logic; memory ordering
   on the RP2350 is checked by reading the code against §10's two sentences.)
3. The attitude history on core 1 gives the same `vision_pose_t` as `pose_at()` for
   the same ODOM stream (both use `pose_interp.c`).

On the robot:
1. `p` with vision running on core 1 and the robot watching (`a`): core 0's loop
   time **typical < 2 ms, max < 20 ms** (ARCHITECTURE_PLAN's budget); core 1's time
   per frame as in V1; "skipped by vision" small and steady. RAM free ≥ 40 KB. Paste.
2. `n` (scan): `k` afterwards: core 1 time per frame unchanged, readers missed 0
   (core 0's loop no longer stalls after A3/S3).
3. `l` as in M0, with WiFi on and `R` recording: the link loses nothing new.
4. The DMA interrupt: `k` reports the camera interrupt's core (1) and its count
   equals frames captured; WiFi traffic for a minute (recording on) doesn't change
   frames/s. (The code check: cyw43 never enables DMA_IRQ_1 on core 0.)
5. `c` still prints the frame and exposure; `picoA_bringup` (driver on core 0)
   still builds and runs with the viewer.

### V4: movers while still

Build: `camera_movers.c` (§9, with `ego`) producing the shared `motion_obs_t`;
logged through `motion_sense` for now; for one session both detectors run and log
("Movement (camera)" and "Movement (camera, old)"); then `camera_motion.c` and its
test are removed.

Host tests (`test_camera_movers.c`; synthetic room as in V1, **two-leg walkers**:
each leg a textured 10 × 60 px sprite with folds, standing still ~60 % of the gait
cycle and swinging at ~2× the body's speed, legs out of phase; 20 seeds; at 25 and
50 frames/s):
1. **Quiet:** 5 minutes still, with exposure steps every 20 s and the cap jump
   once: no movers.
2. **A walker** crossing at 20, 40 and 60°/s (body): **one** mover (not two) within
   3 frames of entering, bearing within 3° of the body's, every frame until it
   leaves, the right sign of rate. Rate within a tolerance taken from the M-S walk
   recordings (expected ~±8°/s after 0.5 s; written in the test once measured).
3. **Slow:** a walker at 3 m crossing at 7°/s: found within 0.5 s.
4. **Small and fast:** a hand (12 × 12 px) at 60 and 100°/s (the old limit was
   ~50°/s): found in ≥ 2 of every 3 frames it is in view.
5. **A box put down:** movement while carried, none 2 frames after it rests; taken
   away: movement while carried, none from the empty place.
6. **Light switched** (×0.5 in one frame, new shading): `camera_movers` sees the
   light ratio, says nothing for that frame, no movers after. A lamp (×1.2) in the
   same frame as a driver step (×1.25): also "light changed"; the driver step alone:
   not.
7. **A soft shadow** (dark blurred ellipse, no texture) crossing the floor: movers
   in **≤ half** as many frames as the old detector on the same frames (both run in
   the test).
8. **Plain mover on texture:** an untextured grey rectangle crossing a textured
   wall: found through the features it covers being LOST, confidence ≤ 0.3.
9. **Two walkers** 20° apart going opposite ways: two observations with opposite
   rates. One walker whose legs are 6° apart: one.
10. **Recordings (M-S):** the walks: one mover per walker, the right direction,
    none in the still parts; the lights recording: no movers from the switch.

On the robot (PicoA with PicoB running; motors off; `v` on):
1. Stand behind the robot, still, a minute: no "Movement (camera)" lines (old
   detector lines may come: paste both).
2. Walk across at ~1 m, left to right: "Movement (camera): N features, +X deg
   (+ = left), going right at R deg/s" about twice a second, **one** mover per line
   (not two for two legs), X from positive to negative, R roughly 57 at 1 m/s; then
   "ended" within ~0.2 s of leaving the view.
3. The same at ~3 m, slowly: found.
4. A hand waved fast ~30 cm in front of the left half: positive degrees, both signs
   of R.
5. Box put down and stepped away: "ended" as your hand leaves (no 1 s wait); taken
   away: only your hand.
6. Room light off/on: **no movement lines**, no "learning the view again", `k`
   shows features back within ~1 s. Paste.
7. The evening lamp and your shadow on the floor: count the lines with "deg up" < −10
   (the floor) for both detectors in a 1-minute walk. **Passes when the new count is
   ≤ half the old.** Paste the counts.
8. Robot facing a white wall at ~50 cm, walk in front of it: found.
9. `p`: RAM free ≥ 40 KB. Paste the whole log with what you did.

### #19 (V6 with A6 + T4): into `movement`, the tracker and behaviour

Owned jointly with TOF_MOTION_PLAN.md and ARCHITECTURE_PLAN.md (`movement` owns
pairing and map-frame bearings). From this plan: camera observations with rate and
sigma, the camera's "leaving" at ±20°, both sensors working while turning.

Host tests: `test_movement.c` / `test_tracker.c` with camera observations (±1°
jitter, rate known): leaving reported from the first observation at the edge with
|rate| ≥ 10°/s; a ToF and a camera observation of the same walker 50 ms apart while
turning at 1 rad/s (2.9° apart in robot bearings) are paired (map bearings agree);
not paired when 15° apart in map bearings. `test_behaviour.c`: a walker leaving
during or right after a turn is followed on from the camera alone.

On the robot: the README's "Watching" steps; 10 walk-pasts at 1 m and 10 at 2 m,
half each way; count the ones that got a turn (before: CHANGELOG 7 Oct, "misses
some"). Paste the counts and the log.

### V5: movers while turning

Build: the movers' observations used during LOOKING turns (behaviour may correct
the turn's target or follow on).

Host tests (V2's panorama with V4's two-leg walkers):
1. Turning 1 and 1.5 rad/s in an empty scene, ramps included, 5 minutes in all, at
   the cap's exposure and at 40 ms: no movers.
2. Turning 1 rad/s while a walker crosses at 40°/s in the same direction and the
   opposite one: found within 3 frames, world rate within the V4 tolerance + 5°/s,
   map bearing within 3°. (Against the turn the leg's measured move reaches ~16 px
   per frame: it may be MISSED; the mover must still be found from the stance leg
   and the lost features.)
3. The robot stops: movers in the first frames after the stop (no gap).

On the robot (both Picos):
1. Empty room, `r`: no "Movement (camera)" lines during the 10 turns. Paste any.
2. `a`, walk past at ~2 m: the turn after you; during it, "Movement (camera)" lines
   show you (map bearings, your speed), and go on right after the stop. Paste.
3. Walk past and keep walking out of view during its turn: it turns on after you
   (count 5 tries).
4. `p`: RAM free ≥ 40 KB.

## 14. Risks and open questions

**Risks**
- **CPU** beyond 25 ms per frame (flash cache misses, ZNCC inner loop): V1's pass
  line; 64 features, ±2 px coarse window, DSP intrinsics, census.
- **RAM**: ~60 KB for vision and its reader; only after S3 (§8). Every robot test
  checks `p`.
- **`ego` computed twice** when several readers need it (~50 k cycles each on core
  0): cheap now; if many readers come, one reader can publish its `ego_fit_t` for
  the others (still outside the record).
- **The lens** has more distortion than one k₁, or Sub4 crops the field of view:
  V2's calibration shows it; add k₂ then.
- **Closed-loop calibration** fails in a room with little texture at the scan's
  start heading: use the best-textured overlap of the 390° instead; or the pixel
  pitch × lens figure as a fallback.
- **Gyro/camera timing**: V2's fitted time offset shows it (leftover moves at ramps).
- **Pitch and roll from odometry** wrong in turns: V2 robot step 3 decides.
- **A dark evening image** (10 ms, gain ≤ 8) leaves too few features: V2 counts
  them per region; the gain cap may go up a step or two, the exposure cap stays.
- **No recorded images**: tuning `vision` itself (thresholds, search) has only
  rendered frames and printed numbers; if that proves too blind, add `IMAGE`
  records later (R0's throughput test says what rate fits).
- **Legs with little texture** (dark plain trousers): fewer features per leg; lost
  features help, and the VL53 sees legs well; M-S has a walk in dark clothes.
- **Movers that dominate the view** (someone close): the gyro prior guards the fit
  (V2 tests 3 and 11).
- **Repetitive texture** (radiator, blinds): the second-best check (V1 test 7).
- **Cross-core bugs**: one ring with a sequence lock and one queue only; V3's tests
  and a code reading of §10.
- **Shadows and reflections** with hard edges stay movers: the VL53 decides.

**Answered (Daniel, 8 Oct; [REWORK_PLAN.md](REWORK_PLAN.md) "Decisions")**: exposure
≤ 10 ms and gain ≤ 8, dark rather than noisy (§6.3); recordings not in git; no
images to the PC and no drawing of features (§11).

**Open questions**
1. Keys: `k` reused for the vision status (`R` records, cross-plan)?

Settled: the record holds measurements only, ego-motion is removed by readers with
`ego` (Daniel); vision keeps running while driving forward, with the movers reader
quiet (review): it costs only core 1, and visual odometry later needs the data.

## 15. Sources

- PX4FLOW: [PX4 docs](https://docs.px4.io/v1.13/en/sensor/px4flow.html),
  [firmware, `flow.c`](https://github.com/PX4/PX4-Flow/blob/master/src/modules/flow/flow.c)
  (8 × 8 tiles, `usada8`, gradient check, half-pixel step, histogram, gyro
  compensation applied to the measured flow afterwards); Honegger, Meier,
  Tanskanen, Pollefeys, "An open source and open hardware embedded metric optical
  flow CMOS camera for indoor and outdoor applications", ICRA 2013, pp. 1736-1741.
- PMW3901 / Flow deck: [PX4 docs](https://docs.px4.io/main/en/sensor/pmw3901.html),
  [121 frames/s, 42° (Pimoroni)](https://shop.pimoroni.com/products/pmw3901-optical-flow-sensor-breakout).
- OpenMV phase correlation: [`Displacement`](https://docs.openmv.io/library/omv.image.Displacement.html),
  [tutorial](https://docs.openmv.io/openmvcam/tutorial/image/matching/displacement-and-keypoints.html),
  [forum: M4 support, noisy sub-pixel](https://forums.openmv.io/t/finding-translation-and-rotation-using-m4/905).
- EdgeFlow / Edge-FS, STM32F4: McGuire et al., [arXiv 1612.06702](https://arxiv.org/abs/1612.06702)
  (128 × 96, 20-30 Hz, yaw flow from the gyro per column; LK had difficulties at
  low resolution).
- FAST: Rosten, Drummond, ["Machine learning for high-speed corner detection"](https://ecse.monash.edu/staff/twd/Research/Rosten-ECCV06.pdf), ECCV 2006.
- KLT: Bouguet, ["Pyramidal implementation of the Lucas Kanade feature tracker"](https://www.cs.ucf.edu/courses/cap4453/bouguetopticalflow.pdf);
  Shi, Tomasi, "Good features to track", CVPR 1994.
- Gyro-aided tracking: Hwangbo, Kim, Kanade, [IMU-assisted KLT (CMU)](https://www.ri.cmu.edu/project/imu-assisted-klt-feature-tracker);
  Lerman et al., ["Enhancing feature tracking with gyro regularization"](https://arxiv.org/abs/1511.01508).
- Rolling shutter with a gyro: Karpenko et al., [Digital video stabilization and rolling shutter correction using gyroscopes](https://graphics.stanford.edu/papers/stabilization/).
- Census transform: Stein, [Efficient computation of optical flow using the census transform](https://link.springer.com/doi/10.1007/978-3-540-28649-3_10), DAGM 2004;
  Hafner et al., [why census is illumination-robust](https://www.mia.uni-saarland.de/Publications/hafner-ssvm13.pdf).
- ORB/BRIEF: Rublee et al., ICCV 2011, [OpenCV tutorial](https://docs.opencv.org/3.4/d1/d89/tutorial_py_orb.html).
- Moving camera, leftover flow: [Optical flow based real-time moving object detection in unconstrained scenes](https://arxiv.org/abs/1807.04890)
  (homography from sparse flow, RANSAC, residual flow = movers);
  [REDBEE](https://arxiv.org/abs/1712.09162) (RANSAC fails with many outliers: IMU added).
- Insect EMDs: Harrison, [Caltech thesis 2000](https://thesis.library.caltech.edu/6076/1/Harrison_rr_2000.pdf);
  Iida, [EMD navigation on a flying robot](https://people.csail.mit.edu/iida/papers/ISR393iida.pdf).
  Image interpolation (I2A): Srinivasan, "An image-interpolation technique for the
  computation of optic flow and egomotion", Biol. Cybern. 71, 1994 (a global
  shift by interpolation; global only, like phase correlation).
- RP2350: [Raspberry Pi product page](https://www.raspberrypi.com/products/rp2350/)
  ("hardware single-precision floating point and DSP instructions"). SMLAD and
  USADA8 are part of the Armv8-M DSP extension; confirm in the RP2350 datasheet's
  Cortex-M33 section before V1 relies on them.
- Cycle counts in §8 are my estimates from operation counts, not measurements.
