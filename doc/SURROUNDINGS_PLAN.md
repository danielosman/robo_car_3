# Surroundings: the ground as a cell, evidence per reading (plan)

Status: **plan, nothing implemented.** It replaces ROBOT_PLAN.md §4.2-§4.5 and the
floor parts of §14 (per-zone floor learning, floor rows, the 2 cm obstacle margin,
the "no floor" counter, four layers from 2 cm). Written next to
[ARCHITECTURE_PLAN.md](ARCHITECTURE_PLAN.md) (modules, `view`, `ray_meas_t`),
TOF_MOTION_PLAN.md (owns the per-reading confidence, `tof_quality`) and
[CAMERA_MOTION_PLAN.md](CAMERA_MOTION_PLAN.md). The cross-plan decisions (shared
types, one recording path R0, the measurement session M-S, the milestone order)
are followed here. Sources in §12.

**Conventions.** VL53 rows are numbered **1-8, row 1 at the top** (README; the code's
indexes are one less): row r's centre is (r − 4.5) × 5.625° below level, so rows 1-4
look up and rows 5-8 look down. (This file counted from 0 until 10 Oct; all its row
numbers were moved up by one then.) The robot is **12 cm tall**
(Daniel; ROBOT_PLAN §3 still says 10 cm and is fixed in S3).

## 1. In short

- **No floor learning.** Geometry alone says whether a reading is the floor:
  the zone's cone (5.6° tall, not a ray) at the measured range either lies inside
  the ground band, lies wholly above it, or straddles its top (§3).
- **Three layers of 10 cm cells:**
  - **G** (ground, −7…+3 cm): "is there floor here?"
  - **L1** (3-13 cm): what blocks the 12 cm robot, with 1 cm to spare
  - **L2** (13-23 cm): overhangs, and margin above the robot.
- **Anything taller than 3 cm is an obstacle.** No special small obstacles. Things
  under 3 cm (cables, rug edges, door sills) count as ground, and the geometry
  can't tell them from the floor anyway (§3).
- **A column is drivable when G is floor, L1 is free and L2 is free** (Daniel's
  rule). One exception, forced by geometry: L2 cells within 25 cm of the robot's
  centre are never seen by any zone, so they count as free once the robot stands
  there (§4.4).
- **Every cell is an int8 log-odds value** (OctoMap-style hit and miss steps,
  clamped), each update scaled by the reading's **confidence** from the shared ToF
  model, so one sure reading counts as much as two doubtful ones.
- **A reading says three things:**
  - the cone was empty up to the surface (free space in L1 and L2, only where the
    cone reaches far enough down, §4.3);
  - what the surface is: floor → G, obstacle → L1/L2, or "can't tell";
  - for the low rows, whether the floor was missing where it had to be (a drop
    → G negative).
- **Differs from Daniel's suggestion** ("ground cell centred 5 cm below the floor",
  i.e. G = −10…0 cm): the top of the ground band is at **+3 cm**. With a top at 0 the
  cone rule could never call row 6's or row 7's readings floor, and the hit-point
  method gives a ring of false obstacles (§3.2). Open question 1.
- Removed: `rangefinder`'s floor learning and the 300 kept start-up frames, the old
  map cells (together −135 KB RAM), `misses_left` / `sightings` / `no_floor` /
  `floor_seen`, `HORIZON_ROW`, `EDGE_FLOOR_SHARE`, `DROP_SHARE`. The new map takes
  ~13 KB.

## 2. What is wrong with today's floor handling

| Today | Problem |
|---|---|
| Each floor zone learns its floor distance and scatter during the start-up turn | Needs a full turn first (the map is empty until then), 300 kept frames (64 KB) and 33 KB of samples; failed when half the turn looked over a desk edge (CHANGELOG 5 Oct); can't follow a change of floor (carpet edge) after the start |
| Rows 5, 6, 7-8 each have special rules (horizon row, row 6 "tells nothing" when far, rows 7-8 tell drops, an unlearned row 5 used like the rows above) | Five cases to keep consistent; each was a fix for one robot run |
| Layer 0 = 2-12 cm, judged by a learned margin per zone; far walls show only as `''` | The 2 cm line is judged per zone in range, the layers by height: two systems |
| Counters: 6 misses clear, 3 "no floor" make `?` | Every reading counts the same: a sure status 5 and a doubtful status 9 alike |
| Free space walked along the zone's centre only | Radial lines of unknown cells inside free areas (REDFLAGS M2) |

## 3. Geometry: the cone decides, not the hit point

### 3.1 The picture

A zone doesn't measure along a line. It sees the nearest (in practice: the
strongest) surface **anywhere in its cone**, 5.6° tall. Side view of one zone that
looks down (row 7), with the four lines the rule uses:

```
 height                                                    range r of the reading
                                                                     :
  7 cm  S . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .:. . .  sensor height (level)
          '--..__                                                    :
           \     ''--..__          UPPER cone edge (angle θ − α)     :
            \            ''--..__                                    :
             \                   ''--..__                            :
              \      the zone's cone     ''--..__                    :
               \                                 ''--..__            :
  3 cm  - - - - \ - - - - - - - - - - - - - - - - - - - - - ''--..__ :  - - -  H: top of the ground band
                 \                                                 ''* z_hi(r)
  0 cm  ==========\==================================================|======  FLOOR
                   \  LOWER cone edge (angle θ + α)                   |
                    \                                                 * z_lo(r)
                                                                  the SLICE at r
```

- **The slice** is the vertical stretch of the cone at the measured range r:
  from **z_lo(r)** (where the lower edge is) to **z_hi(r)** (where the upper edge
  is). The surface the zone saw is somewhere in it; the sensor can't say where.
- **H** is the top of the ground band, 3 cm above the floor. It is a height, not a
  distance: "3 cm" never means "3 cm of range".
- A z_lo below the floor (as drawn) just means the lower edge had already reached
  the floor before r.

```
z_lo(r) = 7 cm − r · sin(θ + α)      θ: the zone's centre angle below level (pitch and roll included)
z_hi(r) = 7 cm − r · sin(θ − α)      α: half the zone, 2.8°, + slack for pitch error (0.5° still, more when braking, §3.4)
```

The slice is ±0.05 r around the centre ray: ±1 cm at 21 cm, ±2.5 cm at 50 cm,
±7.5 cm at 1.5 m. The sensor's own range noise (sigma, a few mm to ~2 cm) is small
next to it.

**The rule** compares the whole slice with H:

```
      FLOOR                    OBSTACLE                 CAN'T TELL
  slice wholly below H     slice wholly above H     slice crosses H

                                   * z_hi                   * z_hi
                                   |                        |
                                   * z_lo                   |
  H - - - - - - - -        H - - - - - - - -        H - - - | - - - -
        * z_hi                                              |
  0 ====|========          0 ===============        0 ======|========
        * z_lo                                              * z_lo
```

| Slice at r | Means | What the map gets |
|---|---|---|
| **z_hi ≤ H** | the floor, or something under 3 cm | **FLOOR**: G gets floor evidence |
| **z_lo > H** | something at least z_lo tall stands there | **OBSTACLE**: the L1/L2 cells overlapping the slice get a hit |
| otherwise | could be the floor, could be a box | **NOTHING** about the surface (free space only) |

### 3.2 Why not the hit point (Daniel's anchoring, checked)

Binning the centre-ray hit point (centre direction × range → height → layer):

| Row | Typical floor reading | Centre-ray height | Floor readings seen | Their centre-ray heights |
|---|---|---|---|---|
| 8 | 21 cm | −0.1 cm | 19-24 | +0.6 … −1.1 |
| 7 | 31 cm | −0.5 cm | 28-33 | +0.2 … −1.0 |
| 6 | 47 cm | **+0.1 cm** | 35-53 | **+1.9** … −0.8 |
| 5 | (lower edge grazes the floor) | | 66-80 | **+3.8 … +3.1** |

- **Ground band −10…0 cm:** row 6's floor lands above 0 about half the time: a ring
  of false obstacles at ~40-50 cm, the ring M2 had on its first run.
- **Ground band −7…+3 cm:** fixes rows 6-8, but not row 5 grazing the floor at
  66-80 cm (+3.1…+3.8 cm, +1.2 cm more per degree of nose-up). It would need a
  5 cm band, and the slack still grows with range. Sigma doesn't help: the error is
  the cone, not the range.

### 3.3 The rule's numbers per row (α = 3.3°)

| Row | Floor patch (where the cone meets a flat floor) | OBSTACLE if r < | FLOOR if r ≥ | Range error needed before the floor reads as an obstacle |
|---|---|---|---|---|
| 1-4 | never (they look level or up) | always (z_lo ≥ 5.7 cm) | never | — |
| 5 | 65.7 cm … ∞ | 37.6 cm | never | 28 cm (or 2.9° of pitch) |
| 6 | 34.4-78.2 cm | 19.7 cm | 44.7 cm | 15 cm |
| 7 | 23.5-37.5 cm | 13.4 cm | 21.4 cm | 10 cm |
| 8 | 17.9-24.8 cm | 10.2 cm | 14.2 cm | 8 cm |

A floor reading has to be wrong by 8-28 cm of range to become an obstacle. Pitch
errors move z by r·sin(Δp): 0.35 cm per degree at 20 cm. Roll −5° during turns
(README) tilts the edge columns by ~1.7°, still inside the margins. **No floor
learning is needed.**

What it costs:
- **The smallest obstacle is 3 cm.** Anything lower is ground and gets driven at.
  (Today an obstacle of 2-3 cm is also seen only within ~25 cm, by rows 6-7.)
- **A 3-7 cm object is first seen** when row 5's slice clears H, at r < 37.6 cm
  along the ray: **~30 cm ahead of the front edge** (the sensor is 7 cm behind it).
  Today's map shows an 8 cm box at ~40 cm along the ray, about the same. Taller
  things are seen by row 4 at any range.
- **The floor is seen out to ~78 cm**, by rows 6-8 only. Row 6 readings of
  34-44.7 cm say nothing about the surface. A row 6 FLOOR reading at r ≥ 44.7 cm
  marks G from where its cone meets the floor (34 cm) to r, so a reading of 47 cm
  covers 34-47 cm; 47-78 cm stays unclaimed. How big the `.` disc really is depends
  on how often row 6 reads ≥ 44.7 cm: measured in M-S (§9). Beyond the disc:
  "open, floor not seen" (`:`), as today.
- **A low box can be read as floor from 45-75 cm.** Row 6's slice at 50 cm is
  −3.2…+2.5 cm; if its return comes from the lowest 2.5 cm of an 8 cm box's face,
  that is FLOOR, and G in the box's column says floor. The column still isn't
  drivable: row 5 sees the box (its slice crosses H, so NOTHING), its free space
  stops 5 cm short of the box, and L1 there stays unknown (§5, test in S2).

### 3.4 Pitch: where it comes from and the slack

PicoB's pitch is a complementary filter (`odometry.c`, τ = 0.5 s): gyro for fast
changes, the accelerometer's gravity direction over ~0.5 s. Braking or speeding up
at a m/s² tilts the measured gravity by atan(a / g): 2.3° at the drive's 0.4 m/s²
ramp, and the filter passes part of that for a few tenths of a second. So the
slack in α is **0.5° + atan(|a| / g)**, with `a` from the change of odometry's
speed: 0.5° standing or cruising, up to ~2.8° while braking. Row 5 needs 2.9° of
pitch error before its floor turns into an obstacle. S2 tests a wrong pitch spike.

### 3.5 Drops

A drop is missing floor where a cone had to meet it. For a zone with a short floor
patch [d1, d2] (rows 6-8; row 5's patch never ends):

- **No credible target** ("nothing" ray), or the nearest credible target **beyond
  d2** (by 10 % + 3 sigma_eff): the cone crossed the floor plane between d1 and d2
  and nothing came back from there → **G cells under the patch get "no floor"**.
- **Weight:** the ray's confidence (a "nothing" ray has 0.5, the shared model's
  value; a far credible return its own confidence), times 1 for rows 7-8 and 1/3
  for row 6 (it reflects off waxed wood ~1 in 24 readings).
- **Veto:** if the zone's `weak_range_m` (nearest non-credible target) lies inside
  [d1, d2], it probably saw the floor faintly: no G update. A glossy floor often
  reflects the light to the wall but may still return a faint floor target.
  M-S checks this.
- **Glitch guard:** a frame in which all 64 zones report nothing gives no drop
  evidence (a sensor hiccup would draw a ring of `?`). M-S shows whether partial
  hiccups happen.
- A far return also claims free space up to it, in L1 and L2 only: the part of
  the cone below H is never used for free space.

## 4. The map

### 4.1 Grid and memory

- 10 × 10 cm columns, **40 × 40 (4 × 4 m)** around the robot, world-aligned,
  re-centred in whole cells, circular indexing: as today.
- Per column: **G, L1, L2 as int8 log-odds** (3 B) and `seen_at` (u16 robot
  seconds, for staleness): 1 600 × 5 B = **8 KB**. One frame's pending updates
  (int8 per cell, §4.3): **4.8 KB**. Total **~13 KB**.
- Nothing above 23 cm is stored: tables and the upper rows' far hits don't matter
  for driving.

### 4.2 Log-odds, numbers

Units: **1/32 of a nat** of log-odds, in an int8. Small units keep the hit/miss
ratio the same at any confidence (with 1/8 nat, w = 0.5 would round 3.5 and 1.5
and shift the balance). Probabilities in brackets.

| | Value | |
|---|---|---|
| Hit (obstacle in L1/L2, floor in G) | **+27 × w** | (0.70 for w = 1: OctoMap's p_hit) |
| Miss (free in L1/L2, no floor in G) | **−13 × w** | (0.40: OctoMap's p_miss) |
| Clamp | **−64 … +80** | (0.12 … 0.92) |
| Occupied / floor | **≥ +16** | (> 0.62) |
| Free / no floor | **≤ −16** | (< 0.38) |
| Unknown | between, or never seen (0) | |

`w` is the ray's confidence from the shared model, times 0.5 when `sigma_eff` >
50 mm (the map's own use of sigma; the other is the free-space stop margin, §4.3).
**Only credible targets (confidence ≥ 0.25) make rays**; "nothing" rays have
w = 0.5. Nothing fades (Daniel, ROBOT_PLAN §14); age is in `seen_at`.

What these numbers give at 15 Hz:

| Case | Frames | Time |
|---|---|---|
| Something appears in a free cell (−64) | 3 sure hits: −37, −10, +17 | 0.2 s |
| The same at confidence 0.5 (status 6 or 9) | 6 | 0.4 s |
| A removed obstacle clears (+80) | 8 sure misses | 0.53 s (today 6, 0.4 s) |
| A drop where nothing was known (0) | 2 far returns (−13, −26), or 3 "nothing" rays | 0.13-0.2 s |
| One glossy reflection on known floor (+80) | −13, then floor again | stays floor |
| A thin thing stays occupied if it is returned in more than | ~1/3 of the frames that look through it (27f = 13(1 − f)), at any confidence | |

### 4.3 One frame

For each ray (one per zone, §6):

1. **Free space.** Five sub-rays (the centre and ±2° up, down, left, right), walked
   in 5 cm steps from the sensor up to r − max(5 cm, 3 sigma_eff) (or `max_m`, 1 m,
   for "nothing"). A point between 3 and 23 cm high gives a miss:
   - **to L1 only where the zone's cone at that distance reaches down to
     z_lo ≤ H + 2 cm (5 cm).** Obstacles stand on the floor, so they always fill
     L1's bottom. A cone passing above a low box says nothing about the box. This
     is how rows 5-8 cover L1's bottom: from about 19 / 10 / 7 / 5 cm along the
     ray. Row 4 only does it beyond 2.3 m, rows 1-3 never;
   - to L2 at any covered height. What matters in L2 is its lowest part (does
     anything hang down toward the robot?), and the cone sees that first.
   A sub-ray stops where it goes below H: the floor or a drop, not free space.
   (Fixes the radial unknown lines: at 2 m a zone is 20 cm wide, two cells.)
2. **The surface** (§3.1):
   - FLOOR: G hits for the cells under the patch, from where the cone meets the
     floor (d1) to r, along the zone's bearing.
   - OBSTACLE: a hit in each of L1/L2 that overlaps [z_lo, min(z_hi, 23 cm)], at
     the cell r·cos(elevation) ahead along the bearing.
   - NOTHING: no surface update.
3. **Drops** (§3.5): "no floor" in G under the patch.
4. **Around the robot:** within 15 cm of its centre (the body, and its turning
   circle), G floor and L1 free at full weight: it stands and turns there (today's
   `stand_on`). L2 cells within 25 cm that are still unknown are set to free once
   (§4.4).

**One update per cell per frame:** many sub-rays cross the cells near the sensor,
and one frame must not count as 20 misses. Updates go into the pending buffer:
- a hit replaces any miss, and a bigger hit replaces a smaller one;
- of several misses, the strongest wins.

At the end of the frame they are added to the cells, clamped, and the buffer is
cleared.

**Changes** (ROBOT_PLAN §4.6, for M5) are reported with hysteresis: an L1 cell that
reaches ≥ +16 after it was last ≤ −16 (APPEARED), or the reverse (CLEARED). Cells
hovering in the unknown band report nothing. M5 groups them; movement (not the map)
is what decisions use for people.

**Cost:** 64 zones × 5 sub-rays × ≤ 40 steps ≈ 13 k steps per frame at most, ~6 k
typically. At 30-50 cycles a step: ~2 ms typically and up to ~4.3 ms worst case.
Target: **typical < 2 ms, worst frame < 5 ms** (printed by `p`, mean and worst).
If it's over: sub-rays only for rows 1-5 (the long ones).

### 4.4 Columns and queries

One function decides what a column is, used by the print and every query:

| Column | Condition | Print |
|---|---|---|
| BLOCKED | L1 ≥ +16 | `##` |
| OVERHANG | L2 ≥ +16, L1 not | `''` |
| NO_FLOOR | G ≤ −16 | `? ` |
| DRIVABLE | G ≥ +16, L1 ≤ −16, **L2 ≤ −16** | `. ` |
| OPEN | L1 ≤ −16, L2 ≤ −16, G unknown | `: ` |
| UNKNOWN | anything else (e.g. L2 still unknown) | blank |

**Is L2 ever free near the robot?** Rows 1-3 look up 8-23°. Row 1's lower edge
(16.4° up) reaches 13 cm at **20 cm ahead of the sensor** (23 cm from the robot's
centre). Rows 1-4 together cover all of 13-23 cm from ~38 cm on. Between 20 and
38 cm only L2's lower part is seen, which is the part that matters (above ~16 cm
nothing touches a 12 cm robot). Inside ~20 cm ahead of the sensor, and beside and
behind the robot within the same distance, **no zone ever sees L2**, however the
robot turns. Without a rule, after the start-up scan the corridor's first cells
past the front edge (7-27 cm ahead of the sensor) would stay UNKNOWN in L2 and the
robot could never start forward.

**The rule:** L2 cells within **25 cm of the robot's centre** that are still
unknown are set to −16 (free) once. Why that's safe enough:
- while driving forward, every cell is seen in L2 at 20-40 cm ahead before it
  gets this close, so the rule only fills cells nobody could have seen;
- at the start, the robot stands there, and anything hanging 13-16 cm above the
  floor within 25 cm of it would be in the way of a 12 cm robot only if the robot
  were already under it.

Host test in S2: after the scan the corridor ahead is DRIVABLE from the front edge
on, and an overhang at 14 cm, 40 cm ahead, is OVERHANG.

Queries (names as in ARCHITECTURE_PLAN §5):

```c
column_t surroundings_column(float x_m, float y_m);
bool  surroundings_drivable(float x_m, float y_m);          // column == DRIVABLE
// From (x, y) along the unit (dx, dy): distance to the first column that isn't
// OPEN or DRIVABLE (need_floor: isn't DRIVABLE), the window's edge, or max_m.
float surroundings_free_distance(float x_m, float y_m, float dx, float dy, float max_m, bool need_floor);
// The same for a corridor width_m wide (lines every 5 cm across it): the minimum.
float surroundings_corridor(float x_m, float y_m, float heading_rad, float width_m, float max_m, bool need_floor);
int   surroundings_age_s(float x_m, float y_m);              // -1 never seen
bool  surroundings_next_change(change_t *c);                // M5
void  surroundings_print(const pose_t *robot);               // `m`
void  surroundings_print_cells(const pose_t *robot);         // `M`: 2 × 2 m around the robot, each layer's value / 16 as a digit
```

- **Most open direction** (where to look; behaviour): as today, the average
  `free_distance(…, need_floor = false)` across the camera's ±25°.
- **Where to drive** (M4): `corridor(…, 0.295 m (23.5 + 2 × 3 cm), …, need_floor = true)`.
  The Safety guard allows no forward motion unless the corridor's first 20 cm past
  the front edge is DRIVABLE.
- **Staleness** (R8, M5): `age_s` per 45° sector of the 0.3-1.5 m ring.

### 4.5 The printed map (`m`)

```
Map 4 x 4 m, 10 cm cells; up = where the robot faced when the scan started.
. floor seen, nothing 3-23 cm   : nothing 3-23 cm, floor not seen   ## obstacle 3-13 cm
'' obstacle only 13-23 cm       ? no floor where it should be (a drop?)
blank unknown   () robot   ** 30 cm ahead of it
```

The same symbols as today with new heights (3 cm instead of 2, 13 instead of 12).
"Obstacle" now also covers walls seen from far away (row 4's slice reaches L1).

`z` prints, per zone:
- the distance and the confidence (0-9);
- what the reading made of it: `F` floor, `#N` obstacle from N cm up, `~` can't
  tell, `v` no floor, `-` nothing within 1 m, blank: no credible target.

## 5. Worked examples

Robot level, α = 3.3°, sure readings (w = 1).

**Floor ahead: rows 8 / 7 / 6 at 21 / 31 / 47 cm.**
- Row 8: slice −1.2 … +1.1 cm → FLOOR. G hits from 16.6 to 19.8 cm ahead of the
  sensor. Free space: the cone is above H only for the first ~11 cm, under the
  robot.
- Row 7: −2.3 … +1.2 → FLOOR, G from 22.4 to 30 cm.
- Row 6: −2.6 … +2.8 → FLOOR (just), G from 34 to 46 cm. A shorter reading:
  - 40 cm: −1.1 … +3.4 → can't tell, no G update; 35 cm likewise;
  - only 19.7 cm or less would be an obstacle.
- Row 5 (its floor patch starts at 66 cm): any reading of 66 cm or more straddles
  H → nothing about the surface; free space in L1 up to it.
- The robot nodding ±2° when braking moves row 6 at 47 cm by ±1.6 cm (and α grows,
  §3.4): still floor or "can't tell", never an obstacle.

**An 8 cm box, front face 60 cm ahead.**
- Row 6: the floor in front of the box (34-60 cm) is nearer → FLOOR. If it
  returned the box's lowest part instead (§3.3): FLOOR in the box's column too.
- Row 5: its slice at 60 cm is 0.6 … 7.5 cm, all box → reads 60 → crosses H →
  nothing. Its free space stops at 55 cm, so the box column's L1 stays unknown:
  **not DRIVABLE** even if G says floor.
- Row 4: slice 6.5 … 13.4 cm; the box fills its lowest 1.5 cm (~20 %). If the VL53
  returns it: OBSTACLE in L1. Likely only now and then.
- Driving closer: from 37.6 cm along the ray (~30 cm ahead of the front edge),
  row 5's slice lies above H (at 37 cm: 3.1 … 7.3) → OBSTACLE, L1 `##` after 3 frames.
- **Backing away again:** rows 1-4 pass over the box at 7-23 cm, but their cones
  don't reach down to 5 cm there, so their misses don't count in L1. Row 5's cone
  reaches down, but it hits the box. The box stays `##` (today it would clear after
  6 readings that pass over it).

**A wall at 1.5 m.**
- Row 4: slice 5.7 … 23 cm → hits in L1 and L2 at the wall: `##` (today `''`,
  because only the centre point counted, 14 cm high).
- Row 3: slice 20 … 38 cm → a hit in L2 where it overlaps (20-23 cm).
- Row 5: −9 … +8 cm → nothing about the surface. Its cone reaches below 5 cm, so
  its sub-rays free L1 up to ~1.45 m. Rows 1-4 free L2.
- Row 6: the floor, 45-78 cm.
- So: `.` to ~75 cm, `:` to the wall, then `##`.

**A table edge** (robot on a 75 cm desk, the edge 20 cm ahead of the sensor).
- Row 8's patch (17-23 cm ahead) is half on the desk: it reads ~19 cm → FLOOR on
  the desk side.
- Row 7's patch (23-36 cm) is all beyond the edge. The floor below is 3.4 m away
  along the ray: no return, or a far one. No faint target inside the patch → "no
  floor" in G there, `?` after 2-3 frames.
- Row 6: a third of a miss on 34-78 cm per frame.
- Result: `.` up to the edge, `?` beyond it, `:` farther (L1 still free: nothing
  stands in the air). Beyond the edge is never drivable: G must be floor.

**A chair leg** (Ø 2.5 cm, 50 cm ahead; seat at 45 cm).
- At 50 cm a zone is 4.9 cm wide, and the leg fills half. Row 4 (slice 6.6 …
  12.3) returns the leg in most frames → OBSTACLE in L1.
- Row 5's sub-rays from neighbouring zones pass through the leg's cell low down →
  misses. The leg stays `##` while it is returned in ≥ 1/3 of the frames that look
  through it (§4.2). The seat is above 23 cm: not in the map.
- At 1.5 m (zone 15 cm wide) the leg may lose to the wall behind it: it is seen as
  the robot comes closer.

**A person walking through** (1 m/s, 1 m ahead, robot still).
- At 1 m the zones see up to ~46 cm: legs. Rows 1-4 see them → L1/L2 hits. A 10 cm
  cell is covered for ~0.3 s (5 frames): from −64 to +80.
- After they pass: 8 frames (0.53 s) of row 5's misses to free each cell. The map
  shows a ~0.5 m trail that clears within a second. Each cell gives one APPEARED
  and one CLEARED change. Decisions about people use movement, not the map.

**A glossy cupboard on waxed wood.** Rows 7-8 sometimes read "beyond the floor" in
front of it (README). With the veto (a faint target inside the patch), those
readings say nothing. Without one they count as "no floor" and give `?` in front
of the cupboard, as today (the safe side; accepted).

**The in-sensor floor echo** (upper zones: a faint target at ~30 cm, then the real
one). The shared model gives the echo less than 0.25, so it isn't credible and
doesn't make the ray. The ray goes to the real target, and free space goes through
30 cm, as today.

## 6. What the map takes from the shared types

Per cross-plan §2 (adopted as written):
- `tof_quality` (TOF_MOTION_PLAN) gives each target `confidence = status_weight ×
  faint_factor` and a separate `sigma_eff_mm`. **CREDIBLE = confidence ≥ 0.25**, the
  same threshold in `tof_motion`, `view_place_tof` and the map. The map never looks
  at statuses.
- `tof_frame_t` keeps the two nearest credible targets per zone and `weak_mm`, the
  nearest non-credible one (added for the drop veto).
- `view_place_tof()` makes one `ray_meas_t` per zone:
  - the nearest credible target → `range_m`, `sigma_m` = sigma_eff, `confidence`;
  - no target at all → `range_m < 0`, `max_m` 1 m (`tof_quality_clear_m`: less
    in sunlight), confidence 0.5;
  - only non-credible targets → no ray (it says nothing).
- `ray_meas_t` carries `half_rad` (the cone, 2.8125° for the VL53) and
  `weak_range_m` (from `weak_mm`) besides the architecture's fields. `dir_map`
  includes pitch **and roll** (`pose_t` gets roll in A2).
- `view_floor_patch(zone, pitch, roll, &d1, &d2)` gives the patch. `tof_motion`
  uses it too (it replaces `rangefinder_floor_limit_m`).
- `surroundings_add(rays, n, robot)` takes the robot's pose for "around the
  robot" (the rays start at the sensor, not the robot's centre).

The map works out the slice, the patch and the sub-rays from `origin_map[2]`,
`dir_map` and `half_rad` alone. There are no zone rows inside the map, so a camera
or bumper ray fits too. The only VL53 knowledge it keeps is the per-row drop weight
(§3.5), passed as a field if another sensor ever needs it.

## 7. What goes, what stays

**Goes:**
- `rangefinder.c`: `forget/learn/finish_floor`, `samples` (33 KB), `floor_down_rad`,
  `obstacle_min_m`, `HORIZON_ROW`, `FIRST_DROP_ROW`, `EDGE_FLOOR_SHARE`,
  `DROP_SHARE`, `FLOOR_*`, `rangefinder_zone_floor`, `floor_distance`,
  `first_floor_row`, `floor_zones`, `ray_kind_t`, `scan_t`. The poll half goes to
  `tof_sensor` and the geometry to `view` (A2, A4 + T1): nothing of `rangefinder`
  is left.
- `surroundings.c`'s kept frames (64 KB), `learn_start/finish`, the "mapping" flag:
  frames go on the map from the first one.
- `world_map.c`'s `cell_t` (misses_left, sightings, no_floor, floor_seen, frame),
  `MISSES_TO_CLEAR`, `NO_FLOOR_SCANS`, `FIRST_LAYER_M`, `see_floor_along`,
  `clear_along` (replaced by sub-rays). `world_map` and `surroundings` merge into
  `surroundings` (architecture).
- `behaviour`: `surroundings_learn_finish()`, the "Floor learned in N of 32 zones"
  line, and with them the scan's blocking map build. The MAPPING state is kept
  only until S3; **architecture A3 must not chunk the old map build** (S3 deletes
  it; cross-plan §7 says so too).
- `z`: the "Learned floor" block.

**Stays:**
- the window, re-centring and circular indexing; `seen_at`;
- one update per cell per frame; under the robot is floor;
- frames placed with `pose_at`; nothing fades; "nothing" trusted to 1 m;
- the 390° start-up scan (coverage, and M5's full-turn check);
- `most_open_heading`; the print layout and most symbols; change counting.

## 8. Rejected alternatives

| Alternative | Why not |
|---|---|
| Ground band −10…0, centre-ray height (Daniel's first idea) | Row 6's floor sits at −0.8…+1.9: a false ring (§3.2) |
| Ground band −7…+3, centre-ray height | Fine for rows 6-8; row 5 grazing the floor at 66-80 cm lands at +3.1…+3.8 (+1.2 per degree): needs a 5 cm band, still range-dependent |
| Keep floor learning, as a per-zone range correction | Not needed once the cone decides (8-28 cm of margin); costs ~96 KB, a full turn before anything is mapped, and failed at the desk |
| "L2 not occupied" instead of free (first draft) | Daniel's rule is achievable: rows 1-3 see L2 from 20 cm ahead of the sensor; only the ring right around the robot needs the rule in §4.4 |
| Misses count in L1 at any height in the layer (first draft) | Cones passing at 8-12 cm erase a 5 cm box within ~8 frames once the robot backs off (§4.3) |
| A height per column (elevation map, as on legged robots) | A VL53 reading gives a 1-15 cm slice, not a height; overhangs need layers anyway |
| OctoMap's octree | 4 800 cells fit in a flat array; an octree saves nothing here |
| Spread an obstacle's evidence over the arc (Elfes' sonar weights) | The arc covers at most 2 layers below 2 m; full weight in both is simpler and on the safe side |
| Readings short of the floor patch but under 3 cm high (row 5 at 38-66 cm) as weak obstacles | Would see the 8 cm box from 60 cm, but also cables and rug edges; revisit with M-S data (open question 3) |
| Non-credible targets for occupancy at a low weight | The floor echo at ~30 cm would draw a ring; they are used only for the drop veto |
| Decay toward unknown (Nav2's spatio-temporal voxel layer) | Daniel: nothing fades; age is the behaviour's business |
| Fixed hit/miss counters (today) | Can't weigh a sure reading more than a doubtful one |
| My own text dump of raw frames (first draft's S0) | One recording path for all plans: R0 |

## 9. Milestones

In the merged order (cross-plan §7): **R0** recording → **M-S** measurement session →
**#7** `tof_quality`, `tof_frame_t`, `view` → **S1** (#8) → **S2** (#11) → **S3**
(#12, must land before V1 because of RAM) → **S4** (#21, after A5). Golden clips
(the old "S5") are added continuously from M-S on. Each milestone ends with
`./run_tests.sh` passing, the robot tests below, `p`'s RAM line (a milestone that
leaves < 40 KB free fails), a red-flag review (REDFLAGS.md) and a CHANGELOG entry.

**Host test scenes** (`picoA/app/test/sim_room.h`, shared with the behaviour and
movement tests; built from today's `cast()` in `test_behaviour.c`):
- the sensor model: each zone reports the nearest of 9 sample rays across its
  5.6° × 5.6° patch, with range noise ±1 cm, a second target, statuses and the
  confidences from `tof_quality`;
- scene options:
  - walls;
  - boxes 5 and 8 cm tall;
  - a 75 cm desk with an edge;
  - a chair (4 legs Ø 2.5 cm, seat at 45 cm);
  - an overhang (a shelf from 14 cm up);
  - a walker (two legs with a gait, cross-plan §1);
  - a glossy floor: with probability p a zone reports the wall instead of the
    floor, plus a faint non-credible floor target;
  - the floor echo (a faint target at 30 cm in rows 1-5);
  - pitch and roll as a function of time, and a separate *measured* pitch that can
    be wrong.

### Our part of M-S (the joint measurement session)

Recorded with R0's `R` (TOF_RAW records with all 4 targets, ODOM, MARKs). Our
situations are in cross-plan §4's list. What each is for here:

| Situation | Decides |
|---|---|
| Open floor; the floor turned by hand | Which statuses rows 5-8 give on the floor; how often row 6 reads ≥ 44.7 cm (the `.` disc) |
| `n` scan; `r` turns | Roll during turns; pitch while starting and stopping |
| Table edge 15 cm | The drop rule: no return vs far return; any faint target in the patch? |
| Glossy cupboard at 40 cm | The veto: does a faint floor target come with the reflections? |
| Box at 60 / 45 / 30 cm | Row 5's and row 4's returns from a low box; row 6 reading the box as floor |
| Chair at 50 cm | How often a leg is returned (vs the 1/3 rule) |
| Carpet | "Nothing" within 1 m on dark carpet; no false drops at its edge |
| Walks at 1 / 2 m | The trail and how fast it clears |

`build/replay file.rec --map` prints, per row, the share of each status, the median
sigma_eff and signal, the share of FLOOR / OBSTACLE / NOTHING / NO_FLOOR, and the map
at the end of the clip. The clips that show something go into
`picoA/app/test/recordings/` with their expected map properties (golden tests).

### S1 — Classify readings (pure function, no map change)

`surroundings_classify(const ray_meas_t *r, slice_t *s)` → FLOOR / OBSTACLE /
NOTHING / NO_FLOOR, with z_lo, z_hi, the patch and the weight. `z` shows its result
next to the old "what each zone makes of it".

Host tests (`test_classify`):
- The §3.3 table: for each row, the patch and the obstacle and floor limits within
  0.5 cm of the numbers above. Row 8 at 21 cm → FLOOR with z_lo −1.2 ± 0.1, z_hi
  1.1 ± 0.1.
- A flat floor with pitch −2°…+2° in 0.5° steps and roll −5…+5°, noise ±2 cm,
  10 000 random readings per zone across its patch. Expected: **0 OBSTACLE**; rows
  7-8 always FLOOR; row 6 FLOOR or NOTHING; row 5 never FLOOR. The same with roll
  ±5° changing *during* a simulated turn, and with the pitch sequence of the M-S
  `n` scan clip.
- The 8 cm box at 60 / 45 / 35 / 30 cm: row 5 NOTHING, NOTHING, OBSTACLE, OBSTACLE.
  At 50 cm, row 6 returning the box's base → FLOOR (documented, §3.3).
- A 2 cm thing at 15-40 cm: never OBSTACLE.
- The desk edge: rows 7-8 beyond the edge → NO_FLOOR. With `weak_range_m` inside the
  patch → NOTHING (veto). A frame with all 64 zones empty → no NO_FLOOR.
- The floor echo plus a wall at 1.5 m (row 3): the ray is the wall, OBSTACLE.
- Braking: true pitch 0, measured pitch +3° for 0.2 s, a = 0.4 m/s² → α slack 2.8°:
  0 OBSTACLE in rows 5-8.
- Golden: the M-S open-floor clips replayed → 0 OBSTACLE in rows 5-8.

Robot (PicoA, PicoB running):
1. On open floor, `z` five times. Expected: rows 7-8 `F`; row 6 `F` or `~`; row 5
   `~` or `-`; rows 1-4 `#N` (walls) or `-`. Paste one.
2. A box at 60 cm, then at 35 cm, `z` at each. Expected: at 60 cm the row-4 zones
   behind the box `~`; at 35 cm `#3`-`#4` there.
3. Robot 15 cm from the table edge (hand ready), `z`. Expected: `v` in row 7 (and
   7) across the zones that look over the edge, `F` on the desk side.
4. Facing the glossy cupboard at ~40 cm, `z` five times: count the `v` (paste).

### S2 — The new map next to the old one

`surroundings` (the merged module) with the log-odds cells, sub-rays, pending
buffer and the column function. It is fed the same frames as the old map: `m`
prints the old map, `M` the new one and `N` its cell values (from S3 on: `m` and
`M`, §4.4). Behaviour still uses the old one.

Host tests (`test_surroundings`):
- One cell:
  - 3 sure hits from −64 → OCCUPIED; 6 hits at w = 0.5;
  - 8 misses from +80 → FREE; never beyond −64 / +80;
  - the share of frames a thin thing must be returned in to stay occupied, at
    w = 1 and w = 0.5: within 3 % of each other.
- One frame where 30 sub-rays cross a cell and one ray ends in it: exactly one hit,
  no misses. Two frames: two updates.
- A thin leg returned in 40 % of the frames stays BLOCKED over 300 frames; at 20 %
  it clears.
- Around the robot: G floor and L1 free within 15 cm after one frame; L2 unknown
  cells within 25 cm become free once; an L2 cell seen occupied inside 25 cm stays
  occupied.
- The window re-centres: cells inside both windows keep their values; the others
  go to 0.
- Changes: a cell oscillating between +10 and −10 reports nothing; −64 → +80 → −64
  reports APPEARED then CLEARED.
- Scenes (the robot does the start-up turn in `sim_room`, then the map is checked
  cell by cell against the scene):
  - Living room (today's walls and the 8 cm box at 40 cm):
    - no BLOCKED cell on open floor;
    - ≥ 80 % of the wall cells within 2 m BLOCKED (not OVERHANG);
    - the box's cells BLOCKED;
    - DRIVABLE everywhere within 40 cm except the box;
    - **0 NO_FLOOR**;
    - no UNKNOWN cell inside the free area within 1.5 m (the radial lines are
      gone);
    - **the corridor ahead (29.5 cm wide) DRIVABLE from the front edge on**, in
      every direction without an obstacle within 50 cm.
  - Overhang: a shelf from 14 cm up, 40 cm ahead → those cells OVERHANG, not
    DRIVABLE; the cells in front of it DRIVABLE.
  - **Low boxes kept:** boxes of 5 and 8 cm seen BLOCKED at 35 cm; the robot backs
    to 1 m and turns ±30° for 20 s → still BLOCKED.
  - **Box read as floor:** an 8 cm box at 45, 50, 60, 70 cm along the ray, robot
    still for 5 s: its column is never DRIVABLE.
  - Braking: drive 1 m forward and stop, 20 times, true pitch −2…+2° over 0.5 s;
    then the same with the measured pitch wrong by ±3° for 0.2 s at each stop: 0
    BLOCKED on open floor.
  - Desk, start 15 cm from the edge:
    - ≥ 70 % of the cells just beyond the edge within 50 cm NO_FLOOR;
    - none beyond the edge DRIVABLE;
    - the desk DRIVABLE.
  - Glossy floor with p = 1/24: 0 NO_FLOOR. With p = 1/2 in front of a "cupboard"
    face: NO_FLOOR only within 20 cm of the face.
  - Chair at 50-70 cm: ≥ 3 of 4 leg cells BLOCKED; the cells between the legs OPEN
    or DRIVABLE; nothing from the seat.
  - Walker crossing at 1 m, 1 m/s, robot still: each cell the legs pass is BLOCKED
    within 0.25 s and FREE ≤ 0.7 s after they left; nothing left after 2 s.
  - Removed box: after the scan the box is removed and the robot turns once more:
    its cells FREE.
- Cost: cell visits per frame counted; ≤ 15 000 in every scene.
- Golden: the M-S clips replayed. Open floor → 0 BLOCKED within 1 m; table edge →
  NO_FLOOR along the edge; boxes → BLOCKED from ≤ 40 cm.

Robot (both Picos, battery on):
1. Flash `picoA_app`. `n`: the scan runs as before ("Floor learned…" still comes
   from the old map). After "Facing heading…": `m`, then `M`. Paste both. Expected in
   `M`:
   - walls `##` also beyond 1 m (where `m` shows `''`);
   - a `.` disc of ~50-75 cm, `:` beyond;
   - no `##` or `?` on open floor, no blank cells inside the free area;
   - `.` right up to the robot's front edge.
2. `q` (square), then `m` and `M`. Expected: no `##` from braking in either.
3. Put a box (8 cm) ~35 cm ahead-left, `n`, `M`: the box `##`. Carry the robot back
   to ~1 m from it (motors off), `a`, wait 20 s, `M`: the box still `##`. Take the
   box away, `r`, then `s` after one turn, `M`: the box cleared.
4. At the table edge (held, motors off), turn the robot slowly by hand ±45°, `M`:
   `?` along the edge, `:` beyond.
5. Facing the glossy cupboard: `n`, `M`. Count the `?` in front of it and compare
   with `m`.
6. `p`: the map's line "map N us per frame (worst M)": N < 2 ms, M < 5 ms; RAM free.

### S3 — Switch over and remove floor learning

Behaviour, the console and M4's coming guard use the new map. The old map, floor
learning and the kept frames are deleted (§7), and so is the MAPPING state. `m`
prints the new map.

Docs updated:
- the README's map section, the M2 robot tests and "Known geometry limits" are
  rewritten;
- ROBOT_PLAN §4 and §14 point here, and §3 gets the 12 cm height.

Host tests:
- `test_behaviour`'s scan in the living room picks the same heading (within 10°)
  as before; the map assertions of S2 move into `test_behaviour`;
- `test_rangefinder` and `test_world_map` are deleted, or reduced to what moved to
  `view` / `tof_sensor`.

Robot:
1. M2 steps 1-7 (rewritten): `n`, map, "Most open direction…". Expected: no "Floor
   learned" line and no motors-off pause; the map has the §4.5 legend; walls and
   furniture where they are; `.` disc, `:` beyond; no `##` or `?` on open floor.
2. On the desk, the scan started 15 cm from the edge (hand ready): `?` along the
   edge from the first half-turn on. The heading printed must not point over the
   edge (`?` and `:` aren't "open" for driving; check that "most open" avoids them
   too).
3. On the carpet (open from M1): `n`, `m`: no `?` and no `##` on the carpet or at
   its edge (a carpet edge is under 3 cm).
4. `p`: RAM free grew by ~135 KB.
5. Regression set of ARCHITECTURE_PLAN §8.

### S4 — Queries for M4 (after A5)

`corridor`, `drivable`, `age_s` and change events; the Safety guard reads them.

Host tests:
- `corridor`:
  - a 40 cm gap between two boxes 60 cm ahead: the robot's 29.5 cm passes (free
    distance 2 m), while a 25 cm gap blocks at the boxes;
  - `need_floor` stops at the first OPEN column;
  - right after the start-up scan, the corridor is DRIVABLE from the front edge.
- Changes: a box appearing gives one APPEARED at its position; a walker gives
  events only while there.
- Staleness: after a scan, all sectors fresh; after 60 s facing one way, the rear
  sectors' age ≥ 60 s.

Robot:
1. `n`, then a new console key `j` (corridor ahead): "Corridor ahead: N cm
   drivable, M cm open". Put a box 50 cm ahead: N ≈ 40. Paste.
2. With M4 (not part of this plan): drive toward the table edge on the desk. It
   stops with the front edge ≥ 5 cm from the edge.

## 10. Risks

- **The VL53 doesn't return "nearest in the cone"** but a weighted mix. The method
  only assumes the reported range lies within the cone's span at that range. A
  merged return (box plus wall, status 9) can place a phantom surface between them.
  Status 9 has half the confidence and twice the sigma_eff, so it needs 6 frames.
- **Row 6 on waxed wood** gives many 35-44 cm readings that say nothing. If row 6
  rarely reaches 44.7 cm, the `.` disc shrinks to ~37 cm (rows 7-8). M-S measures it.
- **Pitch while braking** (§3.4): the slack covers the accelerometer's share; a
  bigger filter error (bumps, a wheel on a sill) could still shift floor and drop
  edges, never by enough to turn the floor into obstacles (2.9° needed).
- **3 cm** is a hard line: a 3 cm book is ground. 3-7 cm objects are seen only from
  ~30 cm ahead of the front edge: 2 s at 0.15 m/s.
- **The L2 ring rule** (§4.4) is an assumption: a shelf at 13-16 cm within 25 cm of
  where the robot was put down would be called free.
- **Low boxes kept forever** while nothing looks at their base: a box taken away
  while the robot is farther than ~2 m stays `##` until row 5 looks there again.
  That is the safe side.
- **CPU:** worst frames may exceed 5 ms; fallback in §4.3.
- **Changes** are noisier with weighting: hysteresis in §4.3, grouping in M5.

## 11. Open questions (for Daniel)

**Answered (Daniel, 8 Oct): see [REWORK_PLAN.md](REWORK_PLAN.md) "Decisions".** 1: 3 cm is fine, no climb test (usually the floor; slow driving in controlled rooms; tilt/shock detection later); 2: L2 must be free, no extra margin, the ring rule stays. 3-5 stay open (measured in S2/M-S); 4 before M4.

1. **Ground band top at 3 cm** (anything lower is driven at) instead of "ground
   centre 5 cm below the floor" (top at 0, which the cone geometry can't support,
   §3.2)? Can the wheels (Ø 9 cm) cross 3 cm reliably?
2. **Clearance:** L1 ends at 13 cm (12 cm robot + 1 cm), and L2 (13-23 cm) must be
   free too, with the ring rule of §4.4. OK, or is "not occupied" enough for L2?
3. Should a reading **short of the floor patch but under 3 cm high** (row 5 at
   38-66 cm) count as a weak obstacle, to see an 8 cm box from 60 cm?
4. **Grid size:** 6 × 6 m costs ~29 KB. Decide before M4.
5. **`max_m` for "nothing":** 1 m still right on dark carpet and black furniture
   (M-S carpet clip)?
6. Answer to ARCHITECTURE_PLAN open question 2: **floor learning goes completely**;
   nothing of it survives as a range correction.

## 12. Sources

- Occupancy grids, inverse sensor model, free space along the beam: A. Elfes,
  "Using occupancy grids for mobile robot perception and navigation", IEEE Computer
  1989, <https://doi.org/10.1109/2.30720>; Thrun, Burgard, Fox, *Probabilistic
  Robotics*, ch. 9, <https://www.probabilistic-robotics.org/>.
- OctoMap (log-odds, clamping, hit 0.7 / miss 0.4, clamp 0.12 / 0.97):
  <https://octomap.github.io/>, Hornung et al., Autonomous Robots 2013,
  <https://doi.org/10.1007/s10514-012-9321-0>; the defaults as documented in MRPT's
  wrapper: <https://docs.mrpt.org/reference/2.0.4/_c_octo_map_base_8h_source.html>.
- Ray walking through a grid: Amanatides & Woo, "A Fast Voxel Traversal Algorithm",
  1987, <http://www.cse.yorku.ca/~amana/research/grid.pdf>.
- Obstacles = anything above a height in a voxel layer, ground filtered by height:
  ROS costmap_2d obstacle/voxel layer (`min_obstacle_height`, `mark_threshold`),
  <http://wiki.ros.org/costmap_2d/hydro/obstacles>. Decay instead (rejected): Nav2
  spatio-temporal voxel layer, <https://github.com/SteveMacenski/spatio_temporal_voxel_layer>.
- Elevation maps with uncertainty (rejected): Fankhauser et al., "Robot-centric
  elevation mapping with uncertainty estimates", 2014,
  <https://www.research-collection.ethz.ch/handle/20.500.11850/154610>.
- VL53L8CX: UM3109 (statuses, sigma, signal),
  <https://www.pololu.com/file/0J2030/um3109-a-guide-for-using-the-vl53l8cx-lowpower-highperformance-timeofflight-multizone-ranging-sensor-stmicroelectronics.pdf>;
  ST forum on statuses (5 valid; 6 and 9 "50 %"; 9 = a target with another just
  behind; 12 = blurred by the sharpener),
  <https://community.st.com/t5/imaging-sensors/vl53l5cx-info-about-target-status/td-p/50674>;
  sigma comes from the histogram's spread and is optimistic,
  <https://community.st.com/t5/imaging-sensors/range-sigma-of-vl53l5cx-is-too-low/m-p/132413>.
- VL53L5CX characterised for indoor robots (range-dependent bias, dark targets much
  noisier, < 10 % invalid): Caroleo, Albini, Maiolino, *Sensors* 2026,
  <https://doi.org/10.3390/s26051639>.
- 8 × 8 ToF on small robots:
  - obstacle avoidance on a Crazyflie with a 64-zone ToF at 15 Hz, no map:
    <https://arxiv.org/abs/2208.12624>;
  - Bitcraze's five-VL53L5CX prototype deck:
    <https://www.bitcraze.io/2022/12/victors-thesis-leave/>;
  - a VL53L5CX + IMU room scanner fitting planes: <https://hackaday.com/tag/mapping/>.

  I found nothing that maps the floor with the zone's cone. The cone-slice test here
  is our own.
