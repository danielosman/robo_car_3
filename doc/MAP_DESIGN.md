# The map: 9 rays per zone, best measurement wins

The map as built (Daniel's design). It replaces SURROUNDINGS_PLAN.md §3 (the cone
rule) and §4.2-§4.3 (log-odds, one frame); the layers and drivability (§4.4) are
the same. Code: `picoA/app/cell_map.c`, fed by `surroundings.c` with every ToF
frame; `m` prints it. Host test: `picoA/app/test/test_cell_map.c` (simulated rooms).

## Words

- **Zone:** one of the VL53's 8 × 8 measurements: a cone 5.6° wide and 5.6° tall
  that reports one distance, the nearest thing in it.
- **Cell:** one map pixel, a cube of 10 × 10 × 10 cm.
- **Column:** the stack of cells above one 10 × 10 cm square of floor. Three layers
  matter: **G** (ground, −7…+3 cm), **L1** (3-13 cm: what blocks the 12 cm robot),
  **L2** (13-23 cm: overhangs and margin).
- **Ray:** a straight line from the sensor. Each zone shoots 9.

## 1. Rays

Each zone shoots 9 rays, at 1/6, 1/2 and 5/6 of its width and of its height (so
every ray is 1.9° from its neighbours, inside a zone and across zones). Every ray
is as long as the zone's reading. A ray **passes** the cells it goes through and
**ends** in the cell where it stops. A zone with nothing in range passes cells up to
1 m and ends nowhere. Nothing assumes a floor: a ray that ends on the floor ends in a
G cell. A **G cell is passed only by a ray that goes out through its bottom** (over
a hole or a drop): a ray crossing its top, the air above whatever is there, says
nothing about the floor. Below G a ray stops; above L2, going up, too.

No cell is skipped between rays up to ~3 m (where two rays are 10 cm apart).

## 2. A ray's weight

    w = c(distance) × v

- **c**, closeness, a sigmoid: `c(d) = 1 / (1 + e^((d − 1.5 m) / 0.25 m))`, d from
  the sensor to the cell. ≈1 up to 0.5 m, 0.88 at 1 m, 0.5 at 1.5 m, 0.12 at 2 m,
  ≈0 at 3 m.
- **v**, the VL53's confidence in the zone's reading: 1 for status 5 (sure), 0.5
  for 6 or 9 (valid, less sure); other statuses are not used. When the shared ToF
  confidence model (TOF_MOTION_PLAN §3: sigma, signal) exists, v is that number.

## 3. What one frame says about a cell

Over the rays touching the cell in this frame:
- **H** = sum of the weights of the rays ending in it, **P** = of those passing it;
- **verdict:** occupied if H > P, else free;
- **q** (the frame's confidence) = |H − P| / max(H + P, 9).

In words, q is how much agreement this frame gives the cell, counted in
full-strength rays, out of 9. Nine sure, close rays that agree give q = 1. Weak
rays (far, or a doubtful reading) give a small q, because the 9 below never
shrinks. Frames are **not** added up.

Example: the chair cell at 1 m, 9 rays passing (c = 0.88).

| The VL53's readings | v | w | P | q | Can it erase (q ≥ 0.3)? |
|---|---|---|---|---|---|
| sure (status 5) | 1.0 | 0.88 | 7.9 | 0.88 | yes, in 3 frames |
| less sure (6 or 9) | 0.5 | 0.44 | 4.0 | 0.44 | yes, in 3 frames |
| sure, only 4 of the 9 rays reach the cell | 1.0 | 0.88 | 3.5 | 0.39 | yes, just |
| less sure, only 4 rays | 0.5 | 0.44 | 1.8 | 0.20 | no |
| any, from 2 m | — | ≤ 0.12 | ≤ 1.1 | ≤ 0.12 | no |

The same holds for hits: a weak reading can neither build an obstacle nor erase one.

## 4. What a cell keeps: its best measurement

Each cell stores a **state** (unknown / occupied / free), the **confidence** of the
measurement that set it, and a **doubt** counter (1 byte in all). With a frame's
verdict and q:

- unknown: take it;
- same verdict: confidence = max(stored, q). Agreeing data only raises it;
- different verdict, **better** (q > stored confidence): replace at once (a guess
  from far away is corrected as soon as the robot sees the cell better);
- different verdict, **good enough** (q ≥ 0.3, in practice rays from within
  ~1.3 m): doubt + 1; after 3 such frames in a row, replace. An agreeing frame
  resets doubt;
- different verdict, **weak** (q < 0.3): ignored.

This is the "activation": a new measurement may overwrite an old one only if it is
good enough, not only if it is better. So:
- **A chair leg seen from 5 cm** (occupied, confidence ≈ 1), then the chair is
  moved: seen from 1 m, 3 good frames erase it. No need to drive up to it again.
- **Standing still for 10 minutes** changes nothing it explored before: far cells
  only get weak frames.
- **Someone walking through:** occupied while there, free 3 good frames after.
- **Noise:** one odd frame never changes a confident cell.

## 5. Around the robot

The robot stands on floor and turns there: G cells within 15 cm of its centre are
occupied (floor) and L1 cells free, at full confidence. L2 cells within 25 cm are
never seen by any zone; still unknown, they are set free once (SURROUNDINGS_PLAN
§4.4).

## 6. Reading the map

A column is:
- **blocked** if L1 is occupied;
- **overhang** if L2 is occupied and L1 isn't;
- **no floor** if G is free (rays passed where the floor should be: a hole, a drop);
- **drivable** if G is occupied (floor seen) and L1 and L2 are free;
- **open** if L1 and L2 are free and G is unknown;
- **unknown** otherwise.

The print (`m`) can hide cells whose confidence is below a threshold.

## 7. Settings to tune (host tests, then the robot)

1.5 m and 0.25 m (the sigmoid), 9 (rays for full confidence), 0.3 (erase), 3
(frames of doubt), 1 m (how far "nothing in range" clears), v per status.

## 8. Known limits and open points

From the host tests (`test_cell_map.c`) and the robot. Fixes need Daniel's yes;
attempts that failed are in REWORK_CHANGELOG.md.

1. **Low obstacles at 0.4-1 m can be missed.** A zone reports the nearest point in
   its cone, and all 9 rays end at that distance. For the 5th zone row a 7 cm box
   at 0.5 m and the plain floor at 0.71 m give the same pattern (6 of 9 ray ends in
   L1, 3 in G), and the row above passes over both inside the same L1 cell. Today
   the passes win: the floor shows no false obstacles, but low obstacles are erased
   or shown too far. Robot: a 6 cm box 40 cm in front wasn't on the map; host test:
   a 7 cm box at 50 cm, 1 of 4 front columns blocked. Suggestion: use the robot's
   height above the floor (7 cm, fixed by the build) and its pitch to recognise a
   zone whose reading is where its lowest ray meets the floor; its rays end at
   floor height, and other readings are real surfaces that passes over them don't
   erase.
   **10 Oct (simulation, a 6.5 cm cup at 25-60 cm):** the cup loses the vote in
   *every* frame, at every distance (hits 13-23 % of passes), so the merge never
   sees it. ~80 % of the passes cross the cup's L1 cell *above* the cup (rows 3-4,
   counting from 1, at 7-13 cm): L1 is too tall for a low object. Beyond ~71 cm
   the 5th row's zone holds floor and object together; the simulation (nearest
   surface) then sees no cup at all, the real VL53 (strongest return) is unknown.
   Next: real data first (TELEMETRY_PLAN.md), then fixes tested on it.
2. **Under a table top at ~0.8 m:** the 4th zone row's rays all stop at the
   underside, the lower ones end in L1 with no ray passing those cells: false
   "blocked" (host test: 6 columns). Driving closer erases them.
3. **A small hole next to floor** gets a few floor hits from zones that see the
   floor beside it. A long drop (stairs) is handled: nothing beyond it is drivable.
4. **Standing still, the floor is seen in rings** at each zone row's distance (~20,
   30, 45, 70 cm); between them `:` (free, floor not seen). Driving fills them.
5. **Far walls** (beyond ~2 m) are weak and patchy: few rays per cell, low weight.
6. **Roll is not used** (the pose has pitch only); turning, the robot rolls 3-5°,
   which moves far end points a few cm up or down.
7. **A fixed 6 × 6 m window** around the origin; a moving window before driving
   (M4).
8. **Cost:** up to ~20 ms per frame (measured with the old map beside it); measure
   again with `p`.
9. **v from the shared ToF confidence model** (sigma, signal) instead of the status
   alone.
10. **The verdict rule:** occupied if H > P, q = |H − P| / max(H + P, 9). Daniel's
    first formulation, "more than half of max(rays, 9)" per verdict, was tried with
    closeness per cell and dropped with fix 1's attempt; it may fit again together
    with a fix for point 1.
11. 2.5 cm objects are ground by design (the 3 cm band): fine.
