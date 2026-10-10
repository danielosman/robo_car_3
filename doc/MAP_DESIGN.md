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

## 9. The READINGS variant (10 Oct, from the recordings)

**Why.** The recordings (TELEMETRY_PLAN §3) show the cup in every scan, seen only by
row 5 at 40-48 cm, and replaying the four scans through this map (§4, "VOTES")
shows why it is lost (§8.1): in the cup's L1 cell, row 5's hits are outvoted by
rays of rows 3-4 passing over the cup inside the same 10 cm-tall cell, and a third
of row 5's rays end below 3 cm, in G, where they count as floor. Whether a cell
wins depends on where the cup stands in the grid: one cell in scans 3-4, none in
scans 1-2. Row 5 never returned bare floor closer than 1 m.

**The rules** (`cell_map_set_variant(CELL_MAP_READINGS, margin)`; Daniel, 10 Oct):
1. **A reading is judged as a whole**, by where its zone's cone first meets the
   floor (its lower edge, with the robot's pitch: row 8 ~18 cm, row 7 ~24, row 6
   ~34, row 5 ~63-71, rows 1-4 never):
   - **closer than `margin` (0.9) × that: an obstacle.** No floor can be there. All
     9 rays' hits count, in L1 or above: rays ending below 3 cm are lifted to L1.
   - **otherwise all rays ending below 3 cm: floor** (as in §3; beyond the floor
     patch's far end, a drop, as before);
   - **otherwise unsure** (straddling 3 cm: grazing the floor, or a low thing on
     it): its rays ending below 3 cm mark the floor, the others nothing.
2. **A ray only clears what it would have hit:** a pass counts against a cell
   only if the ray passed it at or below the highest point hit there (this frame,
   or kept while the cell is occupied) + 1 cm. Rays over a low thing don't erase it.
3. Everything else as §3-§6 (the vote per frame, the merge, around the robot).

**Host tests** (`test_cell_map`, both variants): READINGS passes every room; the
7 cm box at 50 cm (§8.1) is 4 of 4 blocked (VOTES 1 of 4); a chair leg moved away
and a walker are gone 1 frame after.

**Replay** (`tools/replay.sh <take>`, 10 Oct scans; cells ≥ 10 % confident):

| | VOTES | READINGS | READINGS, "lowest hit" (tried) |
|---|---|---|---|
| Cup, scans 1-2 (left, 47-57 cm) | not on the map | 2 cells | 2 cells |
| Cup, scans 3-4 (right, 45 cm) | 1 cell | 1 cell | 1 cell |
| Box (left, 35-45 cm) | partly | +1-2 cells | +2-3 cells |
| Cupboard behind (45-65 cm) | not on the map | not on the map | 3-4 cells |
| Chair base (right, 35-85 cm; Daniel's office chair on wheels) | — | 2-3 cells | 3-6 cells |
| Chair seat area (right, ~1 m) | 2 cells, close votes (48 vs 40) | freed (43 vs 40) | freed |
| 1-1.6 m behind-left | some blocked | some freed | many new overhangs |
| Simulated chair leg moved away | 1 frame | 1 frame | 30 frames (fails) |

"Lowest hit" (rule 2 against the lowest point hit) keeps the cupboard but lets
almost nothing clear a cell hit near the floor: moved things stay ~2 s.

The room (Daniel): the robot started 30 cm further left from scan 3 on (the cup
moves from 19° left to 30° right in the data: it stayed, the robot moved); the
cupboard behind it, later behind-right; an office chair on wheels to the right,
a wall behind the chair. The wall is hidden by the chair: at ~90 cm right rows
1-4 hit the chair's seat and back (11-52 cm high). Row 5's readings beyond ~1 m
land 1-8 cm *below* the floor: reflections in the waxed floor, which both
variants turn into "no floor" (`?`) beyond 1 m (an older limit, §8.3).

**The robot runs READINGS since 10 Oct** (`surroundings.c`). Robot test (10 Oct,
take `0b9377b7-1`): the cup, the box, the chair base and the cupboard on the map;
the replay of the take gives the same blocked cells as the robot printed (441 of
441 within 1 m; whole map 1555 of 1560, the rest from frames before the take).
The `?` beyond ~1 m stay (Daniel, 10 Oct): the floor there isn't sure either way.
**Open:** the cupboard was missing in the earlier scans (both variants).

**Blips (10 Oct, Daniel saw one in the viewer):** an overhang cell 67 cm behind
the robot where nothing stands: one target in one frame (row 3, 61 cm, signal 11,
status 5; the light in that direction was normal). The map takes a cell's first
verdict, so one echo makes it. Tried on the five scans, not kept: (a) occupied
only from a second frame on: the blip goes, but so does a chair-base cell hit in
22 frames that won the vote once; (b) readings under 1 m with signal below 20
ignored (17 of 45 one-frame cells have no stronger hit, none of the 119 cells hit
in 4+ frames): the blip goes, but so do chair-base cells at 60-65 cm in two other
scans (the dark base echoes weakly too). Kept as it is: a blip is rare (one in the
last scan), and the next rays through its cell clear it. Revisit with recordings
of driving.
