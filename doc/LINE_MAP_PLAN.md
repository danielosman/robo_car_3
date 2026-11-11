# Line map on the PC — plan

Step 7 of [DATA_FLOW_PLAN.md](DATA_FLOW_PLAN.md). Confirmed by Daniel on 11 Nov 2026;
the way lines are made changed while building (see "Changed while building").
The earlier draft is [LINE_MAP_PLAN_OLD.md](LINE_MAP_PLAN_OLD.md) (not built).

## In short

The PC turns the robot's stored ToF frames into lines: walls, furniture fronts, boxes.
It keeps three separate maps, one per height band, and builds them the way the cell
map is built: from the frames stored before the step you pick. `robot map --png
--lines` draws them; the page shows them in step 8.

## Where we are

| Substep | What | State |
|---|---|---|
| a | Sights: a frame's zones as flat wedges (`sights.ts`); the helper gives the sensor's position | done |
| c + d | Spots and see-through (`line_map.ts`); lines through the spots (`straight.ts`) | done |
| e | The reader (a boot's line maps at a snapshot) and `robot map --png --lines [--no-cells]` | done |

Each substep was reviewed by a fresh reviewer and committed (not pushed); the history
is in [CHANGELOG.md](CHANGELOG.md), 11 Nov. Step 7 changes no Pico: nothing to flash.

**Left:**
- Daniel's yes to the changed way of making lines (next section).
- Tuning the limits on a drive where the robot really drives.
- The page shows the lines in step 8 (DATA_FLOW_PLAN.md).

## Decisions (Daniel)

- **Three independent maps**, one per band. A wall is stored up to three times.

  | Band | Height | Means |
  |---|---|---|
  | L1 | 3-13 cm | blocks the robot |
  | L2 | 13-23 cm | an overhang |
  | above | over 23 cm | the robot passes under |

- **Wedges, not rays.** A zone is one flat triangle on the floor plan: the sensor at
  its tip, one zone angle (5.6°) wide.
- **Built from the store at a snapshot**, like the cell map. The slider works the
  same. No per-frame page view, no update while the robot drives.
- **`robot map --png`** draws the cell map, as now. `--lines` adds the lines.
  `--lines --no-cells` draws only the lines. `--no-cells` alone is an error.
- **No spatial index** beyond the map's own 5 cm squares.
- `robot check` keeps using the cell map. No pose correction from lines.

## Changed while building (for Daniel to confirm)

The confirmed plan made lines from each frame (pieces) and merged them into the
map's lines. On the stored drive that gave a tangle: about 500 crossing lines in the
"above" band for a handful of real surfaces. One frame's pieces are too short (80 %
have 1 or 2 points) to tell a direction, and each wrong direction started a new line.

Now the map keeps the **hit points** (in 5 cm spots) and makes lines **when read**,
through all the points together. The same drive gives 58 lines in "above", straight,
where the surfaces are. So:

- Proximity clustering: used, to group nearby spots before fitting.
- Ramer-Douglas-Peucker: not used any more. It needs points in order along a
  shape; a cloud of points from many frames has no order.
- `pieces.ts` (substep b) is removed.

What was tried is in the CHANGELOG. Once Daniel has said yes, this section goes.

## The parts

| File (`pc/robot/src/maps/line_map/`) | Knows | Interface |
|---|---|---|
| `sights.ts` | ToF frames, verdicts, "beyond", the band heights | `sightsOf(frame, rays, zoneRad)` → the 64 zones' sights; `inBand(sights, band)` → the sights for one band's map; `heightsAt(sight, d)` |
| `line_map.ts` | one band's spots: hits, see-through, groups | `add(sights)` (one call a frame), `lines()` |
| `straight.ts` | how a group of points becomes straight lines | `straightLines(points)` |
| `index.ts` | the store and the helper: a boot's three maps at a snapshot | `lineMapsAt(store, cellMaps, boot, n)` → each band's lines |

- Only `sights.ts` knows ToF data and the bands. `line_map.ts` reads sights alone;
  `straight.ts` reads points alone. Both are tested with hand-made input.
- The helper (`cell_map_helper.c`, request `R`) also returns the sensor's position.
  No C geometry is copied into TypeScript.

## How a map is built

A **sight** is one zone's wedge: the sensor's position, the wedge's direction and
half width, how far it saw through, how high it is along the way, its weight, and a
hit at its end or none.

1. **Sights** (`sights.ts`). For each zone with a valid reading:
   - the seen-through stretch ends a margin before the reading (the larger of 5 cm,
     2 × sigma, one zone width there), so a wall seen at an angle doesn't remove
     itself;
   - a **hit** only if the verdict is obstacle and the reading is not "beyond"; it
     keeps the heights the zone covers there;
   - floor, unsure and beyond readings give no hit but still see through;
   - zones with no valid reading give nothing (24 "nothing" zones in 49,536);
   - a still, sure reading weighs 1; moving halves it, less sure halves it;
   - **per band:** a hit goes to each band its heights reach, cut to that band.
     What a sight saw through is the same for all three maps.
2. **Spots** (`line_map.ts`). A hit goes into the 5 cm square it lies in. The spot keeps:
   - where its hits lie: their weighted middle (a close hit weighs more);
   - a score: each hit raises it by the sight's weight, up to 5;
   - in how many frames it was hit, and the heights it was hit at.
3. **See-through** (`line_map.ts`). A sight lowers the score of each spot inside its
   wedge and closer than it saw through, if the wedge is there at the heights the
   spot was hit at. A wedge passing over a low box says nothing about the box.
   - Only the wedge's middle half counts. A wedge sliding along a wall would
     otherwise remove the wall under its edge.
   - A spot at zero is forgotten.
4. **Lines, when read** (`line_map.ts`, `straight.ts`). Spots hit in 3 frames or more
   stand.
   - Standing spots at most 3 squares apart form a group (proximity clustering).
   - In a group: try every direction in 3° steps; find the 6 cm wide strip with the
     most points; keep its longest run with no gap over 15 cm; fit a line through
     that run (and again, while that gains points). Take those points out and
     repeat, until fewer than 4 are in a row.
   - Points on no such line (clutter, a lone leg) give no line.

## Numbers from the stored drive (boot `cc0e4e65`, 774 frames)

| Fact | Number |
|---|---|
| Obstacle points per frame (median) | 20, all in rows 1-6 |
| Frames taken while moving | 87 % |
| Obstacle points whose zone spans more than one band | 31 % |
| Hits closer than 1.2 m | 35 %; they lie within about 5 cm of each other |
| Hits farther than 1.2 m | 65 %; smeared 20-30 cm along the ray |
| Lines per band (L1 / L2 / above) | 23 / 29 / 58, median 34 cm long |
| Building the three maps for the whole drive | under 1 s (0.9 s the first read, 0.4 s after) |

## Tests

- `straight.ts`: a wall, a corner, two walls side by side, a doorway, clutter.
- `line_map.ts`: a wall seen three times; a close look against a far one; a thing
  that went away; a sight passing over; a wedge's edge.
- `index.ts`: through the real store and helper: a wall in the stored frames is a
  line in each band; none before its frames; the picture with the lines over the
  cells, and alone.
- By hand, not a test: the stored drive drawn over the cell map
  (`robot map --png --lines`).

## Known limits

- **Far things are thick.** Beyond about 1.2 m the hits smear, so a far wall shows
  as two or three lines side by side until the robot comes closer.
- **Thin things give no line**: a lone chair leg is 1 or 2 spots; a line needs 4.
- **A robot standing still forgets slowly.** Only the middle half of each wedge
  sees through, so from one pose half the view removes nothing. A thing that went
  away goes from the map fully once the robot moves or turns.
- **Heights are the wedge's, not the thing's.** A 5 cm box hit by a wedge covering
  3-13 cm is kept as 3-13 cm, so "passing over says nothing" holds only roughly.
- A wall met at a very shallow angle (calculated: under about 13°) can still remove
  itself. Not measured.
- Odometry drifts: a wall seen again after a long drive may become two lines.
- The limits (strip width, gap, frames to stand, score cap) are first guesses,
  tuned on one stored drive in which the robot mostly turned on the spot.
- The maps are built again on every read (the time is in the numbers above): fine
  for `robot map`; step 8 keeps the last maps read only if the slider feels slow.
