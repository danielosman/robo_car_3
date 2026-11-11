# Line map on the PC — plan

Step 7 of [DATA_FLOW_PLAN.md](DATA_FLOW_PLAN.md). Confirmed by Daniel on 11 Nov 2026.
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
| b | Pieces: one row's hits joined into straight pieces (`pieces.ts`) | done |
| c | Merge: pieces into a band's lines (`line_map.ts`) | next |
| d | See-through: wedges remove what isn't there (`line_map.ts`) | |
| e | The reader (a boot's line maps at a snapshot) and `robot map --png --lines [--no-cells]` | |

Each substep ends with tests green, a fresh reviewer, this table updated and a commit
(no push). Step 7 changes no Pico: nothing to flash.

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
- **No spatial index.** A band's lines are a plain list.
- `robot check` keeps using the cell map. No pose correction from lines.

## The parts

| File (`pc/robot/src/maps/line_map/`) | Knows | Interface |
|---|---|---|
| `sights.ts` | ToF frames, verdicts, "beyond", the band heights | `sightsOf(frame, rays, zoneRad)` → 8 rows of 8 sights; `inBand(rows, band)` → the rows for one band's map; `heightsAt(sight, d)` |
| `pieces.ts` | how one row's hits become straight pieces | `piecesOf(row)` |
| `line_map.ts` | one band's lines: merge, see-through | `add(rows)`, `lines()` |
| `index.ts` | the store and the helper: a boot's three maps at a snapshot | `lineMapsAt(boot, n)` |

- Only `sights.ts` knows ToF data and the bands. The other two read sights alone
  and are tested with hand-made sights.
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
2. **Pieces** (`pieces.ts`). In one band, one row's hits left to right:
   - break where a zone has no hit, or two neighbours' distances differ by more than
     20 % plus 3 cm (a wall met at under 30° breaks into stubs);
   - split each run at its worst-fitting point until all fit (Ramer-Douglas-Peucker;
     limit: the larger of 3 cm, 2 × sigma, 35 % of the zone's width);
   - a piece's ends go half a zone width past its outer points; an end made by a
     split stops at the shared point;
   - a lone point becomes a stub across its zone.
3. **Merge** (`line_map.ts`). A piece joins the nearest line that both its ends lie
   near and whose seen part it touches. Otherwise it starts a new line.
   - A line keeps weighted sums of its points and is refitted from them. Close and
     still readings weigh more.
   - A line keeps a score for each 5 cm step along it. A hit raises the scores of
     the steps the piece covers.
4. **See-through** (`line_map.ts`). A sight lowers the score of each step inside its
   seen-through stretch, if the wedge is there at the heights the step was hit at.
   A wedge passing over a low box says nothing about the box.
   - Wedges meeting a line at under 30° are skipped (they slide along it).
5. **Read.** A line is drawn where its steps have a score above zero and were hit
   in at least two frames. Steps at zero are forgotten; a line with no steps goes.

## Numbers from the stored drive (boot `cc0e4e65`, 774 frames)

| Fact | Number |
|---|---|
| Obstacle points per frame (median) | 20, all in rows 1-6 |
| Neighbour pairs in a row with a depth jump | 23 % |
| Obstacle points whose zone spans more than one band | 31 % |
| Pieces per frame (3 cm limit) | about 12, median 20 cm long |
| Pieces with 1 or 2 points | about 80 % |
| The helper's rays for the whole drive | about 0.2 s |

One frame gives short, weak pieces. The map's quality comes from the merge.

## Tests

- Hand-made: a flat wall, a corner, a leg in front of a wall, a wall at a shallow
  angle, a thing that went away, beyond-range readings (no lines).
- The stored drive replayed, drawn over the cell map (`robot map --png --lines`).

## Known limits

- Odometry drifts: a wall seen again after a long drive may become two lines.
- Two lines that grow into each other are not joined into one.
- The limits (fit, match, margin, score cap) are first guesses, tuned on the stored
  drive.
