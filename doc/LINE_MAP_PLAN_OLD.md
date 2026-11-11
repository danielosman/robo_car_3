# Line map on the PC — plan (draft, 7 Nov 2026)

A map of lines, built on the PC from the robot's compact ToF frames. Not built yet;
it is step 7 of [DATA_FLOW_PLAN.md](DATA_FLOW_PLAN.md) and needs the PC cell map's
helper (step 5) for its rays. Nothing here is decided until Daniel confirms it
before step 7. The robot keeps its own cell map for driving; this map is the PC's.

## Is it doable?

Yes. Walls, furniture fronts and boxes are flat or nearly flat, and one ToF row sees
them as up to 8 points side by side. Joining those points gives lines. A lone point
gives a short line as wide as its zone. Coming closer makes the zones narrower and
the distances surer, so a close look can correct a far one.

Limits to accept:

- A zone is 5.6° wide, so a short line is as wide as the zone at that distance:

  | Distance | Zone width |
  |---|---|
  | 0.5 m | 5 cm |
  | 1 m | 10 cm |
  | 2 m | 20 cm |
  | 2.5 m | 25 cm |

  A 2 cm chair leg at 1 m becomes a 10 cm line.
- Round things (the pouf) become short broken lines.
- Lines say where things are, not what is free. Free floor is the PC's cell map's
  job (DATA_FLOW_PLAN.md); the two maps share only the frame they are drawn in.

## What comes in

Per ToF frame, a **posed_frame** (DATA_FLOW_PLAN.md, sections 5-6) and its
**rays** from the cell map helper:

- posed_frame: frame number, time, the robot's pose then (x, y, yaw, pitch), still
  or moving; per zone: distance in cm (0 nothing, 255 beyond 2.54 m), signal (capped
  at 255), sigma in mm, **kind** (sure / less sure / unsure / nothing), and how many
  targets the sensor saw. One target per zone: the first with status 5, 6 or 9.
- rays: per zone its point in the world, the heights its wedge covers there, and the
  cell map's **verdict** (floor / obstacle / unsure). The line map does no geometry
  of its own: the helper runs the robot's C code, so both maps judge a reading the
  same way.

## Part 1: points

- Each zone of kind sure or less sure gives one point (from the helper's rays).
- By the verdict:
  - **floor**: not used for lines;
  - **obstacle**: a point for lines;
  - **unsure**, and zones of kind unsure or nothing: ignored (but they still clear,
    part 4).
- Each point keeps its uncertainty: across the ray, half a zone (d × tan 2.8°);
  along the ray, the sigma (at least 1 cm, the distance's step).

## Part 2: lines from one frame

- Work row by row, left to right, over the obstacle points.
- Grow a line while the next point fits it: its distance from the line is under a
  limit (for example the larger of 3 cm and 2 × sigma), and there is no jump in
  depth between the two zones.
- When the next point doesn't fit, end the line and start a new one.
- A line through several points goes through their centres. Its two ends go out by
  half a zone, to the zone borders: the end zones saw the surface across their whole
  width.
- A point that fits no neighbour becomes a short line across its zone, at right
  angles to the ray, from border to border (the 5.6° borders).
- Each end is marked:
  - **real end**: the next zone reads farther or sees nothing, so the object ends
    there (within half a zone);
  - **open end**: the edge of the view (column 1 or 8), or the next zone is unsure.
    The thing may go on.
- The same wall seen by several rows gives nearly the same line on the floor plan.
  Lines from different rows that match are joined into one, keeping the lowest and
  highest point seen.

## Rows and height bands

All 8 rows are used (the cell map uses all 64 zones too). Rows are the unit for
**fitting** lines in one frame, but not for **keeping** them:

- A row is a cone, not a level slice: row 3 sees a wall 16 cm up at 0.5 m and 39 cm
  up at 2 m. A map per row would hold the same wall in different rows as the robot
  drives, never merged, and its meaning would change with distance and pitch.
- So each line goes into a **height band** by the heights its zones cover (the
  whole zone, not only its centre ray: at 2 m a zone is ~20 cm tall), the cell
  map's bands:

  | Band | Height | Means |
  |---|---|---|
  | L1 | 3-13 cm | blocks the robot |
  | L2 | 13-23 cm | an overhang; drive stops for it too, as in the cell map |
  | above | over 23 cm | the robot passes under (table tops); walls seen high |

  A line seen across bands goes into each band it reaches.
- Merging and removing happen within a band. The page draws each band in its own
  colour, each can be hidden: the "stacked, different colours" view, but by height.

## Part 3: into the map

The map is a list of lines in a quadtree (each line by its box, padded by its
uncertainty). For each new line:

- **Find**: look up the lines near it in the quadtree.
- **Match**: same direction (within ~10°), close enough sideways (within the two
  lines' uncertainties together), and overlapping or touching along their length.
- **Merge** a match:
  - position and direction: an average weighted by sureness. A close reading counts
    much more than a far one; a reading while still counts more than one while
    moving.
  - ends: an open end only grows the line; a real end replaces the old end if it was
    seen from closer (a smaller zone width there).
  - the line remembers how often it was seen, when last, and the closest distance it
    was seen from.
- **No match**: a new line, shown faint until seen again (or seen twice in one still
  period).

This is how "moving closer corrects the lines": a far line is wide and loose; the
close look is narrow and sure, and wins.

## Part 4: removing what isn't there

Lines must also go away, all or in part (a moved chair, a person, a far line drawn
too long). A zone doesn't see along a thin ray: it sees a 5.6° wedge.

- **What a zone clears:** the part of a line inside its wedge, closer than its
  reading (by more than the uncertainties), and at heights the wedge passed through
  (a reading passing over a low box to the wall behind says nothing about the box).
  A zone with nothing in range clears the same way, up to a set distance (the cell
  map uses 1 m).
- **Only that part:** a line keeps a short list of stretches along it that zones
  saw through. The wedge's two edges cut the line at two points; the stretch between
  them gets a "miss".
- **Not at once:** a stretch is cut out only after misses from more than one frame
  (or one still, close look) outweigh the hits there. Then the line gets shorter,
  splits in two, or goes when nothing is left.
- **Shallow angles don't count:** a wedge meeting a line at under ~30° slides along
  it (TODO.md: the wall cleared by a scan).
- Cost: cutting a line by a wedge is two line crossings; the quadtree gives the few
  lines near each wedge. Not much more than any removal needs.

## Part 5: still periods

The robot stands still 200 ms after sending starts, before it moves, and keeps
sending 200 ms after it stops (both settings). The frames while still are the best: no blur, sure pose.

- First version: every frame counts on its own, still frames with a higher weight.
- Later, if blips show up: take the median per zone over a still period before step
  1.

## The quadtree

Asked by: merge (lines near a new line), remove (lines crossed by a ray), drive
check (lines along a path, as `robot check` does now), drawing (lines in view).

A few thousand lines for a flat: any index is fast enough. `rbush` (an R-tree, in
doc/inspiration/) does the same job ready-made; a quadtree of our own is also small.
To decide when it is built.

## How we test it

- **On stored drives:** the new database keeps every posed frame, so a drive can be
  run through the line map again after each change, without the robot. (The old
  recordings are not used: DATA_FLOW_PLAN.md, "Old data".)
- **Unit tests** on hand-made frames: a flat wall, a corner, a leg in front of a
  wall, a wall seen at a shallow angle.
- **Compare** with the PC's cell map of the same frames, drawn under it.

## Order of work

1. Parts 1-2 (points, lines from one frame); drawn per frame on the page.
2. Part 3 (merge, bands) with the quadtree; a whole drive drawn.
3. Part 4 (removing).
4. Live: the same code on the stream from the robot.
5. Later, if wanted: drive check (`robot check`) on lines instead of cells.

## Open questions

1. **Limits** (fit distance, angle for a match, weights): first guesses above, to be
   tuned on stored drives.
2. **The pose drifts.** Merging trusts odometry; a wall seen twice after a long drive
   may become two lines. Fixing that (matching lines to correct the pose) is a later
   step, not in this plan.
