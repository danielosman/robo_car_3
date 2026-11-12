# Outline map and drift correction — plan (11 Nov 2026)

Agreed in general by Daniel on 11 Nov; steps S (the simulator), A (the outline) and
B (the walls) are built. It follows the line map
([LINE_MAP_PLAN.md](LINE_MAP_PLAN.md), step 7 of [DATA_FLOW_PLAN.md](DATA_FLOW_PLAN.md)),
which Daniel looked at and found is not what he wants: loose lines, too few of them.

Contents: In short · For the fresh session · What Daniel asked for · The three ideas ·
What is stored · How it fits the code · The simulator · Steps · Decided · Open ·
Alternatives for later.

## In short

The map is a **closed outline** of the floor the robot has seen free, one per height
band. It starts as a 35 cm circle, grows as the sensor sees further, and is always
closed: where nothing was seen yet, its edge is a guess, drawn faint. Long straight
edges are remembered as **walls**, kept exact, and used to correct the robot's
**drift**. Every wall and every pose carries how sure we are of it; when they
disagree, the less sure one gives way.

## For the fresh session

- **Steps S, then A to E in order.** The table in "Steps" says which are done:
  start at the first that is not. **Next: step C**; read "Step C: where to start"
  first.
- **Read first:** this file; AGENTS.md; DATA_FLOW_PLAN.md sections 1-3; then
  `pc/robot/src/maps/outline/` and `pc/robot/src/sim/` with their tests.
- **Rules as in DATA_FLOW_PLAN.md section 2:** tests first where they make sense, a
  fresh reviewer after each step, this file's table updated, history in the
  CHANGELOG. Commit (no push) after each step, once its tests pass and the
  reviewer agrees, as in step 7.
- **The three ideas below are one plan**, not a choice: the outline (1) gives the
  walls (2), the walls correct the pose (3).

## What Daniel asked for

- One closed polygon around the robot, not a set of lines. Three polygons, one per
  band (L1, L2, above).
- It starts as a 35 cm circle and is updated as information comes in. Rough at
  first; better with time.
- It grows through doors: one polygon for all the robot has seen, no rooms.
- Things standing in the free floor are separate closed shapes (islands).
- An edge seen again, from closer or from another angle, corrects the old edge. No
  crossing duplicates.
- Each edge has a confidence, shown by colour or brightness.
- Exact only where it pays: long straight edges (walls). Jagged clutter stays rough.
- The edges are for correcting pose drift. Where each edge was seen from is stored
  in the database.
- An edge no longer seen is removed, but only when it is sure the robot has not
  drifted instead: more than one piece of evidence.

## The three ideas

### 1. The outline is the border of the seen-free floor

- Each wedge of a ToF frame says "free up to here". All wedges together, plus the
  start circle, are the seen-free floor.
- Its border is closed by nature. It needs no rule for joining lines.
- A thing standing in free floor is a hole in it: an island, with its own closed
  border.
- Each stretch of the border is one of three kinds:

  | Kind | What is behind it | Drawn |
  |---|---|---|
  | wall | hits, long and straight | strong; brighter when surer |
  | rough | hits, short or jagged (clutter, far smears) | medium, simplified coarsely |
  | guess | nothing seen yet (the view ended there) | faint |

- At the start the whole border is the 35 cm circle, all guess. After a full turn
  it is the room as far as the sensor reaches (2.5 m), with guess edges where it
  saw no end.

### 2. Walls are remembered, with how sure we are

- A wall is a long straight stretch of border backed by hits (first guess: 40 cm
  or more, hits within 3 cm of a line).
- It gets a number and is kept: its line, its two ends, how sure its position and
  direction are, and every sighting (which frame, from what pose, how far off).
- A new sighting of the same wall moves it a little: much if the wall is still
  unsure, little if it is well known. Close and still sightings count more.
- Rough and guess edges are not remembered as things. They are drawn again from the
  seen-free floor each time.

### 3. Drift: the pose and the walls correct each other

This is the stochastic part. In plain words:

- **The pose has a "how sure".** It gets less sure with every turn and every
  centimetre driven (odometry drifts; a 30° turn is about 31°). It gets surer
  after it was checked against walls.
- **Each frame is first checked against the walls.** Its hits should lie on the
  walls it sees. If they lie a little off, one small shift and turn of the robot is
  looked for that puts them back on.
- **Who is blamed** follows from the two "how sure" numbers. An unsure pose against
  a sure wall: the pose moves. A sure pose against a new, unsure wall: the wall
  moves. (This weighing is a Kalman filter; nothing fancier is planned.)
- **One wall is not enough to shift the robot.** One wall corrects the turn and the
  distance to that wall, not the position along it. A full correction needs two
  walls in different directions.
- **Drift or a moved thing?** If one shift of the robot explains all walls in view,
  it is drift. If the other walls fit and one doesn't, that thing moved.
- **Removal.** Seeing through a wall counts only while the pose is well fixed (it
  was just checked against at least two other walls). Otherwise the frame says
  nothing about that wall.
- Hits far off any wall (more than the two "how sure" numbers allow) are not used
  for correction. They are new things.

Limits to accept:

- The sensor sees 45° at once, often one wall. So the turn is corrected often, the
  position less often. Looking round in 30° steps helps.
- A correction fixes the pose from then on. Earlier frames stay where they were
  put (no going back over the whole drive). To be seen whether that is enough.
- Clutter and far smears never correct the pose. Only walls do.

## What is stored (DuckDB)

| Table | Holds |
|---|---|
| `walls` | per boot and band: number, middle, direction, length, how sure (spread across, of the direction), sightings, first and last frame, the snapshot it was built at |
| `wall_sightings` | band, wall, frame, the pose it was seen from and how sure, the hits (JSON), how far off they were (across, slope) |
| `frame_poses` | frame, the odometry pose, the placed pose (the same until step C), how sure |

- Written by `robot walls [n]`: the boot's rows are replaced each time.
- Which walls a frame was checked against: `wall_sightings` by frame.
- The raw frames stay as they are. Everything above can be built again from them.

## How it fits the code

Built (steps A and B), in `pc/robot/src/maps/outline/`:

| File | Knows | Interface |
|---|---|---|
| `sights.ts` | ToF frames, verdicts, "beyond", the band heights | `sightsOf()`, `inBand()`, `heightsAt()` |
| `mapper.ts` | the object on top: per band a map and its walls; the pose and how sure | `new Mapper()`, `add(frame, sights)`, `outlines()`, `walls()`, `sightings()`, `poses()` |
| `outline_map.ts` | one band's 5 cm squares: seen free, hits (spots), see-through; which border point lies on which spot, or on which wall | `new OutlineMap(band)`, `add(sights)`, `outline(walls)` |
| `border.ts` | the closed loops round a set of squares | `connected()`, `bordersOf()` |
| `shape.ts` | a loop's points as wall, rough and guess edges; edges on a known wall carry its number; no knots | `edgesOf(points)` |
| `walls.ts` | one band's walls: their line and how sure, moved by each frame's hits (a Kalman filter), new ones from wall edges, merges | `new Walls()`, `see()`, `found()`, `on()`, `doubtOf()`, `sureOf()` |
| `pose.ts` | how sure the pose is: grows with turning and driving | `grown()`, `lineDoubt()` |
| `colours.ts` | a colour a band, a brightness an edge (walls by how sure) | `edgeRgb()`, `outlineStrokes()` |
| `index.ts` | the store and the helper: a boot's outlines and walls at a snapshot; the tables | `outlinesAt()`, `wallsAt()` |

- `robot map --png --lines [--no-cells]` draws the outlines; `robot walls [n]` lists
  the walls and writes the three tables.
- **New in C and D:** the pose check, removal.
- The cell map is untouched. `robot check` keeps using it.
- PC only. The robot does not get the corrected pose in this plan.

## The simulator (step S)

A small program that tests the ideas before the robot is needed: a made-up room, a
made-up robot driving through it with a made-up ToF and a drifting odometry. It
knows the truth, so every step can be measured, not only looked at.

**What it is**
- `pc/robot/src/sim/`: plain TypeScript, no robot, no store, no helper.
- It produces posed frames and their rays, as the store and the helper give them.
  The real `sightsOf()` turns them into sights. So the map code under test is the
  real code, not a copy.
- `npm run sim -- <scenario>` writes a folder with one picture per step and an
  `index.html` with a slider over them. No server, no new tools.
- The same scenarios run as tests (`npm test`) with numbers checked.

**The room**
- Flat shapes on the floor plan: an outer wall (a polygon, may have doors and
  several rooms) and things inside (boxes, legs, a pouf as a many-sided shape).
- Each shape has a height span (from, to), so the three bands differ: a box 10 cm
  high is in L1 only; a table top is in "above" only.
- A thing can be moved or taken away at a given step (for step D).

**The robot**
- A list of moves like the real ones: `r30 d50 ...`, with a still time before and
  after each move, frames at 10 Hz.
- Two poses: the **true** one (where it is in the room) and the **odometry** one
  (what the frames carry). The map code sees only the odometry pose.
- Drift, as measured on the robot and adjustable: a turn comes out about 3 % too
  large (30° → 31°); a small error per distance driven; a little random wander.

**The ToF**
- 8 × 8 wedges of 5.625°, the sensor 9 cm up, as mounted. A wedge reads what its
  middle ray meets at the wedge's heights (a thing beside that ray if it met
  nothing), the floor, or "beyond" past 2.54 m.
- Floor, obstacle or unsure is the robot's own rule (`tof_rays.c`), copied; a test
  holds the copy against the real helper.
- Noise, from what was measured on boot `cc0e4e65`, each switchable:
  - 1 cm steps of the distance;
  - close and still: within a few cm;
  - beyond about 1.2 m, or while moving: smeared 20-30 cm along the wedge;
  - a wedge that covers two depths (an edge) reads somewhere between them;
  - a few readings missing (unsure).
- With all noise off the outline must match the room almost exactly: that is the
  first test.

**What is drawn** (one picture per step)
- The room as it truly is (grey), the true robot, the odometry robot, and from
  step C the corrected robot.
- The outline per band: wall, rough and guess edges in their colours and
  brightness; islands.

**What is measured**

| Number | Means | Used from |
|---|---|---|
| closed | every outline is a closed loop, no crossings | A |
| area error | seen-free floor against the true free floor the robot could see | A |
| wall error | distance and angle of each remembered wall from the true wall | B |
| duplicates | walls found per true wall (should be 1) | B |
| pose error | corrected pose against true pose, and odometry against true | C |
| wrong removals | walls removed that are still there; moved things still drawn | D |

**Scenarios** (first set)
1. An empty square room, the robot turns 360° in 30° steps in the middle.
2. The same room with a box and a table: islands, bands.
3. Two rooms and a door: the robot drives through; the outline grows.
4. A loop: the robot drives round a room and comes back to its start, with drift.
   Step C must bring the pose error at the end clearly under the odometry's.
5. A box is taken away half way; and the same drive with drift but no change:
   the box must go in the first, the walls must stay in both.
6. A big room (walls beyond 2.5 m): guess edges at the sensor's reach.

**As built** (`pc/robot/src/sim/`, `npm run sim` lists the scenarios)

| File | Knows |
|---|---|
| `room.ts` | shapes with heights and steps; a ray's way through them |
| `drive.ts` | the moves, the two poses, the drift, the seeded random numbers |
| `tof.ts` | one frame: what each zone reads, the noise; gives a posed frame and its rays, as the store and the helper do |
| `scenarios.ts` | the scenarios; `run()` gives each move's frames, sights and truth |
| `measure.ts` | the numbers (so far: pose error, distance from the true faces, closed, the floor's error, each wall against its true face and walls per face) |
| `play.ts` | a scenario through the mapper: what it drew and its walls, step by step |
| `picture.ts`, `main.ts` | the pictures, the slider page, the command |

- `play.ts` is the one place that says which map code runs. `main.ts` draws what it
  gives; the tests measure it. Each step swaps the map there.
- A picture has three panels: L1, L2, above. Each shows the room at that band's
  heights.
- Scenarios: `square`, `things`, `door`, `loop`, `box-removed`, `box-stays`, `big`.
  `--no-noise` and `--no-drift` switch those off. Pictures go to
  `pc/robot/recordings/sim/<scenario>/`.
- With the measured drift (3 % per turn) a full turn ends 11° off, and the `door`
  drive ends 58 cm off. That is what step C has to take out.

**Limits of the simulator**
- It is only as true as its noise. Limits tuned on it are first guesses; the
  stored drive and later a real drive have the last word.
- It does not go through the robot's C geometry or the store. The real reader
  (`outlinesAt`) is tested on stored frames as before.

## Steps

Each step ends with something to look at. S comes first; A to D are each built and
tried in the simulator first, then on the stored drive.

| Step | State | What | To look at |
|---|---|---|---|
| S | done | The simulator: room, robot with drift, ToF with noise, pictures with a slider, the numbers | `npm run sim -- <scenario>`; the scenarios as tests |
| A | done | The outline: seen-free floor per band, its border and islands, simplified; kinds wall / rough / guess by rule, nothing remembered yet | `robot map --png --lines`: a closed polygon from the 35 cm circle on, step by step with the slider's snapshots |
| B | done | Walls remembered: numbers, how sure, sightings; colour by confidence; the three tables | the same picture, walls brighter the surer they are; `robot walls` listing them |
| C | | The pose check: frames checked against walls, corrected poses stored, the map built with them | the same drive drawn with and without correction; the error on returning to the start |
| D | | Removal with the two-wall rule | a moved box goes; a wall stays when the robot drifts |
| E | | The page: the outline per band, switchable (the old step 8) | the page |

How the outline is made (step A, as built):

- A square is **seen free** in a band when a wedge passed through it and covered
  the band's lower part there (things stand on the floor). Closer than 45 cm any
  wedge counts.
- A square hit in 3 frames **stands**: it is not free, whatever saw through it.
  Hits are forgotten by see-through as in the line map.
- The outline is the border of the free squares joined to the start. Each border
  point is moved onto the nearest standing spot up to 30 cm beyond it (sights
  end a margin before what they hit).
- Edges between hits: straight for 40 cm or more, hits within 3 cm: **wall**.
  Otherwise **rough**. No hit, or hits more than 30 cm apart: **guess**.
- Where a loop cuts through itself it is cut at the crossing point; the shorter
  way round (in metres) goes. Loops crossing each other are not prevented; the
  simulator finds none.
- The flag stays `--lines` and the files `-lines.png`, on purpose (the plan's name
  for the picture).

What the outline does and does not show (measured in the simulator):

| Fact | Number |
|---|---|
| Without noise: walls off the true walls | under 1 cm |
| Without noise: floor drawn that isn't there | 0 |
| With noise, no drift: floor the robot saw that is inside the outline | over 80 % |
| Loops open, or crossing, in all 7 scenarios at every step | 0 |
| The stored drive (boot `cc0e4e65`, 774 frames): built in | 1.1 s |

- **L1's floor is known to about 1.65 m only.** Only row 5 looks along the floor.
  Farther walls are hit but stay outside L1's outline until the robot is closer.
- **Far walls are rough, not walls.** Beyond 1.2 m the hits smear, so no stretch is
  straight to 3 cm. In the `square` room (walls 1.5 m away) there is no wall edge
  with noise on. Step B has to find walls there another way (a line through many
  smeared hits), or step C has nothing to correct against.
- **A thing never looked at is a guess island** (the pouf in `loop`).
- **A table top is seen from below:** its far edge shows as a thin shape, not as
  an island, until the robot has seen round it.
- On the stored drive the outlines are closed and follow the cell map's floor;
  nearly all edges are rough or guess (the robot turned on the spot, things are far).

How the walls are made (step B, as built):

- **Walls are their own store per band** (`walls.ts`), fed every frame by the
  mapper. A wall is a line: its middle, its direction, half its length, and a 2 × 2
  covariance (its offset across at the middle, its direction).
- **A new wall** comes from an outline edge of kind wall on no known wall, looked for
  every 10 frames. Its doubt: 5 cm of its own plus the pose's.
- **Two wall rules** in `shape.ts`: straight for 40 cm or more with hits within 3 cm
  (close walls); or 1 m or more whose points fit a line to 5 cm (far walls, whose hits
  smear), its ends trimmed to points on that line (else it reaches round a corner).
- **Each frame's hits on a wall** (in the band, the ray meeting it at 30° or more,
  within 3 spreads and at most 15 cm, up to 20 cm past its ends, at least 3) give one
  measurement: how far their middle lies off it, and their slope if they span 20 cm.
  A hit's spread: 2 cm closer than 1.2 m and still, else 8 cm, along the ray; the
  wedge's width across it. The pose's doubt is added. A Kalman step moves the wall
  by its share; the hits beyond its ends lengthen it.
- **Walls that grow into each other are one** (within 5 cm and 5°, gap at most
  20 cm): the surer line, both lengths, the older number. Sightings are listed by
  the number it has now.
- **How sure a wall is** (`doubtOf`): the filter's own doubt plus the surest pose it
  was seen from. The filter alone takes each frame's pose error as new, but frames
  from one place share it, so it over-trusts. A floor inside the filter instead made
  walls swing (up to 26 cm off).
- **The pose's doubt** grows: 3.5 % of each turn and 2° a metre in heading; 1 cm a
  metre, 1 cm a radian turned and the heading's doubt times the distance in position.
  It never shrinks in step B.
- **Drawing:** border points within 12 cm of a wall are put on its line; a run of
  them is one wall edge with the wall's number and how sure it is. Brightness: wall
  0.7 (unsure) to 1 (sure to 1 cm), rough 0.55, guess 0.3.

What the walls do (measured in the simulator, all noise on):

| Fact | Number |
|---|---|
| No noise, no drift: each of the room's walls found once (square, loop, box-stays) | within 1 cm and 1° |
| Noise, no drift, band above: each wall found once (square, things, loop, box-stays) | within 5 cm and 2° |
| Noise, no drift, L1 and L2 | most walls found; up to 13 cm off; some found twice |
| With drift | walls follow the drifted pose: up to 23 cm and 10° off; in `door` half the walls are on no true wall |
| The stored drive (boot `cc0e4e65`), built and tables written | 2 s; 13 walls, most unsure (the drive turned a lot) |

- **Walls seen later are less sure:** the pose's doubt only grows in B. In `square`
  without drift the four walls read 0.94, 0.71, 0.54, 0.33.
- **L1's walls are few and rough:** its floor is known to 1.65 m only.

### Step C: where to start

Not built. From the sessions that built B and reviewed it:

- **Split the wall update first:** `update()` in `walls.ts` both makes the
  measurement and moves the wall. C needs `measure(wall, hits)` (z, H, the hits'
  own noise, the pose's part) and then one step that shares the change between wall
  and pose by their doubts.
- **One correction per frame from all its walls:** stack each wall's measurement;
  one wall gives the turn and the distance across it, two at an angle give the
  whole pose. The pose's doubt shrinks after it.
- **Place the frame's sights by the corrected pose** before the maps and walls take
  them (the mapper owns that). Store the corrected pose in `frame_poses`.
- **"How sure" counts the pose twice:** the filter's doubt already holds the pose's
  doubt from each measurement, and `doubtOf` adds the surest pose again (with no
  heading-times-distance part). Fine for brightness; C uses it to decide who gives
  way, so look at it first.
- Move `outline_map.ts`'s use of `Walls` to a plain snap function if it gets in the way.
- Test: the `loop` scenario's end error must come out clearly under the odometry's
  (27 cm and 12° at its end).

## Decided (Daniel, 11 Nov)

- The polygon is the border of the seen-free floor. Three of them, one per band.
- Islands are separate closed shapes. The polygon grows through doors; no rooms.
- Guess edges at the sensor's reach (a faint arc in a big room) are fine.
- Past frames are not re-placed after a correction, in the first version.
- Edges no longer seen are removed, with more than one piece of evidence.
- A simulator first. It is for the agent, not for driving by hand: pictures with a
  slider, and the scenarios as tests.
- One commit per step, after tests and review (no push).

## Open for Daniel

1. **A real drive**, later: one that goes round and comes back to its start. It
   needs the robot flashed (the PC speaks wire 5; the robot has the 10 Nov
   firmware). Not needed before step C has passed in the simulator.

## Open technical points

- The outline is the floor joined to the start. Floor that standing things cut
  off from the start is not drawn, also where the robot is now (a door closed
  behind it). For step D.
- The simulator does not smear a reading that ends on the floor, also while
  moving. Whether the real sensor does: boot `cc0e4e65` can tell.
- How fast the pose's "how sure" should fall with driving and turning: first
  numbers from the stored drives (a 30° turn is about 31°).
- The border's simplifying limits (how rough is "rough") and the wall rules (40 cm,
  3 cm; 1 m, 5 cm): tuned on the real drive.
- The snap to a known wall reaches 12 cm (far hits smear about 7 cm): a box standing
  10 cm in front of a wall is drawn on the wall. Tune on the stored drive.
- Walls only grow: a hit up to 20 cm past an end lengthens it, so smeared hits near
  a corner may push a wall round it. For step D.
- With drift, walls are duplicated and off (step C's job). With the simulator's far noise (7 cm) rough
  edges are jagged by about 10 cm.
- Building takes longer than the line map's 1 s once poses are corrected frame by
  frame; the tables double as a cache for the slider.

## Alternatives for later (Daniel, 12 Nov)

The plan is a Kalman filter with walls as landmarks: one pose with a "how sure",
corrected frame by frame, past frames left where they were put. Two known methods
do more. They are kept for discussion once step C is measured, not built now:

- **Graph SLAM (loop closure):** keep every frame's pose; when the robot sees an
  old wall again, solve for the whole path at once and spread the correction back
  over the earlier poses. Fixes the error built up along a loop. The tables
  `frame_poses` and `wall_sightings` already hold what it needs.
- **Particle filter:** many random guesses of the pose round the odometry's, each
  scored by how well the frame fits the map; the best ones are kept. Handles a
  robot that is badly lost, but costs much more, and 64 zones give a blurry score.
- When to look at them: if the `loop` scenario (or a real drive round and back)
  still ends clearly off after step C.
