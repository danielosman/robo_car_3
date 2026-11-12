# Outline map and drift correction — plan (11 Nov 2026)

Agreed in general by Daniel on 11 Nov; steps S (the simulator), A (the outline), B
(the walls), C (the pose check), D (removal) and E (the page) are built; F is left. It follows the line map
([LINE_MAP_PLAN.md](LINE_MAP_PLAN.md), step 7 of [DATA_FLOW_PLAN.md](DATA_FLOW_PLAN.md)),
which Daniel looked at and found is not what he wants: loose lines, too few of them.

Contents: In short · For the fresh session · What Daniel asked for · The three ideas ·
What is stored · How it fits the code · The simulator · Steps · Decided (11 and 12 Nov) · Open ·
Alternatives for later.

## In short

The map is a **closed outline** of the floor the robot has seen free, one per height
band. It starts as a 35 cm circle, grows as the sensor sees further, and is always
closed: where nothing was seen yet, its edge is a guess, drawn faint. Long straight
edges are remembered as **walls**, kept exact, and used to correct the robot's
**drift**. Every wall and every pose carries how sure we are of it; when they
disagree, the less sure one gives way.

## For the fresh session

- **Steps S, then A to F in order.** The table in "Steps" says which are done:
  start at the first that is not. **Next: step F** (the PC's correction sent to the
  robot); read "Step F: where to start" and "Decided (Daniel, 12 Nov)" first.
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
  moves. (This weighing is a Kalman filter; nothing fancier is planned.) As built
  it keeps one covariance over the pose and the walls together (EKF-SLAM): a wall
  placed from an unsure pose shares the pose's error, and seen again from there it
  says nothing about where the pose truly is.
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
| `wall_sightings` | band, wall, frame, the pose it was seen from and how sure, the hits (JSON), how far off they were (across, slope; NULL: only near its ends, it was only lengthened) |
| `frame_poses` | frame, the odometry pose, the corrected pose, how sure after the check |

- Written by `robot walls [n]`: the boot's rows are replaced each time.
- Which walls a frame was checked against: `wall_sightings` by frame.
- The raw frames stay as they are. Everything above can be built again from them.

## How it fits the code

Built (steps A to D), in `pc/robot/src/maps/outline/`:

| File | Knows | Interface |
|---|---|---|
| `sights.ts` | ToF frames, verdicts, "beyond", the band heights; does a wedge see a band's floor | `sightsOf()`, `inBand()`, `heightsAt()`, `seesBand()` |
| `mapper.ts` | the object on top: per band a map and its walls; places each frame by the corrected pose; when the pose is well fixed for removal | `new Mapper({ correct })`, `add(frame, sights)`, `outlines()`, `walls()`, `sightings()`, `poses()`, `removed()`; `fixedWithout()` |
| `outline_map.ts` | one band's 5 cm squares: seen free, hits (spots), see-through; which border point lies on which spot, or on which wall | `new OutlineMap(band)`, `add(sights)`, `forget(wall)`, `outline(walls)` |
| `border.ts` | the closed loops round a set of squares | `connected()`, `bordersOf()` |
| `shape.ts` | a loop's points as wall, rough and guess edges; edges on a known wall carry its number; no knots | `edgesOf(points)` |
| `walls.ts` | one band's walls: their line, ends and how sure; a frame's hits on them as measurements; new ones, merges, corners; walls seen through go | `new Walls(band, check?)`, `measure()`, `apply()`, `see()`, `found()`, `seenThrough()`, `on()`, `doubtOf()`, `sureOf()`; `coords()`, `between()` |
| `pose_check.ts` | the joint filter over the pose and the band "above"'s walls; each wall alone for the other bands | `new PoseCheck()`, `moved()`, `added()`, `removed()`, `recentred()`, `check()`, `doubt()`, `covOf()`; `wallsAlone()` |
| `kalman.ts` | one Kalman step on small matrices | `kalmanStep()` |
| `pose.ts` | how fast the pose's doubt grows with turning and driving | `growth()`, `grown()`, `lineDoubt()` |
| `colours.ts` | a colour a band, a brightness an edge (walls by how sure) | `edgeRgb()`, `outlineStrokes()` |
| `index.ts` | the store and the helper: a boot's outlines and walls at a snapshot; the tables | `outlinesAt()`, `wallsAt()` |

- `robot map --png --lines [--no-cells]` draws the outlines; `robot walls [n]` lists
  the walls and writes the three tables.
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
- Scenarios: `square`, `things`, `door`, `loop`, `box-removed`, `box-stays`,
  `board-removed`, `board-stays`, `big`. The box's sides (30 cm) are too short for a
  wall; the board (1.2 m long, 10 cm thick, 40 cm high; its front 40 cm before the
  room's wall) is a wall in every band.
  `--no-noise` and `--no-drift` switch those off; `--no-correction` turns the pose
  check off (the white robot is the corrected pose). Pictures go to
  `pc/robot/recordings/sim/<scenario>/`.
- With the measured drift (3 % per turn) a full turn ends 11° off, and the `door`
  drive ends 58 cm off. Step C takes most of it out (see "How the pose is checked").

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
| C | done | The pose check: frames checked against walls, corrected poses stored, the map built with them | the same drive drawn with and without correction (`npm run sim -- loop [--no-correction]`); the error on returning to the start |
| D | done | Removal with the two-wall rule | `npm run sim -- board-removed`: the board goes; `board-stays`: it stays with drift |
| E | done | The page: the outline per band, toggled with the cell map, both can be on (the old step 8); everything placed by the corrected pose | the page; `robot map --png --lines`; the log's "outline map, snapshot n" lines |
| F | | The PC's correction sent to the robot after each snapshot; the robot applies it to every pose from then on; the PC turns the robot's poses back to raw odometry (firmware, wire, store) | a drive where the robot's map and the PC's agree |

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
  straight to 3 cm. So step B added a second wall rule (1 m or more fitting a line
  to 5 cm).
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

- **Without the pose check, walls seen later are less sure:** the pose's doubt only
  grows. In `square` without drift the four walls read 0.94, 0.71, 0.54, 0.33.
- **L1's walls are few and rough:** its floor is known to 1.65 m only.
- Since step C's rules for hits near a wall's end and in corners, `things` (no
  drift, correction off) finds 3 of its 4 walls in "above", not 4; a test holds 3.

How the pose is checked (step C, as built):

- **One band checks the pose: "above".** A wall stands in every band; three bands
  would count it three times. "above" finds the most walls; things on the floor are
  mostly below it. L1 and L2 walls then see the frame from the corrected pose, each
  wall alone, as in step B.
- **One covariance over the pose and the band's walls** (`PoseCheck`). Odometry
  moving the pose grows its doubt (and turns the heading's doubt into position
  doubt along the way driven); a new wall placed from the pose shares its error.
  Each frame: the hits on each wall give a measurement (offset of their middle,
  slope if they span 20 cm); one Kalman step gives the pose's change and every
  wall's, also walls not in view. The frame's sights are then placed by the
  corrected pose (`mapper.ts` keeps the shift from odometry).
- **Tried first, as the plan said: no covariance between pose and walls.** Failed:
  each check made pose and wall look sure, new walls took up the drift, `square`
  stayed 9-17° off.
- **Rules found in the simulator** (each fixed a measured failure):
  - a hit whose wedge covers a wall's end doesn't measure it (it may be the next
    wall round the corner); it only lengthens it;
  - a hit that fits two walls at an angle (a corner) is not used; of walls in line,
    it goes to the one it lies least beyond;
  - past a wall's end a hit must lie within 2 of its own spreads of the line;
  - a frame's hits on a wall are never surer than 1 cm and 2° (still), 3 cm and 6°
    (moving: readings smear);
  - a measurement more than 3 spreads off (pose, wall and hits together) is left out;
  - in "above" a new wall is the line through the current frame's hits along the
    outline's wall edge (the outline holds hits placed by older poses); its own
    doubt 5 cm, 10 cm if made while moving. New walls are looked for every 10
    frames and on every still frame.
- **Wall ends move both ways** (Daniel: seen face-on an end is vague; seen from the
  side, the corner is placed better). Each end has a spread. A frame sees an end
  between the last hit on the wall and the first wedge that saw at least 30 cm past
  its line at the band's heights; a Kalman step moves the end. So the end is as
  sure as the wedges are dense there: closer is surer. Two walls at more than 45°
  whose ends are within 30 cm of where their lines cross end there (a corner,
  placed by both lines).
- `new Mapper({ correct: false })` and `npm run sim -- <name> --no-correction` give
  step B's behaviour, to compare.

What the pose check does (simulator, all noise and the measured drift; mean of the
end error over 5 seeds):

| Scenario | Odometry | Corrected |
|---|---|---|
| `square` (a full turn on the spot) | 1 cm, 10.7° | 1 cm, 1.4° |
| `things` | 1 cm, 10.7° | 1 cm, 0.8° |
| `box-stays` (two turns) | 1 cm, 21.1° | 1 cm, 1.4° |
| `loop` (round a room, back to the start) | 27 cm, 12.0° | 9 cm, 2.0° |
| `door` (two rooms) | 55 cm, 21.2° | 23 cm, 4.4° |

- Without noise or drift the pose stays exact (under 1 cm and 0.1° all the way).
- **Weak spot:** with noise but no drift the check alone can push the pose off, up
  to about 30 cm and 7° (`door`, 5 seeds; a test caps seed 1 at 25 cm and 7°). It
  happens while driving through the door: moving frames shift the pose 3-8 cm each
  and its doubt shrinks while the error grows.
- With drift the walls are far closer to the truth than in B (`loop` 7-8 cm against
  16-23 cm). In `square` the walls end within 1 cm of the corners without noise,
  1-7 cm with noise.
- The stored drive (boot `cc0e4e65`): built in 2.9 s; the correction stays within
  6° and 6 cm of odometry (no truth to compare with).

How walls are removed (step D, as built):

- **The pose is well fixed without wall w** (`fixedWithout`, `mapper.ts`): its last
  check against two walls at 45° or more, neither of them w, both still known; since
  then odometry added at most 2 cm and 3° of doubt, frame by frame (about an 85° turn
  or 2 m). In L1 and L2 the fixing walls are always another band's.
- **Seeing through a wall** (`seenThrough`, `walls.ts`): a wedge whose middle ray
  crosses the wall's line at the band's heights and reaches 10 cm past it (a sight
  already ends a margin before what it hit: 15 cm at 1.5 m), inside its ends by its
  wedge's width, 10 cm and two of the end's spreads (ends are vague).
- **Removal:** 2 such wedges in a frame (one alone: far readings smear past real
  walls), in 2 frames, with no sighting of the wall between (3 hits or more; the
  frames need not be close): it is gone, from `PoseCheck` too (`removed()`), and the
  outline forgets the spots on its line (`forget()`, within 10 cm), else they would
  make the wall again. `mapper.removed()` lists the walls removed.

What removal does (simulator, all noise and the measured drift):

| Fact | Number |
|---|---|
| Walls removed that still stand (9 scenarios, 10 seeds) | 0 |
| `board-removed`: the board's walls all gone at the end (10 seeds) | 7 of 10 (else one L2 wall stays) |
| Without noise or drift: the board's walls go | at steps 12-13, the first look there with the pose fixed |
| The stored drive (boot `cc0e4e65`) | nothing removed; walls as before (L1 5, L2 2, above 2); 2.8 s |

- **The rule only acts where the pose gets fixed by two walls.** In a look round on
  the spot that is from the second turn on (the first meets each wall new); in `loop`
  (default seed) never. So "nothing removed" there says little.
- With noise the board goes late (steps 21-23 of 24): the robot's views of the empty
  place in the first turn after it went come before any two-wall check.
- Where one stays: an L2 wall seen through in one frame only.

### Left open

From the reviews of steps C, D and E; none blocks step F.

- Each wall's `cov` is a copy of its part of `PoseCheck`'s covariance, made fresh by
  `sync()`; `Walls` branches on "has a pose check" in several places. Cleaner: one
  "how sure are the walls" object with two kinds (alone, as in B; joint).
- A frame with 1-2 hits on a wall neither resets its count of frames seen through nor
  stops that frame counting.
- The weak spot above: moving frames through a door push the pose off.
- The odometry's doubt growth reads the pose's current doubt ("adds up"); a turn
  scale error kept as part of the state would be the principled form.
- `corners()` overrides the end filter without asking its spread; an end's spread
  never grows again (it could freeze).
- The mapper sequences measure → check → apply itself; one `walls.checkPose(...)`
  could hide that.
- No direct test of `PoseCheck.recentred()`.
- First guesses to check on the stored and a real drive: past an end 2 spreads, end
  gap 50 cm, seeing past 30 cm, corner 30 cm and 45°, a new wall made moving 10 cm,
  the noise floors above; for removal 10 cm past, 2 wedges, 2 frames, 2 cm and 3°.
- Only whole walls go: part of a wall gone (a door opened in it) is not handled.
- The joint state grows by 2 rows a wall: fine at today's sizes, watch on long drives.
- Step E: `CellMaps` and `OutlineMaps` know each other (rays one way, the placement
  the other, joined by `placeBy` in `main.ts`). Cleaner: the rays as their own small
  object given to both. A bug in the outline map now also stops the cell map,
  `robot check` and `robot map`.
- Step E: the page draws objects and the snapshot's ToF dots by the shown snapshot's
  correction, not each by its own snapshot's.
- Step E: `/map`'s `path` is still odometry's (the page uses it only without an
  outline map; `robot map --png` takes the corrected one).

### How the page and the PC's maps work (step E, as built)

- **One outline mapper per boot, kept** (`OutlineMaps`, `maps/outline/index.ts`). It
  only goes forward: fed the frames up to each snapshot in turn, it keeps at each the
  outlines, the correction and how many frame poses it had. A past snapshot is read at
  once; a snapshot stored again with other frames, or a feed that fails part way,
  builds it again from the first frame. Two boots are kept.
- **When a snapshot is stored** (`main.ts`) the mapper is brought up to it and one
  line goes in the log: `outline map, snapshot n: pose corrected by a cm, b° in all;
  this move c cm, d°` (corrected against odometry at the snapshot's last frame).
- **The PC's cell map is placed by the corrected pose** (`CellMaps.placeBy`): each
  frame by its corrected pose, the snapshot's pose corrected. `robot check` still
  asks in the robot's own frame (odometry pose, goto targets): the map turns its
  places into the corrected frame first.
- **The page** (`app.js`): toggles cells and outline (both on: the outline over the
  cells), and L1, L2, above. The robot, the live pose, the path, the snapshot dots, the
  ToF dots, the objects and the camera cone are drawn corrected (the page's
  `corrected()` is a copy of `mapper.ts`'s). A goal click is turned back into the
  robot's frame before `goto`, kept there, and drawn corrected.
- **CLI:** `robot map --png [--lines [--no-cells]]` draws what the page draws (robot
  and path corrected); `robot map`'s JSON pose is corrected; `robot walls` builds its
  own mapper for the snapshot.
- **Numbers** (stored drive `cc0e4e65`, 774 frames, 28 snapshots):

| What | Time |
|---|---|
| First read after a server start | 3.5 s |
| Each new snapshot (one move) | 0.1-0.3 s |
| A past snapshot | 11 ms |

### Step F: where to start

Not built. Goal: the robot uses the PC's best pose from then on. Nothing is moved
back in time (no graph SLAM): past frames, cells and path dots stay where they were
placed.

Terms (all poses x, y, yaw; `corrected(c, p)` and its inverse as in `mapper.ts`):

- `raw` — PicoB's odometry, as today. Never changed.
- `c_pc` — the PC's total correction now (`mapper.correction()`). Best pose =
  `corrected(c_pc, raw)`.
- `c_robot` — the total correction the robot applies. Starts at 0 at power-up.
- The robot's pose = `corrected(c_robot, raw)`. With `c_robot = c_pc`, the robot's
  pose is the PC's best pose.

What is sent (proposed; Daniel agreed "the corrected pose", see "Open for Daniel"):
the **total correction `c_pc`**, not a pose and not a change.

- Like a pose: absolute. A lost message is put right by the next one.
- Unlike a pose: it does not depend on when it is applied. A pose is right only for
  the moment it was computed; the robot may have moved since.

Robot (PicoA; PicoB unchanged):

```
on command "correct x y yaw":          // queued like any command: runs between moves
    c_new = (x, y, yaw)
    move every odometry report kept (body.c, the last 16) from c_robot to c_new
    c_robot = c_new
    answer ok

on each odometry report from PicoB (body.c):
    keep it as corrected(c_robot, raw)  // the one place the correction is applied
```

- So every user of the pose on PicoA (cell map, house map, goto, room rule,
  `pose_at`/`pose_stamp`, vision) and every pose sent to the PC (frames, status,
  live pose, snapshots, answers) is `corrected(c_robot, raw)`. None of them changes.
- The robot's own map: old cells stay; new frames are placed by the new pose.

PC:

```
after a snapshot is stored (main.ts, after OutlineMaps.added):
    if c_pc != c_robot_sent: queue "correct c_pc"; c_robot_sent = c_pc

store: robot_corrections(boot, message_number, x, y, yaw)
    one row per "correct" answer: c_robot from that message on

rawOf(boot, message_number, pose) = uncorrected(c_robot at message_number, pose)
```

- **One rule:** every pose read from the store or the live feed goes through `rawOf`
  before the PC uses it. Then `OutlineMaps`, the mapper and everything after them stay
  as built (they take `raw`). Readers to change: `store.framesBetween` (frames for
  the outline mapper and the cell map), `snapshotsUpTo`/`snapshotFrames`, `store.path`,
  the live pose (page), `robot check`'s start pose.
- **Goto** (page click, `robot steer g`): `target_robot = corrected(c_robot_now,
  uncorrected(c_pc, target_map))`. With `c_robot = c_pc` it is the place clicked.
- **Log:** the existing line, plus `correction sent: a cm, b°` when one is queued.

Needs: a new command (`picoA/app/commands.h`, `wire.ts`, the CLI's list), the
`robot_corrections` table, `rawOf` and its readers, tests on both sides (the host
test `test_behaviour` for PicoA; the drift test in `index.test.ts` for the PC), a
flash of PicoA ("Open for Daniel" 2).

To look at, on a real drive:

- `robot map --robot --png` (the robot's own map) and `robot map --png` (the PC's)
  agree.
- After each "correct" is answered, `robot status`'s pose equals the page's robot
  (the PC's best pose) to 1 cm and 0.5°. The log's "pose corrected by" line does not
  change with F: it compares the best pose with `raw`.

## Decided (Daniel, 11 Nov)

- The polygon is the border of the seen-free floor. Three of them, one per band.
- Islands are separate closed shapes. The polygon grows through doors; no rooms.
- Guess edges at the sensor's reach (a faint arc in a big room) are fine.
- Past frames are not re-placed after a correction, in the first version.
- Edges no longer seen are removed, with more than one piece of evidence.
- A simulator first. It is for the agent, not for driving by hand: pictures with a
  slider, and the scenarios as tests.
- One commit per step, after tests and review (pushed since step B, as Daniel asked).

## Decided (Daniel, 12 Nov)

- The map is updated after each move; a few seconds per move are fine.
- The page shows the corrected pose only: no odometry pose, no old estimates.
- The corrections go in the log (cm and degrees), not on the map.
- Removed walls are not shown; they vanish from the map.
- Cell map and outline map are two toggles; both on, they are drawn over each other.
- The PC's cell map is placed by the corrected pose too (it is the PC's own, built
  from the stored frames).
- `robot map --png --lines` draws what the page draws, for the agent to test and drive.
- Step F, its own step after E: the PC sends the robot its best pose as an absolute
  value, not as a change (a lost message is then put right by the next one); the
  robot's own cell map and steering use it. Past frames, cells and dots are not moved.

## Open for Daniel

1. **Step F sends the total correction, not the corrected pose?** Proposed in
   "Step F: where to start": both are absolute (a lost message is put right by the
   next one); only the correction does not depend on when the robot applies it.
2. **A real drive**, later: one that goes round and comes back to its start. It
   needs the robot flashed (the PC speaks wire 5; the robot has the 10 Nov
   firmware). Step C has passed in the simulator: it can come any time.

## Open technical points

- The outline is the floor joined to the start. Floor that standing things cut
  off from the start is not drawn, also where the robot is now (a door closed
  behind it).
- The simulator does not smear a reading that ends on the floor, also while
  moving. Whether the real sensor does: boot `cc0e4e65` can tell.
- How fast the pose's "how sure" should fall with driving and turning: first
  numbers from the stored drives (a 30° turn is about 31°).
- The border's simplifying limits (how rough is "rough") and the wall rules (40 cm,
  3 cm; 1 m, 5 cm): tuned on the real drive.
- The snap to a known wall reaches 12 cm (far hits smear about 7 cm): a box standing
  10 cm in front of a wall is drawn on the wall. Tune on the stored drive.
- With the simulator's far noise (7 cm) rough edges are jagged by about 10 cm.
- L1 and L2 walls (drawing only) are rougher than "above": up to 18° off in `square`.

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
- When to look at them: if a real drive round and back still ends clearly off. In
  the simulator step C brings `loop` to 9 cm and 2° (odometry 27 cm and 12°).
