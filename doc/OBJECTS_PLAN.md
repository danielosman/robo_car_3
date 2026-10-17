# Objects on the map — plan (first draft)

First draft, 17 Oct 2026. **The first step is built** (17 Oct): `robot detect`,
`robot objects`, the tables and the page's overlays (ARCHITECTURE.md, the PC,
`objects/`); the README's "Objects from the camera" test. The camera calibrated
(f 180 px, tilt −1.8°). Next: the next phase's to-do (the bottom edge from the
pixels, the ToF's distance, the tilt calibrating itself). Background reading is in
[inspiration/](inspiration/): `vlm_bbox_to_floor_map.md` (boxes from a model, then
geometry) and `typescript_2d_spatial_query_libraries.md` (rbush, turf,
detect-collisions, rotated shapes, walls, the camera's view as a triangle).

## The overall goal

Add a layer of **objects** to the cell map: chairs, tables, plants, shelves,
cabinets, walls. Each object is a shape on the 2D floor (a rotated rectangle; a line
for a wall) with a label.

- **First guess from the camera.** A vision model (Gemma 4 26B) draws a box around
  each object in the camera picture. Geometry turns that box into a rough footprint
  on the floor. The model only detects; the math places. A model can't judge
  distance from a picture, but the camera's height and angle give it.
- **Refined by the map.** As the ToF fills in more occupied cells, each object is
  moved and rotated to best cover the cells it should explain. Walls are lines
  fitted to rows of occupied cells.
- **Stored as overlays.** The boxes on the picture and the shapes on the map are
  stored in DuckDB, with each map shape linked to the detections it came from. An
  in-memory spatial index (rbush) is built from them for fast queries.
- **Later, an agent with tools.** It can ask what is near a point, what the robot
  should see from a pose, or which cells no object explains. It decides whether to
  move or rotate an object, or to correct the robot's pose. That last part is the
  "Drift correction" task in [TODO.md](TODO.md), with objects and walls as
  landmarks.

## The first steps

1. **`robot detect n`** on one snapshot of the last boot (7a587226, 132 pictures)
   at a time, run by hand: the model's boxes, each placed on the map and saved.
2. **Show** the boxes over the camera picture and the shapes over the map, for the
   same snapshot, on the page. Daniel looks at each result before running the
   next one.
3. **Calibration run.** Daniel walks to the objects and says what the distances
   really were; the camera's numbers are fitted to that (below).
4. Then we look at the results and decide on the next steps.

## What `robot detect` does

```
robot detect [n]  (CLI, thin)
   │  POST /api/robot/detect {n}
   ▼
API server (pc/robot)
   1. the snapshot's picture (PNG, 160 × 120) and pose
   2. ask the model ──► Gemma 4 26B, OpenAI-compatible server on :8080
   3. check its answer; if wrong, tell the model what is wrong and ask again
   4. place each box on the map (camera geometry + the snapshot's pose)
   5. save: the model's answer, the boxes, the shapes on the map
   │
   ▼
one JSON line: the objects found, each with its box and its place on the map
```

- **The CLI is a thin layer**, as the other commands are: it calls the API and
  prints the answer. The server does the work. `n` is the snapshot's number, the
  newest without it, as for `robot camera`. **One snapshot per call**, no
  `detect all` and no loop for now: Daniel runs each one by hand and looks at the
  result on the page.
- **It runs at once, not through the queue**: it doesn't move the robot, and it works
  on old snapshots with no robot connected.
- **The model** runs on Daniel's machine (started from `~/`) as an
  OpenAI-compatible server on http://127.0.0.1:8080 (the reason the robot server
  moved to 8081). `MODEL_API` and `MODEL` set where and which, defaulting to
  that server and Gemma 4 26B. Deterministic settings (temperature 0).
- **Running it again** on the same snapshot replaces that snapshot's detections
  and their shapes. **To tweak later:** keeping every run (to compare models,
  prompts, or several answers for the same picture) may be wanted once the first
  results are in.

### The model's answer

Gemma 4 returns boxes as `{"box_2d": [y1, x1, y2, x2], "label": "..."}`, on a
**0-1000 grid** (y first, then x), in a ```json block. This is in Google's own
Gemma docs, so it isn't only Gemma 3 or Gemini behaviour
([Gemma image understanding](https://ai.google.dev/gemma/docs/capabilities/vision/image),
[Hugging Face: Welcome Gemma 4](https://huggingface.co/blog/gemma4)). The order is
taken from the docs' own parsing code, so the first run checks it on a picture
where the answer is obvious. The server converts to pixels:
`x = x1 / 1000 × 160`, `y = y1 / 1000 × 120`.

### The fixed list of labels

`chair, table, sofa, door, wall, plant, shelf, cabinet, box, object`.
`object` is for anything that fits none of the others. (Chair, table, sofa, door,
wall and box were proposed earlier; plant, shelf, cabinet and object are Daniel's.
Cabinet replaces the earlier "cupboard". The list is easy to change.)

### Step 0: the model server, tried by hand (17 Oct)

llama.cpp's router on :8080 serves `gemma-4-26b` (and `gemma-4-31b`), Q4 with
the vision projector. The router doesn't load a model on the first request ("model
is not loaded"): it needs `POST /models/load {"model": "gemma-4-26b"}` first, so
the server will have to do that (or say so) when it gets that error.

One picture (boot 7a587226, snapshot 31: a sofa with a pouf in front), the
`report_objects` tool, temperature 0:
- **Tool calls work**: `finish_reason: tool_calls`, the arguments as the schema
  says, `bbox` and labels from the list.
- **The order and grid are right**: y first, 0-1000.
- **Thinking on** (the default): 14-16 s, ~1,370 tokens of thinking, the same
  answer twice. Sofa and box found. The pouf's bottom edge within ~1 px of where
  it meets the floor; the sofa's ~4 px too high.
- **Thinking off** (`chat_template_kwargs: {enable_thinking: false}`): **0.8 s**,
  the same pouf box (to 3 on the 0-1000 grid), but labelled `object`, and the sofa
  missed.
- The sofa's bottom edge is at about row 57-59, on the horizon of a level camera,
  in snapshot 31 and also in 30, where the sofa is much closer. So it isn't the
  camera's tilt: the sofa's front edge doesn't touch the floor. There is a dark gap
  under it, and an edge ~8 cm up looks the same from a camera at 8.8 cm whatever
  the distance. **A bottom edge gives the distance only where the object really
  touches the floor.** Sofas, cabinets and beds on short legs or with a recessed
  base need care: the system prompt asks for where the legs or base meet the floor,
  and the box can say when that isn't visible.

### The tool the model reports with

The model reports through one tool, `report_objects`, with one argument: an
array of `{label, bbox: [y1, x1, y2, x2]}` (0-1000, Gemma's own order and grid;
the key is `bbox` rather than Gemma's `box_2d`, as clearer). An
empty array means "nothing here". Gemma 4 supports function calling; whether the
local server passes it through well is the first thing the first run shows. If
it doesn't, the fallback is the same array as plain JSON in the answer, with the
same checks.

**The tool checks every call** and answers either ok or the errors, so the model
can correct itself and call it again:
- a label not on the list: `"label 'stool' is not one of: chair, table, ..., object.
  Use object if nothing fits."`
- a box outside 0-1000, or with y1 ≥ y2 or x1 ≥ x2;
- arguments that aren't the array asked for.

**An empty list is pushed back once.** The first time the model reports nothing,
the tool answers: "An empty list is unusual: a room in a flat almost always has
something in view (furniture, a wall, a door). Look again; if there really is
nothing, report the empty list again." A second empty list is accepted.

**The session's end is checked.** If the model ends its turn without calling the
tool (or after a call that got errors), the server asks it to call
`report_objects` now, up to 3 times. After that the detection fails, says why,
and the model's answers are kept for looking at.

The same checks are used later, when an agent adds or changes objects through
the API.

### The system prompt

What it has to say (wording to work out on the first runs):
- The picture: a 160 × 120 greyscale camera, **8.8 cm above the floor**, looking
  straight ahead, on a small robot in a flat. Things look tall and are often cut
  off at the top; the floor fills the bottom of the picture.
- The job: find the objects standing on the floor, from the list (`object` if
  nothing fits), and report each with `report_objects`.
- **The bottom edge matters most.** Each box's bottom edge is used to work out how
  far away the object is: it must be exactly where the object touches the floor
  (the lowest point of its legs, base or front), as precise as possible, to the
  pixel. The top edge matters little; the left and right edges give its width
  and bearing.
- One box per object; a chair's legs are one chair. Partly hidden or cut-off
  objects are boxed for the part that is seen. Walls and doors: the box's bottom
  edge along where they meet the floor.
- The coordinate format, with an example, and that the answer goes through the
  tool, not in text.

## The camera geometry

### What is known

| | Value | Source |
|---|---|---|
| Height of the lens | **8.8 cm** above the floor | measured by Daniel, 17 Oct |
| Position | **2.5 cm ahead** of the centre, **0.5 cm right** (y = −0.5 cm) | measured by Daniel, 17 Oct |
| Tilt | **−1.8°** (down) on top of the robot's pitch from the pose | calibrated 17 Oct (below) |
| Lens | **f = 180 px** (~48° across) | calibrated 17 Oct (below) |
| Picture | 160 × 120, from the sensor's 640 × 480 window (subsampled and binned), so it covers close to the full field of view | `picoA/drivers/hm0360_init.h` |
| Sensor | HM0360, 3.6 µm pixels; the 640 × 480 window is 2.30 × 1.73 mm (2.88 mm across: the 1/6″ format Arducam gives) | Arducam |
| Lens | Arducam's listing: **focal length 2.59 mm, DFOV 64°** | [Arducam HM0360](https://www.arducam.com/hm0360-vga-monochrome-dvp-camera-module-for-arduino-giga-r1-wifi-board.html) |

These numbers are constants in the server's code (`CAMERA_GEOMETRY` in
`pc/robot/src/objects/place.ts`, one place), not stored in the database: they don't
change per snapshot. When the calibration changes them, `npm run place-objects`
places every stored box again (the model isn't asked again).

The old plans said 8.5 cm up and no offset to the side; Daniel's new measurement
replaces that. The firmware's `FOCAL_PX 160` (`picoA/app/camera_motion.c`, from
53.1° across) is left as it is for now; the calibration says 180 (below).

**The ToF** (measured by Daniel, 17 Oct): 3.2 cm ahead of the centre, 2.6 cm right,
7 cm up; that is 0.7 cm ahead of the camera and 2.1 cm to its right. In the firmware
(`picoA/app/rangefinder.c`; it was 2.5 cm ahead, 3 cm right).

### Checking the two sets of numbers

The two numbers in Arducam's listing don't agree with each other:

- **From DFOV 64°** (with the sensor's 4:3 window): HFOV **53.1°**, VFOV **41.1°**,
  f ≈ **160 px**. These are the numbers the robot's plan and code use, so the plan
  did the right math from that 64°.
- **From the focal length 2.59 mm**: HFOV **48.0°**, VFOV **36.9°**, DFOV 58°,
  f ≈ **180 px**.
- **The inspiration doc's 51.2° / 38.4°** is 64° split along the 4:3:5 sides
  (64 × 4/5, 64 × 3/5). Angles don't split that way, so those two numbers are
  neither of the above. They aren't more exact; they come from a shortcut.

So the truth is probably **between 48° and 53° across**, and we can't tell which
from the data sheets. **The calibration says f = 180 px, ~48° across** (below).

**Web search (17 Oct): nothing confirms ~58° diagonal.**
- Every listing of the stock lens says 64° diagonal: Arduino Store, UCTRONICS
  (the GIGA module), and OpenMV (Portenta, HM0360).
- OpenMV gives "Actual FoV ... D64.0° H53.1° V41.1°": all three fit f ≈ 160 px.
- So the "2.59 mm" in the listings is probably the error, not the 64°.
- No source gives a FOV for our module, the Arducam B0319. Its only drawing has a
  table copied from another sensor's module, so it doesn't help.
- Himax sells HM0360 modules with 70°, 96°, 129° and 179° lenses. Lenses differ
  between modules.

**So, open:** our one measurement (the pouf's height) says 180 px, the web says
160 px for other modules. 180 px stays in the code for now. The ToF pairs of the
next phase (step 3) can fit f together with the tilt and settle it.

### Why the tilt and the bottom edge matter so much

The **horizon** is the picture's middle row (60), moved up or down by the tilt: the
robot's pitch from the pose plus the camera's own tilt, ~3.1 rows per degree (f 180).
A point on the floor at distance d shows up `f × h / d` rows below it (h 8.8 cm). A
box whose bottom edge is at or above the horizon can't touch the floor: not placed.

| Distance | Rows below the horizon | 1 px error (closer / further) | 1° of tilt error |
|---|---|---|---|
| 26 cm | 60 (the bottom row: anything closer is cut off) | | |
| 1 m | 15.8 | −6 / +7 cm | −17 / +25 cm |
| 1.5 m | 10.6 | −13 / +16 cm | −34 / +64 cm |
| 2 m | 7.9 | −22 / +29 cm | −57 / +132 cm |

So past ~1.5 m a few pixels of the model's bottom edge, or a degree of tilt, make
the camera's distance useless; under ~1 m it is good to a few cm. The lens (f)
matters much less: it scales every distance by the same few %.

### Placing one box (first guess)

1. **The floor contact:** the box's bottom edge. Its left and right corners, through
   the camera's numbers, give two points on the floor: the object's **front edge**,
   the side facing the robot.
2. Each pixel is a ray from the lens: forward f, left `(79.5 − x)`, up `(59.5 − y)`,
   tilted by the pose's pitch plus the camera's own tilt. It meets the floor
   (z = 0) where it has come down 8.8 cm. Then from the camera's position (2.5,
   −0.5 cm) and the snapshot's pose into the map frame.
3. **The depth** behind the front edge isn't seen: a default per label (to tune;
   e.g. chair 50, table 80, sofa 90, shelf 35, cabinet 45, plant 40, box 30,
   object 20 cm). The first shape is a rectangle facing the robot.
4. **When there is no floor contact:**
   - the box reaches the bottom of the picture: the floor contact is cut off, so
     the object is at most as far as the bottom row (~24 cm). It is placed there
     and marked "at most".
   - the bottom edge is at or above the horizon (a picture on a wall, a shelf's top
     part): not placed. Only its bearing is kept.
   - doors and walls have no footprint: a line along the front edge.
5. **Keep it re-runnable.** The model's boxes are stored as they came; the shapes on
   the map are computed from them and the camera's numbers. After the calibration
   the placing runs again without asking the model again.

**ToF as a second distance:** the ToF sees 45° across (the camera ~48°), at 7 cm
up. Where a box's columns overlap the ToF's zones, the ToF's reading is a
measured distance for the same object. **The first step is camera only**
(decided 17 Oct): the ToF is not used for placing yet. It comes in with the
fitting step. Showing both on the page is
useful in any case.

### The calibration (17 Oct, done)

From two objects in boot 7a587226, their floor contact read from the pixels (the
row where the dark object turns into the bright floor), not from the model's box:

| | Floor contact | Distance from the camera | Robot's pitch | Camera's own tilt |
|---|---|---|---|---|
| Pouf, snapshot 31 (near corner) | row 65.0 | ~124 cm (ToF) | −0.70° | **−1.78°** |
| Table, snapshot 1 (legs) | row 60.0 | ~185 cm (Daniel: 178-179 cm, taken from the robot's front) | −0.84° | **−1.88°** |

- **f = 180 px**: the pouf is 35.5 cm high and spans 52 rows (13 to 65) at ~124 cm.
  Its height doesn't depend on the tilt, only on f. 180 px is also what the lens's
  2.59 mm gives. (Its width can't be used: it stands at an angle.)
- **Tilt −1.8°** (the camera looks this much further down than the robot's pitch
  says). Both are in `place.ts`; the saved boxes were placed again.
- **After:** the pouf (the model's box) at 131 cm, 48 cm wide (ToF ~124-128 cm;
  38.5 cm wide, seen at an angle). The table with its true floor contact at ~190 cm
  (185 measured); with the model's bottom edge, 3.6 px too high, at 340 cm. So
  the tilt is fixed, and the model's bottom edge is now the larger error.
- The tilt may change a little over time (the camera's mount). **No more
  calibration by hand** (Daniel, 17 Oct): measuring floor distances by hand adds
  errors of its own. The ToF is the reference from now on (next phase).

## Storage (DuckDB)

DuckDB is already there (`recordings/robot.duckdb`, `@duckdb/node-api`), so no
SQLite. A first idea of the tables:

- `detect_runs`: one per `robot detect` on a snapshot: boot, seq, model, prompt
  version, time, tries, the model's raw answers, the result (ok / failed and why).
- `detections`: one per box: run, label, box (y1, x1, y2, x2 on 0-1000, as the
  model gave it).
- `map_objects`: one per object on the map: boot, label, centre x, y, width, depth,
  angle, kind (rectangle / line), where it came from (camera guess, fitted, by
  hand). The camera's numbers aren't stored (they are constants in the code);
  a camera guess can always be placed again from its detection.
- `object_detections`: which detections an object came from. In the first step one
  object per detection; merging comes later.

On start (and after a change) the server loads `map_objects` into **rbush**: each
shape by its outer axis-aligned box, with the exact corners kept beside it for
turf. The first step only needs "the objects of this snapshot". rbush earns its
place once the agent queries by area.

**One frame per boot.** The map's frame is odometry from that boot's power-up, so
objects belong to their boot. Objects across boots need drift correction or
matching first.

## Showing it

On the page (`public/app.html`, `app.js`), for the snapshot shown:
- the camera picture: each box with its label;
- the map: each shape (rectangle or line) with its label, the same colour as its
  box; the camera's view as a triangle.

A read for the agent: `robot objects [n]`, the snapshot's detections and their
shapes as JSON.

## Next phase: to do

Decided with Daniel, 17 Oct. Nobody will check each box, so the code has to:

1. **Find the bottom edge in the pixels.** Starting from the model's box, search a
   few rows up and down, in the box's columns, for the row where the object turns
   into floor (dark to bright, as read by hand for the table: the model was 3.6 px
   off, the pixels exact). The model gives the label and the left and right edges;
   the code the bottom edge.
2. **The ToF for the real distance.** Where the ToF's zones see the object (its 45°
   view, 7 cm up, from 3.2 cm ahead and 2.6 cm right of the centre), their distance
   is the object's. **Objects with only the camera's distance are kept** (beyond
   the ToF's view or range, or when it saw something else): a first guess.
   `map_objects` gets a column saying whether the distance is **ToF verified**, and
   a verified object is **moved on the cell map** to the ToF's distance.
3. **The tilt calibrates itself.** Each time the ToF and a box see the same object,
   the two give a tilt (as in the calibration above); a running average keeps
   `tiltRad` current with no one checking, and shows when the mount has moved.
   Pairs at different distances also fit f (160 or 180 px: still open, above).

Also: the firmware's `FOCAL_PX` (`camera_motion.c`) is 160, the calibration says
180: for the camera features' bearings, to change with their next robot test.

## Next steps after that (rough)

1. **Fit objects to the cells**: move and rotate each shape to cover the occupied
   cells it should explain (a rotated box fitted to its cluster of cells). Merge
   detections of the same object from different snapshots.
2. **Walls**: lines fitted to rows of occupied cells; large furniture aligned to
   the nearest wall.
3. **Query tools for the agent**: what is in view from a pose (rbush, then exact
   geometry), what explains a cell, which cells nothing explains, what is behind a
   wall.
4. **The agent decides**: move or rotate an object, or correct the robot's pose
   (drift correction from landmarks).
5. **Size limits per label** (a chair can't be 2 m wide), as a rule check rather
   than another model call.

## Open questions

Decided 17 Oct: the model reports with a tool (plain JSON only if the local
server's tool calls don't work); one snapshot per `robot detect`, no `all`;
running it again replaces (to tweak later); the camera's numbers stay in code;
the first step is camera only (no ToF in placing); the tool's key is `bbox`.

1. The default depth per label: the numbers above are guesses.
2. Pictures that are dark or blurred (the robot was turning): skip them, or let the
   model say "nothing"?
