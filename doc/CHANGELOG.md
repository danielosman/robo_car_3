# Changelog

What happened, newest first: robot runs, results, what was tried, decided, kept or
dropped, and how things were fixed. Only history: how things are now is in
[ARCHITECTURE.md](ARCHITECTURE.md), what is left in [TODO.md](TODO.md). Links in
older entries point to the plans of their day, now in [archive/](archive/).

## 18 Oct 2026: Qwen 3.8 tried, Gemma 4 31B kept

Snapshots 1-42 of boot 202610181635 (Daniel looked at the boxes on the page):

| Model | Boxes | Time per picture (no thinking) |
|---|---|---|
| `qwen-3.8-27b` | more objects, boxes off | 4-18 s |
| `gemma-4-26b` | as before | ~1 s |
| `gemma-4-31b` | best, more reliable than 26b | 2-9 s |

- Daniel uses `gemma-4-31b` from now on (`MODEL` in `pc/robot/.env`; the code's
  default stays `gemma-4-26b`).
- Without thinking the boxes were better than with it (Qwen).
- Qwen sees our 160 × 120 picture as 20 tokens, too coarse for boxes; llama.cpp asks
  for at least 1024 (`--image-min-tokens 1024`). That option also applies to Gemma 4
  and crashes it (its picture no longer fits the 512-token batch), so it is only a
  comment in `~/run-llama-router.sh`.
- Saved now: Gemma 31B for snapshots 2-31, Qwen for 1 and 32-42.

## 18 Oct 2026: a thinking budget for robot detect

Qwen 3.8 thought for minutes on one picture (4,000 tokens and still going at about 25
a second). `robot detect` now sends a thinking limit with each request (llama.cpp's
`thinking_budget_tokens`): `REASONING_BUDGET` in `pc/robot/.env` (2000), or
`--reasoning-budget=N` for one run (-1: no limit). When it runs out, llama.cpp ends the
thinking and the model answers. Saved with the run (`reasoning_budget`, shown by
`robot objects`). Checked with Qwen: the thinking stopped at ~2000 tokens, then the tool
call.

Also: the router (`~/run-llama-router.sh`) had no context size, so each model took its
full one; Qwen's 262k ran the Mac out of GPU memory. Now `-c 200000` for every model,
and no `-np 4` (llama.cpp picks the slots; they share the 200k).

## 18 Oct 2026: a prompt for Qwen

`system_prompt.md` is now `system_prompt_gemma4.md`, unchanged. New:
`system_prompt_qwen.md`, the same but with x first, only its example shows the order,
and in the picture's pixels (160 × 120) instead of a 0-1000 grid (Daniel). The model's
name in `MODEL` picks the prompt (`gemma…` or `qwen…`; another name: `robot detect`
says so). Qwen's boxes are turned y first and scaled to 0-1000 before they are checked
and saved, so the database stays as it was. Prompt version
stays 5; the saved model name tells the two apart. Not run against Qwen yet.

## 18 Oct 2026: settings in pc/robot/.env

`npm start` reads `pc/robot/.env` if there is one (`node --env-file-if-exists`), e.g.
`MODEL=qwen-3.8-27b` for `robot detect`. `.env` is not in git; `.env.example` shows
the settings with their defaults (`MODEL=gemma-4-26b`).

## 18 Oct 2026: boot folders named by date

The folders under `pc/robot/recordings/` (`camera/`, `map/`) are named `YYYYMMDDHHMM`
instead of the robot's random boot id (`boot_folders.ts`, ARCHITECTURE.md). The robot
still sends its boot id; `recordings/boots.json` maps it to the folder name, and the
database keeps the boot id. The existing boots were named once by a script: boots with
pictures by the first picture's file time, the 7 with only recordings (0558a0ee,
0b9377b7, 87aa43f1, 54564d43, and 8f3159b3, ace8afeb, 123eef20 from the oldest files
without the boot id in their name) by their `.rec` file's start time. The raw files stay
as they were. The page's Boot dropdown shows the folder name and the snapshot count
(the date is in the name); the takes page and the 3D viewer show a take as
`<folder>-<n>`. Decided with Daniel: no special case for two boots in the
same minute (an error).

## 18 Oct 2026: boxes from an outside agent

`robot detect_save <n> --by=<name> '<json>'` (OBJECTS_PLAN.md, "Boxes from an outside
agent"): an agent outside the server reads `robot camera n`, finds the boxes itself
and saves them. `robot detect` keeps its `report_objects` tool call and now saves
through the same code. The CLI passes `--name=value` as a query value. Decided with
Daniel: named after detect; n required; newest boot only; no command for the rules.
Host tests pass; not run against the server yet.

## 18 Oct 2026: the firmware's camera focal length

`FOCAL_PX` in `picoA/app/camera_motion.c` is 188, was 160 (from the table's width,
OBJECTS_PLAN.md). It only sets the bearings of the camera's movement blobs, which are
only printed in the log while watching; the robot turns by the ToF alone. Not tested
on the robot.

New: [CAMERA_PARAMS.md](CAMERA_PARAMS.md) and [TOF_PARAMS.md](TOF_PARAMS.md), each
sensor's numbers next to its data sheet's. Fixed in the docs: the camera is subsampled
by 4 with no binning (binning gave stripes), not "subsampled and binned". Our f (188
px, 2.71 mm) is close to Arducam's 2.59 mm, not to its 64° diagonal; most likely the
lens's barrel distortion, not checked. The objects plan's next phase: the ToF gives
the distance; the bottom edge from the pixels and the tilt calibrating itself are a
dead path (Daniel).

## 17 Oct 2026: objects from the camera, first step

Planned in [OBJECTS_PLAN.md](OBJECTS_PLAN.md) with Daniel; inspiration in
[inspiration/](inspiration/). The robot server moved from port 8080 to 8081 (8080 is
the local model server now). Daniel measured the camera again: 8.8 cm up, 2.5 cm
ahead of the centre, 0.5 cm right; maybe a few degrees off level.

Checked first: Gemma 4 gives boxes as `[ymin, xmin, ymax, xmax]` on 0-1000 (Google's
Gemma docs); the camera's field of view isn't settled by the data sheets (DFOV 64°
gives 53.1° across, the 2.59 mm focal length 48°; the inspiration doc's 51.2° split
64° wrongly). Tried by hand on snapshot 31 of boot 7a587226: llama.cpp's router
passes tool calls through; with thinking ~14 s, without ~1 s (but the sofa missed).
A sofa's bottom edge sits on the horizon at any distance: its front doesn't touch
the floor, so the prompt asks for where the legs or base meet it.

Built: `robot detect [n]` and `robot objects [n]`, the tables, the overlays on the
page (ARCHITECTURE.md, the PC, `objects/`). The first real run (snapshot 31, 13.8 s):
the pouf as `object` at 229 cm and 94 cm wide; the ToF zones on it read 126-143 cm.
So the camera looks ~2-3° further down than the pose says.

Calibrated the same day, from the floor contacts read from the pixels (not the
model's boxes): the pouf in snapshot 31 (ToF ~124 cm, 35.5 cm high over 52 rows)
and the table in snapshot 1 (Daniel: 178-179 cm). The lens is f 180 px (~48°
across, as its 2.59 mm says), the camera looks 1.8° further down than the robot's
pitch (−1.78° and −1.88° from the two). After: the pouf at 131 cm, 48 cm wide. The
table still at 340 cm instead of ~185: the model put its bottom edge 3.6 px too
high. At 2 m one pixel is ~25 cm, one degree of tilt ~60-130 cm. So no more
calibration by hand: next, the bottom edge from the pixels, the ToF's distance
(objects marked ToF verified and moved), and the tilt calibrating itself
(OBJECTS_PLAN.md). The ToF's position measured again: 3.2 cm ahead of the centre,
2.6 cm right (was 2.5 and 3 in the firmware); the empty-room map test counted the
wall cells on one side of a cell boundary only and failed by luck with the new
position (15 of 20); it now counts either side (20 of 20 with both positions).

## 14 Oct 2026: the command API and the `robot` CLI

Later the same day, after Daniel tried it: the answers became JSON with the same
fields every time (`command` as typed, `result`, `why`, `pose`, `moved` and
`rotated` for every move, `snapshot`), no ids and no "queued" lines; an obstacle
stop is its own result, not a failure. Each action became its own command (`robot
snap`, `stop`, `clear_map`, `recording`, `mode`); `steer` takes only the moves, and
the snap after each move is a setting (`robot config auto_snap`). `status` is only
the robot now; the old snapshot summary went. `tof` gives one matrix per reading,
`light` lux, CIE and rgb (the colour name was made up and went). The robot's done
record says `rotated` too. Camera pictures moved to `recordings/camera/`, map
pictures go to `recordings/map/` (a snapshot is more than its picture). The text
map's readability and a labelled PNG of the whole map are open (CLI_API_PLAN.md).

Built from [CLI_API_PLAN.md](CLI_API_PLAN.md), so an AI agent can steer the robot
and read what it saw. The keys went: PicoA takes one command per line (drive,
rotate, goto, watch, snapshot, clear_map, recording, mode, stop, status), numbered
by the server, and answers each with a done record when it has ended; every action
ends exactly once, also when it can't start. Snapshots come only when a command
asks (the API asks after every move) and when a server connects; the robot starts
in manual. The scan (now `r390`), record 5 s, the direction switch, WiFi, help and
the text prints are gone. PicoB measures the largest jolt (gravity taken out) and
tilt between reports (link version 5); the pose line carries them, with recording
on/off, and the server stores every pose line: the path driven, on the map.

The server's API (`pc/robot/src/api/`, Hono, under `/api/robot/`) has one queue for
the page and the CLI; anything that doesn't end ok empties it. The page steers
through it (boxes for drive, rotate and watch; the map click is a goto), and its
WebSocket only brings the log, which shows everyone's commands and answers. Reads:
status, snap, light, features, tof, camera, map (60 × 60 characters, or a PNG),
log, queue; `--check` says what d, r and g would do on the newest map, with the
robot's own obstacle check in TypeScript, tested against the C one on the same map.
Found on the way: done in floats, "nothing in the way" can come out a hair short of
the distance asked (16 % of the steps of a simulated drive), so the check asks
whether anything was found instead of comparing the two. The robot's own flag is
set at the end of a move, when what is left is already ≤ 0, so it isn't hit.

## 13 Oct 2026: second drive (boot 5b0d72ab)

With the floor fix and the connection changes: "the ground feels more solid now
and the legs are still detected well"; no "nothing from the robot" notes, no
dropped connection (30 takes, 0-6 % datagrams lost). Found: the scan at snapshot
25 cleared a diagonal wall at (-7, 262)-(28, 288) cm. Every take measures that
wall's surface at the same place (no drift); scan 24 saw it at ~19°, and its rays
slid along it through the free corners of the wall's cells, ending further along:
22-25 frames passed those cells, 0-2 hit them, and 3 in a row clear a cell. The
old code does the same (7 → 1 columns in a replay of that scan).
Tried and dropped: a pass clears an occupied cell only within 3 cm of its middle.
The wall then kept 7 of its 9 steps of 10 cm through that scan (instead of 3-5), but
someone walking past a standing robot stayed on the map for good (no ray from there
went through the middle of one of their cells). At the wall's (28, 288) end the scan's
rays went through the cells' middles and 24 readings ended ~15 cm behind the wall, at
about (36, 304) cm, a spot no other position saw: the wall may end earlier there, or
that scan's heading was a few degrees off (not settled).
Also: when the robot connected, the page kept showing the previous drive's map
until an action: it switched to a new boot only while "live" was ticked, and
browsing the old drive had unticked it. Now a new boot's first snapshot always
takes over the page and ticks "live" again. Later the "live" checkbox went: the page
is live on the newest snapshot of the newest boot, and the snapshot row keeps its
title and slider in place (1/3 and 2/3 of the panel, so the slider doesn't shift as
the number changes; the checkbox had made the panel scroll sideways).

## 13 Oct 2026: the floor layer adds up its rays again

Daniel saw on the 3D map that the robot had driven over ground not marked as
floor, while on the 2D page it always drove through green cells. Both were right:
of the 128 cells the go-tos drove over, 9 were "no floor" in the snapshot before
the drive and none right after, but 49 by the last snapshot. Frames from
elsewhere overturned floor the robot had stood on. The cause was the morning's map
change: the ground layer took the strongest ray too, so the few steep rays of a
reading that end a little below the floor (one length for all 9 rays) outweighed
the rays that hit it. The ground layer adds up its rays again, as before (q over 9
rays' worth). Replay of the drive (boot 31bcd26e): "no floor" columns 810 → 408
(old code 275); of the 109 cells under the robot's path 7 end as "no floor" (41
before, 0 with the old code). The rest of the difference is the per-target status:
floor zones reading "beyond the floor" (glossy floor reflections) count fully now.
Walls, objects and legs unchanged (443 blocked, 83 overhang; snapshot 35: `f`
stops at 43 cm). Also: a detection too far to weigh anything (beyond ~2.7 m) no
longer keeps that frame's passes out of its cells. The per-frame arrays are 16-bit
again (the RAM saving of the morning is gone).

Also from the replay: blocked or overhang columns ≥ 1 m from where the robot drove
are 276 (old code 16), mostly the second targets: 38 % of readings have one,
mostly 0.5-1.5 m behind the first, and behind a wall nothing ever clears them
(bands up to ~8 cells deep). Daniel keeps them: the thick walls don't bother him.

## 13 Oct 2026: a drive with the new map; the server stops dropping a slow robot

Daniel's drive (boot 31bcd26e, recording on, 27 takes: 7 scans and 20 go-to clicks,
three of them stopped by an obstacle): "the map looks a lot more solid now, small
obstacles are detected". The room was rearranged and the route different from 12
Oct: no 1-to-1 comparison with that drive; the replay of this one, old map code
against new, is still to do.

The one problem: from 8:42:41 to 8:43:44 the server dropped the robot 7 times
("robot silent for 6 s"), each new connection lasting 6-8 s; then it worked again.
The recording shows why. The signal was fine (-48…-74 dBm) and the robot heard the
server's `\0` at least every 2 s, but at the start of take 7 (the first after the
drops) 11.4 KB sat unacknowledged for ~2 s with the congestion window halved and
the server's window wide open: packets from the robot were being lost, not a busy
server. TCP then waits longer between tries, and nothing arrived for 6 s. Each
drop made it worse: on the new connection the robot first sent its kept text and
the whole snapshot again (31-40 KB), the pose records queued behind it on the
same stream, and 6 s later it was dropped again. The cut pose records ("ual -",
"775.0 manual -") came from those drops. Daniel: a slow link isn't a silent robot,
so say what is true, and why a separate heartbeat? Changed:
- **The server** says after 6 s "nothing from the robot for N s; the connection is
  open (a slow link, or the robot is gone)" (log, and an amber status pill counting
  up), and "data from the robot again, after N s"; it closes only after 30 s.
- **The robot** closes only after 30 s without the server's `\0` too, and sends no
  heartbeat: the pose record (4 a second) carries the signal and says it's there.
- **Snapshots are paced:** at most two chunks (2.8 KB) unacknowledged when the next
  is added, so a pose record waits behind no more than that (before: up to 11 KB,
  lwIP's whole send buffer).
Host tests: a quiet server kept 30 s, then dropped; at most two chunks in flight;
the signal in the pose record; on the server, nothing for a while reported, data
again, then closed. Robot test: README "A slow link".

## 13 Oct 2026: the map: thin legs (snapshot 35)

Snapshot 35 of 12 Oct carries the VL53 frame, its pose and the map, so the robot's
`cell_map.c` was run on that one frame on the Mac (no full telemetry of that drive).
**Why the legs were missed:** leg 1 (row 4 col 2, 537 mm, status 9) put 2.95 rays'
worth of hits into its L1 cell against 18.3 of passes. Six neighbouring zones' rays
went through the same 10 cm cell beside the leg, below its top, to the wall about
1.5 m behind. The cell was judged free, and the cells were "free 15" on the
robot's map. Leg 2's L2 cell (row 3 col 3, 647 mm, status 5) won its frame (8.8
against 6.8) but only added a doubt. Two more things made it worse: a reading's
hits were split by how its 9 rays happened to fall into cells (leg 1: 6 in one
cell, 3 in the next; a 5/4 split could never have overturned a free cell), and
every reading's weight came from its zone's *first* target's status. That first
target is mostly the sensor's echo at ~30 cm (status 12 or 14): nearly every reading
counted half.

Discussed with Daniel and built:
- **A detection beats the passes beside it:** a cell a detection marks gets no
  passes in that frame.
- **A detection marks every cell its rays end in, as sure as the whole reading**
  (closeness × status), not by its rays' shares. Passes also count the strongest
  ray, not the sum (summed, many weak far rays wiped the walls), and as sure as
  where the ray leaves the cell (from where it enters, a pass beat a hit in the same
  cell by a hair at every tie: walls 6 of 20). A cell's q is now as sure as the
  best reading in it, `max(H + P, 1)` instead of 9 rays. The per-frame arrays are
  8-bit: ~21 KB of RAM freed.
- **The second target:** the rangefinder keeps the next sure target behind the
  first (the wall behind a leg), and it is detected the same way. Each target has
  its own status now. The frames export (`npm run frames`) adds the second targets
  (64 more pairs per line; old files still read). The recorded scans in
  `picoA/app/test/data` were exported again.
- **The hit height is kept while a cell is doubted,** not only while occupied: rays
  passing over a low box before it was detected kept clearing it (the 7 cm box
  test: 2 of 4).
- Tried and dropped: "a detection already explained by an occupied cell in its
  footprint marks only that cell", meant to keep a wall seen up close thin when
  driving away. It lost the hallway box (an occupied cell in front of the box's
  face explained its readings away), and without it the drive-away test doesn't
  thicken the wall anyway (8 columns before, 8 after): far readings are under
  q 0.3 and need 3 frames in a row. Also dropped: "an obstacle verdict replaces a
  free cell at once", because it would let one frame's blip into the long-term map.

Results: on snapshot 35's frame with the robot's map as it was, three frames like
it make both legs' cells occupied (leg 1 at 7/15 in both its cells, leg 2 15/15 in
L2, an overhang). `f` from where the robot stood then stops at 43 cm instead of
driving 147 cm. New host tests: a 2 cm leg driven at from 2 m, on a cell line and
in a cell's middle, blocked, `f` stops at 50 cm; a wall seen from 40 cm doesn't
thicken when the robot drives away. All five recorded scans find the cup, box,
chair and cupboard. Scans 4-5 now use scan 3's chair region: same start, and the
chair's base is mapped at −75…−95 cm, outside their old 2-cell region at −60…−70
cm. The old map code with the corrected status loses the cup in scan 1 (`f` would
go 150 cm); the new code finds it. The watch test's 25°/s walker overshot 3.6° (its
known 1-4° spread: the map moved an earlier stop by 2 mm); its limit is now 4°.
Robot test: README "Thin legs on the map".

## 13 Oct 2026: viewer, a shorter snapshot header

The side panel's snapshot header is one line, "Snapshot #33", beside the timeline;
why, the last action and how it ended, the mode, what ran and when it arrived moved
to "About the snapshot" at the bottom (the boot and the date are in the Boot
dropdown already).

## 12 Oct 2026: first run with the viewer (boot 22a46c41)

Daniel drove the robot around the room for ~20 min, almost only by clicking on the
map in manual mode, with a scan now and then: 53 snapshots, 16 tiles of the
whole-house map. Go to arrived within 0-1 cm by odometry every time; one stopped
before an obstacle as it should. Looked good in general. Found: **the map misses
thin legs** (snapshot 35: the VL53 saw legs at 54 and 65 cm, the map has those
columns drivable at full confidence; TODO.md, Next); vision takes ~60 ms a frame,
not ≤ 25; the server dropped the robot twice for 6 s of silence; a cut pose record
showed as text after the reconnect.

## 12 Oct 2026: built: snapshots, the viewer that steers, the moving map, modes, go to, camera features

The five steps agreed below, built in one go and tested on the Mac; nothing on the
robot yet (README: the new robot tests).
- **Snapshots** (`snapshot.c`): ~31 KB after every action, on `S` and for a new
  server, over the console's TCP connection in chunks between the text
  (`wifi_console`), sent again whole after a lost connection; `"\001pose …"` 4 times a
  second. The colour sensor's driver now hands out readings (the bring-up builds
  its USB packet itself) and the app reads it; the camera driver gives the last
  frame to other readers (`camera_last_frame()`). The recorder's ToF and geometry
  layouts are shared, so the PC parses them with the same code.
- **The map follows the robot** (`cell_map_follow()`), by 2 m tiles, over the new
  **whole-house map** (`house_map`, 4-bit columns, 100 tiles); queries outside the
  window read the whole-house map. Replaying the 11 Oct drive (boot 54564d43)
  through it, the window scrolled out and back as the robot went 1.7 m to the left
  and returned. One thing learned on the way: a zone with nothing in range clears
  only 1 m, so a removed box is erased only where a wall behind it gives rays to
  pass (the hallway test needed an end wall).
- **Modes** idle and manual (`mode`); space goes to idle, an action key to manual.
- **Go to** (`behaviour_goto()`): turn, drive, steer, stop at the point or 15 cm
  before an obstacle; at most 3 m; host-tested to < 1 cm in the simulated room.
- **Camera features V1** (`vision`): FAST-9 + Shi-Tomasi, ZNCC coarse to fine,
  Lucas-Kanade for the sub-pixel part. Three changes from the plan, each found by
  the synthetic tests: a parabola alone locked moves to whole pixels (0.25-0.4 px
  RMS at fractional shifts; Lucas-Kanade steps brought it to ~0.06-0.13);
  refreshing templates every frame let a still scene's features wander (σ 0.2 px
  over 1000 frames, 74 % kept; keeping a template while it matches: σ 0.06, 85 %);
  and without the gyro's hint, features moving 6 px a frame or found again after a
  miss were looked for in the wrong place (the window now sits on each feature's
  own last move, ±8 px). A best place on the edge of the search window counts as
  not found (near the image's edge it was often the wrong one). Not met: two of
  the plan's limits (TODO.md, small). 0.2 ms a frame on the Mac.
- **The viewer** (`/`): the map at any snapshot (window over tiles, the old dimmed),
  path, ToF points, the camera picture with features, the ToF grid, colour, pose,
  console, Idle / Manual / Stop, click to go. Checked in headless Chrome against a
  fake robot (`npm run fakerobot`) streaming the replayed drive: a click in manual
  sent `goal -4 -78` and the fake robot drove there; in idle the goal was refused.
- **Found on the way:** the server dropped the space key (it took only `!`-`~`), so
  the console page's Stop button and space key never reached the robot over WiFi.
  Fixed.
- RAM free 194 → 103 KB (snapshot buffer 38, vision ~30, whole-house map 20); stack
  6.5 KB of 8.

## 12 Oct 2026: decided: snapshots over TCP, a viewer that steers, a moving map

Daniel wants to see what the robot does before building the modes, so the order is
now snapshots, mode switching (idle and manual), the moving map with the 4-bit
whole-house map, drive to a point, then camera features (V1). After each action
the robot sends a ~30 KB snapshot over TCP (pose, camera frame and features, one
VL53 frame, the colour reading, the 6 × 6 m window with its anchor and the changed
column tiles); the full UDP telemetry stays as a switch, off at power-up. The
window moves by 2 × 2 m tiles and is the truth over the columns wherever it knows
something; unknown keeps the old column. One PC page shows the map, any past
snapshot and steers by a click on the map (manual mode, live view only; the robot
computes the turn and distance itself). Manual steering became mode 7. Details in
TODO.md.

## 11 Oct 2026: docs cleaned up; the VOTES map removed

The plans had drifted from the code (status lines saying "nothing implemented",
step orders overtaken by the rework), so Daniel asked for a fresh start: every
plan, the old README companions (ROBOT_PLAN, REDFLAGS, REFACTORING, ROBOT_WIFI) and
the camera stripes notes went to `doc/archive/` unchanged; `doc/ARCHITECTURE.md`
now describes the robot as built (written from the code, with Daniel's decisions),
`doc/TODO.md` what is left (big features and ideas apart from small fixes), the
README how we work. The two changelogs (before and during the rework) are this one
file. Before that the map's VOTES variant was removed (READINGS has been the
robot's map since 10 Oct): one map in the replay tool and the viewer, the floor
margin still settable for replay.

## 11 Oct 2026: robot test of `f` with the map: a drive around the room

Boot 54564d43, recording on, real furniture (no cup or box): a scan, then 34 turns
and moves around the room, a scan at the end (36 takes, all stored). Four moves
stopped "obstacle" (moved 8, 22, 30, 17 cm), one didn't start ("closer than 15
cm"), the rest went 50 cm; nothing was touched. Daniel: the stops were 15-25 cm
before the obstacles, looked good. He felt the robot "stop and then accelerate"
sometimes: the drive commands show it in takes 16 (10 → 3 → 10 → 3 cm/s) and 27
(3 → 10 → 3). Replaying the drive through the map: a cell one in front of the
furniture was an overhang, seen from afar (in take 27 with full confidence), and
closer readings erased it, so the room left grew 5-8 cm and the speed, set by the
room left alone, rose. Suggested: no speed-up after braking unless the room grows
more than 20 cm. Daniel: the braking is fine, keep it. Datagrams lost: 0-19 % per
take (the worst on moves 19, 20).

## 11 Oct 2026: `f` stops before obstacles on the map; the whole drive in the viewer

Daniel accepted the suggestions for M4 step 1 (REWORK_PLAN "Next" 3) and asked
whether the stop uses the map the robot keeps: it does, a query on `cell_map`
(`cell_map_obstacle_along()`), no second map. Built in `behaviour.c`: the room left
is checked every loop while moving, the stop point slows the robot like the end of
a move; within 15 cm it doesn't start; back, a note that the VL53 can't see there;
the take ends "obstacle" (TAKE_END reason 7, the PC knows it). Host test: the plain
moves in `test_behaviour` drove into the low box, now found by the map, and stopped
(correct): they now go 30 cm towards the side wall; a 20 cm box put down after the
scan stops `f` 20 cm before its face (15 cm before its cell), `f` again doesn't
start, back stops 25 cm from the wall (its hits fall in the cell before it). The
watch test then failed (3.4° past the 25°/s walker, limit 3°): the moves shift the
random numbers; seeds 4 and 12 fail on the old code too, so it is re-seeded before
watching and the margin noted as open. No recorded drive exists yet; on the five
recorded scans `f` aimed at the cup finds the cup first (41-45 cm from the centre),
now part of `test_map_replay`; low-confidence cells changed no stop there.
Daniel wants to drive around the room and see the map in 3D: the viewer showed one
take at a time, each replayed from an empty map, so it gained the **whole drive**
(`<boot>-all`): every take of a power-up on one timeline (each take's times moved
by the robot's clock, across its wrap), replayed into one map. Checked on boot
0558a0ee (3 takes, 107 s, 1167 frames) in headless Chrome. The 6 × 6 m window
limits the replayed cells too (the same code), not the points. TypeScript 7.0.2
added as a dev dependency (Daniel): `npm run typecheck`; it found the drive view's
lost types and one older Buffer type in `recording.ts`. Then linters (Daniel):
ESLint 10 with typescript-eslint, which supports TypeScript only below 6.1, so
TypeScript went back to 6.0.3 (7.0 is the new native compiler); and cppcheck 2.22
(Homebrew) for the C. `tools/lint.sh` runs all three. First findings, fixed: four
printf `%u` given an int, two test buffers cppcheck saw as uninitialised (one a
false positive), an unused value and a ternary as a statement in the server;
the host tests' 40 asserts on calls are allowed.

## 10 Oct 2026: end of the day; `f` will stop before obstacles

Daniel: `f` keeps moving 50 cm, but stops 15 cm before an obstacle on the map (M4
step 1; recorded in COMMANDS_PLAN and REWORK_PLAN "Next" with the points to settle
before coding; not coded). T6 dropped: done by the day's analysis. The telemetry
plan is done; next is M4. The handover for the next session: REWORK_PLAN "Next".

## 10 Oct 2026: the recorded scans as a regression test

`test_map_replay`: the five scans of 10 Oct (frames in `picoA/app/test/data/`)
through the map; READINGS must find the cup, box, chair base and cupboard in each
(where Daniel put them) and block nothing within 30 cm. Passes; VOTES misses the
cup in 2, the box in 3, the chair in 2. Cost on the Mac: READINGS 1.1-1.6× VOTES
per frame on average, the worst frame the same. On the robot (a scan, recording on):
the longest loop iteration 29.2 ms (`surroundings` 27.3 ms; VOTES ~22 ms), mean 57
µs: fits (frames every 66 ms, DRIVE every 50 ms). The link's "21 lost" didn't
change during the scan (likely at power-up, while the ToF firmware loads).
Daniel: `f` keeps meaning "move 50 cm" (what it does with something blocked ahead
is for M4); driving already feeds the map (every frame, placed by odometry).

## 10 Oct 2026: T4 and T5, the viewer and replay

Built: `/viewer` (three.js 0.186.1 from npm, served locally; first from a CDN
at an old version, changed on Daniel's word; DuckDB 1.5.6-r.1 and ws 8.22.0 to
their newest too): a take's targets in the room coloured by row,
signal, status, height or target, filters, click a point for its data; the
robot's path; a time slider with the robot and its rays; the take replayed in
both map variants as cubes (or their difference); truth boxes per boot and each
variant's score (found, cells not explained). Server: `viewer_api.ts`,
`replay.ts` (builds `tools/replay_map.c` with `cc` when the map code changes; the
tool gained `--json`); `takes.geometry` and `truth.truth_id` added (an import
again fills the geometry). Tests: scoring, truth boxes, a take's view and both
maps replayed from the firmware's sample take. Checked in headless Chrome: the
last scan with the cup, box, chair and cupboard; a test truth box on the cup
scored "found" by both variants (removed again).

## 10 Oct 2026: the robot runs the READINGS map

Robot test (take `0b9377b7-1`, a scan, then `m`): the cup (front-right, row 5),
the box, the chair base and the cupboard on the map. Replayed on the PC, the take
gives the robot's printed map: blocked cells the same 441 of 441 within 1 m (the
whole 4 × 4 m 1555 of 1560: the robot also mapped frames before the take; its
printout was centred one cell over, the robot ending a hair right of 0). Daniel:
the `?` beyond ~1 m (waxed-floor reflections) are fine, the floor isn't sure there.

## 10 Oct 2026: the cup in replay; the READINGS map variant

Replay of the four scans through `cell_map.c` (tools/replay.sh): the cup's L1
cell loses to rays of rows 3-4 passing over it, and a third of row 5's rays end
in G; it is on the map in scans 3-4 (one cell), not in 1-2. Daniel's rules,
built as a second variant (MAP_DESIGN §9): a reading judged as a whole by where
its cone meets the floor (closer: an obstacle, all of it in L1 or above;
straddling: unsure), and a ray only clearing what it would have hit. Tried and
changed while building: hits weighted by the ray's full length (walls 20 %
weaker: now as VOTES); unsure readings giving nothing (floor seen in 60 cells
instead of 129: now their floor-height rays mark floor); passes compared with the
lowest hit (a moved chair leg stayed 30 frames: now the highest). Result: the cup
in all four scans, the 7 cm test box blocked, no false obstacles in the simulated
rooms; the cupboard still missing; cells right of the robot to check with
Daniel. The robot still runs VOTES.

## 10 Oct 2026: T3, the recordings in DuckDB

Built: `store.ts` (the schema of TELEMETRY_PLAN §3; a take imported as it ends,
or again: rows replaced, note and truth kept; each target placed in the world as
`rangefinder.c` / `cell_map.c` do, with the pose interpolated between odometry
reports), `import.ts` (`npm run import`: every take of every raw file again),
the take list page (`/takes`, notes editable), `@duckdb/node-api` 1.5.5-r.5.
Decided while building: the raw per-boot files stay the truth; no per-take copies
in day / boot folders as first planned. All twelve takes of 10 Oct (three TCP
runs and the UDP run) imported in 1.4 s: ~650 k targets.

Found (rows 1-8, top to bottom): the floor rows land where they should (rows 7-8:
z −0.5…−0.3 cm). Row 5 gives no valid return from bare floor under 1 m: all of
them were the box (30 cm), the cup (40-48 cm, row 5 only, in all four scans) or
the cupboard behind; the cup's signal is no stronger than other returns at that
distance. (First read as "row 5 sees the floor at +4.7 cm": wrong, those were the
cupboard.) Rows and columns are numbered 1-8 everywhere now (README
"Conventions"): the console's zone lines, the database, the plans (TOF_MOTION,
SURROUNDINGS, ARCHITECTURE, cross_plan counted from 0 until now; their numbers
moved up by one).

## 10 Oct 2026: T2 over UDP on the robot: done

`2026-10-10-18-08-07-boot-0558a0ee.rec`: no stalls. `5`: nothing lost. Scan: 214
whole frames, 7 half, 5 missing of 226, 3.0 % of datagrams lost. Watch: 877
whole, 28 half, 7 missing of 912, 2.3 % lost, 9 refused on the robot. Losses
mostly single datagrams (31 of 39 runs, at most 6 in a row); odometry gaps 9 in
all, 80-221 ms. 24-28 KB/s (was 30-36). The losses depend on the heading (this run
4-9 % between 0 and 135°, none facing 315-360°): the antenna over the batteries.

## 10 Oct 2026: T2 third run: why it stalls; UDP and a compact format

The third run (`2026-10-10-17-42-34.324.rec`, with `WIFI` records): stalls of 4 s
in the scan and 3 s while watching. The signal follows the heading: −56 dBm
facing one way, −63…−68 over about half a turn (the batteries sit 1-2 cm under
the Pico's antenna); in each stall TCP's window collapsed (25 → 1.5 KB) and it
waited 1.5-3 s for retries; nothing was ever refused on the robot, and the
console's text and heartbeats kept arriving during every stall. So: a few lost
packets, and TCP holding everything behind them.

Daniel: send less, and use UDP. Done: record version 3. The sensor's ambient,
SPADs and signal as u16 (measured ranges far below; capped at 65535), a frame
~1.45 KB instead of ~2; each frame as two halves of 32 zones; datagrams of at
most 1400 B with a number, small records within 50 ms, each ODOM repeated in the
next datagram, the take's first datagram sent twice and its last three times; no
retransmission. The TCP recording connection and its 48 KB buffer are gone
(RAM: recording costs 4 KB, ~132 KB free; lwIP back to 32 KB). The PC listens
on UDP 4212, saves every datagram with its arrival time in a file per boot, keeps
one of each repeat, counts lost datagrams and half / missing frames per take, and
still reads the TCP files. Tests: `test_recorder` (datagrams, halves, caps,
repeats, refusals, the 50 ms flush), `test_wifi_console` (datagrams to the
announced port), Node (both formats, the firmware's own datagrams, the UDP
server).

## 10 Oct 2026: T2 recheck: longer WiFi stalls; measuring them

The recheck (`2026-10-10-17-32-19.506.rec`): odometry complete outside the stalls
(249 of 250 in 5 s), but the connection stalled completely three times in the scan
(6 s worst; 123 frames and 252 other records left out) and once while watching (3
s, 44 frames): even ODOM is lost once the 48 KB buffer is full. The first run had
one 0.9 s stall. The headings during the stalls show no single bad direction. Two
causes fit: the WiFi link (signal, interference; TCP waits 0.5, 1, 1.5 s... after
each loss) or the WiFi chip refusing packets at this rate. Added to tell them
apart: a `WIFI` record every 0.5 s during a take (RSSI; what waits and what TCP
holds, retries, timeout, windows, refusals, for both connections; record version
2), and on the PC `<file>.arrivals.csv` with when each piece of both connections
arrived.

## 10 Oct 2026: T2 robot test, and two fixes

The first recording (`pc/robot/recordings/raw/2026-10-10-17-21-17.795.rec`): `5`
(5.0 s, 76 frames), a scan (15.0 s, 214 frames, 13 left out in one 0.9 s WiFi
stall), a minute of watching (60.3 s, 913 frames, none left out). 30-36 KB/s, ~2
KB a frame (most zones report 2+ targets), no CRC errors, nothing cut; the robot's
counts and the decoder's agree; odometry kept flowing through the stall, as meant.

Found: 5 % of the odometry reports missing (always one at a time): only the latest
report per loop iteration was taken, and iterations take 10-20 ms, sometimes over
20 (drive commands go every ~65 ms instead of 50 for the same reason; harmless
against PicoB's 250 ms). Fixed: `body` keeps the last 16 reports numbered, `pose`
and `recorder` take every new one (tests in `test_body`, `test_recorder`). The
send ring 16 → 48 KB (~1.5 s of stall); RAM ~60 KB free.

## 10 Oct 2026: T2, recording on the robot (TELEMETRY_PLAN)

Daniel's answers: a decoder on the PC in T2; `5` as COMMANDS_PLAN has it; leave
out ToF frames when the WiFi is behind (a 4 KB reserve), no separate WiFi speed
test: the robot test measures it; the latest ODOM right after the take starts; the
32-bit clock stays (runs are shorter than its 71.6 min wrap); `GEOMETRY` doesn't
send each ray's direction (the PC computes it; send as little as possible); a
connection lost during a take cuts it, one that comes up during a take doesn't
record the rest; commits carry no AI co-author trailer (written into the plan).

Built: `recorder.c` (takes, the seven record types of TELEMETRY_PLAN §2.1, CRC-32,
the drop rule, the console's lines taken from stdout as marks, `R`, status in
`p`); `behaviour` starts and ends a take with each action and says how it ended,
and has the record-5-s action (motors off, without PicoB too); `wifi_console`
keeps a second TCP connection for recordings (the two connections share one
`conn_t` code path) with a 16 KB ring, its port from the announcement
(`ROBOCAR-SERVER 4211 4212`); `rangefinder` numbers its frames and gives the raw
results and the zone order; `body` gives the drive commands sent. lwIP memory 32 →
56 KB, TCP segments 32 → 64. On the PC, `recording.ts` (listener on 4212, raw bytes
saved per connection in `pc/robot/recordings/raw/`, decoder, a note per take with
frames, gaps and KB/s) and `decode.ts` (`npm run decode`). Host tests: new
`test_recorder` (every record decoded and CRC-checked; frames left out while
odometry still goes; a cut take; R off), `test_wifi_console` (the recording
connection), `test_behaviour` (a take per action with its end reason; record 5 s);
Node tests decode the firmware's own bytes (`build/test_recorder.rec`). RAM +44
KB (~92 KB free).

Found: a ToF frame is smaller than the plan's 3.1 KB: only the targets found are
sent, ~1.2 KB with one per zone (0.6 KB with none).

## 10 Oct 2026: T1, the commands (COMMANDS_PLAN)

Daniel's answers: stop on space; one turn key (`t`, right in back direction);
motors off as much as possible; `C` clears the map; watching lasts a minute; when
the WiFi can't keep up, ToF frames are dropped and the data must say so; USB and
WiFi behave the same.

Built: the robot is idle with the motors off at start-up and connects to WiFi with
or without USB (no start-up scan, no scan after WiFi). `behaviour` has four actions
(scan 390° only, move, turn, watch 1 min), each switching the motors on at its
start and off at its end, printing "done" with what odometry measured or "stopped"
and why; a new action replaces the running one. The console: space stop, `s`, `f`,
`t`, `r` (direction), `a`, `C`, `p` (with link counters and the direction), `W`; gone:
`g`, `n`, `l`, `q`, `d`, `b`, `w`, the last test result and the 15 s status line.
`robot_test` and its test are removed (moves + telemetry replace them), and with
them `pose_set_origin()` (the map's frame is odometry's from power-up); the most
open direction is gone. The page's buttons follow the keys. Host tests:
`test_behaviour` checks idle, scan, turn ±30°, move ±50 cm, stop, replacing,
clearing and the 1-minute watch.

Found: the behaviour test's 8 cm box at 40 cm was found with 3 of 6 random seeds;
the old test passed by luck of its seed. It is the cup problem (MAP_DESIGN §8.1),
so the test reports it as KNOWN instead of asserting it.

Robot test (Daniel): every key as expected. Scan 14.9 s, +390.8°; moves +50.2-50.3
cm, 0.0-0.1° off; turns +30.5-31.1° (back direction −30.5°); keys pressed while an
action runs replace it (the new one counts from where the robot is then).

Then `v` removed (Daniel): the movement lines are printed only while watching;
idle, nothing is printed (the camera's lines came while idle). Telemetry will record
them as `MARK`s during a take.

## 10 Oct 2026: the cup and the box; plans for commands and telemetry

Daniel's runs: a glossy 6.5 cm cup ~45 cm ahead, a bit left, was on the map once
(one cell, 40 cm ahead, 20 cm **right**); a second scan with the cup and the 7 cm box
to the right showed no cup, and the only new obstacle was 50 cm to the **left**,
beside the robot. The room itself was right in both (not a mirror: the zone order,
the yaw sign, pose timing and the print all check out). The placements are not
explained. Between the runs the robot was turned back by hand (yaw 464.6° → 358.9°),
so both maps share one "up".

Simulation (scratch, not committed), the cup's 3-13 cm cell over a 390° scan: the
cup loses the vote in every frame at 25-60 cm (hits 13-23 % of the passes); ~80 %
of the passes go over the cup inside the layer (rows 3-4 from 1, at 7-13 cm).
Making hits count 4× kept the cup but brought back the ring of false obstacles at
~0.7-0.8 m (as on 9 Oct). Beyond ~71 cm the 5th row sees floor and cup in one zone;
what the real VL53 reads there is unknown (the simulation's nearest-surface model
says floor).

Daniel: the fixes argued from the simulation are weak; get the real data to the PC.
New plans: COMMANDS_PLAN.md (idle at start, one key one operation, scan = 390°
only, move / turn / direction, no test-only actions, no 15 s status) and
TELEMETRY_PLAN.md (raw ToF with all targets + pose per action as a take, `.rec`
files and DuckDB, a three.js viewer, replay). Replaces R0 / M-S and the start-up
scan after a watchdog reset. No code changed.

## 9 Oct 2026 (evening): robot test of smooth following, the old map gone

Daniel's run (scan, `m`, then `a` with `v`, walking across both ways, stopping):
**no more jitter**: following was smooth and stopped facing him. The scan printed no
map; `m` printed the new one. A 6 cm box ~40 cm in front was **not on the map** after
the scan, and later seemed to disturb the tracking when he walked behind it (the
VL53 can't see someone behind something nearer: movement is only "nearer than the
background"). No fixes asked for: written down as open points in the plans.

The one-zone diagnostic answered the earlier question: the one-zone blobs at the
start of each walk were real (e.g. row 4 col 7 read 1.40 m against a 3.27 m
background: Daniel at the edge of the view); a few read against a background of 0
(directions never seen) or just under the margin.

**Tried and dropped on 9 Oct** (kept here so they aren't tried again blindly):
- *A2 started, rolled back* (Daniel: no visible progress): the ToF frame time from
  the INT pin. The current timing error (~5 ms) is 0.3° at 1 rad/s, inside the 1°
  slack of the heading background.
- *A 75-minute soak for the 32-bit clock wrap*: replaced by starting both Picos'
  clocks 30 s before the wrap.
- *Map, after the first robot run (obstacles on the right shown too far)*: (1) an L1
  cell passed only by rays through its bottom 2 cm, (2) the verdict "more than half
  of max(rays, 9)" with closeness per cell. Both made a ring of false obstacles at
  ~0.7 m (the 5th row's rays grazing the floor end 3-6 cm up) and lost far walls;
  reverted. The obstacles turned out to be a chair on wheels (thin parts, gaps).
- *Tracker: a target lost while moving fast toward the edge counts as leaving*
  (predicted direction): Daniel didn't want the robot to try to get ahead of the
  movement. Replaced by following the biggest movement.
- *Following, first version*: turn toward the biggest blob, 5° dead band, minimum
  speed 8.6°/s: stop-and-go every 0.2-0.5 s on the robot. Replaced by the follow
  controller (world directions, fitted speed, 8°/3° start/stop, no overshoot).

## 9 Oct 2026: following movement smoothly; the old map removed

Robot test of watching while turning and the new map (Daniel): detection while
turning works; the new map looks better than the old one. Turning towards the
biggest movement replaced the tracker's leave-and-jump (Daniel: don't predict, start
earlier, turn slowly); then, from the next run's stop-and-go every 0.2-0.5 s: the
direction is kept in world terms (corrects for the robot's own turn since the
frame), the movement's angular speed is fitted over 0.5 s and used as feed-forward,
start at 8° / stop at 3°, no minimum speed, the speed dropped once past where it was
last seen (no overshoot). Host test: a steady walker gets 1 start, 0.5-4.8° lag, at
most 2.9° overshoot when they stop. One-zone blobs at the edges still turn the robot
(Daniel: an animal inspects edge movement); the log now prints the zone's reading
and background to find out what they are. An unsure ToF reading no longer resets a
zone's "steady for 1 s". The old map (`world_map`, floor learning, the scan's kept
frames, the auto print, the motors-off pause after the scan) is removed: `m` prints
the new map, the most open direction comes from it. RAM free 219 KB (was 74).

## 9 Oct 2026: watching while turning, the new map

Reordered with Daniel: visible progress first, groundwork only when a feature needs
it (A2's ToF INT timing was started and rolled back: 5 ms of timing error is 0.3°
at 1 rad/s, inside the 1° slack).

**Watching while turning** (T-R, TOF_MOTION_PLAN §7): `tof_motion` keeps the
background per world direction (8 rows × 256 bins of 1.4°), fed every frame with
the turn from `pose_at` at both ends of the frame. No learning after a stop;
`find_passing()` is gone; floor rows tell nothing while turning; the bins are
forgotten after 10 cm of driving. Host tests: the still-room cases as before;
turning 5 times each way at 0.5/1/1.5 rad/s past a door edge and with the heading
3° off: no movement; someone found in the frame they enter the view while turning,
and in the 2nd still frame after a stop (was ~1.5 s).

**The new map** (`cell_map.c`, design in doc/MAP_DESIGN.md, Daniel's): 9 rays per
zone as long as the reading, passes for free, the end cell for occupied, each ray
weighted by closeness (sigmoid at 1.5 m) and VL53 status; per frame one verdict and
confidence per cell; a cell keeps its best measurement and changes for a better one
or 3 good-enough ones in a row. No floor learning, no floor assumed. Runs beside the
old map (`m` old, `M` new). Host tests: no false obstacles in an empty room or a
hallway, a box blocked, a table top an overhang, nothing beyond stairs down
drivable, a chair leg erased from 1 m, a box kept while standing 10 min 2 m away,
a walker gone after 1 frame. Found: the ground layer may only be "passed" by a ray
going through its bottom (crossing the air above the floor says nothing about it);
standing still the floor is seen in rings at each row's distance; known cases:
under a table top at 15-20 cm ~0.8 m away, 6 false blocked columns (the 4th row's
rays stop at the underside); a small hole beside floor can get a few floor hits.
RAM free 74 KB.

## 9 Oct 2026: rework step A1 (helpers, clock wrap, loop timer)

Shared helpers in `common/`: `stamp.h` (time differences), `geom.h` (`bearing_rad`,
`wrap_pi`), `stats.h` (`nth_value`, `median_value`, `sort_values`), with
`test_helpers`. They replace five `atan2f` bearings, four hand-written time
differences, `pose`'s `after()`, `behaviour`'s `wrap_angle()` and three
`compare_*` + `qsort` copies. `change_grid_observations()` /
`change_grid_biggest()` turn blobs into observations once, for both the VL53 and
the camera detector. Every host test's output was the same before and after.
The fake clock now starts 30 s before `time_us_32()` wraps (the short tests start
nearer), and every test crosses the wrap (checked by printing each test's end
time): no wrap bug found, only test artifacts (a hard-coded frame time, PicoB's
simulated report on multiples of 20 ms of the absolute time). New `loop_stats`:
`p` prints "Loop: N per s, mean, max (slowest: stage)" over the last 10 s and the
free RAM (136 KB). No robot test of its own (Daniel: no regression runs of code
that later steps replace): the first `p` check rides with A2's robot test. No 75
min soak for the wrap: both Picos now start their clock 30 s before
`time_us_32()` wraps (`common/clock_start.h`), so every run crosses it 30 s after
power-up; the movement log still prints seconds since power-up. Rule added to REWORK_PLAN: robot
tests only when a step changes what the robot does or needs the robot's numbers.

## 9 Oct 2026: rework step A0 (Measure) done by analysis

Daniel questioned A0's robot session: RAM and stack follow from the data
structures, and the old loop's stalls go away in A3 anyway. So A0 was done from the
build instead. RAM: everything is static, no `malloc` is linked, 385 of 512 KB
used. Stack: `tools/stack_depth.py` (new) sums GCC's per-function stack sizes along
the call graph, with the calls through function pointers added by hand (printf's
output, lwIP). Core 0 needs ~6 KB at worst: the main loop alone 4.2 KB, 3.2 KB of
it in `observations()` (`tof_motion.c`). That is over the SDK's nominal 2 KB, which
nothing enforces: the stack grows into the two empty scratch banks (8 KB). It works
now, but core 1's stack sits there, so V3 must set both stacks before starting core
1 (already planned). The loop timer and "RAM free" in `p` moved to A1.
ARCHITECTURE_PLAN §1.3 and §8, cross_plan and REWORK_PLAN updated.

## 8 Oct 2026: rework plans (architecture, movement, map)

No code changed. From Daniel's brain dump and an inspiration doc
(`doc/RobotMotionTrackingAlgorithms.md`), four plans were written, reviewed
together by a reviewer and revised: the architecture (today's functions as a
spec, deep modules, red flags), VL53 movement (per-reading confidence, edge times
across neighbouring zones, watching while turning with a heading background),
the map (ground as an ordinary cell layer, the cone-slice rule, confidence-weighted
log-odds, no floor learning) and the camera (features tracked frame to frame into
a record of measurements only; movement detection is one reader and removes the
robot's own turn itself). Decided with Daniel: ground band −7…+3 cm; drivable =
ground + both layers above free; camera dark rather than noisy (≤ 10 ms, gain
≤ 8); no camera images to the PC; recordings not in git; scan after a watchdog
reset. Start: [doc/REWORK_PLAN.md](doc/REWORK_PLAN.md) (order of 21 steps), terms
in [doc/GLOSSARY.md](doc/GLOSSARY.md). It replaces the "fix the misses" next step
of 7 Oct (`find_passing()` goes in T-R).

## 7 Oct 2026: M3b first iteration: turning after a target leaving the view

Agreed with Daniel: as simple as possible for the first iteration. When the target
is about to leave the view (two or more readings, at the edge), turn to where it
will be (estimated from its angular speed and the known turn rate), stop and learn;
if it is seen leaving while learning, continue at the same angular speed. No
smooth following, no turning to a target that stopped, no back to open space or
obstacle check yet. ROBOT_PLAN.md §6.6 has the sentences.

New: the tracker's "leaving" report (`TRACK_LEAVING_*`); `tof_motion` notices
something passing out through an outer column while it learns; `motion_sense`
hands both to the behaviour; `behaviour.c` watches after the scan (or `a`), motors
on, and turns at 1 rad/s (at most 90°). Host tests for all three. Found in the
behaviour test: a fast target's predicted angle beyond 180° was wrapped and the
robot turned the wrong way; now only clamped. The web page (`pc/robot/`) got
buttons for `a`, `v`, `o`, `k` and `c` (M3a's keys were missing).

**On the robot (living room, without USB, the console over WiFi; Daniel walking
around it for ~4 min):** it kind of worked. 11 turns after a target leaving, both
ways, at 10-20 deg/s (+44, −31, −31, −36, −27, −39, +32, −32, +38 deg), turns
ending within ~1° of the command (e.g. +44 asked, +43.8 by the gyro); twice it
turned on after the target left while it learned (−61, −52 deg). Missed: sometimes
it didn't notice the movement at all, and often it didn't notice the target leaving
while it learned. Agreed: good enough for the first iteration. Likely causes, to
look at next: the view is learned only once PicoB reports the robot still (~0.5 s
after stopping), so a target that left before that is never seen; a zone needs 3
sure readings to have a "farthest", so farther than the VL53 reaches (upper rows)
it tells nothing; 2 zones in one frame of the outer column.

## 6 Oct 2026 (evening): M3a: the camera's movement detection

Proposed to Daniel in seven sentences (ROBOT_PLAN.md §6.3) and agreed; key `k`
for the camera's blocks (`v` and `o` were taken: `v` now logs both sensors).
New: `camera_motion` (blocks, background and noise learned over 1 s with the
exposure held, the light taken off as the median change, > 40 % differing = the
light changed: learned again; moved by `change_grid` as on the VL53; steady 1 s =
stopped), wired into `motion_sense` (the exposure is held while still, follows the
light while moving; "Movement (camera)" lines and "the light changed"), host test
`test_camera_motion.c` (25 and 100 frames/s, 11 seeds).

Found in the host test: a small hand sweeping ~66°/s is in each block for only one
used frame and is never found by "2 of 4 frames"; kept the agreed rule (a hand
waving at ~33°/s is found the whole time) and noted the limit in the plan.

**On the robot (walking right to left and back, a hand, a second light on and
off):** both walks and the hand were found on the correct side and moved the right
way (e.g. +22° → −14° for the hand left to right). Three things: every movement
line was followed by "ended": frames skipped to keep ~15 a second returned 0
("nothing moved"); they now return −1 and are ignored (the host test checks it).
Walking past at ~0.5 m, the person filled > 40 % of the view and was taken for a
change of light (twice). A lamp lighting ~25 % of the view was movement for as
long as it was on (~2.5 s: switched off without waiting), though steady blocks
should be taken in after ~1.3 s.

Fixes: the light changed only when more than 40 % of the blocks *start* to differ
in the same frame (a person comes in over several). The host test of someone
walking up until they fill the view then found a second trap: once they cover half
the view, the median change is theirs, so every other block "starts" at once; the
light is now taken only from the blocks that looked like their background in the
last frame. Tried and reverted: "steady" measured from frame to frame, for a bulb
warming up: in the host test a lamp brightening over 5 s still kept moving
(taken-in blocks drift off again); waiting for the `k` prints instead of guessing.
A test-only trap on the way: a `static float light` in the module and in the test
were one variable (tentative definitions in C); renamed `light_shift`.

## 6 Oct 2026 (night): M3a: exposure and movement kept apart

Daniel: the exposure was changed too eagerly; adapt slowly, like an auto-exposure;
a light switched should be movement; and don't mix exposure control with movement
detection: adjust the exposure only when calm, and don't watch with the camera
meanwhile. Now: the driver waits before adjusting (5 s when 25 % off, down to 0.5 s
when far off) and steps exposure × gain by at most ×1.25; frames say when it wants
a change. `camera_motion` holds the exposure while still and lets it change only
after 5 s with nothing moving (or after 30 s of wanting), then learns the view
again. The "40 % start at once = light changed" rule and the median light
correction are gone: a light switched is movement until steady.

Found in the host test: blocks left right at the edge of "differs" after a light
switch flickered in and out of it, and each dip restarted their 1 s steady timer
(movement trickling for seconds, maybe the lamp on the robot); a block that keeps
its new value is now taken in after 1 s even if it dips now and then. The median
light correction kept moving by a few levels while blocks were being taken in,
which also delayed it; slow light is left to each block's drift. Test fixes on the
way: objects drawn with a fixed texture (redrawn noise made them flicker), the
test's clock (0.1 s steps were 80 ms at 25 frames/s), the simulated driver's wait.

## 6 Oct 2026 (last): the tracker fixed, on the robot

Four walks with the fixes: at ~0.5 m right to left and left to right, at ~1.5 m
both ways. Each kept one target from edge to edge (speeds 5-31°/s), no jumps to far
zones, and ended with the correct side ("left the view on the left at +16 deg" … "on
the right at −20 deg"), ~0.5 s after the ToF's movement ended. The tracker does what
§6.6 asks; next is turning to look (M3b).

## 6 Oct 2026 (later still): the tracker on the robot, over WiFi

At ~0.5 m: left to right followed and "left on the right" as it should; right to
left, Daniel was a 24-zone blob centred at +3° plus a 2-zone piece at −17° (his
feet), the tracker matched by centre and stayed on the piece, reported "left on the
right", then jumped to a single zone at 2.76 m. At ~1.3-1.5 m both walks were right
(speeds 13-29°/s, exit sides correct), but at the end of each the target jumped to a
far zone in the same direction (2.80 m, 2.21 m). Fixes: blobs report their leftmost
and rightmost cells; the target is the biggest blob reaching within 10° of where it
is expected, and not more than 0.5 m nearer or farther; target lines are printed
only for frames in which it was seen (one showed an old position). Host test cases
from both logs.

## 6 Oct 2026 (after that): the VL53 tracker

Daniel: implement the tracker now. `tracker.c` (ROBOT_PLAN.md §6.6): one target
among the VL53's blobs, its direction, range and angular speed, lost after 0.5 s
without a blob within 15° of where it is expected: "left the view on the left /
right" from the outer zone columns (beyond ±16°), else "stopped". `motion_sense`
logs it ("Target …" lines with `v`) and forgets it when the robot moves. Found in
the host test: the angular speed from two directions 0.5 s apart was off by up to
~8°/s with ±2° of jitter; now a least-squares line through ~0.8 s. Not yet on the
robot.

## 6 Oct 2026 (end of session): the plan simplified

Proposed next: combining the camera with the VL53 into one target, then turning in
hops, then smooth following. Daniel: too complicated; start with the VL53 alone for
tracking. Written down as ROBOT_PLAN.md §6.6 (tracker in five sentences, then
turning to look in hops); combining (§6.5) and the camera during turns (§6.4) put
off; M3a/M3b in §12 changed to match. Not committed yet.

## 6 Oct 2026 (late night): M3a: the camera on the robot, evening light

Daniel walked right to left and back, walked to the light switch, switched the
light on. The light switch: all 300 blocks moved for ~1.5 s, "ended", then the
exposure was adjusted when calm (40 ms × 10.1, mean 113); two `c` ~10 s apart
within ±1 per block; every block's noise at the 2.0 floor. Most extra camera blobs
were 14-19° below level: Daniel's shadow on the floor (the evening lamp). What
looked like the ToF ending 2-3 s late was a single 5th-row zone at 2.8 m, twice
~15 s apart: most likely his feet on the way to the switch and back. Movement log
lines now start with the robot's time in seconds, to answer such questions from the
log.

## 6 Oct 2026 (later): M3a: the VL53's movement detection

New: `change_grid` (shared by both detectors: scores over 4 frames, thresholds with a
neighbour, absorbing a cell moved for 5 s, blobs), `tof_motion` (each zone's
reference: how often sure / unsure / none, its distances; scores per §6.2),
`motion_sense` (feeds it while PicoB says the robot is still, logs movement), keys
`v` and `o`, and `z` now also prints each zone's signal, ambient light and second
target. `rangefinder` gives the zones' raw readings (`rangefinder_zones()`).

Found in the host test: scoring states with a hand-set weight let rare unsure
frames add up (a 2 % rate of them gave false movement within a minute). Scores are
now bits of surprise (−log₂ of how likely the reading was under the reference),
with a threshold of 20 bits over 4 frames (14 with a neighbour): no false movement
in a minute of a noisy room over 10 random seeds, a person found in 2 frames, an
object put down absorbed after 5.0 s. Which way a target moves moved to the tracker
(it needs the same target in two frames).

**On the robot (step 1, still for ~2.5 min, living room):** ~16 false movements, all
single zones: the same floor-row zone (6th row, left edge) reading 58 cm, its
reflection on the waxed floor, again and again, and far zones (1.7-3.3 m, upper
rows) now and then reading another surface. A reference with one distance per
zone, forgetting in ~3 s, saw each switch as new. Now each zone knows up to 3
readings with how often they come, over ~1 min, learns every reading (one seen
once stays known, and rare), and a zone alone needs 30 bits (3 frames of a new
distance). The host test's room now has those glitches (1-2 frames every ~10 s):
quiet for 5 min over 10 seeds. Each movement line now lists its zones: what they
read against what they are known to read.

**On the robot again (step 1, ~2 min still, then a hand twice):** no false movement;
both hand passes found, on the correct side (+17°, then −18°), at 14-24 cm. The log
showed a new distance scoring 6 bits instead of 10 from its 2nd frame: it had been
learned at once, so a lone zone could reach only 28 of its 30 bits. Readings are now
learned once they leave the 4-frame window (the view takes ~0.8 s to learn); host
test: a thin pole in one zone is found in 3 frames. Zone lines show the bits over
the window.

**Steps 2-4 on the robot, and a rewrite:** walking and a hand were found, but after
a box was put down and taken away, movement went on for a long time. `z` showed why:
almost every zone above the floor rows reports a faint unsure target at ~30 cm
(signal 3-10 against the floor's 150-730), the floor's echo inside the sensor, also
facing just a wall and a door frame; the real target is the next one, sure. Using
the nearest target whatever its status had made the detector watch the echo, and
the bits / known-readings machinery made it hard to see. Daniel: overcomplicated.
Rewritten simply: the nearest sure target per zone, as the map uses; a background
per zone (the middle of its sure readings while learning); movement = clearly
closer than the background (8 cm or 8 %) in 2 of 4 frames; farther is never
movement and becomes the background after 1 s; closer for 5 s becomes the
background. The raw per-zone readings (signal, ambient, second target) are gone from
`rangefinder` and `z`. Two traps the host test found on the way: the farthest
reading as background (one reflection frame while learning became the background)
and "nothing" winning the median for a far target that comes and goes.

**Still, nothing moving (robot facing a wall and a door frame):** steady false
movement, nearly all in the 6th row (−8°): its zones grazing the waxed floor read
the floor (~40 cm) or the wall behind (1.3-2.5 m), and a median background of the
wall made the floor "closer"; also a door frame's edge now and then. Now floor zones
ignore readings beyond their floor patch (the map's limit), the background is the
nearest distance a zone gives regularly (2nd-nearest of 1 s), a zone alone needs 3
of 4 frames, a brief nearer reading that goes back becomes the background, and
farther readings need 10 s in a row. The host room has all of these: quiet over 20
seeds. Detector times are in seconds of `RANGEFINDER_HZ` (Daniel asked about 10 Hz:
to try once 15 Hz works).

**Again (a minute still, a hand, Daniel's wife walking through, a box):** the still
minute was quiet; the hand and the person (small groups of zones at 1.4-2.2 m on
the left) were found; taking the box away gave nothing. But a box put down kept
"moving" for 5 s after it was let go: the 5 s rule called anything new movement.
Movement is change now: a zone closer but steady for 1 s has stopped, and that
distance is its background; `change_grid` lost its absorbing. Host test: a box is
still after 1.1 s, a hand waving for 3 s counts the whole time, 20 seeds.

**On the robot:** box put down: movement while it was handled, "ended" ~1 s after
it was let go; taken away: one line (the hand); the box moved from left to right:
+20°, −3°, −18°, ended. The VL53's movement detection works as intended.

## 6 Oct 2026: M3a started: the camera driver out of bring-up

`picoA/drivers/camera.c`: capture runs continuously into three buffers (one being
filled, the newest complete frame, the one the caller holds), one DMA interrupt
per frame; the PIO program now waits for VSYNC itself, so restarting it late can
only skip a frame, never start halfway. Each frame carries the time of its middle
row's exposure (frame period measured from the frames). `camera_lock_exposure()`
turns auto-exposure off at a whole number of 10 ms with the digital gain making up
the difference. The bring-up firmware uses the driver (same viewer protocol and
modes; 320-wide buffers only there). The app starts the camera at power-up; keys
`c` (frame as blocks, frame rate, exposure) and `e` (lock).

Then reworked from what the robot needs, not from what bring-up had (Daniel): one
readout (160 × 120, Sub4, no binning; the binning experiments, modes 0-4 and the
tone-curve choices removed, and the 320-wide buffers with them); the sensor's
auto-exposure off for good and the driver's own exposure control instead (10 ms
steps, then gain, `camera_hold_exposure()`), so there is no switching between auto
and manual; the frame length follows the exposure and the line period is measured
continuously, nothing assumes a frame rate. The datasheet (V01, found online, kept
next to the repo, README "References") confirms a rolling shutter: frames now give
each row's time. `e` is gone. The bring-up viewer shows the driver's exposure with
a hold checkbox. Builds, host and viewer tests pass; not yet run on the robot
(README, "Camera").

**On the robot (camera steps 1, 2, 5, evening light):** worked first time. Line
period 42.7 µs (measured), exposure 40 ms (the longest step) with gain ×5.9, 24.9
frames/s, mean brightness 98 against the target 100; three presses over ~15 s gave
the same blocks within ±1 (no flicker bands, exposure steady). The middle row's
moment was 24-48 ms before printing (20 ms is half the exposure). Not mirrored: a hand held
at the left edge of the robot's view (from behind the robot) darkened columns 1-8;
the driver then doubled the gain (×5.9 → ×11.6) to bring the mean back to 98, so
the rest of the image got ~1.5 × brighter (why motion detection must hold the
exposure while it has a reference).

## 6 Oct 2026: M3 planned (movement detection and following)

Planning only, no code (ROBOT_PLAN.md §6, §7.1-7.2, §12). Daniel's points: robot
states IDLE / SCAN / WATCH; a common interface for the camera and VL53 detectors;
no assumption that movement goes left or right (react only if the horizontal part
of its direction is > 0.5, else log); follow without stopping between turns, or
someone walking past is gone; use uncertain and flaky VL53 zones, since their
changes are information. Proposed and agreed: a reference per cell (not the
previous frame) in a change grid both detectors share; zone states scored by how
rare they were in the reference; pursuit with the camera reference shifted by the
gyro heading and the map as the ToF reference; M3 split into M3a (still) and M3b
(following by turning). Signal per SPAD, ambient and 2 targets per zone are logged
first and kept only if they help. ST's on-chip motion indicator plugin was
downloaded and checked (identical in ULD 1.2.1, 1.3.0 and 2.0.0) and then not
used: 16 values in 8 × 8 mode, a depth window of ≤ 1.5 m from 40 cm, its own
16-frame reference, useless while turning.

## 5 Oct 2026 (evening): WiFi console (ROBOT_WIFI.md), working on the robot

Daniel wants no USB cable while the robot drives around. PicoA is a Pico 2 W, so
`PICO_BOARD` is now `pico2_w` for every firmware. New: `picoA/app/wifi_console.*`
(joins the WiFi, finds the PC server by its UDP broadcasts, sends the console and
takes keys over TCP, heartbeats both ways, reconnects by itself; a second stdio
output and input next to USB, so `debug_console.c` keeps its `printf`s) with a
host test against a fake WiFi chip and lwIP; `pc/robot/` (Node server and browser
page: status bar, a button per key, the log, a log file per run). Start-up
changed: on USB PicoA does nothing until a key (`n` scans, `w` connects to WiFi);
without USB it tries the WiFi, then scans. The scan no longer starts when a
serial monitor opens. Host tests and the server tests pass.

**On the robot:** worked first time. Powered up without the USB cable, PicoA joined
the WiFi, found the server, did the start-up scan (shown on the page), and took
keys from the page. While setting up, Claude's copy of the template overwrote
Daniel's `wifi_config.h` (Daniel re-entered it); the first rebuild didn't pick up
the new file because the build had never seen it (recompiled; tracked since).

## 5 Oct 2026: M2 passed; the map reworked

**M2 test (latest `picoA_app` of 4 Oct, waxed wood, living room).** The start-up
map matched the room (walls 1.5-2 m away, the table and sofa legs, an object ~40 cm
behind); no `##` on open floor. Using the unlearned 5th row again had turned the
area behind-left from unknown into free out to the walls. Daniel placed objects
to the left, then `n` and `q`: they were mapped where they were, the robot faced
away from them, and after the square (odometry 1.1 / −1.5 cm, 363.0°, as on 4 Oct)
`m` showed walls and objects in place and no obstacles from braking. **M2 passed.**
Red-flag review: REDFLAGS.md, "M2 review pass" (commit `bf30030`).

Found in the M2 test: isolated unknown cells inside the free area (often in radial
lines) cut the free distance short, so the robot turned away from a more open
front-right; map changes counted 1584 after one scan and a square (too noisy for
M5 as they stand). The `**` puzzle (it seemed to point the wrong way) was `n`
having reset the map's up between the scan and the square.

**Gaps.** The floor rays (7 cm up, pointing down) leave the 2-12 cm layer before
they reach the floor, so the last stretch of every floor ray never cleared layer 0.
Fixed: a ray that reaches the floor clears layer 0 all the way to it (anything
standing on the floor 2 cm or taller would have blocked it).

**Nothing fades (Daniel).** Before: R2's 240 s timer (+60 s per sighting, −60 s per
empty reading, counting down in real time), and cells turned unknown after 240 s
unseen. Now: a cell changes only when measured and keeps the time of its last
measurement; an obstacle clears after 6 empty readings in a row. Curiosity about
old cells will be the behaviour's job.

**Drops (`?`).** First version: rows 6-8 marked `?` on no return or a reading 20 %
past their floor, at once. On the robot: a ring of `?` at ~50 cm. Cause: the 6th
row sees floor anywhere from 35 to 71 cm (a 5.6° tall zone on a grazing floor), so
readings of 54-71 cm were floor, not drops. Fixed: "beyond" means past the far end
of the zone's floor patch plus 15 % (test: readings anywhere in the patch, 2157
false drops with the old rule, 0 with the new). The ring stayed thinner: `z` showed
the 6th row sometimes reads the wall (184 cm, a reflection off the waxed floor),
about once in 24 readings. Then: `?` only after 3 such readings with no floor seen
in between, and (Daniel) the 6th row doesn't tell drops at all; only rows 7-8
(steady at 28-31 and 20-22 cm) do. The VL53 status 12/14 zones top right moved to
top left when the robot was turned: the scene (two surfaces in a zone), not the
sensor. Living room afterwards: no ring, 2.0 m free on average, 2 `?` in front of a
white glossy cupboard (reflections; accepted).

**Desk.** A start-up scan 10-15 cm from a desk's edge learned the floor in only 9 of
32 zones (rows 6-7 none): half the turn looked over the edge and the long readings
spoiled the floor; without a floor rows 7-8 can't tell a drop, and the map showed
`.` beyond the edge. Fixed: floor learning takes only readings inside each zone's
floor patch (±2°); rows 7-8 fall back to the floor the sensor's 7 cm height gives;
`.` only where the floor was seen, `:` for free without floor seen. Desk again:
rows 7-8 learned (29 / 21 cm), `?` along the edges ahead and right, `:` beyond, `.`
only on the desk. Floor learning stays (Daniel asked why learn at all): the 6th row
reads 35-53 cm where geometry says 48.

**Removed object.** `n`, object removed, one turn with `r`, `m`: the object cleared,
the rest stayed. `:` showed right next to the robot and between the rings where
the floor rows end. Fixed (Daniel: assume floor in the blind spots): within 15 cm
of the robot is floor with nothing on it, and a floor reading marks its zone's
whole patch. On the robot `.` then also reached 1-1.6 m to the left: nose up ~3°,
the 6th row's patch reached 1.6 m. Fixed: nose up no longer stretches the 6th
row's patch. Last run: solid `.` disc ~50-60 cm around the robot, `:` beyond.

Commits: `bf30030` (M2 passed, review), `727264c` (gaps, no fading, drops, floor
seen), `8b7407f` (blind spots).

## 4 Oct 2026: new motors, M1 finished on waxed wood, M2 written

**Motor change.** All four motors replaced: the old ones were Pololu #2208 (298:1
LP 6V, 12 CPR #3081 encoder boards on the front ones only, encoder signs opposite
to the motor signs); the front-right one sometimes didn't start (it kept stopping
the tests with "right wheels not following"). New: GA46-N20E-0043 with Hall
encoders on all four wheels. The encoder constant was inferred from the first run
(126 rpm with the old 12-count constant = 54 rpm) and confirmed (~55 rpm on all
four wheels at full power, lifted). The left encoders' power was first wired
reversed (LEDs off, boards cool); fine since the fix. Coded: the encoder driver
counts all four in PIO instead of GPIO interrupts, odometry averages front and
rear per side, `drive`'s wheel stop judges each wheel, the bring-up prints every
wheel. All four encoder signs checked. PicoA had still been on protocol v3: both
on v4 since.

**M1 with the new motors (waxed wood).**
- Square: odometry 1.2 cm forward, 1.2 cm right, 362.6°; the real end pose
  practically the same. No safety stops.
- 10 turns: all 10, no stops; gyro 3600.5° in 126 s (28.5 °/s for a 28.6 °/s
  command); wheels −898.3 / +898.7 cm; effective track width 28.6 cm; the robot
  ended ~15° past the mark (gyro 0.42 % low: no correction) and really ~15 cm back,
  5 cm right, which odometry can't see.
- 2 m straight (USB unplugged): odometry 200.2 cm (wheels 199.9 / 200.5 cm),
  heading −0.1°, 0.1 cm sideways; real ~200.5 cm (+0.15 %): `WHEEL_DIAMETER_M`
  stays 9.0 cm. PicoA kept running on the battery; the result was printed on
  reconnecting and with `t`.

**M2 written:** `tof` split (sensor part in `drivers/`, USB streaming in
`bringup/tof_stream.c`); clock sync in `body`; `pose`, `rangefinder`, `world_map`,
`surroundings`, `behaviour`, `motion.h`; console keys `n`, `m`, `z`; host tests incl.
the start-up scan in a simulated room.

- **First run:** the sensor is not mirrored (hand test on `z`). Only the first of up
  to 4 targets per zone was used, so most zones were "unsure"; the floor rows read
  the near edge of their 5.6° floor patch (6th row ~35 cm, not 48), which drew a
  ring of false obstacles at ~60 cm; the row above the floor rows reached the floor
  at its lower edge when the robot nodded (far `''`); the map's up was PicoB's
  power-up heading. Fixed: closest sure target; the 5th row learns the floor too;
  each floor zone learns its effective angle and scatter; readings that may be
  floor at a zone's lower edge are free space; the scan's start is the map's
  origin.
- **Second run:** much better map (floor rows 21 / 29 / 38-43 cm). PicoB stopped
  with "drive commands stopped arriving" after the map print (mapping ~220 frames
  plus printing 3.4 KB kept the loop busy > 250 ms); a false `##` ~35 cm ahead-left
  (maybe the USB cable). Fixed: motors off while the map is built and printed; `m`
  switches them off first; ties in "most open" pick the middle of the widest open
  sector; a floor zone whose learned floor is farther than 1.25 × its centre's
  isn't learned (the 5th row in a room mostly sees walls); `z` shows what each zone
  decided and what each floor zone learned.
- **Third run:** the start-up scan ran without a stop; floor rows 6-8 at ~45 / 30 /
  22 cm, the 5th row rightly not learned; the false obstacle ahead-left gone.
  After it: an unlearned 5th row is used like the rows above.

Commits: `a3c93de` (motors, PIO encoders), `1d81812` (M2).

## 3 Oct 2026: M1 written, first calibration runs, review

Written: the calibration tests as console keys (`robot_test.c`), STATUS with the
gyro bias, wheel distances in ODOM, a 15° tilt stop on PicoB, the status line
every 15 s, no stop on USB unplug, host tests for `body` and `robot_test`.

| Test | Result |
|---|---|
| Tilt stop | Works: lifting one side stopped the square test with "tilted more than 15 deg" |
| Gyro drift, 10 min still | Bias −0.328 °/s, wandering only 0.009 °/s (−0.3328 … −0.3239): at most 0.5° per minute of driving. Yaw frozen while still. No change needed |
| 10 turns | Turn rate held at 28.6 °/s for 7 turns, then "right wheels not following" |
| 2 m straight | Ran, but the result was lost |
| Square (twice) | Odometry 3.6 / −1.9 cm, 363.4° and −0.5 / −3.3 cm, 364.7°; real end pose not measured |

- Front-right motor sticking: the controller reached full power within ~0.6 s and
  the rear-right motor on the same driver turned, so it was the motor, gearbox,
  wiring or driver channel A, not the software. Solved by the motor change.
- Console bugs: the welcome text and last result, printed the moment the USB
  connects, didn't show on the Mac, and later tests overwrote the stored result.
  Fixed: printed ~1 s after connecting, `t` reprints. Unexplained: after `f` and
  `g` (motors in a safety stop) nothing was printed, though the robot drove ~2 m;
  not seen since.
- Review pass (REFACTORING.md, all applied): new `brain` module on PicoB, MOTORS
  request numbers (protocol v4), USB print timeout, test table in `robot_test`,
  shared fakes. Commit `43081ca`.

## 2 Oct 2026: M0 passed

Square test on waxed wood, 50 cm sides at 10 cm/s, turns at 0.5 rad/s:

| | Odometry | Measured | Difference |
|---|---|---|---|
| End position, forward | 1.3 cm | 2.0 cm | 0.7 cm |
| End position, right | 1.1 cm | 1.4 cm | 0.3 cm |
| Total turn | 362.6° | ~362–363° | < 1° |

- Link: 0 bad and 0 lost frames. Speed held at 10.0 cm/s; turns at 26–32 °/s for a
  28.6 °/s command; heading within ±0.2° along each straight leg.
- The ~2.6° extra turn is the square test stopping each turn 0.5–1.8° late, not an
  odometry error.
- Roll dips to −3…−5° during turns in place (the body leans on its tyres).
- Found on the robot and fixed: the IMU breakout is mounted turned 180° (X backward,
  Y right); the app never started the encoders; both encoder signs were reversed.

## Before: PCB bring-up

Every connected part verified with the bring-up firmware and viewer (tag
`pcb-bringup-v1`). Decided while planning M0: taking the camera driver out of
bring-up moved to M3; losing PicoA's serial monitor stopped the robot (dropped in
M1: the 2 m test runs with the USB unplugged).
