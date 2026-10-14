# RoboCar — the command API and the CLI (plan)

Worked out with Daniel on 13-14 Oct 2026, built on 14 Oct. The decisions are at
the end; the design of the API server is under "The API server, module by module".

## Status (14 Oct, end of the session)

- **Built and committed**: everything under "Decided" below (commits e83be77,
  9b338ac, 43aa575 on `main`). Host tests, `npm test` (36), lint and both
  firmware builds pass; tried end to end with the fake robot and the CLI.
- **Not yet on the robot.** First flash **both Picos**: PicoA
  `build/picoA/picoA_app.uf2` (commands, done records, pose line, `rotated`) and
  PicoB `build/picoB/picoB_app.uf2` (jolt and tilt peaks). The link version went
  up to 5: a Pico with old firmware keeps the motors off. Then restart the server
  (`npm start` in `pc/robot/`).
- **Next:** the README test "Steering through the API" (`./robot` in `pc/robot/`).
  Its step 8 gives the first jolt numbers for the bump stop.
- **Open, to discuss:** the text map's readability and the whole-map PNG with cm
  labels ("To do, to be discussed" below).
- Camera pictures moved to `pc/robot/recordings/camera/` and the database paths
  were updated; nothing else in `recordings/` changed.

## What and why

**Until now:** the robot was steered with single keys, the same over the USB serial
monitor and over WiFi (`picoA/app/debug_console.c`). There were about twenty of
them, grown one test at a time: actions with fixed sizes (`f` = 50 cm, `t` = 30°,
`s` = a 390° scan), a direction switch (`r`), switches for recording and the mode,
and text prints of the map, the ToF frame, the camera and the movement backgrounds.
The web page sent the keys over its WebSocket; a click on its map sent `goal x y`.
The robot took a snapshot by itself at the end of every action. Nothing waited for
anything: a key sent while an action ran replaced it.

**What we want:** an AI agent steering the robot from a command line, and reading
what the robot saw, without a person in between. For that, the commands take
numbers (`drive -40`), each command gets an answer that says how it ended and where
the robot is now, commands wait for each other, and everything the robot saw is
readable as JSON (or a picture's file path).

**In short:**
- Commands, each with one job. Queued, one at a time: `steer` (the moves: drive,
  rotate, goto, watch), `snap`, `clear_map`, `recording`, `mode`, and `stop`
  (which skips the queue). At once: `status`, `check`, the reads of a snapshot
  (`map`, `tof`, `camera`, `light`, `features`), `log` and `config`.
- A command API (Hono, in the robot server, its own code in `pc/robot/src/api/`)
  with one queue. The web page and the CLI both send their commands to it;
  nothing reaches the robot any other way.
- Every answer is JSON with the same fields, streamed to the CLI and shown to
  everyone in one log.
- Safety numbers from PicoB (the largest jolt and tilt), stored with the pose, so
  that bump and tilt stops can be set from measurements later.

## The commands

`robot <command> [args] [--flags]`. Units, said once in the help: cm, degrees (+ is
left), pixels, g, dBm, lux. The map frame: x forward from where the robot was at
power-up, y to its left.

**Queued, one at a time, in the order they arrived, whoever sent them:**

| Command | Does |
|---|---|
| `steer <moves>` | the moves, one after the other: `d<cm>` drive (negative: backwards), `r<deg>` rotate (+ left), `g<x>,<y>` goto a point on the map (at most 3 m), `w<s>` watch. Spaces optional; long names work too (`drive 50`). |
| `snap` | takes a snapshot: camera picture, ToF frame, light, map, pose |
| `clear_map` | clears the map |
| `recording on\|off` | records every action in full, or not |
| `mode idle\|manual` | sets the mode (manual at power-up) |
| `stop` | stops at once: motors off, mode idle; skips the queue and drops what waits |

- A move needs its numbers; there are no defaults. The limits (drive ±300 cm,
  rotate ±720°, goto ±300 cm per axis, watch 1-600 s) are in one C header the API
  reads (`picoA/app/commands.h`); the robot checks them again.
- **auto_snap** (`robot config auto_snap on|off`, on by default): the server sends a
  `snap` after every move. It is a command like any other, with its own answer.

**At once, never queued:**

| Command | Answers |
|---|---|
| `status` | the robot now: pose, mode, recording, WiFi, running, waiting, newest snapshot, safety (largest jolt and tilt since power-up) |
| `check <moves>` | what the moves would do on the newest snapshot's map, each from where the one before would end; nothing moves (not watch) |
| `map [n] [--png]` | the 6 × 6 m map window at snapshot n, one character per 10 cm cell, top-left corner, legend; `--png`: the picture's file |
| `tof [n]` | the ToF frame: 8 × 8 matrices `distance`, `status`, `signal`, `targets`, `distance_2` |
| `camera [n]` | the camera picture's file |
| `light [n]` | `lux`, `cie` (x, y), `rgb` (the colour on a screen at a fixed brightness) |
| `features [n]` | the camera features: x, y in the picture, state (new, tracked, missed, lost) |
| `log [lines]` | the last lines of the log |
| `config [auto_snap on\|off]` | the API's settings; changes one |

`n` is a snapshot's number; without it, the newest. Files are under
`pc/robot/recordings/`: camera pictures in `camera/<boot>/<n>.png`, map pictures in
`map/<boot>/<n>.png` (singular, as everything else).

## The answers

Every queued command answers one JSON line when it ends, in order, with the same
fields:

```
{"command":"steer d50","result":"ok","pose":{"x":51,"y":0,"yaw":0},"moved":50,"rotated":0}
{"command":"snap","result":"ok","pose":{"x":51,"y":0,"yaw":0},"snapshot":2}
{"command":"steer r30","result":"safety_stop","why":"tilt","pose":{"x":51,"y":0,"yaw":12},"moved":0,"rotated":12}
{"command":"snap","result":"ok","pose":{"x":51,"y":0,"yaw":12},"snapshot":3}
{"command":"steer d20","result":"dropped","why":"steer r30 ended: safety_stop"}
```

- `command`: as typed. `result`: `ok`; `obstacle` (a move stopped early before an
  obstacle on the map); `limited` (a goto further than 3 m went 3 m); `stopped` (by
  `stop`); `safety_stop` (`why`: tilt, wheels); `failed` (`why`: a bad number, robot
  not connected, no answer, PicoB lost, didn't start); `dropped` (never ran).
- `pose` after moves, snap and stop; `moved` (cm straight from where it started)
  and `rotated` (degrees) after every move; `snapshot` after a snap.
- No `id` and no `queued` lines: the answers come in the order of the commands.
  `--no-wait` answers only `{"queued":[...]}`.
- Reads answer one JSON object: one key a line, matrices one row a line, lists one
  item a line. A read that can't answer: `{"error":"..."}`.

## The queue

- `stop` skips the queue and empties it.
- **Anything that isn't `ok` empties the queue:** the commands still waiting are
  answered `dropped`, so the agent looks at where it ended up before it goes on. A
  move's own snap still runs first, so the newest snapshot shows where the robot
  ended up.
- **The robot not connected** when a command's turn comes: `failed`, "robot not
  connected", and the queue is emptied.
- **No answer in time:** each command may take its expected time twice over plus
  10 s (from the firmware's speeds); then `failed`, "no answer". No retries.
- A snap is answered once its snapshot is stored, so it can be read at once.
- The queue lives in the server's memory: restarting the server empties it.

## The robot's side (firmware)

- **Commands are lines**, `drive -40 #17`: the command and the number the API gave
  it. Over WiFi from the server; typed over USB they work too (without a number).
- **The robot answers each one** with a record when it has ended:
  `"\001done n=17 end=ok x=43.0 y=-4.0 yaw=30.0 moved=31 rotated=0"` (cm, degrees). `end` is
  one word: `ok`, `obstacle`, `stopped`, `tilt`, `wheels`, `no_drive`,
  `picob_lost`, `no_start`, `refused`. A goto adds `goal=` (cm, how far the point
  was), a snapshot `snapshot=` (its number; the pose is the snapshot's). The API
  turns it into the answer; the record itself isn't shown.
- A move answers when the action ends (also when it can't start: a drive blocked
  from the start answers `obstacle` with `moved=0`). A snapshot answers once it is
  wholly handed to the connection, so the snapshot comes before its answer in the
  stream. The others answer at once.
- **The pose line** (`"\001pose x y yaw mode action wifi"`, every 0.25 s while
  the server is connected) gains `recording` (0/1) and the safety numbers: the
  largest jolt along x, y and z (g, gravity taken out) and the largest tilt
  (degrees) since the line before. It says the robot is there, feeds the page's
  live robot and `status`, and its rows are the path on the map.
- **PicoB** keeps the largest jolt and tilt since its last odometry report and
  sends them in it; the link's protocol version goes up.

## The log

One log, and everyone sees all of it: the robot's own text, the server's notes, and
every command's answers with who sent it (`[cli]`, `[page]`). The page gets it over
its WebSocket, `robot log` reads it, and the log file in `logs/` gets the same lines.

## The CLI

```
robot --help                   # everything, for a person or an agent
robot steer --help             # robot <command> --help for each
robot steer r30d50             # rotate 30° left, then drive 50 cm
robot check g120,40            # what going there would do; nothing moves
robot snap
robot map --png
robot stop
```

- A small TypeScript script, `pc/robot/src/cli.ts` (`./robot` in `pc/robot/` runs
  it), holding no state and knowing no commands: `robot <command> [args]
  [--flags]` posts the args to `/api/robot/<command>`, each flag a query
  parameter. It finds the server at `ROBOT_API` (default `http://127.0.0.1:8080`).
  No login: the server is for the home network only.
- It prints each answer as it comes and exits when the last one is in. Ctrl-C
  leaves the commands queued; `robot stop` stops them.
- The help is served by the API (`GET /api/robot/help[/<command>]`), so the CLI and
  the API can't disagree.

## The web page

- Its buttons (stop, mode, snap, clear map, recording, status), the steering
  boxes and the map click are API requests, into the same queue as the agent's.
- Text boxes (not number boxes: those don't take a lone `-` everywhere) next to
  drive, rotate and watch, filled in with 50, 30 and 60.
- The map shows the path driven, from the stored pose lines.
- The WebSocket only brings the log, the robot's status, its pose and new
  snapshots; it takes no keys any more.

## The API server, module by module

A few deep modules with small interfaces. The same word means the same thing
everywhere:

- **command**: one queued action, `{ name: "drive", values: [50] }`; written as typed
  (`steer d50`) in answers and the log, long and numbered (`drive 50 #17`) to the robot.
- **move**: drive, rotate, goto or watch, given to `steer`.
- **answer**: the JSON line about one command; **outcome**: an answer before it is
  named (what the runner returns); **result**: how it ended.
- **done record**: the robot's own line (`\001done n=… end=…`), never shown.
- **sender**: `cli` or `page`. **read**: a command that runs at once.
- **pose**: x, y (m) and yaw (rad) inside the code; cm and degrees in answers.

```
CLI ──HTTP──┐                                  ┌─> RobotLink ──TCP──> PicoA
page ─HTTP──┴─> routes ─> CommandQueue ─> CommandRunner
                  │                            └─< done records, StoredSnapshots
                  ├─> reads ─> Store (DuckDB), LiveRobot, Log, config
                  └─> check ─> RobotMap
```

| Module | Interface | Hides |
|---|---|---|
| `api/commands.ts` | `parseSteer`, `parseAction`, `commandText`, `robotLine`, `timeLimitMs`, `withSnaps` | the grammar, limits, the robot's wording |
| `api/firmware.ts` | `LIMITS`, `MOVE`, `SPEED`, `CAMERA` | reading the numbers from the C headers |
| `api/runner.ts` | `run(cmd): Promise<Outcome>` (never rejects); `StoredSnapshots` | numbering, sending, matching done records, time limits, waiting for a snapshot to be stored |
| `api/queue.ts` | `submit(cmds, sender, onAnswer)`, `stop(sender, onAnswer)`, `state()` | order, emptying on anything not ok, dropped answers |
| `api/answers.ts` | `outcome(cmd, done)`, `poseOut(pose)` | how a done record becomes an answer |
| `api/reads.ts` | a table: name → `{ usage, help, run(ctx, args, flags) → object }` | queries, units |
| `api/check.ts` | `checkMoves(cmds, map, pose)` | the move simulation |
| `api/map.ts` | `RobotMap.column`, `obstacleAlong`, `grid()`, `png()` | the window over the whole-house tiles |
| `api/json.ts` | `formatJson(value)` | matrices and lists laid out to read |
| `api/colour.ts` | `displayRgb(X, Y, Z)` | XYZ to a screen colour |
| `api/help.ts` | `help(command?)` | the text an agent steers from |
| `api/routes.ts` | `createApi(ctx): Hono` | HTTP: paths, streaming, flags |
| `log.ts` | `text()`, `note()`, `answer()`, `tail(n)`, `backlog()`, event `item` | the backlog, the file, the fan-out to pages |
| `live.ts` | `LiveRobot`: `status`, `pose`, `boot`, `peaks`; `flush()` | the live feed; the pose lines into DuckDB |

Errors are values: a command that fails ends with an outcome, never an exception
through the queue.

## To do, to be discussed

- **The map as text is hard to read** (Daniel, 14 Oct). Ideas:
  - A smaller window around the robot (2 × 2 m), turned so the robot faces up:
    what is ahead is at the top.
  - Two characters per cell, so cells are square in a terminal.
  - A list of obstacles instead of a grid: connected blocked cells grouped, each
    with where it is and how big ("blocked, 30 × 10 cm, from (120, 40)").
  - A scan: the free distance in each of 24 directions around the robot (every
    15°), like a lidar.
  - Only the PNG, for agents that read pictures.
- **The map PNG**: the whole map (the window and the whole-house tiles), not only
  the 6 × 6 m, with every row and column labelled with its cm coordinate. The text
  map stays 6 × 6 m.
- The old console page (`/console`) still sends keys over the WebSocket, which
  nothing takes any more. Either it goes, or it moves to the API.
- The movement backgrounds (the old `o` and `k`) are in no snapshot: nothing
  shows them any more.
- `features` gives each feature's place in pixels, not the direction in degrees:
  the camera's field of view isn't measured or in the code yet.
- Safety step 2: the bump stop and the 10° tilt stop, once the measured numbers
  show what normal looks like.
- **Look at snapshots 130 and 131 of boot `7a587226`** (Daniel, 14 Oct):
  - The robot drove onto a bump in the doorway and tilted 9°.
  - The back wheel couldn't get over the bump; the wheels skidded.
  - The robot still answered that everything was fine.
  - Check what the jolt and tilt peaks (and the poses around them) showed, to
    see if a skid can be told apart from normal driving.

## Decided (13-14 Oct)

1. Queued commands: `steer` (drive, rotate, goto, watch), `snap`, `clear_map`, `recording on|off`, `mode idle|manual`, `stop` (skips the queue). At once: `status`, `check`, `map`, `tof`, `camera`, `light`, `features`, `log`, `config`.
2. Scan, record 5 s, direction, WiFi, help and the text prints are gone.
3. Every move needs its numbers; there are no defaults. The limits are checked by the API and again by the robot.
4. `snap` takes a snapshot; reading one is `map`, `tof`, `camera`, `light`, `features`.
5. auto_snap (on by default, `robot config`): a snap after every move, as its own command with its own answer.
6. Every answer is one JSON line with the same fields: `command` (as typed), `result`, `why` (when not ok), `pose`, `moved` and `rotated` (every move), `snapshot`.
7. Results: ok, obstacle, limited, stopped, safety_stop, failed, dropped. An obstacle stop isn't a failure.
8. No ids, no `queued` lines; no units in field names (the help says them once).
9. Anything that isn't ok empties the queue; a move's own snap still runs first.
10. The robot takes commands only from the API (the USB is for debugging).
11. The robot not connected, or no answer in time: `failed`, and the queue is emptied. No retries.
12. A snap is answered once its snapshot is stored.
13. `status` is the robot now; no obstacles in it, no map.
14. `tof`: one 8 × 8 matrix per reading (`distance`, `status`, `signal`, `targets`, `distance_2`).
15. `light`: lux, CIE x, y and rgb; no colour names.
16. Reads answer JSON laid out to read: matrices one row a line.
17. Pictures are answered with their file's path, in `recordings/camera/` and `recordings/map/` (singular).
18. The pose line stays at 0.25 s and gains recording and the safety numbers; the server stores each one with its boot: the path on the map.
19. The page sends its buttons and map clicks to the API, with text boxes (50, 30, 60) next to drive, rotate and watch.
20. The CLI finds the API at `ROBOT_API`, with no login; Ctrl-C leaves the queue running.
21. `robot --help` and `robot <command> --help`, from the text the API serves.
22. Recording and mode are set, not switched; the robot starts in manual; goto works in idle too.
23. For safety, PicoB measures the largest jolt and tilt first; the stops are set from those numbers later.
24. The old console page stays as it is for now.
