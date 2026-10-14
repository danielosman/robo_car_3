# RoboCar — the command API and the CLI (plan)

Worked out with Daniel on 13-14 Oct 2026, built on 14 Oct. The decisions are at
the end; the design of the API server is under "The API server, module by module".

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
readable as short text (or a picture's file path).

**In short:**
- Ten commands, each with one job: drive, rotate, goto, watch, snapshot, clear map,
  recording, mode, stop, and status (a read). The other keys go.
- A command API (Hono, in the robot server, its own code in `pc/robot/src/api/`)
  with one queue. The web page and the CLI (`robot steer r30d50`) both send their
  commands to it; nothing reaches the robot any other way.
- One short answer line per command (`done: d50 ok, at (62, -4) cm, yaw +30°`),
  streamed to the CLI and shown to everyone in one log.
- Read commands for snapshots, the map (as text, one character per cell, with the
  path driven), pictures (as file paths), and a `--check` that says what a move
  would do on the map before it is sent.
- Safety numbers from PicoB (the largest jolt and tilt), stored with the pose, so
  that bump and tilt stops can be set from measurements later.

## The steering commands

`robot steer <commands>`: one or more, run one after the other. Spaces between them
are optional; long names work too (`robot steer rotate 30 drive 50`).

| Command | Short | Does |
|---|---|---|
| `drive <cm>` | `d50`, `d-40` | drive straight; negative = backwards (-300…300) |
| `rotate <deg>` | `r30`, `r-30` | turn on the spot; + = left, − = right (-720…720) |
| `goto <x> <y>` | `g120,40` | go to a point on the map, cm (-300…300 each): turn towards it, drive there |
| `watch <s>` | `w60` | turn towards movement for that many seconds (1…600) |
| `snapshot` | `s` | a full snapshot, saved to the database |
| `clear_map` | `c` | clear the map |
| `recording on\|off` | `R1`, `R0` | recording on or off (set, not switched) |
| `mode idle\|manual` | `m0`, `m1` | idle or manual (set, not switched) |
| `stop` | `x` | motors off, mode idle, the queue emptied; skips the queue, and stands alone |

- A command that needs a number and gets none is an error. There are no defaults.
- The API refuses numbers out of range, and the robot checks them again. The limits
  are in one C header that the API reads (`picoA/app/commands.h`).
- `goto` turns towards the point, drives there correcting its heading, stops before
  obstacles on the map (as `drive` does), and goes at most 3 m towards a point
  farther away. It works in idle too (it switches to manual, as every move does).
- After every move (drive, rotate, goto, watch) the API adds a `snapshot`,
  recording on or not: `robot steer r30d50` runs as `r30 s d50 s`. The robot no
  longer takes one by itself at the end of an action; it still sends one when a
  server connects.
- The robot starts in **manual** mode. The mode changes nothing yet; it is kept for
  the modes to come.
- Removed: scan (`r390` does the same), record 5 s, direction (negative numbers),
  WiFi (it starts and reconnects by itself), help, and the text prints (map, ToF,
  camera, vision, the movement backgrounds). Everything they printed except the
  backgrounds is in a snapshot.

## The queue

- Commands run one at a time, in the order they arrived, whoever sent them.
- `stop` skips the queue: it goes to the robot at once and empties the queue.
- **Anything that isn't `ok` empties the queue:** an error, a move that ended
  early (obstacle, safety stop, PicoB lost, didn't start), or `stop`. The commands
  still waiting are answered `dropped: <command>`, so the agent looks at where it
  ended up before it goes on. A move's own snapshot still runs first (built 14
  Oct: without it the newest snapshot would show the robot before the move); `stop`
  drops everything.
- **The robot not connected** when a command's turn comes: `error: <command> robot
  not connected`, and the queue is emptied.
- **No answer in time:** each command may take its expected time twice over plus
  10 s (from the firmware's speeds). Without an answer by then: `error: <command>
  no answer`, and the queue is emptied. No retries, no waiting for a reconnect: the
  agent decides what to do.
- The queue lives in the server's memory: restarting the server empties it.

## The answers

Every command gets one line, in the order the commands ran:

```
queued:  r30 s d50 s
done:    r30 ok, at (12, -4) cm, yaw +30°
done:    s snapshot 41, at (12, -4) cm, yaw +30°
done:    d50 obstacle after 31 cm, at (43, -4) cm, yaw +30°
dropped: s
```

- Every answer is `<kind>: <command> <how it ended>`, the command in its short form.
- A move answers how it ended and the pose after it, in the map frame (x forward
  at power-up, y left, yaw + = left): `ok`, `obstacle after 31 cm`, `safety stop:
  tilt`, `safety stop: wheels`, `stopped`, `PicoB lost`, `didn't start`. A goto
  to a point farther than 3 m: `ok, 3 m towards it (goal 4.2 m away)`.
- A snapshot answers its number and the pose it was taken at.
- `c`, `R1`, `m1` and `x` answer `ok`.
- Other answers: `error: <command> <why>` and `dropped: <command>`.
- The answers are short on purpose: the agent asks for details with the reads.

## The robot's side (firmware)

- **Commands are lines**, `drive -40 #17`: the command and the number the API gave
  it. Over WiFi from the server; typed over USB they work too (without a number).
- **The robot answers each one** with a record when it has ended:
  `"\001done n=17 end=ok x=43.0 y=-4.0 yaw=30.0 moved=31"` (cm, degrees). `end` is
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

## Reading data

`robot <read> [n]`, plain text, or JSON with `--json`. `n` is a snapshot's number
in the newest boot; without it, the newest snapshot.

| Read | Answers |
|---|---|
| `status` | the pose now, mode, recording, WiFi; what runs and waits; the largest jolt and tilt since power-up |
| `snap [n]` | the snapshot in a few lines: when, the last action and how it ended, the pose, the nearest obstacles ahead, left and right, the light, the number of camera features |
| `light [n]` | lux, the colour (CIE x, y) and a colour name |
| `features [n]` | each camera feature: its place in the picture, the direction to it (degrees, + = left), new, tracked or lost |
| `tof [n]` | the 8 × 8 ToF frame, cm (how it is oriented: `robot tof --help`) |
| `camera [n]` | the camera picture's file path |
| `map [n]` | the 6 × 6 m map window, one character per 10 cm cell (60 × 60), x and y in cm along the edges, the robot and its heading, the path driven since the map was last cleared, a legend; `--png`: the same as an image, answered with its path |
| `log [lines]` | the last lines of the log (default 20) |
| `queue` | what runs and what waits, and who sent it |

## Checking a move first (`--check`)

`robot steer g120,40 --check` moves nothing and queues nothing. It answers what
each move would do on the newest snapshot's map, each from where the one before
would end; after one that wouldn't end `ok`, the rest would be dropped:

```
check: g120,40 turn +34° to yaw +64°, then drive 126 cm; blocked: obstacle after 81 cm, would stop at (52, 31) cm
check: d80 clear: would end at (92, -4) cm, yaw +30°
check: r90 clear: turns to yaw +120°
```

- It is the robot's own check for `drive` and `goto`: a strip as wide as the robot
  plus 5 cm on each side along the line, looked up in the map
  (`cell_map_obstacle_along`), stopping 15 cm before an obstacle. For a turn, it
  looks at the circle the robot sweeps (the robot itself doesn't check turns).
- The numbers come from the firmware's headers; a test runs the TypeScript check
  and the C check on the same map and compares them.
- Only drive, rotate and goto can be checked.
- The map is the newest snapshot's (taken after the last move): `--check` says
  "clear on the map I have"; the robot still checks for itself while it drives.

## The log

One log, and everyone sees all of it: the robot's own text, the server's notes, and
every command's answers with who sent it (`[cli]`, `[page]`). The page gets it over
its WebSocket, `robot log` reads it, and the log file in `logs/` gets the same lines.

## The CLI

```
robot --help                     # everything, for a person or an agent
robot steer --help               # the steering commands; robot <read> --help likewise
robot steer r30d50               # rotate 30°, then drive 50 cm
robot steer g120,40 --check      # what would happen, without moving
robot steer x                    # stop
robot map --png
```

- A small Node script, `pc/robot/robot`, holding no state and knowing no commands:
  `robot <name> [args] [--flags]` becomes a request to `/api/robot/<name>`. It
  finds the server at `ROBOT_API` (default `http://127.0.0.1:8080`). No login: the
  server is for the home network only.
- `steer` streams: each answer is printed as soon as its command has ended, and the
  CLI exits when the last one is answered. `--no-wait` returns after `queued`.
  Ctrl-C leaves the commands in the queue; `robot steer x` stops them.
- The help is served by the API (`GET /api/robot/help[/<name>]`), so the CLI and
  the API can't disagree.

## The web page

- Its buttons (stop, mode, snapshot, clear map, recording, status), the steering
  boxes and the map click are API requests, into the same queue as the agent's.
- Text boxes (not number boxes: those don't take a lone `-` everywhere) next to
  drive, rotate and watch, filled in with 50, 30 and 60.
- The map shows the path driven, from the stored pose lines.
- The WebSocket only brings the log, the robot's status, its pose and new
  snapshots; it takes no keys any more.

## The API server, module by module

The API is a few deep modules with small interfaces. The same word means the same
thing everywhere:

- **command**: one steering instruction, `{ name: "drive", value: 50 }`; written
  short (`d50`) in every answer and log line, long (`drive 50`) to the robot.
- **steer**: a list of commands sent together.
- **answer**: one line about one command (`queued`, `done`, `error`, `dropped`).
- **outcome**: how one command ended on the robot (`ok` or not, and the text).
- **done record**: the robot's own line (`\001done n=… end=…`), never shown.
- **sender**: `cli` or `page`.
- **read**: a command that only reads (`status`, `map`…), never queued.
- **pose**: x, y (m) and yaw (rad) in the map frame inside the code; cm and degrees
  only in text.

```
CLI ──HTTP──┐                                  ┌─> RobotLink ──TCP──> PicoA
page ─HTTP──┴─> routes ─> CommandQueue ─> CommandRunner
                  │                            └─< done records, StoredSnapshots
                  ├─> reads ─> Store (DuckDB), LiveRobot, Log
                  └─> check ─> RobotMap
```

| Module | Interface | Hides |
|---|---|---|
| `api/commands.ts` | `parseSteer(text)`, `shortText(cmd)`, `robotLine(cmd, n)`, `timeLimitMs(cmd)`, `withSnapshots(cmds)` | the grammar (letters, long names, spaces), limits, the robot's wording |
| `api/firmware.ts` | `LIMITS`, `MOVE`, `SPEED` | reading the numbers from the C headers |
| `api/runner.ts` | `run(cmd): Promise<Outcome>` (never rejects); `StoredSnapshots` | numbering, sending, matching done records, time limits, waiting for a snapshot to be stored |
| `api/queue.ts` | `submit(cmds, sender, onAnswer): Promise<void>`, `stop(sender, onAnswer)`, `state()` | order, emptying on anything not `ok`, dropped answers |
| `api/answers.ts` | `describe(cmd, done)`, `answerLine(answer)`, `poseText(pose)` | how an end reads in words |
| `api/reads.ts` | a table: name → `{ usage, help, run(ctx, arg, flags) → { data, text } }` | queries, units, rendering |
| `api/check.ts` | `checkSteer(cmds, map, pose)` | the move simulation |
| `api/map.ts` | `RobotMap.column(cx, cy)`, `obstacleAlong(...)`, `text()`, `png()` | the window over the whole-house tiles |
| `api/colour.ts` | `colourName(x, y, lux)` | white by colour temperature, else a hue |
| `api/help.ts` | `help(name?)` | the text an agent steers from |
| `api/routes.ts` | `createApi(ctx): Hono` | HTTP: paths, streaming, flags |
| `log.ts` | `text()`, `note()`, `answer()`, `tail(n)`, `backlog()`, event `item` | the backlog, the file, the fan-out to pages |
| `live.ts` | `LiveRobot`: `status`, `pose`, `boot`, `peaks`; `flush()` | the live feed; the pose lines into DuckDB |

The routes are generic: `POST /api/robot/steer` (the commands as the body;
`?check`, `?nowait`, `?from=page`), and `GET /api/robot/<read>[/<arg>]` (`?json`,
`?png`), `GET /api/robot/help[/<name>]`. Errors are values: a command that fails
ends with an outcome, never an exception through the queue.

## Gaps (left for later)

- The old console page (`/console`) still sends keys over the WebSocket, which
  nothing takes any more. Either it goes, or it moves to the API.
- The movement backgrounds (the old `o` and `k`) are in no snapshot: nothing
  shows them any more.
- `robot features` gives each feature's place in pixels, not the direction in
  degrees: the camera's field of view isn't measured or in the code yet.
- Safety step 2: the bump stop and the 10° tilt stop, once the measured numbers
  show what normal looks like.

## Decided (13-14 Oct)

1. The commands are drive, rotate, goto, watch, snapshot, status, clear_map, recording, mode and stop; scan, record 5 s, direction, WiFi, help and the text prints go.
2. Every command that takes a number needs one (watch in seconds), and there are no defaults.
3. The limits are drive ±300 cm, rotate ±720°, watch 1–600 s and goto ±300 cm per axis, checked by the API and again by the robot.
4. All steering goes through `robot steer <commands>` (e.g. `robot steer r30d50`), with the letters as in the table.
5. One parser in the API reads the steering text for both the CLI and the page.
6. One CLI call is either one steer or one read, never a mix.
7. Commands run one at a time in the order they arrived, whoever sent them; `stop` skips the queue, and `status` is a read.
8. Any end that isn't `ok` empties the queue, and the waiting commands are answered `dropped: <command>`; a move's own snapshot still runs first, so the newest snapshot shows where the robot ended up (found while building, 14 Oct).
9. The robot takes commands only from the API (the USB is for debugging), so a command replaced by another needs no handling.
10. The API numbers each command; the robot's done record carries the number, how it ended, how far it drove and its pose, and a number nothing waits for is ignored.
11. The robot answers clear_map, recording, mode, stop and snapshot too (a snapshot with its number); the API answers status itself.
12. A snapshot command is answered once that snapshot is stored in DuckDB, so it can be read right away.
13. If the robot isn't connected when a command's turn comes, the answer is `error: <command> robot not connected`.
14. If no answer arrives within the command's time limit, the answer is `error: <command> no answer` and the queue is emptied, with no retries and no waiting for a reconnect.
15. The time limit is the command's expected time (from the firmware's speeds) twice over plus 10 s.
16. The API adds a snapshot after every move, recording on or not.
17. The robot no longer takes a snapshot at the end of each action, but still sends one when a server connects.
18. Every answer is `<kind>: <command> <how it ended>`: a move adds `at (x, y) cm, yaw …°`, a snapshot says `snapshot <n>` and its pose.
19. The safety stops answer `safety stop: tilt` and `safety stop: wheels` now, and `safety stop: bump` later.
20. Everyone sees one log with every command's answers and who sent it, on the page, in `robot log`, and in the log file.
21. The CLI streams its answers, `--no-wait` returns after `queued`, and Ctrl-C leaves the queue running.
22. The API lives in `pc/robot/src/api/`, built with Hono inside the existing HTTP server, under `/api/robot/`.
23. The CLI finds the API at `ROBOT_API` (default `http://127.0.0.1:8080`), with no login.
24. `robot --help` covers everything and `robot <name> --help` each command, all from the text the API serves at `/api/robot/help`.
25. The pose line stays at 0.25 s and gains only recording on/off and the safety numbers; the server stores each one in DuckDB with the boot of the newest snapshot.
26. `robot map [n]` shows the 6 × 6 m map window as 60 × 60 characters, with the robot, the path driven since the last clear_map and a legend, and `--png` answers with an image's path.
27. Pictures (camera, map PNG) are answered with their file's path.
28. `robot steer … --check` moves nothing; it checks each move in turn from where the one before would end, on the newest snapshot's map, and answers in text.
29. The check's numbers come from the firmware's headers, and a test compares the C and TypeScript checks on the same map.
30. Recording and mode are set (`R1`/`R0`, `m0`/`m1`), not switched, and the robot starts in manual.
31. goto works in idle too, like the other moves.
32. The page sends its buttons and map clicks to the API, with text boxes (50, 30, 60 to start) next to drive, rotate and watch.
33. The map click's limit is 300 cm (the old code checked 300 m).
34. For safety, PicoB measures the largest jolt and tilt first; the bump and 10° tilt stops are set from those numbers later.
35. The old console page stays as it is for now.
