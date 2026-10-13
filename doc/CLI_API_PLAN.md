# RoboCar — the command API and the CLI (plan)

A plan, not built yet (13 Oct 2026), worked out with Daniel in one session. What
is decided is listed at the end; nothing is built until Daniel says go.

## What and why

**Today:** the robot is steered with single keys, the same over the USB serial
monitor and over WiFi (`picoA/app/debug_console.c`). There are about twenty of
them, grown one test at a time: actions with fixed sizes (`f` = 50 cm, `t` = 30°,
`s` = a 390° scan), a direction switch (`r`) to make them go the other way,
switches for recording and the mode, and text prints of the map, the ToF frame,
the camera and the movement backgrounds. The web page (`pc/robot/public/app.js`)
shows the keys as buttons and sends them over its WebSocket; a click on its map
sends `goal x y` the same way. The robot takes a snapshot by itself at the end of
every action. Nothing waits for anything: a key sent while an action runs
replaces it.

**What we want:** an AI agent steering the robot from a command line, and reading
what the robot saw, without a person in between. For that, the commands need to
take numbers (`drive -40`, not "press `r`, then `f`"), each command needs an
answer that says how it ended and where the robot is now, commands need to wait
for each other, and everything the robot saw needs to be readable as short text
(or a picture's file path).

**The plan, in short:**
- Fewer commands, each with a number and one job: drive, rotate, goto, watch,
  snapshot, status, clear map, recording, mode, stop. The rest of the keys go.
- A command API (Hono, in the robot server, kept separate in code) with one queue.
  The web page and a new CLI (`robot r30d50`) both send their commands to it;
  nothing reaches the robot any other way. The API adds a snapshot after each
  move, and empties the queue when anything goes wrong.
- One short answer line per command (`done: d50 ok, at (62, -4) cm, yaw +30°`),
  streamed to the CLI and shown to everyone in one log.
- Commands for the agent to read snapshots, the map (as text, one character per
  cell, with the path driven) and pictures, and a `--check` that says what a move
  would do on the map before it is sent.
- Safety numbers from PicoB (the largest jolt and tilt), stored, so that bump and
  tilt stops can be set from measurements later.

## The commands

Each command does one thing. No command combines two actions.

| Command | CLI | Does |
|---|---|---|
| `drive <cm>` | `d50`, `d-40` | drive straight; a negative number drives backwards |
| `rotate <deg>` | `r30`, `r-30` | turn on the spot; + = left, − = right |
| `goto <x> <y>` | `g120,40` | go to a point on the map (cm, map frame): turn towards it, then drive there |
| `watch <s>` | `w60` | watch for movement for that many seconds |
| `snapshot` | `s` | a full snapshot, saved to the database |
| `status` | `p` | what the robot knows about itself (below) |
| `clear_map` | `c` | clear the map |
| `recording on\|off` | `R1`, `R0` | turn recording on or off |
| `mode idle\|manual` | `m0`, `m1` | idle or manual |
| `stop` | `x` | motors off, mode idle; empties the queue (below) |

- A command that needs a number and gets none is an error. There are no defaults
  in the API.
- Limits: the API refuses numbers out of range (drive ±300 cm, rotate ±720°,
  watch 1-600 s, goto ±300 cm on each axis, as the map click now), and the robot
  checks them again.
- `goto` is the robot's `goto` action as it is now: it turns towards the point,
  drives there correcting its heading on the way, stops before obstacles on the
  map (as `drive` does), and goes at most 3 m towards a point farther away. It is
  one action, so it is one command; for an agent, "go to (120, 40)" is easier than
  working out a turn and a distance from its pose.
- Recording and mode are set, not switched: `R1` is on whatever it was before. The
  API needs the recording state to be sure about snapshots (The queue, below).
- The robot starts in **manual** mode (today: idle). Today `goto` is taken only in
  manual mode; in the plan it works like the other moves (in idle it switches to
  manual), so the mode changes nothing yet. It is kept for the modes to come.
- `status` answers in a few lines, the robot's pose now first:

  ```
  at (43, -4) cm, yaw +30°, still, motors off
  recording off, mode manual, WiFi -58 dBm
  running: nothing; queued: nothing
  safety: largest jolt x 0.2 g, y 0.1 g, z 0.3 g; largest tilt 4°
  ```

  It is the robot's live pose (the pose line, 4 a second), not a snapshot's.
- Removed: scan (`rotate 390` does the same: in the firmware, scan was a 390° turn
  and the map is built from every frame anyway), record 5 s (with recording on,
  every action is recorded), direction (negative numbers do its job), WiFi (it
  starts and reconnects by itself; `status` shows it), help, and the text prints:
  map, ToF, camera, vision, ToF and camera backgrounds. Everything they printed
  except the backgrounds is in a snapshot.

## Where it lives

```
CLI (robot r30d50) ──HTTP──┐
                           ├──> API (Hono) ──> queue ──> robot link ──> PicoA
web page ─────────HTTP─────┘         │
         <──WebSocket: the log───────┤
                                     └──> DuckDB, snapshots (reads)
```

- The API is its own code in `pc/robot/src/api/`, built with Hono. It runs in the
  same server process and on the same port as the web page, under `/api/`. The
  rest of the server (robot link, recordings, snapshots, viewer) stays as it is,
  and the API talks to it only through a small interface.
- The web page stops sending keys over its WebSocket. Its buttons and the map click
  become API requests and go into the same queue as the CLI's.
- The CLI is a small Node script (`robot`, in `pc/robot/`) that turns its
  arguments into API requests and prints the answers. It holds no state. It finds
  the API at `ROBOT_API`, default `http://127.0.0.1:8080`. There's no login: the
  server is for the home network only.

## The queue

- Commands run one at a time, in the order they arrived, whoever sent them. A
  command from the page waits its turn behind the agent's, and the other way round.
- `stop` and `status` skip the queue. `stop` goes to the robot at once and empties
  the queue. `status` only reads, so it never waits.
- **Anything that isn't `ok` empties the queue:** an error, a move that ended
  early (obstacle, safety stop, PicoB lost, didn't start), or `stop`. The commands
  still waiting are answered `dropped: <command>`, so the agent looks at where it
  ended up before it goes on. Otherwise `d50 d50` drives into the same obstacle
  twice.
- **The end of a command:** the API sends each command with a number
  (`"\002drive 50 #17\n"`). The robot answers with a short machine-readable line
  when it has ended, `"\001done 17 <how it ended>"`, like the pose line it already
  sends. The API turns it into the answer to that command, `done: d50 …` (The
  answers, below). The robot's line itself isn't shown to anyone.
- **Time limit:** each command may take its own time plus a margin (e.g. watch 60
  → 70 s; drive and rotate from their length). Without a `done` by then it
  answers `error: <command> no answer`, and the queue is emptied, so it can't hang
  for ever.
- **Snapshots after commands:** a setting in the API, on by default. After every
  command that moves the robot (drive, rotate, goto, watch), the API adds a
  `snapshot`.
  While recording is on, it adds none, because the recording already has every
  frame. So with recording off, `r30d50` runs as `r30, snapshot, d50, snapshot`.
- The robot stops taking a snapshot by itself at the end of each action, because
  the API asks for one. It still sends one when a server connects.
- The queue lives in the server's memory: restarting the server empties it.

## The robot's connection

- **Robot not connected** when a command comes: answered at once with
  `error: <command> robot not connected`, not kept until it comes back.
- **The connection drops during a command:** the robot carries on with what it is
  doing (it already does: losing the console doesn't stop it), and reconnects by
  itself, as now. The API waits, and the CLI and the log show
  `info: robot connection lost, waiting` and `info: robot back`. The robot
  remembers the number and the end of the last command it finished and says it
  again when it reconnects (`"\001done 17 …"`), so a `done` sent while the
  connection was down isn't lost. The queue then goes on as if nothing happened.
- **It doesn't come back** within 10 s (or before the command's time limit, if
  that is later): `error: <command> robot lost`, and the queue is emptied.

## The answers

Every command gets one short line from the API, in the order the commands ran:

```
queued:  r30 s d50 s
done:    r30 ok, at (12, -4) cm, yaw +30°
done:    snapshot 41, at (12, -4) cm, yaw +30°
done:    d50 obstacle after 31 cm, at (43, -4) cm, yaw +30°
dropped: s
```

- A move (drive, rotate, goto, watch) always answers `done: <command> <how it ended>,
  at (<x>, <y>) cm, yaw <deg>°`: the pose after it, in the map frame (x forward at
  power-up, y left, yaw + = left). How it ended is the robot's end reason in a few
  words: `ok`, `obstacle after 31 cm`, `safety stop: tilt`, `safety stop: bump`,
  `stopped`, `PicoB lost`, `didn't start`. A goto to a point farther than 3 m
  says so: `ok, 3 m towards it (goal 4.2 m away)`.
- A snapshot answers `done: snapshot <n>, at (<x>, <y>) cm, yaw <deg>°`: its
  number and the pose, like a move (the snapshot's own pose: where the robot was
  when it was taken). Its contents (the camera picture, the ToF frame, the light...) are read
  with the commands under Reading data.
- Other answers: `error: <command> <why>` (e.g. `d` with no number, or out of
  range), `dropped: <command>` (the queue was emptied), and `info: …` (the
  connection, above).
- The answers are short on purpose: the agent asks for details (Reading data,
  below).

## The log

There is one log, and everyone sees all of it: the robot's own text, and every
command's `queued`, `done`, `error`, `dropped` and `info` lines with who sent it
(`cli` or `page`). The page gets it over its WebSocket, as it gets the robot's
text now. So the page shows the agent's commands and their answers as they
happen, and `robot log` shows the page's commands too. The log file in `logs/`
gets the same lines.

## The CLI

```
robot r30d50               # rotate 30°, then drive 50 cm
robot r-90 d100 w30        # spaces are optional
robot g120,40              # go to (120, 40) cm on the map
robot g120,40 --check      # what would happen, without moving
robot rotate 30 drive 50   # long names work too
robot x                    # stop
robot --help               # everything below, for a person or an agent
```

- The CLI streams: it prints each line as soon as that command has ended (the API
  answers in chunks, one line each), and exits when its last command is done.
- If you stop the CLI (Ctrl-C), the commands stay in the queue and run anyway;
  `robot x` stops them. `robot r30d50 --no-wait` returns after `queued`, and the
  answers show up in the log.
- `robot --help` (and `robot <command> --help`) is written so an agent can steer
  from it alone. It lists every command with its letter, its unit and its sign,
  the map frame, what the answers look like, and the commands for reading data.
  The API serves the same text at `GET /api/help`, so the CLI and the API can't
  disagree.

## Reading data (the agent's view)

Plain text by default, or JSON with `--json`. `[n]` is a snapshot's number; without
it, the command reads the latest snapshot. The latest snapshot can be old: none
is taken while recording is on, nor while the robot stands still between
commands. For what the robot sees right now: `robot s` (answers `done: snapshot
42, …`), then e.g. `robot camera 42`.

Pictures are files the server already has or draws: the CLI answers with the
file's path, and the agent opens it.

- `robot snap [n]`: the snapshot in a few lines: when, the action and how it
  ended, the pose, the nearest obstacles ahead, left and right, the light, and the
  number of camera features.
- `robot light [n]`: the colour sensor: lux, the colour (CIE x, y) and a colour
  name.
- `robot features [n]`: the camera features: how many, and each one's place in
  the image, the direction to it (degrees, + = left) and whether it is new,
  tracked or lost.
- `robot tof [n]`: the 8 × 8 ToF frame, cm along each ray, as the robot sees it.
- `robot camera [n]`: the path of the camera picture. The server already saves
  each snapshot's as `recordings/snapshots/<boot>/<seq>.png`.
- `robot map [n]`: the map as it was at snapshot n, as text, **one character per
  cell** (not the old two characters per cell), with x and y in cm along the
  edges, the robot's position and heading marked, the path it drove since power-up
  (or since the map was last cleared) up to then, and a legend. Cells are 10 cm,
  so the 4 × 4 m window is 40 × 40 characters.
- `robot map [n] --png`: the same map drawn as an image; the server saves it
  (`recordings/maps/<boot>/<seq>.png`) and answers with the path.
- `robot status`, `robot log [n lines]`, `robot queue` (what is waiting to run).

## The path driven

Today the server keeps only the latest pose line (4 a second); odometry is stored
only while recording. In the plan, every pose line goes into DuckDB with the boot
and the time (a few bytes, 4 rows a second), so `robot map [n]` can draw the path
up to snapshot n, whether recording was on or not. Each row also has the safety
numbers of its pose line: the largest jolt on x, y and z and the largest tilt
since the line before (Safety, below). So a bump can be found afterwards: when,
where on the map, and how hard.

## Checking a move first (`--check`)

`robot g120,40 --check`, `robot d80 --check` and `robot r90 --check` move
nothing and go into no queue. They answer in text what the move would do on the
latest snapshot's map:

```
check: g120,40  turn +34° to yaw +64°, then drive 126 cm
       blocked: obstacle after 81 cm, would stop at (52, 31) cm
check: d80      clear: would end at (92, -4) cm, yaw +30°
check: r90      clear: turns to yaw +120°
```

- It is the check the robot does before `drive` and while driving `drive` and
  `goto`: a strip as wide as the robot along the line, looked up in the map
  (`cell_map_obstacle_along`), stopping the same distance before an obstacle. The
  API does it again in TypeScript, on the map of the latest snapshot. For a turn,
  it checks the circle the robot sweeps turning on the spot.
- **Two copies of one check** (C on the robot, TypeScript in the API) can drift
  apart. Both take their numbers (the strip's width, the distance kept from an
  obstacle, the 3 m of goto) from one place, and a test runs the TypeScript check
  on recorded maps and compares it with what the robot did there.
- **The map can be a little older** than the robot's own: it is the latest
  snapshot's, taken after the last move. So `--check` says "clear on the map I
  have"; the robot still checks for itself while it drives.
- No picture: the text says the same, and the agent needn't read an image.

## Safety

What PicoB does already: it switches the motors off by itself when pitch or roll
is beyond 15° (lifted, tipping over) and when a wheel doesn't follow (a jam, or
pushing against something). PicoA ends the action as `safety stop`. It reads the
IMU 100 times a second.

The plan, in two steps:

1. **Measure first.** PicoB keeps the largest acceleration on each axis (x, y, z,
   with gravity taken out) and the largest tilt since its last report, and sends
   them with each odometry report. PicoA passes the largest since the last pose
   line in each pose line (every 0.25 s); the server stores them in DuckDB with
   the pose (The path driven, above), and `status` shows the largest so far. A few drives,
   bumps into things on purpose, and the carpet's edges show what normal looks
   like.
2. **Then stop on them.** From those numbers: a bump (a jolt beyond a limit
   along x or y) and a tilt beyond 10° (today 15°) become safety stops in PicoB.
   The motors go off at once; the action ends `safety stop: bump` or `safety
   stop: tilt`, and that empties the queue like any end that isn't `ok`. The 10°
   needs checking against the measured numbers first: PicoB's own comment says
   driving stays within ~5°, but thresholds and the carpet's edge may come near
   10°.

The link message between the Picos changes, so the link's protocol version goes
up (common/link_msgs.h).

## The map click

Today a click sends `goal x y` over the WebSocket, and the robot runs one `goto`
action. In the plan, a click sends the API `goto x y`, which goes into the queue
like the agent's `g120,40` (with its snapshot after it). It works as now, except
that it waits its turn, and its answer shows in the log.

## The web page

- The buttons: stop, mode, status, snapshot, clear map, recording.
- The map shows the path driven, as `robot map --png` does.
- A box for the number next to each command that takes one, filled in at the
  start with drive 50, rotate 30 and watch 60. The boxes are text boxes
  (`type="text"`), not number boxes: a number box doesn't take a lone `-` in some
  browsers and on phone keyboards, and gives an empty value for what it doesn't
  like. The page checks the box (a whole number, a minus allowed) and the API
  checks it again.
- All of them, and the map click, are API requests (above). The log shows every
  command's answers, whoever sent them (The log, above).

## Changes to the firmware

- PicoA: commands are typed lines (`drive -40`), the same over USB and WiFi, in
  place of single keys. They take the same path the server's `goal` and `mode`
  take now, with the command's number added by the API. `goal x y` becomes
  `goto x y`.
- PicoA: the removed keys go. `goto` stays, and works in idle too (it switches to
  manual, as the other moves do). `recording on|off` and `mode idle|manual` in
  place of the switches. Manual mode at power-up.
- PicoA: the `done` line at the end of each action, said again on reconnecting.
  No snapshot at the end of an action any more. The safety numbers in the pose
  line.
- PicoB: the largest acceleration and tilt in each odometry report; later the
  bump and 10° tilt stops (Safety, above).

## Other work it brings

- The fake robot (`fakerobot.ts`) sends `done` lines, drops its connection on
  request, and gives safety numbers, so the queue and reconnecting can be tested
  without the robot.
- Storing the pose lines; drawing the map with the path, as text and as a PNG.
- The `--check` in TypeScript, with its numbers shared with the firmware, and the
  test against recorded maps.
- Tests for the API, the queue, the CLI's parsing and the answers.
- The README's robot tests, ARCHITECTURE.md and TODO.md get updated.

## Gaps (left for later)

- The old console page (`/console`, `pc/robot/public/console.js`) stays as it is
  for now: it still sends the old keys over the WebSocket, which the firmware won't
  know any more. Either it goes, or it moves to the API.
- The movement backgrounds (the old `o` and `k`) are in no snapshot: nothing
  shows them any more.

## Decided (13 Oct)

- The commands in the table and no others; scan, record 5 s, direction, WiFi,
  help and the text prints go.
- A command that needs a number and gets none is an error; no defaults in the API.
- CLI letters as in the table; limits as above.
- `stop` and `status` skip the queue; everything else waits its turn.
- Anything that isn't `ok` empties the queue.
- The robot not connected: an error at once. A dropped connection is waited for
  (10 s) and shown as `info:` lines.
- No `done` in time: an error, and the queue is emptied.
- The CLI streams its answers; it needn't wait (`--no-wait`, Ctrl-C).
- The page's commands share the queue with the agent's, and everyone sees one log.
- Snapshots follow only the commands that move the robot, and none while
  recording.
- Every move answers `done: <command> <how it ended>, at (x, y) cm, yaw …°`.
- A snapshot answers `done: snapshot <n>, at (x, y) cm, yaw …°`, like a move;
  `status` answers with the pose now first.
- The pose lines go into DuckDB with their safety numbers (largest jolt, largest
  tilt).
- `goto x y` stays (`g120,40`); the map click sends it through the API.
- Pictures (camera, map PNG) are answered with their file's path.
- `robot map [n]` is the map at snapshot n with the path driven up to then; the
  pose lines are stored for that.
- `--check` for drive, rotate and goto, in text only; no simulation picture.
- `robot --help`, from the same text as `GET /api/help`.
- `status` shows recording and mode; recording and mode are set, not switched.
- Manual mode at power-up.
- The page: a text box (not a number box) next to drive, rotate and watch, filled
  in with 50, 30 and 60 at the start.
- The CLI finds the API at `ROBOT_API`; no login.
- The old console page stays for now (a gap).
- Safety: measure the largest acceleration and tilt first, then decide the bump
  and 10° tilt stops.
