# RoboCar

Firmware for a fully autonomous robot car on two Pico 2 boards and a custom PCB.
PCB bring-up is complete (tag `pcb-bringup-v1`): every connected part was
verified with the bring-up firmware and viewer described below. The robot
firmware (`app/` per Pico) maps its surroundings with a ToF sensor, watches for
movement and turns to follow it, and drives on command; PicoA (a Pico 2 W) sends
its console and recordings over WiFi to a server on the PC. The code is the source
of truth for configuration and protocol details.

**Start here:** [doc/ARCHITECTURE.md](doc/ARCHITECTURE.md) (how the robot works,
module by module, and Daniel's decisions), [doc/TODO.md](doc/TODO.md) (what is
left), "How we work" below. History: [doc/CHANGELOG.md](doc/CHANGELOG.md).

```
robo_car_3/
├── CMakeLists.txt       # Pico SDK setup + add_subdirectory(picoA/picoB); PICO_BOARD for both
├── pico_sdk_import.cmake
├── .vscode/             # Run / Flash / Debug ask which firmware (picoA/… or picoB/…)
├── run_tests.sh         # host tests, no Pico needed
├── doc/                 # ARCHITECTURE.md, TODO.md, CHANGELOG.md, GLOSSARY.md; archive/: the old plans
├── common/              # shared by both Picos: the inter-Pico link, time/angle/statistics helpers (+ host tests)
├── tools/               # lint.sh; replay.sh + replay_map.c (the map on a recording); stack_depth.py
├── picoA/               # A1 — sensors: camera, ToF, color (MLX90640 planned)
│   ├── CMakeLists.txt   # picoA_drivers library + one executable per firmware
│   ├── drivers/         # ToF, camera (HM0360), OPT4048; lib/vl53l8cx/ = ST ULD driver
│   ├── bringup/         # camera.c: PCB test firmware (USB streaming to pc/bringup)
│   └── app/             # robot firmware, the brain
├── picoB/               # A2 — motors, encoders, IMU
│   ├── CMakeLists.txt   # picoB_drivers library + one executable per firmware
│   ├── drivers/         # imu, motor, encoder
│   ├── bringup/         # main.c: tilt -> motor test firmware
│   └── app/             # robot firmware, the body
├── pc/
│   ├── bringup/         # Node server + browser viewer for picoA_bringup
│   └── robot/           # Node server + browser console for picoA_app over WiFi
└── build/               # one build dir for all firmwares (git-ignored)
    ├── picoA/picoA_app.uf2, picoA_bringup.uf2
    └── picoB/picoB_app.uf2, picoB_bringup.uf2
```

One configure and build produces every firmware. Drivers are shared: each Pico's
`CMakeLists.txt` builds the robot firmware and the bring-up firmware from the same
`picoX_drivers` library, so the bring-up firmware keeps building and can be used
to re-test hardware (e.g. a new PCB revision).
Code shared between the Picos (the inter-Pico link; `stamp.h` time differences
across the clock's wrap, `geom.h` bearings and angles, `stats.h` n-th value and
median; `clock_start.h`: both Picos start their clock 30 s before
`time_us_32()` wraps, so every run crosses the wrap 30 s after power-up) is in `common/`.

## Robot firmware (`picoA_app`, `picoB_app`)

How it works, module by module: [doc/ARCHITECTURE.md](doc/ARCHITECTURE.md). In
short: the robot does nothing by itself; it is steered through the robot server's
command API ([doc/CLI_API_PLAN.md](doc/CLI_API_PLAN.md)), from the `robot` CLI
(`pc/robot/robot`; for an agent or a person) or the page at
http://127.0.0.1:8080/, which share one queue and one log:

```sh
ln -s "$PWD/pc/robot/robot" /usr/local/bin/robot   # once
robot --help                  # everything: the commands, the answers, the reads
robot steer r30d50            # rotate 30° left, then drive 50 cm (a snapshot after each move)
robot steer g120,40 --check   # what going to (120, 40) cm would do on the map; nothing moves
robot snap; robot map; robot camera; robot status
robot steer x                 # stop
```

After every move a snapshot (pose, camera picture and features, ToF frame,
colour, its map) is stored; the page shows any of them back in time, with the path
driven, and sends the robot where you click on the map. With `robot steer R1` each
action is also recorded in full and can be looked at in 3D at
http://127.0.0.1:8080/viewer. Typed on PicoA's USB serial monitor, the same
commands work for debugging (`drive 50`, `status`).

- **Host tests:** `./run_tests.sh` runs them on the Mac, no Pico needed, and stops
  at the first failure. The fake clock starts 30 s before `time_us_32()` wraps, so
  every test crosses the wrap. `npm test` and `npm run typecheck` in `pc/robot/` for
  the server. Run them after every change.
- **Linters:** `tools/lint.sh`: cppcheck on the C (`brew install cppcheck`), ESLint
  and tsc on `pc/robot` (`npm run lint`, `npm run typecheck`). Kept clean.
- **Replay:** `tools/replay.sh <take id>` runs the robot's map code on a recorded
  take (needs the take in DuckDB: `npm run import` in `pc/robot/`).
- **The viewer without the robot:** in `pc/robot/`, `npm run snapreplay -- <boot id>`
  turns a recorded drive into snapshots (the robot's own map and snapshot code);
  then `ROBOT_PORT=4311 PORT=8091 DATA_PORT=4312 ANNOUNCE_TO=127.0.0.1 npm start` (a
  server the robot doesn't find) and `ROBOT_PORT=4311 npm run fakerobot --
  recordings/snapshots/raw/replay-boot-<id>.snap`, and open
  http://127.0.0.1:8091/, or `ROBOT_API=http://127.0.0.1:8091 robot …`. The fake
  robot answers every command, driving its pose at the robot's speeds (the
  snapshots it sends are the file's).

### The map as text (`robot map`)

The 6 × 6 m window around the robot, 10 cm columns, x up (forward at power-up), y
to the left; one character per column, x and y in cm along the edges:

| Symbol | Meaning |
|---|---|
| `.` | drivable: floor seen (a ray ended in the ground layer), 3-23 cm free |
| `:` | free 3-23 cm, floor not seen (between the rings where rays end on the floor, or far) |
| `#` | blocked: something 3-13 cm high |
| `'` | overhang: something 13-23 cm high, nothing below it |
| `?` | no floor: rays went through the ground layer (a drop, a dark floor) |
| blank | unknown |
| `R`, `*`, `o` | the robot, the point 30 cm ahead of it, the path driven since the map was last cleared |

Standing still, the floor is seen in rings at each zone row's distance (~20, 30, 45,
70 cm); driving fills the rest.

A flat face often fills two cells (`##`): the map marks where the face was seen,
not how thick the object is.

### Measured on the robot (waxed wood)

| | Value |
|---|---|
| Odometry after a 50 cm square | within ~1 cm and ~1° of the real end pose (the test overshoots each turn 0.5-1.8°, so it ends at ~363°) |
| Encoder distance | +0.15 % over 2 m: `WHEEL_DIAMETER_M` 9.0 cm, no correction |
| Gyro bias | ~−0.33 °/s, wandering < 0.01 °/s over 10 min; re-measured whenever still |
| Gyro scale | 0.42 % low (~1.5° per full turn): no correction (M5's full-turn check will) |
| Effective track width | 28.6 cm (geometric 22.5 cm: the wheels skid) |
| Turning in place | the body slides ~1.5 cm per turn, which odometry can't see; roll −3…−5° |
| Floor rows 6 / 7 / 8 | see the floor at ~47 / 31 / 21 cm along the ray; rows 7-8 steady to ±1-2 cm |
| Camera (living room, evening light) | line period 42.7 µs (~12 MHz pixel clock, 512 per line); exposure 40 ms × gain 5.9 for mean brightness ~98, so 24.9 frames/s; a still scene's blocks repeat within ±1 between frames (no flicker bands) |
| VL53 unsure readings | most zones above the floor rows report a faint unsure target at ~30 cm (signal 3-10 kcps/SPAD), the distance where rows 7-8 see the floor (150-730): the floor's echo inside the sensor; the real target is the next one, sure. Use only sure targets (status 5, 6, 9) |
| VL53 on the waxed floor | the 6th row sometimes reads a reflection (e.g. the wall); rows 7-8 can read "beyond the floor" in front of glossy furniture (`?`); status 12 zones (two surfaces in one zone) come and go with the scene |

### Memory and stack (computed from the build, PicoA)

| | Value |
|---|---|
| RAM | ~103 KB free of 512 (12 Oct, computed from the build; `p` prints it): the snapshot buffer 38 KB, vision ~30 KB, the whole-house map 20 KB came on top of 11 Oct's ~195 KB free; everything static, no `malloc` linked |
| Core-0 stack | ~6.5 KB worst case: main loop 4.8 KB (`observations()` in `tof_motion.c` 3.2 KB), lwIP interrupt 0.75-1.3 KB, USB 0.25 KB. The SDK's 2 KB is nominal; the stack has 8 KB (both scratch banks) while core 1 is unused |

`tools/stack_depth.py` recomputes the stack (how to build for it: the script's
header). Re-run it after adding big locals and before core 1 is started.

### Robot tests (repeat after changes)

Since 14 Oct the robot takes commands, not keys. The older tests below are written
in the keys; read them as: `f` → `robot steer d50` (`r`, `f` → `d-50`), `t` →
`r30`, `s` (scan) → `r390`, `a` → `w60`, `S` → `s`, `C` → `c`, `R` → `R1` / `R0`,
`M` → `m1` / `m0`, space → `robot steer x`, `p` → `robot status` (or `status` typed
on USB for PicoA's diagnostics: link, loop time, RAM), `m` → `robot map`, `z` →
`robot tof`, `v` → `robot features`, `5` (record 5 s) → `w5` with nothing moving.
The prints `c` (camera blocks) and `o` / `k` (movement backgrounds) are gone:
`robot camera` gives the picture; the backgrounds can't be seen any more. On USB,
type the long form (`drive 50`).

**Steering through the API** (14 Oct; both Picos flashed: the link version went up;
battery on, WiFi, `npm start` in `pc/robot/` restarted, the page open). Paste the
terminal and the page's log:
1. `robot status`: "robot connected (…)", the pose, "mode manual, recording off",
   "running: nothing", and the safety line (jolt near 0 g on x and y, ~0 on z;
   tilt a few degrees at most).
2. `robot steer r390`: "queued: r390 s", then after ~14 s "done: r390 ok, at (…),
   yaw +30°" and "done: s snapshot N, …". The page's log shows both lines with
   `[cli]`, and the new snapshot.
3. `robot map`: the room around `R`, the walls where they are. `robot map --png`
   prints a file path; open it. `robot snap`, `robot tof`, `robot camera` (open the
   file), `robot light`, `robot features`.
4. Facing a piece of furniture 40-70 cm away: `robot steer d100 --check` says
   "blocked: obstacle after N cm"; `robot steer d100 r30`: "done: d100 obstacle
   after M cm" (M within a few cm of N), "done: s snapshot …" (where it stopped),
   "dropped: r30", "dropped: s". The front edge is 15-25 cm from the furniture.
5. `robot steer w60`, and in another terminal `robot queue` and, after a few
   seconds, `robot steer x`: "done: w60 stopped", "done: x ok"; `robot status` says
   "mode idle".
6. On the page: `drive` with 30 in its box, `rotate` with -30, a click on the map
   (the yellow cross goes when its answer comes), Stop. The terminal's
   `robot log` shows them with `[page]`. A box with "abc" says what it wants.
7. Out of range or no number: `robot steer d400`, `robot steer d` answer
   "error: …", nothing moves. `robot steer R1 d20`, then `robot steer R0`: one take
   (the take list), recording shown on the page's button.
8. Bump the robot lightly by hand while it drives (`robot steer d50`): `robot
   status` shows a larger jolt on x afterwards. Paste it: this is the measurement
   the bump stop will be set from.

**M0** (after changes to `drive`, `odometry` or the link):
1. Flash `picoB_app` onto PicoB and `picoA_app` onto PicoA. Keep the robot still
   for ~1 s after power-up (gyro bias). Battery on J11 for the motors.
2. Open a serial monitor on **PicoA**, press `p`. Expected: a status line with
   x/y/yaw, and PicoB's messages as `B: ...`. "PicoB not connected (received 0
   bytes)" means the UART isn't working; `p` also shows the link counters (`bad`
   and `lost` should stay ~0).
3. Without motors, check the IMU orientation: lift the **front** → pitch goes
   positive; lift the **left side** → roll goes positive; turn the robot **left** by
   hand → yaw goes up (`to_robot_frame()` in `picoB/app/odometry.c`).
4. On the floor with room: `f` (50 cm forward), `t` three times (90° left), `f`
   again. Each prints "done" with what odometry measured. Mark the start, measure
   the real position and heading, and compare. Space stops at any time.

**M1** (calibration). Paste the serial log of each step:
1. **Tilt stop:** press `s` (scan) and lift one side of the robot past 15°.
   Expected: `B: Motors off: tilted too far` and "Scan stopped: PicoB switched the
   motors off (tilted too far)".
2. **Gyro drift:** robot on the floor, don't touch it for 10 min; `p` at the start
   and the end: yaw should stay put, the gyro bias wander little.
3. **Gyro scale:** a mark on the floor in line with the robot's front; `t` 12 times
   (360°). Estimate how far it ended from the mark.
4. **Encoder distance:** tape measure, robot at 0, 2.5 m of free floor ahead (a
   move stops 15 cm before an obstacle on the map); `f` four times (2 m). Measure
   where it really got to and compare with the four "done" lines; `r`, then `f`
   four times back.
5. **Carpet:** `s`, `f` and `t` on the carpet. Look for false safety stops and a
   slower turn rate.

**M2** (map; only PicoA):
1. Robot on the floor with ~1 m of room around it, battery on. Keep it still ~1 s
   after power-up (gyro bias).
2. Plug the USB into PicoA and open the serial monitor. After ~1 s: the keys. The
   robot does nothing. Press `s`: "Scan: turning 390 deg left…". The robot turns
   ~14 s.
3. Expected: "Scan done: … turned +390 deg", motors off. Press `m` for the map.
4. Check the map against the room: walls and furniture legs where they are, `.`
   rings and `:` around the robot, no `##` on open floor.
5. `z` prints one ToF frame (cm as the robot sees it; `?N` = unsure, VL53 status N).
6. Move and turn (`f`, `t`) and press `m`: walls and objects stay where they were,
   braking draws no obstacles. To see a removed object clear, scan again (`s`, `m`);
   `C` clears the map.
7. At a table edge (held, motors off), `m`: `?` along the edge, `:` beyond it.

**Camera** (M3a; only PicoA, motors not needed):
1. Flash `picoA_app` onto PicoA, open the serial monitor. No "Camera not working"
   after the keys.
2. `c`: frames/s and the line time (both measured), the exposure and gain the driver
   chose, and the frame as 20 × 15 blocks of brightness 0-99. Expected: exposure 10,
   20, 30 or 40 ms indoors with lights (under 10 ms only in bright daylight), mean
   brightness ~80-125, "held" (movement detection holds it while the robot stands
   still; "free" while it moves; "wants a change" / "adjusting" when off target),
   and "middle row taken N ms ago" well under 100 ms.
   "N since the last c" grows with the time between presses (frames/s × seconds).
3. Hold a hand in front of the **left** half of the camera and press `c`: the left
   columns change (the print is as the robot sees it, not mirrored).
4. Switch the room light off or on, keep still ~10 s, `c`: the exposure or gain
   changed and the mean brightness is back to ~80-125. The driver waits (5 s when a
   little off, down to 0.5 s when far off), then steps by at most ×1.25; while the
   robot stands still, movement detection lets it adjust only after 5 s of calm
   (`k` counts these adjustments).
5. With the room's lights on, press `c` a few times on a still scene: each block
   within ~2 between presses (no flicker bands).
6. Bring-up: `picoA_bringup` with the viewer (below) shows the same image and the
   exposure; "Hold the exposure" freezes it (switch a light: the image gets darker
   or brighter and stays so), unticking lets it adjust again.

**Movement, VL53** (M3a; PicoA, with PicoB running so PicoA knows the robot is still;
motors off). Robot on the floor facing ~2 m of open room:
1. Flash `picoA_app` onto PicoA. Press `a` (watching prints the movement lines; the
   robot turns towards movement, so step 2 turns it too). Stand behind the robot,
   keep still for a minute. Expected: no "Movement" lines (paste any that come).
2. Walk across in front of the robot at ~1 m, left to right as the robot sees it.
   Expected: "Movement (ToF): N zones, +X deg (+ = left), …, 1.0 m" about twice a
   second, X going from positive to negative, then "Movement (ToF) ended".
3. Wave a hand ~30 cm in front of the left half: positive degrees, ~0.3 m.
4. Put a box ~60 cm in front and step away: "ended" ~1 s after you let go (still for
   1 s: it stopped moving). Take it away: no movement from the box itself (it reads
   farther), only from your hand.
5. `o` with nothing moving: each zone's background in cm (`--` nothing); then with
   you standing in view: `*` on the zones that see you.
6. `s` (scan): no movement lines while it turns; afterwards `o` says "watching".

**Movement, camera** (M3a; PicoA, with PicoB running; motors off). Robot on the
floor facing ~2 m of the room, room lights on:
1. Press `a` (watching, 1 min). Stand behind the robot, keep still for a minute.
   Expected: no "Movement (camera)" lines (paste any that come, with a `k`).
2. `k`: "watching", each block's background (as in `c`) and its noise (tenths of a
   brightness level; expected ~10-40). Paste it.
3. Walk across in front of the robot at ~1 m, left to right. Expected: "Movement
   (camera): N blocks, +X deg (+ = left), …" about twice a second, X going from
   positive to negative, then "Movement (camera) ended"; the ToF lines too.
4. Wave a hand ~30 cm in front of the left half: positive degrees.
5. Put a box ~60 cm in front and step away: "ended" ~1 s after you let go. Take it
   away: movement again (the camera can't tell taking away from putting down), "ended"
   ~1 s later.
6. Walk past close (~0.5 m), filling the view: movement the whole time.
7. Switch the room light off (or on) and keep still: "Movement (camera)" for ~1.5 s,
   "ended"; ~5 s later "Camera: adjusting the exposure, then learning the view
   again", no movement lines; a few seconds later `k` says "watching" and `c` shows
   a new exposure.
8. A lamp lighting part of the view: switch it on, keep it on ~10 s, press `k` twice
   in that time, then switch it off. Expected: movement ~1.5 s after each switch, then
   quiet. Paste the log. Also watch your shadow.

**Target, VL53** (M3a; PicoA, with PicoB running; motors off). Robot on the floor
facing ~2 m of room; `a` (watching) on. The tracker follows one target in the VL53's movement:
1. Walk across at ~1 m, left to right. Expected: "Target (new): +15 deg …, 1.0 m",
   then "Target: … going right at N deg/s" twice a second (N roughly your walking
   speed / distance: ~1 m/s at 1 m ≈ 57 deg/s), then "Target left the view on the
   right at −17 deg" (or so) ~0.5 s after you are out of view.
2. The same right to left: "left the view on the left".
3. Walk in and stop in the middle: "about still", then "Target stopped … at N deg"
   ~1.5 s after you stopped (the VL53 takes you in after 1 s, the tracker waits 0.5 s).
4. While you walk, someone else (or a waved hand) on the other side: the target stays
   on you.

**Watching** (both Picos, battery on). Robot on the floor with room to turn:
1. `s` (scan), then `a`: "Watch: for 60 s…". Stand behind the robot: it stays
   put. After a minute: "Watch done", motors off.
2. Walk across at ~1-2 m at a steady pace, left to right. Expected: "Watching:
   movement at N deg, turning towards it", movement lines "(turning at N deg/s)"
   during the turn, following without stop-and-go; stop in front of it: "facing the
   movement", then no more turning.
3. The same right to left, and once fast (it follows at ~29°/s, then "nothing
   moves").
4. A one-zone movement prints the zone's reading and background (to tell a real
   edge movement from noise). Paste the log; space stops.

**WiFi console** (only PicoA):
1. Copy `picoA/app/wifi_config.example.h` to `wifi_config.h`, fill in the network,
   build, flash `picoA_app`.
2. Robot on USB, serial monitor open: the robot does nothing at start-up (motors
   off) and tries the WiFi.
3. `npm ci` then `npm start` in `pc/robot/`; open **http://127.0.0.1:8080/** ("Waiting
   for robot…"; the console alone is at /console). The first time, allow `node`
   to accept incoming connections.
4. Expected ~3 s after power-up: "WiFi: connecting…", "WiFi: connected, IP …",
   "Server: found at …", "Server: connected" (`W` shows how it is connected). The
   page shows "Robot connected" and the keys.
5. Press `p` and `m` on the page: the answers show on the page and in the serial
   monitor. Unplug the USB (battery on): the page keeps working.
6. Power up without USB: the statuses show on the page, the robot waits; `s` on
   the page scans.

**Recording** (both Picos, battery on, WiFi console working).
Paste the page's log (or `pc/robot/logs/`) and the `npm run decode` output:
1. Restart the server (`npm start` in `pc/robot/`) so it runs the new code.
2. `p`: a line "Recording: off (R), server connected; 0 takes since power-up (boot
   …), 0 datagrams, 0 refused". `5`: "Record: recording is off".
3. `R`: "Recording: on; each action is recorded". `5`: "Record: 5 s…", then
   "Record done", "Recorded take 1: … ToF frames sent, 0 failed; … datagrams, 0
   refused; … KB". On the page: "recordings from boot …: saving to
   raw/…-boot-….rec", then "take 1 (5 record, …): done; 5.0 s; ~75 ToF frames
   (15/s), 0 missing; … datagrams, 0 lost…".
4. Robot on the floor with room: `s` (scan). Then `a` (watch) and walk around it
   for the minute. Each ends with a "Recorded take" line and a take note on the
   page. Expected: ~15 frames/s, no stalls; while turning, a few datagrams lost
   (half frames and "missing" frames in the note); how many is what this test
   measures.
5. During a watch press `p`, then space: the take ends "stopped".
6. `npm run decode -- recordings/raw/<the file>.rec` in `pc/robot/`: the same
   takes, no CRC errors, no "cut off".
7. Optional: stop the server during a take: the robot prints "take N cut" and
   goes on; restart it, `R` is still on, the next action is recorded.

**Snapshots and the viewer** (both Picos, battery on, WiFi, `npm start` in
`pc/robot/` restarted for the new code). Paste the page's log (or
`pc/robot/logs/`) and a screenshot of the page:
1. Flash `picoA_app`. After power-up, `status` typed on USB: "RAM free" ≥ 40 KB
   (computed: ~103), "Mode: manual", "Snapshots: 1 sent (the last ~31 KB)…" (one
   goes when the server connects). No "Colour sensor not working".
2. The page shows the robot connected and, at once, snapshot #1: the camera
   picture with its features (small squares; `robot features` lists the same), the
   8 × 8 ToF grid as the robot sees it, the colour swatch and lux (a lamp or a
   coloured sheet in front of the sensor changes them), the pose.
3. `robot steer r390` (a scan): after the turn snapshot #2 comes: the room on the
   map, the robot in it. The grey pill next to Stop says "manual · (x, y) cm" and
   follows the robot.
4. `robot steer s`: another snapshot. Drag the timeline back: the map, picture and
   grid are those of the older snapshot; ▶ to the newest to come back.
5. The page's Stop and the space key: "Stopped", "Mode: idle", "[page] done: x ok".
6. Unplug the server (stop `npm start`) during a snapshot, start it again: the
   snapshot comes again whole (the log says "snapshots from boot …").

**Go to a point** (both Picos, battery on, the viewer open). Room around the robot,
a piece of furniture 1-2 m away:
1. `robot steer r390`, then press Idle and click on the map: the robot switches to
   manual and goes (a go to works in idle too, as every move does).
2. Click a free spot ~1 m away: a yellow cross; "Go to: (x, y) cm, N cm away at A
   deg", the robot turns towards it, drives, "[page] done: g… ok, at (…)"; the
   cross goes, a snapshot follows. Measure where it stopped against the spot
   (expected within a few cm plus odometry's error).
3. Click behind the furniture: "done: g… obstacle after N cm"; the gap from the
   front edge 15-25 cm, as for a drive.
4. `robot steer g400,0` is refused (out of range); from near the start, a click
   ~3 m away and then `robot steer g300,300`: "ok, 3 m towards it (goal 4.x m
   away)" (only where the room allows it; `robot steer x` stops).
5. Drag the timeline back and click: the toast says it's an old snapshot, nothing
   moves.

**The map follows the robot** (both Picos, a hallway or two rooms, ≥ 4 m to drive):
1. `s` at the start, then go-to clicks (or `f`) 3-4 m along the hallway; `s` there.
   Expected: the dashed window on the page moves with the robot by 2 m steps; the
   start's walls stay on the map outside it (from the whole-house map); `p`:
   "… N tiles of the whole-house map".
2. Drive back to the start: what was mapped there is still there; where the window
   knows nothing now and a tile does, the column shows dimmed.
3. `C`: map and whole-house map cleared; the next snapshot's map is empty.

**Thin legs on the map** (13 Oct; both Picos, the viewer open, manual mode). A chair
or table with thin legs (2-4 cm) and a wall ~1 m behind them:
1. Start 2 m from a leg, facing it, and `f` (or click just beyond the leg). Expected:
   "Move stopped: obstacle N cm ahead (map)" with the front edge 15-25 cm from the
   leg; the leg's column blocked (or an overhang) on the page.
2. The same with the leg ~10 cm left or right of the robot's middle line: it still
   stops.
3. Back off 1 m and `S`: the leg stays on the map; walls look no thicker than before
   driving close.
4. Take the chair away, drive past where it stood: its cells clear within a few
   frames.

**A slow link** (13 Oct; both Picos, the viewer open):
1. Drive around as usual for a few minutes: no "nothing from the robot" notes where
   the signal is good; the status pill shows the signal (now from the pose record).
2. Carry or drive the robot where the signal is weak (another room): if bytes stop
   arriving, the pill turns amber, "nothing from it for N s", the log says
   "nothing from the robot for 6 s; the connection is open", and "data from the
   robot again, after N s" when they come. No "closing" unless it lasts 30 s, and
   the snapshot arrives late but whole.
3. Switch PicoA off: after 30 s "nothing from the robot for 30 s: closing the
   connection"; on again, it connects by itself.

**Camera features** (V1; only PicoA, motors off). Robot facing ~2 m of the room,
lights on. Paste each `v`:
1. `v`: "Vision: N frames/s, X ms per frame (max Y), F features (B new, L lost in
   the last second), jitter Z px", then the regions' feature counts and texture,
   and their median moves. Expected: N ~8 (every 3rd frame), **X ≤ 25 ms** with F
   near 96, F 40-96, L near 0, Z < 0.15; `p`'s loop time with it running.
2. Hold a hand in the left half, still: counts go up in the left regions. Move it
   slowly: `v` shows the left regions' median move ≠ 0, the others ~0.
3. Switch the light off/on, wait ~10 s, `v`: F back to roughly what it was, the
   exposure changed (as `c`).
4. Cover the lens: F → 0 within ~1 s, texture ~0; uncover: features back within
   ~0.5 s.
5. `S` and look at the page: the squares sit on the corners of the picture.

**Move stops before obstacles, and a drive around the room** (M4 step 1; both Picos,
WiFi console, the server running). Paste the console log (or `pc/robot/logs/`):
1. Flash `picoA_app`. Power up near the middle of the room (the map is 6 × 6 m
   around the power-up point), keep the robot still ~1 s. `R` (recording on).
2. `s` (scan). Then turn (`t`) until it faces a piece of furniture 40-70 cm away
   and `f`. Expected: "Move stopped: obstacle N cm ahead (map), moved M cm"; N is
   to the map's cell, so the real gap from the front edge is 15-25 cm. Measure it.
3. `f` again: "Move: not moving: obstacle … closer than 15 cm", nothing moves.
4. `r` (back), `f`: it backs off 50 cm ("the VL53 can't see behind…"). `r` again.
5. Drive around the room with `t`, `f` and an `s` now and then; keep a finger on
   space for the first moves (thin legs and things under ~8 cm may be missed).
   End where you started, facing the same way, and `s`.
6. In the viewer (http://127.0.0.1:8080/viewer) pick "whole drive" for this boot:
   the points and the replayed map of every take together, the path on the floor.
   The two scans at the start point show the odometry's drift (a wall seen twice).
   Note any move that stopped early or touched something.

## PicoA bring-up (`picoA_bringup` + `pc/bringup`) — working on the PCB

- **HM0360 / Arducam B0319 camera:** the robot's camera driver (`drivers/camera.c`),
  streamed over USB: **160×120, Sub4, no binning**, exposure set by the driver.
  Binning gives stripes and isn't used ([notes](doc/archive/CAMERA_STRIPES_TODO.md)).
- **VL53L8CX ToF:** working **8×8** ranging over SPI0; default **10 Hz**, up to four
  targets per zone. LPn is not wired to the Pico—the Pololu carrier holds it high.
- **OPT4048 color sensor:** I2C0 0x44 (GP4/5, 400 kHz), auto-range, 100 ms per
  channel (~2.5 complete samples/s). Only coherent, CRC-checked cycles are shown:
  lux, CIE xy/XYZ (datasheet example matrix, not board-calibrated), approximate
  color swatch and raw channel counts.
- **Shared browser viewer:** camera, ToF matrix and OPT4048 panels side by side,
  separate controls, diagnostic metrics and sensor logs. Camera: display gamma /
  auto-stretch, the driver's exposure, gain and frame rate, and "Hold the exposure"
  (`camera_hold_exposure()`). ToF display starts **rotated 90°** to match PCB mounting
  (display only, not camera/ToF calibration). Farthest selection means the
  farthest valid *returned* target, not necessarily every object in the zone.

The MLX90640 is not implemented yet.

## PicoB bring-up (`picoB_bringup`) — tilt → motor test, working on the PCB

- **Hardware:** all four wheels — GA46-N20E-0043 N20 gearmotors (298:1,
  50 RPM at 6 V) with Hall encoders (see "Motors" below), 20 kHz PWM, front on channel A and rear on channel B of each driver.
  Left wheels: J10, driver pins PWMA GP22, PWMB GP28, direction GP27/GP26. Right
  wheels: J9, PWMA GP15, PWMB GP11, direction GP13/GP12. GP14 is STBY for both.
  AIN1+BIN1 / AIN2+BIN2 are tied on the PCB, so both motors on a side share
  direction: if a rear motor spins opposite its front one, swap its wires. If a
  whole side runs backwards, flip `LEFT_FORWARD` / `RIGHT_FORWARD` in
  `picoB/drivers/motor.c` (left is −1, right +1: both sides ran forward with these). All four
  wheels have encoders: J6 pins 1–8 are the right wheels (front 1–4 on GP9/GP8,
  rear 5–8 on GP7/GP6), pins 9–16 the left wheels (front 9–12 on GP5/GP4, rear
  13–16 on GP3/GP2).
  Adafruit #4502 ISM330DHCX breakout on SPI0 (GP16–19, mode 3, 1 MHz). The motors
  need battery power on J11; USB only powers the logic.
- **Differences from the PCB / PCB review:** the left/right labels are swapped on
  the motor and encoder connectors. The left motors are on **J10 (MOT_DRV_R, MR_\*
  nets)** and the right motors on **J9 (MOT_DRV_L, ML_\*)**. The encoders follow
  the motors: left-front on **J6 9–12 (ENC_RF_\*)**, left-rear on **J6 13–16
  (ENC_RB_\*)**, right-front on **J6 1–4 (ENC_LF_\*)**, right-rear on **J6 5–8
  (ENC_LB_\*)**. The firmware follows the actual wiring, not the net names.
- **Motors:** **GA46-N20E-0043**: N20 gearmotor, 298:1 (medium power), 50 RPM at
  6 V, stall 0.6 A (the TB6612FNG gives 1.2 A per channel, so fine), with a Hall
  encoder board (green power LED, powered from 3V3_B on J6). Encoder: **7 pulses
  per channel per motor turn = 28 counts** on every edge, × 298 = **8344 counts
  per wheel turn**; ~55 rpm on all four wheels at full power, lifted (≈ 0.26 m/s).
  All four encoder signs checked (+ = forward). After a motor or encoder change,
  check with `picoB_bringup` (below) before the robot firmware: its wheel stop
  would otherwise switch the motors off within 1 s.
- **IMU:** WHO_AM_I check, 416 Hz, ±2 g / ±250 dps (register values from the ST
  datasheet), gyro bias averaged at power-up. Tilt about X and Y comes from a
  complementary filter (gyro short-term, gravity long-term, 0.5 s time constant),
  so pushes along an axis barely register as tilt.
- **Control:** tilt about the breakout's X axis picks a side, always driving forward.
  The accelerometer reads +1 g along whichever axis points up, so tilt X is positive
  with the breakout's +Y end raised: **+Y up → right wheels, +Y down → left wheels**;
  the other side coasts. ±15° dead zone, then power ramps linearly to 100 % at ±90°.
- **Encoders:** all four wheels, quadrature counted by PIO (`drivers/quadrature.pio`,
  one state machine of `pio0` per wheel, every edge of both channels; internal
  pull-ups on), reported as output-shaft revolutions and RPM (28 × 298 = 8344
  counts/rev); ~55 RPM at full tilt.
- **Serial (USB):** nothing is printed and the driver stays in standby until a
  serial monitor is open; closing it stops the motor. Send `g` to enable, `s` to
  stop, `z` to zero revolutions. 10 Hz lines (units in the header): tilt X/Y (deg),
  accel ax/ay/az (g), gyro gx/gy/gz (deg/s), left/right power, then revolutions/rpm per
  wheel: LF, LR, RF, RR (left/right, front/rear; + = forward), RUN/STOP.

Keep the robot still for ~1 s after power-up (gyro bias).

**Encoder direction check** (after any motor or encoder change), robot lifted so
the wheels turn freely: flash `picoB_bringup`, open a serial monitor, press `g`.
Tilt the +Y end of the IMU breakout up: the right wheels drive forward and **RF and
RR rpm must be positive**. Tilt it down: the left wheels, **LF and LR positive**. A
wheel that turns backwards is a motor wire (swap it; a whole side backwards:
`LEFT_FORWARD` / `RIGHT_FORWARD` in `motor.c`). A wheel that turns forward with
negative rpm: flip its `direction` in `picoB/drivers/encoder.c`. An rpm stuck at 0
while the wheel turns: encoder wiring or power. Paste a few lines of each side.

## Run

1. Build with the Pico VS Code extension (SDK/toolchain in `~/.pico-sdk/`, CMake/Ninja
   from Homebrew) or from a terminal (`PICO_SDK_PATH` etc. are set in `~/.zshrc`):
   ```sh
   cmake -S . -B build -G Ninja   # first time or after deleting build/
   # if build/CMakeCache.txt still says PICO_BOARD pico2: add -DPICO_BOARD=pico2_w once
   cmake --build build
   ```
   If the extension downloads its own CMake/Ninja again, point `.vscode/settings.json` back
   at Homebrew's (one copy of each tool). `PICO_BOARD` is `pico2_w` for every
   firmware (PicoA's WiFi); PicoB's firmware runs the same on its plain Pico 2.
   Robot firmware: `build/picoA/picoA_app.uf2` onto **PicoA**, `build/picoB/picoB_app.uf2`
   onto **PicoB**. Bring-up: `picoA_bringup.uf2` / `picoB_bringup.uf2`. In VS Code, Run /
   Flash / Debug ask which firmware to use (`picoA/picoA_app`, `picoB/picoB_app`, or
   the bring-up ones); they flash whichever Pico is on the USB cable, so pick the
   matching one.
   The steps below are for the PicoA bring-up viewer; PicoB's bring-up only needs a
   serial monitor (see above).
2. With Node.js 24+, run `npm ci`, then `npm start` in `pc/bringup/`.
   With multiple Picos attached, use `npm start -- /dev/tty.usbmodem1101` (replace with PicoA's port;
   list candidates with `ls /dev/tty.usbmodem*`).
3. Open **http://127.0.0.1:8080/**. Restart the Node app after reflashing/reconnecting.
   Sensor initialization and errors appear in the page log.

In `picoA/`: `drivers/camera.c` (camera driver: continuous capture into three
buffers, one interrupt per frame, its own exposure control, row times) with
`drivers/hm0360_init.h` / `hm0360_regs.h` / `hm0360.pio`, and `bringup/camera.c`
(the bring-up main loop and serial commands); `drivers/tof.c` / `drivers/lib/vl53l8cx/`: ST ULD integration
(`bringup/tof_stream.c`: the viewer's ToF packets and commands);
`drivers/opt4048.c`: OPT4048 polling and raw telemetry (conversion to lux/XYZ is in
`pc/bringup/src/opt4048.ts`).
In `picoB/`: `bringup/main.c` (tilt filter, control, serial), `drivers/imu.c`,
`drivers/motor.c`, `drivers/encoder.c` with `drivers/quadrature.pio`.
`pc/bringup/`: USB parsing (`src/protocol.ts`), server (`src/main.ts`) and viewer
(`public/`). Run `npm test` in `pc/bringup/` for software checks;
hardware behavior still needs PCB testing after changes.

## References used

- [PCB review, A1/A2 pin maps, motor/IMU choices](../../kicad/robo_car_3/PCB_REVIEW.md) (`~/kicad/robo_car_3/`).
  Camera pins match the old example; ToF INT moved **GP21 → GP20**, and the old
  GP20 LPn output was removed. SPI remains GP16–GP19.
- [HM0360 V04 datasheet](../hm0360/doc/HM0360-datasheet_v4.pdf), especially §§3.3,
  6.6 and 10.6–10.11; [local summary](../hm0360/doc/HM0360-datasheet.md) omits some
  diagram details—prefer the PDF.
- [HM0360 V01 datasheet (preliminary, April 2019)](https://www.welectron.com/mediafiles/productimg/arducam/Datasheet/HM0360-image-sensor-datasheet.pdf)
  ([local copy](../hm0360/doc/HM0360-datasheet_v01.pdf), [extracted text](../hm0360/doc/HM0360-datasheet_v01.txt),
  searchable, figures missing): the copy on this Mac. Rolling shutter (p. 2), motion
  detection §4.1, CMU timing §9.1, gains §9.2, exposure and flicker §9.3, frame rate
  §9.4, registers §10. Himax marks it confidential: not in the repo.
- [VL53L8CX datasheet (ST)](https://www.st.com/resource/en/datasheet/vl53l8cx.pdf)
  ([local copy](../vl53l8cx/spec/vl53l8cx.pdf)).
- [ST UM3109 ULD guide](https://www.st.com/resource/en/user_manual/um3109-a-guide-for-using-the-vl53l8cx-lowpower-highperformance-timeofflight-multizone-ranging-sensor-stmicroelectronics.pdf)
  ([local copy](../vl53l8cx/spec/um3109-a-guide-for-using-the-vl53l8cx-lowpower-highperformance-timeofflight-multizone-ranging-sensor-stmicroelectronics.pdf)):
  initialization, SPI/LPn, ranging settings, multiple targets and validity statuses.
- [ISM330DHCX datasheet (ST DS13012)](../../kicad/robo_car_3/specs/ism330dhcx.pdf),
  §§9.11–9.33 (registers) and the initialization procedure (p. 37);
  [Adafruit #4502 notes](../../kicad/robo_car_3/specs/adafruit_4502_ISM330DHCX.txt).
- [OPT4048 datasheet (TI SBOSA84)](https://www.ti.com/lit/ds/symlink/opt4048.pdf),
  especially §§8.3.4.5 (CRC) and 9.2.4 (XYZ / lux conversion).
- Source examples: [camera project](../arducam_b0319/arducam_b0319.c),
  [camera experiment history](../arducam_b0319/SESSION_LOG.md),
  [HM0360 tone curves / exposure controls](../hm0360/hm0360.c),
  [ToF project](../vl53l8cx/vl53l8cx.c),
  [OPT4048 project](../opt-4048/main.c) and [notes](../opt-4048/OPT4048_NOTES.md). ST's driver license is retained in
  [`picoA/drivers/lib/vl53l8cx/LICENSE.txt`](picoA/drivers/lib/vl53l8cx/LICENSE.txt).

Local reference links assume this layout (this repo is `~/code/robo_car_3`):

- `~/kicad/robo_car_3/` — KiCad project and PCB review
- `~/code/hm0360/`, `~/code/vl53l8cx/`, `~/code/arducam_b0319/`, `~/code/opt-4048/` —
  sibling example projects (not yet copied to this Mac)

## Docs

- **README.md** (this): how to build, run and test.
- **[doc/ARCHITECTURE.md](doc/ARCHITECTURE.md)**: how the robot works now, module by
  module, where the code is, and Daniel's decisions. Only the current state.
- **[doc/TODO.md](doc/TODO.md)**: what is left: big features and ideas, and small
  fixes to existing modules.
- **[doc/CHANGELOG.md](doc/CHANGELOG.md)**: what happened, newest first (robot runs,
  results, what was tried and dropped, review findings). Only history.
- **[doc/GLOSSARY.md](doc/GLOSSARY.md)**: the words.
- **[doc/archive/](doc/archive/)**: the old plans and reviews, unchanged. TODO.md
  links into them where they hold a design not built yet.

## How we work

- **Visible progress first.** Each step should change something the robot does;
  groundwork (timing, telemetry) only when a feature needs it.
- **Ask before changing behaviour that wasn't agreed.** An idea in TODO.md needs
  Daniel's yes before it is built.
- **A step:** read the code it changes (and the archived plan part, if there is
  one); host tests first where they make sense; `./run_tests.sh` and
  `tools/lint.sh` stay green; build.
- **Robot tests only when a step changes what the robot does** or needs the
  robot's numbers. A pure refactor is checked by the host tests (same output before
  and after) and rides along with the next robot test. Hand Daniel the steps in the
  style of "Robot tests" above and wait for the log.
- **After a step:** update ARCHITECTURE.md (what changed), TODO.md (what is done or
  new), README (how to use or test it), and the CHANGELOG (what happened, including
  any red flags found in a review pass).
- **Commit only when Daniel asks.**

## Conventions

- **VL53 rows and columns are numbered 1-8** everywhere people read them: the docs,
  the robot's console (`zone row 5 col 3`), the database (`row`, `col`). Row 1 is
  the top row, row 8 the bottom; column 1 is on the left as the robot sees it.
  Rows 1-4 look up, rows 5-8 down (row r's centre is (r − 4.5) × 5.625° below
  level); rows 7-8 see the floor close by, row 6 further out, row 5 meets it only
  beyond ~65 cm. Only array indexes in the code start at 0 (`row index 4` is row
  5); comments that name a row say "index" when they mean one. The sensor's own
  zone numbers (0-63, `zone` in the database) are not rows.

## Git conventions

- Commit to the `main` branch, always: no feature branches.
- Do not add Claude (or any AI assistant) as a co-author: no `Co-Authored-By:`
  trailer in commit messages.
