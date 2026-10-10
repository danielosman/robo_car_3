# RoboCar

Firmware for a fully autonomous robot car on two Pico 2 boards and a custom PCB.
PCB bring-up is complete (tag `pcb-bringup-v1`): every connected part was
verified with the bring-up firmware and viewer described below. The robot
firmware (`app/` per Pico) is being built milestone by milestone to
[ROBOT_PLAN.md](ROBOT_PLAN.md). PicoA (a Pico 2 W) also sends its console over
WiFi to a server on the PC ([ROBOT_WIFI.md](ROBOT_WIFI.md)). The code is the source
of truth for configuration and protocol details.

```
robo_car_3/
├── CMakeLists.txt       # Pico SDK setup + add_subdirectory(picoA/picoB); PICO_BOARD for both
├── pico_sdk_import.cmake
├── .vscode/             # Run / Flash / Debug ask which firmware (picoA/… or picoB/…)
├── run_tests.sh         # host tests, no Pico needed
├── common/              # shared by both Picos: the inter-Pico link, time/angle/statistics helpers (+ host tests)
├── tools/               # stack_depth.py: worst-case stack from the call graph
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

## Robot firmware (`picoA_app`, `picoB_app`) — in progress

Built to [ROBOT_PLAN.md](ROBOT_PLAN.md), milestone by milestone. **Done: M0 (link,
wheel control, odometry), M1 (calibration, on waxed wood; `b` and the carpet still
to test), M2 (map).** Now: the rework ([doc/REWORK_PLAN.md](doc/REWORK_PLAN.md)):
the VL53 watches while still and while turning (`tof_motion.c`), the map is
[doc/MAP_DESIGN.md](doc/MAP_DESIGN.md)'s (`cell_map.c`), and watching follows the
biggest movement (`behaviour.c`); the camera's movement detection (`camera_motion.c`)
watches while still. What happened is in [CHANGELOG.md](CHANGELOG.md) and, for the
rework, [doc/REWORK_CHANGELOG.md](doc/REWORK_CHANGELOG.md); red flags in [REDFLAGS.md](REDFLAGS.md).

- **Link** (`common/link.c`, messages in `common/link_msgs.h`): UART0 GP0/GP1 on
  both Picos, 1 Mbaud, COBS frames with CRC-16, protocol v4.
- **Host tests:** `./run_tests.sh` runs them on the Mac, no Pico needed, and stops
  at the first failure: the link, the shared helpers, PicoB's wheel control, odometry and `brain`,
  PicoA's `body`, rangefinder, world map, pose, movement detection (change grid, VL53, camera, tracker), the actions
  (scan, move, turn, watch) in a simulated room, the WiFi console and the loop timer. The fake
  clock starts 30 s before `time_us_32()` wraps, so every test crosses the wrap (`npm test` in `pc/robot/` for the server). Run it after every change.
- **PicoB** (`picoB/app/`): wheel speed control per side on both encoders of the
  side (averaged), with the turn rate trimmed by the gyro (`drive.c`); position
  from the encoders, heading from the gyro with the bias re-measured whenever the
  robot stands still, tilt (`odometry.c`); the link protocol and safety stops
  (`brain.c`, PicoA as PicoB sees it). Motors stay off until PicoA greets it and
  switches them on; it switches them off by itself if PicoA's drive commands stop
  for 250 ms, any wheel doesn't follow its side's target for 1 s, or the robot
  tilts more than 15° (`picoB/app/drive.h`).
- **PicoA map** ([doc/MAP_DESIGN.md](doc/MAP_DESIGN.md)): `rangefinder.c` reads the
  VL53L8CX's 64 zones (8 × 8, 15 Hz) and knows their directions; `cell_map.c` shoots
  9 rays per zone as long as its reading into 10 cm cells in three layers (ground
  −7…+3 cm, 3-13 cm, 13-23 cm), 6 × 6 m: cells passed are evidence for free, the end
  cell for occupied, weighted by closeness and the VL53's status; a cell keeps its
  best measurement and changes for a better one or 3 good-enough ones in a row. No
  floor learning. `pose.c` gives the pose at the moment a frame was measured;
  `surroundings.c` feeds every frame to the map (in odometry's frame, from power-up);
  `behaviour.c` runs the actions. PicoB's clock is translated from the ODOM
  reports in `body.c`.
- **Start-up (`main.c`):** the robot does nothing by itself ([doc/COMMANDS_PLAN.md](doc/COMMANDS_PLAN.md)):
  motors off, it connects to WiFi (with or without USB) and waits for keys, the
  same on USB and over WiFi.
- **Actions** (`behaviour.c`), one per key, motors on while it runs and off at its
  end, which it prints (done with what odometry measured, or stopped and why):
  `s` scan (390° left in place, nothing else), `f` move 50 cm, `t` turn 30°, `a`
  watch for 1 minute. `r` switches the direction: back, `f` moves back and `t`
  turns right. Space stops; a new action replaces the running one.
- **Watching** (`a`, 1 minute). The VL53
  watches still and while turning (a background per world direction, no learning
  after a stop). The robot turns toward the biggest movement and follows it: its
  angular speed plus 2 × the angle still to go, at most ~29°/s, starting when it is
  8° off, stopping within 3° once it is about still, never past where it was last
  seen; movement at the edge of the view is inspected too. A thing that stops is
  background 1 s after the robot stands still. Space stops.
- **PicoA console** (`debug_console.c`), on USB and over WiFi, the same keys (one
  key, one operation; nothing printed by itself): space stop, `p` status (pose, link
  counters, WiFi, the loop time over the last 10 s, `loop_stats.c`, the free RAM, the
  direction), `h` help, `W` connect to WiFi; the actions above; map `C` clear, `m`
  print, `z` one ToF frame; camera `c` one frame as 20 × 15 blocks, with its
  exposure; movement lines only while watching (both sensors; each line starts with the robot's
  time in s), `o` each ToF zone's background,
  `k` each camera block's. Unplugging the USB doesn't stop the robot. The keys
  are printed ~1 s after a serial monitor opens or the robot server
  connects (sooner gets lost on the Mac).
- **WiFi console** (`wifi_console.c`, [ROBOT_WIFI.md](ROBOT_WIFI.md)): joins the
  network in `picoA/app/wifi_config.h` (git-ignored: copy
  `wifi_config.example.h`), finds the server by its UDP announcements, sends
  everything printed and takes keys; reconnects by itself, and losing it doesn't
  stop the robot.

### The map as printed (`m`)

Only on `m` (the scan doesn't print it). 4 × 4 m around the robot, 10 cm columns, up
= where the robot faced when the scan started; one symbol per column (two characters):

| Symbol | Meaning |
|---|---|
| `.` | drivable: floor seen (a ray ended in the ground layer), 3-23 cm free |
| `:` | free 3-23 cm, floor not seen (between the rings where rays end on the floor, or far) |
| `##` | blocked: something 3-13 cm high |
| `''` | overhang: something 13-23 cm high, nothing below it |
| `?` | no floor: rays went through the ground layer (a drop, a dark floor) |
| blank | unknown |
| `()`, `**` | the robot, and the point 30 cm ahead of it |

Standing still, the floor is seen in rings at each zone row's distance (~20, 30, 45,
70 cm); driving fills the rest.

Cells stay as last measured. A flat face often fills two cells (`####`): the map
marks where the face was seen, not how thick the object is.

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

**Known geometry limits:** each zone is 5.6° tall and reports the nearest surface
in it, so a low obstacle reads like the floor behind it until it is close (an 8 cm
box shows up at ~40 cm, not at 60 cm); a low obstacle can fall between two zone
rows at some distances; layer 0 (2-12 cm) is only seen within ~1 m.

**To watch:** a false `##` stays until the robot looks through it 6 times; stray
`##` that never go away would show it. Scans on a desk: keep a hand ready.

### Memory and stack (computed from the build, PicoA)

| | Value |
|---|---|
| RAM | 385 of 512 KB static (.bss 377, .data 8), ~136 KB free; no `malloc` linked |
| Core-0 stack | ~6 KB worst case: main loop 4.2 KB (`observations()` in `tof_motion.c` 3.2 KB), lwIP interrupt 0.75-1.3 KB, USB 0.25 KB. The SDK's 2 KB is nominal; the stack has 8 KB (both scratch banks) while core 1 is unused |

`tools/stack_depth.py` recomputes the stack (how to build for it: the script's
header). Re-run it after adding big locals and before core 1 is started.

### Robot tests (repeat after changes)

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
   and the end: yaw should stay put, the gyro bias wander little. (With telemetry,
   TELEMETRY_PLAN, this becomes a recording.)
3. **Gyro scale:** a mark on the floor in line with the robot's front; `t` 12 times
   (360°). Estimate how far it ended from the mark.
4. **Encoder distance:** tape measure, robot at 0; `f` four times (2 m). Measure
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

**WiFi console** (only PicoA; [ROBOT_WIFI.md](ROBOT_WIFI.md)):
1. Copy `picoA/app/wifi_config.example.h` to `wifi_config.h`, fill in the network,
   build, flash `picoA_app`.
2. Robot on USB, serial monitor open: the robot does nothing at start-up (motors
   off) and tries the WiFi.
3. `npm ci` then `npm start` in `pc/robot/`; open **http://127.0.0.1:8080/** ("Waiting
   for robot…"). The first time, allow `node` to accept incoming connections.
4. Expected ~3 s after power-up: "WiFi: connecting…", "WiFi: connected, IP …",
   "Server: found at …", "Server: connected" (`W` shows how it is connected). The
   page shows "Robot connected" and the keys.
5. Press `p` and `m` on the page: the answers show on the page and in the serial
   monitor. Unplug the USB (battery on): the page keeps working.
6. Power up without USB: the statuses show on the page, the robot waits; `s` on
   the page scans.

## PicoA bring-up (`picoA_bringup` + `pc/bringup`) — working on the PCB

- **HM0360 / Arducam B0319 camera:** the robot's camera driver (`drivers/camera.c`),
  streamed over USB: **160×120, Sub4, no binning**, exposure set by the driver.
  Binning gives stripes and isn't used ([notes](picoA/CAMERA_STRIPES_TODO.md)).
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

README.md, [ROBOT_PLAN.md](ROBOT_PLAN.md) and [ROBOT_WIFI.md](ROBOT_WIFI.md) show
**only the current state**: how things are and how to use and test them. They hold
no history. What happened (sessions, robot runs, results, fixes) goes only in
[CHANGELOG.md](CHANGELOG.md); red flags in [REDFLAGS.md](REDFLAGS.md).

## Git conventions

- Do not add Claude (or any AI assistant) as a co-author: no `Co-Authored-By:`
  trailer in commit messages.
