# RoboCar

Firmware for a fully autonomous robot car on two Pico 2 boards and a custom PCB.
PCB bring-up is complete (tag `pcb-bringup-v1`): every connected part was
verified with the bring-up firmware and viewer described below. The robot
firmware (`app/` per Pico) is being built milestone by milestone to
[ROBOT_PLAN.md](ROBOT_PLAN.md); a PC app comes later. The code is the source of
truth for configuration and protocol details.

```
robo_car_3/
├── CMakeLists.txt       # Pico SDK setup + add_subdirectory(picoA/picoB); PICO_BOARD for both
├── pico_sdk_import.cmake
├── .vscode/             # Run / Flash / Debug ask which firmware (picoA/… or picoB/…)
├── run_tests.sh         # host tests, no Pico needed
├── common/              # shared by both Picos: the inter-Pico link (+ host test)
├── picoA/               # A1 — sensors: camera, ToF, color (MLX90640 planned)
│   ├── CMakeLists.txt   # picoA_drivers library + one executable per firmware
│   ├── drivers/         # ToF, OPT4048, HM0360 registers/PIO; lib/vl53l8cx/ = ST ULD driver
│   ├── bringup/         # camera.c: PCB test firmware (camera driver + USB streaming)
│   └── app/             # robot firmware, the brain
├── picoB/               # A2 — motors, encoders, IMU
│   ├── CMakeLists.txt   # picoB_drivers library + one executable per firmware
│   ├── drivers/         # imu, motor, encoder
│   ├── bringup/         # main.c: tilt -> motor test firmware
│   └── app/             # robot firmware, the body
├── pc/
│   └── bringup/         # Node server + browser viewer for picoA_bringup
└── build/               # one build dir for all firmwares (git-ignored)
    ├── picoA/picoA_app.uf2, picoA_bringup.uf2
    └── picoB/picoB_app.uf2, picoB_bringup.uf2
```

One configure and build produces every firmware. Drivers are shared: each Pico's
`CMakeLists.txt` builds the robot firmware and the bring-up firmware from the same
`picoX_drivers` library, so the bring-up firmware keeps building and can be used
to re-test hardware (e.g. a new PCB revision). The camera driver is still
inside `picoA/bringup/camera.c`; it moves to `drivers/` when the app needs it.
Code shared between the Picos (the inter-Pico link) is in `common/`.

## Robot firmware (`picoA_app`, `picoB_app`) — in progress

Built to [ROBOT_PLAN.md](ROBOT_PLAN.md); red flags found on the way are logged in
[REDFLAGS.md](REDFLAGS.md). **M0 (link, wheel control, odometry) passed on the
robot** (2 Oct 2026). **M1 (calibration): in progress**: tilt stop and gyro drift
done; turns and 2 m straight to repeat. **4 Oct: new motors, all four encoders
connected and checked** (see "Motor change" below); square test passed again. **M2
(map) passed on the robot** (5 Oct 2026; results below the M2 test steps).

- **Link** (`common/link.c`, messages in `common/link_msgs.h`): UART0 GP0/GP1 on
  both Picos, 1 Mbaud, COBS frames with CRC-16.
- **Host tests:** `./run_tests.sh` runs them on the Mac, no Pico needed, and stops
  at the first failure: the link (framing, corruption, lost frames, resync), PicoB's
  wheel control (ramp, speed limits, speed and turn-rate control with a weak motor
  and skid, safety stops), odometry (straight, turning, gyro bias while still, tilt
  signs) and `brain` (greeting, MOTORS and DRIVE, safety stops reported once),
  PicoA's `body` (greeting, DRIVE and MOTORS repeats incl. lost messages, safety
  stops, PicoB restarting, timeout) and robot tests (against a simulated robot,
  incl. early stops). Run it after every change.
- **PicoB** (`picoB/app/`): wheel speed control per side on both encoders of the side (averaged), with
  the turn rate trimmed by the gyro (`drive.c`); position from the encoders, heading
  from the gyro with the bias re-measured whenever the robot stands still, tilt
  (`odometry.c`); the link protocol and safety stops (`brain.c`, PicoA as PicoB
  sees it). Motors stay off until PicoA greets it and switches them on; it switches
  them off by itself if PicoA's drive commands stop for 250 ms, any wheel
  doesn't follow its side's target for 1 s, or the robot tilts more than 15°
  (`picoB/app/drive.h`).
- **PicoA map (M2):** `rangefinder.c` turns the VL53L8CX's 64 zones (8 × 8, 15 Hz)
  into rays in the robot frame and tells floor from obstacles (≥ 2 cm above the
  floor); `world_map.c` keeps 10 cm cells, 4 layers, 4 × 4 m around the robot, with
  cells changing only when measured (an obstacle clears after 6 empty readings in
  a row) and remembering when; `pose.c` gives the pose at the moment a frame was measured;
  `surroundings.c` feeds the frames to the map; `behaviour.c` runs the start-up
  scan. PicoB's clock is translated from the ODOM reports in `body.c` (no extra
  message, PicoB unchanged).
- **Start-up scan:** the first time a serial monitor opens on PicoA after
  power-up (or with `n`), the robot turns 390° in place, learns the floor from what
  the lower zones see all around (fresh every start, nothing stored), prints the
  map, turns to face the most open direction and prints its heading.
- **PicoA** (`picoA/app/`): `body.c` is PicoB as PicoA sees it; `debug_console.c`
  prints a status line every 15 s on USB (none while a test runs) and takes keys:
  `g` motors on, `s` stop, `p` status now, `t` last test result, `l` link
  counters, `h` help; map: `n` start-up scan again, `m` print the map, `z` one ToF
  frame; tests `q` square, `d` drift, `r` 10 turns, `f` / `b` 2 m
  forward / back (`robot_test.c`). Unplugging the USB doesn't stop the robot, so a
  test can run without the cable. A test that stops early says why and what it
  measured so far. The keys and the last result are printed ~1 s after you plug
  in (sooner gets lost on the Mac), and again with `t`.

**M0 result** (square test on waxed wood, 50 cm sides at 10 cm/s, turns at 0.5 rad/s):

| | Odometry | Measured | Difference |
|---|---|---|---|
| End position, forward | 1.3 cm | 2.0 cm | 0.7 cm |
| End position, right | 1.1 cm | 1.4 cm | 0.3 cm |
| Total turn | 362.6° | ~362–363° | < 1° |

- Link: 0 bad and 0 lost frames. Speed held at 10.0 cm/s; turns at 26–32 °/s for a
  28.6 °/s command; heading within ±0.2° along each straight leg.
- The ~2.6° extra turn is the square test stopping each turn 0.5–1.8° late (the
  odometry measured it correctly), not an odometry error.
- **Roll dips to −3…−5° during turns in place:** the body leans on its tyres while
  skid-turning. Harmless for floor detection (which uses pitch), and kept as a
  **signal to use later**: e.g. a cross-check that the robot is really turning, or a
  hint of floor grip (wood vs carpet).
- Found on the robot and fixed: the IMU breakout is mounted turned 180° (X backward,
  Y right); the app never started the encoders; both encoder signs were reversed
  (see REDFLAGS.md).

**M0 test** (repeat after changes to `drive`, `odometry` or the link):
1. Flash `picoB_app` onto PicoB and `picoA_app` onto PicoA. Keep the robot still
   for ~1 s after power-up (gyro bias). Battery on J11 for the motors.
2. Open a serial monitor on **PicoA**. Expected: status lines with x/y/yaw, and
   PicoB's messages as `B: ...`. "PicoB not connected (received 0 bytes)" means the
   UART isn't working; `l` shows the link counters (`bad` and `lost` should stay ~0).
3. Without motors, check the IMU orientation: lift the **front** → pitch goes
   positive; lift the **left side** → roll goes positive; turn the robot **left** by
   hand → yaw goes up. (Verified; `to_robot_frame()` in `picoB/app/odometry.c`.)
4. On the floor with room for a 70 cm square: press `q`. The robot drives forward
   50 cm and turns left 90°, four times, pausing between legs, then prints where
   odometry thinks it is. Mark the start, measure the real end position and heading,
   and compare. `s` stops at any time.

**Square test after the motor change** (4 Oct 2026, new motors, four encoders,
waxed wood): odometry end pose 1.2 cm forward, 1.2 cm right, 362.6°; the real end
pose practically the same. No safety stops.

**10 turns after the motor change** (4 Oct, waxed wood): all 10 turns, no stops;
gyro 3600.5° in 126 s (28.5 °/s for a 28.6 °/s command); wheels −898.3 / +898.7 cm;
**effective track width 28.6 cm** (geometric 22.5 cm: the wheels skid 27 %); the
centre moved 0.1 cm by odometry.
- **Gyro scale:** the robot ended ~15° past the mark: the gyro reads **0.42 % too
  little** (~1.5° per full turn). Under 1 %: **no correction** (M5's full-turn check
  will correct it from the room).
- **Drift while turning in place:** the robot really ended ~15 cm back and 5 cm
  right (~1.5 cm per turn), which odometry can't see: equal and opposite wheel
  distances read as no movement. Small for one 390° scan; M6 (correction against
  the map) handles the build-up.

**2 m straight after the motor change** (4 Oct, waxed wood, USB unplugged):
odometry 200.2 cm (wheels 199.9 / 200.5 cm), heading −0.1°, 0.1 cm sideways; real
~200.5 cm: **+0.15 %**, so `WHEEL_DIAMETER_M` stays 9.0 cm. PicoA kept running on
the battery; the result was printed on reconnecting and with `t`.

**M1 results so far** (3 Oct 2026, waxed wood):

| Test | Result |
|---|---|
| Tilt stop | Works: lifting one side stopped the square test with "tilted more than 15 deg" |
| Gyro drift, 10 min still | Bias −0.328 °/s, wandering only 0.009 °/s (−0.3328 … −0.3239). Even if all of that happened during one minute of driving: 0.5° of heading. Yaw frozen while still (0.00°). **No change needed** |
| 10 turns | Turn rate held at 28.6 °/s (the command) for 7 turns, then "right wheels not following". No gyro-scale result yet |
| 2 m straight | Ran, but the result was lost (see below) |
| Square (twice) | Odometry end pose 3.6 / −1.9 cm, 363.4° and −0.5 / −3.3 cm, 364.7°; real end pose not measured |

- **Front-right motor sticks** (hardware; motor changed on 4 Oct, see "Motor change"): it sometimes doesn't start, then runs
  normally once moving; the rear-right wheel did the work in the turns, and the
  square test then kept stopping with "right wheels not following". The controller
  reaches full power within ~0.6 s and the rear-right motor on the same driver turns,
  so it's the motor, gearbox, its wiring or driver channel A, not the PWM level or
  the software. Checks: hub rubbing on the gearbox or fibres on the shaft, swap the
  two right motors on J9 (does the fault follow the motor?), wiggle the wires while
  driving, voltage at the motor terminals while stuck, battery voltage.
- **Console bugs found:** the welcome text and the last test result, printed the
  moment the USB connects, don't show up on the Mac (probably sent before the monitor
  reads); later tests then overwrite the stored result. Planned: print them ~1 s
  after connecting, and a key `t` to reprint the last result. Unexplained: after
  `f` and `g` were pressed (motors in a safety stop), nothing was printed, not even
  "Forward test…" / "Motors on", though the robot did drive about 2 m.

**M1 test** (calibration; both Picos need the M1 firmware, protocol v4). Paste the
serial log of each step:
1. **Tilt stop:** press `q` and lift one side of the robot past 15°. Expected:
   `B: Motors off: tilted too far` and the test stops. `g` switches the
   motors on again.
2. **Gyro drift, `d`:** robot on the floor, don't touch it for 10 min. Every 15 s:
   yaw (should stay put) and gyro bias. At the end: how far the bias wandered.
3. **Gyro scale, `r`:** a mark on the floor in line with the robot's front. It turns
   10 × 360° left (~2 min), one line per turn, and stops at 3600° by the gyro.
   Estimate how far it ended from the mark (N° short = gyro reads N/36 % too much).
   It also prints the effective track width from the wheels.
4. **Encoder distance, `f` / `b`:** tape measure along a 2 m path, robot at 0. Press
   `f`, then unplug the USB: it drives 2 m and stops. Measure where it really got
   to, plug back in and read its own numbers. `b` drives back.
   **First check that PicoA keeps running on the battery with the USB unplugged**
   (if it doesn't, PicoB stops after 250 ms with "drive commands stopped arriving").
5. **Carpet:** `r`, `f` and `q` again on the carpet, the square across its edge.
   Look for false safety stops and a slower turn rate.

**M2 test** (map; flash only **PicoA** with `picoA_app`, PicoB stays as it is):
1. Robot on the floor with ~1 m of room around it, battery on. Keep it still ~1 s
   after power-up (gyro bias).
2. Plug the USB into PicoA and open the serial monitor. After ~1 s: the keys, then
   "Start-up scan: turning 390 deg…". The robot turns ~14 s.
3. Expected: "Floor learned in 32 of 32 zones … rows 5-8 see it at … cm", then
   the map (40 × 40 cells, up = where the robot faced when the scan started, the
   robot at the centre), then "Most open direction…", a short turn, "Facing
   heading …".
4. Check the map against the room: walls and furniture legs where they are, `.`
   on the open floor, no `##` in the middle of open floor. Within ~1 m obstacles
   show as `##`; farther walls as `''` (only their part above 12 cm is seen).
   `.` = floor seen, nothing on it (only within ~50 cm: the floor rows); `:` =
   nothing in the way, floor not seen (farther, or over a drop); `?` = no floor
   where rows 7-8 expected it, 3 times (a drop, or a floor the sensor can't see);
   blank = never seen (behind objects, far away). Cells stay on the map once seen.
5. `z` prints one ToF frame (cm, as the robot sees it; `?N` = unsure, VL53
   status N). Checked 4 Oct: a hand on the robot's left shortens the left column.
6. Drive the square (`q`) and press `m`: the walls must stay where they were, and
   braking must not draw obstacles on the open floor. Paste the serial log.

Known limits (from the geometry): each zone is 5.6° tall and reports the nearest
surface in it, so a low obstacle reads like the floor behind it until it is close
(an 8 cm box shows up at ~40 cm, not at 60 cm); a low obstacle can fall between
two zone rows at some distances; layer 0 (2–12 cm, what blocks the robot) is only
seen within ~1 m.

First robot run (4 Oct): the sensor is not mirrored (hand test), but most zones
were thrown away (only the first of up to 4 targets was looked at), and the floor
rows read the near edge of their floor patch (6th row 35 cm instead of 48), which
drew a ring of false obstacles at ~60 cm and scattered far `''`. Fixed: closest
sure target, the 5th row learns the floor too, each floor zone learns its own
angle and scatter, readings that may be the floor at a zone's lower edge are free
space. The map's up is now the scan's start.

Second run (4 Oct): good map; PicoB stopped ("drive commands stopped arriving")
after the map print, and one false obstacle ~35 cm ahead-left (maybe the USB
cable). Fixed since, not yet run: the motors are off while the map is built and
printed, and at the end; `m` switches them off before printing; the 5th row no
longer takes far walls for its floor. `z` now also shows what each zone decided
and what each floor zone learned: if a false obstacle stays, point the robot at
it and paste `z`.

Third run (4 Oct): the start-up scan ran to the end without a stop; floor rows
6-8 learned (~45 / 30 / 22 cm), the 5th row not (it sees walls); the false obstacle
ahead-left is gone. Since: an unlearned 5th row is used like the rows above.

**M2 result** (5 Oct 2026, waxed wood, living room):
- Start-up scan with the latest firmware: the map matches the room (walls 1.5-2 m
  away, the table and sofa legs, an object ~40 cm behind); no `##` on open floor.
  Using the unlearned 5th row again turned the area behind-left from unknown into
  free out to the walls.
- Objects placed to the robot's left, then `n` and `q`: the scan mapped them and
  the robot faced away from them; after the square, `m` showed the walls and
  objects where they were and no obstacles from braking. Square: odometry 1.1 /
  −1.5 cm, 363.0° (as on 4 Oct).
- Found: isolated unknown cells inside the free area cut the free distance short
  (fixed after M2, below). Map changes counted 1584 after one scan and a square:
  too noisy as they stand for M5.
- `**` on the map is 30 cm ahead of the robot: with 10 cm cells it shows the
  heading to about ±20°. `n` clears the map and makes the robot's heading at its
  start the map's up.

**After M2 (5 Oct): gaps, drops, nothing fading.** Floor rays clear the low layer
all the way to where they meet the floor (no more gaps). The map no longer changes
with time: an obstacle clears only after 6 empty readings in a row. Rows 7-8 mark
`?` where the floor is missing (no return, or one beyond their floor patch, 3 times
with no floor seen in between); the 6th row's far readings (reflections off the
waxed floor, e.g. 184 cm = the wall) tell nothing. The floor is learned only from
readings that fit each zone's patch, and rows 7-8 fall back to the sensor's 7 cm
height, so a start-up scan next to a desk's edge still learns it. `.` marks only
where the floor was seen (within ~50 cm), `:` free space with the floor unseen.
On the robot: living room solid free area, no false `?` (2 in front of a white
glossy cupboard); on a desk, `?` along the edges and `:` beyond them.

## PicoA bring-up (`picoA_bringup` + `pc/bringup`) — working on the PCB

- **HM0360 / Arducam B0319 camera:** 4-bit capture over PIO/DMA, streamed over USB.
  Default: **160×120, Sub4, no binning**. Horizontal-binning stripes remain
  unresolved; experiments and the even-column workaround are available in the
  viewer. See [follow-up notes](picoA/CAMERA_STRIPES_TODO.md).
- **VL53L8CX ToF:** working **8×8** ranging over SPI0; default **10 Hz**, up to four
  targets per zone. LPn is not wired to the Pico—the Pololu carrier holds it high.
- **OPT4048 color sensor:** I2C0 0x44 (GP4/5, 400 kHz), auto-range, 100 ms per
  channel (~2.5 complete samples/s). Only coherent, CRC-checked cycles are shown:
  lux, CIE xy/XYZ (datasheet example matrix, not board-calibrated), approximate
  color swatch and raw channel counts.
- **Shared browser viewer:** camera, ToF matrix and OPT4048 panels side by side,
  separate controls, diagnostic metrics and sensor logs. Camera controls: capture
  mode (also restarts streaming), display gamma / auto-stretch, and sensor-side
  tone curve, auto-exposure and manual exposure / analog / digital gain, read back
  from the HM0360. ToF display starts **rotated 90°** to match PCB mounting
  (display only, not camera/ToF calibration). Farthest selection means the
  farthest valid *returned* target, not necessarily every object in the zone.

MLX90640 and the inter-Pico link are not implemented yet.

## PicoB bring-up (`picoB_bringup`) — tilt → motor test, working on the PCB

- **Hardware so far:** all four wheels — GA46-N20E-0043 N20 gearmotors (298:1,
  50 RPM at 6 V) with Hall encoders (see "Motor change" below), 20 kHz PWM, front on channel A and rear on channel B of each driver.
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
- **Motor change (4 Oct 2026):** all four motors replaced (the old ones were
  Pololu #2208, 298:1 LP 6V, front ones with #3081 encoder boards) by
  **GA46-N20E-0043**: N20 gearmotor, 298:1 (medium power), 50 RPM at 6 V, stall
  0.6 A (the TB6612FNG gives 1.2 A per channel, so fine), with a Hall encoder
  board (green power LED, powered from 3V3_B on J6). Encoder: **7 pulses per
  channel per motor turn = 28 counts** on every edge, × 298 = **8344 counts per
  wheel turn**, inferred from the first run (126 rpm with the old 12-count
  constant = 54 rpm) and confirmed: ~55 rpm on all four wheels at full power,
  lifted (≈ 0.26 m/s, about the same as the old motors).
  All four encoders are connected. Signs: **all four checked** (+ =
  forward). The left encoders' power was first wired reversed (LEDs off, boards
  cool); they work normally since the fix. Check with `picoB_bringup` (below) before the
  robot firmware: its wheel stop would otherwise switch the motors off within 1 s.
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
  counts/rev). With the old motors: ~50 RPM at full tilt on the battery,
  consistent with the datasheet's 45 RPM at 6 V; new motors: ~55 RPM.
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
   cmake --build build
   ```
   If the extension downloads its own CMake/Ninja again, point `.vscode/settings.json` back
   at Homebrew's (one copy of each tool).
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

In `picoA/`: `bringup/camera.c` (camera driver, main loop and serial commands) with
`drivers/hm0360_init.h` / `hm0360_regs.h` / `hm0360_curves.h` (tone curves) /
`hm0360.pio`: camera; `drivers/tof.c` / `drivers/lib/vl53l8cx/`: ST ULD integration
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
- [VL53L8CX datasheet (ST)](https://www.st.com/resource/en/datasheet/vl53l8cx.pdf)
  ([local copy](../vl53l8cx/spec/vl53l8cx.pdf)).
- [ST UM3109 ULD guide](https://www.st.com/resource/en/user_manual/um3109-a-guide-for-using-the-vl53l8cx-lowpower-highperformance-timeofflight-multizone-ranging-sensor-stmicroelectronics.pdf)
  ([local copy](../vl53l8cx/spec/um3109-a-guide-for-using-the-vl53l8cx-lowpower-highperformance-timeofflight-multizone-ranging-sensor-stmicroelectronics.pdf)):
  initialization, SPI/LPn, ranging settings, multiple targets and validity statuses.
- [ISM330DHCX datasheet (ST DS13012)](../../kicad/robo_car_3/specs/ism330dhcx.pdf),
  §§9.11–9.33 (registers) and the initialization procedure (p. 37);
  [Adafruit #4502 notes](../../kicad/robo_car_3/specs/adafruit_4502_ISM330DHCX.txt).
- [Pololu Micro Metal Gearmotors datasheet Rev 6.2](../../kicad/robo_car_3/specs/pololu_2208_micro_metal_gearmotors_datasheet.pdf):
  298:1 LP 6V data and the 12 CPR encoder (the motors before 4 Oct).
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

## Git conventions

- Do not add Claude (or any AI assistant) as a co-author: no `Co-Authored-By:`
  trailer in commit messages.
