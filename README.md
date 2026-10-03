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
done; turns and 2 m straight to repeat once the front-right motor is fixed.

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
- **PicoB** (`picoB/app/`): wheel speed control per side on the front encoders, with
  the turn rate trimmed by the gyro (`drive.c`); position from the encoders, heading
  from the gyro with the bias re-measured whenever the robot stands still, tilt
  (`odometry.c`); the link protocol and safety stops (`brain.c`, PicoA as PicoB
  sees it). Motors stay off until PicoA greets it and switches them on; it switches
  them off by itself if PicoA's drive commands stop for 250 ms, a front wheel
  doesn't follow its target for 1 s, or the robot tilts more than 15°
  (`picoB/app/drive.h`).
- **PicoA** (`picoA/app/`): `body.c` is PicoB as PicoA sees it; `debug_console.c`
  prints a status line every 15 s on USB (none while a test runs) and takes keys:
  `g` motors on, `s` stop, `p` status now, `t` last test result, `l` link
  counters, `h` help; tests `q` square, `d` drift, `r` 10 turns, `f` / `b` 2 m
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

**M1 results so far** (3 Oct 2026, waxed wood):

| Test | Result |
|---|---|
| Tilt stop | Works: lifting one side stopped the square test with "tilted more than 15 deg" |
| Gyro drift, 10 min still | Bias −0.328 °/s, wandering only 0.009 °/s (−0.3328 … −0.3239). Even if all of that happened during one minute of driving: 0.5° of heading. Yaw frozen while still (0.00°). **No change needed** |
| 10 turns | Turn rate held at 28.6 °/s (the command) for 7 turns, then "right wheels not following". No gyro-scale result yet |
| 2 m straight | Ran, but the result was lost (see below) |
| Square (twice) | Odometry end pose 3.6 / −1.9 cm, 363.4° and −0.5 / −3.3 cm, 364.7°; real end pose not measured |

- **Front-right motor sticks** (hardware): it sometimes doesn't start, then runs
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

- **Hardware so far:** all four wheels — Pololu #2208 motors (298:1 LP 6V, exact
  297.92:1), 20 kHz PWM, front on channel A and rear on channel B of each driver.
  Left wheels: J10, driver pins PWMA GP22, PWMB GP28, direction GP27/GP26. Right
  wheels: J9, PWMA GP15, PWMB GP11, direction GP13/GP12. GP14 is STBY for both.
  AIN1+BIN1 / AIN2+BIN2 are tied on the PCB, so both motors on a side share
  direction: if a rear motor spins opposite its front one, swap its wires. If a
  whole side runs backwards, flip `LEFT_FORWARD` / `RIGHT_FORWARD` in
  `picoB/drivers/motor.c` (left is −1, right +1: both sides ran forward with these). Only the
  two front wheels have encoders (Pololu #3081): left-front on J6 pins 9–12
  (GP5/GP4), right-front on J6 pins 1–4 (GP9/GP8). There are no rear encoders.
  Adafruit #4502 ISM330DHCX breakout on SPI0 (GP16–19, mode 3, 1 MHz). The motors
  need battery power on J11; USB only powers the logic.
- **Differences from the PCB / PCB review:** the left/right labels are swapped on
  the motor and encoder connectors. The left motors are on **J10 (MOT_DRV_R, MR_\*
  nets)** and the right motors on **J9 (MOT_DRV_L, ML_\*)**. The left-front encoder
  is on **J6 pins 9–12 (ENC_RF_\*)** and the right-front one on **J6 pins 1–4
  (ENC_LF_\*)**. J6 pins 5–8 and 13–16 (rear encoders) are unused. The firmware
  follows the actual wiring, not the net names.
- **IMU:** WHO_AM_I check, 416 Hz, ±2 g / ±250 dps (register values from the ST
  datasheet), gyro bias averaged at power-up. Tilt about X and Y comes from a
  complementary filter (gyro short-term, gravity long-term, 0.5 s time constant),
  so pushes along an axis barely register as tilt.
- **Control:** tilt about the breakout's X axis picks a side, always driving forward.
  The accelerometer reads +1 g along whichever axis points up, so tilt X is positive
  with the breakout's +Y end raised: **+Y up → right wheels, +Y down → left wheels**;
  the other side coasts. ±15° dead zone, then power ramps linearly to 100 % at ±90°.
- **Encoders:** both front wheels, quadrature decoded in GPIO interrupts, reported as
  output-shaft revolutions and RPM (12 CPR × 297.92 ≈ 3575 counts/rev). ~50 RPM at full tilt on
  the battery, consistent with the datasheet's 45 RPM at 6 V.
- **Serial (USB):** nothing is printed and the driver stays in standby until a
  serial monitor is open; closing it stops the motor. Send `g` to enable, `s` to
  stop, `z` to zero revolutions. 10 Hz lines (units in the header): tilt X/Y (deg),
  accel ax/ay/az (g), gyro gx/gy/gz (deg/s), left/right power, revL/rpmL (left-front),
  revR/rpmR (right-front), RUN/STOP.

Keep the robot still for ~1 s after power-up (gyro bias). If an encoder counts
backwards while its wheel drives forward, flip that encoder's `direction` in
`picoB/drivers/encoder.c` (they are opposite to the motor signs, measured in the M0
square test).

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
`hm0360.pio`: camera; `drivers/tof.c` / `drivers/lib/vl53l8cx/`: ST ULD integration;
`drivers/opt4048.c`: OPT4048 polling and raw telemetry (conversion to lux/XYZ is in
`pc/bringup/src/opt4048.ts`).
In `picoB/`: `bringup/main.c` (tilt filter, control, serial), `drivers/imu.c`,
`drivers/motor.c`, `drivers/encoder.c`.
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
  298:1 LP 6V data and the 12 CPR encoder.
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
