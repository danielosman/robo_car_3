# RoboCar — custom PCB bring-up

Eventually a robot car; currently a test project for the two Pico 2 boards on the
custom PCB. The code is the source of truth for configuration and protocol details.

```
robo_car_3/
├── CMakeLists.txt       # Pico SDK setup + add_subdirectory(picoA/picoB); PICO_BOARD for both
├── pico_sdk_import.cmake
├── .vscode/             # Run / Flash / Debug ask: picoA or picoB
├── picoA/               # A1 — sensors: camera, ToF, color (MLX90640 planned)
│   ├── CMakeLists.txt
│   └── lib/vl53l8cx/    # ST ULD driver
├── picoB/               # A2 — motors, encoders, IMU
│   └── CMakeLists.txt
├── pc/                  # Node server + browser viewer for PicoA
└── build/               # one build dir for both (git-ignored)
    ├── picoA/picoA.uf2
    └── picoB/picoB.uf2
```

One configure and build produces both firmwares. Code shared between the Picos
(e.g. the inter-Pico UART protocol) will go in a `common/` folder.

## PicoA — working on the PCB

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

## PicoB — tilt → motor test (working on the PCB)

- **Hardware so far:** one Pololu #2208 motor (298:1 LP 6V, exact 297.92:1) on the
  left driver's AO1/AO2 — GP15 PWM (20 kHz), GP13/GP12 IN1/IN2, GP14 STBY — with a
  Pololu #3081 magnetic encoder on J6 pins 1–4 (GP9/GP8). Adafruit #4502
  ISM330DHCX breakout on SPI0 (GP16–19, mode 3, 1 MHz). The motor needs battery
  power on J11; USB only powers the logic.
- **IMU:** WHO_AM_I check, 416 Hz, ±2 g / ±250 dps (register values from the ST
  datasheet), gyro bias averaged at power-up. Tilt about X and Y comes from a
  complementary filter (gyro short-term, gravity long-term, 0.5 s time constant),
  so pushes along an axis barely register as tilt.
- **Control:** tilt about the breakout's X axis drives the motor — ±15° dead zone,
  then power ramps linearly to 100 % at ±90°; the sign sets direction.
- **Encoder:** quadrature decoded in GPIO interrupts, reported as output-shaft
  revolutions and RPM (12 CPR × 297.92 ≈ 3575 counts/rev). ~50 RPM at full tilt on
  the battery, consistent with the datasheet's 45 RPM at 6 V.
- **Serial (USB):** nothing is printed and the driver stays in standby until a
  serial monitor is open; closing it stops the motor. Send `g` to enable, `s` to
  stop, `z` to zero revolutions. 10 Hz lines (units in the header): tilt X/Y (deg),
  accel ax/ay/az (g), gyro gx/gy/gz (deg/s), power, rev, rpm, RUN/STOP.

Keep the robot still for ~1 s after power-up (gyro bias). If the encoder counts
backwards relative to motor direction, flip `DIRECTION` in `picoB/encoder.c`.

## Run

1. Build with the Pico VS Code extension (SDK/toolchain in `~/.pico-sdk/`, CMake/Ninja
   from Homebrew) or from a terminal (`PICO_SDK_PATH` etc. are set in `~/.zshrc`):
   ```sh
   cmake -S . -B build -G Ninja   # first time or after deleting build/
   cmake --build build
   ```
   If the extension downloads its own CMake/Ninja again, point `.vscode/settings.json` back
   at Homebrew's (one copy of each tool).
   Flash `build/picoA/picoA.uf2` onto **PicoA** and `build/picoB/picoB.uf2` onto
   **PicoB**. In VS Code, Run / Flash / Debug ask which firmware to use (`picoA` or
   `picoB`); they flash whichever Pico is on the USB cable, so pick the matching one.
   The steps below are for PicoA; PicoB only needs a serial monitor (see above).
2. With Node.js 24+, run `npm ci`, then `npm start` in `pc/`.
   With multiple Picos attached, use `npm start -- /dev/tty.usbmodem1101` (replace with PicoA's port;
   list candidates with `ls /dev/tty.usbmodem*`).
3. Open **http://127.0.0.1:8080/**. Restart the Node app after reflashing/reconnecting.
   Sensor initialization and errors appear in the page log.

In `picoA/`: `camera.c` (also the main loop and serial commands) / `hm0360_init.h` /
`hm0360_regs.h` / `hm0360_curves.h` (tone curves) / `hm0360.pio`: camera;
`tof.c` / `lib/vl53l8cx/`: ST ULD integration;
`opt4048.c`: OPT4048 polling and raw telemetry (conversion to lux/XYZ is in `pc/src/opt4048.ts`);
In `picoB/`: `main.c` (tilt filter, control, serial), `imu.c`, `motor.c`, `encoder.c`.
`pc/`: USB parsing (`src/protocol.ts`), server (`src/main.ts`) and viewer (`public/`). Run `npm test` in `pc/` for software checks;
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
  [`picoA/lib/vl53l8cx/LICENSE.txt`](picoA/lib/vl53l8cx/LICENSE.txt).

Local reference links assume this layout (this repo is `~/code/robo_car_3`):

- `~/kicad/robo_car_3/` — KiCad project and PCB review
- `~/code/hm0360/`, `~/code/vl53l8cx/`, `~/code/arducam_b0319/`, `~/code/opt-4048/` —
  sibling example projects (not yet copied to this Mac)
