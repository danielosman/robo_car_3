# RoboCar — custom PCB bring-up

Eventually a robot car; currently a test project for **PicoA (Pico 2)** on the
custom PCB. The code is the source of truth for configuration and protocol details.

## Working on the PCB

- **HM0360 / Arducam B0319 camera:** 4-bit capture over PIO/DMA, streamed over USB.
  Default: **160×120, Sub4, no binning**. Horizontal-binning stripes remain
  unresolved; experiments and the even-column workaround are available in the
  viewer. See [follow-up notes](CAMERA_STRIPES_TODO.md).
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

MLX90640, motors and the inter-Pico link are not implemented yet.

## Run

1. Build with the Pico SDK / Pico VS Code extension; flash
   `build/robo_car_3.uf2` onto **PicoA**, not the motor Pico.
2. With Node.js 24+, run `npm ci`, then `npm start` in `pc/`.
   With multiple Picos attached, use `npm start -- COM7` (replace with PicoA's port).
3. Open **http://127.0.0.1:8080/**. Restart the Node app after reflashing/reconnecting.
   Sensor initialization and errors appear in the page log.

`camera.c` (also the main loop and serial commands) / `hm0360_init.h` /
`hm0360_regs.h` / `hm0360_curves.h` (tone curves) / `hm0360.pio`: camera;
`tof.c` / `lib/vl53l8cx/`: ST ULD integration;
`opt4048.c`: OPT4048 polling and raw telemetry (conversion to lux/XYZ is in `pc/src/opt4048.ts`);
`pc/`: USB parsing (`src/protocol.ts`), server (`src/main.ts`) and viewer (`public/`). Run `npm test` in `pc/` for software checks;
hardware behavior still needs PCB testing after changes.

## References used

- [PCB review and A1 pin map](file:///C:/Users/daniel/Documents/robo_car_3/PCB_REVIEW.md).
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
- [OPT4048 datasheet (TI SBOSA84)](https://www.ti.com/lit/ds/symlink/opt4048.pdf),
  especially §§8.3.4.5 (CRC) and 9.2.4 (XYZ / lux conversion).
- Source examples: [camera project](../arducam_b0319/arducam_b0319.c),
  [camera experiment history](../arducam_b0319/SESSION_LOG.md),
  [HM0360 tone curves / exposure controls](../hm0360/hm0360.c),
  [ToF project](../vl53l8cx/vl53l8cx.c),
  [OPT4048 project](../opt-4048/main.c) and [notes](../opt-4048/OPT4048_NOTES.md). ST's driver license is retained in
  [`lib/vl53l8cx/LICENSE.txt`](lib/vl53l8cx/LICENSE.txt).

Local reference links assume the original sibling-project layout.
