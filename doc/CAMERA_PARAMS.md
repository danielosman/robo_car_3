# Camera parameters

The camera's numbers as we take them now, and the data sheets' numbers next to them.
The code's copies: `CAMERA_GEOMETRY` in `pc/robot/src/objects/place.ts` (placing
objects) and `FOCAL_PX` in `picoA/app/camera_motion.c` (movement bearings in the
log). How they were measured: [OBJECTS_PLAN.md](OBJECTS_PLAN.md), "The camera
geometry". The ToF: [TOF_PARAMS.md](TOF_PARAMS.md).

## The module

- **Arducam HM0360 VGA module (B0319)**, its stock lens.
- **Sensor: Himax HM0360.** Greyscale, rolling shutter, 3.6 µm pixels, 1/6″.
- **Its pixels:** 656 × 496 in all, 640 × 480 of them active (Himax data sheet).

## The picture we read

- **The window:** the sensor's 640 × 480 active pixels (`WIN_MODE`, `hm0360_init.h`).
- **Subsampled by 4, no binning:** every 4th column and row is read, giving 160 × 120.
  Binning gave vertical black stripes
  ([archive/CAMERA_STRIPES_TODO.md](archive/CAMERA_STRIPES_TODO.md)).
- **The readout modes** ("Full (VGA), Bin2 / Sub2, Bin4 / Sub4, Fast ROI") are pairs:
  each size has two ways to get it. 320 × 240 by Bin2 (average 2 × 2) or Sub2 (keep
  every 2nd); 160 × 120 by Bin4 (average) or Sub4 (keep every 4th). Himax's data sheet
  says the same: "QVGA (sub2 or bin2), QQVGA (sub4 or bin4)".
- **The registers:** `0x0380` / `0x0381` set the size (Sub2 or Sub4, across and
  down); `0x0382` turns that into binning (bit 1 across, bit 0 down). So Bin4 is
  `0380=02, 0381=02, 0382=03`, and Sub2 with Bin4 isn't a mode.
- **Both binning modes were tried and both gave stripes:** Bin4 (`0382=03`) and Bin2
  across (Sub2 with horizontal binning). So the stripes don't come from a mode that
  doesn't exist; the cause is still open (the stripes doc's list).
- So the picture shows **the whole 640 × 480 window**, and one picture pixel is 4
  sensor pixels apart: 14.4 µm on the sensor.
- About 24 frames a second (the corner tracker uses every 3rd, ~8 a second). Exposure
  in 10 ms steps, against mains flicker.

## What we take as true

| | Value | How we know |
|---|---|---|
| Focal length | **f = 188 px** (picture pixels) = 752 sensor pixels = **2.71 mm** | the table's width at a ToF distance, 17 Oct |
| Field of view | **46° across, 35° up-down, 56° diagonal** | from f 188, no distortion |
| Height of the lens | **8.8 cm** above the floor | measured by Daniel, 17 Oct |
| Position | **2.5 cm ahead** of the robot's centre, **0.5 cm right** | measured by Daniel, 17 Oct |
| Tilt | **−1.86°** (down), on top of the robot's pitch | the pouf and the table, 17 Oct |
| Picture centre | (80, 60) | assumed |
| Distortion | none | assumed |
| Yaw (aim left-right), roll | 0 | assumed |

## Arducam's specs (the sensor)

As Arducam lists them (copied 18 Oct):

| | |
|---|---|
| Active pixel array | 656 × 496 (Himax: 656 × 496 in all, 640 × 480 effective) |
| Pixel size, technology | 3.6 µm × 3.6 µm, BSI |
| Diagonal (optical format) | 2.88 mm (1/6″) |
| Colour filter | Bayer or monochrome (ours: monochrome) |
| Frame rate at 24 MHz | QQVGA 1 fps to VGA 60 fps |
| Readout modes | Full (VGA), Bin2 (mono) / Sub2, Bin4 (mono) / Sub4, Fast ROI |
| Signal to noise, max | 45.5 dB |
| Dynamic range (1×) | 85 dB |
| Sensitivity | 5.5 V/lux·s at 530 nm; 15 V/(µW·cm⁻²·s) at 850 nm |
| Chief ray angle, max | 35.74° |
| Supply | analog 2.8 V, digital 1.2 V (1.8 V LDO option), I/O 1.8-3.0 V |
| Reference clock (EXTCLK) | 6-24 MHz; internal oscillator too |
| Serial interface | I²C, 1 MHz max, single or burst |
| Current (8-bit parallel, >5 pF load) | QVGA (Sub2) 2 fps 140 µA; QVGA 60 fps 3.2 mA; VGA 60 fps 7 mA |

## The lens's data sheet numbers, for comparison

Arducam's listing gives two numbers that don't agree with each other, and neither
agrees with ours.

| | Ours | From the 2.59 mm focal length | From the 64° diagonal |
|---|---|---|---|
| f, picture pixels | **188** | 180 | 160 |
| f, mm | **2.71** | 2.59 | 2.30 |
| Across | **46°** | 48° | 53° |
| Up-down | **35°** | 37° | 41° |
| Diagonal | **56°** | 58° | 64° |

- Every listing of the stock lens says 64° diagonal (Arducam, Arduino, UCTRONICS,
  OpenMV). No source gives a field of view for the B0319 itself.

## Why f is in pixels

- f in pixels is the distance from the lens to the picture, measured in the
  picture's own pixels. A pixel `p` pixels from the centre looks `atan(p / f)` off
  the middle.
- These are **the 160 × 120 picture's pixels**, because that is the picture the code
  sees. f doubles at 320 wide and is 4× at 640.
- In sensor pixels f is 188 × 4 = 752, and in mm it is 752 × 3.6 µm = 2.71 mm.

## Why our f doesn't match the 64°

**In short:** both are likely right. The lens squeezes the edges (barrel distortion),
so near the middle it acts like f ≈ 188 px (56° diagonal), while the whole frame
reaches 64° at the corners, as if f were 160. Not checked with a calibration (Daniel,
18 Oct: none planned); for us it means ~2° of bearing error near the left and right
edges.

- **Not the sensor's size.** The full array is 2.5% wider than the window we read
  (656 vs 640). The gap is 17% (188 vs 160).
- **Most likely the lens's distortion.** A data sheet's diagonal angle is the real
  angle to the corner. A wide lens squeezes the edges (barrel distortion), so the
  corner sees further out than a lens without distortion and the same f would. Our
  f is measured near the middle. To reach 64° at the corners, they would have to be
  squeezed by about 15%.
- That would make the **2.59 mm right** (within 5% of ours) and the 64° the real
  angle including distortion. Not checked yet.
- Other possibilities: the 64° belongs to another lens, or our f is a few % high (a
  table seen at a slight angle reads wider).
- **What it would change:** near the left and right edges, bearings would be off by
  ~2°. In the middle, nothing.
- **How to check:** a straight vertical edge (a door frame) near the picture's side
  bends outward with barrel distortion. Or the same object, at the same ToF
  distance, measured in the middle and at the edge. Or a checkerboard calibration
  (OpenCV): it gives f and the distortion; with barrel distortion its k₁ is negative.
- **Our 188 isn't only the middle:** the table's legs reach column 147, 67 px from
  the centre (the edge is 80). With barrel distortion the true f at the centre is a
  few % higher. The pouf's height (near the middle, up-down) gave 182, so the two
  measurements only agree to a few %.

Sources: [Arducam HM0360 module](https://www.arducam.com/hm0360-vga-monochrome-dvp-camera-module-for-arduino-giga-r1-wifi-board.html),
[HM0360 data sheet](https://www.welectron.com/mediafiles/productimg/arducam/Datasheet/HM0360-image-sensor-datasheet.pdf).
