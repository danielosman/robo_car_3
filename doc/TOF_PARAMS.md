# ToF parameters

The ToF's numbers as we take them now, and the data sheet's numbers next to them. The
code's copies: `picoA/app/rangefinder.c` and `rangefinder.h`. The camera:
[CAMERA_PARAMS.md](CAMERA_PARAMS.md).

## The sensor

- **ST VL53L8CX**, 8 × 8 zones, up to 4 targets per zone (all four are read).
- **Settings:** 15 frames a second, 5 ms integration, sharpener 5.
- **Zones are numbered 1-8** where people read them, row 1 at the top (README,
  "Conventions").

## What we take as true

| | Value | How we know |
|---|---|---|
| Field of view | **45° × 45°**, 8 even zones of **5.625°** | data sheet |
| Aim | level: rows 1-4 look up, rows 5-8 down at the floor | assumed, no tilt modelled |
| Mounting | turned 90° on the PCB | bring-up viewer |
| Height | **7 cm** above the floor | measured by Daniel |
| Position | **3.2 cm ahead** of the robot's centre, **2.6 cm right** | measured by Daniel, 17 Oct |
| From the camera | 0.7 cm ahead, 2.1 cm right, 1.8 cm lower | from the two above |
| Targets kept | per zone the nearest sure one (status 5, 6 or 9), and the next sure one behind it | `rangefinder.c` |
| Skipped | the sensor's own echo at ~30 cm (status 12 or 14) | seen in recordings |

## The data sheet, for comparison

| | Ours | Data sheet |
|---|---|---|
| Field of view | 45° × 45° | 45° × 45°, 65° diagonal |
| Light cone | — | 43.4° wide at 75% of the peak signal, 57.9° at 10% |
| Range | — | 400 cm in the dark; ~2.8 m in bright indirect light (5000 lux, Pololu) |
| Frame rate | 15 a second | at most 15 at 8 × 8 (60 at 4 × 4) |
| Accuracy | — | not found in the sources checked |

- The light cone is weaker at the edges: the outer zones get less signal and are less
  sure, mostly at long range.

## Not measured

- **Tilt**, the aim up or down: taken as level. Only the floor zones care: 1° moves
  where row 8 meets the floor (~20 cm away) by ~5%, and row 5 (~140 cm) by 25% or
  more. Distances to objects don't change.
- **Yaw**, the aim left or right: taken as straight ahead.
- **The zone angles:** taken as even, 5.625° each.

Sources: [VL53L8CX data sheet (ST)](https://www.st.com/resource/en/datasheet/vl53l8cx.pdf),
[Pololu VL53L8CX carrier](https://www.pololu.com/product/3419).
