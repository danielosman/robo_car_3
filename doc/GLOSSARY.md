# Glossary

The words the plans and the code use, and what each one means. One word per
thing: if a plan uses another word for one of these, the plan changes. Add terms
here as they come up.

## Robot and time

| Term | Meaning |
|---|---|
| **PicoA / PicoB** | PicoA: sensors and the brain (camera, VL53, WiFi). PicoB: wheels, encoders, IMU (gyro) |
| **pose** | Where the robot is and which way it faces (x, y, heading), plus pitch and roll; `pose.c` gives it for any moment in the recent past |
| **heading** | The robot's direction in the map, from the gyro. Angles: **+ = left** |
| **ego-motion** | The robot's own movement (turning, driving), as the gyro and encoders measure it. Everything a sensor sees shifts when the robot moves; that shift is ego-motion, not something moving in the room |
| **stamp** (`stamp_t`) | The time a measurement was **taken** (not received or processed), in PicoA's clock |
| **watchdog** | A hardware timer the firmware resets regularly; if the firmware hangs, it restarts the chip after ~1 s |

## Sensors

| Term | Meaning |
|---|---|
| **frame** | One complete reading of a sensor: a camera image (160 × 120 pixels, every 40 ms) or a VL53 reading of all 64 zones (every 67 ms) |
| **zone** | One of the VL53's 8 × 8 measuring directions, 5.6° wide; it reports the nearest surface inside its cone. Rows numbered 0 (top) to 7 (bottom) |
| **target** | One distance a zone reports; a zone can report several (nearest first) |
| **status** | The VL53's own verdict on a target (5 = valid, 6/9/10 = less sure, others = invalid) |
| **sigma** | The VL53's estimate of a target's distance uncertainty (mm) |
| **confidence** | How much a reading is believed, 0-1 (0-255 stored), from its status (`tof_quality`). One model for movement and the map |
| **credible** | Confidence ≥ 0.25: the reading is used at all; below it, it is ignored |
| **exposure / gain** | How long each camera row collects light / how much the signal is amplified. While watching: exposure ≤ 10 ms, gain ≤ 8 (sharp and dark rather than noisy) |
| **rolling shutter** | The camera exposes its rows one after another, so each row has its own time (`row0_us + row · line_us`) |
| **view** (`view`) | The module that knows where each sensor looks: zone and pixel directions, mounting, lens |

## Camera

| Term | Meaning |
|---|---|
| **feature** | A small spot in the image that can be found again in the next frame: in practice a corner, where brightness changes in two directions (a picture-frame corner, a table leg on the floor, a pattern). Plain walls have none; straight edges are only half useful |
| **tracking** | Finding each feature again in the next frame and giving it an id and an age |
| **measured move** | How far a feature moved in the image between two frames, as measured |
| **region** | One of 8 × 6 areas of the image (20 × 20 pixels) with summaries: feature count, texture, brightness, median measured move |
| **vision frame** (`vision_frame_t`) | The universal record of one camera frame: the features and regions with their measured moves, plus the robot's pose at the frame's times. Reports what was measured; nothing is subtracted |
| **predicted move** | How far a feature would move from ego-motion alone, computed from the robot's turn. Readers compute it, not the record |
| **leftover move** | Measured move minus predicted move: what the feature did on its own |
| **`vision`** | The module that finds and tracks features and fills the vision frame. Its readers: `camera_movers` now, others later (edges, landmarks, checking the gyro) |
| **search hint** | Inside `vision`: the gyro says where to look for a feature in the next frame. It changes where `vision` searches, never what it reports |
| **`ego`** | Small helper used by readers: from a vision frame and the robot's turn it computes predicted and leftover moves. Not part of the record |

## Movement

| Term | Meaning |
|---|---|
| **mover** | Something moving in the room (a person, a hand): camera features with a similar leftover move close together, or VL53 zones changing in a chain |
| **background** | What the sensor sees when nothing moves. The VL53 keeps it per **heading** (the heading background), so it stays valid while the robot turns |
| **edge time** | The moment a VL53 zone starts or stops seeing something nearer than its background; edge times along neighbouring zones give direction and speed |
| **observation** (`motion_obs_t`) | One sensor's report of a mover: direction (+ = left), distance, angular speed, confidence, stamp, which sensor |
| **event** | appeared / gone / stopped / removed, about a mover |
| **`movement`** | The module that takes observations from both sensors and keeps the tracked movers |

## Map

| Term | Meaning |
|---|---|
| **cell** | A 10 × 10 × 10 cm box of the map |
| **layer** | A height band of cells: **G** ground (−7…+3 cm above the floor), **L1** 3-13 cm, **L2** 13-23 cm |
| **slice** | The heights a zone's cone covers at the measured distance (z_lo…z_hi). Wholly in G → floor; wholly above → obstacle; across +3 cm → only free space up to it |
| **log-odds** | How a cell keeps its evidence: one number, raised by hits and lowered by misses, each step scaled by the reading's confidence |
| **free / occupied / unknown** | A cell's state from its log-odds |
| **drivable** | A column whose G is floor and whose L1 and L2 are both free |

## Process

| Term | Meaning |
|---|---|
| **step** | One item of the rework order (REWORK_PLAN.md), e.g. A2, T-R, V1; ends with a working robot and its robot tests |
| **recording / replay** | Sensor data (VL53 frames, odometry, keys) saved over WiFi to the Mac, then run through the same code there. No camera images for now |
| **measurement session** (M-S) | One session on the robot that records the situations the plans need and checks their assumptions |
| **host test** | A test that runs on the Mac (`./run_tests.sh`), no robot |
| **robot test** | Steps Daniel does on the robot, with what to expect and what to paste |
