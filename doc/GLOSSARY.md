# Glossary

The words the docs and the code use, and what each one means. One word per thing.
Add terms here as they come up. Older plans in [archive/](archive/) used some
words that are gone (log-odds, slice, credible, step); their meaning is in those
plans.

## Robot and time

| Term | Meaning |
|---|---|
| **PicoA / PicoB** | PicoA: sensors and the brain (camera, VL53, WiFi). PicoB: wheels, encoders, IMU (gyro) |
| **pose** | Where the robot is and which way it faces (x, y, heading), plus pitch; `pose.c` gives it for any moment of the last ~1.3 s |
| **heading** | The robot's direction in the map, from the gyro. Angles: **+ = left** |
| **ego-motion** | The robot's own movement (turning, driving), as the gyro and encoders measure it. Everything a sensor sees shifts when the robot moves; that shift is ego-motion, not something moving in the room |
| **action** | Something the robot does over time on a key: scan, move, turn, watch, record 5 s, go to (a click on the map). One at a time; it ends done, stopped, replaced, on an obstacle, or on a safety stop |
| **mode** | Who chooses the actions: **idle** (nobody; after power-up and stop) or **manual** (the operator, by keys and clicks on the map) |
| **goal** | A point on the map (its frame) the robot is sent to by a click in manual mode |
| **safety stop** | PicoB switching the motors off by itself: no drive command for 250 ms, a wheel not following, or tilted beyond 15° |

## Sensors

| Term | Meaning |
|---|---|
| **frame** | One complete reading of a sensor: a camera image (160 × 120 pixels) or a VL53 reading of all 64 zones (every 67 ms) |
| **zone** | One of the VL53's 8 × 8 measuring directions, 5.6° wide; it reports the nearest surfaces inside its cone. Rows and columns numbered 1-8 (row 1 at the top, column 1 on the left as the robot sees it) |
| **target** | One distance a zone reports; a zone can report up to four (nearest first) |
| **status** | The VL53's own verdict on a target (5 = valid, 6 and 9 = valid, less sure, others = not used) |
| **sigma** | The VL53's estimate of a target's distance uncertainty (mm) |
| **exposure / gain** | How long each camera row collects light / how much the signal is amplified |
| **rolling shutter** | The camera exposes its rows one after another, so each row has its own time (`camera_row_time()`) |

## Map

| Term | Meaning |
|---|---|
| **cell** | A 10 × 10 × 10 cm box of the map: unknown, free or occupied, with the confidence of the measurement that set it |
| **layer** | A height band of cells: **G** ground (−7…+3 cm above the floor), **L1** 3-13 cm, **L2** 13-23 cm |
| **column** | The three cells above one 10 × 10 cm square of floor; read as blocked, overhang, no floor, drivable, open or unknown |
| **ray** | A straight line from the sensor; each zone shoots 9, as long as its reading. It **passes** the cells it crosses and **hits** the one it ends in |
| **reading** | A zone's distance in one frame, judged as a whole: **obstacle** (closer than its cone can meet the floor), **floor**, or **unsure** |
| **q** | One frame's confidence in its verdict on a cell, 0-1: how many full-strength rays agree, out of 9 |
| **drivable** | A column whose G is floor and whose L1 and L2 are both free |
| **window** | The 6 × 6 m of cells the robot keeps around itself, 3 × 3 tiles; it scrolls by a tile to keep the robot in its middle tile. Its **anchor** is its corner's world cell |
| **tile** | 2 × 2 m (20 × 20 columns) of the whole-house map |
| **whole-house map** | Every column the window has seen, as its class (4 bits), in tiles; the window overwrites it where it knows something |

## Movement

| Term | Meaning |
|---|---|
| **background** | What a sensor sees when nothing moves. The VL53 keeps it per world direction (heading bins), so it stays valid while the robot turns; the camera per block, learned after each stop |
| **blob** | Moved cells (zones or camera blocks) that touch, reported as one observation |
| **observation** (`motion_obs_t`) | One detector's report of a blob: direction (+ = left), extent, distance if known, strength, time |
| **target** | (in the tracker) the one blob the tracker follows; only logged |
| **mover** | Something moving in the room (a person, a hand) |

## Recording

| Term | Meaning |
|---|---|
| **take** | One action recorded, from its start to its end (`R` on) |
| **boot** | One power-up of PicoA, with a random id; all takes of a boot share one odometry frame |
| **datagram** | One UDP packet of records (≤ 1400 B), numbered, so the PC sees what was lost |
| **replay** | Running the robot's own code (the map) on a recorded take on the Mac |
| **snapshot** | What the robot knows at one moment, sent after every action, on `S` and to a new server: pose, camera picture and features, one ToF frame, colour, map window and changed tiles |
| **full telemetry** | The recordings (takes), as opposed to snapshots; `R`, off at power-up |
| **truth box** | Where something really stood, placed by hand in the viewer to score the map |

## Process

| Term | Meaning |
|---|---|
| **host test** | A test that runs on the Mac (`./run_tests.sh`), no robot |
| **robot test** | Steps Daniel does on the robot, with what to expect and what to paste |

## Camera tracking ([archive/CAMERA_MOTION_PLAN.md](archive/CAMERA_MOTION_PLAN.md); features built, V1; the rest planned)

| Term | Meaning |
|---|---|
| **feature** | A small spot in the image that can be found again in the next frame: in practice a corner. Plain walls have none; straight edges are only half useful |
| **tracking** | Finding each feature again in the next frame and giving it an id and an age |
| **measured move** | How far a feature moved in the image between two frames, as measured |
| **predicted move** | How far a feature would move from ego-motion alone, computed from the robot's turn |
| **leftover move** | Measured move minus predicted move: what the feature did on its own |
| **region** | One of 8 × 6 areas of the image (20 × 20 pixels) with summaries |
| **vision frame** | The record of one camera frame: features and regions with their measured moves, and the pose |
| **`vision`, `ego`** | The module that finds and tracks features; the helper that computes predicted and leftover moves |
