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
| **action** | Something the robot does over time on a command: drive, turn, go to (a click on the map), watch. One at a time, each one **move**; it ends done, stopped, replaced, on an obstacle, or on a safety stop |
| **move** | One action with its still time around it, the unit the PC stores (`move.c`): start (motors on, standing still) → still **pre_ms** → act → still **post_ms** (the settling) → end. Sends move start and move end; the ToF frames and imu batches go to the PC only during a move. Numbered from 1 per boot. One command is one move (a go to's turn and drive too) |
| **pre_ms / post_ms** | How long a move stands still before it acts and after (200 ms each; `robot config pre_ms N`) |
| **mode** | Who chooses the actions: **idle** (nobody; after power-up and stop) or **manual** (the operator, by keys and clicks on the map) |
| **goal** | A point on the map (its frame) the robot is sent to by a click in manual mode |
| **safety stop** | PicoB switching the motors off by itself: no drive command for 250 ms, a wheel not following, or tilted beyond 15° |

## Sensors

| Term | Meaning |
|---|---|
| **frame** | One complete reading of a sensor: a camera image (160 × 120 pixels) or a VL53 reading of all 64 zones (every 100 ms; 67 ms until Nov) |
| **zone** | One of the VL53's 8 × 8 measuring directions, 5.6° wide; it reports the nearest surfaces inside its cone. Rows and columns numbered 1-8 (row 1 at the top, column 1 on the left as the robot sees it) |
| **target** | One distance a zone reports; the sensor reports up to four (nearest first). The robot keeps one: the first with status 5, 6 or 9 |
| **status** | The VL53's own verdict on a target (5 = valid, 6 and 9 = valid, less sure, others = not used). Only `tof.c` reads the numbers; the rest read the **kind** |
| **kind** | How sure a zone's target is: **sure** (status 5), **less sure** (6 or 9), **unsure** (targets, none valid), **nothing** (no target in range) |
| **tof_frame** | One VL53 frame as `tof` reads it: per zone distance (mm), signal, sigma, kind and how many targets the sensor saw; the frame's number, middle time and its own period |
| **posed_frame** | A tof_frame and where the robot was while it was measured: x, y, yaw (±180°), pitch, its turn during the frame, still, pose known. Made by the **stamp** (`pose_stamp()`) |
| **use** | What a frame is packed for: **map**, **watch**, **snapshot**. Each use gets a format from a table in `tof_format` |
| **tof_format** | A byte layout of a posed frame and its pack and unpack. Today one, **compact** (277 B: 21 B header, 4 B a zone, distance in whole cm, 255 = beyond 2.54 m), for every use |
| **rays** (`tof_rays`) | A zone's reading placed in the world: its 9 rays, their end point and height span, and its **verdict** (floor, obstacle, unsure) |
| **sigma** | The VL53's estimate of a target's distance uncertainty (mm) |
| **exposure / gain** | How long each camera row collects light / how much the signal is amplified |
| **mounting** | Where the sensors sit on the robot: the ToF's and the camera's position, the ToF's zone angle and turn, the camera's f and tilt, the IMU's axes. One file, `common/mounting.json`; the C code gets it as a parameter (`mounting_t`); the PC keeps a copy per **boot**, taken when the boot was first seen |
| **imu_sample** | One IMU reading, 416 a second: accel in g and gyro in °/s, the sensor's axes, the gyro's power-up bias taken out (PicoB's driver, `imu_sample_t`). Odometry uses it |
| **imu_record** | 4 imu_samples averaged (104 a second): accel and gyro x, y, z as the sensor's own int16 counts (0.061 mg, 8.75 mdps a count), its axes, nothing taken out (raw), and the time (the middle of its samples). Made by PicoB's `odometry`, sent to PicoA in `MSG_IMU`, on PicoA's clock after `body`, stored on the PC (`imu` table): training data |
| **imu_batch** | Up to 10 imu_records of a move (~0.1 s), evenly spaced: the move's number, the first record's time, the period, the count, the records. Made by `move` during a move, sent on bulk; dropped and counted when the lane is full |
| **odom_report** | PicoB's odometry, 50 a second (`MSG_ODOM`): pose (x, y, yaw, pitch, roll), speeds, wheel distances, still, motors on, stop reason. No peaks since step 4c |
| **rolling shutter** | The camera exposes its rows one after another, so each row has its own time (`camera_row_time()`) |

## Map

| Term | Meaning |
|---|---|
| **cell** | A 10 × 10 × 10 cm box of the map: unknown, free or occupied, with the confidence of the measurement that set it |
| **layer** | A height band of cells: **G** ground (−7…+3 cm above the floor), **L1** 3-13 cm, **L2** 13-23 cm |
| **column** | The three cells above one 10 × 10 cm square of floor; read as blocked, overhang, no floor, drivable, open or unknown |
| **ray** | A straight line from the sensor; each zone shoots 9, as long as its reading. It **passes** the cells it crosses and **hits** the one it ends in |
| **reading** | A zone's distance in one frame, judged as a whole (the **verdict**, `tof_rays`): **obstacle** (closer than its cone can meet the floor), **floor**, or **unsure** |
| **q** | One frame's confidence in its verdict on a cell, 0-1: how many full-strength rays agree, out of 9 |
| **drivable** | A column whose G is floor and whose L1 and L2 are both free |
| **window** | The 6 × 6 m of cells the robot keeps around itself, 3 × 3 tiles; it scrolls by a tile to keep the robot in its middle tile. Its **anchor** is its corner's world cell |
| **tile** | 2 × 2 m (20 × 20 columns) of the whole-house map |
| **whole-house map** | Every column the window has seen, as its class (4 bits), in tiles; the window overwrites it where it knows something |

## Watching

| Term | Meaning |
|---|---|
| **watch** | Watching for movement around the robot: the module `watch` (`watch.h`, the one interface; inside it `watch_tof`, `watch_camera`, `tracker`). Was "movement detection" (`motion_sense`, `tof_motion`, `camera_motion`) until 7 Nov |
| **background** | What a sensor sees when nothing moves. The VL53 keeps it per world direction (heading bins), so it stays valid while the robot turns; the camera per block, learned after each stop |
| **blob** | Moved cells (zones or camera blocks) that touch, reported as one observation |
| **observation** (`watch_obs_t`) | One detector's report of a blob: direction (+ = left), extent, distance if known, strength, time |
| **target** | (in the tracker) the one blob the tracker follows; only logged |
| **mover** | Something moving in the room (a person, a hand) |

## Data on the PC

| Term | Meaning |
|---|---|
| **boot** | One power-up of PicoA, named by a random 8-hex id; all its moves share one odometry frame. On the PC (`boot.ts`): its own copy of the mounting, its messages numbered in the order they came, its clock |
| **boot time** | Seconds since a boot's power-up (`t_s`): PicoA's 32-bit µs clock (`t_us`, wraps every 71.6 min) unwrapped so it only grows |
| **store** | The one DuckDB file the PC writes as messages arrive (`store.ts`, `recordings/robot2.duckdb`): boots, moves, frames, zones, imu, status, snapshots, objects. The old `robot.duckdb` (recordings, takes) is only read by the code at tag `data-v1` |
| **snapshot** | What the robot knows at one moment, sent when a command asks (after every move) and to a new server, in snapshot parts on the bulk lane: pose, camera picture and features, one compact ToF frame, colour, map window and changed tiles |
| **replay** | The fake robot sending a stored boot's messages to a test server again (`npm run fakerobot -- <boot>`) |
| **take** | (until 7 Nov) one action recorded with `recording on`, sent as UDP datagrams; gone with the recorder |

## The connection to the PC

| Term | Meaning |
|---|---|
| **message** | One unit on the robot's TCP connection: `[type 1 B][length 2 B][payload]`. Made and read only by **wire** (`wire.c` on the robot, `wire.ts` on the PC, the same table) |
| **wire** | The messages' bytes: the envelope and every payload but the ToF frame's (`tof_format` owns that). Its **wire version** is in hello; the PC refuses a robot with another |
| **hello** | The robot's first message on each connection: wire version, boot id, build |
| **status** | The robot's state, 4 times a second while connected: pose, mode, running move, WiFi signal. Was the "pose line" (which carried PicoB's jolt and tilt peaks until step 4c) |
| **move start / move end** | A move's messages on control: number, command, kind, parameter, time, still times; how it ended, frames sent and dropped, first and last frame number, how far it went, imu batches sent and dropped |
| **answer** | How a command ended: its number, the end word, the pose, and for a move how far it went, for a snapshot its number. Was the "done record" |
| **command** | A command message from the PC: its number and its text (`drive 50`) |
| **lane** | One of `link_pc`'s two queues on the one connection: **control** (hello, console lines, answers, status, map cleared, move start and end; small, sent first, never blocks) and **bulk** (tof frames and imu batches during moves, snapshot parts; a queue, sent only while control is empty) |
| **droppable** | A message that may go when its lane is full: console lines and status on control (to make room), tof frames and imu batches on bulk (counted). Answers, hello, move start and end and snapshot parts are never dropped |
| **snapshot part** | Up to 1400 bytes of a snapshot, one bulk message; a snapshot's parts in order |
| **golden bytes** | `picoA/app/test/data/wire_golden.hex`: one of each message, which both sides must encode and decode the same |

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
