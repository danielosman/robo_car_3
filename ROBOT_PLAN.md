# RoboCar — robot app plan

Status: **M0 ✅ (link, wheel control, odometry), M1 ✅ (calibration on waxed wood;
`b` and the carpet still to test), M2 ✅ (map, §14). Now: M3a (movement detection
while still: camera driver ✅, VL53 ✅, camera detector ✅ first robot tests, VL53
tracker ✅); M3b turning to look: first iteration ✅ on the robot, misses some (§6.6).** This plan covers
the real firmware (`picoA/app/`, `picoB/app/`, `common/`) of a **fully autonomous**
robot, built on the drivers verified in the bring-up (tag `pcb-bringup-v1`). It
describes the current design; what happened in each session is in
[CHANGELOG.md](CHANGELOG.md). Start a new session with "Where we are" below.

## Where we are (handover for the next session)

**Next session, in this order:**
1. Open from M1: the `b` test and the carpet tests (README, "M1"); on the carpet
   also `n` (floor learned, no false `?`).
2. M3a (movement detection while still, §6). Done and tested on the robot:
   - the camera driver (`picoA/drivers/camera.c`: its own exposure, 10 ms steps
     then gain, hold; row times for the rolling shutter), key `c`;
   - the VL53's movement detection (`change_grid`, `tof_motion`: closer than the
     background, §6.2 in six sentences; logged by `motion_sense`, keys `v` / `o`;
     README, "Movement, VL53").

   - the camera's movement detection (`camera_motion`, §6.3), with the exposure
     kept apart from it (adjusted only when calm); key `k`. On the robot: people
     and a light switch found as expected; it also sees shadows on the floor (the
     evening lamp). Movement log lines start with the robot's time in seconds.
     Known limit: a small thing moving faster than ~50°/s isn't found.

   **Tracking with the VL53 only (§6.6), decided with Daniel:** the tracker
   (`tracker.c`) works on the robot (walks at 0.5 and 1.5 m, both ways: one target
   edge to edge, the correct exit side). **Turning to look (M3b, first iteration,
   §6.6) works on the robot**: turns after people leaving both ways, within ~1° of
   the command, and on again when they left while it learned. **Next: the misses**
   (CHANGELOG, 7 Oct): sometimes no movement noticed at all, and often the target
   leaving while it learns is not noticed (learning starts only ~0.5 s after the
   stop; zones beyond the VL53's reach have no "farthest"). Look at a `v` log of
   misses first. Then back to open space and no turn into a near obstacle. Combining the camera with
   the VL53 (§6.5) and the camera during turns (§6.4) are put off: the camera keeps
   running and logging, but nothing uses it yet.

   Open, to try later: the VL53 at 10 Hz instead of 15 (`RANGEFINDER_HZ`; 100 ms per
   reading, ~20 % less noise; the map was tuned at 15). For pursuit the camera
   driver should prefer 10 ms exposures with more gain (40 ms blurs ~7 px at 60°/s;
   the living room in the evening needs 40 ms × 5.9).
3. M4 then adds: drive forward only while rows 7-8 see the floor right ahead
   (`map_floor_seen`; `:` or `?` ahead means stop), and face the farthest drivable
   corridor (§14).

**Files:**

| Where | What |
|---|---|
| `ROBOT_PLAN.md` | this plan: requirements, design, milestones, decisions (current state) |
| `README.md` | hardware, wiring differences from the PCB, how to build/flash, the map symbols, measured robot facts, robot test procedures (current state) |
| `CHANGELOG.md` | what happened session by session: robot runs, results, fixes |
| `REDFLAGS.md` | APOSD red-flag log per milestone, plus bugs found on the robot |
| `REFACTORING.md` | code review of M0+M1 (standards + spec), all applied |
| `ROBOT_WIFI.md` | PicoA's console over WiFi to `pc/robot/` (a browser page), and what PicoA does at power-up |
| `common/link.*`, `common/link_msgs.h` | inter-Pico link and its messages, timing constants, stop reasons |
| `picoB/app/` | `main.c` (the loop), `brain.*` (PicoA as PicoB sees it: messages, safety stops), `drive.*` (wheel control), `odometry.*` |
| `picoA/app/` | `main.c`, `body.*` (PicoB as PicoA sees it, incl. its clock), `pose.*` (pose at a given time), `rangefinder.*` (ToF rays, floor, drops), `world_map.*`, `surroundings.*` (frames → map), `behaviour.*` (start-up scan, watching and turning to look), `motion.h` (turn/drive profiles), `motion_sense.*` (movement while still: feeds the detectors, logs), `tof_motion.*` (VL53 movement), `camera_motion.*` (camera movement), `tracker.*` (one target in the VL53's movement), `change_grid.*` (last 4 frames per cell, blobs), `motion_obs.h`, `debug_console.*`, `robot_test.*` (square, drift, turns, straight) |
| `picoA/drivers/camera.*` | HM0360 camera: continuous capture, own exposure control, row times; shared with the bring-up firmware |
| `picoX/drivers/`, `picoX/bringup/` | drivers shared with the bring-up firmware; bring-up test firmware |
| `*/test/`, `run_tests.sh` | host tests with stubbed drivers |

**Build, test, flash:**
- `cmake -S . -B build -G Ninja` (first time), `cmake --build build`.
- `./run_tests.sh`: host tests (link; PicoB drive, odometry, brain; PicoA body,
  robot_test, rangefinder, world map, pose, start-up scan, movement detection); run after every change.
  It stops at the first failure.
- Flash `build/picoA/picoA_app.uf2` → PicoA, `build/picoB/picoB_app.uf2` → PicoB.
  Only the Pico whose code changed needs reflashing; a change to `common/link_msgs.h`
  means both. Keep the robot still ~1 s after PicoB starts (gyro bias).
- The console: a serial monitor on **PicoA's** USB, or without the cable the
  browser page of `pc/robot/` over WiFi (`npm start` there, ROBOT_WIFI.md); keys and
  the map symbols in the README. On USB at power-up PicoA waits (`n` scans, `w`
  joins the WiFi); without USB it joins the WiFi, then scans.

**Hardware facts** (beyond §3 and the README):
- IMU breakout is mounted **turned 180° about Z: X backward, Y right, Z up**
  (`to_robot_frame()` in `picoB/app/odometry.c`). At rest pitch/roll read ~±1°.
- Motor signs `LEFT_FORWARD −1`, `RIGHT_FORWARD +1` (`picoB/drivers/motor.c`); all
  four encoder signs checked, + = forward (`picoB/drivers/encoder.c`).
- Full power is ~0.26 m/s; the drive limits commands to 0.2 m/s and 1.5 rad/s and
  ramps at 0.4 m/s² / 3 rad/s².
- In-place turns make the body roll −3…−5° (leaning on its tyres): a possible
  signal for later (§12 Later), and slide it ~1.5 cm per turn while odometry sees
  ~0 (the wheels skid symmetrically). Gyro scale 0.42 % low; effective track width
  28.6 cm on waxed wood (geometric 22.5 cm).

**How we work** (agreed with Daniel):
- Plan first; no code until Daniel says go. Explain concepts plainly when asked.
- Daniel tests on the robot and pastes the serial log; say exactly which Pico to
  reflash, and what to look for.
- Every logic module gets a host test; stubs fail if a driver is used before its
  init.
- Red-flag review at the end of each milestone, logged in REDFLAGS.md; deviations
  from the PCB go in the README.
- README, this plan and ROBOT_WIFI.md show only the current state; history
  (session results, robot runs, fixes) goes only in CHANGELOG.md.
- Commit and push only when Daniel asks. **No `Co-Authored-By` lines** in commits
  (README convention).
- **Keep it simple** (learned in M3a): an algorithm should fit in a few plain
  sentences, written down before the code; Daniel judges it by the robot's log.
  Look at what the sensor really reports (its flags, `z`, `c`) before adding
  machinery. Design from what the robot needs now: earlier code (bring-up, old
  experiments) is not a reason to keep anything.

**Not yet implemented from §9:** CALIBRATE, GYRO_SCALE (M5), STATUS beyond the gyro
bias, the slip flag.

## 0. Overview

The robot is a curious observer. It sits still and watches. When something moves,
it turns toward it and drives up to it, stopping 50 cm away. When it hasn't looked
around for a while, it turns a full circle to refresh what it knows, and anything
that changed in the meantime gets investigated like movement. With nothing to do,
it turns to face the most open part of the room and watches from there. Its needs,
in order: understand its surroundings, follow movement without crashing, face
open space.

To do that it needs three things:

- **A short-term memory of its surroundings.** PicoA builds a 4 × 4 m map of 10 cm
  cells from the VL53L8CX. Each cell remembers whether something is there, whether
  the floor was seen, and when it was last measured. A cell changes only when it
  is measured again; old cells are a reason to look again, not to forget.
- **Knowing where it is.** PicoB tracks the robot's position and heading from the
  gyro (heading) and the wheel encoders (distance). PicoA corrects heading drift
  using the VL53: after each full turn it compares the start of the turn with the
  end, and it matches readings against the map.
- **Noticing movement.** PicoA compares each camera frame and ToF frame with a
  reference of what it saw: learned while standing still, and carried along while
  it turns to follow something. While driving, anything that contradicts the map
  counts as a change.

The work is split between the boards. **PicoA is the brain:** sensors, map, pose
correction, decisions. **PicoB is the body:** wheel speed
control, odometry, and a safety stop if PicoA goes quiet. They talk over the
on-board UART.

The code is written as deep modules, following *A Philosophy of Software Design*:
each module hides a body of knowledge behind a small interface. Red flags are
logged during coding and fixed in a review pass after every milestone. Work goes
in milestones, each one tested on the robot: link and driving, calibration, map,
movement detection, behaviour, full-turn scanning, then pose correction. Camera
help for the pose and a Kalman filter are kept as later steps.

Sections 1–2 restate your requirements and what I propose differently. The rest
is the design. §13 lists what's settled.

---

## 1. Requirements (your brain dump, cleaned up)

| # | Requirement |
|---|---|
| R1 | **Map.** PicoA keeps a map of the surroundings from the VL53L8CX: 10 × 10 × 10 cm cells, 4 × 4 m (the sensor's range is 4 m), 4 height layers. |
| R2 | **Cell memory.** A cell changes only when it is measured, never with time, and remembers when it was last measured. A reading that sees it occupied makes it occupied; it becomes empty only after **6 empty readings in a row** (a sighting in between starts the count again). Curiosity about old cells is the behaviour's job, using the age, not the map forgetting. |
| R3 | **Pose.** Estimate the robot's pose from the wheel encoders (all four) (wheels Ø 9 cm, 1 cm wide), the IMU and, later, the camera. Start without the camera and without a Kalman filter; keep both as future steps. |
| R4 | **Protocol.** Design the PicoA ↔ PicoB protocol. |
| R5 | **Attracted by movement.** Detect movement with the camera and the VL53 while the robot stands still, and keep detecting it while turning to follow it. Any movement means something in the surroundings moved. |
| R6 | **Direction of movement.** Know where the movement is and which way it goes (no assumption that it goes left or right), including whether it left the field of view to the left or the right. React only if it moves mainly horizontally; log the rest. |
| R14 | **States.** IDLE (console, tests), SCAN (390° at start-up), WATCH (follow movement, return to open space); start-up without USB goes SCAN → WATCH, console keys switch. |
| R7 | **Go to it.** Turn toward the movement and drive toward it until 50 cm away. |
| R8 | **Keep the map fresh.** When cells around the robot haven't been seen for a while (or never), rotate 360° to see them again. |
| R9 | **Changes are movement.** If a refresh shows that something changed, investigate it like movement. |
| R10 | **Code quality.** Deep modules per *A Philosophy of Software Design* (APOSD). Track its red flags and fix them in a review pass. |
| R11 | **Cheap maths.** Precision isn't critical; use dot products where possible. |
| R12 | **Face open space.** With nothing to scan or follow, turn toward the direction with the most free space and watch from there, instead of staying in front of a wall. |
| R13 | **Order of needs:** 1. understand the surroundings, 2. follow movement and don't crash, 3. face open space. |

## 2. Where I propose something different

| Your idea | Proposal | Why |
|---|---|---|
| The Pico is weak at floating point (R11) | Use **float32** freely, never `double`; still use unit vectors and dot/cross products instead of angles and trig | The Pico 2 (RP2350, Cortex-M33) has a **hardware single-precision FPU**; that limitation applied to the old RP2040. `double` is still slow, so compile with `-Wdouble-promotion` and write `1.0f`. Dot/cross products still pay off: "is it ahead?" is one dot product, "left or right?" is the sign of one cross product, with no `atan2` in loops. |
| One timer per cell (R2) | Keep your timer **and** add a "last observed" time per cell | With only the timer, "seen empty" and "never seen" look the same. R8 needs that difference: cells nobody has looked at recently are *unknown*, not empty. |
| Movement detection with the camera (R5) | Camera and ToF movement detection while **still**, and while **turning to follow** with the references carried along (§6.4); while driving, use **map contradictions** | While the robot drives, everything in the image moves. A turn in place only slides the image, which can be undone with the gyro heading. A map contradiction ("a hit where we recently saw free space") works while driving and during the 360° scan, and is exactly R9. |

---

## 3. Robot geometry

| | Value | Source |
|---|---|---|
| Size | **23.5 cm wide × 19 cm long × 10 cm high**, wheels included; front edge 9.5 cm ahead of the centre | you |
| Wheels | Ø 9 cm, 1 cm wide; 282.7 mm per revolution, 8 344 counts/rev → **29.5 counts/mm** | you, encoder bring-up |
| Track width | **≈ 22.5 cm** (23.5 − 1, wheel centre to wheel centre) | derived |
| Wheelbase | **≈ 10 cm** (19 − 9) | derived |
| Turning in place | outermost point sweeps **≈ 15 cm** radius | derived |
| Top speed | ~55 RPM at full power, lifted → **~0.26 m/s** | bring-up |
| VL53L8CX | **2.5 cm ahead of the centre, 3 cm right** of the centre line, **7 cm** above the floor; facing forward, no tilt; 45° × 45°, 8 × 8 zones of 5.6° | you, datasheet |
| Camera | **2.5 cm ahead of the centre**, on the centre line, **8.5 cm** above the floor; facing forward, no tilt; HFOV 53.1°, VFOV 41.1°, DFOV 64°, f 2.59 mm | you |
| Smallest obstacle | **2 cm** high. Bumps up to ~1 cm are driven over (Ø 9 cm wheels); the extra 1 cm is margin for noise and pitch | you, §4.2 |

A wheelbase much shorter than the track means the wheels scrub less in turns,
which helps skid steering.

**Distances to targets and obstacles are measured from the robot's front edge**
(9.5 cm ahead of the centre, 7 cm ahead of the sensors). "Stop 50 cm away" means
50 cm between the front edge and the target. Only `rangefinder` knows where the
sensors are; it reports points in the robot frame, and the front edge is a
robot-frame constant.

The sensors sit 7 cm behind the front edge. The VL53's lowest ray is 4.1 cm above
the floor where it passes the front edge, so the chassis in front of the sensor
must stay below that. The floor calibration (§4.2) will show it if it doesn't: a
zone reading a constant ~7 cm. The front wheels are outside both sensors' fields
of view (≥ 48° to the side).

**Camera bearing without trig:** 160 columns across 53.1° gives a focal length of
~160 px. Column `c` therefore looks along the vector `(160, 79.5 − c)` (forward,
left) in the robot frame; normalise it and use dot products. This assumes the
160 × 120 Sub4 mode covers the full field of view (to be checked).

---

## 4. Map (PicoA)

### 4.1 Grid

- Cells 10 × 10 × 10 cm. **40 × 40 cells (4 × 4 m), 4 height layers.**
- Aligned with the world (doesn't rotate with the robot). The window **re-centres in
  whole-cell steps** when the robot moves more than ~0.5 m from its centre. Cells
  that fall off the edge are forgotten. Circular indexing, so nothing is copied.
  Hits beyond the window edge (> 2 m in some directions) are simply not stored.
- Per cell: `observed_at` (u16 seconds of robot time), misses still needed to clear
  it, sightings since it was last free, and in layer 0 the no-floor count and
  "floor seen". 6 400 cells × 8 B = **~51 KB** of PicoA's 520 KB. A sweep every
  minute caps ages at ~8 h, so the u16 wrap (18 h) never matters.

### 4.2 Floor and layers: no false obstacles from the floor

The VL53 is 7 cm up and level. Its top four zone rows never see the floor. The
bottom four see the floor at these distances along the ray:

| Zone row (from the top) | Ray angle | Floor distance | Change for 1° of pitch |
|---|---|---|---|
| 5th | −2.8° | 143 cm | **51 cm** |
| 6th | −8.4° | 48 cm | 5.6 cm |
| 7th | −14.1° | 29 cm | 2.0 cm |
| 8th | −19.7° | 21 cm | 1.0 cm |

Deciding "floor or obstacle" by computing a hit's height and comparing it with a
layer boundary breaks exactly as you suspected: near the floor, 1 cm of range
error or 1° of pitch moves a hit across the boundary. Instead, **floor detection
works in range, per zone:**

1. **Learn each zone's floor distance** at every start, during the start-up turn
   (§14). This absorbs the cone shape, the sensor's distance convention and the
   floor's reflectivity.
2. **Adjust for pitch** using PicoB's pitch (the robot nods when accelerating and
   braking).
3. A reading is an **obstacle only if it is shorter than the floor by more than a
   2 cm-tall object would cause.** A ray going down at angle θ hits a 2 cm-tall
   object 2 cm / sin θ earlier than the floor, so each row gets its own margin:

   | Zone row | Floor distance | 1 cm bump shortens it by | **2 cm obstacle shortens it by (= margin)** | Change for 1° of pitch |
   |---|---|---|---|---|
   | 8th | 21 cm | 3 cm | **6 cm** | 1.0 cm |
   | 7th | 29 cm | 4 cm | **8 cm** | 2.0 cm |
   | 6th | 48 cm | 7 cm | **14 cm** | 5.7 cm |

   Anything shorter by less than the margin is floor: every cell along the ray is
   free, and a 1 cm bump disappears. VL53 noise (~1 cm) and the remaining pitch
   error after IMU compensation stay inside the margin.
4. The 5th row (floor at 143 cm, 51 cm per degree of pitch) can't do this. Its ray
   is still 2 cm above the floor at ~100 cm, so **any hit closer than ~95 cm is an
   obstacle at least 2 cm tall**. Farther hits on that row aren't used for
   obstacles.
5. **Drops:** rows 7-8 must see their floor; no return, or one beyond their floor
   patch, marks "no floor" (`?`, §14).

The result: small floor bumps and cables lower than ~2 cm are invisible to the
map, which is what we want, since the wheels drive over them.

**Layers are aligned to the robot's 10 cm height**, so the important seam isn't
near any noise:

| Layer | Height | Meaning |
|---|---|---|
| (floor) | 0–2 cm | ignored: floor, bumps, cables |
| 0 | 2–12 cm | **blocks the robot** (10 cm high + 2 cm margin) |
| 1 | 12–22 cm | overhang the robot fits under (e.g. a low shelf) |
| 2 | 22–32 cm | overhang |
| 3 | 32–42 cm | overhang |

**Layer seams don't need to be sharp.** A height error of a few cm only moves a
hit to the neighbouring layer. Driving only checks layer 0, so the one seam that
matters is 12 cm: an overhang at 12–14 cm could be misjudged. A robot that refuses
to drive under a shelf with only 2–4 cm of clearance is the safe direction. The
floor/obstacle decision is made in range, per zone, by step 3, not with a layer
boundary.

### 4.3 Cell states

| State | Condition |
|---|---|
| UNKNOWN | never observed (blank on the map) |
| OCCUPIED | seen occupied, and not yet seen through 6 times in a row since |
| NO_FLOOR | layer 0: a floor ray (rows 7-8) expected the floor here and got no return, or one more than 15 % beyond the far end of the zone's floor patch (a drop? a dark floor?), 3 times with the floor not seen here in between; stays until the floor is seen here (`?` on the map) |
| FREE | observed, not occupied |

Nothing changes with time: a cell once seen stays as last measured; `observed_at`
keeps how old that is (for the refresh, R8). Layer 0 also keeps whether the floor
was seen (`.` vs `:` on the print). Exploring means: fill the blanks, avoid `?`,
look again at old cells.

### 4.4 Update rule (your R2)

Applied for **every ToF frame** (10–15 Hz):

```
hit  (a range ended in this cell):   misses_left = 6; sightings += 1
miss (a range passed through it):    if misses_left > 0: misses_left -= 1   (FREE at 0)
both:                                observed_at = now
```

- A removed obstacle clears after 6 frames that see through it (~0.4 s at 15 Hz);
  a noisy miss or five can't clear it.
- **Within one frame, a cell gets at most one update; if any ray ends in it, it's
  a hit.** Near the sensor many of the 64 rays pass through the same few cells, and
  one frame shouldn't count as 20 misses. This is the only limit; there is no
  waiting or averaging across frames.
- A cell that isn't seen again stays as it was; only its age grows.

### 4.5 Adding a ToF frame

For each of the 64 zones, rotate its precomputed direction vector by the robot's
pose. Walk along the ray in half-cell steps: cells before the measured distance get
a miss, and the cell at the distance gets a hit (or a miss, if §4.2 says it's
floor). **A ray that reaches the floor clears layer 0 all the way to it**, also
where it is under 2 cm: anything standing on the floor 2 cm or taller would have
blocked it. **Zones with no target mark the first 1 m of the ray as misses and leave farther cells untouched**: "saw nothing" is only trusted close up. Cost: 64 rays ×
≤ 40 steps ≈ 2 500 cell updates per frame, which is trivial.

### 4.6 Changes (R9)

While adding a frame, the map reports a **change** when:
- a **hit** lands in a FREE cell, or
- a **miss** clears a cell that was seen occupied in 2 or more frames.

Changes come with their world position. This works while driving and while
scanning.

### 4.7 Questions the map answers

- **Staleness (R8):** "how much of the ring 0.3–1.5 m around the robot is UNKNOWN or old,
  per 45° sector?" The behaviour uses that to decide when to scan and which way to
  turn first.
- **Free distance (R12):** "from this point, in this direction, how far until an
  obstacle (layer 0) or an UNKNOWN cell?" The behaviour uses it to find open space
  and to check a path before driving it.

---

## 5. Pose estimation

**Frames:** world (the map), robot (x forward, y left, z up, origin at the centre
between the wheels), sensor (each sensor's own; only its module knows how it's
mounted). The pose is x, y in metres, plus heading as a unit vector (cos, sin),
with yaw in degrees kept for display.

### 5.1 Heading: the IMU gyro (PicoB) is the main source

The ISM330DHCX is a 6-axis IMU: a 3-axis **accelerometer** (linear acceleration,
which also shows which way gravity points; that's where the tilt comes from) and a
3-axis **gyroscope**. The gyroscope doesn't give a heading. It measures **how fast
the robot is turning** about each axis, in degrees per second. Adding that rate up
over time (integrating it) gives **how far the robot has turned since we started
counting**: turning at 30°/s for 13 s adds up to 390°. That's a relative angle;
with no compass (magnetometer) it has no idea where north is, and it doesn't need
to. The bring-up firmware already does this for tilt (it integrates gx and gy);
heading is the same thing with gz, the axis pointing up.

The gyro is the primary heading sensor. Integrate gyro z at 416 Hz. Its bias is
calibrated at power-up and **re-estimated every time the robot stands still**
(encoders not moving and the gyro reading under 3°/s, for ≥ 0.5 s; the gyro check
also catches the robot being turned by hand, without odometry needing to know the
motor commands). Two error sources
remain:
- **Bias drift:** small, and re-measured at every stop.
- **Scale error:** the sensor's sensitivity is off by up to a few %, so turning a
  real 360° may add up to e.g. 352° or 367°. This is calibrated by the full-turn
  check (§5.4), and the correction is kept.

### 5.2 Distance: encoders (PicoB)

The average of the left and right side, each side the average of its front and
rear encoder, applied along the current heading.

### 5.3 Rotation from the encoders: secondary

While turning in place, rotation = (right wheel distance − left wheel distance) /
track width. With the geometric track width of 22.5 cm, a full turn is π × 22.5 ≈
**70.7 cm per wheel (~20 900 counts)**. The counts are precise, but **skid
steering** makes them inaccurate: in a turn the wheels slide sideways, so the
wheels travel further than the geometry says. The *effective* track width is
larger than 22.5 cm and depends on the floor (our short wheelbase helps). M1
measures it, and the full-turn check keeps it updated.

**Your floors:** mostly smooth waxed wood, with a carpet in one place. Wheels slide
easily on waxed wood and grip on carpet, so the effective track width differs
between the two, and the robot can be on both at once. That's another reason the
gyro, not the encoders, sets the heading: the gyro doesn't care about the floor.
The carpet's edge is lower than 2 cm, so the map ignores it (§4.2).

So the encoders are the second opinion on rotation. If the gyro and the encoders
disagree a lot, it flags **slip or a stall** (a wheel spinning or a wheel blocked).
They'd also be the fallback if the gyro ever failed.

### 5.4 Knowing when a full turn is done, and correcting with it

1. **During the turn:** PicoB integrates the gyro's turn rate (with the stored scale
   correction). The scan keeps turning until the angle turned since the start adds
   up to **390°**: a full turn plus 30° of overlap. The encoders show the turn is
   really happening, not slipping.
2. **Build a range profile:** while turning, PicoA takes each ToF frame's middle
   rows (above the floor) and drops each column's distance into **2° bearing bins**
   around the robot, using the gyro heading at the moment of the frame. Hit points
   are converted to bearings from the robot's centre, so the sensor's 3 cm offset
   doesn't skew them. At 30°/s and 15 frames/s a new frame arrives every 2°, so the
   8 columns fill the bins densely.
3. **Compare start and end:** the first 30° of the turn and the last 30° look at
   the same part of the room. Shift one against the other by −10° … +10° and pick
   the shift where the distances agree best (sum of absolute differences: integer,
   cheap). That shift is the gyro's error over this turn.
4. **Accept it only if it's clear:** enough bins with valid ranges, and the best
   shift clearly better than its neighbours. A plain wall or a round room gives no
   clear answer, so the turn is ignored. Something moving in the overlap is caught
   the same way, and is also a map change.
5. **Use it:** correct the heading by the shift and nudge the stored gyro scale
   correction toward it (a fraction per turn, so one bad turn can't spoil it).
   Update the effective track width from the encoder counts over the now-known
   angle.

The VL53's range noise doesn't stop this from working: the matching uses the
*shape* of the profile (where distances jump at door frames, furniture legs,
corners), and a 2° bin is finer than a 5.6° zone because consecutive frames
overlap. Expected accuracy is ~1–2° per turn in a furnished room. M5 will measure
it.

### 5.5 Correction against the map (PicoA, later in M6)

About once a second, when the map around the robot is well known: try small pose
offsets (±10 cm, ±5°) and score each by how many ToF hits land in OCCUPIED cells.
If one is clearly better, apply part of it (e.g. 20 %), so corrections are smooth.
PicoA keeps corrections as a map ← odometry offset; **PicoB's odometry is never
reset**.

### 5.6 Tilt

Pitch and roll come from the existing complementary filter on PicoB. PicoA needs
pitch for floor detection (§4.2).

### 5.7 Why no Kalman filter yet

- The sources measure different things: the gyro gives turn rate, the encoders
  give distance, and the VL53 gives position relative to the room. Each has one
  job, plus cross-checks.
- An EKF needs a noise model for each source, and we have no measurements yet.
  M1–M5 will produce logs.
- **The pose interface (`pose_at(t)`) stays the same** if the inside is later
  replaced by an EKF. That's §12, decided from those logs.

---

## 6. Movement detection (PicoA)

Two detectors, the camera and the VL53, feed one module, `motion_sense`. It works
in two modes:

- **Still:** the robot stands still (PicoB stationary, then 0.5 s for vibration to
  settle). The most sensitive mode; waiting for movement happens here.
- **Pursuit:** the robot turns in place to follow a target (§7.2). The references
  are carried along with the turn (§6.4), and detection looks only near where the
  target is expected.

While driving (M4 on), neither detector runs; map changes (§4.6) take over.

### 6.1 Movement against a background

Comparing each frame with the previous one only sees the edges of a moving thing and
misses slow movement, so each frame is compared with a **background** learned when
the robot stops. A cell (a VL53 zone, a camera block) has **moved** when it differs
from its background in 2 of the last 4 frames; moved cells that touch form a blob,
with a **where** vector (unit, from the sensor, robot frame), a range if the sensor
has one, and a point. `change_grid` keeps the last 4 frames and the blobs; what
"differs" means, and when something has stopped (§6.2: steady for 1 s), is each
sensor's own: the two work in entirely different ways (§6.2, §6.3).
Which way a target moves needs the same target in two frames, so the tracker (§6.5)
computes it.

### 6.2 VL53: closer than the background

In simple sentences (`tof_motion.c`):
1. Each zone uses the frame's nearest **sure** target, as the map does, or nothing.
   Unsure readings tell nothing: on the robot most of them are a faint echo of the
   floor (the strongest return by far) that every zone picks up, ~1 % of its signal
   at the floor's distance; the sensor flags them so they can be dropped. In the
   floor rows, a reading beyond the zone's patch of floor (or nothing) tells nothing
   either: on the waxed floor the grazing 6th row reads the wall behind half the
   time (`rangefinder_floor_limit_m`, the map's own limit).
2. When the robot stops, each zone learns its **background** over 1 s: the nearest
   distance it gives regularly (its 2nd-nearest sure reading, so one odd near frame
   doesn't count); nothing only if it had no sure reading. A zone switching between
   a near and a far surface has the near one: the far one is just farther.
3. A zone has **moved** when it reads clearly **closer** than its background (8 cm,
   or 8 % if more) in 2 of the last 4 frames next to another such zone, or 3 alone
   (a person or a hand always spans several zones). Something that enters the view
   is always in front of what was there.
4. A zone that reads nearer briefly without moving, then goes back, shows a nearer
   surface now and then (a door frame's edge): that is its background from then on.
5. Farther readings (something taken away, a reflection) are never movement; 10 s
   farther in a row is the new background (not sooner, or a surface shown only now
   and then would be forgotten). At the background, it follows slowly.
6. A zone that reads closer but steady (within the same 8 cm / 8 %) for 1 s has
   stopped moving (a box put down): that distance is its background. Movement is
   change; a waving hand or a walking person keeps changing and keeps counting.

Times are in seconds (`RANGEFINDER_HZ`, 15 Hz; 10 Hz would measure 100 ms instead of
66 ms per frame, ~20 % less range noise: to try once 15 Hz works).

Host test (`test_tof_motion.c`, 20 random seeds): 5 minutes of a room with range
noise, unsure frames, a floor zone's reflection and far zones' farther surfaces or
nothing for 1-2 frames every ~10 s, a far zone sure half the time, a floor zone
reading the wall behind half the time, a door frame's edge 1 frame in 20: no
movement; someone walking up until they fill the view: movement, not light; a
person found in 2 frames, a thin pole in one zone in 3, a hand waving
for 3 s counted the whole time, a box still after ~1 s, taking it away not
movement.

### 6.3 Camera: different from the background

In simple sentences (`camera_motion.c`):
1. Each 8 × 8-pixel block of the 160 × 120 image is a cell (20 × 15); its value is
   its mean brightness. Frames still settling after an exposure change are skipped,
   and frames are used at most ~15 a second (times are in seconds, not frames).
2. When the robot stops, the exposure is let reach its target, then held, and each
   block learns over 1 s what it normally reads (the middle value) and how much it
   wobbles (its noise: the mean distance from the middle, at least 2 of 255). A
   block at its background follows it slowly (dusk).
3. A block **differs** when it is off its background by more than 4 × its noise.
4. A block has **moved** when it differs in 2 of the last 4 frames next to another
   such block, or in 3 alone (`change_grid`, as on the VL53).
5. A block that changed and keeps its new value (within the same 4 × noise) for 1 s
   has stopped (a box put down, the place it was taken from, a light switched),
   also if it dips under "differs" now and then: that is its background from then on.
6. **Exposure and movement are kept apart.** The driver sets the exposure slowly
   (camera driver below). While the robot stands still the exposure is held; when
   the driver wants a change, it gets it only when calm: nothing moved for 5 s (or
   after 30 s of wanting, for something that never stops moving). Then the camera
   stops watching, the exposure is adjusted, the view learned again (~1-3 s blind;
   the VL53 keeps watching). A light switched is movement, then the exposure follows.
7. Where: a blob's blocks give the direction, from column and row along
   `(160, 79.5 − x, 59.5 − y)` (§3), normalised; its time is its middle row's
   (`camera_row_time`). No range: that comes from the VL53 or the map (§6.5).

Unlike the VL53 the camera can't tell nearer from farther: taking something away is
movement too, until it is steady; a moving shadow or a reflection on the waxed
floor is movement as well.

- **The driver sets the exposure itself, always** (the sensor's auto-exposure is
  off): whole 10 ms steps, since lights flicker at 100 Hz on 50 Hz mains and the
  rolling shutter would turn flicker into moving bands; the shortest step needing at
  most ×4 gain, then more gain; under 10 ms only in daylight. It adapts slowly, like
  an auto-exposure: off target by > 25 %, it waits (5 s when a little off, down to
  0.5 s when far off: a passing shadow is gone by then), then steps exposure × gain
  by at most ×1.25 until on target. Held, frames say it wants a change.
- **Rolling shutter:** each row is exposed one line period after the row above
  (datasheet V01, p. 2); turn compensation (§6.4) takes the heading per block row.

Host test (`test_camera_motion.c`, 11 seeds, at 25 and 100 frames/s): a textured
room with pixel noise and a wobble of the whole image, a driver that waits and
steps: no movement in 5 min, nor at dusk; darker still: the exposure adjusted when
calm, no movement; the light switched off or on: movement, still after ~1.4 s, a
new exposure ~8 s after the switch; someone walking up until they fill the view:
movement all the way, the exposure adjusted only once they stand still; a hand that
never stops: adjusted after 30 s anyway; a person found in 2 frames, a small object
in one block in 3, a hand waving at ~33°/s the whole time, a box still after ~1.2 s,
and its place after it is taken away.

### 6.4 During pursuit

- **Camera: shift the reference by the turn.** The camera is 2.5 cm from the turning
  centre, so a turn in place is almost a pure rotation, which slides the whole image
  sideways by ~3 px per degree at every depth. Each frame takes the heading turned
  since the reference from `pose_at(frame time)`, shifts the reference by that many
  pixels and compares only the columns both see. Motion blur at 60°/s with 10 ms
  exposure is ~2 px, below a block. New columns at the leading edge get a reference
  after a few frames.
- **ToF: the map is the reference.** In the robot frame a turn changes every zone; in
  the world frame the room stays put. A hit in a cell the map recently saw free,
  near the predicted target position, is the target (§4.6's map changes, which are
  too noisy alone but usable when filtered by the prediction).
- Detection searches only within ~15° of the predicted bearing.

### 6.5 Combining, tracking, events (later: the full design)

- The camera sees ±26.5°, the ToF ±22.5°. If their where vectors agree (dot >
  cos 10°; the ToF's 3 cm sideways offset shifts it ≤ 3° at 50 cm), it's one
  observation with range. Seen by only one sensor, a cell needs a higher score (one
  more frame).
- The tracker follows one target (the strongest blob; once tracked, it keeps it over a
  stronger newcomer unless it is lost), with its angular speed.
- **React or log:** project which way onto the horizontal (drop its vertical part); a
  length **> 0.5** (it moves mainly sideways, or toward or away from the robot) is a
  `MOTION` event; ≤ 0.5 (mainly up and down: a hand waving, someone sitting down)
  is only logged.
- When the target disappears: last seen in the outer ~15 % of the camera's field of
  view → `EXITED(side, last bearing)` (side: the sign of cross(forward, where));
  otherwise `STOPPED`.

**Events produced:** `MOTION(where, which way, range or none, angular speed)`,
`EXITED(side, last bearing)`, `STOPPED`, `MAP_CHANGE(world point)`.


### 6.6 Tracking with the VL53 only, then turning to look

Why the VL53 alone: it only sees real things (no shadows, lamps, light switches),
gives a range, and its blobs are few (8 × 8 zones). The camera's finer direction
and wider view (±26.5° against ±22.5°) can be added later (§6.5).

**Tracker** (`tracker.c`, log only, the robot stays still):
1. Each VL53 frame's blobs (`tof_motion`) are the candidates.
2. The tracker follows one target: the biggest blob at first; afterwards the
   biggest blob that reaches within 10° of where the target is expected (blobs
   report their leftmost and rightmost cells: someone close is a big blob plus
   pieces, and its centre says little), and is not more than 0.5 m nearer or
   farther (a far zone in the same direction is something else).
3. It keeps the target's direction, range and angular speed (a straight line
   through its directions of the last ~0.8 s: two directions alone were too noisy).
4. No blob near it for 0.5 s: lost. Last seen in an outer zone column (the outer
   ~15 % of the view) → "exited left / right"; else "stopped".
5. Log: `Target: +12 deg, 0.6 m, going right at 20 deg/s` twice a second, then
   "left the view on the right at −17 deg" or "stopped at …".

Host test (`test_tracker.c`, 20 seeds, ±2° jitter, 1 frame in 8 missing): walking
at 20 and 40°/s, the speed within 5 / 8°/s and the exit side right; stopping
inside: stopped; the target kept over a bigger newcomer and through a 0.47 s gap;
from the robot's logs: someone close (a big blob from −15° to +20° and a 2-zone
piece at −17°) followed by the big blob, a far zone at the edge not taken.

**Turning to look** (M3b, first iteration; `behaviour.c`, the WATCH state):
1. The robot watches standing still, motors on (after the start-up scan, or `a`).
2. The tracker reports the target **leaving** (once per target): seen in an outer
   zone column (beyond ±16°), moving outward at ≥ 10°/s, its angular speed measured
   over at least 3 frames (≥ 0.15 s).
3. The robot turns to where the target will be when the turn ends, if it keeps its
   angular speed (the turn takes its angle at 1 rad/s plus 0.5 s), at most 90°.
4. It stops; once it stands still the VL53 learns the view (1 s), as after every stop.
5. While learning, the VL53 notices something **passing out** of the view: in an
   outer column, 2 zones in one frame clearly nearer (8 cm / 8 %) than the farthest
   the zone gives regularly (its 3rd-farthest sure distance: gone for more than 2
   frames). If that is on the side the target was going, it left again: from where
   and when it was last seen there, at the same angular speed, back to 3. Otherwise
   it watches (1).

On the robot: turns within ~1° of the command; it misses some movement, and often
the target leaving while it learns (CHANGELOG, 7 Oct). A target that stops inside
the view, or leaves too slowly or too fast to be seen at the edge, gets no turn. Moving straight away is not movement for the VL53 (only
the camera logs it). Not yet: back to open space after ~20 s, no turn into a near
obstacle (§7.2), smooth following while turning (§6.4) only if the hops show it is
needed.

Host tests: `test_tracker.c` (leaving once, at the edge, before the exit; not
when coming in at the edge), `test_tof_motion.c` (the quiet room learned 300 times:
nothing passes; passing out left / right noticed with its time; staying at the
edge, gone one frame, passing in the middle: not), `test_behaviour.c` (after the
scan: watching; turns of −71°, −85° when it leaves again while learning, +43°, and
at most 90° for 120°/s; leaving seen while learning the other way, or not after a
turn: no turn).

---

## 7. Behaviour (PicoA)

### 7.1 Robot states

| State | What the robot does | Entered by |
|---|---|---|
| **IDLE** | stands still, motors off, takes console keys; the tests (`q`, `r`, `f`, `b`, `d`) run only from here | power-up on USB, `i`, `s`, a safety stop |
| **SCAN** | turns 390°, builds the map, turns to the most open direction (§14) | power-up without USB after trying the WiFi, `n` |
| **WATCH** | watches for movement, turns after a target leaving the view (§6.6); later: returns to open space when nothing moves | the end of SCAN, `a` |

Power-up without USB: WiFi, SCAN, WATCH. On USB: IDLE; `n` (scan, then watch) or
`a` starts it. The motors stay on while watching. A safety stop drops to IDLE, so nothing restarts the motors by
itself. Every change of state is printed.

### 7.2 Following movement by turning (later; M3b's first iteration is §6.6)

The full design, for when the hops of §6.6 are not enough. In WATCH, with no driving yet:

1. **Still:** detect (§6). A `MOTION` event → Pursuit. (Within ±10° of straight ahead
   and not moving sideways: no turn, keep watching.)
2. **Pursuit:** turn at the target's angular speed to keep it centred, aiming ahead
   by angular speed × ~0.3 s, at up to 1.5 rad/s (a person at 1 m/s crosses at
   ~1 rad/s 1 m away). No limit on how far: it follows for as long as the movement
   goes on. Detection continues during the turn (§6.4).
   - `EXITED` (lost at an edge): keep turning that way along the prediction for ~1 s,
     or ~40° past the edge of the field of view.
   - `STOPPED` or lost: stop, settle 0.5 s, learn the references, back to Still.
3. **Back to open space:** ~20 s in Still with no movement → turn to the most open
   direction (§14's chooser), only if it is ≥ 1.5 × better than the current heading.
4. **Turning safety:** no turn while the map has an obstacle within the 15 cm turning
   radius (the turning part of the Safety guard below).

### 7.3 The order of needs (M4, M5)

The robot's needs, in order:

1. **Understand its surroundings** (R8, R9)
2. **Follow movement, and don't crash** (R5–R7)
3. **Face open space** (R12)

Runs at 20 Hz. The highest-priority need that wants something chooses the drive
command (v, ω) for PicoB. **Not crashing isn't a competing need but a guard on every
command:** whatever the behaviour asks for passes through Safety before it goes to
PicoB.

| Priority | Behaviour | When | What it does |
|---|---|---|---|
| 1 | **Scan** (R8) | too much of the surroundings unknown or old (e.g. > 30 % of near cells, or a rear sector), or right after power-up | Stop, then turn 390° at ~30°/s (~13 s) with the full-turn check (§5.4). Map changes found during it become Investigate events |
| 2 | **Investigate** (R7, R9) | an event < 30 s old | Turn toward the bearing (the sign of the cross product picks the direction; dot > cos 5° counts as aligned), then drive toward the target. **Stop when the front edge is 50 cm from it** (ToF range on that bearing). For EXITED: turn ~40° past the edge of the field of view on that side, then watch. A new event re-targets. |
| 3 | **Face open space** (R12) | nothing to scan or investigate | Turn toward the direction with the most free space ahead, then stand still and watch for movement (R5). Movement detection runs whenever the robot is still, whatever the behaviour |

| Guard | Condition | Effect on the command |
|---|---|---|
| **Safety** | an obstacle (layer 0) < 20 cm ahead of the front edge in the robot's path (a corridor 23.5 cm wide plus ~3 cm margin each side) | no forward motion; turning still allowed |
| | an obstacle in the map inside the 15 cm turning radius | no turning toward it |
| | link lost, IMU error | stop (PicoB also stops on its own after 250 ms without a command) |

**Consequences of this order:**
- Movement seen during a scan isn't chased straight away. It's queued as an event
  and investigated right after the scan (events stay valid for 30 s). A scan takes
  ~13 s.
- During an approach the robot looks forward, so cells behind it only go stale
  after minutes. An approach takes seconds, so a scan will rarely interrupt one.
- After reaching a target (often near a wall or furniture), the robot doesn't stay
  facing it. With nothing else to do, it turns to face open space and watches from
  there.

**Choosing open space:** for candidate headings every 10°, the map gives the
average free distance across the camera's field of view (±25°): how far the robot
could look before hitting an obstacle (layer 0) or unknown cells. The robot turns
to the best heading only if it's clearly better than where it's facing now (e.g.
1.5× the free distance), so it doesn't fidget.

Speed limits: **0.15 m/s** driving, **~0.5 rad/s** (≈ 30°/s) turning. These leave
headroom below the motors' maximum for closed-loop control.

v1 drives in a straight line to the target. If Safety stops the robot before it's
50 cm away, it counts as arrived. Path planning around obstacles (A* on the 2D
grid) is in §12.

---

## 8. What each Pico does

```
              PicoA "brain"                                PicoB "body"
  ┌──────────────────────────────────────┐          ┌──────────────────────────┐
  │ camera ─┐                            │  DRIVE   │ drive: wheel speed       │
  │ ToF ────┼─► motion_sense ─┐          │ ───────► │   control, safety stop   │
  │         └─► world_map ────┼─► behaviour         │                          │
  │ pose ◄── ODOM ◄───────────┼──────────│ ◄─────── │ odometry: gyro heading,  │
  │   (full-turn + map correction)       │   ODOM   │   encoders, tilt         │
  │ debug log ─► USB serial, WiFi        │          │                          │
  └──────────────────────────────────────┘          └──────────────────────────┘
```

- **PicoA (brain):** camera, ToF, map, movement detection, pose
  correction, behaviour, and a debug log on its USB serial and over WiFi.
- **PicoB (body):** wheel speed control, odometry, tilt, and its own safety stop.
- PicoB never decides where to go; PicoA never touches the motors.

---

## 9. PicoA ↔ PicoB protocol

**Physical:** UART0, **GP0 TX / GP1 RX on both Picos** (crossed on the PCB).
**1 Mbaud** 8N1 (short on-board traces; drop to 460 800 if errors show up). No
flow control.

**Framing:** each message is `[type u8][seq u8][body][CRC-16/CCITT]`, COBS-encoded
and ended by a `0x00` byte. A bad CRC means the frame is dropped and an error
counter goes up. Any `0x00` restarts the receiver, so a lost byte only costs one
message.

**Body:** packed little-endian structs, float32 in SI units (m, rad, s), defined
**once** in `common/link_msgs.h`.

| Message | Direction | Rate | Contents | Notes |
|---|---|---|---|---|
| HELLO ✅ | both | at boot, 1 Hz until answered | protocol version, is_reply | Version mismatch: PicoB keeps the motors off |
| DRIVE ✅ | A → B | 20 Hz | v (m/s), ω (rad/s) | **PicoB stops if no DRIVE for 250 ms** |
| MOTORS ✅ | A → B | on change, re-sent until PicoB has acted on it | on / off, request number | On clears a safety stop. PicoB echoes the last request it acted on in ODOM and ignores repeats, so a repeat can't undo a safety stop (M1) |
| ODOM ✅ | B → A | 50 Hz | t_B (µs, u32), x, y, yaw (not wrapped, so a full turn reads +2π), v, ω, pitch, roll, wheel distances per side, stationary, motors on, stop reason, IMU error | Stop reasons: no DRIVE, left/right wheels not following (jam, encoder), tilt > 15°. After a safety stop the motors stay off until PicoA switches them on |
| (clock sync) | | | | **No message needed:** ODOM already carries t_B; PicoA notes each report's arrival on its own clock, and the smallest difference over the last 4 s (minus the frame's 0.5 ms on the wire) is the offset. Within ~0.15 ms in the host test; follows drift; resets when PicoB restarts (`body.c`) |
| CALIBRATE | A → B | on request | — | Hold still: gyro bias calibration |
| GYRO_SCALE | A → B | after a full-turn check | scale correction | Stored by PicoB and applied to the gyro from then on |
| STATUS ✅ | B → A | 2 Hz | gyro bias (M1); later wheel speeds, PWM, gyro scale, error counters | |
| LOG ✅ | B → A | rare | text, ≤ 64 chars | PicoA prints it in its own debug log, so one serial monitor shows both boards |

✅ = implemented (protocol version 4). The timing constants and stop reasons live in
`common/link_msgs.h`.

Bandwidth: ODOM ≈ 45 B × 50 Hz ≈ 2.3 kB/s; everything together is under 5 % of the
link.

**No PC app in v1:** the robot is fully autonomous. During development, PicoA's
console (USB serial, or over WiFi to the browser page of `pc/robot/`, ROBOT_WIFI.md)
prints a readable debug log (pose, events, behaviour changes, link
errors, PicoB's LOG lines) and accepts a few single-key commands in a serial
monitor, like the bring-up firmware (`s` stop, `g` go, `m` print the map near the
robot as text). A PC link with the same framing comes later (§12).

---

## 10. Modules

Each module hides a body of knowledge behind a small interface. Sketches only;
the names are open to change.

**common/**

| Module | Interface | Hides |
|---|---|---|
| `link` | `link_init()`, `link_send(type, body, len)`, `link_receive(&msg)` | UART, IRQ/DMA, COBS, CRC, sequence numbers, error counters |
| `vec2` (helper header) | `dot`, `cross`, `rotate`, `normalize` | — (shared maths, not a module boundary) |

**PicoB**

| Module | Interface | Hides |
|---|---|---|
| `drive` | `drive_set(v, ω)`, `drive_stop()`, `drive_update()` | side → driver mapping, forward signs, per-side wheel speed PI, effective track width, ramping, PWM, both motors on a side sharing direction and power, the per-wheel stop |
| `odometry` | `odom_update()`, `odom_get(&odom)`, `odom_set_gyro_scale(s)` | gyro integration, bias and scale correction, stationary detection, encoder scaling, slip flag, tilt filter. Also provides the measured wheel speeds `drive` needs, so the wheel diameter lives in one place |
| `brain` (M1) | `brain_init(imu_ok)`, `brain_update()`, `brain_log(fmt, …)` | PicoA as PicoB sees it: the link protocol from PicoB's side, greeting and version lock, applying MOTORS and DRIVE, the safety stops (no DRIVE, `drive` faults) and reporting each once, report rates |

**PicoA**

| Module | Interface | Hides |
|---|---|---|
| `body` | `body_update()`, `body_connected()`, `body_odom()`, `body_status()`, `body_motors(on)`, `body_drive(v, ω)` | PicoB as PicoA sees it: the link protocol, greeting and version check, repeating DRIVE for PicoB's safety stop, re-sending MOTORS until PicoB has acted on it, printing PicoB's log lines |
| `robot_test` (M0, M1) | `robot_test_start(test)`, `robot_test_stop()`, `robot_test_update()`, `robot_test_result()` | the calibration tests as a table (steps, progress, result), driving to distances and angles, early stops and their results |
| `camera` (driver, `picoA/drivers/camera.c`) ✅ | `camera_init()`, `camera_update()`, `camera_frame(&frame)` (non-blocking: the newest complete frame, with its exposure, gain, a settling flag and its row times, `camera_row_time()`), `camera_hold_exposure(bool)`, `camera_exposure(&e)` | PIO/DMA capture into three buffers, HM0360 registers, its own exposure control (10 ms steps against flicker, then gain; the sensor's auto-exposure off), the frame length following the exposure, the measured line period, the rolling shutter |
| `rangefinder` (on top of the `tof` driver) | `rangefinder_poll(&scan)` → 64 rays **in the robot frame** (origin, unit direction, distance, and floor / obstacle / no target) plus a timestamp | zone order, the 90° rotation, the 3 cm offset and 7 cm height, per-zone floor calibration and pitch adjustment, VL53 status codes and settings |
| `world_map` | `map_add_scan(scan, pose, changes)`, `map_cell_state(point)`, `map_staleness(pose, sectors)`, `map_free_distance(point, direction)` | cell size, layers, rolling window, timers, ray walking, change detection |
| `pose` | `pose_on_odom(odom)`, `pose_on_scan(scan)`, `pose_at(t)` | odometry history and interpolation, clock offset, full-turn check, map ← odometry correction, map matching |
| `change_grid` ✅ | `change_grid_init()`, `change_grid_add(scores)`, `change_grid_moved()`, `change_grid_blobs()` | the last 4 frames per cell, the thresholds, blob labelling |
| `tof_motion` ✅ | `tof_motion_restart()`, `tof_motion_add(frame, obs)` | the zone backgrounds, "closer than the background" (§6.2), blobs to observations |
| `tracker` | `tracker_reset()`, `tracker_add(obs, n, t)` → new / exited left / right / stopped, `tracker_target()` | which blob is the target, its angular speed, when it is lost and where (§6.6) |
| `camera_motion` | `camera_motion_restart()`, `camera_motion_add(frame, obs)`, `camera_motion_exposure_adjustments()` | blocks, their backgrounds and noise, holding the exposure and letting it change only when calm, "different from the background" (§6.3), column/row → direction |
| `motion_sense` (started) | `motion_sense_update()`, `motion_sense_watching()`; later `motion_next_event(&ev)` | when the detectors watch (still) and when they learn again, the camera's exposure follows the view while moving, the movement log; later turn compensation, tracking, react-or-log, exit side, camera/ToF agreement, still/pursuit gating |
| `behaviour` | `behaviour_step(now)` → drive command | the order of needs, the Safety guard, targets, scan progress, the 50 cm rule, choosing open space |
| `debug_console` | `debug_console_update()` | what gets printed and how often, single-key commands |

Floor detection lives in `rangefinder`, not `world_map`: it's knowledge about the
sensor (its height, zone angles, calibration). The map only receives readings
already labelled floor / obstacle / no target.

**Who owns which knowledge** (avoids APOSD's *information leakage*):
- The cell size is known only to `world_map`.
- Zone layout, sensor mounting and floor calibration are known only to
  `rangefinder`; everyone else sees robot-frame rays.
- Camera geometry (focal length, column → bearing) is known only to
  `camera_motion`.
- Message layouts are known only to `link_msgs.h`.
- Wheel diameter and track width are known only to PicoB's `odometry` and `drive`.

---

## 11. APOSD red flags: tracking and review

The book's red flags, and where each is likely to show up in this project:

| Red flag | Where it threatens here |
|---|---|
| **Shallow module** | thin wrappers like `tof_get_zone(i)`; a `pid.c` that is only setters and getters |
| **Information leakage** | zone layout known to both the map and movement detection; cell size used in behaviour; message structs parsed outside `link` |
| **Temporal decomposition** | modules named after loop steps (read → process → send) instead of by what they know |
| **Overexposure** | callers having to set VL53 resolution and frequency to use `rangefinder`; defaults should cover the common case |
| **Pass-through method** | `main` → `pose` → `map` forwarding scans without adding anything |
| **Repetition** | rotate/angle-wrap maths copied around; protocol structs duplicated in A and B |
| **Special-general mixture** | behaviour-specific logic ("is the target reached?") inside `world_map` |
| **Conjoined methods** | `map_begin_update()` / `map_end_update()` pairs that only make sense together |
| **Comment repeats code** | `x++; // increment x` |
| **Implementation documentation contaminates interface** | header comments explaining the algorithm instead of what the function promises |
| **Vague name** | `data`, `state`, `process`, `handle` |
| **Hard to pick name** | a module that's hard to name usually has a muddled purpose; take it as a design warning |
| **Hard to describe** | an interface comment that needs paragraphs |
| **Nonobvious code** | units and frames left implicit; names carry them instead (`range_m`, `heading_world`) |

**Process:**
1. **Comments first** (APOSD ch. 15): write each module's header (interface plus a
   comment on what it promises) before its implementation.
2. **`REDFLAGS.md` log:** every red flag found while coding is recorded (module,
   flag, how it was fixed or why it was accepted).
3. **Review pass at the end of every milestone,** using the table above as the
   checklist. Fixes land before the next milestone starts.
4. **Host tests for every module with logic** (`./run_tests.sh`): each compiles one
   module on the Mac against stubbed drivers and checks its behaviour (e.g. the
   drive ramp, odometry against a simulated robot). Hardware tests on the robot
   then only have to confirm the hardware and the tuning.

---

## 12. Milestones

Each milestone ends with a test on the robot and a red-flag review.

| | Milestone | Done when |
|---|---|---|
| M0 ✅ | **Foundations:** `common/link` (its byte and frame counters double as the UART test); PicoB `drive` (closed-loop wheel speed) and `odometry`; HELLO / MOTORS / DRIVE / ODOM / LOG; PicoA `body`, `debug_console` and a square test | End pose within ~1 cm and ~1° of odometry after a 50 cm square |
| M1 ✅ | **Calibration:** gyro drift standing still, gyro scale (10 × 360° against a mark), encoder distance (2 m), the same on the carpet; host test for PicoA `body`. Effective track width only matters for feedforward (the gyro trim corrects turning), so it's measured but not critical | Measured numbers in the README; constants adjusted only if off by > 1 % (none needed on waxed wood; carpet still to measure) |
| M2 ✅ | **Map:** per-zone floor learning, map from ToF and odometry, drops, map printed as text in the debug log (§14) | Open floor shows no obstacles, also while braking; walls stay put while turning; drops marked |
| M3a | **Movement detection while still** (§6.1–6.3, 6.6): camera driver out of bring-up; the change grid; VL53 and camera detectors; tracking with the VL53 only, exit side (combining the camera, react-or-log: later, §6.5) | The target in the debug log matches what you do in front of the robot |
| M3b | **Turning to look** (§6.6, §7.1): states IDLE / SCAN / WATCH; turn after a target leaving the view, stop, learn, on again if it left meanwhile (first iteration, written); then back to open space and no turn into a near obstacle. Smooth pursuit (§6.4, §7.2) only if the hops need it | The robot turns to face someone who walked past it, then returns to open space |
| M4 | **Behaviour:** Investigate (driving to the target), the Safety guard for driving | Robot drives to movement, stops 50 cm away, then turns to face open space |
| M5 | **Staleness, 390° scan with the full-turn check, map changes** | Heading error after a scan ~1–2°; move an object while the robot isn't looking and the robot finds it |
| M6 | **Correction against the map** | Heading and position drift corrected over a few minutes of operation |

**Later (kept on purpose, not in v1):**
- **Roll during turns:** in-place turns make the body roll −3…−5° as it leans on its
  tyres. Could serve as a cross-check that the robot is really turning
  (e.g. a wheel slipping without turning the body), or as a hint of floor grip
  (waxed wood vs carpet).
- **Slip flag** in ODOM: encoders vs gyro disagreeing while driving.
- **Camera in the pose:** track keypoints (e.g. FAST corners on the 160 × 120
  image) while turning to measure the rotation angle, as a third opinion next to the
  gyro and encoders. Also a stationary check (unchanged image) to support gyro bias
  re-estimation.
- **Kalman filter:** a 3-state EKF (x, y, heading) replacing the inside of `pose`,
  tuned from M1–M6 logs. The interface stays the same.
- **Path planning** around obstacles (A* on the 2D grid).
- **Battery watch:** once the BT_SNS wire (pack → ÷3 divider → PicoA GP26) is
  fitted: stop scanning and chasing below 6.4 V (3.2 V/cell), motors off below
  6.0 V (3.0 V/cell). Until then the BMS cutting the power (2.5–3.0 V/cell) is the
  only protection, so keep an eye on run time.
- **PC link:** the robot sends its map, pose and events to the PC as data (not text), and the page of `pc/robot/` draws them. Same framing as the inter-Pico link, its own message set, over the WiFi connection that carries the console today (ROBOT_WIFI.md).

---

## 13. Open questions and decisions

- Detail values to confirm on the robot: 20 s before returning to open space,
  thresholds and frame counts in §6 (from M3a logs).

Settled: robot dimensions and height, sensor positions, camera field of view,
indoors on waxed wood with one carpet, no thermal camera for now, order of needs,
fully autonomous with no PC app in v1, ToF zones with no target mark the first 1 m
empty and leave farther cells alone, a blocked path counts as arrived, no battery sensor for now.

Decided along the way:
- The camera driver comes out of bring-up in M3a (nothing earlier uses the camera).
- Movement (§6, §7.1-7.2): states IDLE / SCAN / WATCH; detection against a learned
  reference, not the previous frame; uncertain and flaky VL53 zones are used
  (changes of state score); detection continues while turning to follow (no stop
  between steps, no limit on the turn); react only if the horizontal part of the
  movement's direction is > 0.5, else log; turning to follow comes in M3b, driving
  to the target stays M4; ST's motion indicator not used.
- No separate UART echo test: the link's counters (`l` in the console) do that job.
- **Safety stops** (PicoB): no DRIVE for 250 ms, a wheel not following its target
  for 1 s (stopped, far too slow or turning the wrong way; each of the four wheels),
  or pitch or roll beyond 15° (lifted, tipping over, climbing; normal driving stays
  within ~5°). After one, the motors stay off until switched on again; PicoA does
  not restart them by itself.
- Losing PicoA's serial monitor doesn't stop the robot (the 2 m test runs with the
  USB unplugged); PicoB's own stops remain.
- The square test's 0.5–1.8° overshoot per turn is not fixed: it's the test's stop
  logic, and M4's behaviour steers to headings using the pose.
- Calibration tests are console keys in one firmware (`d`, `r`, `f`, `b`), each
  printing progress and a result; the status line comes every 15 s so long tests
  stay readable.
- Map decisions (floor learning at every start, nothing fading, drops from rows 7-8
  only, floor assumed in the blind spots): §14.

---

## 14. The map as built (M2)

Goal (§12): **an open floor shows no obstacles, also while braking; walls stay put
while turning.** Everything runs on PicoA. How it works now:

**Sensor and frames.** ToF 8 × 8 at 15 Hz, continuous; each zone gives its closest
sure target (status 5, 6, 9). The sensor is turned 90° on the PCB (the bring-up
viewer's rotation); not mirrored. Each frame is placed with the pose at the moment
it was measured (`pose_at`); PicoB's clock comes from ODOM's t_B (§9).

**Start-up scan** (the first need, §7): at power-up without USB, after trying the
WiFi (ROBOT_WIFI.md), or with `n`: the map is cleared and the
robot's heading becomes the map's x axis (up on the print); it turns 390°, keeping
the frames; the floor is learned from them and they are mapped; motors off while
the map is built and printed (it takes longer than PicoB's 250 ms DRIVE timeout);
then it turns to the most open direction (average free distance in layer 0 across
the camera's ±25°, every 10°; ties: the middle of the widest open sector) and
stops with the motors off.

**Floor learning** (fresh at every start, nothing stored). Each floor zone (rows
5-8) takes only readings that fit its patch of floor (between its upper and lower
edges, ±2°, converted to the robot level with the pitch): shorter ones are
obstacles, longer ones drops. Of those, a long and common distance (around the
75th percentile) is the floor; the zone learns its effective angle (where a ray
from 7 cm meets the floor there) and its scatter (its obstacle margin, at least
2 cm). A zone whose floor comes out beyond 1.25 × where its centre meets a flat
floor isn't learned (the 5th row in a room mostly sees walls). Rows 7-8 must
always have a floor: where none was learned they take the floor the sensor's 7 cm
height gives. Why learn at all: a zone reports the nearest surface in its 5.6°
patch, so the 6th row reads 35-53 cm where its centre's geometry says 48; rows 7-8
are close to geometry.

**What each ray says** (`rangefinder_scan`):
- Rows 1-4 (and an unlearned 5th row): an obstacle if the hit is ≥ 2 cm above the
  floor and not possibly the floor at the zone's lower edge; else clear up to it.
  No target: clear for 1 m. The 5th row: hits only closer than 0.95 m.
- Rows 6-8 (learned): floor if the reading fits the floor (clear to it, and the
  floor seen over the zone's patch, from where its lower edge meets the floor to
  the reading); an obstacle if shorter by more than the zone's margin.
- Rows 7-8: **no floor** when there is no return, or one more than 15 % beyond the
  far end of the zone's floor patch (pitch included). Row 8 sees the floor ~14 cm
  ahead of the front edge: at 10 cm/s, 3 readings (0.2 s) plus braking (~1.3 cm)
  stop ~10 cm short of an edge.
- Row 6: far or missing readings tell nothing (on the shiny floor they are often
  reflections, e.g. the wall); nose up doesn't stretch its patch (it would reach
  1.6 m), nose down shortens it.

**Cells** (§4.3, §4.4): a hit makes a cell occupied, 6 readings in a row through
it clear it; nothing changes with time; each cell keeps the time of its last
measurement. A ray that reaches the floor clears layer 0 all the way to it (below
2 cm it passes under anything that would block the robot). Layer 0 also keeps
"floor seen" (from floor readings, and within 15 cm of the robot: it stands and
turns there) and the no-floor count (`?` after 3, the floor not seen in between;
seeing the floor resets it). Free distance (`map_free_distance`) stops at
occupied, `?` and unknown cells; `:` counts as free.

**Known behaviour on the robot** (see also the README's measured facts):
- A flat face often fills 2 cells (`####`): zone width (~5 cm at 50 cm), range
  noise, the changing view during the turn, cell edges; rays can't pass through
  the face, so the back cell stays. Possible refinement: clear a cell behind a face
  when the face is seen again from closer.
- With 6 misses to clear, a false `##` stays until the robot looks through it 6
  times.
- Glossy furniture on the waxed floor can give a few `?` right in front of it
  (accepted: next to an obstacle, and on the safe side).
- Map changes (§4.6) are counted but noisy (~1600 after one scan and a square): M5
  must filter them (e.g. a change counts only if it persists or clusters).

**Geometry limits:** layer 0 (2-12 cm) is only seen within ~1 m (farther walls show
only above 12 cm, `''`); a low obstacle reads like the floor behind it until close
(an 8 cm box shows up at ~40 cm); a 3 cm box at exactly 25 cm falls between the 6th
and 7th rows (seen at 27-34 cm and 16-20 cm while approaching); if the sensor
reports perpendicular rather than along-the-ray distance, side zones read up to
6 % short (the floor learning absorbs it for the floor).

**For M4 (Daniel):** the robot should face the farthest path it can actually drive:
a corridor the robot's width (23.5 cm plus margin) straight ahead, through `.` and
`:` (rows 7-8 confirm the floor while driving), stopped by `##`, `?` and blank.
Today's "most open" (average over the camera's ±25°) is for where to look. Drive
forward only while rows 7-8 see the floor right ahead.
