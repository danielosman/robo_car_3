# RoboCar — robot app plan

Status: **draft for review, no code yet.** This plan covers the real firmware
(`picoA/app/`, `picoB/app/`, `common/`) of a **fully autonomous** robot, built on
the drivers verified in the bring-up (tag `pcb-bringup-v1`).

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
  cells from the VL53L8CX. Each cell remembers whether something is there and how
  recently it was seen. Repeated sightings make a cell more certain, and old
  knowledge fades.
- **Knowing where it is.** PicoB tracks the robot's position and heading from the
  gyro (heading) and the wheel encoders (distance). PicoA corrects heading drift
  using the VL53: after each full turn it compares the start of the turn with the
  end, and it matches readings against the map.
- **Noticing movement.** While standing still, PicoA compares each camera frame and
  ToF frame with what it saw before. While moving, anything that contradicts the
  map counts as a change.

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
| R2 | **Cell memory.** Each cell holds a timer of at most 240 s that counts down in real time. Each reading that sees the cell occupied adds 60 s; each reading that sees it empty removes 60 s. At 0 the cell is empty. One noisy empty reading only takes a solid cell from 240 to 180, so it stays occupied; four empty readings in a row clear it quickly. |
| R3 | **Pose.** Estimate the robot's pose from the front-wheel encoders (wheels Ø 9 cm, 1 cm wide), the IMU and, later, the camera. Start without the camera and without a Kalman filter; keep both as future steps. |
| R4 | **Protocol.** Design the PicoA ↔ PicoB protocol. |
| R5 | **Attracted by movement.** Detect movement with the camera and the VL53 while the robot stands still. Any movement means something in the surroundings moved. |
| R6 | **Direction of movement.** Know where the movement went, including whether it left the field of view to the left or the right. |
| R7 | **Go to it.** Turn toward the movement and drive toward it until 50 cm away. |
| R8 | **Keep the map fresh.** When cells behind the robot expire, rotate 360° to see them again. |
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
| Movement detection with the camera (R5) | Camera and ToF movement detection **only while stationary**; while driving, use **map contradictions** | While the robot moves, everything in the image moves. A map contradiction ("a hit where we recently saw free space") works while driving and during the 360° scan, and is exactly R9. |

---

## 3. Robot geometry

| | Value | Source |
|---|---|---|
| Size | **23.5 cm wide × 19 cm long × 10 cm high**, wheels included; front edge 9.5 cm ahead of the centre | you |
| Wheels | Ø 9 cm, 1 cm wide; 282.7 mm per revolution, 3 575 counts/rev → **12.6 counts/mm** | you, encoder bring-up |
| Track width | **≈ 22.5 cm** (23.5 − 1, wheel centre to wheel centre) | derived |
| Wheelbase | **≈ 10 cm** (19 − 9) | derived |
| Turning in place | outermost point sweeps **≈ 15 cm** radius | derived |
| Top speed | ~50 RPM at full power → **~0.24 m/s** | bring-up |
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
- Per cell: `occupied_until` and `observed_at`, both u16 seconds of robot time.
  6 400 cells × 4 B = **~26 KB** of PicoA's 520 KB. A sweep every few minutes clamps
  old timestamps so the u16 wrap (18 h) never matters.

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

1. **Calibrate each zone's floor distance once** on an open floor: median of 2 s of
   readings, stored. This absorbs the cone shape, the sensor's distance
   convention and the floor's reflectivity.
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
   obstacles, and its floor readings are ignored.

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
| UNKNOWN | never observed, or `now − observed_at > 240 s` |
| OCCUPIED | `occupied_until > now` |
| FREE | observed within 240 s and not occupied |

### 4.4 Update rule (your R2)

Applied for **every ToF frame** (10–15 Hz):

```
hit  (a range ended in this cell):   occupied_until = min(now + 240, max(occupied_until, now) + 60)
miss (a range passed through it):    occupied_until = occupied_until - 60   (FREE once ≤ now)
both:                                observed_at = now
```

- One noisy miss takes a solid cell from 240 to 180, so it stays OCCUPIED. Four
  misses in a row (~0.3 s) clear it. One sighting gives 60 s, and the next miss
  clears it.
- **Within one frame, a cell gets at most one update; if any ray ends in it, it's
  a hit.** Near the sensor many of the 64 rays pass through the same few cells, and
  one frame shouldn't count as 20 misses. This is the only limit; there is no
  waiting or averaging across frames.
- A cell that isn't seen again keeps counting down: one sighting fades to FREE
  after 60 s, the cell stays *known* until 240 s, then becomes UNKNOWN.

### 4.5 Adding a ToF frame

For each of the 64 zones, rotate its precomputed direction vector by the robot's
pose. Walk along the ray in half-cell steps: cells before the measured distance get
a miss, and the cell at the distance gets a hit (or nothing, if §4.2 says it's
floor). **Zones with no target mark the first 1 m of the ray as misses and leave farther cells untouched**: "saw nothing" is only trusted close up. Cost: 64 rays ×
≤ 40 steps ≈ 2 500 cell updates per frame, which is trivial.

### 4.6 Changes (R9)

While adding a frame, the map reports a **change** when:
- a **hit** lands in a FREE cell (seen empty within the last 240 s), or
- a **miss** clears a cell that had built up ≥ 120 s of occupancy.

Changes come with their world position. This works while driving and while
scanning.

### 4.7 Questions the map answers

- **Staleness (R8):** "how much of the ring 0.3–1.5 m around the robot is UNKNOWN,
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
(motors commanded to zero, encoders not moving, for ≥ 0.5 s). Two error sources
remain:
- **Bias drift:** small, and re-measured at every stop.
- **Scale error:** the sensor's sensitivity is off by up to a few %, so turning a
  real 360° may add up to e.g. 352° or 367°. This is calibrated by the full-turn
  check (§5.4), and the correction is kept.

### 5.2 Distance: encoders (PicoB)

The average of the two front encoders, applied along the current heading.

### 5.3 Rotation from the encoders: secondary

While turning in place, rotation = (right wheel distance − left wheel distance) /
track width. With the geometric track width of 22.5 cm, a full turn is π × 22.5 ≈
**70.7 cm per wheel (~8 900 counts)**. The counts are precise, but **skid
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

Active only while PicoB reports **stationary for ≥ 0.5 s** (lets vibration settle).
Camera auto-exposure is **locked** while detecting; otherwise exposure changes look
like movement.

### 6.1 ToF

Each zone keeps a slow background average of its distance and a noise estimate. A
zone has **moved** if `|distance − background| > max(60 mm, 4 × noise)` in two
frames in a row. Neighbouring moved zones form a **blob**: its bearing is the
centre column's direction vector, its range the nearest moved distance.

### 6.2 Camera

Shrink 160 × 120 to 40 × 30 blocks (integer sums of 4 × 4 pixels) and compare them
with a slow background. A block has moved if the difference is above a threshold.
If more than ~40 % of blocks change at once, it's a lighting change, so ignore the
frame. The blob's bearing is the column vector from §3. The camera gives
**bearing only, no range**.

### 6.3 Combining them

The ToF's 45° field of view lies inside the camera's 53°, so the camera sees ~4° more
on each side. If the camera and ToF bearings agree (dot product > cos 10°), it's
one event with bearing and range. The 3 cm sideways offset shifts the ToF bearing
by up to ~3° at 50 cm and less further away, which is within that tolerance.
Camera only (e.g. in the outer 4°): bearing only, with the range from the map if
it has something there.

### 6.4 Tracking and exit side (R6)

The tracker remembers the active blob's last bearings. When the blob disappears:
- last seen in the outer ~15 % of the camera's field of view → **EXITED_LEFT /
  EXITED_RIGHT**; the side is the sign of cross(forward, bearing).
- otherwise → **LOST** (it stopped moving or went out of range).

### 6.5 While moving

No camera or ToF movement detection. Map changes (§4.6) take over.

**Events produced:** `MOTION(bearing, range or none)`, `EXITED(side, last
bearing)`, `MAP_CHANGE(world point)`.

---

## 7. Behaviour (PicoA)

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
| 1 | **Scan** (R8) | too much of the surroundings UNKNOWN (e.g. > 30 % of near cells, or a rear sector mostly unknown), or right after power-up | Stop, then turn 390° at ~30°/s (~13 s) with the full-turn check (§5.4). Map changes found during it become Investigate events |
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
  │ debug log ─► USB serial              │          │                          │
  └──────────────────────────────────────┘          └──────────────────────────┘
```

- **PicoA (brain):** camera, ToF, map, movement detection, pose
  correction, behaviour, and a debug log on its USB serial.
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
| HELLO | both | at boot, 1 Hz until answered | protocol version, firmware name | Version mismatch: PicoB keeps the motors off |
| DRIVE | A → B | 20 Hz | v (m/s), ω (rad/s) | **PicoB stops if no DRIVE for 250 ms** |
| MOTORS | A → B | on change | enable / disable | |
| ODOM | B → A | 50 Hz | t_B (µs, u32), x, y, yaw (not wrapped, so a full turn reads +2π), v, ω, pitch, roll, flags: stationary, slip, motors on, stopped by watchdog | |
| TIME_PING / TIME_PONG | A → B / B → A | 1 Hz | t_A / t_A + t_B | PicoA works out PicoB's clock offset, so ToF and camera frames can be matched to the pose at the time they were taken |
| CALIBRATE | A → B | on request | — | Hold still: gyro bias calibration |
| GYRO_SCALE | A → B | after a full-turn check | scale correction | Stored by PicoB and applied to the gyro from then on |
| STATUS | B → A | 2 Hz | wheel speeds, PWM, gyro bias and scale, error counters | |
| LOG | B → A | rare | text | PicoA prints it in its own debug log, so one serial monitor shows both boards |

Bandwidth: ODOM ≈ 45 B × 50 Hz ≈ 2.3 kB/s; everything together is under 5 % of the
link.

**No PC app in v1:** the robot is fully autonomous. During development, PicoA's
USB serial prints a readable debug log (pose, events, behaviour changes, link
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
| `link` | `link_init()`, `link_send(type, body, len)`, `link_poll(&msg)` | UART, IRQ/DMA, COBS, CRC, sequence numbers, error counters |
| `vec2` (helper header) | `dot`, `cross`, `rotate`, `normalize` | — (shared maths, not a module boundary) |

**PicoB**

| Module | Interface | Hides |
|---|---|---|
| `drive` | `drive_set(v, ω)`, `drive_stop()`, `drive_update()` | side → driver mapping, forward signs, per-side wheel speed PI, effective track width, ramping, PWM, rear motors following the front ones |
| `odometry` | `odom_update()`, `odom_get(&odom)`, `odom_set_gyro_scale(s)` | gyro integration, bias and scale correction, stationary detection, encoder scaling, slip flag, tilt filter. Also provides the measured wheel speeds `drive` needs, so the wheel diameter lives in one place |

**PicoA**

| Module | Interface | Hides |
|---|---|---|
| `body` | `body_update()`, `body_connected()`, `body_odom()`, `body_motors(on)`, `body_drive(v, ω)` | PicoB as PicoA sees it: the link protocol, greeting and version check, repeating DRIVE for PicoB's safety stop, re-sending MOTORS until PicoB agrees, printing PicoB's log lines |
| `camera` (taken out of bring-up `camera.c` in M3) | `camera_start()`, `camera_frame(&frame)` (non-blocking), `camera_lock_exposure(bool)` | PIO/DMA, HM0360 registers and modes |
| `rangefinder` (on top of the `tof` driver) | `rangefinder_poll(&scan)` → 64 rays **in the robot frame** (origin, unit direction, distance, and floor / obstacle / no target) plus a timestamp | zone order, the 90° rotation, the 3 cm offset and 7 cm height, per-zone floor calibration and pitch adjustment, VL53 status codes and settings |
| `world_map` | `map_add_scan(scan, pose, changes)`, `map_cell_state(point)`, `map_staleness(pose, sectors)`, `map_free_distance(point, direction)` | cell size, layers, rolling window, timers, ray walking, change detection |
| `pose` | `pose_on_odom(odom)`, `pose_on_scan(scan)`, `pose_at(t)` | odometry history and interpolation, clock offset, full-turn check, map ← odometry correction, map matching |
| `motion_sense` | `motion_on_scan(scan)`, `motion_on_frame(frame)`, `motion_next_event(&ev)` | backgrounds, thresholds, blobs, tracking, exit side, camera/ToF agreement, stationary gating |
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
  `motion_sense`'s camera part.
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
| M0 | **Foundations:** `common/link` (its byte and frame counters double as the first UART test: the link was never brought up on the PCB); PicoB `drive` (closed-loop wheel speed) and `odometry`; HELLO / MOTORS / DRIVE / ODOM / LOG; PicoA `body`, `debug_console` and a square test | The square test drives a 50 cm square; the pose in the debug log matches where the robot ends up |
| M1 | **Calibration:** gyro bias drift and scale (turn 10 × 360° against a mark on the floor), effective track width, encoder distance on your floor | Measured numbers in the README |
| M2 | **Map:** per-zone floor calibration, map from ToF and odometry, map printed as text in the debug log | Open floor shows no obstacles, also while braking; walls stay put while turning |
| M3 | **Movement detection** while stationary (ToF, then camera); exit side. Take the camera driver out of bring-up first (moved from M0: nothing earlier uses the camera) | Events in the debug log match what you do in front of the robot |
| M4 | **Behaviour:** Investigate, Face open space, the Safety guard | Robot turns to movement, stops 50 cm away, then turns to face open space |
| M5 | **Staleness, 390° scan with the full-turn check, map changes** | Heading error after a scan ~1–2°; move an object while the robot isn't looking and the robot finds it |
| M6 | **Correction against the map** | Heading and position drift corrected over a few minutes of operation |

**Later (kept on purpose, not in v1):**
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
- **PC link:** the robot sends its map, pose and events to a PC app (`pc/app/`), and the PC can send commands back. Same framing as the inter-Pico link, its own message set; USB first, wireless (Pico 2 W) possibly later.

---

## 13. Open questions and decisions

None at the moment.

Settled: robot dimensions and height, sensor positions, camera field of view,
indoors on waxed wood with one carpet, no thermal camera for now, order of needs,
fully autonomous with no PC app in v1, ToF zones with no target mark the first 1 m
empty and leave farther cells alone, a blocked path counts as arrived, no battery sensor for now.
