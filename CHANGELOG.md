# Changelog

What happened, session by session, newest first: robot runs, results, what was
found and how it was fixed. [README.md](README.md) and
[ROBOT_PLAN.md](ROBOT_PLAN.md) show only the current state; the red flags of each
milestone are in [REDFLAGS.md](REDFLAGS.md).

## 5 Oct 2026: M2 passed; the map reworked

**M2 test (latest `picoA_app` of 4 Oct, waxed wood, living room).** The start-up
map matched the room (walls 1.5-2 m away, the table and sofa legs, an object ~40 cm
behind); no `##` on open floor. Using the unlearned 5th row again had turned the
area behind-left from unknown into free out to the walls. Daniel placed objects
to the left, then `n` and `q`: they were mapped where they were, the robot faced
away from them, and after the square (odometry 1.1 / −1.5 cm, 363.0°, as on 4 Oct)
`m` showed walls and objects in place and no obstacles from braking. **M2 passed.**
Red-flag review: REDFLAGS.md, "M2 review pass" (commit `bf30030`).

Found in the M2 test: isolated unknown cells inside the free area (often in radial
lines) cut the free distance short, so the robot turned away from a more open
front-right; map changes counted 1584 after one scan and a square (too noisy for
M5 as they stand). The `**` puzzle (it seemed to point the wrong way) was `n`
having reset the map's up between the scan and the square.

**Gaps.** The floor rays (7 cm up, pointing down) leave the 2-12 cm layer before
they reach the floor, so the last stretch of every floor ray never cleared layer 0.
Fixed: a ray that reaches the floor clears layer 0 all the way to it (anything
standing on the floor 2 cm or taller would have blocked it).

**Nothing fades (Daniel).** Before: R2's 240 s timer (+60 s per sighting, −60 s per
empty reading, counting down in real time), and cells turned unknown after 240 s
unseen. Now: a cell changes only when measured and keeps the time of its last
measurement; an obstacle clears after 6 empty readings in a row. Curiosity about
old cells will be the behaviour's job.

**Drops (`?`).** First version: rows 6-8 marked `?` on no return or a reading 20 %
past their floor, at once. On the robot: a ring of `?` at ~50 cm. Cause: the 6th
row sees floor anywhere from 35 to 71 cm (a 5.6° tall zone on a grazing floor), so
readings of 54-71 cm were floor, not drops. Fixed: "beyond" means past the far end
of the zone's floor patch plus 15 % (test: readings anywhere in the patch, 2157
false drops with the old rule, 0 with the new). The ring stayed thinner: `z` showed
the 6th row sometimes reads the wall (184 cm, a reflection off the waxed floor),
about once in 24 readings. Then: `?` only after 3 such readings with no floor seen
in between, and (Daniel) the 6th row doesn't tell drops at all; only rows 7-8
(steady at 28-31 and 20-22 cm) do. The VL53 status 12/14 zones top right moved to
top left when the robot was turned: the scene (two surfaces in a zone), not the
sensor. Living room afterwards: no ring, 2.0 m free on average, 2 `?` in front of a
white glossy cupboard (reflections; accepted).

**Desk.** A start-up scan 10-15 cm from a desk's edge learned the floor in only 9 of
32 zones (rows 6-7 none): half the turn looked over the edge and the long readings
spoiled the floor; without a floor rows 7-8 can't tell a drop, and the map showed
`.` beyond the edge. Fixed: floor learning takes only readings inside each zone's
floor patch (±2°); rows 7-8 fall back to the floor the sensor's 7 cm height gives;
`.` only where the floor was seen, `:` for free without floor seen. Desk again:
rows 7-8 learned (29 / 21 cm), `?` along the edges ahead and right, `:` beyond, `.`
only on the desk. Floor learning stays (Daniel asked why learn at all): the 6th row
reads 35-53 cm where geometry says 48.

**Removed object.** `n`, object removed, one turn with `r`, `m`: the object cleared,
the rest stayed. `:` showed right next to the robot and between the rings where
the floor rows end. Fixed (Daniel: assume floor in the blind spots): within 15 cm
of the robot is floor with nothing on it, and a floor reading marks its zone's
whole patch. On the robot `.` then also reached 1-1.6 m to the left: nose up ~3°,
the 6th row's patch reached 1.6 m. Fixed: nose up no longer stretches the 6th
row's patch. Last run: solid `.` disc ~50-60 cm around the robot, `:` beyond.

Commits: `bf30030` (M2 passed, review), `727264c` (gaps, no fading, drops, floor
seen), `8b7407f` (blind spots).

## 4 Oct 2026: new motors, M1 finished on waxed wood, M2 written

**Motor change.** All four motors replaced: the old ones were Pololu #2208 (298:1
LP 6V, 12 CPR #3081 encoder boards on the front ones only, encoder signs opposite
to the motor signs); the front-right one sometimes didn't start (it kept stopping
the tests with "right wheels not following"). New: GA46-N20E-0043 with Hall
encoders on all four wheels. The encoder constant was inferred from the first run
(126 rpm with the old 12-count constant = 54 rpm) and confirmed (~55 rpm on all
four wheels at full power, lifted). The left encoders' power was first wired
reversed (LEDs off, boards cool); fine since the fix. Coded: the encoder driver
counts all four in PIO instead of GPIO interrupts, odometry averages front and
rear per side, `drive`'s wheel stop judges each wheel, the bring-up prints every
wheel. All four encoder signs checked. PicoA had still been on protocol v3: both
on v4 since.

**M1 with the new motors (waxed wood).**
- Square: odometry 1.2 cm forward, 1.2 cm right, 362.6°; the real end pose
  practically the same. No safety stops.
- 10 turns: all 10, no stops; gyro 3600.5° in 126 s (28.5 °/s for a 28.6 °/s
  command); wheels −898.3 / +898.7 cm; effective track width 28.6 cm; the robot
  ended ~15° past the mark (gyro 0.42 % low: no correction) and really ~15 cm back,
  5 cm right, which odometry can't see.
- 2 m straight (USB unplugged): odometry 200.2 cm (wheels 199.9 / 200.5 cm),
  heading −0.1°, 0.1 cm sideways; real ~200.5 cm (+0.15 %): `WHEEL_DIAMETER_M`
  stays 9.0 cm. PicoA kept running on the battery; the result was printed on
  reconnecting and with `t`.

**M2 written:** `tof` split (sensor part in `drivers/`, USB streaming in
`bringup/tof_stream.c`); clock sync in `body`; `pose`, `rangefinder`, `world_map`,
`surroundings`, `behaviour`, `motion.h`; console keys `n`, `m`, `z`; host tests incl.
the start-up scan in a simulated room.

- **First run:** the sensor is not mirrored (hand test on `z`). Only the first of up
  to 4 targets per zone was used, so most zones were "unsure"; the floor rows read
  the near edge of their 5.6° floor patch (6th row ~35 cm, not 48), which drew a
  ring of false obstacles at ~60 cm; the row above the floor rows reached the floor
  at its lower edge when the robot nodded (far `''`); the map's up was PicoB's
  power-up heading. Fixed: closest sure target; the 5th row learns the floor too;
  each floor zone learns its effective angle and scatter; readings that may be
  floor at a zone's lower edge are free space; the scan's start is the map's
  origin.
- **Second run:** much better map (floor rows 21 / 29 / 38-43 cm). PicoB stopped
  with "drive commands stopped arriving" after the map print (mapping ~220 frames
  plus printing 3.4 KB kept the loop busy > 250 ms); a false `##` ~35 cm ahead-left
  (maybe the USB cable). Fixed: motors off while the map is built and printed; `m`
  switches them off first; ties in "most open" pick the middle of the widest open
  sector; a floor zone whose learned floor is farther than 1.25 × its centre's
  isn't learned (the 5th row in a room mostly sees walls); `z` shows what each zone
  decided and what each floor zone learned.
- **Third run:** the start-up scan ran without a stop; floor rows 6-8 at ~45 / 30 /
  22 cm, the 5th row rightly not learned; the false obstacle ahead-left gone.
  After it: an unlearned 5th row is used like the rows above.

Commits: `a3c93de` (motors, PIO encoders), `1d81812` (M2).

## 3 Oct 2026: M1 written, first calibration runs, review

Written: the calibration tests as console keys (`robot_test.c`), STATUS with the
gyro bias, wheel distances in ODOM, a 15° tilt stop on PicoB, the status line
every 15 s, no stop on USB unplug, host tests for `body` and `robot_test`.

| Test | Result |
|---|---|
| Tilt stop | Works: lifting one side stopped the square test with "tilted more than 15 deg" |
| Gyro drift, 10 min still | Bias −0.328 °/s, wandering only 0.009 °/s (−0.3328 … −0.3239): at most 0.5° per minute of driving. Yaw frozen while still. No change needed |
| 10 turns | Turn rate held at 28.6 °/s for 7 turns, then "right wheels not following" |
| 2 m straight | Ran, but the result was lost |
| Square (twice) | Odometry 3.6 / −1.9 cm, 363.4° and −0.5 / −3.3 cm, 364.7°; real end pose not measured |

- Front-right motor sticking: the controller reached full power within ~0.6 s and
  the rear-right motor on the same driver turned, so it was the motor, gearbox,
  wiring or driver channel A, not the software. Solved by the motor change.
- Console bugs: the welcome text and last result, printed the moment the USB
  connects, didn't show on the Mac, and later tests overwrote the stored result.
  Fixed: printed ~1 s after connecting, `t` reprints. Unexplained: after `f` and
  `g` (motors in a safety stop) nothing was printed, though the robot drove ~2 m;
  not seen since.
- Review pass (REFACTORING.md, all applied): new `brain` module on PicoB, MOTORS
  request numbers (protocol v4), USB print timeout, test table in `robot_test`,
  shared fakes. Commit `43081ca`.

## 2 Oct 2026: M0 passed

Square test on waxed wood, 50 cm sides at 10 cm/s, turns at 0.5 rad/s:

| | Odometry | Measured | Difference |
|---|---|---|---|
| End position, forward | 1.3 cm | 2.0 cm | 0.7 cm |
| End position, right | 1.1 cm | 1.4 cm | 0.3 cm |
| Total turn | 362.6° | ~362–363° | < 1° |

- Link: 0 bad and 0 lost frames. Speed held at 10.0 cm/s; turns at 26–32 °/s for a
  28.6 °/s command; heading within ±0.2° along each straight leg.
- The ~2.6° extra turn is the square test stopping each turn 0.5–1.8° late, not an
  odometry error.
- Roll dips to −3…−5° during turns in place (the body leans on its tyres).
- Found on the robot and fixed: the IMU breakout is mounted turned 180° (X backward,
  Y right); the app never started the encoders; both encoder signs were reversed.

## Before: PCB bring-up

Every connected part verified with the bring-up firmware and viewer (tag
`pcb-bringup-v1`). Decided while planning M0: taking the camera driver out of
bring-up moved to M3; losing PicoA's serial monitor stopped the robot (dropped in
M1: the 2 m test runs with the USB unplugged).
