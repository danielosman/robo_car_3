# Changelog

What happened, session by session, newest first: robot runs, results, what was
found and how it was fixed. [README.md](README.md) and
[ROBOT_PLAN.md](ROBOT_PLAN.md) show only the current state; the red flags of each
milestone are in [REDFLAGS.md](REDFLAGS.md).

## 7 Oct 2026: M3b first iteration: turning after a target leaving the view

Agreed with Daniel: as simple as possible for the first iteration. When the target
is about to leave the view (two or more readings, at the edge), turn to where it
will be (estimated from its angular speed and the known turn rate), stop and learn;
if it is seen leaving while learning, continue at the same angular speed. No
smooth following, no turning to a target that stopped, no back to open space or
obstacle check yet. ROBOT_PLAN.md §6.6 has the sentences.

New: the tracker's "leaving" report (`TRACK_LEAVING_*`); `tof_motion` notices
something passing out through an outer column while it learns; `motion_sense`
hands both to the behaviour; `behaviour.c` watches after the scan (or `a`), motors
on, and turns at 1 rad/s (at most 90°). Host tests for all three. Found in the
behaviour test: a fast target's predicted angle beyond 180° was wrapped and the
robot turned the wrong way; now only clamped. The web page (`pc/robot/`) got
buttons for `a`, `v`, `o`, `k` and `c` (M3a's keys were missing).

**On the robot (living room, without USB, the console over WiFi; Daniel walking
around it for ~4 min):** it kind of worked. 11 turns after a target leaving, both
ways, at 10-20 deg/s (+44, −31, −31, −36, −27, −39, +32, −32, +38 deg), turns
ending within ~1° of the command (e.g. +44 asked, +43.8 by the gyro); twice it
turned on after the target left while it learned (−61, −52 deg). Missed: sometimes
it didn't notice the movement at all, and often it didn't notice the target leaving
while it learned. Agreed: good enough for the first iteration. Likely causes, to
look at next: the view is learned only once PicoB reports the robot still (~0.5 s
after stopping), so a target that left before that is never seen; a zone needs 3
sure readings to have a "farthest", so farther than the VL53 reaches (upper rows)
it tells nothing; 2 zones in one frame of the outer column. The WiFi connection
dropped at the end ("robot silent for 6 s"); cause not looked at.

## 6 Oct 2026 (evening): M3a: the camera's movement detection

Proposed to Daniel in seven sentences (ROBOT_PLAN.md §6.3) and agreed; key `k`
for the camera's blocks (`v` and `o` were taken: `v` now logs both sensors).
New: `camera_motion` (blocks, background and noise learned over 1 s with the
exposure held, the light taken off as the median change, > 40 % differing = the
light changed: learned again; moved by `change_grid` as on the VL53; steady 1 s =
stopped), wired into `motion_sense` (the exposure is held while still, follows the
light while moving; "Movement (camera)" lines and "the light changed"), host test
`test_camera_motion.c` (25 and 100 frames/s, 11 seeds).

Found in the host test: a small hand sweeping ~66°/s is in each block for only one
used frame and is never found by "2 of 4 frames"; kept the agreed rule (a hand
waving at ~33°/s is found the whole time) and noted the limit in the plan.

**On the robot (walking right to left and back, a hand, a second light on and
off):** both walks and the hand were found on the correct side and moved the right
way (e.g. +22° → −14° for the hand left to right). Three things: every movement
line was followed by "ended": frames skipped to keep ~15 a second returned 0
("nothing moved"); they now return −1 and are ignored (the host test checks it).
Walking past at ~0.5 m, the person filled > 40 % of the view and was taken for a
change of light (twice). A lamp lighting ~25 % of the view was movement for as
long as it was on (~2.5 s: switched off without waiting), though steady blocks
should be taken in after ~1.3 s.

Fixes: the light changed only when more than 40 % of the blocks *start* to differ
in the same frame (a person comes in over several). The host test of someone
walking up until they fill the view then found a second trap: once they cover half
the view, the median change is theirs, so every other block "starts" at once; the
light is now taken only from the blocks that looked like their background in the
last frame. Tried and reverted: "steady" measured from frame to frame, for a bulb
warming up: in the host test a lamp brightening over 5 s still kept moving
(taken-in blocks drift off again); waiting for the `k` prints instead of guessing.
A test-only trap on the way: a `static float light` in the module and in the test
were one variable (tentative definitions in C); renamed `light_shift`.

## 6 Oct 2026 (night): M3a: exposure and movement kept apart

Daniel: the exposure was changed too eagerly; adapt slowly, like an auto-exposure;
a light switched should be movement; and don't mix exposure control with movement
detection: adjust the exposure only when calm, and don't watch with the camera
meanwhile. Now: the driver waits before adjusting (5 s when 25 % off, down to 0.5 s
when far off) and steps exposure × gain by at most ×1.25; frames say when it wants
a change. `camera_motion` holds the exposure while still and lets it change only
after 5 s with nothing moving (or after 30 s of wanting), then learns the view
again. The "40 % start at once = light changed" rule and the median light
correction are gone: a light switched is movement until steady.

Found in the host test: blocks left right at the edge of "differs" after a light
switch flickered in and out of it, and each dip restarted their 1 s steady timer
(movement trickling for seconds, maybe the lamp on the robot); a block that keeps
its new value is now taken in after 1 s even if it dips now and then. The median
light correction kept moving by a few levels while blocks were being taken in,
which also delayed it; slow light is left to each block's drift. Test fixes on the
way: objects drawn with a fixed texture (redrawn noise made them flicker), the
test's clock (0.1 s steps were 80 ms at 25 frames/s), the simulated driver's wait.

## 6 Oct 2026 (last): the tracker fixed, on the robot

Four walks with the fixes: at ~0.5 m right to left and left to right, at ~1.5 m
both ways. Each kept one target from edge to edge (speeds 5-31°/s), no jumps to far
zones, and ended with the correct side ("left the view on the left at +16 deg" … "on
the right at −20 deg"), ~0.5 s after the ToF's movement ended. The tracker does what
§6.6 asks; next is turning to look (M3b).

## 6 Oct 2026 (later still): the tracker on the robot, over WiFi

At ~0.5 m: left to right followed and "left on the right" as it should; right to
left, Daniel was a 24-zone blob centred at +3° plus a 2-zone piece at −17° (his
feet), the tracker matched by centre and stayed on the piece, reported "left on the
right", then jumped to a single zone at 2.76 m. At ~1.3-1.5 m both walks were right
(speeds 13-29°/s, exit sides correct), but at the end of each the target jumped to a
far zone in the same direction (2.80 m, 2.21 m). Fixes: blobs report their leftmost
and rightmost cells; the target is the biggest blob reaching within 10° of where it
is expected, and not more than 0.5 m nearer or farther; target lines are printed
only for frames in which it was seen (one showed an old position). Host test cases
from both logs.

## 6 Oct 2026 (after that): the VL53 tracker

Daniel: implement the tracker now. `tracker.c` (ROBOT_PLAN.md §6.6): one target
among the VL53's blobs, its direction, range and angular speed, lost after 0.5 s
without a blob within 15° of where it is expected: "left the view on the left /
right" from the outer zone columns (beyond ±16°), else "stopped". `motion_sense`
logs it ("Target …" lines with `v`) and forgets it when the robot moves. Found in
the host test: the angular speed from two directions 0.5 s apart was off by up to
~8°/s with ±2° of jitter; now a least-squares line through ~0.8 s. Not yet on the
robot.

## 6 Oct 2026 (end of session): the plan simplified

Proposed next: combining the camera with the VL53 into one target, then turning in
hops, then smooth following. Daniel: too complicated; start with the VL53 alone for
tracking. Written down as ROBOT_PLAN.md §6.6 (tracker in five sentences, then
turning to look in hops); combining (§6.5) and the camera during turns (§6.4) put
off; M3a/M3b in §12 changed to match. Not committed yet.

## 6 Oct 2026 (late night): M3a: the camera on the robot, evening light

Daniel walked right to left and back, walked to the light switch, switched the
light on. The light switch: all 300 blocks moved for ~1.5 s, "ended", then the
exposure was adjusted when calm (40 ms × 10.1, mean 113); two `c` ~10 s apart
within ±1 per block; every block's noise at the 2.0 floor. Most extra camera blobs
were 14-19° below level: Daniel's shadow on the floor (the evening lamp). What
looked like the ToF ending 2-3 s late was a single 5th-row zone at 2.8 m, twice
~15 s apart: most likely his feet on the way to the switch and back. Movement log
lines now start with the robot's time in seconds, to answer such questions from the
log.

## 6 Oct 2026 (later): M3a: the VL53's movement detection

New: `change_grid` (shared by both detectors: scores over 4 frames, thresholds with a
neighbour, absorbing a cell moved for 5 s, blobs), `tof_motion` (each zone's
reference: how often sure / unsure / none, its distances; scores per §6.2),
`motion_sense` (feeds it while PicoB says the robot is still, logs movement), keys
`v` and `o`, and `z` now also prints each zone's signal, ambient light and second
target. `rangefinder` gives the zones' raw readings (`rangefinder_zones()`).

Found in the host test: scoring states with a hand-set weight let rare unsure
frames add up (a 2 % rate of them gave false movement within a minute). Scores are
now bits of surprise (−log₂ of how likely the reading was under the reference),
with a threshold of 20 bits over 4 frames (14 with a neighbour): no false movement
in a minute of a noisy room over 10 random seeds, a person found in 2 frames, an
object put down absorbed after 5.0 s. Which way a target moves moved to the tracker
(it needs the same target in two frames).

**On the robot (step 1, still for ~2.5 min, living room):** ~16 false movements, all
single zones: the same floor-row zone (6th row, left edge) reading 58 cm, its
reflection on the waxed floor, again and again, and far zones (1.7-3.3 m, upper
rows) now and then reading another surface. A reference with one distance per
zone, forgetting in ~3 s, saw each switch as new. Now each zone knows up to 3
readings with how often they come, over ~1 min, learns every reading (one seen
once stays known, and rare), and a zone alone needs 30 bits (3 frames of a new
distance). The host test's room now has those glitches (1-2 frames every ~10 s):
quiet for 5 min over 10 seeds. Each movement line now lists its zones: what they
read against what they are known to read.

**On the robot again (step 1, ~2 min still, then a hand twice):** no false movement;
both hand passes found, on the correct side (+17°, then −18°), at 14-24 cm. The log
showed a new distance scoring 6 bits instead of 10 from its 2nd frame: it had been
learned at once, so a lone zone could reach only 28 of its 30 bits. Readings are now
learned once they leave the 4-frame window (the view takes ~0.8 s to learn); host
test: a thin pole in one zone is found in 3 frames. Zone lines show the bits over
the window.

**Steps 2-4 on the robot, and a rewrite:** walking and a hand were found, but after
a box was put down and taken away, movement went on for a long time. `z` showed why:
almost every zone above the floor rows reports a faint unsure target at ~30 cm
(signal 3-10 against the floor's 150-730), the floor's echo inside the sensor, also
facing just a wall and a door frame; the real target is the next one, sure. Using
the nearest target whatever its status had made the detector watch the echo, and
the bits / known-readings machinery made it hard to see. Daniel: overcomplicated.
Rewritten simply: the nearest sure target per zone, as the map uses; a background
per zone (the middle of its sure readings while learning); movement = clearly
closer than the background (8 cm or 8 %) in 2 of 4 frames; farther is never
movement and becomes the background after 1 s; closer for 5 s becomes the
background. The raw per-zone readings (signal, ambient, second target) are gone from
`rangefinder` and `z`. Two traps the host test found on the way: the farthest
reading as background (one reflection frame while learning became the background)
and "nothing" winning the median for a far target that comes and goes.

**Still, nothing moving (robot facing a wall and a door frame):** steady false
movement, nearly all in the 6th row (−8°): its zones grazing the waxed floor read
the floor (~40 cm) or the wall behind (1.3-2.5 m), and a median background of the
wall made the floor "closer"; also a door frame's edge now and then. Now floor zones
ignore readings beyond their floor patch (the map's limit), the background is the
nearest distance a zone gives regularly (2nd-nearest of 1 s), a zone alone needs 3
of 4 frames, a brief nearer reading that goes back becomes the background, and
farther readings need 10 s in a row. The host room has all of these: quiet over 20
seeds. Detector times are in seconds of `RANGEFINDER_HZ` (Daniel asked about 10 Hz:
to try once 15 Hz works).

**Again (a minute still, a hand, Daniel's wife walking through, a box):** the still
minute was quiet; the hand and the person (small groups of zones at 1.4-2.2 m on
the left) were found; taking the box away gave nothing. But a box put down kept
"moving" for 5 s after it was let go: the 5 s rule called anything new movement.
Movement is change now: a zone closer but steady for 1 s has stopped, and that
distance is its background; `change_grid` lost its absorbing. Host test: a box is
still after 1.1 s, a hand waving for 3 s counts the whole time, 20 seeds.

**On the robot:** box put down: movement while it was handled, "ended" ~1 s after
it was let go; taken away: one line (the hand); the box moved from left to right:
+20°, −3°, −18°, ended. The VL53's movement detection works as intended.

## 6 Oct 2026: M3a started: the camera driver out of bring-up

`picoA/drivers/camera.c`: capture runs continuously into three buffers (one being
filled, the newest complete frame, the one the caller holds), one DMA interrupt
per frame; the PIO program now waits for VSYNC itself, so restarting it late can
only skip a frame, never start halfway. Each frame carries the time of its middle
row's exposure (frame period measured from the frames). `camera_lock_exposure()`
turns auto-exposure off at a whole number of 10 ms with the digital gain making up
the difference. The bring-up firmware uses the driver (same viewer protocol and
modes; 320-wide buffers only there). The app starts the camera at power-up; keys
`c` (frame as blocks, frame rate, exposure) and `e` (lock).

Then reworked from what the robot needs, not from what bring-up had (Daniel): one
readout (160 × 120, Sub4, no binning; the binning experiments, modes 0-4 and the
tone-curve choices removed, and the 320-wide buffers with them); the sensor's
auto-exposure off for good and the driver's own exposure control instead (10 ms
steps, then gain, `camera_hold_exposure()`), so there is no switching between auto
and manual; the frame length follows the exposure and the line period is measured
continuously, nothing assumes a frame rate. The datasheet (V01, found online, kept
next to the repo, README "References") confirms a rolling shutter: frames now give
each row's time. `e` is gone. The bring-up viewer shows the driver's exposure with
a hold checkbox. Builds, host and viewer tests pass; not yet run on the robot
(README, "Camera").

**On the robot (camera steps 1, 2, 5, evening light):** worked first time. Line
period 42.7 µs (measured), exposure 40 ms (the longest step) with gain ×5.9, 24.9
frames/s, mean brightness 98 against the target 100; three presses over ~15 s gave
the same blocks within ±1 (no flicker bands, exposure steady). The middle row's
moment was 24-48 ms before printing (20 ms is half the exposure). Not mirrored: a hand held
at the left edge of the robot's view (from behind the robot) darkened columns 1-8;
the driver then doubled the gain (×5.9 → ×11.6) to bring the mean back to 98, so
the rest of the image got ~1.5 × brighter (why motion detection must hold the
exposure while it has a reference).

## 6 Oct 2026: M3 planned (movement detection and following)

Planning only, no code (ROBOT_PLAN.md §6, §7.1-7.2, §12). Daniel's points: robot
states IDLE / SCAN / WATCH; a common interface for the camera and VL53 detectors;
no assumption that movement goes left or right (react only if the horizontal part
of its direction is > 0.5, else log); follow without stopping between turns, or
someone walking past is gone; use uncertain and flaky VL53 zones, since their
changes are information. Proposed and agreed: a reference per cell (not the
previous frame) in a change grid both detectors share; zone states scored by how
rare they were in the reference; pursuit with the camera reference shifted by the
gyro heading and the map as the ToF reference; M3 split into M3a (still) and M3b
(following by turning). Signal per SPAD, ambient and 2 targets per zone are logged
first and kept only if they help. ST's on-chip motion indicator plugin was
downloaded and checked (identical in ULD 1.2.1, 1.3.0 and 2.0.0) and then not
used: 16 values in 8 × 8 mode, a depth window of ≤ 1.5 m from 40 cm, its own
16-frame reference, useless while turning.

## 5 Oct 2026 (evening): WiFi console (ROBOT_WIFI.md), working on the robot

Daniel wants no USB cable while the robot drives around. PicoA is a Pico 2 W, so
`PICO_BOARD` is now `pico2_w` for every firmware. New: `picoA/app/wifi_console.*`
(joins the WiFi, finds the PC server by its UDP broadcasts, sends the console and
takes keys over TCP, heartbeats both ways, reconnects by itself; a second stdio
output and input next to USB, so `debug_console.c` keeps its `printf`s) with a
host test against a fake WiFi chip and lwIP; `pc/robot/` (Node server and browser
page: status bar, a button per key, the log, a log file per run). Start-up
changed: on USB PicoA does nothing until a key (`n` scans, `w` connects to WiFi);
without USB it tries the WiFi, then scans. The scan no longer starts when a
serial monitor opens. Host tests and the server tests pass.

**On the robot:** worked first time. Powered up without the USB cable, PicoA joined
the WiFi, found the server, did the start-up scan (shown on the page), and took
keys from the page. While setting up, Claude's copy of the template overwrote
Daniel's `wifi_config.h` (Daniel re-entered it); the first rebuild didn't pick up
the new file because the build had never seen it (recompiled; tracked since).

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
