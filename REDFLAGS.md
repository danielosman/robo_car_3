# Red-flag log

Red flags from *A Philosophy of Software Design*, found while writing the robot
firmware or in the review pass at the end of each milestone (ROBOT_PLAN.md §11).
Each entry says how it was fixed, or why it was accepted.

## M0 — link, drive, odometry

| Module | Red flag | What | Resolution |
|---|---|---|---|
| `link_msgs.h`, PicoA `body`, PicoB `main` | Information leakage | The 250 ms safety timeout, the 50 Hz report rate and the 1 s HELLO period were hard-coded on both Picos; changing one side would silently break the other | **Fixed:** defined once in `link_msgs.h` (`LINK_DRIVE_TIMEOUT_US`, `LINK_ODOM_PERIOD_US`, `LINK_HELLO_PERIOD_US`); PicoA derives its periods from them |
| PicoA `body`, PicoB `main` | Vague name | Both had a `handle()` for incoming messages | **Fixed:** renamed `on_message()`, next to `on_hello()`, `on_drive()` etc. |
| PicoB `odometry` | Nonobvious code | The wheel step relies on the heading step running first (it uses this step's turn); nothing said so | **Fixed:** comment at the call site |
| PicoA `body`, PicoB `main` | Repetition | The HELLO exchange (send, reply unless it's a reply, version check) is written on both sides | **Accepted:** ~10 lines each; the two sides react differently to a mismatch (PicoB locks the motors, PicoA tells you to reflash). Revisit if the exchange grows |
| PicoB `odometry`, `link_msgs.h` | Repetition | `odom_t` (PicoB's estimate) and `odom_report_t` (what PicoB sends) share most fields | **Accepted:** they're different abstractions. `odom_t` has wheel speeds that only `drive` needs; the report adds motor and safety state owned by `main` and `drive`. One struct for both would leak the wire format into the controller |
| PicoB `odometry` | Nonobvious code | The IMU's mounting (which axis is forward) was an assumption | **Fixed:** the M0 test showed X backward, Y right (turned 180° about Z); `to_robot_frame()` and the odometry test now use that |
| PicoA `body` | (test gap, not an APOSD flag) | The greeting, motor re-send and connection logic has no host test yet; PicoB's side and the link do | **Open:** add a test with a fake link in M1, or sooner if the M0 test shows link trouble |
| PicoB `odometry` | (bug found on the robot, test gap) | `encoder_init()` was never called, so the wheels read 0 and the square test drove on at full power. The host tests' fake encoders worked without being started | **Fixed:** odometry starts the encoders; the fake encoders and motors now fail a test if used before their init. Added a stall stop in `drive` (a wheel driven but not turning for 1 s switches the motors off) so a missing encoder or a jammed wheel can't run away again |
| PicoB `drivers/encoder.c`, `drive` | (bug found on the robot) | Both encoder signs were reversed (set to match the motor signs in bring-up, never checked with RPM going forward), so the speed loop ran away at full power. The stall stop didn't catch a wheel turning the wrong way | **Fixed:** both encoder signs flipped; the stall stop became "wheel not following its target for 1 s" (stopped, far too slow or wrong way), with a host test for reversed encoders. Stop texts shortened to fit a LOG message |
