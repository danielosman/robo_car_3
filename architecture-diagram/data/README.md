# Architecture data

The robot's modules and the data that flows between them, with links into the code.
This folder is all an agent needs to understand a comment on the diagram ("look at
flow `tilt-stop`"), or to add to it: find the flow here, then follow its links into
the code. Never read `../view/`: it only draws this data.

## Files

- `schema.ts`: the shape of the data, each field with a one-line comment; also the
  origins, in the order they are drawn. The one place the shape is written down.
- `modules/<origin>.json`: the modules of one origin, with their public members.
- `flows.json`: the data flows.

This README holds the words, the rules a field's comment can't say, the examples,
the check and how to fill the data.

## Words

| Word | Means |
|---|---|
| **module** | C: a `.h` file and its `.c`, if any (`picoB/app/odometry.h`). TS: one `.ts` file. Browser: one `.js` file. One module, one box. |
| **member** | a module's public functions and types. C: what the `.h` declares; TS: what the file exports, and a class's public methods (`RobotLink.sendCommand`); the HTTP routes and the WebSocket a TS file serves (kind `route`). Browser JS has no exports: its members are the top-level functions that talk to the server. A constant is a member only when a flow or a rate names it. |
| **origin** | the area a module is drawn in: a program, or a part of one with its own interface. `picoB`, `common` (C in `common/`, built into both Picos), `picoA`, `pc` (the server, `pc/robot/src` outside `api/`), `api` (the command API in the server, `pc/robot/src/api`, reached over HTTP), `browser` (the viewer, `pc/robot/public`). |
| **DTO** | the type of the data on a flow: a public type of some module (`odometry_t`). A message's DTO is its payload type; the message constant (`MESSAGE_LOG`) only when the payload has no type. |
| **flow** | one piece of data's path through modules, as a list of steps. |
| **change node** | a step where the data changes. Kinds: **pick** (keeps some fields), **turn** (changes the values or the type: a filter, a new clock, packing), **add** (joins other data in). |

Boxes are interfaces; nodes are changes. Every change is its own node: five changes
inside one module are five nodes on that module's box. A module with several headers
is several boxes: watch is six (`watch`, `watch_tof`, `watch_camera`, `watch_observation`,
`tracker`, `change_grid`).

## In scope

- The robot firmware (`picoA/app`, `picoB/app`, their `drivers/`, `common/`) and the
  robot server and its page (`pc/robot`): the viewer at `/` (`app.js`).
- `common/` holds the PicoA–PicoB link (`link.h`, `link_messages.h`), the mounting, and
  helpers (`clock_start.h`, `crc32.h`, `geometry.h`, `stamp.h`, `statistics.h`, `units.h`).
- Headers with only types (`tof_frame.h`, `watch_observation.h`) are boxes too.
- `main.ts` is a module (it serves the pages, the API and the WebSocket). `main.c` on
  each Pico is not (no header): its pipelines are where data is handed over, and
  steps link to those lines.
- Not: the bring-up firmware, `pc/bringup`, tests and test helpers, `lwipopts.h`,
  `wifi_config*.h`, the camera's register headers (`hm0360_*.h`), the ST library in
  `picoA/drivers/lib`.
- Planned: the modules in DATA_FLOW_PLAN.md section 8 marked "step N" (the PC's cell
  map, `maps/cell_map.ts` with its helper; `maps/line_map/`, a folder, one module),
  and LINE_MAP_PLAN.md.

## References

The code owns every code fact. The data holds names and search strings, never copies
of signatures, comments or line numbers.

- A code reference is `path` (from the repo root) + `code`: a string found exactly
  once in that file. Example: `"void imu_calibrate_gyro("`, because the name alone
  is there twice. To follow one: open the file, search for the string.
- A member's `path` is its module's, unless given.
- **Planned** things (`status: planned`) have no code yet: they give `plan`, a
  reference into a plan (`{ "path": "doc/DATA_FLOW_PLAN.md", "code": "| step 5:" }`),
  instead of `path` + `code`. A `leaving` thing has both.
- What the side bar shows for a reference: the comment lines just above it, then
  the declaration up to its `;` or its closing brace. For a struct, the reference is
  the line that names it (`} odometry_t;`) and the text runs from its `typedef`. For a
  class, only its public method lines.

## A module

```json
{
  "id": "picoB.odometry",
  "name": "odometry",
  "origin": "picoB",
  "path": "picoB/app/odometry.h",
  "members": [
    { "name": "odometry_update", "kind": "function", "code": "void odometry_update(" },
    { "name": "odometry_get", "kind": "function", "code": "const odometry_t *odometry_get(" },
    { "name": "odometry_t", "kind": "type", "code": "} odometry_t;", "bytes": 68 }
  ]
}
```

- `id`: `<origin>.<name>`. Flows name members as `<module id>.<member>`.
- `summary` (optional): one sentence, only where the header's top comment doesn't
  say it.

### Bytes

`bytes` is on each type that a flow carries: the size of one message of it. It sets
the ribbon's width, so two people must get the same number:

- **C type:** its `sizeof` on the Pico (32-bit; padding counts). Get it from a small
  program built with `cc` (as the host tests are), not by head. `odometry_t` is 68,
  `odometry_report_t` 45 (packed).
- **On the wire:** the message's payload. The packed compact ToF frame is 277
  (`TOF_COMPACT_BYTES`); the ToF wire message is 289 (12 B header + 277).
- **Varying size:** the largest the code allows (an imu batch 135, a snapshot part
  1400).
- **TS object:** the wire message it was decoded from; if none, as JSON.
- **JSON** (HTTP, the WebSocket): a typical message's length.
- A pick's ribbon: the kept fields' bytes, on the pick step.

## A flow

Odometry on PicoB, and the tilt stop branching off it, as the code does it today:

```json
[
  {
    "id": "odometry",
    "name": "Odometry",
    "description": "PicoB turns the IMU and the wheel counts into the pose, speeds and tilt, and sends them to PicoA.",
    "steps": [
      { "at": "picoB.imu.imu_read", "dto": "picoB.imu.imu_sample_t",
        "rate": { "text": "416 Hz", "path": "picoB/drivers/imu.c", "code": "write_reg(CTRL2_G, 0x60)" } },
      { "change": "turn", "name": "robot frame", "at": "picoB.odometry.odometry_update",
        "what": "the sensor's axes into the robot's",
        "path": "picoB/app/odometry.c", "code": "to_robot_frame(&s, acceleration_g, gyro_dps);" },
      { "change": "turn", "name": "heading", "at": "picoB.odometry.odometry_update",
        "what": "gyro z, less its bias, into the heading",
        "path": "picoB/app/odometry.c", "code": "update_heading(gyro_dps[2], dt);" },
      { "change": "add", "name": "wheels", "at": "picoB.odometry.odometry_update",
        "what": "the wheel counts (flow wheel-counts) into distance, speed and position",
        "path": "picoB/app/odometry.c", "code": "update_wheels(dt);" },
      { "change": "turn", "name": "tilt", "at": "picoB.odometry.odometry_update",
        "what": "acceleration and gyro into pitch and roll (filter, 0.5 s)", "dto": "picoB.odometry.odometry_t",
        "path": "picoB/app/odometry.c", "code": "update_tilt(acceleration_g, gyro_dps, dt);" },
      { "at": "picoB.odometry.odometry_get" },
      { "change": "turn", "name": "report", "at": "picoB.brain.brain_update",
        "what": "the fields PicoA needs, plus the motors' state", "dto": "common.link_messages.odometry_report_t",
        "rate": { "text": "50 Hz", "path": "picoB/app/brain.c", "code": "next_odometry = make_timeout_time_us(LINK_ODOMETRY_PERIOD_US);" },
        "path": "picoB/app/brain.c", "code": "odometry_report_t r = {" }
    ]
  },
  {
    "id": "tilt-stop",
    "name": "Tilt stop",
    "description": "PicoB switches the motors off when pitch or roll passes 15°.",
    "parent": "odometry",
    "steps": [
      { "at": "picoB.odometry.odometry_get" },
      { "change": "pick", "name": "pitch, roll", "at": "picoB.drive.drive_update",
        "keeps": ["pitch_rad", "roll_rad"], "bytes": 8,
        "rate": { "text": "100 Hz", "path": "picoB/app/main.c", "code": "#define CONTROL_PERIOD_US" },
        "path": "picoB/app/drive.c", "code": "fabsf(odometry->pitch_rad) > MAX_TILT_RAD" }
    ]
  }
]
```

(The odometry flow goes on to PicoA: `link_send`, `body`, `pose`. Left out here.)

- `id`: short, lower case, with dashes: the name used in comments.
- `steps`: in order. A step is a member (`at`) or a change node (`change`).
  - Every step names a member (`at`). A change node's is the member whose code
    makes the change; the node sits on that member's row.
  - A node's `name` is unique within its flow: "node `tilt` in `odometry`".
  - A step's `path` + `code`: for a node, the line of the change; for a member step,
    where the data is handed over, when that isn't the member's declaration (on
    PicoA most hand-overs are in `main.c`: `tof_pipeline`, `imu_pipeline`).
  - The first step is a member step, not a node.
- `dto`: the type leaving this step; a step without one carries what it got. A step
  with `bytes` and no `dto` carries bytes alone (after a pick, or packed bytes).
- Ribbon width is what the code hands over: a function that gets a whole struct
  and uses two fields gets a fat ribbon; the pick's node sits on its row.
- From and to are the first and last step: not written twice.
- `rate`: where the flow starts at a fixed rate, or where a step changes it;
  `path` + `code` point at what sets it. An event can stand for a rate where it
  replaces one ("when the window moves"), so a subflow doesn't show its parent's.
  Left out for flows on demand (a command).
- `parent`: the flow this one branches off. Its first step names a member the
  parent passes (a member step or a node's `at`), and carries what the parent
  carries after its last step there; a pick's `keeps` are fields of that type.
- `add`: the data that joins is its own flow, ending at the node's member.
- `status` and `plan`: on a whole flow only. When the plan changes a flow, there are
  two: the built one (`leaving`) and the planned one (`planned`).
- Where the code and a plan disagree, the built flow follows the code.

## The check

`npm run resolve` (in `architecture-diagram/`; also run by `tools/lint.sh`) fails
when:

- a file doesn't exist, or a `code` string is found 0 times or more than once;
- a JSON object has a key `schema.ts` doesn't know (a typo like `"dtos"`), or an
  origin, kind, change or status not in its lists;
- a flow names a module, member or DTO that isn't here, or a step has no `at`;
- a subflow's first step isn't on its parent; a top-level flow's first step carries
  no type or bytes;
- a pick keeps a field its DTO's declaration doesn't have (comments left out);
- a type a flow carries has no `bytes`;
- a planned or leaving module or flow has no `plan`.

It also writes, for the view, what each step carries (type, bytes, rate) and where
its code is: the view draws by these and repeats none of the rules.

Not checked: `bytes` against the code, and whether a module lists all its members.

## Findings

A goal of the diagram, not a side effect: whoever fills or reviews the data looks
out for

- **dead or unneeded code:** a member nobody calls, data sent and ignored, a
  message type nobody handles;
- **flow errors:** a wrong or missing rate, a type changed where it shouldn't be, a
  step that skips a check or a clock change, data copied twice;
- **naming inconsistencies:** one thing with two names (C vs TS, robot vs PC, code
  vs plan), or one name for two things (DATA_FLOW_PLAN.md section 5: one name per
  piece of data).

Each finding goes to Daniel with its code reference; nothing is removed or renamed
without his yes. The data shows the code as it is, dead parts included, until the
code changes.

Found so far:

| Date | Finding | Outcome |
|---|---|---|
| 8 Nov | `console.js` sent key presses over the WebSocket; the server ignored them since 14 Oct (`9b338ac`) | removed (Daniel's yes) |
| 8 Nov | the console page at `/console` shows only what the viewer at `/` shows | removed (Daniel's yes) |
| 8 Nov | names: two `Command` types on the PC (`wire.ts`: number and text; `api/commands.ts`: name and values), two `Answer` (`wire.ts`, `api/answers.ts`), two `Pose` (`link.ts` x, y, yaw; `store.ts` with pitch) | fixed: the wire's are `RobotCommand`, `RobotAnswer`; `store.ts`'s `PitchedPose` (`Pose` plus pitch) |
| 8 Nov | the tilt check reads pitch and roll inside `drive_update`, which gets all of `odometry_t` (it uses the speeds too) | none: the diagram shows it as it is |
| 8 Nov | dead: `link_pc_settled` (`link_pc.h`) has no caller; `move_busy` (`move.h`) only tests call | removed (tests ask `move_running()`, "" when none runs) |
| 8 Nov | built into PicoA but used only by host tests: the decoders of robot→PC messages (`wire_decode_hello`, `_console`, `_answer`, `_status`, `_move_start`, `_move_end`, `_imu_batch`) and `wire_encode_command`, `wire_encode_heartbeat`; on the PC `wireCheck` (tests only) | kept: one file owns the bytes both ways, and the linker leaves them out of the firmware (checked with `nm`; `wire.h` says so). The server now runs `wireCheck()` at start, as the robot runs `wire_check()` |
| 8 Nov | names: `rotate` (command word, PC `MoveName`) vs `turn` (move kind, `behaviour_turn`); `snap` (PC) vs `snapshot` (robot); a ToF message is `tof` (wire), `"frame"` (link event), `MoveFrame`, `FrameRow`; an imu batch is `imu_batch` (wire) vs `"imu"` (event); one command has two texts, `commandText` ("steer d50", log and answers) and `robotText` ("drive 50", the robot) | fixed: the move kind is `rotate` everywhere (`MOVE_ROTATE`, `behaviour_rotate`; `behaviour_move` is `behaviour_drive`, its console lines "Drive:"); the robot's word is `snap`; the PC names messages as the wire does (`tof`, `imuBatch` events; `TofMessage`, `TofRow`, `ImuBatchRow`). The two texts stay: one echoes what was typed, one is the robot's words |
| 8 Nov | `odometry_report_t.t_us` is stamped when the report is sent (`brain.c` `send_odometry`), while `link_messages.h` says "measured at t_us": the odometry is from the last IMU sample, up to ~2.4 ms earlier | fixed in the comment: `t_us` is the send time (body.c learns the clocks from it); the ~2.4 ms age is ~0.1° at a fast turn, ignored |
| 8 Nov | IMU records taken before PicoA's greeting are dropped and counted nowhere (now in `body.c`, `MESSAGE_IMU`: `greeted && clock_known`, since step 4f) | none: rare, at start-up |
| 8 Nov | dead data: `odometry_report_t.wheel_left_m`, `wheel_right_m` (8 B of 45, 50 a second) are sent to PicoA, which never reads them | fixed (step 4f): gone from the report and from `odometry_t`; the report is 37 B |
| 8 Nov | one rule in two places: IMU records PicoA can't place yet are held back by PicoB (`greeted`) and dropped by PicoA (`clock_known`) | fixed (step 4f): PicoB sends from its start; `body` keeps a record only when greeted with the same version and its clock is known |
| 8 Nov | data: `house_map` had no flows; the odometry ribbon to `brain` said 416 Hz (brain reads at 50 Hz) | fixed in the data: flows `house-map`, `house-map-back`; the 50 Hz on `odometry_get` |
| 8 Nov | nothing checked that the PC's command words and move kinds match the robot's C (`commands.c`, `move.c` `kind_name`) | fixed: two tests in `pc/robot/src/api/commands.test.ts` read the C and compare |
| 8 Nov | data: the `command` flow first went through `parseAction`; the viewer sends `steer`, which goes through `parseSteer` and `withSnaps` (auto snap) | fixed in the data |

## How to fill it

- By hand or by an agent reading the code, then `npm run resolve` until it is clean.
- **First:** only the modules the first flows pass through, each with all its
  members; then those flows. The other modules after Daniel's first look. Done
  8 Nov: 37 modules (2 planned, 1 leaving), 9 flows.
- **First flows** (done, with `wheel-counts` for odometry's add and `tof-map`,
  `tof-watch` branching off `tof-frame`): `odometry` and `tilt-stop` (above, with the
  part on PicoA); the
  IMU to the PC (`imu_counts_t` → `imu_record_t` → PicoA `body` (PicoA's clock) →
  `move` → imu batch → the store); a ToF frame to the map, watch and the PC; a
  command from the viewer to a move and its answer.
- **Planned parts** (DATA_FLOW_PLAN.md sections 4 and 8): the PC's cell map,
  `maps/line_map/`, `house_map` leaving.
- **After a code change:** `tools/lint.sh` runs the check; fix what it reports.
