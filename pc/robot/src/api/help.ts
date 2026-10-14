// The help the CLI prints and the API serves (GET /api/robot/help[/<name>]): written
// so that an agent can steer the robot from it alone.
import { COMMANDS } from "./commands.ts";
import { READS } from "./reads.ts";

const FRAME = `The map frame: x forward from where the robot was at power-up, y to its left, in cm;
yaw in degrees, + = turned left. "at (43, -4) cm, yaw +30°" is where the robot is and which way it faces.`;

const STEER = `robot steer <commands> [--check] [--no-wait]

Steers the robot: the commands run one after the other, after whatever is queued
before them (the web page's too). Spaces are optional; long names work too.
  robot steer r30d50            rotate 30° left, then drive 50 cm
  robot steer rotate 30 drive 50
  robot steer g120,40           go to (120, 40) cm on the map
  robot steer x                 stop: everything, at once

${COMMANDS.map(c => {
  const values = c.values.map(v => (v.kind === "number" ? `${v.min}..${v.max} ${v.unit}` : v.words.join("|"))).join(", ");
  return `  ${c.letter.padEnd(2)} ${(c.name + (values ? ` <${values}>` : "")).padEnd(32)} ${c.does}`;
}).join("\n")}

Every number is required. After every move (d, r, g, w) a snapshot (s) is taken by
itself, so each move's result can be read at once.

Answers, one line per command, as each ends:
  queued:  r30 s d50 s
  done:    r30 ok, at (12, -4) cm, yaw +30°
  done:    s snapshot 41, at (12, -4) cm, yaw +30°
  done:    d50 obstacle after 31 cm, at (43, -4) cm, yaw +30°
  dropped: s
A move ends ok, or: obstacle after N cm, stopped, safety stop: tilt | wheels,
PicoB lost, didn't start. Anything that doesn't end ok (or an error: robot not
connected, no answer, refused) empties the queue: what still waited is "dropped",
after the snapshot of that move, which shows where the robot ended up. Look at it
(robot snap, robot map) before going on.

--check   moves nothing: says what each d, r, g would do on the newest snapshot's map,
          each from where the one before would end:
            check: d80 clear: would end at (92, -4) cm, yaw +30°
            check: g120,40 turn +34° to yaw +64°, then drive 126 cm; blocked: obstacle after 81 cm, would stop at (52, 31) cm
--no-wait returns after "queued"; the answers are in robot log. Ctrl-C leaves the
          commands queued; robot steer x stops them.

${FRAME}`;

function general(): string {
  const reads = Object.values(READS).map(r => `  robot ${r.usage.padEnd(22)} ${r.help.split("\n")[0]}`);
  return `robot: steer the RoboCar and read what it saw (the robot server at ROBOT_API,
default http://127.0.0.1:8080).

  robot steer <commands>         steer it (robot steer --help: the commands)
${reads.join("\n")}

  robot <name> --help            more about one
  --json                         the data as JSON instead of text (the reads)

n is a snapshot's number (the newest boot's); without it, the newest snapshot. A
snapshot is taken after every move. Pictures are answered with their file's path.

${STEER}`;
}

// The help for one command (steer, or a read), or everything; null for an unknown name.
export function help(name?: string): string | null {
  if (name === undefined) return general();
  if (name === "steer") return STEER;
  const r = READS[name];
  return r ? `robot ${r.usage}\n\n${r.help}${/^(snap|map)$/.test(name) ? `\n\n${FRAME}` : ""}` : null;
}
