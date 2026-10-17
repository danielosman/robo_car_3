// The help the CLI prints and the API serves (GET /api/robot/help[/<command>]): short
// sentences an agent can steer the robot from.
import { MOVES } from "./commands.ts";
import { READS } from "./reads.ts";
import { OBJECT_COMMANDS } from "../objects/api.ts";

const UNITS = `Units: distances and positions in cm, angles in degrees (+ is left), picture positions in
pixels, jolt in g, WiFi in dBm, light in lux.

The map frame: x points forward from where the robot was at power-up, y to its left.
pose is where the robot is: {"x":51,"y":0,"yaw":30} is 51 cm forward, facing 30° left.`;

const ANSWERS = `Every queued command answers one JSON line when it ends, in order:
  {"command":"steer d50","result":"ok","pose":{"x":51,"y":0,"yaw":0},"moved":50,"rotated":0}
  {"command":"snap","result":"ok","pose":{"x":51,"y":0,"yaw":0},"snapshot":2}
result is one of:
  ok           it did what was asked
  obstacle     a move stopped early, before an obstacle on the map
  limited      a goto further than 3 m went 3 m
  stopped      robot stop ended it
  safety_stop  the robot stopped itself; why says tilt or wheels
  failed       why says what went wrong (a bad number, robot not connected, no answer...)
  dropped      it never ran, because a command before it didn't end ok
After anything that isn't ok, the queue is emptied. A move's own snap still runs first,
so the newest snapshot shows where the robot ended up. Look at it before going on.
moved is cm in a straight line from where the move started; rotated is degrees turned.`;

const STEER = `robot steer <moves> [--no-wait]

Moves the robot. The moves run one after the other, after anything already queued
(also the web page's commands). Spaces between moves are optional.
  robot steer r30d50       turn 30° left, then drive 50 cm
  robot steer g120,40      go to x 120, y 40 on the map
  robot steer d-40 w60     drive 40 cm backwards, then watch for a minute

${MOVES.map(m => {
  const values = m.values.map(v => `${v.min}..${v.max}`).join(",");
  return `  ${(m.letter + values).padEnd(18)} ${m.does}`;
}).join("\n")}

Every move needs its numbers. With auto_snap on (the default, robot config), a snap
runs after each move, with its own answer line.
--no-wait answers only {"queued":[...]} and returns; the answers are in robot log.
Ctrl-C leaves the moves queued; robot stop stops them.

${ANSWERS}`;

const ACTIONS_HELP: Record<string, string> = {
  snap: "robot snap\n\nTakes a snapshot: the camera picture, the ToF frame, the light, the map, the pose. It is saved,\n" +
        "and its number comes back. Read it with robot map, tof, camera, light and features.",
  stop: "robot stop\n\nStops the robot at once: motors off, mode idle. It skips the queue, and everything waiting is\n" +
        "dropped. The move it interrupted answers \"stopped\".",
  clear_map: "robot clear_map\n\nClears the map. The path on the map starts again from here.",
  recording: "robot recording on|off\n\nRecords every action in full (every sensor reading), for looking at later in\n" +
             "the 3D viewer. Off at power-up.",
  mode: "robot mode idle|manual\n\nidle: the robot stands with the motors off. manual: commands steer it (after power-up).\n" +
        "A move in idle switches to manual.",
  check: "robot check <moves>\n\nSays what drive, rotate and goto would do on the newest snapshot's map. Nothing moves.\n" +
         "Each move is checked from where the one before would end. The answers look like steer's.\n" +
         "  robot check d80 r90 g120,40",
};

function general(): string {
  const line = (r: { usage: string; help: string }) => `  robot ${r.usage.padEnd(26)} ${r.help.split(/(?<=\.)\s/)[0]}`;
  const reads = Object.values(READS).map(line), objects = Object.values(OBJECT_COMMANDS).map(line);
  return `robot: steers the RoboCar and reads what it saw. Talks to the robot server
(ROBOT_API, default http://127.0.0.1:8081; npm start in pc/robot/).

Queued, one at a time, in order:
  robot steer <moves>              Moves the robot (robot steer --help).
  robot snap                       Takes a snapshot.
  robot clear_map                  Clears the map.
  robot recording on|off           Records every action in full, or not.
  robot mode idle|manual           Sets the mode.
  robot stop                       Stops at once, skipping the queue; drops what waits.

At once:
  robot check <moves>              Says what moves would do on the map; nothing moves.
${reads.join("\n")}

Objects on the map (the vision model; at once, any snapshot):
${objects.join("\n")}

n is a snapshot's number; without it, the newest snapshot.
robot <command> --help says more about one command.

${UNITS}

${ANSWERS}`;
}

// The help for one command, or everything; null for an unknown name.
export function help(name?: string): string | null {
  if (name === undefined) return general();
  if (name === "steer") return `${STEER}\n\n${UNITS}`;
  if (ACTIONS_HELP[name]) return ACTIONS_HELP[name];
  const r = READS[name] ?? OBJECT_COMMANDS[name];
  return r ? `robot ${r.usage}\n\n${r.help}\n\n${UNITS}` : null;
}
