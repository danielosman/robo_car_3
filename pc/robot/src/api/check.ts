// `robot steer … --check`: what drive, rotate and goto would do on the map, each from
// where the one before would end, without moving (doc/CLI_API_PLAN.md). The robot's
// own rules (picoA/app/behaviour.c): a drive or a goto's drive looks along a strip
// pathHalfM to each side and stops stopBeforeM before an obstacle, measured from the
// edge; a goto turns towards the point first and goes at most gotoMaxM. The robot
// doesn't check turns; the check says when an obstacle is inside the circle the
// robot sweeps. After a move that wouldn't end ok, the rest would be dropped.
import type { Pose } from "../robot.ts";
import { MOVE } from "./firmware.ts";
import { shortText, type Command } from "./commands.ts";
import { degText, poseText } from "./answers.ts";
import type { RobotMap } from "./map.ts";

export const CHECKABLE = new Set(["drive", "rotate", "goto"]);

interface Result { ok: boolean; end: Pose; text: string }
const cm = (m: number) => Math.round(m * 100);

// Driving distanceM (+ forward, - back) from p.
function drive(map: RobotMap, p: Pose, distanceM: number): Result {
  const dir = distanceM < 0 ? -1 : 1, dx = dir * Math.cos(p.yaw), dy = dir * Math.sin(p.yaw), want = Math.abs(distanceM);
  const look = MOVE.edgeM + MOVE.stopBeforeM + want, along = map.obstacleAlong(p.x, p.y, dx, dy, MOVE.pathHalfM, look);
  const gap = along - MOVE.edgeM, blocked = along < look; // nothing found: along is look itself
  const room = !blocked ? want : gap <= MOVE.stopBeforeM + 0.01 ? 0 : gap - MOVE.stopBeforeM;
  const go = Math.min(want, room), end = { x: p.x + go * dx, y: p.y + go * dy, yaw: p.yaw };
  return blocked && room < want
    ? { ok: false, end, text: `blocked: obstacle after ${cm(room)} cm, would stop ${poseText(end)}` }
    : { ok: true, end, text: `clear: would end ${poseText(end)}` };
}

function rotate(map: RobotMap, p: Pose, deg: number): Result {
  const end = { ...p, yaw: p.yaw + deg * Math.PI / 180 }, r = Math.hypot(MOVE.edgeM, MOVE.halfWidthM);
  return map.obstacleWithin(p.x, p.y, r)
    ? { ok: true, end, text: `turns to yaw ${degText(end.yaw)}; careful: an obstacle within the ${cm(r)} cm the robot sweeps (it doesn't check turns)` }
    : { ok: true, end, text: `clear: turns to yaw ${degText(end.yaw)}` };
}

function goTo(map: RobotMap, p: Pose, x: number, y: number): Result {
  const dist = Math.hypot(x - p.x, y - p.y);
  if (dist < 0.05) return { ok: true, end: p, text: "already there" };
  const bearing = Math.atan2(y - p.y, x - p.x), turn = Math.atan2(Math.sin(bearing - p.yaw), Math.cos(bearing - p.yaw));
  const go = Math.min(dist, MOVE.gotoMaxM), driven = drive(map, { ...p, yaw: p.yaw + turn }, go);
  const far = dist > MOVE.gotoMaxM ? ` (${MOVE.gotoMaxM} m towards it; the goal is ${dist.toFixed(1)} m away)` : "";
  return { ...driven, text: `turn ${degText(turn)} to yaw ${degText(bearing)}, then drive ${cm(go)} cm${far}; ${driven.text}` };
}

// One line per command, "check: <command> <what it would do>"; only drive, rotate and goto.
export function checkSteer(commands: Command[], map: RobotMap, start: Pose): string[] {
  let p = start, dropped = false;
  return commands.map(c => {
    const label = `check: ${shortText(c)}`;
    if (dropped) return `${label} would be dropped`;
    const [a, b] = c.values;
    const r = c.name === "drive" ? drive(map, p, a / 100) : c.name === "rotate" ? rotate(map, p, a) : goTo(map, p, a / 100, b / 100);
    p = r.end;
    dropped = !r.ok;
    return `${label} ${r.text}`;
  });
}
