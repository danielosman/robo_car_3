// `robot check <moves>`: what the moves would do on the newest snapshot's map, each
// from where the one before would end, without moving (doc/CLI_API_PLAN.md). Each is
// answered as the move itself would be. The robot's own rules (picoA/app/behaviour.c):
// a drive or a goto's drive looks along a strip pathHalfM to each side and stops
// stopBeforeM before an obstacle, measured from the edge; a goto turns towards the
// point first and goes at most gotoMaxM. The robot doesn't check turns: a turn with an
// obstacle inside the circle it sweeps gets a warning. After a move that wouldn't end
// ok, the rest would be dropped. watch can't be checked.
import type { Pose } from "../robot.ts";
import { MOVE } from "./firmware.ts";
import { commandText, type Command } from "./commands.ts";
import { poseOut, type Answer } from "./answers.ts";
import type { RobotMap } from "./map.ts";

export type CheckAnswer = Answer & { warning?: string };
interface Moved { end: Pose; result: "ok" | "obstacle"; moved: number }

// Driving distanceM (+ forward, - back) from p.
function drive(map: RobotMap, p: Pose, distanceM: number): Moved {
  const dir = distanceM < 0 ? -1 : 1, dx = dir * Math.cos(p.yaw), dy = dir * Math.sin(p.yaw), want = Math.abs(distanceM);
  const look = MOVE.edgeM + MOVE.stopBeforeM + want, along = map.obstacleAlong(p.x, p.y, dx, dy, MOVE.pathHalfM, look);
  const gap = along - MOVE.edgeM, blocked = along < look; // nothing found: along is look itself
  const room = !blocked ? want : gap <= MOVE.stopBeforeM + 0.01 ? 0 : gap - MOVE.stopBeforeM;
  const go = Math.min(want, room);
  return { end: { x: p.x + go * dx, y: p.y + go * dy, yaw: p.yaw }, result: blocked && room < want ? "obstacle" : "ok", moved: go };
}

function checkOne(c: Command, map: RobotMap, p: Pose): { answer: CheckAnswer; end: Pose } {
  const command = commandText(c), [a, b] = c.values;
  const answer = (result: Answer["result"], end: Pose, moved: number, rotatedRad: number, warning?: string) =>
    ({ answer: { command, result, pose: poseOut(end), moved: Math.round(moved * 100), rotated: Math.round(rotatedRad * 180 / Math.PI), ...(warning ? { warning } : {}) }, end });
  if (c.name === "drive") { const d = drive(map, p, a / 100); return answer(d.result, d.end, d.moved, 0); }
  if (c.name === "rotate") {
    const end = { ...p, yaw: p.yaw + a * Math.PI / 180 }, r = Math.hypot(MOVE.edgeM, MOVE.halfWidthM);
    const warning = map.obstacleWithin(p.x, p.y, r) ? `an obstacle within ${Math.round(r * 100)} cm: the robot doesn't check turns` : undefined;
    return answer("ok", end, 0, end.yaw - p.yaw, warning);
  }
  // goto
  const x = a / 100, y = b / 100, dist = Math.hypot(x - p.x, y - p.y);
  if (dist < 0.05) return answer("ok", p, 0, 0);
  const bearing = Math.atan2(y - p.y, x - p.x), turn = Math.atan2(Math.sin(bearing - p.yaw), Math.cos(bearing - p.yaw));
  const d = drive(map, { ...p, yaw: p.yaw + turn }, Math.min(dist, MOVE.gotoMaxM));
  return answer(d.result === "ok" && dist > MOVE.gotoMaxM ? "limited" : d.result, d.end, d.moved, turn);
}

// One answer per move; only drive, rotate and goto.
export function checkMoves(commands: Command[], map: RobotMap, start: Pose): CheckAnswer[] {
  let p = start, ended: string | null = null;
  return commands.map(c => {
    if (ended) return { command: commandText(c), result: "dropped", why: ended };
    const { answer, end } = checkOne(c, map, p);
    p = end;
    if (answer.result !== "ok") ended = `${answer.command} ended: ${answer.result}`;
    return answer;
  });
}
