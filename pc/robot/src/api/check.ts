// `robot check <moves>`: what the moves would do on the PC's cell map at the newest
// snapshot, each from where the one before would end, without moving (doc/CLI_API_PLAN.md).
// Each is answered as the move itself would be. A drive, and a goto's drive, ends as the
// robot's own rule ends it (picoA/app/room.h, asked through maps/cell_map.ts); a goto
// turns towards the point first and goes at most gotoMaxM (behaviour.h). The robot
// doesn't check turns: a turn with an obstacle inside the circle it sweeps gets a
// warning. After a move that wouldn't end ok, the rest would be dropped. watch can't be
// checked.
import type { Pose } from "../link.ts";
import { MOVE } from "./firmware.ts";
import { commandText, type Command } from "./commands.ts";
import { poseOut, type Answer } from "./answers.ts";
import type { CellMap } from "../maps/cell_map.ts";

export type CheckAnswer = Answer & { warning?: string };
interface Moved { end: Pose; result: "ok" | "obstacle"; moved: number }

// Driving distanceM (+ forward, - back) from p.
async function drive(map: CellMap, p: Pose, distanceM: number): Promise<Moved> {
  const direction = distanceM < 0 ? -1 : 1, d = await map.drive(p.x, p.y, p.yaw, distanceM);
  const end = { x: p.x + direction * d.movedM * Math.cos(p.yaw), y: p.y + direction * d.movedM * Math.sin(p.yaw), yaw: p.yaw };
  return { end, result: d.obstacle ? "obstacle" : "ok", moved: d.movedM };
}

async function checkOne(c: Command, map: CellMap, p: Pose): Promise<{ answer: CheckAnswer; end: Pose }> {
  const command = commandText(c), [a, b] = c.values;
  const answer = (result: Answer["result"], end: Pose, moved: number, rotatedRad: number, warning?: string) =>
    ({ answer: { command, result, pose: poseOut(end), moved: Math.round(moved * 100), rotated: Math.round(rotatedRad * 180 / Math.PI), ...(warning ? { warning } : {}) }, end });
  if (c.name === "drive") { const d = await drive(map, p, a / 100); return answer(d.result, d.end, d.moved, 0); }
  if (c.name === "rotate") {
    const end = { ...p, yaw: p.yaw + a * Math.PI / 180 }, r = Math.hypot(MOVE.edgeM, MOVE.halfWidthM);
    const warning = await map.obstacleWithin(p.x, p.y, r) ? `an obstacle within ${Math.round(r * 100)} cm: the robot doesn't check turns` : undefined;
    return answer("ok", end, 0, end.yaw - p.yaw, warning);
  }
  // goto
  const x = a / 100, y = b / 100, dist = Math.hypot(x - p.x, y - p.y);
  if (dist < MOVE.gotoThereM) return answer("ok", p, 0, 0);
  const bearing = Math.atan2(y - p.y, x - p.x), turn = Math.atan2(Math.sin(bearing - p.yaw), Math.cos(bearing - p.yaw));
  const d = await drive(map, { ...p, yaw: p.yaw + turn }, Math.min(dist, MOVE.gotoMaxM));
  return answer(d.result === "ok" && dist > MOVE.gotoMaxM ? "limited" : d.result, d.end, d.moved, turn);
}

// One answer per move; only drive, rotate and goto.
export async function checkMoves(commands: Command[], map: CellMap, start: Pose): Promise<CheckAnswer[]> {
  let p = start, ended: string | null = null;
  const answers: CheckAnswer[] = [];
  for (const c of commands) {
    if (ended) { answers.push({ command: commandText(c), result: "dropped", why: ended }); continue; }
    const { answer, end } = await checkOne(c, map, p);
    p = end;
    if (answer.result !== "ok") ended = `${answer.command} ended: ${answer.result}`;
    answers.push(answer);
  }
  return answers;
}
