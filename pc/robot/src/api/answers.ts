// The answers (doc/CLI_API_PLAN.md): one JSON line per queued command, when it has
// ended, always the same fields:
//   command  as typed: "steer d50", "snap", "mode idle"
//   result   ok, obstacle (a move stopped early before an obstacle on the map),
//            limited (a goto further than 3 m went 3 m), stopped (by stop),
//            safety_stop, failed, dropped (never ran: one before it didn't end ok)
//   why      only when result isn't ok, obstacle or limited; for obstacle only when the stop was
//            floor not seen yet (UNSEEN_FLOOR), not a thing. A command the robot refused says
//            which rule, which value, which limit (refusal), never only "refused"
//   pose     where the robot is after it (moves, snap, stop): x, y cm, yaw degrees
//   moved, rotated   for a move: cm straight from where it started, degrees turned
//   snapshot for a snap: its number
// Units are cm and degrees, said once in the help.
import type { DoneRecord, Pose } from "../link.ts";
import { LIMITS, MOVE } from "./firmware.ts";
import { isMove, MOVES, robotText, type Command } from "./commands.ts";

export type Sender = "cli" | "page";
export type Result = "ok" | "obstacle" | "limited" | "stopped" | "safety_stop" | "failed" | "dropped";
export interface PoseOut { x: number; y: number; yaw: number }
export interface Answer {
  command: string; result: Result; why?: string; pose?: PoseOut; moved?: number; rotated?: number; snapshot?: number;
  printout?: string[]; // report: the robot's status lines
}
// How a command ended, before it is named: the answer without its command.
export type Outcome = Omit<Answer, "command">;

export const failed = (why: string): Outcome => ({ result: "failed", why });

// A drive or goto stopped (or refused) for floor not seen yet, not a thing in the way
// (the robot's end "unseen_floor"; picoA/app/room.h; robot check says it too).
export const UNSEEN_FLOOR = "floor not seen yet: the robot drives only onto floor it has seen within 25 cm of its centre " +
  "(floor first); nothing is in the way. Turning to look (r30, then back) shows it the floor";

// Yaw in degrees, (-180, 180].
export function yawDeg(rad: number): number {
  let d = Math.round(rad * 180 / Math.PI) % 360;
  if (d > 180) d -= 360;
  if (d <= -180) d += 360;
  return d + 0; // not -0
}
export const poseOut = (p: Pose): PoseOut => ({ x: Math.round(p.x * 100) + 0, y: Math.round(p.y * 100) + 0, yaw: yawDeg(p.yaw) });

// The robot's end words (picoA/app/commands.h) that aren't ok, obstacle or stopped.
const SAFETY: Record<string, string> = { tilt: "tilt", wheels: "wheels", no_drive: "no drive commands", safety: "unknown" };
const FAILED: Record<string, string> = { picob_lost: "PicoB lost", no_start: "didn't start", replaced: "replaced by another command" };

// A goto's point (cm, the robot's own frame) against where the robot is (m), as the robot's
// out_of_range says it (the rule is the robot's: picoA/app/commands.c).
const gotoFar = (point: { x: number; y: number }, robot: { x: number; y: number }) =>
  `the point is ${(point.x - robot.x * 100).toFixed(1)} cm in x and ${(point.y - robot.y * 100).toFixed(1)} cm in y from ` +
  `the robot (at ${Math.round(robot.x * 100)}, ${Math.round(robot.y * 100)} cm); a goto reaches at most ${LIMITS.gotoCm} cm in each`;

// Why the robot refused a command (its end words, picoA/app/wire.h): the rule, the value,
// the limit. text: the command as the robot got it ("goto 320 -171", "correct 5 0 120").
export function refusal(text: string, d: DoneRecord): string {
  const [name, ...values] = text.split(" "), spec = MOVES.find(m => m.name === name);
  switch (d.end) {
    case "out_of_range":
      if (name === "goto" && d.pose) return `refused by the robot: "${text}": ${gotoFar({ x: Number(values[0]), y: Number(values[1]) }, d.pose)}`;
      return `refused by the robot: "${text}": a number out of its range` +
             (spec ? ` (${spec.values.map(v => `${v.min} to ${v.max} ${v.unit}`).join(", ")})` : "");
    case "busy_move": return `refused by the robot: "${text}" waits until a move has ended`;
    case "busy_snapshot": return `refused by the robot: "${text}" waits until the snapshot asked for has gone out`;
    case "busy_bulk": return name === "map" ? "refused by the robot: its bulk lane is full (frames or a snapshot going out); ask again"
                                            : `refused by the robot: "${text}" waits until frames, a snapshot or the map have gone out`;
    default: return `refused by the robot: "${text}" is not a command it knows (its firmware older than this server?)`;
  }
}
const REFUSALS = new Set(["refused", "out_of_range", "busy_move", "busy_snapshot", "busy_bulk"]);

export function outcome(c: Command, d: DoneRecord): Outcome {
  const out: Outcome = d.end === "ok" ? { result: c.name === "goto" && (d.goalM ?? 0) > MOVE.gotoMaxM ? "limited" : "ok" }
    : d.end === "obstacle" || d.end === "stopped" ? { result: d.end }
    : d.end === "unseen_floor" ? { result: "obstacle", why: UNSEEN_FLOOR }
    : d.end in SAFETY ? { result: "safety_stop", why: SAFETY[d.end] }
    : REFUSALS.has(d.end) ? failed(refusal(robotText(c), d))
    : failed(FAILED[d.end] ?? d.end);
  if (REFUSALS.has(d.end)) return out;
  if ((isMove(c) || c.name === "snap" || c.name === "stop") && d.pose) out.pose = poseOut(d.pose);
  if (isMove(c)) { out.moved = Math.round((d.movedM ?? 0) * 100); out.rotated = Math.round(d.rotatedDeg ?? 0); }
  if (c.name === "snap" && d.snapshot !== null) out.snapshot = d.snapshot;
  return out;
}
