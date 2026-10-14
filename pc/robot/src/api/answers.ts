// The answers (doc/CLI_API_PLAN.md): one JSON line per queued command, when it has
// ended, always the same fields:
//   command  as typed: "steer d50", "snap", "recording on"
//   result   ok, obstacle (a move stopped early before an obstacle on the map),
//            limited (a goto further than 3 m went 3 m), stopped (by stop),
//            safety_stop, failed, dropped (never ran: one before it didn't end ok)
//   why      only when result isn't ok, obstacle or limited
//   pose     where the robot is after it (moves, snap, stop): x, y cm, yaw degrees
//   moved, rotated   for a move: cm straight from where it started, degrees turned
//   snapshot for a snap: its number
// Units are cm and degrees, said once in the help.
import type { DoneRecord, Pose } from "../robot.ts";
import { MOVE } from "./firmware.ts";
import { isMove, type Command } from "./commands.ts";

export type Sender = "cli" | "page";
export type Result = "ok" | "obstacle" | "limited" | "stopped" | "safety_stop" | "failed" | "dropped";
export interface PoseOut { x: number; y: number; yaw: number }
export interface Answer {
  command: string; result: Result; why?: string; pose?: PoseOut; moved?: number; rotated?: number; snapshot?: number;
}
// How a command ended, before it is named: the answer without its command.
export type Outcome = Omit<Answer, "command">;

export const failed = (why: string): Outcome => ({ result: "failed", why });

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
const FAILED: Record<string, string> = { picob_lost: "PicoB lost", no_start: "didn't start", replaced: "replaced by another command",
                                         refused: "refused by the robot (robot log says why)" };

export function outcome(c: Command, d: DoneRecord): Outcome {
  const out: Outcome = d.end === "ok" ? { result: c.name === "goto" && (d.goalM ?? 0) > MOVE.gotoMaxM ? "limited" : "ok" }
    : d.end === "obstacle" || d.end === "stopped" ? { result: d.end }
    : d.end in SAFETY ? { result: "safety_stop", why: SAFETY[d.end] }
    : failed(FAILED[d.end] ?? d.end);
  if (d.end === "refused") return out;
  if ((isMove(c) || c.name === "snap" || c.name === "stop") && d.pose) out.pose = poseOut(d.pose);
  if (isMove(c)) { out.moved = Math.round((d.movedM ?? 0) * 100); out.rotated = Math.round(d.rotatedDeg ?? 0); }
  if (c.name === "snap" && d.snapshot !== null) out.snapshot = d.snapshot;
  return out;
}
