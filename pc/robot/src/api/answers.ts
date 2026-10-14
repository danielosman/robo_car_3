// The answers (doc/CLI_API_PLAN.md): one line per command, "<kind>: <command> <how it
// ended>", the command in its short form. How a command ended on the robot (its done
// record) is an outcome; this module puts both into words.
import type { DoneRecord, Pose } from "../robot.ts";
import { MOVE } from "./firmware.ts";
import { isMove, shortText, type Command } from "./commands.ts";

export type Sender = "cli" | "page";
export type AnswerKind = "queued" | "done" | "error" | "dropped";
export interface Answer { kind: AnswerKind; command: string; text: string; sender: Sender }

// How a command ended: done (ok or not) or an error, and its words.
export interface Outcome { kind: "done" | "error"; ok: boolean; text: string }

export const answerLine = (a: Answer) => `${a.kind}: ${a.command}${a.text ? ` ${a.text}` : ""}`;
export const answerFor = (c: Command, o: Outcome, sender: Sender): Answer => ({ kind: o.kind, command: shortText(c), text: o.text, sender });
export const failed = (text: string): Outcome => ({ kind: "error", ok: false, text });

const cm = (m: number) => String(Math.round(m * 100));
// Yaw in (-180, 180], "+30°".
export function degText(rad: number): string {
  let d = Math.round(rad * 180 / Math.PI) % 360;
  if (d > 180) d -= 360;
  if (d <= -180) d += 360;
  return `${d >= 0 ? "+" : ""}${d}°`;
}
export const pointText = (x: number, y: number) => `(${cm(x)}, ${cm(y)}) cm`;
export const poseText = (p: Pose) => `at ${pointText(p.x, p.y)}, yaw ${degText(p.yaw)}`;

// The robot's end words (picoA/app/commands.h).
const ENDS: Record<string, string> = {
  stopped: "stopped", tilt: "safety stop: tilt", wheels: "safety stop: wheels", no_drive: "safety stop: no drive commands",
  safety: "safety stop", picob_lost: "PicoB lost", no_start: "didn't start", replaced: "replaced by another command",
};

export function describe(c: Command, d: DoneRecord): Outcome {
  if (d.end === "refused") return failed("refused by the robot (robot log says why)");
  const ok = d.end === "ok";
  let how = ok ? "ok" : d.end === "obstacle" ? `obstacle after ${cm(d.movedM ?? 0)} cm` : ENDS[d.end] ?? `ended: ${d.end}`;
  if (ok && c.name === "goto" && d.goalM !== null && d.goalM > MOVE.gotoMaxM)
    how = `ok, ${MOVE.gotoMaxM} m towards it (goal ${d.goalM.toFixed(1)} m away)`;
  if (c.name === "snapshot" && d.snapshot !== null) how = `snapshot ${d.snapshot}`;
  const withPose = (isMove(c) || c.name === "snapshot") && d.pose;
  return { kind: "done", ok, text: withPose ? `${how}, ${poseText(d.pose!)}` : how };
}
