// Numbers the API shares with the robot's firmware, read from its C headers when the
// server starts, so the two can't drift apart: the commands' limits (commands.h),
// the robot's size and goto's limits (room.h, behaviour.h), the speeds (motion.h) and the
// camera's picture (drivers/camera.h). Only plain-number #defines are read; a
// missing one stops the server.
import { readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const PICOA = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../../../picoA");

// The plain-number #defines of a header: "#define NAME 0.15f // comment".
export function readDefines(text: string): Map<string, number> {
  const out = new Map<string, number>();
  for (const m of text.matchAll(/^#define\s+(\w+)\s+(-?\d+(?:\.\d+)?)f?\b/gm)) out.set(m[1], Number(m[2]));
  return out;
}

function header(file: string, names: readonly string[]): Record<string, number> {
  const defines = readDefines(readFileSync(path.join(PICOA, file), "utf8"));
  return Object.fromEntries(names.map(n => {
    const v = defines.get(n);
    if (v === undefined) throw new Error(`picoA/${file}: no plain-number #define ${n}`);
    return [n, v];
  }));
}

const c = header("app/commands.h", ["COMMAND_DRIVE_MAX_CM", "COMMAND_ROTATE_MAX_DEG", "COMMAND_GOTO_MAX_CM",
                                "COMMAND_WATCH_MIN_S", "COMMAND_WATCH_MAX_S", "COMMAND_STILL_MAX_MS"]);
const r = header("app/room.h", ["EDGE_M", "HALF_WIDTH_M"]);
const b = header("app/behaviour.h", ["GOTO_MAX_M", "GOTO_THERE_M"]);
const m = header("app/motion.h", ["MOTION_SPEED_MPS", "MOTION_TURN_RADPS"]);
const cam = header("drivers/camera.h", ["CAMERA_WIDTH", "CAMERA_HEIGHT"]);

// What the robot takes (cm, degrees, seconds; the still times in ms).
export const LIMITS = {
  driveCm: c.COMMAND_DRIVE_MAX_CM, rotateDeg: c.COMMAND_ROTATE_MAX_DEG, gotoCm: c.COMMAND_GOTO_MAX_CM,
  watchMinS: c.COMMAND_WATCH_MIN_S, watchMaxS: c.COMMAND_WATCH_MAX_S, stillMaxMs: c.COMMAND_STILL_MAX_MS,
} as const;

// The robot's front and back edges are edgeM from its centre, its sides halfWidthM; a goto
// goes at most gotoMaxM, and not at all to a point closer than gotoThereM (m). Where a
// move stops before an obstacle is the robot's own rule, asked through the helper.
export const MOVE = {
  edgeM: r.EDGE_M, halfWidthM: r.HALF_WIDTH_M, gotoMaxM: b.GOTO_MAX_M, gotoThereM: b.GOTO_THERE_M,
} as const;

// Full speed driving (m/s) and turning (rad/s).
export const SPEED = { driveMps: m.MOTION_SPEED_MPS, turnRadps: m.MOTION_TURN_RADPS } as const;

// The camera's picture, px.
export const CAMERA = { width: cam.CAMERA_WIDTH, height: cam.CAMERA_HEIGHT } as const;
