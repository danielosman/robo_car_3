// Numbers the API shares with the robot's firmware, read from its C headers when the
// server starts, so the two can't drift apart: the commands' limits (commands.h),
// how a move keeps clear of obstacles (behaviour.h), the speeds (motion.h) and the
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
                                "COMMAND_WATCH_MIN_S", "COMMAND_WATCH_MAX_S"]);
const b = header("app/behaviour.h", ["STOP_BEFORE_M", "EDGE_M", "HALF_WIDTH_M", "PATH_HALF_M", "GOTO_MAX_M"]);
const m = header("app/motion.h", ["MOTION_SPEED_MPS", "MOTION_TURN_RADPS"]);
const cam = header("drivers/camera.h", ["CAMERA_WIDTH", "CAMERA_HEIGHT"]);

// What the robot takes (cm, degrees, seconds).
export const LIMITS = {
  driveCm: c.COMMAND_DRIVE_MAX_CM, rotateDeg: c.COMMAND_ROTATE_MAX_DEG, gotoCm: c.COMMAND_GOTO_MAX_CM,
  watchMinS: c.COMMAND_WATCH_MIN_S, watchMaxS: c.COMMAND_WATCH_MAX_S,
} as const;

// A move stops stopBeforeM before an obstacle on the map, measured from the front or
// back edge (edgeM from the centre), looking along a strip pathHalfM to each side; a
// goto goes at most gotoMaxM. The robot is halfWidthM to each side (m).
export const MOVE = {
  stopBeforeM: b.STOP_BEFORE_M, edgeM: b.EDGE_M, halfWidthM: b.HALF_WIDTH_M, pathHalfM: b.PATH_HALF_M, gotoMaxM: b.GOTO_MAX_M,
} as const;

// Full speed driving (m/s) and turning (rad/s).
export const SPEED = { driveMps: m.MOTION_SPEED_MPS, turnRadps: m.MOTION_TURN_RADPS } as const;

// The camera's picture, px.
export const CAMERA = { width: cam.CAMERA_WIDTH, height: cam.CAMERA_HEIGHT } as const;
