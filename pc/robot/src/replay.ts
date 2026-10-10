// Replay for the viewer (doc/TELEMETRY_PLAN.md §5): a take's frames as the robot's
// map receives them, run through the robot's own map code (tools/replay_map.c, built
// here from picoA/app/cell_map.c) in both variants, VOTES and READINGS.
import { execFile } from "node:child_process";
import { mkdirSync, statSync, writeFileSync } from "node:fs";
import path from "node:path";
import { promisify } from "node:util";
import { fileURLToPath } from "node:url";
import type { Store } from "./store.ts";

const run = promisify(execFile);
const repo = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../..");
const BUILD = path.join(repo, "build", "replay");
const BINARY = path.join(BUILD, "replay_map");
const SOURCES = ["tools/replay_map.c", "picoA/app/cell_map.c", "picoA/app/cell_map.h", "picoA/app/rangefinder.c",
  "picoA/app/rangefinder.h"].map(f => path.join(repo, f));

export const isTakeId = (id: string) => /^[0-9a-f]{8}-\d+$/.test(id);

// One line per whole frame with a pose: frame number, t (s), x, y, yaw, pitch, then
// per ray (row index × 8 + column index) the closest sure distance in mm (as
// rangefinder.c's closest_sure_mm: 0 nothing in range, 65535 not sure) and the first
// target's status.
export async function exportFrames(store: Store, take: string): Promise<string> {
  if (!isTakeId(take)) throw new Error(`not a take id: ${take}`);
  const frames = await store.query(`SELECT frame_no, t_s, x, y, yaw, pitch FROM tof_frames
    WHERE take_id = '${take}' AND zones = 64 AND x IS NOT NULL ORDER BY frame_no`);
  const zones = await store.query(`SELECT z.frame_no, (z.row - 1) * 8 + (z.col - 1) AS ray, z.targets,
      (SELECT min_by(t.distance_mm, t.target) FROM tof_targets t WHERE t.take_id = z.take_id AND t.frame_no = z.frame_no
         AND t.zone = z.zone AND t.status IN (5, 6, 9) AND t.distance_mm > 0) AS sure_mm,
      (SELECT min_by(t.status, t.target) FROM tof_targets t WHERE t.take_id = z.take_id AND t.frame_no = z.frame_no
         AND t.zone = z.zone) AS first_status
    FROM tof_zones z WHERE z.take_id = '${take}'`);
  const rays = new Map<number, string[]>();
  for (const z of zones) {
    const f = z.frame_no as number;
    if (!rays.has(f)) rays.set(f, Array(64).fill("65535 0"));
    rays.get(f)![z.ray as number] = z.targets === 0 ? "0 0" : z.sure_mm === null ? `65535 ${z.first_status}` : `${z.sure_mm} ${z.first_status}`;
  }
  return frames.filter(f => rays.has(f.frame_no as number))
    .map(f => `${f.frame_no} ${f.t_s} ${f.x} ${f.y} ${f.yaw} ${f.pitch} ${rays.get(f.frame_no as number)!.join(" ")}\n`).join("");
}

const newest = (files: string[]) => Math.max(...files.map(f => statSync(f).mtimeMs));

// Builds the replay tool when it is missing or older than the map code.
async function binary(): Promise<number> {
  mkdirSync(BUILD, { recursive: true });
  let built = 0;
  try { built = statSync(BINARY).mtimeMs; } catch { /* not built yet */ }
  if (built < newest(SOURCES)) {
    await run("cc", ["-std=c11", "-O2", `-I${repo}/picoA/app/test/stubs`, `-I${repo}/common/test/fakes`, `-I${repo}/common`,
      `-I${repo}/picoA/app`, "-o", BINARY, path.join(repo, "tools/replay_map.c"), "-lm"]);
    built = statSync(BINARY).mtimeMs;
  }
  return built;
}

// [x, y, column, ground, L1, L2, confidence %] per cell (tools/replay_map.c --json).
export type Cell = [number, number, number, number, number, number, number];
export interface Maps { frames: number; margin: number; votes: Cell[]; readings: Cell[] }
export const COLUMN = { UNKNOWN: 0, BLOCKED: 1, OVERHANG: 2, NO_FLOOR: 3, DRIVABLE: 4, OPEN: 5 } as const;

const cache = new Map<string, Maps>();

// Both variants' maps of a take; kept until the map code changes.
export async function replayMaps(store: Store, take: string, margin = 0.9): Promise<Maps> {
  const built = await binary();
  const key = `${take} ${margin} ${built}`;
  const kept = cache.get(key);
  if (kept) return kept;
  const file = path.join(BUILD, `${take}.frames`);
  writeFileSync(file, await exportFrames(store, take));
  const { stdout } = await run(BINARY, [file, String(margin), "--json"], { maxBuffer: 64 << 20 });
  const maps = JSON.parse(stdout) as Maps;
  cache.set(key, maps);
  return maps;
}
