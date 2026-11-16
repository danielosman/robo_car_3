// Scores the PC's pose correction (doc/OUTLINE_MAP_PLAN.md, "Options: more data, then the
// algorithm"): the stored drives with measured truth replayed through the mapper, and the
// simulator's scenarios at several turn drifts; the end error of the corrected pose and of
// odometry. A change to the pose check is kept only if it lowers the corrected error.
//   npm run score [real|sim]
// The real drives' frames and sights are read from the store (ROBOT_DB) once, with the
// helper, and kept in recordings/score/<boot>.json (a replay then takes seconds).
import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { Mapper } from "../maps/outline/mapper.ts";
import { sightsOf, type Sight } from "../maps/outline/sights.ts";
import type { Pose } from "../maps/outline/pose.ts";
import { CellMaps } from "../maps/cell_map.ts";
import { DB_PATH, Store } from "../store.ts";
import { SCENARIOS, run } from "./scenarios.ts";
import { MEASURED_DRIFT } from "./drive.ts";

const DEG = Math.PI / 180;
// Daniel's measurements (cm, degrees left; the start pose is 0, 0, 0): at a snapshot's last frame.
const TRUTH = [
  { boot: "0a203718", snapshot: 14, name: "drive 1, after the look round", x: 0, y: 0, yawDeg: 18 },
  { boot: "0a203718", snapshot: 34, name: "drive 1, end", x: 15.5, y: -0.5, yawDeg: 0 },
  { boot: "da005bff", snapshot: 26, name: "drive 2, end of the square", x: 11, y: -14, yawDeg: 20 },
];
const SIM = ["square", "things", "door", "loop", "box-stays", "board-stays"], DRIFTS = [1.03, 1.01, 1.003], SEEDS = 3;

const CACHE = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../recordings/score");
type Frame = { frame: { number: number; still: boolean } & Pose; sights: Sight[] };

const err = (p: Pose, t: Pose) => ({ cm: Math.hypot(p.x - t.x, p.y - t.y) * 100, deg: Math.abs(Math.atan2(Math.sin(p.yaw - t.yaw), Math.cos(p.yaw - t.yaw))) / DEG });
const show = (e: { cm: number; deg: number }) => `${e.cm.toFixed(1).padStart(5)} cm ${e.deg.toFixed(1).padStart(5)}°`;

// A boot's frames up to its last measured snapshot, and each measured snapshot's last frame number.
async function framesOf(store: Store | null, boot: string): Promise<{ frames: Frame[]; at: Record<number, number> }> {
  const file = path.join(CACHE, `${boot}.json`);
  if (existsSync(file)) return JSON.parse(readFileSync(file, "utf8"));
  if (!store) throw new Error("no store");
  const snapshots = await store.snapshotsUpTo(boot, Math.max(...TRUTH.filter(t => t.boot === boot).map(t => t.snapshot)));
  const mounting = await store.mountingOf(boot);
  if (!mounting) throw new Error(`boot ${boot}: not in the store`);
  const maps = new CellMaps(store), zoneRad = mounting.tof.zone_deg * DEG, frames: Frame[] = [], at: Record<number, number> = {};
  for (const { frame } of await store.framesBetween(boot, 0, snapshots.at(-1)!.lastFrame))
    frames.push({ frame: { number: frame.number, x: frame.x, y: frame.y, yaw: frame.yaw, still: frame.still },
                  sights: sightsOf(frame, await maps.rays(boot, frame), zoneRad).filter(s => s !== null) });
  for (const t of TRUTH.filter(x => x.boot === boot)) {
    const s = snapshots.find(x => x.sequence === t.snapshot);
    if (!s) throw new Error(`boot ${boot}: no snapshot ${t.snapshot}`);
    at[t.snapshot] = (await store.framesBetween(boot, 0, s.lastFrame)).at(-1)!.frame.number;
  }
  mkdirSync(CACHE, { recursive: true });
  writeFileSync(file, JSON.stringify({ frames, at }));
  return { frames, at };
}

async function real(): Promise<void> {
  const boots = [...new Set(TRUTH.map(t => t.boot))];
  const store = boots.every(b => existsSync(path.join(CACHE, `${b}.json`))) ? null : new Store(process.env.ROBOT_DB ?? DB_PATH);
  let corrected = 0, odometry = 0;
  for (const boot of boots) {
    const { frames, at } = await framesOf(store, boot), mapper = new Mapper();
    for (const f of frames) mapper.add(f.frame, f.sights);
    console.log(`boot ${boot}: turn scale learnt ${(mapper.turnScale()! * 100).toFixed(2)} %`);
    for (const t of TRUTH.filter(x => x.boot === boot)) {
      const p = mapper.poses().filter(q => q.frame <= at[t.snapshot]).at(-1)!, truth = { x: t.x / 100, y: t.y / 100, yaw: t.yawDeg * DEG };
      const c = err(p.pose, truth), o = err(p.odometry, truth);
      corrected += c.cm + c.deg; odometry += o.cm + o.deg;
      console.log(`${t.name.padEnd(30)} odometry ${show(o)}   corrected ${show(c)}`);
    }
  }
  console.log(`real drives, cm + degrees summed: corrected ${corrected.toFixed(1)}, odometry ${odometry.toFixed(1)}`);
}

function sim(): void {
  for (const scale of DRIFTS) {
    let corrected = 0, odometry = 0;
    const rows: string[] = [];
    for (const name of SIM) {
      let c = { cm: 0, deg: 0 }, o = { cm: 0, deg: 0 };
      for (let seed = 1; seed <= SEEDS; seed++) {
        const s = { ...SCENARIOS.find(x => x.name === name)!, seed, drift: { ...MEASURED_DRIFT, turnScale: scale } }, mapper = new Mapper();
        let truth: Pose = { x: 0, y: 0, yaw: 0 };
        for (const step of run(s)) for (const f of step.frames) { mapper.add(f.frame, f.sights); truth = f.truth; }
        const p = mapper.poses().at(-1)!, ce = err(p.pose, truth), oe = err(p.odometry, truth);
        c = { cm: c.cm + ce.cm / SEEDS, deg: c.deg + ce.deg / SEEDS }; o = { cm: o.cm + oe.cm / SEEDS, deg: o.deg + oe.deg / SEEDS };
      }
      corrected += c.cm + c.deg; odometry += o.cm + o.deg;
      rows.push(`  ${name.padEnd(12)} odometry ${show(o)}   corrected ${show(c)}`);
    }
    console.log(`simulator, turns ${((scale - 1) * 100).toFixed(1)} % off: corrected ${corrected.toFixed(1)}, odometry ${odometry.toFixed(1)}\n${rows.join("\n")}`);
  }
}

const only = process.argv[2];
if (only !== "sim") await real();
if (only !== "real") sim();
process.exit(0);
