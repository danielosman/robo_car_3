// The commands that run at once, never queued (doc/CLI_API_PLAN.md): `robot <name>
// [args]`, each answering one JSON object. status is the robot now; map, tof,
// camera, light and features read a snapshot (its number, or the newest); log the
// log; config the API's settings (and sets one). Units: cm, degrees, pixels, g, dBm,
// lux, said once in the help.
import { mkdir, writeFile } from "node:fs/promises";
import path from "node:path";
import type { Store } from "../store.ts";
import type { LiveRobot } from "../live.ts";
import type { Log } from "../log.ts";
import { snapshotDetail, snapshotMap } from "../snapshots_api.ts";
import { FEATURE } from "../snapshot.ts";
import { CAMERA } from "./firmware.ts";
import { poseOut } from "./answers.ts";
import { displayRgb } from "./colour.ts";
import { MAP_LEGEND, RobotMap } from "./map.ts";
import type { CommandQueue } from "./queue.ts";
import type { StoredSnapshots } from "./runner.ts";

export interface Config { autoSnap: boolean }
export interface ApiContext {
  store: Store; live: LiveRobot; queue: CommandQueue; log: Log; snapshots: StoredSnapshots; config: Config; recordingsDir: string;
}
export interface Read {
  usage: string;  // "map [n] [--png]"
  help: string;   // a few simple sentences
  run(ctx: ApiContext, args: string[], flags: Set<string>): Promise<object>;
}

// A read that can't answer: the message says why; status is the HTTP status.
export class ReadError extends Error {
  readonly status: number;
  constructor(message: string, status = 404) { super(message); this.status = status; }
}

type SnapshotId = { boot: string; seq: number };

export async function findSnapshot(store: Store, arg: string | undefined): Promise<SnapshotId> {
  if (arg !== undefined && !/^\d+$/.test(arg)) throw new ReadError(`${arg} is not a snapshot number`, 400);
  return store.read(async c => {
    const newest = (await c.runAndReadAll("SELECT boot_id, seq FROM snapshots ORDER BY arrived_at DESC, seq DESC LIMIT 1")).getRowObjects()[0];
    if (!newest) throw new ReadError("no snapshots yet (robot snap takes one)");
    const boot = String(newest.boot_id);
    if (arg === undefined) return { boot, seq: Number(newest.seq) };
    const seq = Number(arg);
    const found = (await c.runAndReadAll("SELECT 1 FROM snapshots WHERE boot_id = $1 AND seq = $2", [boot, seq])).getRowObjects();
    if (!found.length) throw new ReadError(`no snapshot ${seq} (the newest is ${newest.seq})`);
    return { boot, seq };
  });
}

export async function mapOf(store: Store, id: SnapshotId): Promise<RobotMap> {
  const view = await snapshotMap(store, id.boot, id.seq);
  if (!view?.window) throw new ReadError(`snapshot ${id.seq} has no map`);
  return new RobotMap(view);
}

async function snapshotRow(store: Store, id: SnapshotId) {
  const d = (await snapshotDetail(store, id.boot, id.seq))!;
  return { ...d, row: d.snapshot as Record<string, unknown> };
}

const round = (v: number, digits = 0) => Number(v.toFixed(digits));
const atMostOne = (args: string[], usage: string) => { if (args.length > 1) throw new ReadError(`usage: robot ${usage}`, 400); };

export const READS: Record<string, Read> = {
  status: {
    usage: "status",
    help: "Shows the robot right now: where it is, its mode, recording on or off, the WiFi signal, what runs and\n" +
          "what waits in the queue, and the number of the newest snapshot. safety is the largest jolt (g) and tilt\n" +
          "PicoB measured since power-up.",
    async run({ live, queue, snapshots }) {
      const p = live.pose, q = queue.state();
      return {
        connected: live.connected, pose: p && poseOut(p), mode: p?.mode ?? null, recording: p?.recording ?? null,
        wifi: p?.rssi ?? null, running: q.running, waiting: q.waiting, newest_snapshot: snapshots.newest(),
        safety: { jolt: { x: live.peaks.jolt[0], y: live.peaks.jolt[1], z: live.peaks.jolt[2] }, tilt: round(live.peaks.tilt * 180 / Math.PI) },
      };
    },
  },
  map: {
    usage: "map [n] [--png]",
    help: "Shows the map at snapshot n (default: the newest): the 6 × 6 m around the robot, one character per\n" +
          "10 cm. The top row is the largest x (forward from where the robot started), the left column the largest\n" +
          "y (left). x_top and y_left say where the top-left cell is. --png writes the same as a picture and gives\n" +
          "its file.",
    async run({ store, recordingsDir }, args, flags) {
      atMostOne(args, this.usage);
      const id = await findSnapshot(store, args[0]), map = await mapOf(store, id);
      if (flags.has("png")) {
        const file = path.join(recordingsDir, "map", id.boot, `${id.seq}.png`);
        await mkdir(path.dirname(file), { recursive: true });
        await writeFile(file, map.png()!);
        return { snapshot: id.seq, file };
      }
      const g = map.grid()!, r = map.view.robot;
      return { snapshot: id.seq, pose: r && poseOut(r), x_top: g.xTop, y_left: g.yLeft, rows: g.rows, legend: MAP_LEGEND };
    },
  },
  tof: {
    usage: "tof [n]",
    help: "Shows the ToF sensor's frame at snapshot n: 8 × 8 zones, as the robot sees them. The top row looks\n" +
          "highest; the bottom rows see the floor 20-50 cm ahead. The left column looks left. 45° across.\n" +
          "One matrix per reading: distance (cm, the nearest target), status (5, 6 and 9 mean sure), signal\n" +
          "(its strength), targets (how many the zone saw), distance_2 (cm, the second target). null: none.",
    async run({ store }, args) {
      atMostOne(args, this.usage);
      const id = await findSnapshot(store, args[0]), d = await snapshotRow(store, id);
      const matrix = (fill: number | null) => Array.from({ length: 8 }, () => Array<number | null>(8).fill(fill));
      const m = { distance: matrix(null), status: matrix(null), signal: matrix(null), targets: matrix(0), distance_2: matrix(null) };
      for (const t of d.targets) {
        const r = Number(t.row) - 1, c = Number(t.col) - 1, k = Number(t.target);
        if (r < 0 || r > 7 || c < 0 || c > 7) continue;
        m.targets[r][c] = (m.targets[r][c] ?? 0) + 1;
        if (k === 0) { m.distance[r][c] = Math.round(Number(t.distance_mm) / 10); m.status[r][c] = Number(t.status); m.signal[r][c] = Number(t.signal); }
        if (k === 1) m.distance_2[r][c] = Math.round(Number(t.distance_mm) / 10);
      }
      return { snapshot: id.seq, ...m };
    },
  },
  camera: {
    usage: "camera [n]",
    help: `Gives the file of the camera picture at snapshot n: a grey PNG, ${CAMERA.width} × ${CAMERA.height}, as the robot sees it.`,
    async run({ store, recordingsDir }, args) {
      atMostOne(args, this.usage);
      const id = await findSnapshot(store, args[0]), { row } = await snapshotRow(store, id);
      if (!row.image) throw new ReadError(`snapshot ${id.seq} has no picture`);
      return { snapshot: id.seq, file: path.join(recordingsDir, String(row.image)) };
    },
  },
  light: {
    usage: "light [n]",
    help: "Shows the colour sensor's reading at snapshot n: lux (how bright), cie (the colour as CIE 1931 x, y),\n" +
          "and rgb (that colour on a screen, at a fixed brightness).",
    async run({ store }, args) {
      atMostOne(args, this.usage);
      const id = await findSnapshot(store, args[0]), { row } = await snapshotRow(store, id);
      if (row.lux === null) throw new ReadError(`snapshot ${id.seq} has no colour reading`);
      return { snapshot: id.seq, lux: round(Number(row.lux), 1), cie: { x: round(Number(row.cie_x), 4), y: round(Number(row.cie_y), 4) },
               rgb: displayRgb(Number(row.color_x), Number(row.color_y), Number(row.color_z)) };
    },
  },
  features: {
    usage: "features [n]",
    help: `Lists the camera features (corners the robot tracks) at snapshot n: x, y in the ${CAMERA.width} × ${CAMERA.height}\n` +
          "picture (pixels from the top left), and state: new, tracked, missed (not found in this picture) or lost.",
    async run({ store }, args) {
      atMostOne(args, this.usage);
      const id = await findSnapshot(store, args[0]), d = await snapshotRow(store, id);
      const state = (flags: number, age: number) => flags & FEATURE.LOST ? "lost" : flags & FEATURE.MISSED ? "missed"
        : flags & FEATURE.NEW || age === 0 ? "new" : "tracked";
      return { snapshot: id.seq,
               features: d.features.map(f => ({ x: round(Number(f.x), 1), y: round(Number(f.y), 1), state: state(Number(f.flags), Number(f.age)) })) };
    },
  },
  log: {
    usage: "log [lines]",
    help: "Shows the last lines of the log (default 20). The log has the robot's own text, the server's notes, and\n" +
          "every answer, with who sent the command ([cli] or [page]).",
    async run({ log }, args) {
      atMostOne(args, this.usage);
      if (args[0] !== undefined && !/^\d+$/.test(args[0])) throw new ReadError(`${args[0]} is not a number of lines`, 400);
      return { lines: log.tail(Math.min(1000, Number(args[0] ?? 20))) };
    },
  },
  config: {
    usage: "config [auto_snap on|off]",
    help: "Shows the API's settings, or changes one. auto_snap on (the default): a snap runs after every move.",
    async run({ config }, args) {
      if (args.length) {
        if (args.length !== 2 || args[0] !== "auto_snap" || !/^(on|off)$/.test(args[1])) throw new ReadError("usage: robot config auto_snap on|off", 400);
        config.autoSnap = args[1] === "on";
      }
      return { auto_snap: config.autoSnap };
    },
  },
};
