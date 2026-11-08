// The commands that run at once, never queued (doc/CLI_API_PLAN.md): `robot <name>
// [args]`, each answering one JSON object. status is the robot now; map, tof,
// camera, light and features read a snapshot (its number, or the newest); frames a
// move's frames, imu its imu records (text, or --json); log the log; config the settings (auto_snap is
// set here; pre_ms and post_ms go to the robot through the queue: routes.ts). Units:
// cm, degrees, pixels, dBm, lux, said once in the help.
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
import type { Chat } from "../objects/model.ts";
import type { BootFolders } from "../boot_folders.ts";
import { framesJson, framesText, imuJson, imuText } from "./frames.ts";

export interface Config { autoSnap: boolean }
export interface ApiContext {
  store: Store; live: LiveRobot; queue: CommandQueue; log: Log; snapshots: StoredSnapshots; config: Config; recordingsDir: string;
  bootFolders: BootFolders;  // the robot's boot ids to our folder names (map --png)
  chat?: Chat; modelName?: string;  // the vision model for robot detect (objects/)
}
export interface Read {
  usage: string;  // "map [n] [--png]"
  help: string;   // a few simple sentences
  // req: the request as it came, for a command whose argument isn't words (detect_save's
  // JSON) or whose flags carry a value (--by=<name>: query.by)
  run(ctx: ApiContext, args: string[], flags: Set<string>, req?: { body: string; query: Record<string, string> }): Promise<object | TextAnswer>;
}

// An answer that is text, not JSON (robot frames and imu without --json).
export class TextAnswer {
  readonly text: string;
  constructor(text: string) { this.text = text; }
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

// robot frames' and imu's <boot> <move>: the boot's id (from its id or folder name) and folder.
async function moveOf(store: Store, bootFolders: BootFolders, args: string[], usage: string): Promise<{ boot: string; folder: string | null }> {
  if (args.length !== 2 || !/^\d+$/.test(args[1])) throw new ReadError(`usage: robot ${usage}`, 400);
  const boot = /^[0-9a-f]{8}$/.test(args[0]) ? args[0] : bootFolders.bootOfFolder(args[0]);
  if (!boot || !(await store.hasBoot(boot))) throw new ReadError(`no boot ${args[0]} in the database`);
  return { boot, folder: bootFolders.knownFolder(boot) };
}
const atMostOne = (args: string[], usage: string) => { if (args.length > 1) throw new ReadError(`usage: robot ${usage}`, 400); };

export const READS: Record<string, Read> = {
  status: {
    usage: "status",
    help: "Shows the robot right now: where it is, its mode, the move running (moving: drive, rotate, goto, watch),\n" +
          "the WiFi signal, what runs and what waits in the queue, and the number of the newest snapshot.",
    async run({ live, queue, snapshots }) {
      const p = live.pose, q = queue.state();
      return {
        connected: live.connected, boot: live.boot, pose: p && poseOut(p), mode: p?.mode ?? null, moving: p ? p.action || null : null,
        wifi: p?.rssi ?? null, running: q.running, waiting: q.waiting, newest_snapshot: snapshots.newest(),
      };
    },
  },
  map: {
    usage: "map [n] [--png]",
    help: "Shows the map at snapshot n (default: the newest): the 6 × 6 m around the robot, one character per\n" +
          "10 cm. The top row is the largest x (forward from where the robot started), the left column the largest\n" +
          "y (left). x_top and y_left say where the top-left cell is. --png writes the same as a picture and gives\n" +
          "its file.",
    async run({ store, recordingsDir, bootFolders }, args, flags) {
      atMostOne(args, this.usage);
      const id = await findSnapshot(store, args[0]), map = await mapOf(store, id);
      if (flags.has("png")) {
        const file = path.join(recordingsDir, "map", bootFolders.folderForBoot(id.boot), `${id.seq}.png`);
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
          "highest; the bottom rows see the floor ~25-60 cm ahead. The left column looks left. 45° across.\n" +
          "One target per zone. One matrix per reading: distance (cm; 255 means beyond 2.54 m, null nothing in range),\n" +
          "kind (sure, less_sure, unsure: the distance tells nothing, nothing), signal (its strength, capped at 255),\n" +
          "sigma (the distance's noise, mm, capped at 255), targets (how many the sensor saw in the zone).",
    async run({ store }, args) {
      atMostOne(args, this.usage);
      const id = await findSnapshot(store, args[0]), d = await snapshotRow(store, id);
      if (!d.zones.length) throw new ReadError(`snapshot ${id.seq} has no ToF frame`);
      const matrix = <T>(fill: T) => Array.from({ length: 8 }, () => Array<T>(8).fill(fill));
      const m = { distance: matrix<number | null>(null), kind: matrix("nothing"), signal: matrix(0), sigma: matrix(0), targets: matrix(0) };
      for (const z of d.zones) {
        const r = Number(z.row) - 1, c = Number(z.col) - 1, kind = String(z.kind);
        if (r < 0 || r > 7 || c < 0 || c > 7) continue;
        m.distance[r][c] = kind === "nothing" ? null : Math.round(Number(z.distance_mm) / 10);
        m.kind[r][c] = kind;
        m.signal[r][c] = Number(z.signal);
        m.sigma[r][c] = Number(z.sigma_mm);
        m.targets[r][c] = Number(z.targets);
      }
      return { snapshot: id.seq, ...m };
    },
  },
  frames: {
    usage: "frames <boot> <move> [--json]",
    help: "Shows a move's ToF frames as the robot sent them, from the database: the move (kind, how it ended, frames\n" +
          "sent and dropped, its imu records), then each frame's pose and its zones' distance, kind, signal and sigma.\n" +
          "boot: its id (8 hex digits, robot status says it) or its folder name (YYYYMMDDHHMM); move: its number (1 is\n" +
          "the boot's first). --json gives the same for a program: the robot's own units (m, rad, mm), one 8 × 8 matrix\n" +
          "per reading.",
    async run({ store, bootFolders }, args, flags) {
      const { boot, folder } = await moveOf(store, bootFolders, args, this.usage);
      const found = await store.moveFrames(boot, Number(args[1]));
      if (!found) throw new ReadError(`boot ${args[0]} has no move ${args[1]}`);
      return flags.has("json") ? framesJson(boot, folder, found.move, found.frames, found.imu)
        : new TextAnswer(framesText(boot, folder, found.move, found.frames, found.imu));
    },
  },
  imu: {
    usage: "imu <boot> <move> [--json]",
    help: "Shows a move's imu records as the robot sent them, from the database (104 a second during a move): the move,\n" +
          "then each record's time and the IMU's raw counts, accel and gyro x, y, z, in the sensor's own axes. boot and\n" +
          "move as for robot frames. --json gives the same for a program.",
    async run({ store, bootFolders }, args, flags) {
      const { boot, folder } = await moveOf(store, bootFolders, args, this.usage);
      const found = await store.moveImu(boot, Number(args[1]));
      if (!found) throw new ReadError(`boot ${args[0]} has no move ${args[1]}`);
      if (flags.has("json")) return imuJson(boot, folder, found.move, found.imu, found.records);
      return new TextAnswer(imuText(boot, folder, found.move, found.frames, found.imu, found.records));
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
    usage: "config [auto_snap on|off | pre_ms <ms> | post_ms <ms>]",
    help: "Shows the settings, or changes one. auto_snap on (the default): a snap runs after every move.\n" +
          "pre_ms, post_ms: how long the robot stands still before and after each move (0-2000 ms, 200 at power-up);\n" +
          "they go to the robot in the queue, after what waits, and are answered like the other commands. null: not\n" +
          "known yet (the robot says them with each move).",
    async run({ config, live }, args) {
      if (args.length) {
        if (args.length !== 2 || args[0] !== "auto_snap" || !/^(on|off)$/.test(args[1])) throw new ReadError(`usage: robot ${this.usage}`, 400);
        config.autoSnap = args[1] === "on";
      }
      return { auto_snap: config.autoSnap, pre_ms: live.still?.preMs ?? null, post_ms: live.still?.postMs ?? null };
    },
  },
};
