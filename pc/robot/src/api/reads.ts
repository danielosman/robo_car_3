// The commands that run at once, never queued (doc/CLI_API_PLAN.md): `robot <name>
// [args]`, each answering one JSON object. status is the robot now; map (the PC's cell
// map, or with --robot the robot's own), tof,
// camera, light and features read a snapshot (its number, or the newest); frames a
// move's frames, imu its imu records (text, or --json); log the log; config the settings (auto_snap is
// set here; pre_ms and post_ms go to the robot through the queue: routes.ts). Units:
// cm, degrees, pixels, dBm, lux, said once in the help.
import { mkdir, writeFile } from "node:fs/promises";
import path from "node:path";
import type { Store } from "../store.ts";
import type { LiveRobot } from "../live.ts";
import type { Log } from "../log.ts";
import { cellMapAt, snapshotDetail, type MapView } from "../snapshots_api.ts";
import { FEATURE } from "../snapshot.ts";
import { CAMERA } from "./firmware.ts";
import { poseOut } from "./answers.ts";
import { displayRgb } from "./colour.ts";
import { mapPng } from "./map.ts";
import { lineMapsAt } from "../maps/line_map/index.ts";
import type { CellMaps } from "../maps/cell_map.ts";
import type { CommandQueue } from "./queue.ts";
import type { StoredSnapshots } from "./runner.ts";
import type { Chat } from "../objects/model.ts";
import type { BootFolders } from "../boot_folders.ts";
import { framesJson, framesText, imuJson, imuText } from "./frames.ts";

export interface Config { autoSnap: boolean }
export interface ApiContext {
  store: Store; maps: CellMaps; live: LiveRobot; queue: CommandQueue; log: Log; snapshots: StoredSnapshots; config: Config; recordingsDirectory: string;
  bootFolders: BootFolders;  // the robot's boot ids to our folder names (map --png)
  chat?: Chat; modelName?: string;  // the vision model for robot detect (objects/)
}
export interface Read {
  usage: string;  // "map [n] [--robot] [--png]"
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

type SnapshotId = { boot: string; sequence: number };

export async function findSnapshot(store: Store, arg: string | undefined): Promise<SnapshotId> {
  if (arg !== undefined && !/^\d+$/.test(arg)) throw new ReadError(`${arg} is not a snapshot number`, 400);
  return store.read(async c => {
    const newest = (await c.runAndReadAll("SELECT boot_id, sequence FROM snapshots ORDER BY arrived_at DESC, sequence DESC LIMIT 1")).getRowObjects()[0];
    if (!newest) throw new ReadError("no snapshots yet (robot snap takes one)");
    const boot = String(newest.boot_id);
    if (arg === undefined) return { boot, sequence: Number(newest.sequence) };
    const sequence = Number(arg);
    const found = (await c.runAndReadAll("SELECT 1 FROM snapshots WHERE boot_id = $1 AND sequence = $2", [boot, sequence])).getRowObjects();
    if (!found.length) throw new ReadError(`no snapshot ${sequence} (the newest is ${newest.sequence})`);
    return { boot, sequence };
  });
}

// The PC's cell map at a snapshot.
async function mapAt(ctx: ApiContext, id: SnapshotId): Promise<MapView> {
  const view = await cellMapAt(ctx.store, ctx.maps, id.boot, id.sequence);
  if (!view) throw new ReadError(`snapshot ${id.sequence} has no map`);
  return view;
}

async function snapshotRow(store: Store, id: SnapshotId) {
  const d = (await snapshotDetail(store, id.boot, id.sequence))!;
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

// How to read robot map's answer (picoA/app/cell_map.h, house_map.h).
const MAP_FORMAT = {
  cell: "10 cm; cell (ix, iy) covers x ix*10 to ix*10+10 cm, y iy*10 to iy*10+10 cm, in the map frame",
  window: "the cell at index i of each layer is ix = origin_cell_x + floor(i / cells_y), iy = origin_cell_y + i % cells_y",
  cell_byte: "bits 0-1 the state: 0 unknown, 1 free, 2 occupied; bits 2-3 the doubt; bits 4-7 the confidence, 0-15",
  tile: "20 x 20 cells; tile (tx, ty) starts at cell (tx*20-10, ty*20-10); columns[i] is cell (start_x + floor(i / 20), start_y + i % 20)",
  tile_column: "0 unknown, 1 blocked (L1 occupied), 2 overhang (L2 occupied, L1 not), 3 no floor (G free), 4 drivable (G occupied, L1 and L2 free), 5 open (L1 and L2 free, G unknown)",
};

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
    usage: "map [n] [--robot] [--png [--lines [--no-cells]]]",
    help: "The map at snapshot n (default: the newest): the PC's, built from the ToF frames that came before it by\n" +
          "the robot's own map code. The window (6 × 6 m around the robot, 10 cm cells) as three layers of cell bytes,\n" +
          "G (the floor), L1 (3-13 cm), L2 (13-23 cm), and the whole-house tiles outside it. format says how to read\n" +
          "them. --robot asks the connected robot for its own 6 × 6 m window right now. --png draws the whole map\n" +
          "(or, with --robot, the robot's window) as a picture and gives its file. --png --lines draws the PC's line\n" +
          "maps over it (white: L1, 3-13 cm; yellow: L2, 13-23 cm; cyan: above); with --no-cells, the lines alone.",
    async run(ctx, args, flags) {
      atMostOne(args, this.usage);
      const withLines = flags.has("lines"), withCells = !flags.has("nocells");
      if (withLines && (!flags.has("png") || flags.has("robot"))) throw new ReadError("--lines draws the PC's line maps: use it with --png, without --robot", 400);
      if (!withCells && !withLines) throw new ReadError("--no-cells leaves nothing to draw: use it with --png --lines", 400);
      if (flags.has("robot")) {
        if (args.length) throw new ReadError("--robot asks the robot now (it takes no snapshot number)", 400);
        const asked = await ctx.queue.askMap();
        if ("error" in asked) throw new ReadError(asked.error, 404);
        const w = asked.window, r = asked.pose ?? ctx.live.pose, n = w.cellsX * w.cellsY;
        if (flags.has("png")) {
          const cols = await ctx.maps.columnsOf(w.cells);
          const folder = ctx.live.boot ? ctx.bootFolders.folderForBoot(ctx.live.boot) : "live";
          const file = path.join(ctx.recordingsDirectory, "map", folder, "robot.png");
          await mkdir(path.dirname(file), { recursive: true });
          await writeFile(file, mapPng({ window: { ...w, ...cols }, tiles: [], robot: r, path: [] }));
          return { map: "robot", pose: r && poseOut(r), file };
        }
        const layer = (k: number) => [...w.cells.subarray(k * n, (k + 1) * n)];
        return {
          map: "robot", pose: r && poseOut(r),
          window: { origin_cell_x: w.originX, origin_cell_y: w.originY, cells_x: w.cellsX, cells_y: w.cellsY,
                    G: layer(0), L1: layer(1), L2: layer(2) },
          format: MAP_FORMAT,
        };
      }
      const id = await findSnapshot(ctx.store, args[0]), map = await mapAt(ctx, id);
      if (flags.has("png")) {
        const name = `${id.sequence}${withLines ? (withCells ? "-lines" : "-lines-only") : ""}.png`;
        const file = path.join(ctx.recordingsDirectory, "map", ctx.bootFolders.folderForBoot(id.boot), name);
        const lines = withLines ? await lineMapsAt(ctx.store, ctx.maps, id.boot, id.sequence) ?? [] : [];
        await mkdir(path.dirname(file), { recursive: true });
        await writeFile(file, mapPng(map, { cells: withCells, lines }));
        return { snapshot: id.sequence, map: "pc", file };
      }
      const w = map.window, r = map.robot, n = w.cellsX * w.cellsY;
      const layer = (k: number) => [...w.cells.subarray(k * n, (k + 1) * n)];
      return {
        snapshot: id.sequence, map: "pc", pose: r && poseOut(r),
        window: { origin_cell_x: w.originX, origin_cell_y: w.originY, cells_x: w.cellsX, cells_y: w.cellsY,
                  G: layer(0), L1: layer(1), L2: layer(2) },
        tiles: map.tiles.map(t => ({ tx: t.tx, ty: t.ty, columns: t.columns })),
        format: MAP_FORMAT,
      };
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
      if (!d.zones.length) throw new ReadError(`snapshot ${id.sequence} has no ToF frame`);
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
      return { snapshot: id.sequence, ...m };
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
          "then each record's time and the IMU's raw counts, acceleration and gyro x, y, z, in the sensor's own axes. boot and\n" +
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
    async run({ store, recordingsDirectory }, args) {
      atMostOne(args, this.usage);
      const id = await findSnapshot(store, args[0]), { row } = await snapshotRow(store, id);
      if (!row.image) throw new ReadError(`snapshot ${id.sequence} has no picture`);
      return { snapshot: id.sequence, file: path.join(recordingsDirectory, String(row.image)) };
    },
  },
  light: {
    usage: "light [n]",
    help: "Shows the colour sensor's reading at snapshot n: lux (how bright), cie (the colour as CIE 1931 x, y),\n" +
          "and rgb (that colour on a screen, at a fixed brightness).",
    async run({ store }, args) {
      atMostOne(args, this.usage);
      const id = await findSnapshot(store, args[0]), { row } = await snapshotRow(store, id);
      if (row.lux === null) throw new ReadError(`snapshot ${id.sequence} has no colour reading`);
      return { snapshot: id.sequence, lux: round(Number(row.lux), 1), cie: { x: round(Number(row.cie_x), 4), y: round(Number(row.cie_y), 4) },
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
      return { snapshot: id.sequence,
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
