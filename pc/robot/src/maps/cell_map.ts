// The PC's cell map (doc/DATA_FLOW_PLAN.md, step 5): the robot's own map code, run in a
// helper program on the Mac (native/cell_map_helper.c), one per boot with that boot's
// mounting. A boot's map is built from its frames in the store, so it is the same
// after a server restart, and "the map at snapshot n" is the map of the frames that
// came before snapshot n. The PC's map is never cleared by the robot's clear_map
// (doc/DATA_FLOW_PLAN.md, decisions).
//   maps.atSnapshot(boot, n, (m, pose) => ...)  the map then and the robot's pose in the
//                                    snapshot: drive (the robot's stop rule, room.h),
//                                    obstacleWithin, view (its cells)
//   maps.viewAtSnapshot(boot, n)     its cells, kept for the last few maps read (the page's timeline)
//   maps.rays(boot, frame)           each zone's reading placed in the world (tof_rays)
// The helper is built (cc) once per server run, from the robot's sources as they are.
import { spawn, execFileSync, type ChildProcessWithoutNullStreams } from "node:child_process";
import { mkdirSync, renameSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import type { Store } from "../store.ts";
import type { Pose } from "../link.ts";
import type { Mounting } from "../mounting.ts";
import { packTof, type PosedFrame } from "../tof_format.ts";

const REPO = fileURLToPath(new URL("../../../../", import.meta.url));
const HELPER = path.join(REPO, "build", "host", "cell_map_helper");
const SOURCES = ["pc/robot/native/cell_map_helper.c", "picoA/app/cell_map.c", "picoA/app/tof_rays.c", "picoA/app/house_map.c",
                 "picoA/app/tof_format.c", "picoA/app/room.c"];
const VIEWS_KEPT = 16; // per boot

export const LAYERS = 3, GRID = 60, TILE_COLS = 20; // picoA/app/cell_map.h, house_map.h
const TILE_BYTES = TILE_COLS * TILE_COLS / 2;
const VERDICTS = ["floor", "obstacle", "unsure"] as const;

// The map's cells as the robot's code holds them (picoA/app/cell_map.h, house_map.h).
export interface CellMapView {
  originX: number; originY: number; cellsX: number; cellsY: number;  // the window, in world cells
  cells: Buffer;                                                      // [layer][ix][iy], one byte a cell
  tiles: { tx: number; ty: number; columns: Buffer }[];              // the whole-house map outside it, 4 bits a column
}
// A zone's reading in the world (picoA/app/tof_rays.h); null: no valid reading, or no pose.
export type ZoneRays = { verdict: (typeof VERDICTS)[number]; point: [number, number, number]; lowM: number; highM: number } | null;

// What robot map, robot check and the page read of a map.
export interface CellMap {
  // A drive of distanceM (+ forward) from (x, y) facing yaw, as the robot's rule would end
  // it on this map (picoA/app/room.h): how far it goes, whether an obstacle stops it short.
  drive(x: number, y: number, yaw: number, distanceM: number): Promise<{ movedM: number; obstacle: boolean }>;
  obstacleWithin(x: number, y: number, r: number): Promise<boolean>; // a blocked or overhang column closer than r
  view(): Promise<CellMapView>;
}

let built = false;
// Builds the helper into a new file and renames it, so tests running at once don't clash.
function buildHelper(): void {
  if (built) return;
  mkdirSync(path.dirname(HELPER), { recursive: true });
  const fresh = `${HELPER}.${process.pid}`;
  execFileSync("cc", ["-std=c11", "-O2", "-Wall", "-Wextra", "-Wdouble-promotion", "-Werror", "-Icommon", "-IpicoA/app",
                      "-o", fresh, ...SOURCES, "-lm"], { cwd: REPO, stdio: ["ignore", "ignore", "pipe"] });
  renameSync(fresh, HELPER);
  built = true;
}

const f32 = (...v: number[]) => { const b = Buffer.alloc(4 * v.length); v.forEach((x, i) => b.writeFloatLE(x, 4 * i)); return b; };

// One helper process: requests go in order, answers come back in the same order.
class Helper implements CellMap {
  private child: ChildProcessWithoutNullStreams;
  private input = Buffer.alloc(0);
  private waiting: { resolve: (b: Buffer) => void; reject: (e: Error) => void }[] = [];
  private failed: Error | null = null;

  constructor(m: Mounting, note: (text: string) => void) {
    buildHelper();
    const t = m.tof; // the helper takes these five in this order (native/cell_map_helper.c)
    this.child = spawn(HELPER, [t.x_m, t.y_m, t.z_m, t.zone_deg, t.zones_turn_deg].map(String));
    this.child.stdout.on("data", (b: Buffer) => this.take(b));
    this.child.stderr.on("data", (b: Buffer) => { for (const line of b.toString().split("\n")) if (line) note(`cell map: ${line}`); });
    this.child.on("error", err => this.fail(err));
    this.child.stdin.on("error", err => this.fail(err)); // the helper gone mid-write: an error for the callers, not a crash
    this.child.on("exit", code => this.fail(new Error(`the cell map helper stopped (${code})`)));
  }

  get dead(): boolean { return this.failed !== null; }

  private take(b: Buffer): void {
    this.input = Buffer.concat([this.input, b]);
    while (this.input.length >= 4 && this.input.length >= 4 + this.input.readUInt32LE(0)) {
      const n = this.input.readUInt32LE(0), answer = Buffer.from(this.input.subarray(4, 4 + n));
      this.input = this.input.subarray(4 + n);
      this.waiting.shift()?.resolve(answer);
    }
  }

  private fail(err: Error): void {
    this.failed ??= err;
    for (const w of this.waiting.splice(0)) w.reject(this.failed);
  }

  private ask(op: string, payload: Uint8Array = new Uint8Array(0)): Promise<Buffer> {
    if (this.failed) return Promise.reject(this.failed);
    return new Promise((resolve, reject) => {
      this.waiting.push({ resolve, reject });
      this.child.stdin.write(Buffer.concat([Buffer.from(op, "latin1"), Buffer.from(payload)]));
    });
  }

  add(frame: PosedFrame): Promise<Buffer> { return this.ask("F", packTof("compact", frame)); }
  clear(): Promise<Buffer> { return this.ask("C"); }

  async drive(x: number, y: number, yaw: number, distanceM: number): Promise<{ movedM: number; obstacle: boolean }> {
    const b = await this.ask("D", f32(x, y, yaw, distanceM)), moved = b.readFloatLE(0), obstacle = b[4] === 1;
    // Not stopped short: the whole distance itself, not its 32-bit copy.
    return { movedM: obstacle ? moved : Math.abs(distanceM), obstacle };
  }

  async obstacleWithin(x: number, y: number, r: number): Promise<boolean> {
    return (await this.ask("W", f32(x, y, r)))[0] === 1;
  }

  async view(): Promise<CellMapView> {
    const b = await this.ask("V"), cellBytes = LAYERS * GRID * GRID, tiles: CellMapView["tiles"] = [];
    for (let i = 0, at = 8 + cellBytes + 2; i < b.readUInt16LE(8 + cellBytes); i++, at += 4 + TILE_BYTES)
      tiles.push({ tx: b.readInt16LE(at), ty: b.readInt16LE(at + 2), columns: Buffer.from(b.subarray(at + 4, at + 4 + TILE_BYTES)) });
    return { originX: b.readInt32LE(0), originY: b.readInt32LE(4), cellsX: GRID, cellsY: GRID,
             cells: Buffer.from(b.subarray(8, 8 + cellBytes)), tiles };
  }

  async rays(frame: PosedFrame): Promise<ZoneRays[]> {
    const b = await this.ask("R", packTof("compact", frame)), out: ZoneRays[] = [];
    for (let i = 0, at = 0; i < 64; i++, at += 21) {
      const v = b[at], f = (k: number) => b.readFloatLE(at + 1 + 4 * k);
      out.push(v === 255 ? null : { verdict: VERDICTS[v], point: [f(0), f(1), f(2)], lowM: f(3), highM: f(4) });
    }
    return out;
  }

  close(): void {
    this.fail(new Error("the cell map helper was closed"));
    this.child.stdin.end();
  }
}

// A boot's helper: fedUpTo, the message number of the last frame it has been given.
interface BootMap { helper: Helper; fedUpTo: number; chain: Promise<unknown>; views: Map<number, CellMapView> }

// Each boot's map, built from the store when it is read. Keeps the last few boots' helpers.
export class CellMaps {
  private store: Store;
  private note: (text: string) => void;
  private keep: number;
  private boots = new Map<string, Promise<BootMap>>();

  constructor(store: Store, note: (text: string) => void = () => {}, keep = 2) {
    this.store = store;
    this.note = note;
    this.keep = keep;
  }

  // Runs fn alone on the boot's map at snapshot `sequence` and the robot's pose in it; null
  // if there is no such snapshot.
  async atSnapshot<T>(boot: string, sequence: number, fn: (m: CellMap, pose: Pose | null) => Promise<T>): Promise<T | null> {
    const s = await this.store.snapshotFrames(boot, sequence);
    return s && this.upTo(boot, s.lastFrame, m => fn(m, s.pose));
  }

  // The map's cells at snapshot `sequence` and the pose in it (as atSnapshot); a map read
  // before is kept (a past map never changes), so moving along the page's timeline is quick.
  async viewAtSnapshot(boot: string, sequence: number): Promise<{ view: CellMapView; pose: Pose | null } | null> {
    const s = await this.store.snapshotFrames(boot, sequence);
    if (!s) return null;
    const b = await this.boot(boot), kept = b.views.get(s.lastFrame);
    const view = kept ?? await this.upTo(boot, s.lastFrame, m => m.view());
    b.views.delete(s.lastFrame);
    b.views.set(s.lastFrame, view);
    for (const old of b.views.keys()) if (b.views.size > VIEWS_KEPT) b.views.delete(old);
    return { view, pose: s.pose };
  }

  // Each zone of a frame of the boot, placed in the world with the boot's mounting.
  async rays(boot: string, frame: PosedFrame): Promise<ZoneRays[]> {
    return (await this.boot(boot)).helper.rays(frame);
  }

  close(): void {
    for (const b of this.boots.values()) void b.then(m => m.helper.close(), () => {});
    this.boots.clear();
  }

  // Runs fn alone on the map of the boot's frames up to message lastFrame: the frames since
  // the last read are added; going back, the map starts again from the first frame.
  private async upTo<T>(boot: string, lastFrame: number, fn: (m: CellMap) => Promise<T>): Promise<T> {
    const b = await this.boot(boot);
    const run = b.chain.then(async () => {
      if (b.fedUpTo > lastFrame) { await b.helper.clear(); b.fedUpTo = 0; }
      const frames = await this.store.framesBetween(boot, b.fedUpTo, lastFrame);
      await Promise.all(frames.map(f => b.helper.add(f.frame)));
      b.fedUpTo = lastFrame;
      return fn(b.helper);
    });
    b.chain = run.catch(() => {});
    return run;
  }

  // The boot's helper (started with its mounting), the newest used last; a dead one is started again.
  private async boot(boot: string): Promise<BootMap> {
    let b = this.boots.get(boot);
    if (b && (await b.catch(() => null))?.helper.dead !== false) b = undefined;
    this.boots.delete(boot);
    b ??= this.store.mountingOf(boot).then(m => {
      if (!m) throw new Error(`no boot ${boot} in the database`);
      return { helper: new Helper(m, this.note), fedUpTo: 0, chain: Promise.resolve(), views: new Map() };
    });
    this.boots.set(boot, b);
    for (const [old, p] of this.boots) {
      if (this.boots.size <= this.keep) break;
      this.boots.delete(old);
      void p.then(m => m.chain.then(() => m.helper.close()), () => {});
    }
    return b;
  }
}
