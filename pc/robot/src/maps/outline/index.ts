// The PC's outline maps (doc/OUTLINE_MAP_PLAN.md): a boot's outlines, walls and corrected
// poses at a snapshot, built from the boot's frames that came before the snapshot, as the
// PC's cell map is (maps/cell_map.ts). Each frame's zones are placed by the cell map's
// helper (the robot's own geometry, with the boot's mounting), turned into sights and given
// to the mapper (mapper.ts).
// One mapper is kept per boot and only goes forward: it is fed the frames up to each
// snapshot in turn, and at each one what the page draws is kept (the outlines, the
// correction). So a new snapshot costs only its own frames, and a past one is read at once.
// A snapshot whose frames differ from those the mapper was fed starts the boot's mapper
// again; that happens only when a snapshot is stored again (the tests do).
// Two boots are kept: reading a third may push out the live one, which is then built
// again from its first frame at the next snapshot (a few seconds).
//   new OutlineMaps(store, cellMaps, note)
//   outlines.at(boot, n)       the map at snapshot n for the page and robot map: each band's outline, the
//                              correction then, and the robot, its path and the snapshots, all placed by
//                              the corrected pose; null if there is no such snapshot
//   outlines.placement(boot, n) each frame's corrected pose up to snapshot n and the correction then,
//                              for the PC's cell map (cell_map.ts placeBy); null if there is no such snapshot
//   outlines.added(boot, n)    snapshot n has come: the map brought up to it; the correction goes in the log
//   outlines.walls(boot, n)    each band's walls at snapshot n; writes the tables walls, wall_sightings and
//                              frame_poses (the boot's rows, replaced)
import type { Store } from "../../store.ts";
import type { CellMaps, Placement } from "../cell_map.ts";
import { corrected, Mapper, type BandOutline, type BandWalls } from "./mapper.ts";
import type { Pose } from "./pose.ts";
import { sightsOf } from "./sights.ts";
import { doubtOf } from "./walls.ts";

export type { BandName } from "./sights.ts";
export type { Edge, Loop } from "./outline_map.ts";
export type { BandOutline, BandWalls };
export { edgeRgb, outlineStrokes, BAND_RGB, type Rgb, type Stroke } from "./colours.ts";

// The map at a snapshot. correction: odometry's pose to the corrected one (mapper.ts corrected()).
export interface OutlineView {
  bands: BandOutline[];
  correction: Pose;
  robot: Pose | null;                                  // the snapshot's pose (null: PicoB not connected)
  path: { x: number; y: number }[];                    // each frame's pose since power-up
  snapshots: { sequence: number; x: number; y: number }[];
}

// What is kept of a snapshot the mapper passed: poses, how many frame poses it had then.
interface Stage { sequence: number; lastFrame: number; poses: number; correction: Pose; robot: Pose | null; bands: BandOutline[] }
interface Track { mapper: Mapper; zoneRad: number; stages: Map<number, Stage>; last: Stage | null }

const NONE: Pose = { x: 0, y: 0, yaw: 0 };
const BOOTS_KEPT = 2;

export class OutlineMaps implements Placement {
  private readonly boots = new Map<string, { track: Track | null; chain: Promise<unknown> }>();

  private readonly store: Store;
  private readonly cellMaps: CellMaps;
  private readonly note: (text: string) => void;

  constructor(store: Store, cellMaps: CellMaps, note: (text: string) => void = () => {}) {
    this.store = store;
    this.cellMaps = cellMaps;
    this.note = note;
  }

  async at(boot: string, sequence: number): Promise<OutlineView | null> {
    return this.alone(boot, async b => {
      const t = await this.upTo(b, boot, sequence), stage = t?.stages.get(sequence);
      if (!t || !stage) return null;
      const stages = [...t.stages.values()].filter(s => s.sequence <= sequence && s.robot);
      return {
        bands: stage.bands, correction: stage.correction, robot: stage.robot,
        path: t.mapper.poses().slice(0, stage.poses).map(p => ({ x: p.pose.x, y: p.pose.y })),
        snapshots: stages.map(s => ({ sequence: s.sequence, x: s.robot!.x, y: s.robot!.y })),
      };
    });
  }

  async placement(boot: string, sequence: number): Promise<{ correction: Pose; poseOf(frame: number): Pose | undefined } | null> {
    return this.alone(boot, async b => {
      const t = await this.upTo(b, boot, sequence), stage = t?.stages.get(sequence);
      if (!t || !stage) return null;
      const poses = new Map(t.mapper.poses().slice(0, stage.poses).map(p => [p.frame, p.pose]));
      return { correction: stage.correction, poseOf: (frame: number) => poses.get(frame) };
    });
  }

  // One log line: how far the corrected pose is from odometry's at the snapshot's last
  // frame, and how much of that this move added.
  async added(boot: string, sequence: number): Promise<void> {
    try {
      const line = await this.alone(boot, async b => {
        const t = await this.upTo(b, boot, sequence), stage = t?.stages.get(sequence);
        const odometry = t && stage && t.mapper.poses()[stage.poses - 1]?.odometry;
        if (!t || !stage || !odometry) return null;
        const before = [...t.stages.values()].filter(s => s.sequence < sequence).at(-1)?.correction ?? NONE;
        const now = corrected(stage.correction, odometry), was = corrected(before, odometry);
        const cm = (a: Pose, b: Pose) => (Math.hypot(a.x - b.x, a.y - b.y) * 100).toFixed(1);
        const deg = (a: Pose, b: Pose) => (Math.round((a.yaw - b.yaw) * 1800 / Math.PI) / 10 || 0).toFixed(1); // no "-0.0"
        return `outline map, snapshot ${sequence}: pose corrected by ${cm(now, odometry)} cm, ${deg(now, odometry)}° in all; ` +
               `this move ${cm(now, was)} cm, ${deg(now, was)}°`;
      });
      if (line) this.note(line);
    } catch (err) { this.note(`outline map, snapshot ${sequence}: not built (${(err as Error).message})`); }
  }

  async walls(boot: string, sequence: number): Promise<BandWalls[] | null> {
    // A mapper of its own, built for the snapshot (a few seconds): the kept one goes on
    // with the next snapshot's frames while the tables are written.
    const own = await this.fresh(boot);
    if (!own) return null;
    const mapper = (await this.feed(own, boot, sequence)).mapper;
    if (own.last?.sequence !== sequence) return null;
    const walls = mapper.walls(), mm = (m: number) => Math.round(m * 1000) / 1000;
    await this.store.write(async c => {
      await c.run("BEGIN TRANSACTION");
      for (const table of ["walls", "wall_sightings", "frame_poses"]) await c.run(`DELETE FROM ${table} WHERE boot_id = $1`, [boot]);
      for (const { band, walls: list } of walls)
        for (const w of list)
          await c.run("INSERT INTO walls VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13)",
            [boot, band, w.number, w.x, w.y, w.angle, 2 * w.halfM, doubtOf(w).acrossM, doubtOf(w).angleRad,
             w.sightings, w.firstFrame, w.lastFrame, sequence]);
      const sightings = await c.createAppender("wall_sightings");
      for (const s of mapper.sightings()) {
        for (const v of [boot, s.band]) sightings.appendVarchar(v);
        for (const v of [s.wall, s.frame]) sightings.appendInteger(v);
        for (const v of [s.pose.x, s.pose.y, s.pose.yaw, s.doubt.xyM, s.doubt.yawRad]) sightings.appendDouble(v);
        sightings.appendVarchar(JSON.stringify(s.hits.map(([x, y]) => [mm(x), mm(y)])));
        if (s.offM === null) sightings.appendNull(); else sightings.appendDouble(s.offM);
        if (s.turnRad === null) sightings.appendNull(); else sightings.appendDouble(s.turnRad);
        sightings.endRow();
      }
      sightings.closeSync();
      const poses = await c.createAppender("frame_poses");
      for (const p of mapper.poses()) {
        poses.appendVarchar(boot);
        poses.appendInteger(p.frame);
        for (const v of [p.odometry.x, p.odometry.y, p.odometry.yaw, p.pose.x, p.pose.y, p.pose.yaw, p.doubt.xyM, p.doubt.yawRad]) poses.appendDouble(v);
        poses.endRow();
      }
      poses.closeSync();
      await c.run("COMMIT");
    });
    return walls;
  }

  // Runs fn alone on the boot's kept mapper (one call at a time a boot); the newest boot used last.
  private alone<T>(boot: string, fn: (b: { track: Track | null }) => Promise<T>): Promise<T> {
    const b = this.boots.get(boot) ?? { track: null, chain: Promise.resolve() };
    this.boots.delete(boot);
    this.boots.set(boot, b);
    for (const old of this.boots.keys()) if (this.boots.size > BOOTS_KEPT) this.boots.delete(old);
    const run = b.chain.then(() => fn(b));
    b.chain = run.catch(() => {});
    return run;
  }

  // The boot's kept mapper brought up to snapshot `sequence`; started again if a snapshot it
  // passed now has other frames. null if the boot is unknown.
  private async upTo(b: { track: Track | null }, boot: string, sequence: number): Promise<Track | null> {
    const snapshots = await this.store.snapshotsUpTo(boot, sequence);
    const changed = (t: Track) => snapshots.some(s => {
      const kept = t.stages.get(s.sequence);
      return kept ? kept.lastFrame !== s.lastFrame : s.lastFrame < (t.last?.lastFrame ?? 0);
    });
    if (!b.track || changed(b.track)) b.track = await this.fresh(boot);
    // A feed that fails part way has given the mapper some frames of a snapshot: it is
    // built again next time, so no frame is given twice.
    try { return b.track && await this.feed(b.track, boot, sequence, snapshots); }
    catch (err) { b.track = null; throw err; }
  }

  private async fresh(boot: string): Promise<Track | null> {
    const mounting = await this.store.mountingOf(boot);
    return mounting && { mapper: new Mapper(), zoneRad: mounting.tof.zone_deg * Math.PI / 180, stages: new Map(), last: null };
  }

  // Feeds the track the frames up to each snapshot it has not passed, up to `sequence`.
  private async feed(t: Track, boot: string, sequence: number,
                     snapshots?: Awaited<ReturnType<Store["snapshotsUpTo"]>>): Promise<Track> {
    for (const s of snapshots ?? await this.store.snapshotsUpTo(boot, sequence)) {
      if (t.stages.has(s.sequence)) continue;
      for (const { frame } of await this.store.framesBetween(boot, t.last?.lastFrame ?? 0, s.lastFrame))
        t.mapper.add(frame, sightsOf(frame, await this.cellMaps.rays(boot, frame), t.zoneRad).filter(x => x !== null));
      const correction = t.mapper.correction();
      t.last = { sequence: s.sequence, lastFrame: s.lastFrame, poses: t.mapper.poses().length, correction,
                 robot: s.pose && corrected(correction, s.pose), bands: t.mapper.outlines() };
      t.stages.set(s.sequence, t.last);
    }
    return t;
  }
}
