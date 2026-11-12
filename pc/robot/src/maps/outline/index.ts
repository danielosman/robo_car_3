// The PC's outline maps (doc/OUTLINE_MAP_PLAN.md): a boot's outlines and walls at a
// snapshot, built from the boot's frames that came before the snapshot, as the PC's cell
// map is (maps/cell_map.ts). Each frame's zones are placed by the cell map's helper (the
// robot's own geometry, with the boot's mounting), turned into sights and given to the
// mapper (mapper.ts). Built again on every read.
//   outlinesAt(store, cellMaps, boot, n)   each band's outline and islands
//   wallsAt(store, cellMaps, boot, n)      each band's walls; writes the tables walls,
//                                          wall_sightings and frame_poses (the boot's rows, replaced)
import type { Store } from "../../store.ts";
import type { CellMaps } from "../cell_map.ts";
import { Mapper, type BandOutline, type BandWalls } from "./mapper.ts";
import { sightsOf } from "./sights.ts";
import { doubtOf } from "./walls.ts";

export type { BandName } from "./sights.ts";
export type { Edge, Loop } from "./outline_map.ts";
export type { BandOutline, BandWalls };
export { edgeRgb, outlineStrokes, BAND_RGB, type Rgb, type Stroke } from "./colours.ts";

// The boot's mapper after its frames up to snapshot `sequence`; null if there is no such snapshot.
async function mapperAt(store: Store, cellMaps: CellMaps, boot: string, sequence: number): Promise<Mapper | null> {
  const snapshot = await store.snapshotFrames(boot, sequence), mounting = await store.mountingOf(boot);
  if (!snapshot || !mounting) return null;
  const zoneRad = mounting.tof.zone_deg * Math.PI / 180, mapper = new Mapper();
  for (const { frame } of await store.framesBetween(boot, 0, snapshot.lastFrame))
    mapper.add(frame, sightsOf(frame, await cellMaps.rays(boot, frame), zoneRad).filter(s => s !== null));
  return mapper;
}

export async function outlinesAt(store: Store, cellMaps: CellMaps, boot: string, sequence: number): Promise<BandOutline[] | null> {
  return (await mapperAt(store, cellMaps, boot, sequence))?.outlines() ?? null;
}

export async function wallsAt(store: Store, cellMaps: CellMaps, boot: string, sequence: number): Promise<BandWalls[] | null> {
  const mapper = await mapperAt(store, cellMaps, boot, sequence);
  if (!mapper) return null;
  const walls = mapper.walls(), mm = (m: number) => Math.round(m * 1000) / 1000;
  await store.write(async c => {
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
      sightings.appendDouble(s.offM);
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
