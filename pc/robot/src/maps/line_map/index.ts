// The PC's line maps (doc/LINE_MAP_PLAN.md): a boot's lines at a snapshot, one map per
// height band, built from the boot's frames that came before the snapshot, as the PC's
// cell map is (maps/cell_map.ts). Each frame's zones are placed by the cell map's helper
// (the robot's own geometry, with the boot's mounting), turned into sights and added to
// each band's map. Built again on every read (how long: doc/LINE_MAP_PLAN.md, the numbers).
import type { Store } from "../../store.ts";
import type { CellMaps } from "../cell_map.ts";
import { LineMap, type Segment } from "./line_map.ts";
import { BANDS, inBand, sightsOf, type BandName } from "./sights.ts";

export type { BandName, Segment };
// One band's lines; the bands come in sights.ts's order (BANDS): L1, L2, above.
export interface BandLines { band: BandName; lines: Segment[] }

// The boot's lines at snapshot `sequence`; null if there is no such snapshot.
export async function lineMapsAt(store: Store, cellMaps: CellMaps, boot: string, sequence: number): Promise<BandLines[] | null> {
  const snapshot = await store.snapshotFrames(boot, sequence), mounting = await store.mountingOf(boot);
  if (!snapshot || !mounting) return null;
  const zoneRad = mounting.tof.zone_deg * Math.PI / 180, maps = BANDS.map(() => new LineMap());
  for (const { frame } of await store.framesBetween(boot, 0, snapshot.lastFrame)) {
    const sights = sightsOf(frame, await cellMaps.rays(boot, frame), zoneRad).filter(s => s !== null);
    BANDS.forEach((band, i) => maps[i].add(inBand(sights, band)));
  }
  return BANDS.map((band, i) => ({ band: band.name, lines: maps[i].lines() }));
}
