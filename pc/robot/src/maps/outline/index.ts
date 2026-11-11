// The PC's outline maps (doc/OUTLINE_MAP_PLAN.md): a boot's outlines at a snapshot, one
// map per height band, built from the boot's frames that came before the snapshot, as
// the PC's cell map is (maps/cell_map.ts). Each frame's zones are placed by the cell
// map's helper (the robot's own geometry, with the boot's mounting), turned into sights
// and added to each band's map. Built again on every read.
//   outlinesAt(store, cellMaps, boot, n)   each band's outline and islands
//   outlineStrokes(outline)                one band's edges as coloured lines to draw
import type { Store } from "../../store.ts";
import type { CellMaps } from "../cell_map.ts";
import { OutlineMap, type Edge, type Loop } from "./outline_map.ts";
import { BANDS, sightsOf, type BandName } from "./sights.ts";

export type { BandName, Edge, Loop };
// One band's outline; the bands come in sights.ts's order (BANDS): L1, L2, above.
export interface BandOutline { band: BandName; loops: Loop[] }

// The boot's outlines at snapshot `sequence`; null if there is no such snapshot.
export async function outlinesAt(store: Store, cellMaps: CellMaps, boot: string, sequence: number): Promise<BandOutline[] | null> {
  const snapshot = await store.snapshotFrames(boot, sequence), mounting = await store.mountingOf(boot);
  if (!snapshot || !mounting) return null;
  const zoneRad = mounting.tof.zone_deg * Math.PI / 180, maps = BANDS.map(band => new OutlineMap(band));
  for (const { frame } of await store.framesBetween(boot, 0, snapshot.lastFrame)) {
    const sights = sightsOf(frame, await cellMaps.rays(boot, frame), zoneRad).filter(s => s !== null);
    for (const map of maps) map.add(sights);
  }
  return BANDS.map((band, i) => ({ band: band.name, loops: maps[i].outline() }));
}

export type Rgb = readonly [number, number, number];
// A line to draw, in metres.
export interface Stroke { ax: number; ay: number; bx: number; by: number; rgb: Rgb }

// The bands' colours (white: L1, yellow: L2, cyan: above) and how bright each kind of
// edge is drawn: a wall strong, a rough edge medium, a guess faint.
export const BAND_RGB: Record<BandName, Rgb> = { L1: [0xff, 0xff, 0xff], L2: [0xf2, 0xe2, 0x4a], above: [0x30, 0xd0, 0xd0] };
const SHADE = { wall: 1, rough: 0.6, guess: 0.3 } as const;
export const edgeRgb = (band: BandName, kind: Edge["kind"]): Rgb =>
  [Math.round(BAND_RGB[band][0] * SHADE[kind]), Math.round(BAND_RGB[band][1] * SHADE[kind]), Math.round(BAND_RGB[band][2] * SHADE[kind])];

// One band's edges as lines, the faint ones first so the strong ones are drawn over them.
export function outlineStrokes(outline: BandOutline): Stroke[] {
  const order = ["guess", "rough", "wall"] as const;
  return outline.loops.flatMap(loop => loop.edges).sort((a, b) => order.indexOf(a.kind) - order.indexOf(b.kind))
    .map(e => ({ ax: e.ax, ay: e.ay, bx: e.bx, by: e.by, rgb: edgeRgb(outline.band, e.kind) }));
}
