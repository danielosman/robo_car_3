// The line map's input (doc/LINE_MAP_PLAN.md): a ToF frame's zones as sights. A sight is
// one zone's flat wedge on the floor plan: the sensor at its tip, one zone angle wide,
// how far it saw through, how high it is along the way, and what it hit at its end, if
// anything. Only this file knows ToF frames, verdicts and the band heights; the line
// map itself (line_map.ts) reads sights alone.
//   sightsOf(frame, rays, zoneRad)  the frame's 64 zones as sights, in zone order
//   inBand(sights, band)            the same sights for one band's map: only the hits that
//                                   reach the band, their heights cut to it
//   heightsAt(sight, d)             the heights the wedge covers d from the sensor
import { isBeyond, type PosedFrame } from "../../tof_format.ts";
import type { ZoneRays } from "../cell_map.ts";

// The height bands, the cell map's layers L1 and L2 (picoA/app/cell_map.c) and all above.
export type BandName = "L1" | "L2" | "above";
export interface Band { name: BandName; lowM: number; highM: number }
export const BANDS: readonly Band[] = [
  { name: "L1", lowM: 0.03, highM: 0.13 },      // blocks the robot
  { name: "L2", lowM: 0.13, highM: 0.23 },      // an overhang
  { name: "above", lowM: 0.23, highM: Infinity }, // the robot passes under
];

// Something standing at a sight's end: where (the wedge's middle), how far from the
// sensor, and the heights the wedge covers there.
export interface Hit { x: number; y: number; rangeM: number; lowM: number; highM: number }

// Distances are on the floor plan, from the sensor. Heights are above the floor.
export interface Sight {
  x: number; y: number;      // the sensor
  bearing: number;           // the wedge's middle (rad, map frame)
  halfWidth: number;         // rad
  reachM: number;            // nothing stood in the wedge closer than this (0: it saw through nothing)
  z: number;                 // the sensor's height; with the two slopes, the wedge's heights along the way:
  lowPerM: number; highPerM: number; // read them with heightsAt()
  weight: number;            // 1: a sure reading while still; less when less sure or moving
  hit: Hit | null;
}

// The lowest and highest height the wedge covers at distance d from the sensor. They are
// the helper's lowest and highest ray (at 1/6 and 5/6 of the zone, as the cell map takes
// them), so a little inside the zone's true borders.
export function heightsAt(s: Sight, d: number): [number, number] {
  return [s.z + s.lowPerM * d, s.z + s.highPerM * d];
}

const MARGIN_M = 0.05; // the seen-through stretch ends at least this far before the reading

// Each zone's sight (index: the zone's, tof_format.ts zoneAt); null: it saw nothing usable.
export function sightsOf(frame: PosedFrame, rays: ZoneRays[], zoneRad: number): (Sight | null)[] {
  return rays.map((r, zone) => sightOf(frame, zone, r, zoneRad));
}

function sightOf(frame: PosedFrame, zone: number, r: ZoneRays, zoneRad: number): Sight | null {
  if (!r) return null; // no valid reading, or no pose
  const z = frame.zones[zone], [x, y, height] = r.origin;
  const dx = r.point[0] - x, dy = r.point[1] - y, rangeM = Math.hypot(dx, dy);
  if (rangeM < 1e-3) return null;
  const sigmaM = z.sigmaMm / 1000;
  // A wall seen at an angle is closer at one edge of the wedge than at its middle: by up
  // to a zone width at 30°, the shallowest angle that still counts (line_map.ts).
  const margin = Math.max(MARGIN_M, 2 * sigmaM, rangeM * zoneRad);
  const stands = r.verdict === "obstacle" && !isBeyond(frame, z);
  return {
    x, y, bearing: Math.atan2(dy, dx), halfWidth: zoneRad / 2, reachM: Math.max(0, rangeM - margin),
    z: height, lowPerM: (r.lowM - height) / rangeM, highPerM: (r.highM - height) / rangeM,
    weight: (frame.still ? 1 : 0.5) * (z.kind === "sure" ? 1 : 0.5),
    hit: stands ? { x: r.point[0], y: r.point[1], rangeM, lowM: r.lowM, highM: r.highM } : null,
  };
}

// The sights as one band's map takes them: a hit stays if its heights reach the band, cut
// to it. What a sight saw through is the same for every band.
export function inBand(sights: Sight[], band: Band): Sight[] {
  return sights.map(s => {
    if (!s.hit) return s;
    const lowM = Math.max(s.hit.lowM, band.lowM), highM = Math.min(s.hit.highM, band.highM);
    return { ...s, hit: lowM < highM ? { ...s.hit, lowM, highM } : null };
  });
}
