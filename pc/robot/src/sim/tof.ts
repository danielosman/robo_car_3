// The simulator's ToF (doc/OUTLINE_MAP_PLAN.md, "The simulator"): 8 x 8 zones of one zone
// angle each, the sensor where the mounting puts it. It measures the room from the
// robot's true pose and hands the reading on placed by the odometry pose, in the form
// the real reader gets from the store and the helper: a posed frame and its zones'
// rays. So sightsOf() (maps/outline/sights.ts) and all after it is the real code.
//   tofFrame(faces, moment, number, noise, random)  one frame and its rays
// A zone reads what its middle ray meets at the zone's heights (a thing, the floor or
// "beyond"); if that ray met nothing, the nearest thing the rays beside it met (a thin
// thing). The zone geometry and the verdict rule are copies of the robot's
// (picoA/app/tof_rays.c, their owner); a test holds them against the real helper. The
// noise is as measured on boot cc0e4e65 (doc/LINE_MAP_PLAN.md, the numbers), each kind
// switchable.
import { loadMounting } from "../mounting.ts";
import { COMPACT_BEYOND_MM, TOF_COLS, TOF_ROWS, type PosedFrame, type Zone } from "../tof_format.ts";
import type { ZoneRays } from "../maps/cell_map.ts";
import { cast, type Face } from "./room.ts";
import { FRAME_S, type Moment, type Random } from "./drive.ts";

export interface Noise {
  steps: boolean;   // the distance comes in 1 cm steps
  near: boolean;    // close and still: within a few cm
  far: boolean;     // beyond 1.2 m, or while moving: smeared 20-30 cm along the zone
  edges: boolean;   // a zone that covers two depths reads somewhere between them
  missing: boolean; // a few readings are unsure
}
export const NO_NOISE: Noise = { steps: false, near: false, far: false, edges: false, missing: false };
export const ALL_NOISE: Noise = { steps: true, near: true, far: true, edges: true, missing: true };

const REACH_M = 2.54;             // farther reads as "beyond"
const FAR_M = 1.2;                // from here on a reading smears
const NEAR_SPREAD_M = 0.01, FAR_SPREAD_M = 0.07;
const TWO_DEPTHS_M = 0.15;        // a zone whose rays end this far apart covers two depths
const MISSING_SHARE = 0.03;
const FLOOR_TOP_M = 0.03, FLOOR_MARGIN = 0.9; // tof_rays.h: TOF_FLOOR_TOP_M, TOF_FLOOR_MARGIN
const ACROSS = [-0.4, -0.2, 0, 0.2, 0.4]; // the rays across a zone, in zone widths from its middle

const mounting = loadMounting().tof;
export const ZONE_RAD = mounting.zone_deg * Math.PI / 180;

export function tofFrame(faces: Face[], moment: Moment, number: number, noise: Noise, random: Random): { frame: PosedFrame; rays: ZoneRays[] } {
  const { truth, odometry, still } = moment;
  const sensor = (p: typeof truth) => [p.x + mounting.x_m * Math.cos(p.yaw) - mounting.y_m * Math.sin(p.yaw),
                                       p.y + mounting.x_m * Math.sin(p.yaw) + mounting.y_m * Math.cos(p.yaw)] as const;
  const [tx, ty] = sensor(truth), [ox, oy] = sensor(odometry), z = mounting.z_m;
  const zones: Zone[] = [], rays: ZoneRays[] = [];
  for (let row = 0; row < TOF_ROWS; row++)
    for (let col = 0; col < TOF_COLS; col++) {
      // Row 1 looks up most, column 1 is on the left (README, conventions).
      const up = (3.5 - row) * ZONE_RAD, left = (3.5 - col) * ZONE_RAD, cos = Math.cos(up);
      // The zone's heights as the helper gives them: its rays at 1/6 and 5/6 of the zone, per metre along the middle ray.
      const lowPerRange = Math.sin(up - ZONE_RAD / 3), highPerRange = Math.sin(up + ZONE_RAD / 3);
      const lowPerM = lowPerRange / cos, highPerM = highPerRange / cos; // per metre on the floor plan
      const floorM = up < 0 ? z / Math.tan(-up) : Infinity; // where the zone's middle meets the floor
      // Each ray across the zone: how far it goes on the floor plan (Infinity: beyond).
      const depths = ACROSS.map(a => {
        const thing = cast(faces, tx, ty, truth.yaw + left + a * ZONE_RAD, REACH_M * cos,
                           (d, f) => z + lowPerM * d < f.highM && z + highPerM * d > f.lowM);
        const d = Math.min(thing?.d ?? Infinity, floorM);
        return d <= REACH_M * cos ? d : Infinity;
      });
      const ended = depths.filter(Number.isFinite), nearest = Math.min(...ended), farthest = Math.max(...ended);
      // The zone's middle ray gives the reading; if it met nothing, the nearest ray that did (a thin thing).
      let d = Number.isFinite(depths[2]) ? depths[2] : ended.length ? nearest : Infinity;
      // Two depths: a jump between neighbouring rays (a slanted wall has none).
      const jump = depths.some((depth, i) => i > 0 && Number.isFinite(depth) && Number.isFinite(depths[i - 1]) && Math.abs(depth - depths[i - 1]) > TWO_DEPTHS_M);
      if (noise.edges && jump) d = nearest + random.uniform() * (farthest - nearest);
      let range = d / cos; // along the ray, as the sensor measures
      // A reading that ends on the floor doesn't smear (a first guess: doc/OUTLINE_MAP_PLAN.md, open points).
      const smeared = (!still || range > FAR_M) && d !== floorM;
      if (Number.isFinite(range)) {
        if (smeared && noise.far) range += random.normal() * FAR_SPREAD_M;
        if (!smeared && noise.near) range += random.normal() * NEAR_SPREAD_M;
        if (noise.steps) range = Math.round(range * 100) / 100;
        range = Math.max(0.02, range);
      }
      const beyond = range > REACH_M, mm = beyond ? COMPACT_BEYOND_MM : Math.round(range * 1000);
      const unsure = noise.missing && random.uniform() < MISSING_SHARE;
      zones.push({ distanceMm: mm, signal: 50, sigmaMm: smeared ? 20 : 5, kind: unsure ? "unsure" : "sure", targets: 1 });
      // Placed by the odometry pose, as the helper places a stored frame.
      const length = mm / 1000, flat = length * cos, bearing = odometry.yaw + left, highM = z + length * highPerRange;
      // The robot's rule (tof_rays.c, verdict): closer than where the zone's lower edge meets the floor is an
      // obstacle; otherwise floor if its rays end at floor height, unsure if they straddle it.
      const floorFromM = up < ZONE_RAD / 2 ? z / Math.sin(ZONE_RAD / 2 - up) : Infinity;
      rays.push({
        verdict: unsure ? "unsure" : length < FLOOR_MARGIN * floorFromM ? "obstacle" : highM >= FLOOR_TOP_M ? "unsure" : "floor",
        point: [ox + flat * Math.cos(bearing), oy + flat * Math.sin(bearing), z + length * Math.sin(up)],
        lowM: z + length * lowPerRange, highM,
        origin: [ox, oy, z],
      });
    }
  const frame: PosedFrame = { number, robotClockUs: Math.round(number * FRAME_S * 1e6), periodUs: FRAME_S * 1e6, x: odometry.x, y: odometry.y, yaw: odometry.yaw,
                              pitch: 0, turn: 0, still, poseKnown: true, beyondMm: COMPACT_BEYOND_MM, zones };
  return { frame, rays };
}
