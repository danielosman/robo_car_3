// The simulator's numbers (doc/OUTLINE_MAP_PLAN.md, "What is measured"): what the map
// under test drew against the truth the simulator knows.
//   poseError(pose, truth)   how far off a pose is, and by what angle
//   offFaces(faces, x, y)    how far a point is from the nearest true face
//   closed(loops)            are the loops closed, and do their edges cross anywhere
//   floorError(...)          the outline's floor against the floor the robot could truly see
import type { Band } from "../maps/outline/sights.ts";
import type { Loop } from "../maps/outline/index.ts";
import { cast, type Face } from "./room.ts";
import type { Pose } from "./drive.ts";

export function poseError(pose: Pose, truth: Pose): { m: number; rad: number } {
  const turn = (pose.yaw - truth.yaw) % (2 * Math.PI);
  return { m: Math.hypot(pose.x - truth.x, pose.y - truth.y), rad: Math.abs(turn > Math.PI ? turn - 2 * Math.PI : turn < -Math.PI ? turn + 2 * Math.PI : turn) };
}

export function offFaces(faces: Face[], x: number, y: number): number {
  let best = Infinity;
  for (const f of faces) {
    const ex = f.bx - f.ax, ey = f.by - f.ay;
    const t = Math.max(0, Math.min(1, ((x - f.ax) * ex + (y - f.ay) * ey) / (ex * ex + ey * ey || 1)));
    best = Math.min(best, Math.hypot(x - f.ax - t * ex, y - f.ay - t * ey));
  }
  return best;
}

// `open`: places where a loop's edge doesn't start where the edge before it ended.
// `crossings`: pairs of edges (of any loops) that cut through each other. Edges that
// only touch, or lie on each other (a thin wall seen from both sides), don't count.
export function closed(loops: Loop[]): { open: number; crossings: number } {
  let open = 0, crossings = 0;
  for (const { edges } of loops)
    edges.forEach((e, i) => { const next = edges[(i + 1) % edges.length]; if (Math.hypot(next.ax - e.bx, next.ay - e.by) > 1e-9) open++; });
  const all = loops.flatMap(l => l.edges), EPS = 1e-9;
  const side = (ax: number, ay: number, bx: number, by: number, x: number, y: number) => (bx - ax) * (y - ay) - (by - ay) * (x - ax);
  for (let i = 0; i < all.length; i++)
    for (let j = i + 1; j < all.length; j++) {
      const a = all[i], b = all[j];
      const b1 = side(a.ax, a.ay, a.bx, a.by, b.ax, b.ay), b2 = side(a.ax, a.ay, a.bx, a.by, b.bx, b.by);
      const a1 = side(b.ax, b.ay, b.bx, b.by, a.ax, a.ay), a2 = side(b.ax, b.ay, b.bx, b.by, a.bx, a.by);
      if (b1 * b2 < -EPS && a1 * a2 < -EPS) crossings++;
    }
  return { open, crossings };
}

// Is (x, y) inside the loops: inside the outline and in no island.
export function inside(loops: Loop[], x: number, y: number): boolean {
  let cuts = 0;
  for (const { edges } of loops)
    for (const e of edges)
      if ((e.ay > y) !== (e.by > y) && x < e.ax + (y - e.ay) / (e.by - e.ay) * (e.bx - e.ax)) cuts++;
  return cuts % 2 === 1;
}

const GRID_M = 0.05, SEE_M = 2.5, HALF_VIEW = 22.5 * Math.PI / 180;

// The outline's floor against the truth, on a 5 cm grid, for one band:
//   areaM2   the floor inside the outline
//   wrongM2  of it, the floor no true pose had a clear line to (behind a wall, inside a thing)
//   seen     the share, inside the outline, of the floor the robot could see: in view from
//            one of the true poses, within the sensor's reach, nothing of the band between
export function floorError(loops: Loop[], faces: Face[], band: Band, poses: Pose[]): { areaM2: number; wrongM2: number; seen: number } {
  const own = faces.filter(f => f.lowM < band.highM && f.highM > band.lowM);
  const xs = [...own.flatMap(f => [f.ax, f.bx]), ...loops.flatMap(l => l.edges.map(e => e.ax))], ys = [...own.flatMap(f => [f.ay, f.by]), ...loops.flatMap(l => l.edges.map(e => e.ay))];
  let area = 0, wrong = 0, could = 0, did = 0;
  for (let x = Math.min(...xs) - GRID_M / 2; x < Math.max(...xs) + GRID_M; x += GRID_M)
    for (let y = Math.min(...ys) - GRID_M / 2; y < Math.max(...ys) + GRID_M; y += GRID_M) {
      let clear = false, inView = false;
      for (const p of poses) {
        const d = Math.hypot(x - p.x, y - p.y), bearing = Math.atan2(y - p.y, x - p.x);
        if (clear && (inView || d > SEE_M)) continue;
        if (cast(own, p.x, p.y, bearing, d)) continue;
        clear = true;
        const turn = Math.atan2(Math.sin(bearing - p.yaw), Math.cos(bearing - p.yaw));
        if (d <= SEE_M && Math.abs(turn) <= HALF_VIEW) inView = true;
      }
      const drawn = inside(loops, x, y);
      if (drawn) { area++; if (!clear) wrong++; }
      if (inView) { could++; if (drawn) did++; }
    }
  return { areaM2: area * GRID_M ** 2, wrongM2: wrong * GRID_M ** 2, seen: could ? did / could : 1 };
}
