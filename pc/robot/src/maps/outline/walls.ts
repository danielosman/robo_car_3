// One band's walls (doc/OUTLINE_MAP_PLAN.md, idea 2): the long straight stretches of the
// outline, remembered with a number and how sure their place is, and moved a little by
// every frame that sees them again.
//   walls.see(frame, pose, doubt, sights)   one frame's hits (one band's) on the walls:
//                                           each wall seen moves towards them; its sightings
//   walls.found(edges, frame, pose, doubt)  new walls from the outline's wall edges on no known wall
//   walls.on(x, y)                          the wall a border point lies on, and the point put on its line
//   walls.all(), walls.numberNow(n)         the walls; the number a wall has now (two can become one)
//   doubtOf(wall), sureOf(wall)             how sure a wall's place is: its spreads; 0 to 1 (the brightness it is drawn with)
// A wall is a line: its middle, its direction and half its length. How sure it is: the
// spread of its offset across (at the middle) and of its direction, with their covariance.
// A frame's hits on a wall give one measurement: how far their middle lies off the line
// and, if they span enough, their slope against it. It moves the wall by the wall's share
// of the doubt (a Kalman filter); the rest of the doubt is the hits' own and the pose's
// (pose.ts). Step C will move the pose by its share too.
import type { Edge } from "./shape.ts";
import type { Sight } from "./sights.ts";
import { lineDoubt, type Pose, type PoseDoubt } from "./pose.ts";

export interface Wall {
  number: number;
  x: number; y: number;  // its middle
  angle: number;         // its direction (rad; which way it runs means nothing)
  halfM: number;         // half its length
  // How sure its place is from its hits: the spread of its offset across at the middle
  // (m²), across·direction, of its direction (rad²). The filter's own: it takes each frame's
  // pose error as new, but frames seen from one place share it. So how sure the wall
  // truly is (doubtOf) adds the surest pose it was seen from.
  cov: [number, number, number];
  surest: PoseDoubt;
  sightings: number; firstFrame: number; lastFrame: number;
}

// A frame's sighting of a wall: from what pose, the hits on it, and how far off the wall
// they were before it moved (their middle across it; their slope against it, if they spanned enough).
export interface Sighting {
  wall: number; frame: number; pose: Pose; doubt: PoseDoubt;
  hits: [number, number][]; offM: number; turnRad: number | null;
}

const EXTEND_M = 0.2;      // a hit this far beyond a wall's end can still be on it (and lengthens it)
const MIN_INCIDENCE = 0.5; // a ray meeting the wall at less than 30° (its sine) places a hit badly along it: not used
const NEAR_M = 1.2;        // closer and still, a hit is placed within about 2 cm; else about 8 (sights.ts, the smear)
const NEAR_SD_M = 0.02, FAR_SD_M = 0.08;
const GATE_SDS = 3;        // a hit farther off a wall than this many spreads is not on it: a new thing ...
const GATE_MAX_M = 0.15;   // ... and never farther than this, however unsure (a corner's other wall is close by)
const MIN_HITS = 3;        // fewer hits on a wall in a frame tell nothing about it
const SLOPE_SPAN_M = 0.2;  // hits spanning this much along a wall also give its direction
const MIN_SD_M = 0.01;     // a frame's hits are not independent: their middle is never surer than this
const NEW_SD_M = 0.05;     // a new wall's own spread (the outline's wall rules: its hits fit a line to 3 to 5 cm)
const ON_M = 0.1, ON_RAD = 10 * Math.PI / 180;      // an edge this close to a known wall is that wall, not a new one
const MERGE_M = 0.05, MERGE_RAD = 5 * Math.PI / 180; // two walls this close in line, end to end or overlapping, are one
const SNAP_M = 0.12;       // a border point this close to a wall is put on it (drawing): far hits smear by about 7 cm
const SURE_SD_M = 0.01, UNSURE_SD_M = 0.1;         // drawn brightest at this spread across, faintest at this

const dir = (w: Wall) => [Math.cos(w.angle), Math.sin(w.angle)] as const;
// (x, y) in a wall's frame: along it from its middle, and across it (left of its direction).
function coords(w: Wall, x: number, y: number): { u: number; v: number } {
  const [dx, dy] = dir(w), px = x - w.x, py = y - w.y;
  return { u: px * dx + py * dy, v: py * dx - px * dy };
}
const halfTurn = (a: number) => ((a % Math.PI) + Math.PI) % Math.PI;
// The angle between two directions, ignoring which way they run (0 to π/2).
const between = (a: number, b: number) => { const d = halfTurn(a - b); return Math.min(d, Math.PI - d); };
// The across spread² a wall's own doubt gives at `u` along it.
const wallVar = (w: Wall, u: number) => w.cov[0] + 2 * u * w.cov[1] + u * u * w.cov[2];

// The wall's middle moved `s` along it: its offset across there is offset + s·direction.
function recentred(w: Wall, s: number): void {
  const [dx, dy] = dir(w), [nn, nt, tt] = w.cov;
  w.x += s * dx; w.y += s * dy;
  w.cov = [nn + 2 * s * nt + s * s * tt, nt + s * tt, tt];
}

// How sure a wall's place is: the spread across it (at its middle) and of its direction.
export function doubtOf(w: Wall): { acrossM: number; angleRad: number } {
  return { acrossM: Math.sqrt(w.cov[0] + w.surest.xyM ** 2), angleRad: Math.sqrt(w.cov[2] + w.surest.yawRad ** 2) };
}

export function sureOf(w: Wall): number {
  const sd = doubtOf(w).acrossM;
  return Math.max(0, Math.min(1, Math.log(UNSURE_SD_M / sd) / Math.log(UNSURE_SD_M / SURE_SD_M)));
}

interface Taken { u: number; v: number; weight: number; x: number; y: number }

export class Walls {
  private walls: Wall[] = [];
  private next = 1;
  private renamed = new Map<number, number>(); // a wall merged into another: its number, the other's

  all(): readonly Wall[] { return this.walls; }

  numberNow(n: number): number {
    for (let to = this.renamed.get(n); to !== undefined; to = this.renamed.get(n)) n = to;
    return n;
  }

  see(frame: number, pose: Pose, doubt: PoseDoubt, sights: Sight[]): Sighting[] {
    const taken = new Map<Wall, Taken[]>();
    for (const s of sights) {
      if (!s.hit) continue;
      const own = s.weight >= 1 && s.hit.rangeM < NEAR_M ? NEAR_SD_M : FAR_SD_M;
      let best: { w: Wall; t: Taken; score: number } | null = null;
      for (const w of this.walls) {
        const { u, v } = coords(w, s.hit.x, s.hit.y);
        if (Math.abs(u) > w.halfM + EXTEND_M) continue;
        const incidence = Math.abs(Math.sin(s.bearing - w.angle));
        if (incidence < MIN_INCIDENCE) continue;
        // The hit's spread across the wall: its distance's along the ray, the wedge's width across it.
        const hitVar = (own * incidence) ** 2 + (s.hit.rangeM * Math.tan(s.halfWidth) * Math.sqrt(1 - incidence ** 2)) ** 2;
        const sd = Math.sqrt(hitVar + wallVar(w, u) + lineDoubt(doubt, u - coords(w, pose.x, pose.y).u)[0]);
        if (Math.abs(v) > Math.min(GATE_MAX_M, GATE_SDS * sd)) continue;
        if (!best || Math.abs(v) / sd < best.score) best = { w, t: { u, v, weight: 1 / hitVar, x: s.hit.x, y: s.hit.y }, score: Math.abs(v) / sd };
      }
      if (best) taken.set(best.w, [...(taken.get(best.w) ?? []), best.t]);
    }
    const out: Sighting[] = [];
    for (const [w, hits] of taken) if (hits.length >= MIN_HITS) out.push(this.update(w, hits, frame, pose, doubt));
    this.merge();
    return out;
  }

  private update(w: Wall, hits: Taken[], frame: number, pose: Pose, doubt: PoseDoubt): Sighting {
    const total = hits.reduce((m, h) => m + h.weight, 0);
    const u0 = hits.reduce((m, h) => m + h.weight * h.u, 0) / total, off = hits.reduce((m, h) => m + h.weight * h.v, 0) / total;
    const spread = hits.reduce((m, h) => m + h.weight * (h.u - u0) ** 2, 0);
    const span = Math.max(...hits.map(h => h.u)) - Math.min(...hits.map(h => h.u));
    const [pa, pc, pd] = lineDoubt(doubt, u0 - coords(w, pose.x, pose.y).u);
    const offVar = Math.max(1 / total, MIN_SD_M ** 2) + pa;
    // The measurement: the hits' middle across the wall at u0 (= offset + u0·turn), and their slope (= turn).
    const H: [number, number][] = [[1, u0]], z = [off], R = [[offVar]];
    let turn: number | null = null;
    if (span >= SLOPE_SPAN_M) {
      turn = Math.atan(hits.reduce((m, h) => m + h.weight * (h.u - u0) * (h.v - off), 0) / spread);
      H.push([0, 1]); z.push(turn);
      R[0].push(pc); R.push([pc, 1 / spread + pd]);
    }
    const [dOff, dTurn] = kalman(w, H, z, R);
    w.surest = { xyM: Math.min(w.surest.xyM, doubt.xyM), yawRad: Math.min(w.surest.yawRad, doubt.yawRad) };
    // Move the line: across at its middle, and turned about it.
    const [dx, dy] = dir(w);
    w.x -= dOff * dy; w.y += dOff * dx;
    w.angle += dTurn;
    // Lengthen it to the hits beyond its ends.
    const us = hits.map(h => coords(w, h.x, h.y).u), from = Math.min(-w.halfM, ...us), to = Math.max(w.halfM, ...us);
    recentred(w, (from + to) / 2);
    w.halfM = (to - from) / 2;
    w.sightings++; w.lastFrame = frame;
    return { wall: w.number, frame, pose, doubt, hits: hits.map(h => [h.x, h.y]), offM: off, turnRad: turn };
  }

  found(edges: Edge[], frame: number, pose: Pose, doubt: PoseDoubt): Wall[] {
    const made: Wall[] = [];
    for (const e of edges) {
      if (e.kind !== "wall" || e.wall || this.walls.some(w => this.covers(w, e))) continue;
      const halfM = Math.hypot(e.bx - e.ax, e.by - e.ay) / 2;
      const w: Wall = { number: this.next++, x: (e.ax + e.bx) / 2, y: (e.ay + e.by) / 2, angle: Math.atan2(e.by - e.ay, e.bx - e.ax),
                        halfM, cov: [0, 0, 0], surest: doubt, sightings: 0, firstFrame: frame, lastFrame: frame };
      // Its own doubt (its hits lie within a few cm of it), and the pose's it was seen from.
      const [pa, pc, pd] = lineDoubt(doubt, -coords(w, pose.x, pose.y).u);
      w.cov = [NEW_SD_M ** 2 + pa, pc, (NEW_SD_M / halfM) ** 2 + pd];
      this.walls.push(w);
      made.push(w);
    }
    this.merge();
    return made;
  }

  on(x: number, y: number): { wall: Wall; x: number; y: number } | null {
    let best: { wall: Wall; x: number; y: number } | null = null, bestM = SNAP_M;
    for (const w of this.walls) {
      const { u, v } = coords(w, x, y);
      if (Math.abs(u) > w.halfM + SNAP_M || Math.abs(v) > bestM) continue;
      const [dx, dy] = dir(w);
      best = { wall: w, x: w.x + u * dx, y: w.y + u * dy };
      bestM = Math.abs(v);
    }
    return best;
  }

  // Is the edge on the wall: about the same direction, both its ends close to the line, not beyond its ends.
  private covers(w: Wall, e: Edge): boolean {
    const a = coords(w, e.ax, e.ay), b = coords(w, e.bx, e.by);
    return between(w.angle, Math.atan2(e.by - e.ay, e.bx - e.ax)) < ON_RAD && Math.abs(a.v) < ON_M && Math.abs(b.v) < ON_M &&
           Math.max(a.u, b.u) > -w.halfM - EXTEND_M && Math.min(a.u, b.u) < w.halfM + EXTEND_M;
  }

  // Two walls that grew into each other are one: the surer one's line, both lengths, the older number.
  private merge(): void {
    for (let pair = this.pairToMerge(); pair; pair = this.pairToMerge()) {
      const [a, b] = pair, keep = a.cov[0] <= b.cov[0] ? a : b, other = keep === a ? b : a, [dx, dy] = dir(other);
      const us = [-keep.halfM, keep.halfM, ...[-1, 1].map(k => coords(keep, other.x + k * other.halfM * dx, other.y + k * other.halfM * dy).u)];
      const from = Math.min(...us), to = Math.max(...us);
      recentred(keep, (from + to) / 2);
      const merged: Wall = { ...keep, number: a.number, halfM: (to - from) / 2, sightings: a.sightings + b.sightings,
                             surest: { xyM: Math.min(a.surest.xyM, b.surest.xyM), yawRad: Math.min(a.surest.yawRad, b.surest.yawRad) },
                             firstFrame: Math.min(a.firstFrame, b.firstFrame), lastFrame: Math.max(a.lastFrame, b.lastFrame) };
      this.walls = this.walls.filter(w => w !== a && w !== b).concat(merged).sort((p, q) => p.number - q.number);
      this.renamed.set(b.number, a.number);
    }
  }

  // Two walls in line, end to end or overlapping: the older first.
  private pairToMerge(): [Wall, Wall] | null {
    for (const a of this.walls)
      for (const b of this.walls) {
        if (a.number >= b.number || between(a.angle, b.angle) > MERGE_RAD) continue;
        if (Math.abs(coords(a, b.x, b.y).v) > MERGE_M || Math.abs(coords(b, a.x, a.y).v) > MERGE_M) continue;
        if (Math.abs(coords(a, b.x, b.y).u) > a.halfM + b.halfM + EXTEND_M) continue;
        return [a, b];
      }
    return null;
  }
}

// One Kalman step on a wall's (offset, turn): the measurement z = H·(offset, turn) with
// noise R; the wall's covariance shrinks. Gives the change of (offset, turn).
function kalman(w: Wall, H: [number, number][], z: number[], R: number[][]): [number, number] {
  const P = [[w.cov[0], w.cov[1]], [w.cov[1], w.cov[2]]];
  const PHt = [0, 1].map(i => H.map(h => P[i][0] * h[0] + P[i][1] * h[1]));      // 2 × m
  const S = H.map((h, r) => H.map((_, c) => h[0] * PHt[0][c] + h[1] * PHt[1][c] + R[r][c])); // m × m
  const Sinv = S.length === 1 ? [[1 / S[0][0]]] : inverse2(S);
  const K = PHt.map(row => Sinv[0].map((_, c) => row.reduce((m, v, k) => m + v * Sinv[k][c], 0))); // 2 × m
  const change = K.map(row => row.reduce((m, k, i) => m + k * z[i], 0)) as [number, number];
  // P := (I - K H) P
  const KH = [0, 1].map(i => [0, 1].map(j => K[i].reduce((m, k, r) => m + k * H[r][j], 0)));
  const next = [0, 1].map(i => [0, 1].map(j => P[i][j] - (KH[i][0] * P[0][j] + KH[i][1] * P[1][j])));
  w.cov = [next[0][0], (next[0][1] + next[1][0]) / 2, next[1][1]];
  return change;
}

function inverse2(m: number[][]): number[][] {
  const det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
  return [[m[1][1] / det, -m[0][1] / det], [-m[1][0] / det, m[0][0] / det]];
}
