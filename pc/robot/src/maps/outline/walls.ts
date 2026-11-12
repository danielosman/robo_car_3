// One band's walls (doc/OUTLINE_MAP_PLAN.md, idea 2): the long straight stretches of the
// outline, remembered with a number and how sure their place is, and moved a little by
// every frame that sees them again.
//   walls.see(frame, pose, doubt, sights)   one frame's hits (one band's) on the walls:
//                                           each wall seen moves towards them; its sightings
//   walls.measure(...), walls.apply(...)    the same in two halves, so the pose can take its share between (pose_check.ts)
//   walls.found(edges, frame, pose, doubt, sights)  new walls where the outline has a wall edge on no known wall:
//                                           the line through this frame's hits along it
//   walls.on(x, y)                          the wall a border point lies on, and the point put on its line
//   walls.all(), walls.numberNow(n)         the walls; the number a wall has now (two can become one)
//   priorOf(wall), doubtOf(wall), sureOf(wall)  a wall's whole doubt; its spreads; 0 to 1 (the brightness it is drawn with)
// A wall is a line: its middle, its direction and half its length. How sure it is: the
// spread of its offset across (at the middle) and of its direction, with their covariance.
// A frame's hits on a wall give one measurement: how far their middle lies off the line
// and, if they span enough, their slope against it. It moves the wall by the wall's share
// of the doubt and the pose by the pose's (pose_check.ts, a Kalman filter).
import type { Edge } from "./shape.ts";
import { seesBand, type Band, type Sight } from "./sights.ts";
import { lineDoubt, NO_DOUBT, type Pose, type PoseDoubt } from "./pose.ts";
import { wallsAlone, type PoseCheck, type WallChange } from "./pose_check.ts";

export interface Wall {
  number: number;
  x: number; y: number;  // its middle
  angle: number;         // its direction (rad; which way it runs means nothing)
  halfM: number;         // half its length
  endSd: [number, number]; // how sure its two ends are (m): the one behind its middle, the one ahead
  // How sure its place is: the spread of its offset across at the middle (m²),
  // across·direction, of its direction (rad²). With the pose check (pose_check.ts) that is
  // all. Without it, it is the filter's own: it takes each frame's pose error as new, but
  // frames seen from one place share it, so how sure the wall truly is (priorOf, doubtOf)
  // adds the surest pose it was seen from (zero with the pose check).
  cov: [number, number, number];
  surest: PoseDoubt;
  sightings: number; firstFrame: number; lastFrame: number;
}

// A frame's hits on one wall as a measurement: z = hw·(the wall's change: offset, turn)
// − hp·(the pose's change: x, y, yaw) + noise r (the hits' own). z: how far the hits'
// middle lies off the wall (left of its direction) and, if they span enough, their slope against it;
// empty if too few hits lie away from its ends: then it only lengthens the wall.
// ends: where the frame saw the wall end (between its last hit on the wall and the first
// wedge that saw past the wall's line), and how sure that is.
export interface Measurement { wall: Wall; z: number[]; hw: number[][]; hp: number[][]; r: number[][]; hits: [number, number][]; ends: EndSeen[] }
export interface EndSeen { side: -1 | 1; x: number; y: number; sd: number }

// A frame's sighting of a wall: from what pose, the hits on it, and how far off the wall
// they were before it moved (their middle across it; their slope against it, if they spanned enough;
// null: too few away from its ends to measure it, it was only lengthened).
export interface Sighting {
  wall: number; frame: number; pose: Pose; doubt: PoseDoubt;
  hits: [number, number][]; offM: number | null; turnRad: number | null;
}

const EXTEND_M = 0.2;      // a hit this far beyond a wall's end can still be on it (and lengthens it) ...
const BEYOND_SDS = 2;      // ... if it lies this many of its own spreads from the line
const MIN_INCIDENCE = 0.5; // a ray meeting the wall at less than 30° (its sine) places a hit badly along it: not used
const NEAR_M = 1.2;        // closer and still, a hit is placed within about 2 cm; else about 8 (sights.ts, the smear)
const NEAR_SD_M = 0.02, FAR_SD_M = 0.08;
const GATE_SDS = 3;        // a hit farther off a wall than this many spreads is not on it: a new thing ...
const GATE_MAX_M = 0.15;   // ... and never farther than this, however unsure (a corner's other wall is close by)
const MIN_HITS = 3;        // fewer hits on a wall in a frame tell nothing about it
const SLOPE_SPAN_M = 0.2;  // hits spanning this much along a wall also give its direction
// A frame's hits are not independent: their middle is never surer than this, their
// slope than this; while moving (they smear) the second pair.
const MIN_SD_M = 0.01, MIN_TURN_RAD = 2 * Math.PI / 180, MOVING_MIN_SD_M = 0.03, MOVING_MIN_TURN_RAD = 6 * Math.PI / 180;
const NEW_NEAR_M = 0.1, NEW_OFF_M = 0.05, NEW_HITS = 5, NEW_SPAN_M = 0.3; // a new wall's line: this frame's hits close to its edge, at least this many, spanning this much
const NEW_SD_M = 0.05, NEW_MOVING_SD_M = 0.1; // a new wall's own spread (its hits fit a line to a few cm); made while moving
const ON_M = 0.1, ON_RAD = 10 * Math.PI / 180;      // an edge this close to a known wall is that wall, not a new one
const MERGE_M = 0.05, MERGE_RAD = 5 * Math.PI / 180; // two walls this close in line, end to end or overlapping, are one
const NEW_END_SD_M = 0.15; // a new wall's ends: from the outline, a few squares either way
const END_GAP_M = 0.5;     // the first wedge past a wall's last hit, this close along it, shows where it ends,
const PAST_M = 0.3;        // if it saw this far beyond the wall's line
const CORNER_RAD = 45 * Math.PI / 180, CORNER_M = 0.3; // two walls at this angle, ends this close to where they cross: a corner
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

// A wall's whole doubt, as a covariance like `cov`: the filter's own and the surest pose's.
export function priorOf(w: Wall): Wall["cov"] {
  return [w.cov[0] + w.surest.xyM ** 2, w.cov[1], w.cov[2] + w.surest.yawRad ** 2];
}

// How sure a wall's place is: the spread across it (at its middle) and of its direction.
export function doubtOf(w: Wall): { acrossM: number; angleRad: number } {
  const [across, , angle] = priorOf(w);
  return { acrossM: Math.sqrt(across), angleRad: Math.sqrt(angle) };
}

export function sureOf(w: Wall): number {
  const sd = doubtOf(w).acrossM;
  return Math.max(0, Math.min(1, Math.log(UNSURE_SD_M / sd) / Math.log(UNSURE_SD_M / SURE_SD_M)));
}

// A hit on a wall; atEnd: its wedge covers the wall's end, so it may read the surface round the corner.
interface Taken { u: number; v: number; weight: number; x: number; y: number; atEnd: boolean; still: boolean }

export class Walls {
  private walls: Wall[] = [];
  private next = 1;
  private renamed = new Map<number, number>(); // a wall merged into another: its number, the other's
  // The pose check (pose_check.ts), if these walls check the pose: then it keeps how sure
  // each wall is, together with the pose, and `cov` is a copy of the wall's part.
  private readonly check: PoseCheck | null;
  private readonly band: Band;

  constructor(band: Band, check: PoseCheck | null = null) { this.band = band; this.check = check; }

  all(): readonly Wall[] { return this.walls; }

  numberNow(n: number): number {
    for (let to = this.renamed.get(n); to !== undefined; to = this.renamed.get(n)) n = to;
    return n;
  }

  // One frame's hits on the walls, each wall moved by its own share (the pose isn't corrected).
  see(frame: number, pose: Pose, doubt: PoseDoubt, sights: Sight[]): Sighting[] {
    const ms = this.measure(pose, doubt, sights), changes = wallsAlone(doubt, ms);
    return this.apply(frame, pose, doubt, ms, new Map(ms.map((m, k) => [m.wall, changes[k]])));
  }

  // The frame's hits on each wall (each hit on the wall it fits best, if any), as one
  // measurement a wall: their middle across it and, if they span enough, their slope.
  measure(pose: Pose, doubt: PoseDoubt, sights: Sight[]): Measurement[] {
    const taken = new Map<Wall, Taken[]>();
    for (const s of sights) {
      if (!s.hit) continue;
      const own = s.weight >= 1 && s.hit.rangeM < NEAR_M ? NEAR_SD_M : FAR_SD_M;
      const fits: { w: Wall; t: Taken }[] = [];
      for (const w of this.walls) {
        const { u, v } = coords(w, s.hit.x, s.hit.y);
        if (Math.abs(u) > w.halfM + EXTEND_M) continue;
        const incidence = Math.abs(Math.sin(s.bearing - w.angle));
        if (incidence < MIN_INCIDENCE) continue;
        // The hit's spread across the wall: its distance's along the ray, the wedge's width across it.
        const hitVar = (own * incidence) ** 2 + (s.hit.rangeM * Math.tan(s.halfWidth) * Math.sqrt(1 - incidence ** 2)) ** 2;
        const sd = Math.sqrt(hitVar + wallVar(w, u) + lineDoubt(doubt, u - coords(w, pose.x, pose.y).u)[0]);
        if (Math.abs(v) > Math.min(GATE_MAX_M, GATE_SDS * sd)) continue;
        // Past its ends a hit is often the next wall round a corner, not known yet: there
        // it has to lie on the line by its own spread, however unsure the pose.
        if (Math.abs(u) > w.halfM && Math.abs(v) > BEYOND_SDS * Math.sqrt(hitVar)) continue;
        const atEnd = w.halfM - Math.abs(u) < 2 * s.hit.rangeM * Math.tan(s.halfWidth);
        fits.push({ w, t: { u, v, weight: 1 / hitVar, x: s.hit.x, y: s.hit.y, atEnd, still: s.still } });
      }
      // A hit that could be on two walls at an angle (in a corner) says nothing sure about
      // either: not used. Of walls in line, it goes to the one it lies least beyond.
      if (!fits.length || fits.some(f => between(f.w.angle, fits[0].w.angle) > MERGE_RAD)) continue;
      const beyond = (f: (typeof fits)[number]) => Math.abs(f.t.u) - f.w.halfM;
      const best = fits.reduce((a, b) => (beyond(b) < beyond(a) ? b : a));
      taken.set(best.w, [...(taken.get(best.w) ?? []), best.t]);
    }
    return [...taken].filter(([, hits]) => hits.length >= MIN_HITS).map(([w, hits]) => ({ ...measured(w, hits, pose), ends: endsSeen(w, hits, sights, this.band) }));
  }

  // The walls moved by their changes (pose_check.ts: those measured, and with the pose
  // check any other its correction moves too); the measured ones lengthened to their hits; their sightings.
  apply(frame: number, pose: Pose, doubt: PoseDoubt, ms: Measurement[], changes: Map<Wall, WallChange>): Sighting[] {
    for (const [w, { change: [dOff, dTurn], cov }] of changes) {
      w.cov = cov;
      // Move the line: across at its middle, and turned about it.
      const [dx, dy] = dir(w);
      w.x -= dOff * dy; w.y += dOff * dx;
      w.angle += dTurn;
    }
    const out = ms.map(m => {
      const w = m.wall;
      if (!this.check) w.surest = { xyM: Math.min(w.surest.xyM, doubt.xyM), yawRad: Math.min(w.surest.yawRad, doubt.yawRad) };
      // Lengthen it to the hits beyond its ends; then each end seen moves by its share.
      const us = m.hits.map(([x, y]) => coords(w, x, y).u), ends = [Math.min(-w.halfM, ...us), Math.max(w.halfM, ...us)];
      for (const e of m.ends) {
        const k = e.side < 0 ? 0 : 1, gain = w.endSd[k] ** 2 / (w.endSd[k] ** 2 + e.sd ** 2);
        ends[k] += gain * (coords(w, e.x, e.y).u - ends[k]);
        w.endSd[k] = Math.sqrt(1 - gain) * w.endSd[k];
      }
      this.setEnds(w, ends[0], ends[1]);
      w.sightings++; w.lastFrame = frame;
      return { wall: w.number, frame, pose, doubt, hits: m.hits, offM: m.z[0] ?? null, turnRad: m.z[1] ?? null };
    });
    this.merge();
    this.corners();
    this.sync();
    return out;
  }

  // The wall from `from` to `to` along it (from its middle now).
  private setEnds(w: Wall, from: number, to: number): void {
    if (to < from) [from, to] = [(from + to) / 2, (from + to) / 2];
    this.recentre(w, (from + to) / 2);
    w.halfM = (to - from) / 2;
  }

  // Two walls at more than CORNER_RAD to each other whose ends are both within CORNER_M of
  // where their lines cross meet there: a corner, placed by both lines.
  private corners(): void {
    this.sync();
    for (const a of this.walls)
      for (const b of this.walls) {
        if (a.number >= b.number || between(a.angle, b.angle) < CORNER_RAD) continue;
        const [ax, ay] = dir(a), [bx, by] = dir(b), cross = ax * by - ay * bx;
        const t = ((b.x - a.x) * by - (b.y - a.y) * bx) / cross, px = a.x + t * ax, py = a.y + t * ay;
        const near = (w: Wall) => { const u = coords(w, px, py).u; return { u, k: u < 0 ? 0 : 1, off: Math.abs(Math.abs(u) - w.halfM) }; };
        const na = near(a), nb = near(b);
        if (na.off > CORNER_M || nb.off > CORNER_M) continue;
        for (const [w, n, other] of [[a, na, b], [b, nb, a]] as const) {
          const ends = [-w.halfM, w.halfM];
          ends[n.k] = n.u;
          w.endSd[n.k] = Math.min(w.endSd[n.k], Math.sqrt(other.cov[0]) / Math.sin(between(a.angle, b.angle)));
          this.setEnds(w, ends[0], ends[1]);
        }
      }
  }

  // With the pose check, each wall's cov is a copy of its part of the check's covariance:
  // made fresh before every read inside Walls (merge, corners) and after every change.
  private sync(): void {
    if (this.check) for (const w of this.walls) w.cov = this.check.covOf(w);
  }

  private recentre(w: Wall, s: number): void {
    recentred(w, s);
    if (this.check) { this.check.recentred(w, s); w.cov = this.check.covOf(w); }
  }

  // New walls where the outline has a wall edge on no known wall. With the pose check, the
  // outline holds hits placed by older, less corrected poses; so there the edge only says
  // where to look, and the wall is the line through this frame's hits along it (none: not now).
  found(edges: Edge[], frame: number, pose: Pose, doubt: PoseDoubt, sights: Sight[]): Wall[] {
    const made: Wall[] = [];
    for (const e of edges) {
      if (e.kind !== "wall" || e.wall || this.walls.some(w => this.covers(w, e))) continue;
      const line = this.check ? fitted(e, sights) : { x: (e.ax + e.bx) / 2, y: (e.ay + e.by) / 2, angle: Math.atan2(e.by - e.ay, e.bx - e.ax),
                                                      halfM: Math.hypot(e.bx - e.ax, e.by - e.ay) / 2, moving: false };
      if (!line) continue;
      const w: Wall = { number: this.next++, x: line.x, y: line.y, angle: line.angle, halfM: line.halfM, endSd: [NEW_END_SD_M, NEW_END_SD_M], cov: [0, 0, 0], surest: this.check ? NO_DOUBT : doubt, sightings: 0, firstFrame: frame, lastFrame: frame };
      // Its own doubt (its hits lie within a few cm of it; more while moving: they smear), and the pose's it was seen from.
      const sd = line.moving ? NEW_MOVING_SD_M : NEW_SD_M, own: Wall["cov"] = [sd ** 2, 0, (sd / w.halfM) ** 2];
      if (this.check) { this.check.added(w, pose, own); w.cov = this.check.covOf(w); } else {
        const [pa, pc, pd] = lineDoubt(doubt, -coords(w, pose.x, pose.y).u);
        w.cov = [own[0] + pa, pc, own[2] + pd];
      }
      this.walls.push(w);
      made.push(w);
    }
    this.merge();
    this.corners();
    this.sync();
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
    this.sync();
    for (let pair = this.pairToMerge(); pair; pair = this.pairToMerge()) {
      const [a, b] = pair, keep = a.cov[0] <= b.cov[0] ? a : b, other = keep === a ? b : a, [dx, dy] = dir(other);
      this.renamed.set(b.number, a.number);
      // Both lengths: each end of the merged wall, and how sure it is, from the wall it came from.
      const ends = [{ u: -keep.halfM, sd: keep.endSd[0] }, { u: keep.halfM, sd: keep.endSd[1] },
        ...[-1, 1].map((k, i) => ({ u: coords(keep, other.x + k * other.halfM * dx, other.y + k * other.halfM * dy).u, sd: other.endSd[i] }))];
      const first = ends.reduce((p, q) => (q.u < p.u ? q : p)), last = ends.reduce((p, q) => (q.u > p.u ? q : p));
      const from = first.u, to = last.u;
      keep.endSd = [first.sd, last.sd];
      this.check?.removed(other);
      this.recentre(keep, (from + to) / 2);
      Object.assign(keep, { number: a.number, halfM: (to - from) / 2, sightings: a.sightings + b.sightings,
                            surest: { xyM: Math.min(a.surest.xyM, b.surest.xyM), yawRad: Math.min(a.surest.yawRad, b.surest.yawRad) },
                            firstFrame: Math.min(a.firstFrame, b.firstFrame), lastFrame: Math.max(a.lastFrame, b.lastFrame) });
      this.walls = this.walls.filter(w => w !== other);
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

// A wall's hits as one measurement (pose_check.ts): z = hw·(wall's change) − hp·(pose's change) + noise r.
// Only the hits away from the wall's ends measure it (too few: z is empty); all lengthen it (apply).
function measured(w: Wall, all: Taken[], pose: Pose): Measurement {
  const hits = all.filter(h => !h.atEnd);
  if (hits.length < MIN_HITS) return { wall: w, z: [], hw: [], hp: [], r: [], hits: all.map(h => [h.x, h.y]), ends: [] };
  const total = hits.reduce((m, h) => m + h.weight, 0);
  const u0 = hits.reduce((m, h) => m + h.weight * h.u, 0) / total, off = hits.reduce((m, h) => m + h.weight * h.v, 0) / total;
  const spread = hits.reduce((m, h) => m + h.weight * (h.u - u0) ** 2, 0);
  const span = Math.max(...hits.map(h => h.u)) - Math.min(...hits.map(h => h.u));
  // Moving the pose moves the hits: across the wall by the shift's part across it, and by
  // the turn times how far along the wall they are from the robot; their slope by the turn.
  const [dx, dy] = dir(w), along = u0 - coords(w, pose.x, pose.y).u;
  const still = hits.every(h => h.still), minSd = still ? MIN_SD_M : MOVING_MIN_SD_M, minTurn = still ? MIN_TURN_RAD : MOVING_MIN_TURN_RAD;
  const m: Measurement = { wall: w, z: [off], hw: [[1, u0]], hp: [[-dy, dx, along]], r: [[Math.max(1 / total, minSd ** 2)]], hits: all.map(h => [h.x, h.y]), ends: [] };
  if (span >= SLOPE_SPAN_M) {
    m.z.push(Math.atan(hits.reduce((s, h) => s + h.weight * (h.u - u0) * (h.v - off), 0) / spread));
    m.hw.push([0, 1]); m.hp.push([0, 0, 1]);
    m.r[0].push(0); m.r.push([0, Math.max(1 / spread, minTurn ** 2)]);
  }
  return m;
}

// The line through a frame's hits along an edge (within NEW_NEAR_M of it, not past its
// ends: there they may be a corner's other wall), fitted twice: the second time without
// the hits farther than NEW_OFF_M off the first line. As long as the edge and the hits
// together; moving: most of its hits were taken while moving (they smear);
// null: too few hits, or too short a span.
function fitted(e: Edge, sights: Sight[]): (Pick<Wall, "x" | "y" | "angle" | "halfM"> & { moving: boolean }) | null {
  const edge = lineOf((e.ax + e.bx) / 2, (e.ay + e.by) / 2, Math.atan2(e.by - e.ay, e.bx - e.ax), Math.hypot(e.bx - e.ax, e.by - e.ay) / 2);
  let near = sights.flatMap(s => s.hit ? [{ x: s.hit.x, y: s.hit.y, still: s.still }] : [])
    .filter(h => { const c = coords(edge, h.x, h.y); return Math.abs(c.v) <= NEW_NEAR_M && Math.abs(c.u) <= edge.halfM; });
  let line: Wall | null = null;
  for (let round = 0; round < 2; round++) {
    if (line) { const l = line; near = near.filter(h => Math.abs(coords(l, h.x, h.y).v) <= NEW_OFF_M); }
    if (near.length < NEW_HITS) return null;
    const mx = near.reduce((m, h) => m + h.x, 0) / near.length, my = near.reduce((m, h) => m + h.y, 0) / near.length;
    const a = near.reduce((m, h) => m + (h.x - mx) ** 2, 0), b = near.reduce((m, h) => m + (h.x - mx) * (h.y - my), 0), c = near.reduce((m, h) => m + (h.y - my) ** 2, 0);
    line = lineOf(mx, my, Math.atan2(2 * b, a - c) / 2, 0);
  }
  const l = line!;
  if (Math.cos(l.angle - edge.angle) < 0) l.angle += Math.PI; // the edge's way round
  const us = near.map(h => coords(l, h.x, h.y).u);
  if (Math.max(...us) - Math.min(...us) < NEW_SPAN_M || between(l.angle, edge.angle) > ON_RAD) return null;
  us.push(coords(l, e.ax, e.ay).u, coords(l, e.bx, e.by).u);
  const from = Math.min(...us), to = Math.max(...us), [dx, dy] = dir(l), s = (from + to) / 2;
  const moving = near.filter(h => !h.still).length > near.length / 2;
  return { x: l.x + dx * s, y: l.y + dy * s, angle: l.angle, halfM: (to - from) / 2, moving };
}

const lineOf = (x: number, y: number, angle: number, halfM: number): Wall =>
  ({ number: 0, x, y, angle, halfM, endSd: [0, 0], cov: [0, 0, 0], surest: NO_DOUBT, sightings: 0, firstFrame: 0, lastFrame: 0 });

// Where the frame saw the wall end, each side: between the last hit on it and the first
// wedge whose middle ray crossed its line past that hit (closer than END_GAP_M) and saw
// through at the band's heights: the end is between them, within half their gap (never
// surer than half a wedge's width there). The gap is the wedges' spacing along the wall
// there: small seen close and square on, large far or at a slant.
function endsSeen(w: Wall, hits: Taken[], sights: Sight[], band: Band): EndSeen[] {
  const out: EndSeen[] = [], [dx, dy] = dir(w);
  for (const side of [-1, 1] as const) {
    const on = Math.max(...hits.map(h => side * h.u));
    let past: { u: number; wedge: number } | null = null;
    for (const s of sights) {
      const p = coords(w, s.x, s.y), du = Math.cos(s.bearing - w.angle), dv = Math.sin(s.bearing - w.angle);
      if (Math.abs(dv) < MIN_INCIDENCE * 0.5) continue; // along the wall: it says little about where it ends
      const t = -p.v / dv;                              // how far along the ray it crosses the line
      // The line is behind it, it didn't see well past it (a hit smeared behind the wall is
      // no opening), or it passed over the band.
      if (t <= 0 || s.reachM < t + PAST_M || !seesBand(band, s, t)) continue;
      const u = side * (p.u + t * du);
      if (u > on && u - on <= END_GAP_M && (!past || u < past.u)) past = { u, wedge: t * Math.tan(s.halfWidth) };
    }
    if (!past) continue;
    const u = side * (on + past.u) / 2;
    out.push({ side, x: w.x + u * dx, y: w.y + u * dy, sd: Math.max((past.u - on) / 2, past.wedge) });
  }
  return out;
}
