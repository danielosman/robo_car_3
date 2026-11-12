// Who gives way, the pose or the walls (doc/OUTLINE_MAP_PLAN.md, idea 3): a frame's
// measurements of the walls it sees (walls.ts, measure) share the gap between the hits and
// the walls out by how sure each is. A Kalman filter.
//   wallsAlone(doubt, measurements)   each wall moved by its own share, the pose's doubt as noise
//                                     (the bands whose walls don't check the pose)
//   new PoseCheck()                   the pose and one band's walls, with how sure each is and how
//                                     their errors go together (one covariance over all of them):
//     check.moved(from, to)           odometry moved the pose: its doubt grows
//     check.added(wall, pose, own)    a new wall, placed from the pose: it shares the pose's error
//     check.removed(wall), check.recentred(wall, s)   a wall merged away; a wall's middle moved along it
//     check.check(measurements)       the pose's change and every wall's that put the hits back on the walls;
//                                     a measurement too far off is left out
//     check.doubt(), check.covOf(wall)
// A measurement says z = hw·(wall's change) − hp·(pose's change) + noise: a wall moved
// towards the hits, or the pose moved so that the hits land on the wall.
// Why one covariance over all: a wall placed from an unsure pose is off by the pose's
// error. Seen again from there it tells nothing about where the pose truly is, only how
// it moved since. Without the covariance each check made pose and wall look sure, and the
// walls took up the drift (tried first: `square` stayed 9° off).
import { kalmanStep, type Matrix } from "./kalman.ts";
import { growth, type Pose, type PoseDoubt } from "./pose.ts";
import type { Measurement, Wall } from "./walls.ts";

// A wall's change: its offset across at its middle and its turn; its covariance after.
export interface WallChange { change: [number, number]; cov: Wall["cov"] }

const poseCov = (d: PoseDoubt): Matrix => [[d.xyM ** 2, 0, 0], [0, d.xyM ** 2, 0], [0, 0, d.yawRad ** 2]];
const wallCov = (w: Wall): Matrix => [[w.cov[0], w.cov[1]], [w.cov[1], w.cov[2]]];
const asWall = (c: Matrix): Wall["cov"] => [c[0][0], c[0][1], c[1][1]];

export function wallsAlone(doubt: PoseDoubt, ms: Measurement[]): WallChange[] {
  const P = poseCov(doubt);
  return ms.map(m => {
    const fromPose = m.hp.map(a => m.hp.map(b => a.reduce((s, v, i) => s + v * P[i].reduce((t, p, j) => t + p * b[j], 0), 0)));
    if (!m.z.length) return { change: [0, 0], cov: m.wall.cov };
    const { change, cov } = kalmanStep(wallCov(m.wall), m.hw, m.z, m.r.map((row, i) => row.map((v, j) => v + fromPose[i][j])));
    return { change: [change[0], change[1]], cov: asWall(cov) };
  });
}

const OFF_SDS = 3;

export class PoseCheck {
  private walls: Wall[] = [];  // the state: the pose's change (x, y, yaw), then each wall's (offset, turn)
  private P: Matrix = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];

  doubt(): PoseDoubt { return { xyM: Math.sqrt(Math.max(this.P[0][0], this.P[1][1], 0)), yawRad: Math.sqrt(Math.max(this.P[2][2], 0)) }; }
  covOf(w: Wall): Wall["cov"] { const i = this.at(w); return [this.P[i][i], this.P[i][i + 1], this.P[i + 1][i + 1]]; }
  private at(w: Wall): number { const k = this.walls.indexOf(w); if (k < 0) throw new Error(`wall ${w.number} is not checked`); return 3 + 2 * k; }

  // Rows (and the same columns) of the state replaced by T times them: T maps `rows` to themselves.
  private transform(rows: number[], T: Matrix): void {
    const P = this.P, n = P.length;
    const byRow = rows.map((_, i) => Array.from({ length: n }, (__, c) => T[i].reduce((s, t, k) => s + t * P[rows[k]][c], 0)));
    rows.forEach((r, i) => { P[r] = byRow[i]; });
    const byCol = rows.map((_, i) => Array.from({ length: n }, (__, r) => T[i].reduce((s, t, k) => s + t * P[r][rows[k]], 0)));
    rows.forEach((c, i) => { for (let r = 0; r < n; r++) P[r][c] = byCol[i][r]; });
  }

  moved(from: Pose, to: Pose): void {
    // An error in the heading turns the way driven: the position is off across it.
    this.transform([0, 1, 2], [[1, 0, -(to.y - from.y)], [0, 1, to.x - from.x], [0, 0, 1]]);
    // The odometry's own drift: a spread that adds up (pose.ts).
    const more = growth(from, to), sd = this.doubt();
    const xy = (sd.xyM + more.xyM) ** 2 - sd.xyM ** 2, yaw = (sd.yawRad + more.yawRad) ** 2 - sd.yawRad ** 2;
    this.P[0][0] += xy; this.P[1][1] += xy; this.P[2][2] += yaw;
  }

  added(w: Wall, pose: Pose, own: Wall["cov"]): void {
    // How the wall's offset at its middle and its turn follow the pose's error (as a measurement's hp).
    const dx = Math.cos(w.angle), dy = Math.sin(w.angle), along = (w.x - pose.x) * dx + (w.y - pose.y) * dy;
    const G = [[-dy, dx, along], [0, 0, 1]], P = this.P, n = P.length;
    const GP = G.map(g => Array.from({ length: n }, (_, c) => g.reduce((s, v, k) => s + v * P[k][c], 0)));
    const GPGt = GP.map(row => G.map(g => g.reduce((s, v, k) => s + v * row[k], 0)));
    P.forEach((row, r) => row.push(GP[0][r], GP[1][r]));
    P.push([...GP[0], GPGt[0][0] + own[0], GPGt[0][1] + own[1]], [...GP[1], GPGt[1][0] + own[1], GPGt[1][1] + own[2]]);
    this.walls.push(w);
  }

  removed(w: Wall): void {
    const i = this.at(w);
    this.P = this.P.filter((_, r) => r !== i && r !== i + 1).map(row => row.filter((_, c) => c !== i && c !== i + 1));
    this.walls = this.walls.filter(x => x !== w);
  }

  // The wall's middle moved s along it: its offset there is offset + s·turn.
  recentred(w: Wall, s: number): void { const i = this.at(w); this.transform([i, i + 1], [[1, s], [0, 1]]); }

  // ms: the frame's measurements; those too far off to be the wall (OFF_SDS spreads, the
  // pose's, the wall's and the hits' doubt together) are left out: a thing moved or a
  // corner's other wall. Gives what was used.
  check(ms: Measurement[]): { pose: [number, number, number]; walls: Map<Wall, WallChange>; used: Measurement[] } {
    const n = this.P.length, rowsOf = (m: Measurement) => {
      const i = this.at(m.wall);
      return m.z.map((_, r) => {
        const row = new Array<number>(n).fill(0);
        m.hp[r].forEach((v, j) => { row[j] = -v; });
        row[i] = m.hw[r][0]; row[i + 1] = m.hw[r][1];
        return row;
      });
    };
    const used = ms.filter(m => {
      if (!m.z.length) return true; // it only lengthens the wall
      const G = rowsOf(m), S = G.map((a, r) => G.map((b, c) => a.reduce((s, v, i) => s + v * this.P[i].reduce((t, p, j) => t + p * b[j], 0), 0) + m.r[r][c]));
      const far = S.length === 1 ? m.z[0] ** 2 / S[0][0]
        : (S[1][1] * m.z[0] ** 2 - 2 * S[0][1] * m.z[0] * m.z[1] + S[0][0] * m.z[1] ** 2) / (S[0][0] * S[1][1] - S[0][1] ** 2);
      return far <= OFF_SDS ** 2 * m.z.length;
    });
    if (!used.some(m => m.z.length)) return { pose: [0, 0, 0], walls: new Map(), used };
    const G = used.flatMap(rowsOf), z = used.flatMap(m => m.z);
    // The hits' own noise: each measurement's block on the diagonal.
    const R: Matrix = z.map(() => new Array<number>(z.length).fill(0));
    let at = 0;
    for (const m of used) { m.r.forEach((row, i) => row.forEach((v, j) => { R[at + i][at + j] = v; })); at += m.z.length; }
    const { change, cov } = kalmanStep(this.P, G, z, R);
    this.P = cov;
    const walls = new Map<Wall, WallChange>();
    this.walls.forEach((w, k) => walls.set(w, { change: [change[3 + 2 * k], change[4 + 2 * k]], cov: this.covOf(w) }));
    return { pose: [change[0], change[1], change[2]], walls, used };
  }
}
