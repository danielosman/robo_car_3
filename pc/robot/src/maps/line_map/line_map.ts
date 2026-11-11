// One band's map of lines (doc/LINE_MAP_PLAN.md): what stands where, as straight lines on
// the floor plan, built up frame by frame from sights (sights.ts, inBand).
//   map.add(sights) one frame's sights (one call a frame: a spot counts the frames it was hit in)
//   map.lines()     the lines standing now
// The map keeps spots, not lines: for each 5 cm square that was hit, where its hits lie
// (their weighted middle), a score and the heights it was hit at. A hit raises the
// spot's score; a sight that saw through the spot lowers it. Lines are drawn when read,
// through the spots that stand (straight.ts): one frame's hits are too few and too
// noisy to tell a line's direction, many frames' hits are not.
import { heightsAt, type Sight } from "./sights.ts";
import { straightLines, type Point, type Segment } from "./straight.ts";

export type { Segment };

const SPOT_M = 0.05;
const SCORE_MAX = 5;      // a spot's score stops here, so a thing that went can be forgotten
const FRAMES_TO_SHOW = 3; // a spot hit in fewer frames isn't drawn: noise
// Spots this many squares apart or closer are looked at together for lines: no closer
// than the gap a line may have (straight.ts, GAP_M).
const NEAR_SPOTS = 3;
const HEIGHT_M = 0.01;    // a wedge this little into a spot's heights doesn't count as seeing through it
// Only the wedge's middle half sees through: a wedge sliding along a wall reads what is
// behind the wall's end, and would remove the wall under its edge.
const SEE_SHARE = 0.5;

interface Spot {
  ix: number; iy: number;                         // its square
  score: number; frames: number; lastFrame: number;
  weight: number; sumX: number; sumY: number;     // its hits: their middle is (sumX, sumY) / weight
  lowM: number; highM: number;                    // the heights it was hit at
}

const keyOf = (ix: number, iy: number) => (ix + 32768) * 65536 + iy + 32768; // ±1.6 km of squares
const square = (m: number) => Math.floor(m / SPOT_M);
const middle = (s: Spot): Point => ({ x: s.sumX / s.weight, y: s.sumY / s.weight });

export class LineMap {
  private spots = new Map<number, Spot>();
  private frame = 0;

  add(sights: Sight[]): void {
    this.frame++;
    for (const s of sights) this.hit(s);
    for (const s of sights) this.seeThrough(s);
  }

  // The straight lines through each group of standing spots that lie near each other.
  lines(): Segment[] {
    const standing = new Map([...this.spots].filter(([, s]) => s.frames >= FRAMES_TO_SHOW));
    const lines: Segment[] = [];
    for (const first of standing.values()) { // a spot taken into a group leaves `standing`, so it isn't met again
      standing.delete(keyOf(first.ix, first.iy));
      const group: Point[] = [], todo = [first];
      for (let s = todo.pop(); s; s = todo.pop()) {
        group.push(middle(s));
        for (let dx = -NEAR_SPOTS; dx <= NEAR_SPOTS; dx++)
          for (let dy = -NEAR_SPOTS; dy <= NEAR_SPOTS; dy++) {
            const key = keyOf(s.ix + dx, s.iy + dy), near = standing.get(key);
            if (near) { standing.delete(key); todo.push(near); }
          }
      }
      lines.push(...straightLines(group));
    }
    return lines;
  }

  private hit(s: Sight): void {
    if (!s.hit) return;
    const h = s.hit, ix = square(h.x), iy = square(h.y), key = keyOf(ix, iy);
    const spot = this.spots.get(key) ?? { ix, iy, score: 0, frames: 0, lastFrame: 0, weight: 0, sumX: 0, sumY: 0, lowM: h.lowM, highM: h.highM };
    // A close hit is placed more exactly than a far one: by its zone's width there.
    const weight = s.weight / (h.rangeM * Math.tan(s.halfWidth)) ** 2;
    spot.weight += weight;
    spot.sumX += weight * h.x;
    spot.sumY += weight * h.y;
    spot.score = Math.min(SCORE_MAX, spot.score + s.weight);
    if (spot.lastFrame !== this.frame) { spot.frames++; spot.lastFrame = this.frame; }
    spot.lowM = Math.min(spot.lowM, h.lowM);
    spot.highM = Math.max(spot.highM, h.highM);
    this.spots.set(key, spot);
  }

  // Lowers the spots the sight saw through: those in its wedge's middle, closer than its
  // reach, hit at heights the wedge covers there. A spot at zero is forgotten.
  private seeThrough(s: Sight): void {
    if (s.reachM <= 0) return;
    const ux = Math.cos(s.bearing), uy = Math.sin(s.bearing), spread = Math.tan(s.halfWidth * SEE_SHARE);
    // The squares the wedge can touch: those around its tip and its two far corners.
    const xs = [s.x, s.x + s.reachM * (ux - spread * uy), s.x + s.reachM * (ux + spread * uy)];
    const ys = [s.y, s.y + s.reachM * (uy + spread * ux), s.y + s.reachM * (uy - spread * ux)];
    for (let ix = square(Math.min(...xs)); ix <= square(Math.max(...xs)); ix++)
      for (let iy = square(Math.min(...ys)); iy <= square(Math.max(...ys)); iy++) {
        const spot = this.spots.get(keyOf(ix, iy));
        if (!spot) continue;
        const p = middle(spot), along = (p.x - s.x) * ux + (p.y - s.y) * uy, across = (p.x - s.x) * uy - (p.y - s.y) * ux;
        if (along <= 0 || along > s.reachM || Math.abs(across) > along * spread) continue;
        const [low, high] = heightsAt(s, along);
        if (low > spot.highM - HEIGHT_M || high < spot.lowM + HEIGHT_M) continue; // it passed over or under
        spot.score -= s.weight;
        if (spot.score <= 0) this.spots.delete(keyOf(ix, iy));
      }
  }
}
