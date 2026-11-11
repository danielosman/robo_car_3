// One band's outline map (doc/OUTLINE_MAP_PLAN.md, idea 1): the floor seen free at the
// band's heights, built up frame by frame from sights (sights.ts), and its border: one
// closed outline, and a closed island round each thing standing in it.
//   new OutlineMap(band)  the floor starts as a 35 cm circle round the start
//   map.add(sights)       one frame's sights (one call a frame: a spot counts the frames it was hit in)
//   map.outline()         the outline and the islands, as edges of three kinds (shape.ts)
// The map keeps 5 cm squares. A square is seen free when a sight's wedge passed through
// it at the band's heights. A square that was hit keeps where its hits lie (their
// weighted middle), a score and the heights it was hit at: a spot. A hit raises the
// spot's score; a sight that saw through the spot lowers it. A spot hit in three
// frames stands: its square is not free, whatever saw through it.
// The outline is read from that: the border of the free squares joined to the start,
// each border point moved onto the standing spot just beyond it, if there is one.
import { bordersOf, connected, keyOf, type Key } from "./border.ts";
import { edgesOf, type BorderPoint, type Edge } from "./shape.ts";
import { heightsAt, inBand, type Band, type Sight } from "./sights.ts";

export type { Edge };
// A closed run of edges: the outline of the seen-free floor, or an island in it.
export interface Loop { island: boolean; edges: Edge[] }

const SPOT_M = 0.05;
const START_M = 0.35;     // the floor seen free at the start: a circle of this radius (as the cell map's, picoA/app/cell_map.c)
const SCORE_MAX = 5;      // a spot's score stops here, so a thing that went can be forgotten
const FRAMES_TO_STAND = 3; // a spot hit in fewer frames doesn't stand: noise
const HEIGHT_M = 0.01;    // a wedge this little into a spot's heights doesn't count as seeing through it
// Only the wedge's middle half sees through a spot: a wedge sliding along a wall reads
// what is behind the wall's end, and would remove the wall under its edge.
const SEE_SHARE = 0.5;
// A wedge sees a band's floor free where it covers the band's lower part: things stand
// on the floor, so a wedge passing over the band's top says nothing.
const LOWER_PART = 0.6, LOWER_PART_MAX_M = 0.1, INTO_BAND_M = 0.02;
// A border point is moved onto the nearest standing spot this close beyond it. Sights
// end a margin before what they hit (sights.ts), so the free floor ends before the hits.
const SNAP_M = 0.3;
const SNAP_CONE = 0.5;    // "beyond": within 60° of straight out of the free floor
// Along the border the same spot can come up again a few points later (the free floor's
// edge is a staircase; each step looks for its spot a little differently): the points
// between are dropped, so the border doesn't run back and forth along a wall.
const SAME_AGAIN = 8;
// Closer to the sensor than this a wedge is too thin to cover a band's lower part (the
// robot stands there): what it saw through counts as free for every band.
const NEAR_M = 0.45;
const SMALL_HOLE = 12;    // a hole with this few border points and no hit is a gap between wedges, not an island

interface Spot {
  score: number; frames: number; lastFrame: number;
  weight: number; sumX: number; sumY: number;     // its hits: their middle is (sumX, sumY) / weight
  lowM: number; highM: number;                    // the heights it was hit at
}

const square = (m: number) => Math.floor(m / SPOT_M);
const middle = (s: Spot) => ({ x: s.sumX / s.weight, y: s.sumY / s.weight });

export class OutlineMap {
  private spots = new Map<Key, Spot>();
  private free = new Set<Key>();
  private frame = 0;
  private readonly start: Key;
  private readonly band: Band;
  private readonly lowerPartM: number;

  constructor(band: Band, start: { x: number; y: number } = { x: 0, y: 0 }) {
    this.band = band;
    this.lowerPartM = band.lowM + Math.min(LOWER_PART_MAX_M, LOWER_PART * (band.highM - band.lowM));
    this.start = keyOf(square(start.x), square(start.y));
    const reach = Math.ceil(START_M / SPOT_M);
    for (let ix = square(start.x) - reach; ix <= square(start.x) + reach; ix++)
      for (let iy = square(start.y) - reach; iy <= square(start.y) + reach; iy++)
        if (Math.hypot((ix + 0.5) * SPOT_M - start.x, (iy + 0.5) * SPOT_M - start.y) <= START_M) this.free.add(keyOf(ix, iy));
  }

  add(sights: Sight[]): void {
    this.frame++;
    const own = inBand(sights, this.band);
    for (const s of own) this.hit(s);
    for (const s of own) this.seeThrough(s);
  }

  outline(): Loop[] {
    // The start square never stands: the outline is the floor joined to it.
    const standing = new Map([...this.spots].filter(([key, s]) => s.frames >= FRAMES_TO_STAND && key !== this.start));
    const floor = connected(new Set([...this.free].filter(key => !standing.has(key))), this.start);
    const loops: Loop[] = [];
    for (const border of bordersOf(floor)) {
      let area = 0; // twice the area the border goes round: more than 0 for a hole (anticlockwise), less for the outer border
      const points: (BorderPoint & { spot: Spot | null })[] = [];
      border.forEach((c, i) => {
        const next = border[(i + 1) % border.length];
        area += (c.ix + c.dx / 2) * (next.iy + next.dy / 2) - (next.ix + next.dx / 2) * (c.iy + c.dy / 2);
        const x = (c.ix + 0.5 + c.dx / 2) * SPOT_M, y = (c.iy + 0.5 + c.dy / 2) * SPOT_M;
        const spot = standing.get(keyOf(c.ix + c.dx, c.iy + c.dy)) ?? this.spotBeyond(standing, x, y, c.dx, c.dy);
        if (spot) {
          const again = points.findLastIndex((p, k) => k >= points.length - SAME_AGAIN && p.spot === spot);
          if (again >= 0) { points.length = again + 1; return; }
        }
        points.push(spot ? { ...middle(spot), hit: true, spot } : { x, y, hit: false, spot: null });
      });
      const island = area > 0;
      if (island && border.length <= SMALL_HOLE && !points.some(p => p.hit)) continue;
      loops.push({ island, edges: edgesOf(points) });
    }
    return loops;
  }

  // The standing spot nearest to the border point (x, y) and beyond it: out of the free floor, along (dx, dy).
  private spotBeyond(standing: Map<Key, Spot>, x: number, y: number, dx: number, dy: number): Spot | null {
    const reach = Math.ceil(SNAP_M / SPOT_M), ix = square(x + dx * SPOT_M / 2), iy = square(y + dy * SPOT_M / 2);
    let best: Spot | null = null, bestM = SNAP_M;
    for (let a = -reach; a <= reach; a++)
      for (let b = -reach; b <= reach; b++) {
        const spot = standing.get(keyOf(ix + a, iy + b));
        if (!spot) continue;
        const p = middle(spot), vx = p.x - x, vy = p.y - y, d = Math.hypot(vx, vy);
        if (d < bestM && vx * dx + vy * dy >= SNAP_CONE * d) { best = spot; bestM = d; }
      }
    return best;
  }

  private hit(s: Sight): void {
    if (!s.hit) return;
    const h = s.hit, key = keyOf(square(h.x), square(h.y));
    const spot = this.spots.get(key) ?? { score: 0, frames: 0, lastFrame: 0, weight: 0, sumX: 0, sumY: 0, lowM: h.lowM, highM: h.highM };
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

  // What the sight saw through, closer than its reach: the squares in its wedge are seen
  // free where the wedge covers the band's lower part; the spots in its wedge's middle,
  // hit at heights the wedge covers there, are lowered. A spot at zero is forgotten.
  private seeThrough(s: Sight): void {
    if (s.reachM <= 0) return;
    const ux = Math.cos(s.bearing), uy = Math.sin(s.bearing), spread = Math.tan(s.halfWidth);
    // The squares the wedge can touch: those around its tip and its two far corners.
    const xs = [s.x, s.x + s.reachM * (ux - spread * uy), s.x + s.reachM * (ux + spread * uy)];
    const ys = [s.y, s.y + s.reachM * (uy + spread * ux), s.y + s.reachM * (uy - spread * ux)];
    for (let ix = square(Math.min(...xs)); ix <= square(Math.max(...xs)); ix++)
      for (let iy = square(Math.min(...ys)); iy <= square(Math.max(...ys)); iy++) {
        const key = keyOf(ix, iy), x = (ix + 0.5) * SPOT_M, y = (iy + 0.5) * SPOT_M;
        const along = (x - s.x) * ux + (y - s.y) * uy, across = (x - s.x) * uy - (y - s.y) * ux;
        if (along > 0 && along <= s.reachM && Math.abs(across) <= along * spread) {
          const [low, high] = heightsAt(s, along);
          if (along <= NEAR_M || (low <= this.lowerPartM && high >= this.band.lowM + INTO_BAND_M)) this.free.add(key);
        }
        const spot = this.spots.get(key);
        if (!spot) continue;
        const p = middle(spot), spotAlong = (p.x - s.x) * ux + (p.y - s.y) * uy, spotAcross = (p.x - s.x) * uy - (p.y - s.y) * ux;
        if (spotAlong <= 0 || spotAlong > s.reachM || Math.abs(spotAcross) > spotAlong * spread * SEE_SHARE) continue;
        const [low, high] = heightsAt(s, spotAlong);
        if (low > spot.highM - HEIGHT_M || high < spot.lowM + HEIGHT_M) continue; // it passed over or under
        spot.score -= s.weight;
        if (spot.score <= 0) this.spots.delete(key);
      }
  }
}
