// Straight lines through a group of points (doc/LINE_MAP_PLAN.md, "Lines"): the straight
// stretch that goes through the most points is taken out as a line, again and again,
// until too few points are left. Points that lie on no such stretch (clutter) give no
// line. Pure geometry: no map, no sights.
export interface Point { x: number; y: number }
export interface Segment { ax: number; ay: number; bx: number; by: number }

const DIRECTIONS = 60;  // tried over the half turn: 3° apart
const STRIP_M = 0.03;   // a point is on a line when this close to it
// A stretch ends where the next point along it is farther than this (the map groups
// points no farther apart: line_map.ts, NEAR_SPOTS).
const GAP_M = 0.15;
const REFITS = 3;       // a stretch is fitted and gathered again this often while it grows
const MIN_POINTS = 4;   // fewer points are not a line

export function straightLines(points: Point[]): Segment[] {
  const lines: Segment[] = [];
  let left = points;
  while (left.length >= MIN_POINTS) {
    let stretch = longestStretch(left);
    if (stretch.length < MIN_POINTS) break;
    // The direction tried is up to 1.5° off, which cuts a long wall short: fit a line
    // through the stretch and take the points on that, while it gains points.
    for (let i = 0; i < REFITS; i++) {
      const fitted = stretchOn(left, fit(stretch));
      if (fitted.length <= stretch.length) break;
      stretch = fitted;
    }
    lines.push(segmentOf(stretch, fit(stretch)));
    const taken = new Set(stretch);
    left = left.filter(p => !taken.has(p));
  }
  return lines;
}

// A line through (x, y) along the unit vector (ux, uy).
interface Line { x: number; y: number; ux: number; uy: number }
const along = (l: Line, p: Point) => (p.x - l.x) * l.ux + (p.y - l.y) * l.uy;
const off = (l: Line, p: Point) => (p.x - l.x) * l.uy - (p.y - l.y) * l.ux;

// Over every direction: the strip 2 x STRIP_M wide with the most points in it, and of
// those the longest run without a gap. The longest such run of all.
function longestStretch(points: Point[]): Point[] {
  let best: Point[] = [];
  for (let d = 0; d < DIRECTIONS; d++) {
    const angle = d * Math.PI / DIRECTIONS, line = { x: 0, y: 0, ux: Math.cos(angle), uy: Math.sin(angle) };
    const across = points.map(p => ({ p, off: off(line, p) })).sort((a, b) => a.off - b.off);
    let from = 0, most = 0, mostFrom = 0;
    for (let to = 0; to < across.length; to++) {
      while (across[to].off - across[from].off > 2 * STRIP_M) from++;
      if (to - from + 1 > most) { most = to - from + 1; mostFrom = from; }
    }
    if (most <= best.length) continue;
    const run = longestRun(across.slice(mostFrom, mostFrom + most).map(a => a.p), line);
    if (run.length > best.length) best = run;
  }
  return best;
}

// Of the points, in order along the line: the most in a row with no gap between neighbours.
function longestRun(points: Point[], line: Line): Point[] {
  const ordered = points.map(p => ({ p, at: along(line, p) })).sort((a, b) => a.at - b.at);
  let best: Point[] = [], from = 0;
  for (let i = 1; i <= ordered.length; i++) {
    if (i < ordered.length && ordered[i].at - ordered[i - 1].at <= GAP_M) continue;
    if (i - from > best.length) best = ordered.slice(from, i).map(o => o.p);
    from = i;
  }
  return best;
}

// The points on the line (within STRIP_M): their longest run without a gap.
function stretchOn(points: Point[], line: Line): Point[] {
  return longestRun(points.filter(p => Math.abs(off(line, p)) <= STRIP_M), line);
}

// The line through the points' middle, along their longest spread.
function fit(points: Point[]): Line {
  const n = points.length, x = points.reduce((s, p) => s + p.x, 0) / n, y = points.reduce((s, p) => s + p.y, 0) / n;
  let xx = 0, xy = 0, yy = 0;
  for (const p of points) { xx += (p.x - x) ** 2; xy += (p.x - x) * (p.y - y); yy += (p.y - y) ** 2; }
  const angle = Math.atan2(2 * xy, xx - yy) / 2;
  return { x, y, ux: Math.cos(angle), uy: Math.sin(angle) };
}

function segmentOf(points: Point[], line: Line): Segment {
  const at = points.map(p => along(line, p)), a = Math.min(...at), b = Math.max(...at);
  return { ax: line.x + line.ux * a, ay: line.y + line.uy * a, bx: line.x + line.ux * b, by: line.y + line.uy * b };
}
