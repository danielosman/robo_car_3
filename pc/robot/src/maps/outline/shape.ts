// A border loop's points as edges of three kinds (doc/OUTLINE_MAP_PLAN.md, idea 1): wall
// (backed by hits, long and straight), rough (backed by hits, short or jagged), guess
// (nothing seen there yet). Pure geometry: no map, no sights.
//   edgesOf(points)   the closed loop's edges, in order: each ends where the next starts
export type EdgeKind = "wall" | "rough" | "guess";
export interface Edge { ax: number; ay: number; bx: number; by: number; kind: EdgeKind }
// A point of the border: on a hit, or where the seen-free floor just ends.
export interface BorderPoint { x: number; y: number; hit: boolean }

const WALL_MIN_M = 0.4;   // a wall is a straight stretch at least this long ...
const WALL_OFF_M = 0.03;  // ... whose hits lie this close to it
const ROUGH_OFF_M = 0.06; // rough and guess stretches are simplified this coarsely
const GUESS_OFF_M = 0.04;
const GAP_M = 0.3;        // two hits farther apart are not joined by a hit edge: an opening may be between them (a zone is 25 cm wide at the sensor's reach)

const far = (a: BorderPoint, b: BorderPoint) => Math.hypot(b.x - a.x, b.y - a.y);

// Ramer-Douglas-Peucker: the points of an open run to keep so that none is farther than
// `off` from the kept line. The run's two ends always stay.
function simplified(run: BorderPoint[], off: number): BorderPoint[] {
  const keep = new Array<boolean>(run.length).fill(false), todo: [number, number][] = [[0, run.length - 1]];
  keep[0] = keep[run.length - 1] = true;
  for (let span = todo.pop(); span; span = todo.pop()) {
    const [from, to] = span, a = run[from], b = run[to], length = far(a, b);
    let worst = 0, at = -1;
    for (let i = from + 1; i < to; i++) {
      const p = run[i];
      const d = length < 1e-9 ? far(a, p) : Math.abs((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)) / length;
      if (d > worst) { worst = d; at = i; }
    }
    if (worst > off) { keep[at] = true; todo.push([from, at], [at, to]); }
  }
  return run.filter((_, i) => keep[i]);
}

const edges = (run: BorderPoint[], kind: EdgeKind): Edge[] =>
  run.slice(1).map((b, i) => ({ ax: run[i].x, ay: run[i].y, bx: b.x, by: b.y, kind }));

// An open run of hits: its long straight stretches are walls; what lies between them is rough.
function hitEdges(run: BorderPoint[]): Edge[] {
  const corners = simplified(run, WALL_OFF_M), out: Edge[] = [];
  let roughFrom = 0; // the run's index where the rough stretch being gathered starts
  let at = 0;
  for (let c = 1; c < corners.length; c++) {
    const from = at;
    at = run.indexOf(corners[c], from + 1);
    if (far(corners[c - 1], corners[c]) < WALL_MIN_M) continue;
    if (from > roughFrom) out.push(...edges(simplified(run.slice(roughFrom, from + 1), ROUGH_OFF_M), "rough"));
    out.push(...edges([corners[c - 1], corners[c]], "wall"));
    roughFrom = at;
  }
  if (run.length - 1 > roughFrom) out.push(...edges(simplified(run.slice(roughFrom), ROUGH_OFF_M), "rough"));
  return out;
}

// The loop without knots. Moving border points onto hits, and simplifying, can make a
// loop cut through itself (round a thing's corner, a door post). Where two edges cross,
// the shorter way round between them (in metres) is dropped: both edges end at the crossing point.
// How far along edge e the point (x, y) on it lies: 0 at its start, 1 at its end.
const shareAlong = (e: Edge, x: number, y: number) => { const m = Math.hypot(e.bx - e.ax, e.by - e.ay); return m < 1e-12 ? 0 : Math.hypot(x - e.ax, y - e.ay) / m; };

function unknotted(loop: Edge[]): Edge[] {
  const edges = loop.map(e => ({ ...e }));
  const side = (e: Edge, x: number, y: number) => (e.bx - e.ax) * (y - e.ay) - (e.by - e.ay) * (x - e.ax);
  for (let again = true; again && edges.length > 3;) {
    again = false;
    for (let i = 0; i < edges.length && !again; i++)
      for (let j = i + 2; j < edges.length && !again; j++) {
        const a = edges[i], b = edges[j], a1 = side(b, a.ax, a.ay), a2 = side(b, a.bx, a.by);
        if (a1 * a2 >= -1e-12 || side(a, b.ax, b.ay) * side(a, b.bx, b.by) >= -1e-12) continue;
        const t = a1 / (a1 - a2), x = a.ax + t * (a.bx - a.ax), y = a.ay + t * (a.by - a.ay);
        const far = (e: Edge) => Math.hypot(e.bx - e.ax, e.by - e.ay), sum = (list: Edge[]) => list.reduce((m, e) => m + far(e), 0);
        const between = sum(edges.slice(i + 1, j)) + (1 - t) * far(a) + far(b) * shareAlong(b, x, y);
        const round = sum(edges) - between;
        if (between <= round) { // drop the edges between i and j
          a.bx = b.ax = x; a.by = b.ay = y;
          edges.splice(i + 1, j - i - 1);
        } else {                               // drop the edges after j and before i
          b.bx = a.ax = x; b.by = a.ay = y;
          edges.splice(j + 1);
          edges.splice(0, i);
        }
        again = true;
      }
  }
  return edges;
}

export function edgesOf(points: BorderPoint[]): Edge[] {
  return unknotted(kindsOf(points));
}

function kindsOf(points: BorderPoint[]): Edge[] {
  const n = points.length;
  if (n === 0) return [];
  if (n === 1) return [{ ax: points[0].x, ay: points[0].y, bx: points[0].x, by: points[0].y, kind: points[0].hit ? "rough" : "guess" }];
  // The step from point i to the next is backed by hits if both are hits, close together.
  const solid = points.map((p, i) => { const q = points[(i + 1) % n]; return p.hit && q.hit && far(p, q) <= GAP_M; });
  let start = solid.findIndex((s, i) => s !== solid[(i + n - 1) % n]);
  if (start < 0) {
    // One kind all round: cut the loop at its first point and at the point farthest from it.
    let farthest = 1;
    for (let i = 2; i < n; i++) if (far(points[0], points[i]) > far(points[0], points[farthest])) farthest = i;
    const halves = [points.slice(0, farthest + 1), [...points.slice(farthest), points[0]]];
    return halves.flatMap(run => solid[0] ? hitEdges(run) : edges(simplified(run, GUESS_OFF_M), "guess"));
  }
  const out: Edge[] = [];
  for (let done = 0; done < n;) {
    const run = [points[start]], kind = solid[start];
    while (done < n && solid[start] === kind) { start = (start + 1) % n; run.push(points[start]); done++; }
    out.push(...(kind ? hitEdges(run) : edges(simplified(run, GUESS_OFF_M), "guess")));
  }
  return out;
}
