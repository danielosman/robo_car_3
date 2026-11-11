// The simulator's room (doc/OUTLINE_MAP_PLAN.md, "The simulator"): flat shapes on the
// floor plan, each with a height span, some only there for part of the drive. The room
// is the truth the map under test is measured against.
//   wall(points, ...)  an open run of wall faces (leave a gap for a door)
//   box(...), round(...)  closed things: a box, a many-sided round thing (a leg, a pouf)
//   facesAt(room, step)   the faces standing at a step; allFaces(room): at any step
//   cast(faces, ...)      how far a ray goes before it meets a face at its height
export interface Point { x: number; y: number }

// A shape: its corners in order, closed (a thing) or open (a run of walls), the heights
// it spans, and the steps (moves, counted from 0) it stands in: from `fromStep` up to,
// not including, `untilStep`.
export interface Shape { points: Point[]; closed: boolean; lowM: number; highM: number; fromStep: number; untilStep: number }
export type Room = Shape[];

interface Span { lowM?: number; highM?: number; fromStep?: number; untilStep?: number }
const shape = (points: Point[], closed: boolean, s: Span): Shape =>
  ({ points, closed, lowM: s.lowM ?? 0, highM: s.highM ?? Infinity, fromStep: s.fromStep ?? 0, untilStep: s.untilStep ?? Infinity });

// A run of walls through the corners given as x, y, x, y, ...; floor to ceiling unless `span` says otherwise.
export function wall(xy: number[], span: Span = {}): Shape {
  const points: Point[] = [];
  for (let i = 0; i < xy.length; i += 2) points.push({ x: xy[i], y: xy[i + 1] });
  return shape(points, false, span);
}

// A box with its sides along x and y, from (x0, y0) to (x1, y1).
export const box = (x0: number, y0: number, x1: number, y1: number, span: Span = {}): Shape =>
  shape([{ x: x0, y: y0 }, { x: x1, y: y0 }, { x: x1, y: y1 }, { x: x0, y: y1 }], true, span);

// A round thing of radius r around (x, y), as a shape of `sides` sides.
export const round = (x: number, y: number, r: number, sides: number, span: Span = {}): Shape =>
  shape(Array.from({ length: sides }, (_, i) => ({ x: x + r * Math.cos(2 * Math.PI * i / sides), y: y + r * Math.sin(2 * Math.PI * i / sides) })), true, span);

// One straight face of a shape, with the shape's heights.
export interface Face { ax: number; ay: number; bx: number; by: number; lowM: number; highM: number; shape: Shape }

export function facesAt(room: Room, step: number): Face[] {
  const faces: Face[] = [];
  for (const s of room) {
    if (step < s.fromStep || step >= s.untilStep) continue;
    for (let i = 0; i + 1 < s.points.length + (s.closed ? 1 : 0); i++) {
      const a = s.points[i], b = s.points[(i + 1) % s.points.length];
      faces.push({ ax: a.x, ay: a.y, bx: b.x, by: b.y, lowM: s.lowM, highM: s.highM, shape: s });
    }
  }
  return faces;
}

export const allFaces = (room: Room): Face[] => facesAt(room.map(s => ({ ...s, fromStep: 0, untilStep: Infinity })), 0);

// How far (on the floor plan) a ray from (x, y) along `bearing` goes before it meets a
// face, and that face; null: it meets none within maxM. `reaches(d, face)` says whether
// the ray is at the face's heights d from its start (default: always).
export function cast(faces: Face[], x: number, y: number, bearing: number, maxM: number,
                     reaches: (d: number, face: Face) => boolean = () => true): { d: number; face: Face } | null {
  const ux = Math.cos(bearing), uy = Math.sin(bearing);
  let best: { d: number; face: Face } | null = null;
  for (const f of faces) {
    const ex = f.bx - f.ax, ey = f.by - f.ay, cross = ux * ey - uy * ex;
    if (Math.abs(cross) < 1e-12) continue; // along the face
    const d = ((f.ax - x) * ey - (f.ay - y) * ex) / cross, t = ((f.ax - x) * uy - (f.ay - y) * ux) / cross;
    if (d <= 1e-9 || d > maxM || t < 0 || t > 1 || (best && d >= best.d) || !reaches(d, f)) continue;
    best = { d, face: f };
  }
  return best;
}
