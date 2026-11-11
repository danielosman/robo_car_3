// The border of a set of squares on a grid (doc/OUTLINE_MAP_PLAN.md, idea 1): closed
// loops, by nature. Pure geometry on whole-number squares: no map, no sights.
//   connected(inside, from)   the squares of `inside` joined to `from` side by side
//   bordersOf(inside)         the loops round them: the outer border and every hole
// A loop is a ring of crossings. A crossing is the step from a square inside to its
// neighbour outside; the border passes between the two. Going along a loop the inside
// is on the right: an outer border runs clockwise, a hole's anticlockwise (x right, y up).
// Squares that touch only at a corner are not joined, so loops never cross or touch.

export type Key = number;
export const keyOf = (ix: number, iy: number): Key => (ix + 32768) * 65536 + iy + 32768; // ±32767 squares
export const squareOf = (key: Key): [number, number] => [Math.floor(key / 65536) - 32768, (key % 65536) - 32768];

// The step from the square (ix, iy) inside to its neighbour (ix + dx, iy + dy) outside.
export interface Crossing { ix: number; iy: number; dx: number; dy: number }

const SIDES = [[1, 0], [0, 1], [-1, 0], [0, -1]] as const;

export function connected(inside: Set<Key>, from: Key): Set<Key> {
  const found = new Set<Key>();
  if (!inside.has(from)) return found;
  found.add(from);
  const todo = [from];
  for (let key = todo.pop(); key !== undefined; key = todo.pop()) {
    const [ix, iy] = squareOf(key);
    for (const [dx, dy] of SIDES) {
      const next = keyOf(ix + dx, iy + dy);
      if (inside.has(next) && !found.has(next)) { found.add(next); todo.push(next); }
    }
  }
  return found;
}

export function bordersOf(inside: Set<Key>): Crossing[][] {
  // A crossing's name: its inside square and which side it leaves by.
  const name = (ix: number, iy: number, side: number) => keyOf(ix, iy) * 4 + side;
  const crossings = new Map<number, Crossing>(), next = new Map<number, number>();
  const has = (ix: number, iy: number) => inside.has(keyOf(ix, iy));
  // Every corner point where four squares meet, (cx, cy) being its upper right square:
  // the crossings between those four squares are joined there, two by two.
  const corners = new Set<Key>();
  for (const key of inside) {
    const [ix, iy] = squareOf(key);
    for (const [cx, cy] of [[ix, iy], [ix + 1, iy], [ix, iy + 1], [ix + 1, iy + 1]]) corners.add(keyOf(cx, cy));
  }
  for (const corner of corners) {
    const [cx, cy] = squareOf(corner);
    // The four squares round the corner, anticlockwise from the lower left one.
    const round = [[cx - 1, cy - 1], [cx, cy - 1], [cx, cy], [cx - 1, cy]];
    for (let i = 0; i < 4; i++) {
      const [ix, iy] = round[i];
      if (!has(ix, iy)) continue;
      // The border comes in between this square and the one before it round the corner
      // (outside), and leaves between this square and the one after; if that one is
      // inside too, between it and the next (and so on). So the inside is on its right.
      const [bx, by] = round[(i + 3) % 4];
      if (has(bx, by)) continue; // the border doesn't come in at this square
      let j = i;
      while (has(...(round[(j + 1) % 4] as [number, number])) && (j + 1) % 4 !== (i + 3) % 4) j = (j + 1) % 4;
      const [jx, jy] = round[j], [ax, ay] = round[(j + 1) % 4];
      const from = { ix, iy, dx: bx - ix, dy: by - iy }, to = { ix: jx, iy: jy, dx: ax - jx, dy: ay - jy };
      const sideOf = (c: Crossing) => SIDES.findIndex(([dx, dy]) => dx === c.dx && dy === c.dy);
      const fromName = name(from.ix, from.iy, sideOf(from)), toName = name(to.ix, to.iy, sideOf(to));
      crossings.set(fromName, from);
      crossings.set(toName, to);
      next.set(fromName, toName);
    }
  }
  const loops: Crossing[][] = [], seen = new Set<number>();
  for (const start of next.keys()) {
    if (seen.has(start)) continue;
    const loop: Crossing[] = [];
    for (let at = start; !seen.has(at); at = next.get(at)!) { seen.add(at); loop.push(crossings.get(at)!); }
    loops.push(loop);
  }
  return loops;
}
