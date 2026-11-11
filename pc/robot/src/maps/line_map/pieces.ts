// One row's hits as straight pieces (doc/LINE_MAP_PLAN.md, "Pieces"). A row is a scan
// from left to right, so neighbours are known: a run of hits is broken where a zone has
// no hit or two neighbours differ much in distance, then split at its worst-fitting point until
// every point fits (Ramer-Douglas-Peucker). A lone hit is a stub across its zone.
// Pure 2D geometry on sights; one band at a time (sights.ts, inBand).
import type { Hit, Sight, SightRow } from "./sights.ts";

// A straight stretch of something standing, seen in one frame.
export interface Piece {
  ax: number; ay: number; bx: number; by: number;  // its ends, a on the left as seen
  // The hits it goes through. fitWeight: how much a hit pulls a line fitted through it
  // (the sight's weight / its zone's width squared, 1/m²: a close hit pulls much more).
  points: { x: number; y: number; fitWeight: number }[];
  slackM: number;   // how far a point of it may lie from the line it belongs to
  weight: number;   // how much the sighting counts as evidence (the sights' weights, averaged: 0-1)
  lowM: number; highM: number; // the heights it was seen at
}

// Neighbours whose distances differ by more than this share of the nearer one plus JUMP_M
// are not one thing. A wall met at 30° steps by 17 % from zone to zone; a shallower one
// breaks into stubs.
const JUMP_SHARE = 0.2;
const JUMP_M = 0.03;
const FIT_M = 0.03;      // a point fits a line when closer than this, or 2 sigma, or
const FIT_SHARE = 0.35;  // this share of its zone's width, whichever is the largest

type Seen = Sight & { hit: Hit };
const widthM = (s: Seen) => 2 * s.hit.rangeM * Math.tan(s.halfWidth);
const slackM = (s: Seen) => Math.max(FIT_M, 2 * s.hit.sigmaM, FIT_SHARE * widthM(s));

export function piecesOf(row: SightRow): Piece[] {
  const pieces: Piece[] = [];
  let run: Seen[] = [];
  const end = () => { if (run.length) split(run, pieces, false, false); run = []; };
  for (const s of row) {
    if (!s?.hit) { end(); continue; }
    const seen = s as Seen, last = run.at(-1);
    if (last && Math.abs(seen.hit.rangeM - last.hit.rangeM) > JUMP_SHARE * Math.min(seen.hit.rangeM, last.hit.rangeM) + JUMP_M) end();
    run.push(seen);
  }
  end();
  return pieces;
}

// How far p lies from the line through a and b.
function offLine(p: Hit, a: Hit, b: Hit): number {
  const dx = b.x - a.x, dy = b.y - a.y;
  return Math.abs(dx * (p.y - a.y) - dy * (p.x - a.x)) / Math.hypot(dx, dy);
}

// The run as pieces: one if every point between its ends fits the line through them;
// otherwise split at the point that fits worst, which ends one piece and starts the next.
// cutBefore, cutAfter: the run's first, last hit is such a shared point.
function split(run: Seen[], pieces: Piece[], cutBefore: boolean, cutAfter: boolean): void {
  let worst = 0, at = -1;
  for (let i = 1; i < run.length - 1; i++) {
    const over = offLine(run[i].hit, run[0].hit, run.at(-1)!.hit) - slackM(run[i]);
    if (over > worst) { worst = over; at = i; }
  }
  if (at < 0) { pieces.push(pieceOf(run, cutBefore, cutAfter)); return; }
  split(run.slice(0, at + 1), pieces, cutBefore, true);
  split(run.slice(at), pieces, true, cutAfter);
}

function pieceOf(run: Seen[], cutBefore: boolean, cutAfter: boolean): Piece {
  const first = run[0], last = run.at(-1)!;
  // Along the piece, from its left end to its right: for a lone hit, across its wedge.
  let ux = last.hit.x - first.hit.x, uy = last.hit.y - first.hit.y;
  if (run.length === 1) { ux = Math.sin(first.bearing); uy = -Math.cos(first.bearing); } else { const length = Math.hypot(ux, uy); ux /= length; uy /= length; }
  // An outer zone saw it across its whole width: that end goes out to the zone's border.
  // An end at a split stops at the shared hit: the next piece starts there.
  const before = cutBefore ? 0 : widthM(first) / 2, after = cutAfter ? 0 : widthM(last) / 2;
  return {
    ax: first.hit.x - ux * before, ay: first.hit.y - uy * before, bx: last.hit.x + ux * after, by: last.hit.y + uy * after,
    points: run.map(s => ({ x: s.hit.x, y: s.hit.y, fitWeight: s.weight / widthM(s) ** 2 })),
    slackM: Math.max(...run.map(slackM)),
    weight: run.reduce((sum, s) => sum + s.weight, 0) / run.length,
    lowM: Math.min(...run.map(s => s.hit.lowM)), highM: Math.max(...run.map(s => s.hit.highM)),
  };
}
