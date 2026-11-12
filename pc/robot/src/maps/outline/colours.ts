// How the outlines are drawn (doc/OUTLINE_MAP_PLAN.md, idea 1): a colour a band (white:
// L1, yellow: L2, cyan: above) and a brightness an edge: a wall strong, brighter the surer
// its place; a rough edge medium; a guess faint.
//   edgeRgb(band, edge)        an edge's colour
//   outlineStrokes(outline)    one band's edges as coloured lines to draw
import type { BandName } from "./sights.ts";
import type { Edge } from "./shape.ts";
import type { BandOutline } from "./mapper.ts";

export type Rgb = readonly [number, number, number];
// A line to draw, in metres.
export interface Stroke { ax: number; ay: number; bx: number; by: number; rgb: Rgb }

export const BAND_RGB: Record<BandName, Rgb> = { L1: [0xff, 0xff, 0xff], L2: [0xf2, 0xe2, 0x4a], above: [0x30, 0xd0, 0xd0] };
// A wall found by the rule but not known yet is drawn as an unsure known wall.
const GUESS = 0.3, ROUGH = 0.55, WALL_UNSURE = 0.7, WALL_SURE = 1;

const shadeOf = (e: Edge) => e.kind === "guess" ? GUESS : e.kind === "rough" ? ROUGH : WALL_UNSURE + (WALL_SURE - WALL_UNSURE) * (e.wall?.sure ?? 0);

export const edgeRgb = (band: BandName, e: Edge): Rgb => {
  const shade = shadeOf(e);
  return [Math.round(BAND_RGB[band][0] * shade), Math.round(BAND_RGB[band][1] * shade), Math.round(BAND_RGB[band][2] * shade)];
};

// One band's edges as lines, the faint ones first so the strong ones are drawn over them.
export function outlineStrokes(outline: BandOutline): Stroke[] {
  return outline.loops.flatMap(loop => loop.edges).sort((a, b) => shadeOf(a) - shadeOf(b))
    .map(e => ({ ax: e.ax, ay: e.ay, bx: e.bx, by: e.by, rgb: edgeRgb(outline.band, e) }));
}
