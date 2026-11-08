// Where everything goes: boxes (placed by origin, or where they were dragged), the
// ribbons between them and the change nodes. Pure: no drawing, recomputed on a drag.
import type { Flow, Module } from "../../data/schema.ts";
import { carriedAlong, flows, isChange, memberById, modules, origins, rootFlow } from "./data.ts";

export const BOX_W = 240, HEAD_H = 28, ROW_H = 16, PAD = 8;
const COL_GAP = 70, GROUP_GAP = 160, MAX_COL_H = 1500, TOP = 60, AREA_PAD = 18;

export interface Box { module: Module; x: number; y: number; w: number; h: number; row: Map<string, number> }
export interface End { box: Box; member: string; side: 1 | -1; x: number; y: number }
export interface Ribbon { flow: Flow; root: Flow; from: End; to: End; bytes: number; rate?: string; showRate: boolean; width: number; stepIndex: number }
export interface Node { flow: Flow; root: Flow; stepIndex: number; x: number; y: number }
// An origin's area: a dashed frame around its boxes, its label at the top.
export interface Area { text: string; x: number; y: number; w: number; h: number }
export interface Layout { boxes: Map<string, Box>; ribbons: Ribbon[]; nodes: Node[]; areas: Area[] }

export const boxHeight = (m: Module) => HEAD_H + m.members.length * ROW_H + PAD;
export const rowY = (b: Box, member: string) => b.y + HEAD_H + (b.row.get(member) ?? 0) * ROW_H + ROW_H / 2;

// Width by bytes, log scale: 1 B → 2 px, 20 KB → a row's height.
export function widthOf(bytes: number): number {
  const t = Math.log10(Math.max(1, bytes)) / Math.log10(20000);
  return 2 + Math.min(1, t) * (ROW_H - 4);
}

// The first place each module appears in the flows: the order boxes stack in.
function appearance(): Map<string, number> {
  const order = new Map<string, number>();
  for (const f of flows) for (const s of f.steps) {
    const mod = s.at ? memberById.get(s.at)?.module.id : undefined;
    if (mod && !order.has(mod)) order.set(mod, order.size);
  }
  return order;
}

// The starting place of every box: a group per origin, left to right; inside, boxes
// stack in order of appearance, a new column when one gets too tall.
export function initialPositions(): Map<string, [number, number]> {
  const order = appearance(), out = new Map<string, [number, number]>();
  let x = 40;
  for (const o of origins) {
    const mine = modules.filter(m => m.origin === o.id)
      .sort((a, b) => (order.get(a.id) ?? 1e9) - (order.get(b.id) ?? 1e9));
    if (!mine.length) continue;
    let y = TOP, colX = x;
    for (const m of mine) {
      const h = boxHeight(m);
      if (y > TOP && y + h > MAX_COL_H) { colX += BOX_W + COL_GAP; y = TOP; }
      out.set(m.id, [colX, y]);
      y += h + 24;
    }
    x = colX + BOX_W + GROUP_GAP;
  }
  return out;
}

export function layout(positions: Map<string, [number, number]>): Layout {
  const boxes = new Map<string, Box>();
  for (const m of modules) {
    const [x, y] = positions.get(m.id) ?? [0, 0];
    boxes.set(m.id, { module: m, x, y, w: BOX_W, h: boxHeight(m), row: new Map(m.members.map((mb, i) => [mb.name, i])) });
  }
  const areas: Area[] = origins.flatMap(o => {
    const mine = [...boxes.values()].filter(b => b.module.origin === o.id);
    if (!mine.length) return [];
    const x = Math.min(...mine.map(b => b.x)) - AREA_PAD, y = Math.min(...mine.map(b => b.y)) - AREA_PAD - 22;
    const w = Math.max(...mine.map(b => b.x + b.w)) + AREA_PAD - x, h = Math.max(...mine.map(b => b.y + b.h)) + AREA_PAD - y;
    return [{ text: o.label, x, y, w, h }];
  });

  const ribbons: Ribbon[] = [], nodes: Node[] = [];
  const nodesOnRow = new Map<string, number>();
  const anchor = (at: string) => {
    const t = memberById.get(at);
    return t ? { box: boxes.get(t.module.id)!, member: t.member.name } : null;
  };

  for (const f of flows) {
    const root = rootFlow(f), carried = carriedAlong(f);
    let prev: { box: Box; member: string; i: number } | null = null;
    let lastRate: string | undefined;
    f.steps.forEach((s, i) => {
      const a = s.at ? anchor(s.at) : null;
      if (!a) return; // resolve reports it
      if (isChange(s)) { // a node on its member's row, right to left
        const key = `${a.box.module.id}|${a.member}`, k = nodesOnRow.get(key) ?? 0;
        nodesOnRow.set(key, k + 1);
        nodes.push({ flow: f, root, stepIndex: i, x: a.box.x + a.box.w - 10 - 12 * k, y: rowY(a.box, a.member) });
      }
      // a ribbon from the last box to this one (none inside one box)
      if (prev && prev.box !== a.box) {
        const c = carried[prev.i] ?? {};
        const [fromSide, toSide] = sides(prev.box, a.box);
        const bytes = c.bytes ?? 0;
        ribbons.push({ flow: f, root, from: end(prev.box, prev.member, fromSide), to: end(a.box, a.member, toSide), bytes,
          rate: c.rate, showRate: !!c.rate && c.rate !== lastRate, width: widthOf(bytes), stepIndex: prev.i });
        lastRate = c.rate;
      }
      prev = { box: a.box, member: a.member, i };
    });
  }
  spread(ribbons);
  return { boxes, ribbons, nodes, areas };
}

// Which side each end leaves from: the sides that face each other; both right when
// the boxes overlap across (one column).
function sides(a: Box, b: Box): [1 | -1, 1 | -1] {
  if (b.x >= a.x + a.w) return [1, -1];
  if (b.x + b.w <= a.x) return [-1, 1];
  return [1, 1];
}
const end = (box: Box, member: string, side: 1 | -1): End =>
  ({ box, member, side, x: side === 1 ? box.x + box.w : box.x, y: rowY(box, member) });

// Ribbons at one row and side sit side by side, scaled down to fit the row.
function spread(ribbons: Ribbon[]): void {
  const at = new Map<string, { r: Ribbon; e: End }[]>();
  for (const r of ribbons) for (const e of [r.from, r.to]) {
    const key = `${e.box.module.id}|${e.member}|${e.side}`;
    at.set(key, [...(at.get(key) ?? []), { r, e }]);
  }
  const scale = new Map<Ribbon, number>();
  for (const list of at.values()) {
    const total = list.reduce((s, x) => s + x.r.width, 0) + (list.length - 1);
    const k = Math.min(1, (ROW_H - 2) / total);
    for (const x of list) scale.set(x.r, Math.min(scale.get(x.r) ?? 1, k));
  }
  for (const r of ribbons) r.width *= scale.get(r) ?? 1;
  for (const list of at.values()) {
    const total = list.reduce((s, x) => s + x.r.width, 0) + (list.length - 1);
    let y = -total / 2;
    for (const x of list) { x.e.y += y + x.r.width / 2; y += x.r.width + 1; }
  }
}

// A ribbon's path: a cubic curve leaving and entering along each end's side.
export function ribbonPath(r: Ribbon): string {
  const { from: a, to: b } = r;
  const dx = Math.max(60, Math.abs(b.x - a.x) / 2);
  return `M${a.x},${a.y} C${a.x + a.side * dx},${a.y} ${b.x + b.side * dx},${b.y} ${b.x},${b.y}`;
}
