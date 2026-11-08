// The canvas: boxes, ribbons, change nodes and rate labels, drawn with d3 from a
// layout. Boxes drag (d3.drag), the canvas pans and zooms (d3.zoom).
import * as d3 from "d3";
import type { Flow } from "../../data/schema.ts";
import { flows, isChange } from "./data.ts";
import { HEAD_H, ROW_H, ribbonPath, type Box, type Layout, type Node, type Ribbon } from "./layout.ts";
import type { Selection } from "./panels.ts";

export interface Handlers {
  select(s: Selection): void;
  drag(id: string, x: number, y: number, done: boolean): void;
}

const SLOTS = 8; // the palette's categorical slots (style.css --series-1..8), in fixed order
const tops = flows.filter(f => !f.parent);
export const colorOf = (root: Flow) => `var(--series-${(tops.indexOf(root) % SLOTS) + 1})`;

export function createCanvas(svgEl: SVGSVGElement) {
  const svg = d3.select(svgEl);
  const world = svg.append("g");
  const layers = Object.fromEntries(["groups", "ribbons", "boxes", "nodes", "rates"].map(k => [k, world.append("g").attr("class", k)])) as
    Record<"groups" | "ribbons" | "boxes" | "nodes" | "rates", d3.Selection<SVGGElement, unknown, null, undefined>>;
  const zoom = d3.zoom<SVGSVGElement, unknown>().scaleExtent([0.1, 3]).on("zoom", e => world.attr("transform", e.transform.toString()));
  svg.call(zoom);
  let fitted = false;
  // The first draw shows the boxes the selected flow passes, or everything.
  function fit(l: Layout, flowId?: string | null): void {
    const mine = flowId ? new Set(l.ribbons.filter(r => r.flow.id === flowId).flatMap(r => [r.from.box, r.to.box])) : null;
    const bs = mine?.size ? [...mine] : [...l.boxes.values()];
    if (!bs.length) return;
    const x0 = Math.min(...bs.map(b => b.x)) - 20, x1 = Math.max(...bs.map(b => b.x + b.w)) + 20;
    const y0 = Math.min(...bs.map(b => b.y)) - 40, y1 = Math.max(...bs.map(b => b.y + b.h)) + 20;
    const { width, height } = svgEl.getBoundingClientRect();
    const k = Math.min(1, width / (x1 - x0), height / (y1 - y0));
    svg.call(zoom.transform, d3.zoomIdentity.translate(-x0 * k, -y0 * k).scale(k));
  }

  function draw(l: Layout, sel: Selection, on: Handlers): void {
    const selFlow = sel?.kind === "flow" ? sel.id : sel?.kind === "node" ? sel.flow : null;
    svg.classed("dim", selFlow !== null);
    // the members the selected flow passes: their rows are marked
    const touched = new Set<string>();
    for (const f of flows) if (f.id === selFlow) for (const s of f.steps) if (s.at) touched.add(s.at);

    layers.groups.selectAll<SVGTextElement, Layout["groups"][number]>("text").data(l.groups, g => g.text)
      .join("text").attr("class", "group").attr("x", g => g.x).attr("y", g => g.y).text(g => g.text);

    // ribbons: a visible path and a wider, invisible one to click
    const ribbonKey = (r: Ribbon) => `${r.flow.id}|${r.stepIndex}`;
    const rib = layers.ribbons.selectAll<SVGGElement, Ribbon>("g.ribbon").data(l.ribbons, ribbonKey)
      .join(enter => {
        const g = enter.append("g").attr("class", "ribbon");
        g.append("path").attr("class", "band");
        g.append("path").attr("class", "hit");
        g.append("title");
        return g;
      });
    rib.classed("on", r => r.flow.id === selFlow)
      .classed("planned", r => r.flow.status === "planned").classed("leaving", r => r.flow.status === "leaving")
      .on("click", (e: MouseEvent, r) => { e.stopPropagation(); on.select({ kind: "flow", id: r.flow.id }); });
    rib.select("path.band").attr("d", ribbonPath).attr("stroke", r => colorOf(r.root)).attr("stroke-width", r => r.width);
    rib.select("path.hit").attr("d", ribbonPath).attr("stroke-width", r => Math.max(10, r.width));
    rib.select("title").text(r => `${r.flow.name} (${r.flow.id}): ${r.bytes} B${r.rate ? `, ${r.rate}` : ""}`);

    // rate labels at the middle of the ribbon where a rate starts
    const mid = (r: Ribbon) => {
      const dx = Math.max(60, Math.abs(r.to.x - r.from.x) / 2);
      const x = (r.from.x + 3 * (r.from.x + r.from.side * dx) + 3 * (r.to.x + r.to.side * dx) + r.to.x) / 8;
      return [x, (r.from.y + r.to.y) / 2] as const;
    };
    const seen = new Set<string>();
    const rated = l.ribbons.filter(r => {
      const key = `${r.from.box.module.id}|${r.from.member}|${r.rate}`;
      if (!r.showRate || seen.has(key)) return false;
      seen.add(key);
      return true;
    });
    const rateKey = (r: Ribbon) => `${r.from.box.module.id}|${r.from.member}|${r.rate}`;
    const selectedRates = new Set(l.ribbons.filter(r => r.flow.id === selFlow).map(rateKey));
    layers.rates.selectAll<SVGTextElement, Ribbon>("text").data(rated, ribbonKey)
      .join("text").attr("class", "rate").classed("on", r => selectedRates.has(rateKey(r)))
      .attr("x", r => mid(r)[0]).attr("y", r => mid(r)[1] - r.width / 2 - 3).text(r => r.rate ?? "");

    // boxes
    const box = layers.boxes.selectAll<SVGGElement, Box>("g.box").data([...l.boxes.values()], b => b.module.id)
      .join(enter => {
        const g = enter.append("g").attr("class", "box");
        g.append("rect").attr("class", "frame");
        g.append("text").attr("class", "name");
        g.append("line").attr("class", "rule");
        return g;
      });
    box.attr("transform", b => `translate(${b.x},${b.y})`)
      .classed("on", b => sel?.kind === "module" && sel.id === b.module.id)
      .classed("planned", b => b.module.status === "planned").classed("leaving", b => b.module.status === "leaving")
      .on("click", (e: MouseEvent, b) => { if (e.defaultPrevented) return; e.stopPropagation(); on.select({ kind: "module", id: b.module.id }); });
    box.select("rect.frame").attr("width", b => b.w).attr("height", b => b.h).attr("rx", 4);
    box.select("text.name").attr("x", 10).attr("y", 18).text(b => b.module.name)
      .append("tspan").attr("class", "origin").text(b => `  ${b.module.origin}`);
    box.select("line.rule").attr("x1", 0).attr("x2", b => b.w).attr("y1", HEAD_H - 4).attr("y2", HEAD_H - 4);
    box.selectAll<SVGTextElement, { name: string; id: string; planned: boolean }>("text.member")
      .data(b => b.module.members.map(m => ({ name: m.name, id: `${b.module.id}.${m.name}`, planned: m.status === "planned" })), m => m.id)
      .join("text").attr("class", "member").attr("x", 10)
      .attr("y", (_, i) => HEAD_H + i * ROW_H + ROW_H / 2 + 4)
      .classed("touched", m => touched.has(m.id)).classed("planned", m => m.planned)
      .text(m => m.name);
    // The drag's subject is the box as it was when the drag began (its x, y), so
    // e.x, e.y are its new place; the box objects are made anew on every draw.
    box.call(d3.drag<SVGGElement, Box>()
      .on("drag", (e, b) => on.drag(b.module.id, e.x, e.y, false))
      .on("end", (e, b) => on.drag(b.module.id, e.x, e.y, true)));

    // change nodes: pick hollow, turn filled, add dashed (style.css)
    const nodeKey = (n: Node) => `${n.flow.id}|${n.stepIndex}`;
    layers.nodes.selectAll<SVGCircleElement, Node>("circle").data(l.nodes, nodeKey)
      .join(enter => { const c = enter.append("circle"); c.append("title"); return c; })
      .attr("class", n => { const s = n.flow.steps[n.stepIndex]; return `node ${isChange(s) ? s.change : ""}`; })
      .classed("on", n => n.flow.id === selFlow || (sel?.kind === "node" && sel.flow === n.flow.id && sel.step === n.stepIndex))
      .attr("cx", n => n.x).attr("cy", n => n.y).attr("r", 5)
      .style("--c", n => colorOf(n.root))
      .on("click", (e: MouseEvent, n) => { e.stopPropagation(); on.select({ kind: "node", flow: n.flow.id, step: n.stepIndex }); })
      .select("title").text(n => {
        const s = n.flow.steps[n.stepIndex];
        return isChange(s) ? `${s.change} "${s.name}" in ${n.flow.id}${s.what ? `: ${s.what}` : ""}` : "";
      });

    svg.on("click", () => on.select(null));
    if (!fitted) { fit(l, selFlow); fitted = true; }
  }
  const refit = (l: Layout) => fit(l);
  return { draw, refit };
}
