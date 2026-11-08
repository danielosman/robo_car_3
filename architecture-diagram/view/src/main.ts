// The page: the flow list, the canvas and the side bar, kept in step. Box positions
// after a drag are kept in the browser; "reset layout" forgets them.
import { createCanvas, colorOf } from "./draw.ts";
import { modules, resolved } from "./data.ts";
import { initialPositions, layout } from "./layout.ts";
import { renderList, renderSide, type Selection } from "./panels.ts";

const STORE = "architecture-diagram/positions";
const $ = (id: string) => document.getElementById(id)!;

function savedPositions(): Map<string, [number, number]> {
  const out = initialPositions();
  try {
    const saved = JSON.parse(localStorage.getItem(STORE) ?? "{}") as Record<string, [number, number]>;
    for (const m of modules) if (saved[m.id]) out.set(m.id, saved[m.id]);
  } catch { /* a broken entry: the starting layout */ }
  return out;
}

let positions = savedPositions();
let selection: Selection = null;
const canvas = createCanvas($("canvas") as unknown as SVGSVGElement);

function render(): void {
  canvas.draw(layout(positions), selection, { select, drag });
  renderList($("list"), colorOf, selection, select);
  renderSide($("side"), selection, resolved.errors);
}
// The selection is in the address (#flow=tilt-stop, #module=picoB.drive,
// #node=tilt-stop/1), so a link shows one flow, module or node.
function fromHash(): Selection {
  const [kind, value] = decodeURIComponent(location.hash.slice(1)).split("=");
  if (kind === "flow" && value) return { kind, id: value };
  if (kind === "module" && value) return { kind, id: value };
  if (kind === "node" && value) { const [flow, step] = value.split("/"); return { kind, flow, step: Number(step) }; }
  return null;
}
function toHash(s: Selection): string {
  if (!s) return "";
  return s.kind === "node" ? `#node=${s.flow}/${s.step}` : `#${s.kind}=${s.id}`;
}
// A click on what is selected unselects it (the background doesn't).
function select(s: Selection): void {
  selection = s && toHash(s) === toHash(selection) ? null : s;
  history.replaceState(null, "", toHash(selection) || location.pathname);
  render();
}
function drag(id: string, x: number, y: number, done: boolean): void {
  positions.set(id, [x, y]);
  if (done) localStorage.setItem(STORE, JSON.stringify(Object.fromEntries(positions)));
  canvas.draw(layout(positions), selection, { select, drag });
}

// The side bars fold away; which are folded is kept in the browser.
const FOLDED = "architecture-diagram/folded";
function fold(which: "list" | "side", folded: boolean): void {
  $("main").classList.toggle(`hide-${which}`, folded);
  $(`fold-${which}`).textContent = `${folded ? "show" : "hide"} ${which === "list" ? "flows" : "details"}`;
  const now = new Set(JSON.parse(localStorage.getItem(FOLDED) ?? "[]") as string[]);
  if (folded) now.add(which); else now.delete(which);
  localStorage.setItem(FOLDED, JSON.stringify([...now]));
}
for (const which of ["list", "side"] as const) {
  $(`fold-${which}`).addEventListener("click", () => fold(which, !$("main").classList.contains(`hide-${which}`)));
  fold(which, (JSON.parse(localStorage.getItem(FOLDED) ?? "[]") as string[]).includes(which));
}

$("reset").addEventListener("click", () => {
  localStorage.removeItem(STORE);
  positions = initialPositions();
  render();
  canvas.refit(layout(positions));
});
window.addEventListener("hashchange", () => { selection = fromHash(); render(); });
selection = fromHash();
render();
