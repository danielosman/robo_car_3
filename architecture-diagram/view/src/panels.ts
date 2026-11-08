// The flow list (left) and the side bar (right): plain HTML from the data.
import type { Flow, Module } from "../../data/schema.ts";
import { carriedAlong, ends, flows, isChange, memberById, memberPlace, moduleById, place, rootFlow, stepPlace, vscodeLink, type Place } from "./data.ts";

export type Selection = { kind: "module"; id: string } | { kind: "flow"; id: string } | { kind: "node"; flow: string; step: number } | null;

const esc = (s: string) => s.replace(/[&<>"]/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]!);
// Joins a template's parts (no escaping: values go through esc() where they need it).
const h = (strings: TemplateStringsArray, ...values: unknown[]) =>
  strings.reduce((out, s, i) => out + s + (i < values.length ? String(values[i] ?? "") : ""), "");

const link = (p: Place | null, label?: string) =>
  p ? h`<a class="code" href="${vscodeLink(p)}" title="open in VS Code">${esc(label ?? `${p.path}:${p.line}`)}</a>` : "";
const codeBlock = (p: Place | null) => (p ? h`<pre>${esc(p.text)}</pre>` : "");
const statusTag = (s?: string) => (s && s !== "built" ? h`<span class="tag ${s}">${s}</span>` : "");
const shortName = (id: string) => id.split(".").slice(1).join(".");
const bytesText = (b?: number) => (b === undefined ? "" : b >= 1024 ? `${(b / 1024).toFixed(1)} KB` : `${b} B`);

// --- the flow list ---
export function renderList(el: HTMLElement, colorOf: (f: Flow) => string, selected: Selection, select: (s: Selection) => void): void {
  const tops = flows.filter(f => !f.parent);
  const item = (f: Flow, sub: boolean) => {
    const [from, to] = ends(f);
    const on = selected?.kind === "flow" && selected.id === f.id;
    return h`<li class="${sub ? "sub" : ""} ${on ? "on" : ""}" data-flow="${f.id}">
      <div class="title"><span class="swatch" style="background:${colorOf(rootFlow(f))}"></span>${esc(f.name)} ${statusTag(f.status)}</div>
      <div class="id">${f.id}</div>
      <div class="desc">${esc(f.description)}</div>
      <div class="ends">${esc(shortName(from))} → ${esc(shortName(to))}</div></li>`;
  };
  const children = (f: Flow): string => flows.filter(c => c.parent === f.id).map(c => item(c, true) + children(c)).join("");
  el.innerHTML = h`<div class="list-head"><h2>Data flows</h2><button id="unselect" ${selected ? "" : "disabled"}>unselect</button></div>
    <ul>${tops.map(f => item(f, false) + children(f)).join("")}</ul>`;
  el.querySelector("#unselect")!.addEventListener("click", () => select(null));
  el.querySelectorAll<HTMLElement>("li[data-flow]").forEach(li => li.addEventListener("click", () => select({ kind: "flow", id: li.dataset.flow! })));
}

// --- the side bar ---
export function renderSide(el: HTMLElement, s: Selection, errors: string[]): void {
  const problems = errors.length ? h`<div class="errors"><b>npm run resolve: ${errors.length} errors</b><ul>${errors.map(e => h`<li>${esc(e)}</li>`).join("")}</ul></div>` : "";
  if (!s) { el.innerHTML = problems + `<p class="hint">Click a box, a ribbon, a circle or a flow; click it again to unselect. Drag boxes; scroll to zoom.</p>`; return; }
  if (s.kind === "module") el.innerHTML = problems + moduleHtml(moduleById.get(s.id) ?? null);
  else if (s.kind === "flow") el.innerHTML = problems + flowHtml(s.id);
  else el.innerHTML = problems + nodeHtml(s.flow, s.step);
}

function moduleHtml(m: Module | null): string {
  if (!m) return "";
  const head: Place | null = m.path ? { path: m.path, line: 1, text: "" } : null;
  const planLink = m.plan ? link(place(m.plan.path, m.plan.code), `plan: ${m.plan.path}`) : "";
  const members = m.members.map(mb => {
    const id = `${m.id}.${mb.name}`, p = memberPlace(id);
    const plan = mb.plan ? link(place(mb.plan.path, mb.plan.code), "plan") : "";
    return h`<div class="member"><div><b>${esc(mb.name)}</b> <span class="kind">${mb.kind}</span>
      ${mb.bytes !== undefined ? h`<span class="kind">${bytesText(mb.bytes)}</span>` : ""} ${statusTag(mb.status)} ${link(p)} ${plan}</div>${codeBlock(p)}</div>`;
  }).join("");
  return h`<h2>${esc(m.name)} ${statusTag(m.status)}</h2>
    <div class="meta">${m.id} · ${head ? link(head, m.path) : "no code yet"} ${planLink}</div>
    ${m.summary ? h`<p>${esc(m.summary)}</p>` : ""}
    <h3>Members</h3>${members}`;
}

function stepHtml(f: Flow, i: number, carried: ReturnType<typeof carriedAlong>): string {
  const s = f.steps[i], c = carried[i] ?? {}, p = stepPlace(f, i);
  const at = s.at ? h`<span class="at">${esc(s.at)}</span>` : "";
  const dto = s.dto ? memberById.get(s.dto) : undefined;
  const type = s.dto ? h`<details><summary>type <b>${esc(shortName(s.dto))}</b> ${bytesText(dto?.member.bytes)}</summary>${codeBlock(memberPlace(s.dto))}</details>`
    : s.bytes !== undefined ? h`<div class="kind">${bytesText(s.bytes)}, no type</div>` : "";
  const rate = s.rate ? h`<div class="rate">${esc(s.rate.text)} ${link(place(s.rate.path, s.rate.code), "set here")}</div>` : "";
  const label = isChange(s) ? h`<span class="node ${s.change}">${s.change}</span> <b>${esc(s.name)}</b> ${at}` : at;
  return h`<li><div>${label}</div>${isChange(s) && s.what ? h`<div>${esc(s.what)}</div>` : ""}
    ${isChange(s) && s.keeps ? h`<div class="kind">keeps ${s.keeps.join(", ")}</div>` : ""}
    ${type}${rate}<div class="carried">carries ${c.dto ? esc(shortName(c.dto)) : "bytes"} · ${bytesText(c.bytes)}${c.rate ? ` · ${esc(c.rate)}` : ""}</div>
    <details><summary>code ${link(p)}</summary>${codeBlock(p)}</details></li>`;
}

function flowHtml(id: string): string {
  const f = flows.find(x => x.id === id);
  if (!f) return "";
  const carried = carriedAlong(f), [from, to] = ends(f);
  const subs = flows.filter(c => c.parent === f.id).map(c => c.id).join(", ");
  return h`<h2>${esc(f.name)} ${statusTag(f.status)}</h2>
    <div class="meta">flow <b>${f.id}</b>${f.parent ? ` · branches off ${f.parent}` : ""}${subs ? ` · subflows ${subs}` : ""}</div>
    <p>${esc(f.description)}</p>
    <div class="meta">${esc(from)} → ${esc(to)}</div>
    ${f.plan ? link(place(f.plan.path, f.plan.code), "plan") : ""}
    <h3>Steps</h3><ol class="steps">${f.steps.map((_, i) => stepHtml(f, i, carried)).join("")}</ol>`;
}

function nodeHtml(flowId: string, i: number): string {
  const f = flows.find(x => x.id === flowId);
  const s = f?.steps[i];
  if (!f || !s) return "";
  if (!isChange(s)) return flowHtml(flowId); // a member step: its flow
  const carried = carriedAlong(f), before = carried[i - 1], after = carried[i], p = stepPlace(f, i);
  const typeOf = (c?: { dto?: string; bytes?: number }) =>
    c ? h`${c.dto ? esc(shortName(c.dto)) : "bytes"} (${bytesText(c.bytes)})` : "?";
  return h`<h2><span class="node ${s.change}">${s.change}</span> ${esc(s.name)}</h2>
    <div class="meta">node <b>${esc(s.name)}</b> in flow <b>${f.id}</b>${s.at ? ` · at ${esc(s.at)}` : ""}</div>
    ${s.what ? h`<p>${esc(s.what)}</p>` : ""}
    ${s.keeps ? h`<p>keeps ${s.keeps.join(", ")}</p>` : ""}
    <p>${typeOf(before)} → ${typeOf(after)}${after?.rate && after.rate !== before?.rate ? ` · now ${esc(after.rate)}` : ""}</p>
    <h3>Code ${link(p)}</h3>${codeBlock(p)}
    ${after?.dto && after.dto !== before?.dto ? h`<h3>New type ${link(memberPlace(after.dto))}</h3>${codeBlock(memberPlace(after.dto))}` : ""}`;
}
