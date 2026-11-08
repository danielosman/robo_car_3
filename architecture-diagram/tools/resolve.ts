// npm run resolve: checks data/ against itself and the robot's code (data/README.md,
// "The check"), and writes generated/code.json: each reference's line and the text the
// side bar shows, and what each flow's steps carry. The only code that reads both the
// data and the robot's code, and the one owner of the rules the view draws by.
// --no-fail: write and report, but exit 0 (npm run dev shows the errors on the page).
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import {
  CHANGES, KINDS, KEYS, ORIGINS, STATUSES, refKey,
  type Carried, type CodeRef, type Flow, type Member, type Module, type Resolved, type Step,
} from "../data/schema.ts";

const here = path.dirname(fileURLToPath(import.meta.url));
const diagram = path.resolve(here, "..");
const root = path.resolve(diagram, "..");
const errors: string[] = [];
const refs: Resolved["refs"] = {};

const readJson = (file: string): unknown => JSON.parse(readFileSync(file, "utf8"));
const modulesDir = path.join(diagram, "data", "modules");
const modules: Module[] = readdirSync(modulesDir).filter(f => f.endsWith(".json")).sort()
  .flatMap(f => readJson(path.join(modulesDir, f)) as Module[]);
const flows = readJson(path.join(diagram, "data", "flows.json")) as Flow[];

// --- keys and values: refuse what schema.ts doesn't know ---
function keysOf(where: string, o: object, allowed: readonly string[]): void {
  for (const k of Object.keys(o)) if (!allowed.includes(k)) errors.push(`${where}: unknown key "${k}"`);
}
function oneOf(where: string, value: string | undefined, allowed: readonly string[]): void {
  if (value !== undefined && !allowed.includes(value)) errors.push(`${where}: "${value}" is not one of ${allowed.join(", ")}`);
}
const originIds = ORIGINS.map(o => o.id);
for (const m of modules) {
  keysOf(`module ${m.id}`, m, KEYS.module);
  oneOf(`module ${m.id} origin`, m.origin, originIds);
  oneOf(`module ${m.id} status`, m.status, STATUSES);
  if (m.plan) keysOf(`module ${m.id} plan`, m.plan, KEYS.ref);
  for (const mb of m.members ?? []) {
    keysOf(`member ${m.id}.${mb.name}`, mb, KEYS.member);
    oneOf(`member ${m.id}.${mb.name} kind`, mb.kind, KINDS);
    oneOf(`member ${m.id}.${mb.name} status`, mb.status, STATUSES);
    if (mb.plan) keysOf(`member ${m.id}.${mb.name} plan`, mb.plan, KEYS.ref);
  }
}
for (const f of flows) {
  keysOf(`flow ${f.id}`, f, KEYS.flow);
  oneOf(`flow ${f.id} status`, f.status, STATUSES);
  if (f.plan) keysOf(`flow ${f.id} plan`, f.plan, KEYS.ref);
  f.steps.forEach((s, i) => {
    keysOf(`flow ${f.id} step ${i + 1}`, s, KEYS.step);
    if ("change" in s) oneOf(`flow ${f.id} step ${i + 1} change`, s.change, CHANGES);
    if (s.rate) keysOf(`flow ${f.id} step ${i + 1} rate`, s.rate, KEYS.rate);
  });
}

// --- finding code ---
const files = new Map<string, string[] | null>();
function lines(file: string): string[] | null {
  if (!files.has(file)) {
    const full = path.join(root, file);
    files.set(file, existsSync(full) ? readFileSync(full, "utf8").split("\n") : null);
  }
  return files.get(file)!;
}

const isComment = (l: string) => /^\s*(\/\/|\/\*|\*)/.test(l);
const code = (l: string) => l.replace(/\/\/.*$/, "");
const opens = (l: string) => (code(l).match(/[{(]/g) ?? []).length - (code(l).match(/[})]/g) ?? []).length;

// The comment lines right above line i (stopping at a blank or code line).
function commentAbove(ls: string[], i: number): string[] {
  const out: string[] = [];
  for (let j = i - 1; j >= 0 && isComment(ls[j]) && out.length < 12; j--) out.unshift(ls[j]);
  return out;
}

// A method's line without its body: up to the "{" after its parameters.
function signature(l: string): string {
  let depth = 0;
  for (let k = 0; k < l.length; k++) {
    if (l[k] === "(") depth++;
    else if (l[k] === ")") depth--;
    else if (l[k] === "{" && depth === 0) return l.slice(0, k).trimEnd();
  }
  return l.trimEnd();
}

// A declaration: its comment, then from its first line to its end (`;`, or the brace
// that closes it; a TS function only up to its opening brace; a class only its public
// method lines). A struct's reference is its last line (`} name;`): it starts at the
// line whose brace that one closes.
function declaration(ls: string[], i: number): string {
  let start = i;
  if (opens(ls[i]) < 0) {
    let depth = 0;
    for (let j = i; j >= 0; j--) { depth += opens(ls[j]); if (depth >= 0) { start = j; break; } }
    return [...commentAbove(ls, start), ...ls.slice(start, i + 1)].join("\n");
  }
  const head = ls[i];
  if (/\bclass\s/.test(head)) {
    const body: string[] = [head];
    for (let j = i + 1; j < ls.length && !/^}/.test(ls[j]); j++)
      if (/^ {2}(async )?[a-zA-Z]\w*(<[^>]*>)?\(/.test(ls[j])) body.push(signature(ls[j]));
    return [...commentAbove(ls, i), ...body, "}"].join("\n");
  }
  const tsFunction = /\bfunction\b|=>/.test(head) || /^ {2}(async )?\w+\(/.test(head);
  const tsType = /\btype\s+\w+(<[^>]*>)?\s*=/.test(head); // a union's lines end with "}": it ends at its ";"
  let end = i, depth = 0;
  for (let j = i; j < ls.length && j < i + 60; j++) {
    depth += opens(ls[j]);
    end = j;
    const c = code(ls[j]).trimEnd();
    if (tsFunction && c.endsWith("{")) break;
    if (depth <= 0 && (c.endsWith(";") || (c.endsWith("}") && !tsType))) break;
  }
  return [...commentAbove(ls, i), ...ls.slice(i, end + 1)].join("\n");
}

// A call site or a change: the line with two around it.
const context = (ls: string[], i: number) => ls.slice(Math.max(0, i - 2), i + 3).join("\n");

// Finds ref in its file; records its line and text; an error if not exactly once.
function resolve(where: string, ref: CodeRef, kind: "declaration" | "context"): string | null {
  const ls = lines(ref.path);
  if (!ls) { errors.push(`${where}: no file ${ref.path}`); return null; }
  const at = ls.flatMap((l, i) => (l.includes(ref.code) ? [i] : []));
  const total = ls.join("\n").split(ref.code).length - 1;
  if (total !== 1 || at.length !== 1) {
    errors.push(`${where}: "${ref.code}" found ${total} times in ${ref.path} (must be once)`);
    return null;
  }
  const key = refKey(ref.path, ref.code);
  refs[key] ??= { path: ref.path, line: at[0] + 1, text: kind === "declaration" ? declaration(ls, at[0]) : context(ls, at[0]) };
  return key;
}

// --- modules and members ---
const memberById = new Map<string, { module: Module; member: Member }>();
const members: Resolved["members"] = {};
const moduleIds = new Set<string>();
for (const m of modules) {
  if (moduleIds.has(m.id)) errors.push(`module ${m.id}: twice`);
  moduleIds.add(m.id);
  if (m.id !== `${m.origin}.${m.name}`) errors.push(`module ${m.id}: id must be ${m.origin}.${m.name}`);
  const planned = m.status === "planned";
  if (m.plan) resolve(`module ${m.id} plan`, m.plan, "context");
  if ((planned || m.status === "leaving") && !m.plan) errors.push(`module ${m.id}: ${m.status}, but no plan`);
  if (!planned && !m.path) errors.push(`module ${m.id}: no path`);
  if (!planned && m.path && !lines(m.path)) errors.push(`module ${m.id}: no file ${m.path}`);
  for (const mb of m.members) {
    const id = `${m.id}.${mb.name}`;
    if (memberById.has(id)) errors.push(`member ${id}: twice`);
    memberById.set(id, { module: m, member: mb });
    if (mb.plan) resolve(`member ${id} plan`, mb.plan, "context");
    if (planned || mb.status === "planned") {
      if (!mb.plan && !m.plan) errors.push(`member ${id}: planned, but no plan`);
      continue;
    }
    const file = mb.path ?? m.path;
    if (!mb.code || !file) { errors.push(`member ${id}: no code or path`); continue; }
    const key = resolve(`member ${id}`, { path: file, code: mb.code }, "declaration");
    if (key) members[id] = key;
  }
}

// A type member's declaration without comments: what a pick's fields are checked against.
const typeCode = (id: string) => {
  const key = members[id];
  return key ? refs[key].text.split("\n").filter(l => !isComment(l)).map(code).join("\n") : null;
};

// --- flows ---
const flowById = new Map(flows.map(f => [f.id, f]));
if (flowById.size !== flows.length) errors.push("flows: an id is there twice");
const isChange = (s: Step): s is Extract<Step, { change: string }> => "change" in s;

// What each step carries after it: its type, bytes and rate. A subflow starts with
// what its parent carries after the parent's last step at the subflow's first member.
// A step with bytes and no type carries bytes alone (after a pick, or packed bytes).
const carriedMemo = new Map<string, Carried[]>();
function carriedAlong(f: Flow, depth = 0): Carried[] {
  if (carriedMemo.has(f.id)) return carriedMemo.get(f.id)!;
  let state: Carried = {};
  const first = f.steps[0];
  if (f.parent && depth < 10 && first?.at) {
    const p = flowById.get(f.parent);
    if (p) {
      const i = p.steps.findLastIndex(s => s.at === first.at);
      if (i >= 0) state = { ...carriedAlong(p, depth + 1)[i] };
    }
  }
  const out = f.steps.map(s => {
    if (s.dto) state = { ...state, dto: s.dto, bytes: memberById.get(s.dto)?.member.bytes };
    else if (s.bytes !== undefined) state = { ...state, dto: undefined, bytes: s.bytes };
    if (s.rate) state = { ...state, rate: s.rate.text };
    return { ...state };
  });
  carriedMemo.set(f.id, out);
  return out;
}

const steps: Resolved["steps"] = {};
for (const f of flows) {
  const where = `flow ${f.id}`;
  if (!/^[a-z0-9]+(-[a-z0-9]+)*$/.test(f.id)) errors.push(`${where}: id must be lower case with dashes`);
  if (f.plan) resolve(`${where} plan`, f.plan, "context");
  if ((f.status === "planned" || f.status === "leaving") && !f.plan) errors.push(`${where}: ${f.status}, but no plan`);
  if (f.steps.length < 2) errors.push(`${where}: fewer than two steps`);
  const first = f.steps[0];
  if (first && isChange(first)) errors.push(`${where}: the first step must be a member`);
  if (f.parent) {
    const p = flowById.get(f.parent);
    if (!p) errors.push(`${where}: no parent ${f.parent}`);
    else if (first && !p.steps.some(s => s.at === first.at)) errors.push(`${where}: its first step (${first.at}) isn't on ${f.parent}`);
  } else if (first && !first.dto && first.bytes === undefined) errors.push(`${where}: the first step carries no type or bytes`);
  const names = new Set<string>();
  const carried = carriedAlong(f);
  steps[f.id] = f.steps.map((s, i) => {
    const w = `${where} step ${i + 1}`;
    const target = s.at ? memberById.get(s.at) : undefined;
    if (!s.at) errors.push(`${w}: no at (every step names a member)`);
    else if (!target) errors.push(`${w}: no member ${s.at}`);
    if (s.dto) {
      const t = memberById.get(s.dto);
      if (!t) errors.push(`${w}: no type ${s.dto}`);
      else if (t.member.kind !== "type") errors.push(`${w}: ${s.dto} is not a type`);
      else if (t.member.bytes === undefined) errors.push(`${w}: ${s.dto} has no bytes`);
    }
    let place: string | null | undefined;
    if (s.code) {
      const file = s.path ?? target?.member.path ?? target?.module.path;
      if (!file) errors.push(`${w}: code without a path`);
      else place = resolve(w, { path: file, code: s.code }, "context");
    } else if (s.at) place = members[s.at];
    if (s.rate) resolve(`${w} rate`, s.rate, "context");
    if (isChange(s)) {
      if (names.has(s.name)) errors.push(`${w}: node name "${s.name}" twice`);
      names.add(s.name);
      if (!s.code) errors.push(`${w}: a node needs code (the line of the change)`);
      if (s.change === "pick") {
        if (!s.keeps?.length) errors.push(`${w}: a pick keeps no fields`);
        if (s.bytes === undefined) errors.push(`${w}: a pick needs bytes`);
        const before = i > 0 ? carried[i - 1].dto : undefined;
        const text = before ? typeCode(before) : null;
        if (!text) errors.push(`${w}: no type before the pick to check its fields against`);
        else for (const k of s.keeps ?? []) if (!new RegExp(`\\b${k}\\b`).test(text)) errors.push(`${w}: ${before} has no field ${k}`);
      }
    }
    return { ...(place ? { place } : {}), carried: carried[i] };
  });
}

// --- out ---
const out: Resolved = { root, refs, members, steps, errors };
mkdirSync(path.join(diagram, "generated"), { recursive: true });
writeFileSync(path.join(diagram, "generated", "code.json"), JSON.stringify(out, null, 1));
for (const e of errors) console.log(`error: ${e}`);
console.log(`${modules.length} modules, ${flows.length} flows, ${Object.keys(refs).length} code references: ${errors.length ? `${errors.length} errors` : "clean"}`);
if (errors.length && !process.argv.includes("--no-fail")) process.exit(1);
