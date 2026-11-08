// The data as the view uses it: data/ and generated/code.json, loaded and indexed.
// Nothing here knows the robot: every module, flow and origin comes from data/. The
// rules (what a step carries, where its code is) are tools/resolve.ts's: read, not redone.
import { ORIGINS, refKey, type Carried, type Flow, type Member, type Module, type Place, type Resolved, type Step } from "../../data/schema.ts";
import flowsJson from "../../data/flows.json" with { type: "json" };
import resolvedJson from "../../generated/code.json" with { type: "json" };

const moduleFiles = import.meta.glob<Module[]>("../../data/modules/*.json", { eager: true, import: "default" });

export const modules: Module[] = Object.values(moduleFiles).flat();
export const flows = flowsJson as Flow[];
export const resolved = resolvedJson as Resolved;
export const origins = ORIGINS;
export type { Place };

export const moduleById = new Map(modules.map(m => [m.id, m]));
export const memberById = new Map<string, { module: Module; member: Member }>(
  modules.flatMap(m => m.members.map(mb => [`${m.id}.${mb.name}`, { module: m, member: mb }] as const)));
export const flowById = new Map(flows.map(f => [f.id, f]));

export const isChange = (s: Step): s is Extract<Step, { change: string }> => "change" in s;

// Places in the code, as resolve found them (null when it didn't).
export const place = (path: string | undefined, code: string | undefined): Place | null =>
  (path && code ? resolved.refs[refKey(path, code)] ?? null : null);
export const memberPlace = (id: string): Place | null => resolved.refs[resolved.members[id] ?? ""] ?? null;
export const stepPlace = (f: Flow, i: number): Place | null => resolved.refs[resolved.steps[f.id]?.[i]?.place ?? ""] ?? null;
export const vscodeLink = (p: Place) => `vscode://file/${resolved.root}/${p.path}:${p.line}`;

// What each step of a flow carries after it (type, bytes, rate).
export const carriedAlong = (f: Flow): Carried[] => (resolved.steps[f.id] ?? []).map(s => s.carried);

// The top-level flow a flow belongs to (itself, or its parent's, up).
export function rootFlow(f: Flow): Flow {
  let r = f;
  for (let i = 0; i < 10 && r.parent && flowById.has(r.parent); i++) r = flowById.get(r.parent)!;
  return r;
}

// A flow's first and last member: "from" and "to".
export const ends = (f: Flow): [string, string] => [f.steps[0]?.at ?? "?", f.steps[f.steps.length - 1]?.at ?? "?"];
