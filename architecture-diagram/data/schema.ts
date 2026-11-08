// The shape of the architecture data (data/README.md has the rules and examples).
// Plain types and two lists, nothing else: the view and tools/resolve.ts import it,
// Node runs it as is (type stripping), so no enums or namespaces.

// Where a module runs, in the order the groups are drawn, left to right.
export const ORIGINS = [
  { id: "picoB", label: "PicoB" },
  { id: "common", label: "common (both Picos)" },
  { id: "picoA", label: "PicoA" },
  { id: "pc", label: "PC server" },
  { id: "api", label: "PC command API" }, // pc/robot/src/api: runs in the server, its own interface (HTTP)
  { id: "browser", label: "Browser" },
] as const;
export type Origin = (typeof ORIGINS)[number]["id"];

// built (default), planned (a plan names it, no code yet), leaving (a plan removes it).
export const STATUSES = ["built", "planned", "leaving"] as const;
export type Status = (typeof STATUSES)[number];
export const KINDS = ["function", "type", "constant", "route"] as const;
// pick (keeps some fields), turn (new values or a new type), add (joins other data).
export const CHANGES = ["pick", "turn", "add"] as const;

// A place in a file: `code` is a string found exactly once in `path` (from the repo root).
export interface CodeRef { path: string; code: string }

export interface Member {
  name: string;               // as in the code; a class's method as Class.method; a route as "POST /api/robot/:name"
  kind: (typeof KINDS)[number];
  code?: string;              // the declaration, found in `path` (the module's unless given); none when planned
  path?: string;
  bytes?: number;             // types a flow carries: one message's size (data/README.md, "Bytes")
  status?: Status;
  plan?: CodeRef;             // planned or leaving: where the plan says so
}

export interface Module {
  id: string;                 // <origin>.<name>
  name: string;
  origin: Origin;
  path?: string;              // the header or the file; none when planned
  summary?: string;           // one sentence, only where the file's top comment doesn't say it
  status?: Status;
  plan?: CodeRef;
  members: Member[];
}

// A fixed rate: shown on the ribbon from this step on; path + code: what sets it.
export interface Rate { text: string; path: string; code: string }

interface StepBase {
  dto?: string;               // the type leaving this step, a member id (<module id>.<member>)
  bytes?: number;             // only where no type gives it: after a pick, or bytes with no type
  rate?: Rate;
  path?: string;              // with code: the call site, or the line of the change
  code?: string;
}
// The data passes a module's member.
export interface MemberStep extends StepBase { at: string }
// The data changes (CHANGES above).
export interface ChangeStep extends StepBase {
  change: (typeof CHANGES)[number];
  name: string;               // unique within its flow
  at: string;                 // the member whose code makes the change; the node sits on its row
  what?: string;              // what changes, in a few words
  keeps?: string[];           // pick: the fields kept
}
export type Step = MemberStep | ChangeStep;

export interface Flow {
  id: string;                 // short, lower case, dashes: the name used in comments
  name: string;
  description: string;        // 1-2 sentences
  parent?: string;            // the flow this one branches off
  status?: Status;
  plan?: CodeRef;
  steps: Step[];
}

// The keys each object may have: resolve refuses any other (a typo like "dtos").
// keys<T>() makes the compiler hold each list and its type to the same keys.
const keys = <T>() => <K extends readonly (keyof T)[]>(k: K & ([keyof T] extends [K[number]] ? unknown : never)) => k;
export const KEYS = {
  module: keys<Module>()(["id", "name", "origin", "path", "summary", "status", "plan", "members"] as const),
  member: keys<Member>()(["name", "kind", "code", "path", "bytes", "status", "plan"] as const),
  flow: keys<Flow>()(["id", "name", "description", "parent", "status", "plan", "steps"] as const),
  step: keys<MemberStep & ChangeStep>()(["at", "change", "name", "what", "keeps", "dto", "bytes", "rate", "path", "code"] as const),
  rate: keys<Rate>()(["text", "path", "code"] as const),
  ref: keys<CodeRef>()(["path", "code"] as const),
};

// What tools/resolve.ts writes to generated/code.json for the view: the code found,
// and the rules applied (what each step carries, where it is), so the view repeats none.
export interface Place { path: string; line: number; text: string }
export interface Carried { dto?: string; bytes?: number; rate?: string }
export interface Resolved {
  root: string;               // the repo's full path, for vscode:// links
  refs: Record<string, Place>; // by refKey()
  members: Record<string, string>; // member id → its declaration's refKey
  steps: Record<string, { place?: string; carried: Carried }[]>; // flow id → per step: refKey of its code, what it carries after it
  errors: string[];
}
export const refKey = (path: string, code: string) => `${path}|${code}`;
