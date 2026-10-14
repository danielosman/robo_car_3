// The steering language (doc/CLI_API_PLAN.md): what `robot steer` takes, and how a
// command is written in answers (short: "d50") and to the robot (long, numbered:
// "drive 50 #17"). A command is a name and its values: numbers, or a word's index
// (recording off/on = 0/1, mode idle/manual = 0/1).
//
// Grammar: commands one after another, spaces optional. Each is a long name or its
// letter (the longest that matches; letters are case-sensitive), then its values:
// whole numbers (a minus allowed), two of them separated by a comma or spaces;
// words, or 0/1 for them. "r30d50", "g120,40 s", "rotate 30 drive 50", "R1", "mode idle".
import { LIMITS, SPEED } from "./firmware.ts";

export type CommandName = "drive" | "rotate" | "goto" | "watch" | "snapshot" | "clear_map" | "recording" | "mode" | "stop";
// afterMove: a snapshot the API added after a move (withSnapshots).
export interface Command { name: CommandName; values: number[]; afterMove?: boolean }

type Value = { kind: "number"; unit: string; min: number; max: number } | { kind: "word"; words: readonly string[] };
export interface CommandSpec {
  name: CommandName; letter: string; values: Value[];
  moves: boolean;  // a snapshot follows it
  does: string;    // for the help
}

const number = (unit: string, max: number, min = -max): Value => ({ kind: "number", unit, min, max });
const word = (...words: string[]): Value => ({ kind: "word", words });

export const COMMANDS: readonly CommandSpec[] = [
  { name: "drive", letter: "d", values: [number("cm", LIMITS.driveCm)], moves: true,
    does: "drive straight; negative = backwards; stops before obstacles on the map" },
  { name: "rotate", letter: "r", values: [number("°", LIMITS.rotateDeg)], moves: true,
    does: "turn on the spot; + = left, - = right" },
  { name: "goto", letter: "g", values: [number("cm", LIMITS.gotoCm), number("cm", LIMITS.gotoCm)], moves: true,
    does: "go to a point on the map: turn towards it, drive there (at most 3 m towards it), stop before obstacles" },
  { name: "watch", letter: "w", values: [number("s", LIMITS.watchMaxS, LIMITS.watchMinS)], moves: true,
    does: "for that many seconds, turn towards anything that moves" },
  { name: "snapshot", letter: "s", values: [], moves: false, does: "a snapshot (camera, ToF, light, map), saved" },
  { name: "clear_map", letter: "c", values: [], moves: false, does: "clear the map" },
  { name: "recording", letter: "R", values: [word("off", "on")], moves: false, does: "recording off or on (R0, R1)" },
  { name: "mode", letter: "m", values: [word("idle", "manual")], moves: false, does: "mode idle or manual (m0, m1)" },
  { name: "stop", letter: "x", values: [], moves: false,
    does: "motors off, mode idle, the queue emptied; skips the queue, and stands alone" },
];

const SPEC = new Map(COMMANDS.map(s => [s.name, s]));
const spec = (name: CommandName) => SPEC.get(name)!;
// Every way to name a command, longest first: the scanner takes the first that matches.
const NAMES = COMMANDS.flatMap(s => [[s.name, s] as const, [s.letter, s] as const]).sort((a, b) => b[0].length - a[0].length);

export type Parsed = { commands: Command[] } | { error: string };

export function parseSteer(text: string): Parsed {
  const commands: Command[] = [];
  let at = 0;
  const space = () => { while (at < text.length && /\s/.test(text[at])) at++; };
  for (space(); at < text.length; space()) {
    const found = NAMES.find(([name]) => text.startsWith(name, at));
    if (!found) return { error: `${text.slice(at).split(/\s/)[0]}: not a command (robot steer --help lists them)` };
    const [written, s] = found;
    const start = at;
    at += written.length;
    const values: number[] = [];
    for (const [k, v] of s.values.entries()) {
      const sep = (k > 0 ? /^\s*,\s*|^\s+/ : /^\s*/).exec(text.slice(at));
      if (sep) at += sep[0].length;
      const rest = text.slice(at);
      if (v.kind === "number") {
        const m = /^-?\d+/.exec(rest);
        if (!m) return { error: `${written} needs ${s.values.length > 1 ? `${s.values.length} numbers` : "a number"} (${v.unit})` };
        at += m[0].length;
        values.push(Number(m[0]));
      } else {
        const m = /^[01](?!\d)|^[a-z]+/.exec(rest);
        const index = !m ? -1 : /^[01]$/.test(m[0]) ? Number(m[0]) : v.words.indexOf(m[0]);
        if (index < 0) return { error: `${written} needs ${v.words.join(" or ")} (or 0/1)` };
        at += m![0].length;
        values.push(index);
      }
    }
    const out = s.values.findIndex((v, i) => v.kind === "number" && (values[i] < v.min || values[i] > v.max));
    if (out >= 0) {
      const v = s.values[out] as Extract<Value, { kind: "number" }>;
      return { error: `${text.slice(start, at)} out of range: ${s.name} takes ${v.min} to ${v.max} ${v.unit}` };
    }
    commands.push({ name: s.name, values });
  }
  if (!commands.length) return { error: "no commands (robot steer --help lists them)" };
  if (commands.length > 1 && commands.some(c => c.name === "stop")) return { error: "x stands alone: it stops everything at once" };
  return { commands };
}

// "d50", "g120,40", "R1".
export function shortText(c: Command): string { return spec(c.name).letter + c.values.join(","); }

// "drive 50 #17", "goto 120 40 #18", "recording on #19".
export function robotLine(c: Command, n: number): string {
  const values = spec(c.name).values.map((v, i) => (v.kind === "word" ? v.words[c.values[i]] : String(c.values[i])));
  return [c.name, ...values, `#${n}`].join(" ");
}

export const isMove = (c: Command) => spec(c.name).moves;

// The commands with a snapshot after each move.
export function withSnapshots(commands: Command[]): Command[] {
  return commands.flatMap(c => (isMove(c) ? [c, { name: "snapshot" as const, values: [], afterMove: true }] : [c]));
}

// How long the robot may take to answer: what the command should take, twice, plus
// 10 s (starting, settling, the link).
export function timeLimitMs(c: Command): number {
  const turnS = (deg: number) => (Math.abs(deg) * Math.PI / 180) / SPEED.turnRadps;
  const driveS = (cm: number) => Math.abs(cm) / 100 / SPEED.driveMps;
  const expectedS = c.name === "drive" ? driveS(c.values[0])
    : c.name === "rotate" ? turnS(c.values[0])
    : c.name === "goto" ? turnS(180) + driveS(LIMITS.gotoCm * Math.SQRT2)
    : c.name === "watch" ? c.values[0]
    : 0;
  return Math.round((2 * expectedS + 10) * 1000);
}
