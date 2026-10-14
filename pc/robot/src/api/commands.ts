// The commands that go into the queue (doc/CLI_API_PLAN.md): the moves, given to
// `robot steer` ("d50 r30 g120,40 w60"), and the other actions, each its own
// command (snap, stop, clear_map, recording on|off, mode idle|manual). A command is
// a name and its values: numbers, or a word's index (recording off/on = 0/1, mode
// idle/manual = 0/1). Written in answers as typed ("steer d50", "recording on"),
// and to the robot long and numbered ("drive 50 #17").
//
// steer's grammar: moves one after another, spaces optional; each a letter (d, r, g,
// w) or its long name, then whole numbers (a minus allowed), goto's two separated by
// a comma or spaces. "r30d50", "g120,40 w60", "rotate 30 drive 50".
import { LIMITS, SPEED } from "./firmware.ts";

export type MoveName = "drive" | "rotate" | "goto" | "watch";
export type CommandName = MoveName | "snap" | "stop" | "clear_map" | "recording" | "mode";
// auto: a snap the API added after a move (auto_snap).
export interface Command { name: CommandName; values: number[]; auto?: boolean }

interface Range { min: number; max: number; unit: string }
export interface MoveSpec { name: MoveName; letter: string; values: Range[]; does: string }

const range = (unit: string, max: number, min = -max): Range => ({ min, max, unit });

export const MOVES: readonly MoveSpec[] = [
  { name: "drive", letter: "d", values: [range("cm", LIMITS.driveCm)],
    does: "Drive straight. Negative drives backwards. Stops before an obstacle on the map." },
  { name: "rotate", letter: "r", values: [range("degrees", LIMITS.rotateDeg)],
    does: "Turn on the spot. Positive turns left, negative right." },
  { name: "goto", letter: "g", values: [range("cm", LIMITS.gotoCm), range("cm", LIMITS.gotoCm)],
    does: "Go to a point on the map: turn towards it, then drive there. At most 3 m; stops before obstacles." },
  { name: "watch", letter: "w", values: [range("s", LIMITS.watchMaxS, LIMITS.watchMinS)],
    does: "For that many seconds, turn towards anything that moves." },
];
// The other actions: their words, if they take one.
export const ACTIONS: Record<Exclude<CommandName, MoveName>, readonly string[] | null> = {
  snap: null, stop: null, clear_map: null, recording: ["off", "on"], mode: ["idle", "manual"],
};
// The robot's word for each (picoA/app/commands.h).
const ROBOT_NAME: Record<CommandName, string> = {
  drive: "drive", rotate: "rotate", goto: "goto", watch: "watch",
  snap: "snapshot", stop: "stop", clear_map: "clear_map", recording: "recording", mode: "mode",
};

const MOVE = new Map(MOVES.map(m => [m.name, m]));
const isMoveName = (n: CommandName): n is MoveName => MOVE.has(n as MoveName);
// Every way to name a move, longest first: the scanner takes the first that matches.
const NAMES = MOVES.flatMap(m => [[m.name, m] as const, [m.letter, m] as const]).sort((a, b) => b[0].length - a[0].length);

export type Parsed = { commands: Command[] } | { error: string };

// The moves of `robot steer`.
export function parseSteer(text: string): Parsed {
  const commands: Command[] = [];
  let at = 0;
  const space = () => { while (at < text.length && /\s/.test(text[at])) at++; };
  for (space(); at < text.length; space()) {
    const found = NAMES.find(([name]) => text.startsWith(name, at));
    if (!found) return { error: `${text.slice(at).split(/\s/)[0]} is not a move (robot steer --help)` };
    const [written, m] = found;
    const start = at;
    at += written.length;
    const values: number[] = [];
    for (const k of m.values.keys()) {
      const sep = (k > 0 ? /^\s*,\s*|^\s+/ : /^\s*/).exec(text.slice(at));
      if (sep) at += sep[0].length;
      const n = /^-?\d+/.exec(text.slice(at));
      if (!n) return { error: `${written} needs ${m.values.length > 1 ? `${m.values.length} numbers` : "a number"}` };
      at += n[0].length;
      values.push(Number(n[0]));
    }
    const out = m.values.findIndex((v, i) => values[i] < v.min || values[i] > v.max);
    if (out >= 0) return { error: `${text.slice(start, at)}: ${m.letter} takes ${m.values[out].min} to ${m.values[out].max}` };
    commands.push({ name: m.name, values });
  }
  if (!commands.length) return { error: "no moves (robot steer --help)" };
  return { commands };
}

// One of the other actions with its argument ("on", "idle", or 0/1); null if name
// isn't one.
export function parseAction(name: string, arg: string): Parsed | null {
  if (!Object.hasOwn(ACTIONS, name)) return null;
  const n = name as keyof typeof ACTIONS, words = ACTIONS[n], a = arg.trim();
  if (!words) return a ? { error: `${name} takes nothing` } : { commands: [{ name: n, values: [] }] };
  const index = /^[01]$/.test(a) ? Number(a) : words.indexOf(a);
  return index < 0 ? { error: `${name} takes ${words.join(" or ")}` } : { commands: [{ name: n, values: [index] }] };
}

// As typed: "steer d50", "steer g120,40", "snap", "recording on".
export function commandText(c: Command): string {
  if (isMoveName(c.name)) return `steer ${MOVE.get(c.name)!.letter}${c.values.join(",")}`;
  const words = ACTIONS[c.name];
  return words ? `${c.name} ${words[c.values[0]]}` : c.name;
}

// To the robot: "drive 50 #17", "goto 120 40 #18", "recording on #19", "snapshot #20".
export function robotLine(c: Command, n: number): string {
  const words = isMoveName(c.name) ? null : ACTIONS[c.name];
  const values = words ? [words[c.values[0]]] : c.values.map(String);
  return [ROBOT_NAME[c.name], ...values, `#${n}`].join(" ");
}

export const isMove = (c: Command) => isMoveName(c.name);

// The commands with a snap after each move, when autoSnap is on.
export function withSnaps(commands: Command[], autoSnap: boolean): Command[] {
  return autoSnap ? commands.flatMap(c => (isMove(c) ? [c, { name: "snap" as const, values: [], auto: true }] : [c])) : commands;
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
