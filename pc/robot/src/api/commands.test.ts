// Tests for commands.ts: the moves of `robot steer` (letters and long names, spaces,
// limits, errors), the other actions and their words, how commands are written in
// answers and to the robot, the snaps after moves, the time limits. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { ACTIONS, MOVES, commandText, parseAction, parseSteer, robotText, timeLimitMs, withSnaps, type Command, type CommandName } from "./commands.ts";
import { LIMITS } from "./firmware.ts";
import { MOVE_KINDS } from "../wire.ts";

const steer = (text: string) => {
  const p = parseSteer(text);
  return "error" in p ? `error: ${p.error}` : p.commands.map(commandText).join(" | ");
};
const commands = (text: string) => (parseSteer(text) as { commands: Command[] }).commands;

test("moves: letters and long names, with or without spaces", () => {
  assert.equal(steer("r30d50"), "steer r30 | steer d50");
  assert.equal(steer("r-90 d100 w30"), "steer r-90 | steer d100 | steer w30");
  assert.equal(steer("rotate 30 drive 50"), "steer r30 | steer d50");
  assert.equal(steer("g120,40"), "steer g120,40");
  assert.equal(steer("goto 120 40 w5"), "steer g120,40 | steer w5");
  assert.equal(steer(`r${LIMITS.rotateDeg}`), `steer r${LIMITS.rotateDeg}`);
});

test("moves: no number, out of range, not a move", () => {
  assert.equal(steer("d"), "error: d needs a number");
  assert.equal(steer("g120"), "error: g needs 2 numbers");
  assert.equal(steer("d400"), "error: d400: d takes -300 to 300");
  assert.equal(steer("g120,400"), "error: g120,400: g takes -300 to 300");
  assert.equal(steer("w0"), "error: w0: w takes 1 to 600");
  assert.equal(steer("d50 s"), "error: s is not a move (robot steer --help)");
  assert.equal(steer(""), "error: no moves (robot steer --help)");
});

test("the other actions and their words", () => {
  const one = (name: string, arg: string) => {
    const p = parseAction(name, arg);
    return p === null ? null : "error" in p ? `error: ${p.error}` : commandText(p.commands[0]);
  };
  assert.equal(one("snap", ""), "snap");
  assert.equal(one("snap", "2"), "error: snap takes nothing");
  assert.equal(one("recording", "on"), null); // gone with the recorder
  assert.equal(one("config", "pre_ms 300"), "config pre_ms 300");
  assert.equal(one("config", "post_ms 0"), "config post_ms 0");
  assert.equal(one("config", "post_ms 2001"), "error: config takes pre_ms or post_ms and 0 to 2000 (ms)");
  assert.equal(one("config", "wait_ms 5"), "error: config takes pre_ms or post_ms and 0 to 2000 (ms)");
  assert.equal(one("mode", "manual"), "mode manual");
  assert.equal(one("mode", "fast"), "error: mode takes idle or manual");
  assert.equal(one("clear_map", ""), "clear_map");
  assert.equal(one("steer", ""), null);
  assert.equal(one("toString", ""), null);
});

test("to the robot; a snap after each move; time limits", () => {
  const moves = commands("d-40 g120,40 w60");
  const actions = [parseAction("config", "post_ms 500"), parseAction("mode", "idle"), parseAction("snap", "")]
    .map(p => (p as { commands: Command[] }).commands[0]);
  assert.deepEqual([...moves, ...actions].map(robotText),
                   ["drive -40", "goto 120 40", "watch 60", "config post_ms 500", "mode idle", "snap"]);
  assert.equal(withSnaps(moves, true).map(commandText).join(" | "), "steer d-40 | snap | steer g120,40 | snap | steer w60 | snap");
  assert.ok(withSnaps(moves, true)[1].auto);
  assert.equal(withSnaps(moves, false).length, 3);
  assert.equal(timeLimitMs(commands("d100")[0]), (2 * 10 + 10) * 1000); // 1 m at 10 cm/s, twice, plus 10 s
  assert.equal(timeLimitMs(commands("w60")[0]), 130000);
  assert.equal(timeLimitMs({ name: "snap", values: [] }), 10000);
});

// The robot's C names the same things the PC sends (the PC sends its command names as
// the robot's words, robotText): read from picoA/app, as firmware.ts reads the limits.
const picoA = (file: string) => readFileSync(new URL(`../../../../picoA/app/${file}`, import.meta.url), "utf8");

test("the robot's words: commands.c takes every command the PC sends, and nothing besides", () => {
  const c = picoA("commands.c");
  const robot = new Set([...c.matchAll(/IS\("(\w+)/g), ...c.matchAll(/IS_WORD\("(\w+)"\)/g)].map(m => m[1]));
  const pc = [...MOVES, ...Object.keys(ACTIONS).map(name => ({ name: name as CommandName, values: [] }))]
    .map(c => robotText({ name: c.name, values: c.name === "mode" || c.name === "config" ? [0] : [] }).split(" ")[0]);
  assert.deepEqual([...robot].sort(), [...new Set(pc)].sort()); // report is the robot's status
  // the words some commands take: mode idle|manual, config pre_ms|post_ms
  const words = new Set([...c.matchAll(/strcmp\(word, "(\w+)"\)/g)].map(m => m[1]));
  assert.deepEqual([...words].sort(), Object.values(ACTIONS).flatMap(w => w ?? []).sort());
});

test("the move kinds: move.c's kind_name is wire.ts's MOVE_KINDS, in order (the wire sends the index)", () => {
  const names = /kind_name\[\] = \{([^}]*)\}/.exec(picoA("move.c"))![1].match(/"(\w+)"/g)!.map(s => s.slice(1, -1));
  assert.deepEqual(names, [...MOVE_KINDS]);
});
