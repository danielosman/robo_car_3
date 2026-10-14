// Tests for commands.ts: the steering language (short and long names, spaces,
// limits, errors), how commands are written to the robot and in answers, the
// snapshots after moves, the time limits. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { parseSteer, robotLine, shortText, timeLimitMs, withSnapshots, type Command } from "./commands.ts";
import { LIMITS } from "./firmware.ts";

const short = (text: string) => {
  const p = parseSteer(text);
  if ("error" in p) return `error: ${p.error}`;
  return p.commands.map(shortText).join(" ");
};

test("short and long names, with or without spaces", () => {
  assert.equal(short("r30d50"), "r30 d50");
  assert.equal(short("r-90 d100 w30"), "r-90 d100 w30");
  assert.equal(short("rotate 30 drive 50"), "r30 d50");
  assert.equal(short("g120,40"), "g120,40");
  assert.equal(short("goto 120 40 s"), "g120,40 s");
  assert.equal(short("g-120, -40c"), "g-120,-40 c");
  assert.equal(short("scR1m0"), "s c R1 m0");
  assert.equal(short("recording on mode manual"), "R1 m1");
  assert.equal(short("R0 mode idle snapshot clear_map"), "R0 m0 s c");
  assert.equal(short("  x  "), "x");
  assert.equal(short("stop"), "x");
});

test("errors: no number, out of range, not a command, stop with others", () => {
  assert.equal(short("d"), "error: d needs a number (cm)");
  assert.equal(short("d r30"), "error: d needs a number (cm)");
  assert.equal(short("g120"), "error: g needs 2 numbers (cm)");
  assert.equal(short("d400"), "error: d400 out of range: drive takes -300 to 300 cm");
  assert.equal(short("g120,400"), "error: g120,400 out of range: goto takes -300 to 300 cm");
  assert.equal(short("w0"), "error: w0 out of range: watch takes 1 to 600 s");
  assert.equal(short("R2"), "error: R needs off or on (or 0/1)");
  assert.equal(short("mode fast"), "error: mode needs idle or manual (or 0/1)");
  assert.equal(short("d50 q"), "error: q: not a command (robot steer --help lists them)");
  assert.equal(short("d50 x"), "error: x stands alone: it stops everything at once");
  assert.equal(short(""), "error: no commands (robot steer --help lists them)");
  assert.equal(short(`r${LIMITS.rotateDeg}`), `r${LIMITS.rotateDeg}`);
});

test("to the robot, numbered; a snapshot after each move; time limits", () => {
  const p = parseSteer("d-40 g120,40 R1 m0 w60 s") as { commands: Command[] };
  assert.deepEqual(p.commands.map((c, i) => robotLine(c, i + 1)),
                   ["drive -40 #1", "goto 120 40 #2", "recording on #3", "mode idle #4", "watch 60 #5", "snapshot #6"]);
  assert.equal(withSnapshots(p.commands).map(shortText).join(" "), "d-40 s g120,40 s R1 m0 w60 s s");
  const [d] = (parseSteer("d100") as { commands: Command[] }).commands;
  assert.equal(timeLimitMs(d), (2 * 10 + 10) * 1000); // 1 m at 10 cm/s, twice, plus 10 s
  assert.equal(timeLimitMs({ name: "watch", values: [60] }), 130000);
  assert.equal(timeLimitMs({ name: "snapshot", values: [] }), 10000);
});
