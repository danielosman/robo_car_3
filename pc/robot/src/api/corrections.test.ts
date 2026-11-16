// Tests for the PC's correction sent to the robot (corrections.ts): sent before a move and
// after a snap when it differs, not again when the robot has it; a goto's point turned
// into the robot's frame (by the correction the robot has, also when it didn't answer);
// sent again after no answer; the log; c_robot from the store after a server restart;
// the live pose back to raw. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { Corrections } from "./corrections.ts";
import { corrected, uncorrected, type Planar } from "../correction.ts";
import type { DoneRecord } from "../link.ts";
import type { Command } from "./commands.ts";
import type { Outcome } from "./answers.ts";

const done = (end: string, snapshot: number | null = null): DoneRecord =>
  ({ n: 1, end, pose: null, movedM: null, rotatedDeg: null, goalM: null, snapshot });

// A robot that takes "correct" (it says "corrected", then answers), or doesn't answer.
function setup(o: { stored?: Planar[]; answers?: boolean } = {}) {
  const sent: string[] = [], ran: Command[] = [], notes: string[] = [];
  const pc: Planar = { x: 0.1, y: -0.02, yaw: 0.1 };
  const deps = {
    live: { boot: "b1" as string | null, pose: { x: 1, y: 0, yaw: 0 } as Planar | null },
    snapshots: { newestOf: (b: string) => (b === "b1" ? 3 : null) },
    outlines: { correction: async () => pc },
    store: { corrections: async () => (o.stored ?? []).map(c => ({ c })) },
    note: (t: string) => notes.push(t),
    config: { correct: true },
  };
  let answers: boolean | "refuse" = o.answers ?? true;
  const inner = {
    reply: null as Outcome | null,  // a move's answer, if not ok
    async run(c: Command): Promise<Outcome> { ran.push(c); return c.name === "snap" ? { result: "ok", snapshot: 3 } : inner.reply ?? { result: "ok" }; },
    async send(text: string): Promise<DoneRecord | string> {
      sent.push(text);
      if (!answers) return "no answer";
      if (answers === "refuse") return done("busy_bulk");
      const [x, y, cdeg] = text.split(" ").slice(1).map(Number);
      corrections.heard("b1", { x: x / 1000, y: y / 1000, yaw: cdeg / 100 * Math.PI / 180 });
      return done("ok");
    },
  };
  const corrections = new Corrections(inner, deps);
  return { corrections, sent, ran, notes, pc, deps, inner, answer: (on: boolean | "refuse") => { answers = on; } };
}

test("sent before a move when it differs, once; a goto's point as given once the robot has it; the log", async () => {
  const { corrections, sent, ran, notes } = setup();
  await corrections.run({ name: "goto", values: [100, 50] });
  assert.deepEqual(sent, ["correct 100 -20 573"]);
  assert.deepEqual(ran, [{ name: "goto", values: [100, 50] }]);
  assert.match(notes[0], /^correction sent: the robot's pose moved [\d.]+ cm, 5\.7°$/);
  await corrections.run({ name: "drive", values: [20] });
  assert.equal(sent.length, 1, "the robot has it: not again");
  await corrections.run({ name: "snap", values: [] });
  assert.equal(sent.length, 1);
});

test("after a snap; no answer: the goto turned by the correction the robot has, sent again before the next move", async () => {
  const { corrections, sent, ran, notes, pc, answer } = setup({ answers: false });
  await corrections.run({ name: "snap", values: [] });
  assert.deepEqual(sent, ["correct 100 -20 573"]);
  assert.match(notes[0], /not taken \(no answer\)/);
  await corrections.run({ name: "goto", values: [100, 50] });
  assert.equal(sent.length, 2, "sent again");
  // The robot still has 0: the point goes as raw odometry, so it drives to the place meant.
  const robotFrame = uncorrected(pc, { x: 1, y: 0.5, yaw: 0 });
  const back = corrected(pc, robotFrame);
  assert.ok(Math.hypot(back.x - 1, back.y - 0.5) < 1e-12 && Math.abs(robotFrame.x - 0.9 * Math.cos(0.1) - 0.52 * Math.sin(0.1)) < 1e-12);
  assert.deepEqual(ran.at(-1), { name: "goto", values: [Math.round(robotFrame.x * 100), Math.round(robotFrame.y * 100)] });
  answer(true);
  await corrections.run({ name: "rotate", values: [30] });
  assert.equal(sent.length, 3);
  await corrections.run({ name: "goto", values: [100, 50] });
  assert.equal(sent.length, 3);
  assert.deepEqual(ran.at(-1), { name: "goto", values: [100, 50] });
});

test("c_robot from the store (a server restarted mid-boot); the live pose back to raw; no boot or snapshot: nothing sent", async () => {
  const { corrections, sent, pc, deps } = setup({ stored: [{ x: 0, y: 0, yaw: 0.05 }, { x: 0.1, y: -0.02, yaw: 0.1 }] });
  assert.deepEqual(await corrections.robotHas("b1"), pc);
  await corrections.run({ name: "drive", values: [20] });
  assert.deepEqual(sent, [], "the robot has it already");
  const raw = corrections.raw("b1", { x: 1, y: 2, yaw: 0.5, mode: "manual" });
  const back = corrected(pc, raw);
  assert.ok(Math.hypot(back.x - 1, back.y - 2) < 1e-12 && Math.abs(back.yaw - 0.5) < 1e-12 && raw.mode === "manual");
  deps.live.boot = "b2";
  await corrections.run({ name: "drive", values: [20] });
  assert.deepEqual(sent, []);
});

test("refused by the robot (something still on its bulk lane): logged, c_robot kept, sent again", async () => {
  const { corrections, sent, notes, answer } = setup();
  answer("refuse");
  await corrections.run({ name: "drive", values: [20] });
  assert.equal(notes[0], 'correction not taken (refused by the robot: "correct 100 -20 573" waits until frames, a snapshot or the map ' +
                         "have gone out): sent again before the next move");
  assert.deepEqual(await corrections.robotHas("b1"), { x: 0, y: 0, yaw: 0 });
  answer(true);
  await corrections.run({ name: "drive", values: [20] });
  assert.equal(sent.length, 2);
});

test("a goto the robot refuses: the answer adds the map's point and both corrections when they turned it", async () => {
  const { corrections, deps, inner } = setup();
  deps.config.correct = false;            // the robot keeps 0, the PC has 10 cm, 5.7°
  inner.reply = { result: "failed", why: "refused by the robot: \"goto 400 -39\": the point is 300.4 cm in x ..." };
  const out = await corrections.run({ name: "goto", values: [420, 0] });
  assert.match(out.why ?? "", /^refused by the robot: .*\. The map's 420, 0 cm is 4\d\d, -\d+ cm in the robot's own frame, turned by the PC's correction \(10 cm, 5\.7°\) and the robot's \(0 cm, 0\.0°\)$/);
  deps.config.correct = true;             // the robot takes it: the point goes as given, the answer as the robot's
  assert.equal((await corrections.run({ name: "goto", values: [420, 0] })).why, inner.reply.why);
});

test("config correct off: nothing sent; a goto still turned by the correction the robot has", async () => {
  const { corrections, sent, ran, pc, deps } = setup();
  deps.config.correct = false;
  await corrections.run({ name: "goto", values: [100, 50] });
  assert.deepEqual(sent, []);
  const robotFrame = uncorrected(pc, { x: 1, y: 0.5, yaw: 0 });
  assert.deepEqual(ran.at(-1), { name: "goto", values: [Math.round(robotFrame.x * 100), Math.round(robotFrame.y * 100)] });
});
