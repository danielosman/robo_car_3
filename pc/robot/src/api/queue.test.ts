// Tests for queue.ts and runner.ts: commands one at a time in the order they came,
// whoever sent them; anything not ok empties the queue ("dropped"); stop skips it;
// every answer to its sender and to everyone. The runner: numbered lines to the
// robot, done records matched by number, not connected, no answer, a snapshot
// answered once it is stored. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { EventEmitter } from "node:events";
import { parseSteer, withSnapshots, type Command } from "./commands.ts";
import { answerLine, type Answer, type Outcome } from "./answers.ts";
import { CommandQueue } from "./queue.ts";
import { CommandRunner, StoredSnapshots, type Runner } from "./runner.ts";
import type { DoneRecord } from "../robot.ts";

const steer = (text: string) => withSnapshots((parseSteer(text) as { commands: Command[] }).commands);

// A runner the test ends by hand: each run waits until finish() is called.
class FakeRunner implements Runner {
  started: string[] = [];
  private pending: ((o: Outcome) => void)[] = [];
  run(c: Command): Promise<Outcome> {
    this.started.push(c.name);
    return new Promise(resolve => this.pending.push(resolve));
  }
  finish(o: Outcome = { kind: "done", ok: true, text: "ok" }): void { this.pending.shift()!(o); }
  get waiting() { return this.pending.length; }
}
const tick = () => new Promise(r => setImmediate(r));

test("one at a time, in order, whoever sent them; anything not ok drops the rest, after the move's snapshot", async () => {
  const runner = new FakeRunner(), everyone: string[] = [], cli: string[] = [], page: string[] = [];
  const q = new CommandQueue(runner, a => everyone.push(`${a.sender} ${answerLine(a)}`));
  const first = q.submit(steer("r30"), "cli", a => cli.push(answerLine(a)));
  const second = q.submit(steer("d50 d50"), "page", a => page.push(answerLine(a)));
  await tick();
  assert.deepEqual(runner.started, ["rotate"]); // the page's wait their turn
  assert.deepEqual(q.state(), { running: { command: "r30", sender: "cli" },
    waiting: ["s", "d50", "s", "d50", "s"].map((command, i) => ({ command, sender: i ? "page" : "cli" })) });
  runner.finish(); await tick();
  runner.finish({ kind: "done", ok: true, text: "snapshot 4" }); await first;
  assert.deepEqual(cli, ["queued: r30 s", "done: r30 ok", "done: s snapshot 4"]);
  await tick();
  runner.finish({ kind: "done", ok: false, text: "obstacle after 31 cm" });
  await tick();
  assert.deepEqual(runner.started.at(-1), "snapshot"); // where it ended up
  runner.finish({ kind: "done", ok: true, text: "snapshot 5" });
  await second;
  assert.deepEqual(page, ["queued: d50 s d50 s", "done: d50 obstacle after 31 cm", "done: s snapshot 5", "dropped: d50", "dropped: s"]);
  assert.equal(everyone.length, cli.length + page.length);
  assert.ok(everyone.includes("page dropped: d50"));
  assert.deepEqual(q.state(), { running: null, waiting: [] });
});

test("an error drops everything still waiting", async () => {
  const runner = new FakeRunner(), answers: string[] = [];
  const q = new CommandQueue(runner, () => {});
  const asked = q.submit((parseSteer("c s") as { commands: Command[] }).commands, "cli", a => answers.push(answerLine(a)));
  await tick();
  runner.finish({ kind: "error", ok: false, text: "robot not connected" });
  await asked;
  assert.deepEqual(answers, ["queued: c s", "error: c robot not connected", "dropped: s"]);
});

test("stop skips the queue and empties it", async () => {
  const runner = new FakeRunner(), answers: string[] = [];
  const q = new CommandQueue(runner, () => {});
  const moves = q.submit(steer("d100 r90"), "page", a => answers.push(answerLine(a)));
  await tick();
  const stopped = q.stop("cli", a => answers.push(answerLine(a)));
  assert.deepEqual(runner.started, ["drive", "stop"]); // at once, past the running drive
  assert.deepEqual(answers.slice(1), ["dropped: s", "dropped: r90", "dropped: s"]);
  runner.finish({ kind: "done", ok: false, text: "stopped" }); // the drive
  runner.finish(); // the stop
  await Promise.all([moves, stopped]);
  assert.deepEqual(answers.slice(4).sort(), ["done: d100 stopped", "done: x ok"]);
});

// The robot as the runner sees it: the lines sent; done records sent back by hand.
class FakeRobot extends EventEmitter {
  connected = true;
  lines: string[] = [];
  sendLine(line: string): boolean { if (this.connected) this.lines.push(line); return this.connected; }
  done(d: Partial<DoneRecord> & { n: number; end: string }): void {
    this.emit("done", { pose: { x: 0.43, y: -0.04, yaw: Math.PI / 6 }, movedM: null, goalM: null, snapshot: null, ...d });
  }
}

test("the runner: numbered lines, done records by number, not connected, no answer", async () => {
  const robot = new FakeRobot(), stored = new StoredSnapshots();
  const runner = new CommandRunner(robot, stored, c => (c.name === "watch" ? 30 : 1000));
  const [drive, watch, snap] = steer("d50 w5").filter((_, i) => i !== 1);
  const driving = runner.run(drive);
  assert.deepEqual(robot.lines, ["drive 50 #1"]);
  robot.done({ n: 7, end: "ok" }); // nothing waits for 7: ignored
  robot.done({ n: 1, end: "obstacle", movedM: 0.31 });
  assert.deepEqual(await driving, { kind: "done", ok: false, text: "obstacle after 31 cm, at (43, -4) cm, yaw +30°" });
  assert.deepEqual(await runner.run(watch), { kind: "error", ok: false, text: "no answer" });
  const snapping = runner.run(snap);
  robot.done({ n: 3, end: "ok", snapshot: 41 });
  await tick();
  let settled = false;
  void snapping.then(() => { settled = true; });
  await tick();
  assert.ok(!settled); // not stored yet
  stored.add("1a2b3c4d", 41);
  assert.deepEqual(await snapping, { kind: "done", ok: true, text: "snapshot 41, at (43, -4) cm, yaw +30°" });
  robot.connected = false;
  assert.deepEqual(await runner.run(drive), { kind: "error", ok: false, text: "robot not connected" });
});

test("answers: goto beyond 3 m, safety stops, refused", async () => {
  const robot = new FakeRobot(), runner = new CommandRunner(robot, new StoredSnapshots());
  const run = async (text: string, d: Partial<DoneRecord> & { end: string }) => {
    const going = runner.run(steer(text)[0]);
    robot.done({ n: Number(/#(\d+)$/.exec(robot.lines.at(-1)!)![1]), ...d });
    return (await going).text;
  };
  assert.equal(await run("g300,300", { end: "ok", goalM: 4.24 }), "ok, 3 m towards it (goal 4.2 m away), at (43, -4) cm, yaw +30°");
  assert.equal(await run("r30", { end: "tilt" }), "safety stop: tilt, at (43, -4) cm, yaw +30°");
  assert.equal(await run("d30", { end: "wheels" }), "safety stop: wheels, at (43, -4) cm, yaw +30°");
  assert.equal(await run("c", { end: "ok" }), "ok");
  assert.equal(await run("d30", { end: "refused" }), "refused by the robot (robot log says why)");
});

test("answer lines", () => {
  const a: Answer = { kind: "dropped", command: "s", text: "", sender: "cli" };
  assert.equal(answerLine(a), "dropped: s");
  assert.equal(answerLine({ ...a, kind: "done", command: "r30", text: "ok" }), "done: r30 ok");
});
