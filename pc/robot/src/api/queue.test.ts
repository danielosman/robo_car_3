// Tests for queue.ts, runner.ts and answers.ts: commands one at a time in the order
// they came, whoever sent them; anything not ok empties the queue ("dropped"), after a
// move's own snap; stop skips it; every answer to its sender and to everyone. The
// runner: numbered lines to the robot, done records matched by number, not connected,
// no answer, a snap answered once its snapshot is stored. The answers' fields.
import { test } from "node:test";
import assert from "node:assert/strict";
import { EventEmitter } from "node:events";
import { parseAction, parseSteer, withSnaps, type Command } from "./commands.ts";
import type { Answer, Outcome, Sender } from "./answers.ts";
import { CommandQueue } from "./queue.ts";
import { CommandRunner, StoredSnapshots, type Runner } from "./runner.ts";
import type { DoneRecord } from "../robot.ts";

const steer = (text: string) => withSnaps((parseSteer(text) as { commands: Command[] }).commands, true);
const action = (name: string, arg = "") => (parseAction(name, arg) as { commands: Command[] }).commands;
const short = (a: Answer) => `${a.command} ${a.result}${a.why ? ` (${a.why})` : ""}`;

// A runner the test ends by hand: each run waits until finish() is called.
class FakeRunner implements Runner {
  started: string[] = [];
  private pending: ((o: Outcome) => void)[] = [];
  run(c: Command): Promise<Outcome> {
    this.started.push(c.name);
    return new Promise(resolve => this.pending.push(resolve));
  }
  finish(o: Outcome = { result: "ok" }): void { this.pending.shift()!(o); }
}
const tick = () => new Promise(r => setImmediate(r));

test("one at a time, in order, whoever sent them; not ok drops the rest, after the move's snap", async () => {
  const runner = new FakeRunner(), everyone: string[] = [], cli: string[] = [], page: string[] = [];
  const q = new CommandQueue(runner, (a: Answer, s: Sender) => everyone.push(`${s} ${short(a)}`));
  const first = q.submit(steer("r30"), "cli", a => cli.push(short(a)));
  const second = q.submit(steer("d50 d50"), "page", a => page.push(short(a)));
  await tick();
  assert.deepEqual(runner.started, ["rotate"]); // the page's wait their turn
  assert.deepEqual(q.state(), { running: "steer r30", waiting: ["snap", "steer d50", "snap", "steer d50", "snap"] });
  runner.finish(); await tick();
  runner.finish({ result: "ok", snapshot: 4 }); await first;
  assert.deepEqual(cli, ["steer r30 ok", "snap ok"]);
  await tick();
  runner.finish({ result: "obstacle", moved: 31, rotated: 0 });
  await tick();
  assert.equal(runner.started.at(-1), "snap"); // where it ended up
  runner.finish({ result: "ok", snapshot: 5 });
  await second;
  assert.deepEqual(page, ["steer d50 obstacle", "snap ok", "steer d50 dropped (steer d50 ended: obstacle)", "snap dropped (steer d50 ended: obstacle)"]);
  assert.equal(everyone.length, cli.length + page.length);
  assert.ok(everyone.includes("page snap ok"));
  assert.deepEqual(q.state(), { running: null, waiting: [] });
});

test("a failure drops everything still waiting", async () => {
  const runner = new FakeRunner(), answers: string[] = [];
  const q = new CommandQueue(runner, () => {});
  const asked = q.submit([...action("clear_map"), ...action("snap")], "cli", a => answers.push(short(a)));
  await tick();
  runner.finish({ result: "failed", why: "robot not connected" });
  await asked;
  assert.deepEqual(answers, ["clear_map failed (robot not connected)", "snap dropped (clear_map ended: failed)"]);
});

test("stop skips the queue and empties it", async () => {
  const runner = new FakeRunner(), answers: string[] = [];
  const q = new CommandQueue(runner, () => {});
  const moves = q.submit(steer("d100 r90"), "page", a => answers.push(short(a)));
  await tick();
  const stopped = q.stop("cli", a => answers.push(short(a)));
  assert.deepEqual(runner.started, ["drive", "stop"]); // at once, past the running drive
  assert.deepEqual(answers, ["snap dropped (stop)", "steer r90 dropped (stop)", "snap dropped (stop)"]);
  runner.finish({ result: "stopped" }); // the drive
  runner.finish(); // the stop
  await Promise.all([moves, stopped]);
  assert.deepEqual(answers.slice(3).sort(), ["steer d100 stopped", "stop ok"]);
});

// The robot as the runner sees it: the lines sent; done records sent back by hand.
class FakeRobot extends EventEmitter {
  connected = true;
  lines: string[] = [];
  sendLine(line: string): boolean { if (this.connected) this.lines.push(line); return this.connected; }
  done(d: Partial<DoneRecord> & { end: string }): void {
    const n = Number(/#(\d+)$/.exec(this.lines.at(-1)!)![1]);
    this.emit("done", { n, pose: { x: 0.43, y: -0.04, yaw: Math.PI / 6 }, movedM: null, rotatedDeg: null, goalM: null, snapshot: null, ...d });
  }
}

test("the runner: numbered lines, done records by number, not connected, no answer, a stored snap", async () => {
  const robot = new FakeRobot(), stored = new StoredSnapshots();
  const runner = new CommandRunner(robot, stored, c => (c.name === "watch" ? 30 : 1000));
  const [drive, watch] = (parseSteer("d50 w5") as { commands: Command[] }).commands;
  const driving = runner.run(drive);
  assert.deepEqual(robot.lines, ["drive 50 #1"]);
  robot.emit("done", { n: 7, end: "ok" }); // nothing waits for 7: ignored
  robot.done({ end: "obstacle", movedM: 0.31, rotatedDeg: 0 });
  assert.deepEqual(await driving, { result: "obstacle", pose: { x: 43, y: -4, yaw: 30 }, moved: 31, rotated: 0 });
  assert.deepEqual(await runner.run(watch), { result: "failed", why: "no answer" });
  const snapping = runner.run(action("snap")[0]);
  robot.done({ end: "ok", snapshot: 41 });
  let settled = false;
  void snapping.then(() => { settled = true; });
  await tick();
  assert.ok(!settled); // not stored yet
  stored.add("1a2b3c4d", 41);
  assert.deepEqual(await snapping, { result: "ok", pose: { x: 43, y: -4, yaw: 30 }, snapshot: 41 });
  assert.equal(stored.newest(), 41);
  robot.connected = false;
  assert.deepEqual(await runner.run(drive), { result: "failed", why: "robot not connected" });
});

test("answers: limited, safety stops, refused, an action without a pose", async () => {
  const robot = new FakeRobot(), runner = new CommandRunner(robot, new StoredSnapshots());
  const run = async (c: Command, d: Partial<DoneRecord> & { end: string }) => {
    const going = runner.run(c);
    robot.done(d);
    return going;
  };
  const move = (text: string) => (parseSteer(text) as { commands: Command[] }).commands[0];
  assert.deepEqual(await run(move("g300,300"), { end: "ok", goalM: 4.24, movedM: 3, rotatedDeg: 45 }),
                   { result: "limited", pose: { x: 43, y: -4, yaw: 30 }, moved: 300, rotated: 45 });
  assert.deepEqual(await run(move("r30"), { end: "tilt", movedM: 0, rotatedDeg: 12 }),
                   { result: "safety_stop", why: "tilt", pose: { x: 43, y: -4, yaw: 30 }, moved: 0, rotated: 12 });
  assert.equal((await run(move("d30"), { end: "wheels" })).why, "wheels");
  assert.deepEqual(await run(action("recording", "on")[0], { end: "ok" }), { result: "ok" });
  assert.deepEqual(await run(move("d30"), { end: "refused" }), { result: "failed", why: "refused by the robot (robot log says why)" });
});
