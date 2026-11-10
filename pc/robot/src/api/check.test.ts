// Tests for check.ts: check's answers for drive, rotate and goto, each from where the
// one before ends, on a made-up map (where a drive stops is the robot's own rule, tested
// in maps/cell_map.test.ts and picoA's test_behaviour.c). Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import type { CellMap } from "../maps/cell_map.ts";
import { parseSteer, type Command } from "./commands.ts";
import { checkMoves } from "./check.ts";

// A drive forward (yaw 0) stops at x = -24 cm (a wall there); within r of (0, 0) an
// obstacle for turns.
const wall: CellMap = {
  async drive(x, _y, yaw, distanceM) {
    const room = yaw === 0 && distanceM > 0 ? Math.max(0, -0.24 - x) : Infinity;
    return room < distanceM ? { movedM: room, obstacle: true } : { movedM: Math.abs(distanceM), obstacle: false };
  },
  async obstacleWithin(x, y, r) { return Math.hypot(x, y) < r; },
  async view() { throw new Error("not used"); },
};
const steer = (text: string) => (parseSteer(text) as { commands: Command[] }).commands;
const start = { x: -1, y: 0.15, yaw: 0 };

test("check: drive, rotate, goto, each from where the one before ends", async () => {
  const short = async (text: string, from = start) => (await checkMoves(steer(text), wall, from)).map(a =>
    `${a.command} ${a.result}${a.why ? ` (${a.why})` : ""}${a.pose ? ` at ${a.pose.x},${a.pose.y},${a.pose.yaw} moved ${a.moved} rotated ${a.rotated}` : ""}`);
  assert.deepEqual(await short("d50 r90"), ["steer d50 ok at -50,15,0 moved 50 rotated 0", "steer r90 ok at -50,15,90 moved 0 rotated 90"]);
  assert.deepEqual(await short("d100 r30"), ["steer d100 obstacle at -24,15,0 moved 76 rotated 0", "steer r30 dropped (steer d100 ended: obstacle)"]);
  assert.equal((await short("d76 d10"))[1], "steer d10 obstacle at -24,15,0 moved 0 rotated 0");
  assert.equal((await short("d-30"))[0], "steer d-30 ok at -130,15,0 moved 30 rotated 0");
  assert.deepEqual(await short("g-100,-185"), ["steer g-100,-185 ok at -100,-185,-90 moved 200 rotated -90"]);
  assert.match((await short("g-250,250", { x: -2.5, y: -2.5, yaw: 0 }))[0], /^steer g-250,250 limited .* moved 300 rotated 90$/);
  const [turn] = await checkMoves(steer("r90"), wall, { x: 0.05, y: 0, yaw: 0 });
  assert.match(turn.warning ?? "", /an obstacle within 15 cm/);
});
