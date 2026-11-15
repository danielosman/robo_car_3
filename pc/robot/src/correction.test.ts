// Tests for correction.ts: corrected and uncorrected undo each other; which correction a
// message has (the one set before it; the message that sets it is not yet corrected); rawOf.
// Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { correctionAt, corrected, NO_CORRECTION, rawOf, uncorrected } from "./correction.ts";

const c = { x: 0.1, y: -0.2, yaw: 0.3 }, d = { x: -1, y: 0, yaw: -0.5 };

test("corrected: turned about the origin, then moved; uncorrected undoes it; other fields kept", () => {
  const p = corrected({ x: 1, y: 0, yaw: Math.PI / 2 }, { x: 1, y: 0, yaw: 0.25, mode: "manual" });
  assert.ok(Math.abs(p.x - 1) < 1e-12 && Math.abs(p.y - 1) < 1e-12 && Math.abs(p.yaw - 0.25 - Math.PI / 2) < 1e-12 && p.mode === "manual");
  // The robot's twin (picoA/app/correction.h) gives the same in test_body.c: (2, 0.5) by 90° and 1 m along y.
  const r = corrected({ x: 0, y: 1, yaw: Math.PI / 2 }, { x: 2, y: 0.5, yaw: 0.25 });
  assert.ok(Math.abs(r.x + 0.5) < 1e-12 && Math.abs(r.y - 3) < 1e-12);
  const q = uncorrected(c, corrected(c, { x: 2, y: 3, yaw: 1 }));
  assert.ok(Math.hypot(q.x - 2, q.y - 3) < 1e-12 && Math.abs(q.yaw - 1) < 1e-12);
});

test("the correction of a message: the last set before it; none before the first", () => {
  const rows = [{ messageNumber: 10, c }, { messageNumber: 20, c: d }];
  assert.deepEqual(correctionAt(rows, 5), NO_CORRECTION);
  assert.deepEqual(correctionAt(rows, 10), NO_CORRECTION);  // the "corrected" message itself
  assert.deepEqual(correctionAt(rows, 11), c);
  assert.deepEqual(correctionAt(rows, 20), c);
  assert.deepEqual(correctionAt(rows, 21), d);
  const p = { x: 1, y: 2, yaw: 0.5 };
  assert.deepEqual(rawOf([], 30, p), p);
  const back = corrected(d, rawOf(rows, 30, p));
  assert.ok(Math.hypot(back.x - 1, back.y - 2) < 1e-12 && Math.abs(back.yaw - 0.5) < 1e-12);
});
