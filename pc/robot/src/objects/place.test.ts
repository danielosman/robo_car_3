// Tests for place.ts and labels.ts' check: a box's bottom edge on the floor, through the
// camera at 8.8 cm; the shape behind it in the map frame. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { CAMERA_GEOMETRY, corners, floorPoint, place } from "./place.ts";
import { checkReport } from "./labels.ts";

// A level camera, so the rows below are easy to work out; the real one is tilted (one test below).
const CAM = { ...CAMERA_GEOMETRY, tiltRad: 0 };
const near = (a: number | null, b: number, eps = 1e-3) => assert.ok(a !== null && Math.abs(a - b) < eps, `${a} is not ${b}`);
// The 0-1000 row whose floor point is d m ahead of a level camera.
const rowAt = (d: number) => (CAM.height / 2 + CAM.focalPx * CAM.z / d) / CAM.height * 1000;
const origin = { x: 0, y: 0, yaw: 0, pitch: 0 };

test("a box's bottom edge 1 m ahead: front edge, width, the shape behind it", () => {
  const p = place({ label: "chair", bbox: [300, 400, rowAt(1), 600] }, origin, CAM);
  assert.equal(p.floor, "floor");
  assert.equal(p.kind, "rect");
  near(p.distanceM, 1);
  near(p.width, 32 / CAM.focalPx);     // 32 px at 1 m
  near(p.angle, -Math.PI / 2);         // from its left end to its right end: towards -y
  near(p.cx, CAM.x + 1 + 0.25);        // half a chair's 50 cm behind the front edge
  near(p.cy, CAM.y);
  const c = corners(p);
  assert.equal(c.length, 4);
  near(Math.min(...c.map(([x]) => x)), CAM.x + 1);
  near(Math.max(...c.map(([x]) => x)), CAM.x + 1.5);
});

test("in the map frame: the robot's pose turns and moves it", () => {
  const p = place({ label: "chair", bbox: [300, 400, rowAt(1), 600] }, { x: 1, y: 2, yaw: Math.PI / 2, pitch: 0 }, CAM);
  near(p.cx, 1 - CAM.y);
  near(p.cy, 2 + CAM.x + 1.25);
  near(p.angle, 0);
});

test("off to the side: the bearing from the column", () => {
  const p = place({ label: "box", bbox: [300, 0, rowAt(2), 100] }, origin, CAM);
  const [fx, fy] = floorPoint(8, CAM.height / 2 + CAM.focalPx * CAM.z / 2, 0, CAM)!;
  near(Math.atan2(fy - CAM.y, fx - CAM.x), Math.atan(72 / CAM.focalPx));
  assert.ok(p.cy! > 0.5, "to the left");
});

test("not on the floor: at or above the horizon, too far; cut off at the bottom", () => {
  assert.equal(place({ label: "sofa", bbox: [100, 0, 500, 1000] }, origin, CAM).floor, "above_horizon");
  assert.equal(place({ label: "sofa", bbox: [100, 0, 480, 1000] }, origin, CAM).kind, null);
  assert.equal(place({ label: "table", bbox: [300, 400, rowAt(6), 600] }, origin, CAM).floor, "too_far");
  const cut = place({ label: "chair", bbox: [300, 400, 1000, 600] }, origin, CAM);
  assert.equal(cut.floor, "cut_off");
  near(cut.distanceM, CAM.focalPx * CAM.z / (CAM.height / 2)); // the bottom row: ~26 cm
});

test("the robot's pitch: nose up puts the same row further away", () => {
  const up = place({ label: "chair", bbox: [300, 400, rowAt(1), 600] }, { ...origin, pitch: 1 * Math.PI / 180 }, CAM);
  assert.ok(up.distanceM! > 1.2, `${up.distanceM}`);
});

test("doors and doorways are lines along the front edge", () => {
  const p = place({ label: "door", bbox: [0, 300, rowAt(1.5), 700] }, origin, CAM);
  assert.equal(p.kind, "line");
  near(p.cx, CAM.x + 1.5);
  assert.equal(corners(p).length, 2);
  assert.equal(place({ label: "doorway", bbox: [0, 300, rowAt(1.5), 700] }, origin, CAM).kind, "line");
});

test("checkReport: labels from the list, boxes on 0-1000, what to fix", () => {
  assert.deepEqual(checkReport({ objects: [{ label: "chair", bbox: [10, 20, 30.4, 40], confidence: 72.6 }] }),
                   { ok: true, detections: [{ label: "chair", bbox: [10, 20, 30, 40], confidence: 73 }] });
  assert.deepEqual(checkReport({ objects: [] }), { ok: true, detections: [] });
  const bad = checkReport({ objects: [{ label: "pouf", bbox: [10, 20, 30, 40], confidence: 50 }, { label: "box", bbox: [50, 20, 30, 40], confidence: 50 },
                                      { label: "box", bbox: [10, 20, 30, 1200], confidence: 50 }, { label: "box", bbox: [1, 2], confidence: 50 },
                                      { label: "box", bbox: [10, 20, 30, 40] }, { label: "box", bbox: [10, 20, 30, 40], confidence: 120 }] });
  assert.equal(bad.ok, false);
  const errors = (bad as { errors: string[] }).errors;
  assert.match(errors[0], /objects\[0\]: label "pouf" is not one of: chair, .*object\. Use object if nothing fits/);
  assert.match(errors[1], /objects\[1\]: bbox must have ymin < ymax/);
  assert.match(errors[2], /objects\[2\]: bbox values must be 0-1000/);
  assert.match(errors[3], /objects\[3\]: bbox must be four numbers/);
  assert.match(errors[4], /objects\[4\]: confidence must be a number 0-100/);
  assert.match(errors[5], /objects\[5\]: confidence must be a number 0-100/);
  assert.equal(checkReport({ objects: [{ label: "wall", bbox: [10, 20, 30, 40], confidence: 50 }] }).ok, false);
  assert.equal(checkReport([{ label: "chair" }]).ok, false);
  assert.equal(checkReport(null).ok, false);
});

test("the camera's tilt: looking down puts the same row closer, like the robot's nose down", () => {
  const box = { label: "chair" as const, bbox: [300, 400, rowAt(1), 600] as [number, number, number, number] };
  const down = place(box, origin, { ...CAM, tiltRad: -1 * Math.PI / 180 }), noseDown = place(box, { ...origin, pitch: -1 * Math.PI / 180 }, CAM);
  assert.ok(down.distanceM! < 0.9, `${down.distanceM}`);
  near(down.distanceM, noseDown.distanceM!);
  near(place(box, origin).distanceM, place(box, { ...origin, pitch: CAMERA_GEOMETRY.tiltRad }, CAM).distanceM!); // the default is tilted
});
