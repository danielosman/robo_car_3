// Tests for link.ts with a robot played on localhost: the announcement; hello first
// (connected only then, with the boot); console lines, status (the pose and the
// signal; no peaks), answers (the done records the runner waits for), map cleared,
// moves, tof frames, imu batches and snapshot parts as events, whatever TCP's chunks (split anywhere, merged); command
// messages and heartbeats to the robot; a robot of another wire version, one whose
// first message isn't hello and an old robot sending text refused with a note; a
// robot heard from nothing reported and later closed; a second robot replacing the
// first. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import dgram from "node:dgram";
import net from "node:net";
import { once } from "node:events";
import { RobotLink, type DoneRecord, type LivePose, type MapWindow, type RobotStatus } from "./link.ts";
import { packTof, unpackTof, type PosedFrame } from "./tof_format.ts";
import {
  HAS, MAP_PART_CELLS, MAP_PARTS, TYPE, WIRE_VERSION, WireReader, decode, encodeAnswer, encodeConsole, encodeHello, encodeMapCleared, encodeMapPart,
  encodeMoveEnd, encodeMoveStart, encodeImuBatch, encodeSnapshotPart, encodeStatus, encodeTof, type ImuBatch, type Message, type MoveEnd, type TofMessage, type MoveStart,
} from "./wire.ts";

async function startLink(quietMs = 3000, dropMs = 6000) {
  const listener = dgram.createSocket("udp4");
  await new Promise<void>(r => listener.bind(0, "127.0.0.1", () => r()));
  const link = new RobotLink({
    robotPort: 0, announcePort: listener.address().port, announceTo: ["127.0.0.1"],
    announceEveryMs: 50, heartbeatEveryMs: 50, quietMs, dropMs,
  });
  const statuses: RobotStatus[] = [], notes: string[] = [];
  link.on("status", s => statuses.push(s));
  link.on("note", n => notes.push(n));
  const port = await link.start();
  const [message] = await once(listener, "message");
  listener.close();
  return { link, port, announcement: String(message), statuses, notes };
}

const hello = (version = WIRE_VERSION, bootId = "7a587226") => encodeHello({ version, bootId, build: "Nov  7 2026 18:00:00" });
const status = (rssi = -60) => encodeStatus({ x: 0.01, y: 0.02, yaw: 0.5, mode: "manual", action: "rotate", rssi });

// A robot: what it received from the server, as messages.
async function robot(port: number, first: Buffer | null = hello()) {
  const socket = net.connect(port, "127.0.0.1");
  await once(socket, "connect");
  const reader = new WireReader(), received: Message[] = [];
  socket.on("data", (d: Buffer) => received.push(...reader.push(d).map(decode)));
  if (first) socket.write(first);
  return { socket, received };
}

const until = async (ok: () => boolean) => { for (let i = 0; i < 200 && !ok(); i++) await new Promise(r => setTimeout(r, 10)); assert.ok(ok()); };

test("announces itself; hello first; console lines, status, commands, heartbeats", async () => {
  const { link, port, announcement, statuses, notes } = await startLink();
  try {
    assert.equal(announcement, `ROBOCAR-SERVER ${port}\n`); // no recording port any more
    const text: string[] = [], hellos: unknown[] = [], poses: LivePose[] = [];
    link.on("text", t => text.push(t));
    link.on("hello", h => hellos.push(h));
    link.on("pose", p => poses.push(p));
    const r = await robot(port, null);
    await new Promise(res => setTimeout(res, 50));
    assert.equal(link.status.state, "waiting"); // connected only once hello came
    r.socket.write(hello());
    await until(() => link.status.state === "connected");
    assert.deepEqual(hellos, [{ version: WIRE_VERSION, bootId: "7a587226", build: "Nov  7 2026 18:00:00" }]);
    assert.ok(notes.some(n => n === "robot connected from 127.0.0.1 (boot 7a587226, firmware of Nov  7 2026 18:00:00)"));
    r.socket.write(Buffer.concat([encodeConsole("WiFi: connected"), status(), encodeConsole("Turn: 90°")]));
    await until(() => text.join("") === "WiFi: connected\nTurn: 90°\n");
    await until(() => link.status.state === "connected" && link.status.rssi === -60);
    assert.equal(poses.length, 1);
    const p = poses[0];
    assert.deepEqual([p.x, p.y, p.mode, p.action, p.rssi], [Math.fround(0.01), Math.fround(0.02), "manual", "rotate", -60]);
    assert.deepEqual(Object.keys(p).sort(), ["action", "at", "mode", "rssi", "x", "y", "yaw"]); // no peaks since step 4c
    assert.ok(link.sendCommand(1, "status"));
    assert.throws(() => link.sendCommand(2, "goto\n1 2")); // one line of printable text only
    await until(() => r.received.some(m => m.type === "command"));
    assert.deepEqual(r.received.find(m => m.type === "command"), { type: "command", n: 1, text: "status" });
    await until(() => r.received.some(m => m.type === "heartbeat"));
    r.socket.destroy();
    await until(() => link.status.state === "lost");
    assert.equal(link.sendCommand(3, "status"), false);
    assert.deepEqual(statuses.map(s => s.state), ["connected", "connected", "lost"]);
  } finally { link.stop(); }
});

test("answers, map cleared, map window, moves, tof frames and snapshot parts, whatever the TCP chunks", async () => {
  const { link, port } = await startLink();
  try {
    const snapshots: Buffer[] = [], poses: LivePose[] = [], dones: DoneRecord[] = [], text: string[] = [];
    const events: string[] = [], starts: MoveStart[] = [], ends: MoveEnd[] = [], frames: PosedFrame[] = [], windows: MapWindow[] = [];
    let cleared = 0;
    link.on("moveStart", m => { starts.push(m); events.push(`start ${m.move}`); });
    link.on("moveEnd", m => { ends.push(m); events.push(`end ${m.move}`); });
    link.on("tof", (f: TofMessage) => { frames.push(f.frame); events.push(`${f.format} ${f.frame.number} of ${f.move}`); });
    link.on("snapshot", b => snapshots.push(b));
    link.on("mapWindow", w => windows.push(w));
    link.on("pose", p => poses.push(p));
    link.on("done", d => dones.push(d));
    link.on("text", t => text.push(t));
    link.on("mapCleared", () => cleared++);
    const snap = Buffer.alloc(3000, 9);
    snap.write("SNAP", 0, "latin1");
    snap.writeUInt32LE(snap.length, 5);
    const start: MoveStart = { move: 3, command: 4, kind: "goto", parameter: 4.25, tUs: 0xfffffff0, preMs: 200, postMs: 300, refused: false };
    const end: MoveEnd = { move: 3, end: "obstacle", tUs: 25, framesSent: 2, framesDropped: 0, moved: 1.25, rotated: 0.5,
                          imuSent: 1, imuDropped: 0 };
    const imu: ImuBatch = { move: 3, records: [0, 1, 2].map(i => ({ tUs: (0xffffe000 + i * 9615), acceleration: [i, -i - 1, 16384], gyro: [-32768, 32767, i] })) };
    const batches: ImuBatch[] = [];
    link.on("imuBatch", (b: ImuBatch) => { batches.push(b); events.push(`imu ${b.move}`); });
    const zones = Array.from({ length: 64 }, (_, k) => ({ distanceMm: 10 * k, signal: k, sigmaMm: 2, kind: k ? "sure" as const : "nothing" as const, targets: k ? 1 : 0 }));
    const tof = (number: number): PosedFrame => ({ number, robotClockUs: 100 * number, periodUs: 100000, x: 1.5, y: -0.25, yaw: 0.5, pitch: 0, turn: 0,
                                                    still: false, poseKnown: true, beyondMm: 2550, zones });
    const mapCells = Buffer.from(Array.from({ length: MAP_PARTS * MAP_PART_CELLS }, (_, i) => (i * 7 + 1) & 0xff));
    const mapParts = Array.from({ length: MAP_PARTS }, (_, part) =>
      encodeMapPart({ part, originCx: -20, originCy: 5, cells: mapCells.subarray(part * MAP_PART_CELLS, (part + 1) * MAP_PART_CELLS) }));
    const stream = Buffer.concat([
      encodeMoveStart(start), encodeTof("compact", 3, 5000, tof(50)), encodeMoveEnd(end), encodeTof("compact", 3, 5100, tof(51)), encodeImuBatch(imu),
      encodeConsole("Go to: ..."),
      encodeAnswer({ n: 4, end: "obstacle", has: HAS.MOVE | HAS.GOAL, x: 1.2, y: -0.35, yaw: -0.25, moved: 1.25, rotated: Math.PI / 2, goal: 4.2, snapshot: 0 }),
      encodeSnapshotPart(snap.subarray(0, 1400)), status(), encodeSnapshotPart(snap.subarray(1400, 2800)), encodeMapCleared(),
      encodeSnapshotPart(snap.subarray(2800)), ...mapParts,
      encodeAnswer({ n: 5, end: "ok", has: HAS.SNAPSHOT, x: 1, y: 2, yaw: 0, moved: 0, rotated: 0, goal: 0, snapshot: 41 }),
    ]);
    for (const size of [1, 2, 3, 7, 1000, stream.length]) {
      snapshots.length = poses.length = dones.length = text.length = cleared = windows.length = 0;
      events.length = starts.length = ends.length = frames.length = batches.length = 0;
      const r = await robot(port);
      await until(() => link.status.state === "connected");
      for (let at = 0; at < stream.length; at += size) {
        r.socket.write(stream.subarray(at, at + size));
        if (size < 10 && at % 500 === 0) await new Promise(res => setImmediate(res)); // separate TCP chunks now and then
      }
      await until(() => dones.length === 2 && snapshots.length === 1 && poses.length === 1 && cleared === 1 && windows.length === 1);
      assert.deepEqual(snapshots[0], snap);
      assert.deepEqual(windows[0], { originX: -20, originY: 5, cellsX: 60, cellsY: 60, cells: mapCells });
      assert.deepEqual(text, ["Go to: ...\n"]);
      assert.deepEqual(events, ["start 3", "compact 50 of 3", "end 3", "compact 51 of 3", "imu 3"]); // in the order they came
      assert.deepEqual(batches, [imu]); // the imu batch's records as they went (times evenly spaced: exact)
      assert.deepEqual(starts, [start]);
      assert.deepEqual(ends, [end]);
      assert.deepEqual(frames[1], unpackTof("compact", packTof("compact", tof(51)))); // as packed: yaw in 0.01°
      const d = dones[0];
      assert.equal(d.n, 4);
      assert.equal(d.end, "obstacle");
      assert.deepEqual(d.pose, { x: Math.fround(1.2), y: Math.fround(-0.35), yaw: -0.25 });
      assert.equal(d.movedM, 1.25);
      assert.ok(Math.abs(d.rotatedDeg! - 90) < 1e-4);
      assert.equal(d.goalM, Math.fround(4.2));
      assert.equal(d.snapshot, null);
      assert.deepEqual([dones[1].n, dones[1].snapshot, dones[1].movedM, dones[1].goalM], [5, 41, null, null]);
      r.socket.destroy();
      await until(() => link.status.state === "lost");
    }
  } finally { link.stop(); }
});

test("refused: another wire version, no hello first, an old robot's text, a message for the robot", async () => {
  const { link, port, notes } = await startLink();
  try {
    const cases: [Buffer, RegExp][] = [
      [hello(WIRE_VERSION + 1), new RegExp(`speaks wire version ${WIRE_VERSION + 1}, this server ${WIRE_VERSION} \\(flash PicoA`)],
      [status(), /its first message is status, not hello/],
      [Buffer.from("WiFi: connected\n\x01pose 1 2 3 idle -\n"), /unknown message type 0x57 \(not a robot speaking the wire, or an old firmware\)/],
      [Buffer.concat([hello(), Buffer.from([TYPE.HEARTBEAT, 0, 0])]), /a message for the robot/],
      [Buffer.concat([hello(), Buffer.from([TYPE.ANSWER, 3, 0, 1, 2, 3])]), /too short/],
    ];
    for (const [bytes, why] of cases) {
      notes.length = 0;
      const r = await robot(port, bytes);
      await once(r.socket, "close");
      assert.ok(notes.some(n => /^robot connection from 127\.0\.0\.1 refused: /.test(n) && why.test(n)), `${why}: ${notes.join(" | ")}`);
      assert.notEqual(link.status.state, "connected");
    }
  } finally { link.stop(); }
});

test("another version with another hello length: refused for its version, said once per robot", async () => {
  const { link, port, notes } = await startLink();
  try {
    const other = Buffer.concat([Buffer.from([TYPE.HELLO, 40, 0, WIRE_VERSION + 1]), Buffer.alloc(39)]);
    for (let i = 0; i < 3; i++) { const r = await robot(port, other); await once(r.socket, "close"); }
    const said = notes.filter(n => n.includes(`it speaks wire version ${WIRE_VERSION + 1}, this server ${WIRE_VERSION}`));
    assert.equal(said.length, 1); // it reconnects every few seconds: said once
    assert.ok(!notes.some(n => /not 29/.test(n)));
    const r = await robot(port, hello(WIRE_VERSION + 2)); // another version: said again
    await once(r.socket, "close");
    assert.equal(notes.filter(n => n.includes("refused: it speaks wire version")).length, 2);
    assert.notEqual(link.status.state, "connected");
  } finally { link.stop(); }
});

test("nothing from the robot: reported, then closed; a new robot replaces the old one", async () => {
  const { link, port, notes } = await startLink(200, 600);
  try {
    const a = await robot(port);
    await until(() => link.status.state === "connected");
    const b = await robot(port);
    await once(a.socket, "close");
    assert.equal(link.status.state, "connected");
    b.socket.write(status(-58));
    await until(() => link.status.state === "connected" && link.status.rssi === -58); // the signal from the status
    // Nothing for 200 ms: said so, the connection kept.
    await until(() => link.status.state === "connected" && link.status.quietSince !== null);
    assert.ok(notes.some(n => /^nothing from the robot for \d+ s; the connection is open/.test(n)));
    assert.ok(!b.socket.destroyed);
    b.socket.write(encodeConsole("late"));
    await until(() => link.status.state === "connected" && link.status.quietSince === null);
    assert.ok(notes.some(n => /^data from the robot again, after \d+ s$/.test(n)));
    // Nothing for 600 ms: closed.
    await once(b.socket, "close");
    await until(() => link.status.state === "lost");
    assert.ok(notes.some(n => /^nothing from the robot for \d+ s: closing the connection/.test(n)));
  } finally { link.stop(); }
});
