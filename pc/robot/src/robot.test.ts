// Tests for robot.ts: splitting the robot's status records out of its text, and
// the link end to end with a fake robot on localhost (announcement, text, signal,
// keys, a robot heard from nothing reported and later closed, a second robot replacing the first). Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import dgram from "node:dgram";
import net from "node:net";
import { once } from "node:events";
import { RecordSplitter, RobotLink, parsePose, type LivePose, type RobotStatus } from "./robot.ts";

test("status records are taken out of the text, wherever they land", () => {
  const s = new RecordSplitter();
  assert.deepEqual(s.push(Buffer.from("x +1.0 cm  y \x01rssi -5")), { text: "x +1.0 cm  y ", records: [], snapshotBytes: [] });
  assert.deepEqual(s.push(Buffer.from("5\n+2.0 cm\r\n")), { text: "+2.0 cm\n", records: ["rssi -55"], snapshotBytes: [] });
  const deg = Buffer.from("90°\n");
  assert.equal(s.push(deg.subarray(0, 3)).text, "90");
  assert.equal(s.push(deg.subarray(3)).text, "°\n");
});

// [0x02][length u16][bytes], as wifi_console.c sends snapshot bytes.
const chunk = (bytes: Buffer) => Buffer.concat([Buffer.from([2, bytes.length & 255, bytes.length >> 8]), bytes]);

test("snapshot chunks come out of the stream between any two bytes, also inside a record", () => {
  const s = new RecordSplitter();
  const a = Buffer.from("SNAP\x01\x02\n\x01binary"), b = Buffer.alloc(300, 7);
  const stream = Buffer.concat([Buffer.from("line 1\n\x01rss"), chunk(a), Buffer.from("i -5"), chunk(b), Buffer.from("5\nline 2\n")]);
  for (const size of [1, 2, 3, 7, 1000]) {
    let text = "";
    const records: string[] = [], bytes: Buffer[] = [];
    for (let at = 0; at < stream.length; at += size) {
      const out = s.push(stream.subarray(at, at + size));
      text += out.text;
      records.push(...out.records);
      bytes.push(...out.snapshotBytes);
    }
    assert.equal(text, "line 1\nline 2\n");
    assert.deepEqual(records, ["rssi -55"]);
    assert.deepEqual(Buffer.concat(bytes), Buffer.concat([a, b]));
  }
});

test("pose records", () => {
  const p = parsePose("pose 125.0 -50.0 43.0 manual goto")!;
  assert.equal(p.x, 1.25);
  assert.equal(p.y, -0.5);
  assert.ok(Math.abs(p.yaw - 43 * Math.PI / 180) < 1e-12);
  assert.equal(p.mode, "manual");
  assert.equal(p.action, "goto");
  assert.equal(parsePose("pose 1 2 3 idle -")!.action, "");
  assert.equal(p.rssi, null); // older firmware: no signal
  assert.equal(parsePose("pose 125.0 -50.0 43.0 manual goto -61")!.rssi, -61);
  assert.equal(parsePose("pose 1 2 3 idle - 0")!.rssi, null); // not connected to the server when written
  assert.equal(parsePose("rssi -50"), null);
});

async function startLink(quietMs = 3000, dropMs = 6000) {
  const listener = dgram.createSocket("udp4");
  await new Promise<void>(r => listener.bind(0, "127.0.0.1", () => r()));
  const link = new RobotLink({
    robotPort: 0, dataPort: 4999, announcePort: listener.address().port, announceTo: ["127.0.0.1"],
    announceEveryMs: 50, heartbeatEveryMs: 50, quietMs, dropMs,
  });
  const statuses: RobotStatus[] = [];
  link.on("status", s => statuses.push(s));
  const port = await link.start();
  const [msg] = await once(listener, "message");
  listener.close();
  return { link, port, announcement: String(msg), statuses };
}

async function robot(port: number) {
  const socket = net.connect(port, "127.0.0.1");
  await once(socket, "connect");
  const received: string[] = [];
  socket.on("data", d => received.push(String(d)));
  return { socket, received };
}

const until = async (ok: () => boolean) => { for (let i = 0; i < 200 && !ok(); i++) await new Promise(r => setTimeout(r, 10)); assert.ok(ok()); };

test("announces itself, passes text and keys, shows the signal", async () => {
  const { link, port, announcement, statuses } = await startLink();
  try {
    assert.equal(announcement, `ROBOCAR-SERVER ${port} 4999\n`);
    const text: string[] = [];
    link.on("text", t => text.push(t));
    const r = await robot(port);
    await until(() => link.status.state === "connected");
    r.socket.write("WiFi: connected\n\x01rssi -60\nKeys: g\n");
    await until(() => text.join("") === "WiFi: connected\nKeys: g\n");
    await until(() => link.status.state === "connected" && link.status.rssi === -60);
    assert.ok(link.sendKey("p"));
    await until(() => r.received.join("").replace(/\0/g, "") === "p");
    await until(() => r.received.join("").includes("\0")); // heartbeats
    r.socket.destroy();
    await until(() => link.status.state === "lost");
    assert.equal(link.sendKey("p"), false);
    assert.deepEqual(statuses.map(s => s.state), ["connected", "connected", "lost"]);
  } finally { link.stop(); }
});

test("commands to the robot; snapshots and poses from it", async () => {
  const { link, port } = await startLink();
  try {
    const snaps: Buffer[] = [], poses: LivePose[] = [];
    link.on("snapshot", b => snaps.push(b));
    link.on("pose", p => poses.push(p));
    const r = await robot(port);
    await until(() => link.status.state === "connected");
    assert.ok(link.sendCommand("goal 120 -35"));
    assert.ok(!link.sendCommand("goal\n1 2")); // one line of printable text only
    await until(() => r.received.join("").replace(/\0/g, "") === "\x02goal 120 -35\n");
    const snap = Buffer.alloc(40);
    snap.write("SNAP", 0, "latin1");
    snap.writeUInt32LE(40, 5);
    r.socket.write(Buffer.concat([chunk(snap.subarray(0, 25)), Buffer.from("\x01pose 1 2 3 idle -\n"), chunk(snap.subarray(25))]));
    await until(() => snaps.length === 1 && poses.length === 1);
    assert.deepEqual(snaps[0], snap);
    assert.equal(poses[0].y, 0.02);
    r.socket.destroy();
  } finally { link.stop(); }
});

test("nothing from the robot: reported, then closed; a new robot replaces the old one", async () => {
  const { link, port } = await startLink(200, 600);
  try {
    const notes: string[] = [];
    link.on("note", n => notes.push(n));
    const a = await robot(port);
    await until(() => link.status.state === "connected");
    const b = await robot(port);
    await once(a.socket, "close");
    assert.equal(link.status.state, "connected");
    b.socket.write("\x01pose 1 2 3 idle - -58\n");
    await until(() => link.status.state === "connected" && link.status.rssi === -58); // the signal from the pose record
    // Nothing for 200 ms: said so, the connection kept.
    await until(() => link.status.state === "connected" && link.status.quietSince !== null);
    assert.ok(notes.some(n => /^nothing from the robot for \d+ s; the connection is open/.test(n)));
    assert.ok(!b.socket.destroyed);
    b.socket.write("late\n");
    await until(() => link.status.state === "connected" && link.status.quietSince === null);
    assert.ok(notes.some(n => /^data from the robot again, after \d+ s$/.test(n)));
    // Nothing for 600 ms: closed.
    await once(b.socket, "close");
    await until(() => link.status.state === "lost");
    assert.ok(notes.some(n => /^nothing from the robot for \d+ s: closing the connection/.test(n)));
  } finally { link.stop(); }
});
