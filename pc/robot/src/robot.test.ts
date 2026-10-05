// Tests for robot.ts: splitting the robot's status records out of its text, and
// the link end to end with a fake robot on localhost (announcement, text, signal,
// keys, silence, a second robot replacing the first). Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import dgram from "node:dgram";
import net from "node:net";
import { once } from "node:events";
import { RecordSplitter, RobotLink, type RobotStatus } from "./robot.ts";

test("status records are taken out of the text, wherever they land", () => {
  const s = new RecordSplitter();
  assert.deepEqual(s.push(Buffer.from("x +1.0 cm  y \x01rssi -5")), { text: "x +1.0 cm  y ", records: [] });
  assert.deepEqual(s.push(Buffer.from("5\n+2.0 cm\r\n")), { text: "+2.0 cm\n", records: ["rssi -55"] });
  const deg = Buffer.from("90°\n");
  assert.equal(s.push(deg.subarray(0, 3)).text, "90");
  assert.equal(s.push(deg.subarray(3)).text, "°\n");
});

async function startLink(silenceMs = 3000) {
  const listener = dgram.createSocket("udp4");
  await new Promise<void>(r => listener.bind(0, "127.0.0.1", () => r()));
  const link = new RobotLink({
    robotPort: 0, announcePort: listener.address().port, announceTo: ["127.0.0.1"],
    announceEveryMs: 50, heartbeatEveryMs: 50, silenceMs,
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
    assert.equal(announcement, `ROBOCAR-SERVER ${port}\n`);
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

test("a silent robot is dropped; a new robot replaces the old one", async () => {
  const { link, port } = await startLink(300);
  try {
    const a = await robot(port);
    await until(() => link.status.state === "connected");
    const b = await robot(port);
    await once(a.socket, "close");
    assert.equal(link.status.state, "connected");
    b.socket.write("\x01rssi -50\n");
    await new Promise(r => setTimeout(r, 200));
    assert.equal(link.status.state, "connected"); // heard 200 ms ago
    await once(b.socket, "close");                // then silent for 300 ms
    await until(() => link.status.state === "lost");
  } finally { link.stop(); }
});
