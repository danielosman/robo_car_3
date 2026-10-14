// A fake robot for trying the server, the page and the `robot` CLI without the robot:
// connects to the server as PicoA does (ROBOT_PORT on 127.0.0.1), speaks its protocol
// (picoA/app/commands.h, snapshot.h): takes command lines, answers each with a done
// record when it has ended, sends the pose line 4 times a second (with recording and
// small jolt and tilt numbers), and for `snapshot` the next snapshot of a raw
// snapshot file (e.g. npm run snapreplay's), one after another. It drives its pose at
// the robot's speeds (10 cm/s, 29°/s); it knows no map, so it never meets an obstacle.
//   npm run fakerobot -- <file.snap>
import net from "node:net";
import { readFileSync } from "node:fs";
import { readSnapshotFile, SNAP_MAGIC } from "./snapshot.ts";
import { ROBOT_PORT } from "./robot.ts";

const [file] = process.argv.slice(2);
if (!file) { console.error("usage: npm run fakerobot -- <file.snap>"); process.exit(2); }
const raw = readFileSync(file), snaps: Buffer[] = [];
for (let at = 8; at + 17 <= raw.length;) { // the file's snapshots as bytes, as the robot sends them
  const length = raw.readUInt32LE(at + 13);
  snaps.push(raw.subarray(at + 8, at + 8 + length));
  at += 8 + length;
}
const parsed = [...readSnapshotFile(raw)].map(s => s.snapshot);
if (!snaps.length || snaps[0].subarray(0, 4).toString("latin1") !== SNAP_MAGIC) { console.error("no snapshots in", file); process.exit(1); }

const STEP_S = 0.05, SPEED_MPS = 0.1, TURN_RADPS = 0.5, GOTO_MAX_M = 3;
const socket = net.connect(Number(process.env.ROBOT_PORT ?? ROBOT_PORT), "127.0.0.1");
const pose = { x: parsed[0]?.pose?.x ?? 0, y: parsed[0]?.pose?.y ?? 0, yaw: parsed[0]?.pose?.yaw ?? 0 };
let mode = "manual", recording = false, next = 0;
// The running action: what it still has to do, step by step; its command's number.
let action: { name: string; take: string; n: number; steps: (() => boolean)[]; start: { x: number; y: number }; goal?: number } | null = null;
const say = (text: string) => { socket.write(text); process.stdout.write(`robot: ${text}`); };
const record = (text: string) => socket.write(`\x01${text}\n`);
const f1 = (v: number) => v.toFixed(1);

function done(n: number, end: string, more = "", at = pose): void {
  record(`done n=${n} end=${end} x=${f1(at.x * 100)} y=${f1(at.y * 100)} yaw=${f1(at.yaw * 180 / Math.PI)}${more}`);
}

function end(how: string): void {
  if (!action) return;
  const a = action, moved = Math.hypot(pose.x - a.start.x, pose.y - a.start.y) * 100;
  action = null;
  say(`${a.name} ${how}\n`);
  done(a.n, how === "done" ? "ok" : how, ` moved=${moved.toFixed(0)}${a.goal !== undefined ? ` goal=${(a.goal * 100).toFixed(0)}` : ""}`);
}

const turn = (rad: number) => {
  let left = Math.abs(rad);
  return () => { const d = Math.min(left, TURN_RADPS * STEP_S); pose.yaw += Math.sign(rad) * d; left -= d; return left <= 0; };
};
const drive = (m: number, dir = () => pose.yaw) => {
  let left = Math.abs(m);
  return () => {
    const d = Math.min(left, SPEED_MPS * STEP_S), s = Math.sign(m);
    pose.x += s * d * Math.cos(dir()); pose.y += s * d * Math.sin(dir()); left -= d;
    return left <= 0;
  };
};
const wait = (s: number) => { let left = s; return () => (left -= STEP_S) <= 0; };

// name as the robot prints it, take as in recordings and the pose line.
function begin(name: string, take: string, n: number, steps: (() => boolean)[], goal?: number): void {
  end("replaced");
  if (mode === "idle") { mode = "manual"; say("Mode: manual\n"); }
  action = { name, take, n, steps, start: { x: pose.x, y: pose.y }, goal };
  say(`${name}: started\n`);
}

function sendSnapshot(n: number | null): void {
  const s = snaps[next % snaps.length], p = parsed[next % snaps.length];
  next++;
  for (let at = 0; at < s.length; at += 1400) { // in chunks, between the text, as wifi_console.c does
    const chunk = s.subarray(at, at + 1400), head = Buffer.from([2, chunk.length & 255, chunk.length >> 8]);
    socket.write(Buffer.concat([head, chunk]));
  }
  if (n !== null) done(n, "ok", ` snapshot=${p?.seq ?? 0}`, p?.pose ?? pose);
}

function command(line: string): void {
  const m = /^(.*?)(?:\s*#(\d+))?$/.exec(line.trim())!, text = m[1], n = Number(m[2] ?? 0);
  const num = (re: RegExp) => re.exec(text)?.slice(1).map(Number) ?? null;
  let v: number[] | null;
  if ((v = num(/^drive (-?\d+)$/))) begin("Move", "move", n, [drive(v[0] / 100)]);
  else if ((v = num(/^rotate (-?\d+)$/))) begin("Turn", "turn", n, [turn(v[0] * Math.PI / 180)]);
  else if ((v = num(/^watch (\d+)$/))) begin("Watch", "watch", n, [wait(v[0])]);
  else if ((v = num(/^goto (-?\d+) (-?\d+)$/))) {
    const dx = v[0] / 100 - pose.x, dy = v[1] / 100 - pose.y, d = Math.hypot(dx, dy), bearing = Math.atan2(dy, dx);
    const towards = Math.atan2(Math.sin(bearing - pose.yaw), Math.cos(bearing - pose.yaw));
    begin("Go to", "goto", n, [turn(towards), drive(Math.min(d, GOTO_MAX_M))], d);
  } else if (text === "snapshot") sendSnapshot(n);
  else if (text === "clear_map") { say("Map cleared\n"); done(n, "ok"); }
  else if (/^recording (on|off)$/.test(text)) { recording = text.endsWith("on"); say(`Recording: ${recording ? "on" : "off"}\n`); done(n, "ok"); }
  else if (/^mode (idle|manual)$/.test(text)) {
    mode = text.slice(5);
    if (mode === "idle") end("stopped");
    say(`Mode: ${mode}\n`);
    done(n, "ok");
  } else if (text === "stop") { end("stopped"); mode = "idle"; say("Stopped, motors off\nMode: idle\n"); done(n, "ok"); }
  else if (text === "status") { say(`fake robot at (${f1(pose.x * 100)}, ${f1(pose.y * 100)}) cm\n`); done(n, "ok"); }
  else { say(`Command refused: "${text}"\n`); done(n, "refused"); }
}

socket.on("connect", () => {
  say("WiFi: connected (fake robot)\nServer: connected\n");
  sendSnapshot(null); // a server connected
  setInterval(() => {
    if (!action) return;
    while (action.steps.length && action.steps[0]()) action.steps.shift();
    if (!action.steps.length) end("done");
  }, STEP_S * 1000);
  setInterval(() => {
    const jolt = () => (Math.random() * 0.05).toFixed(2), tilt = (Math.random() * 1.5).toFixed(1);
    record(`pose ${f1(pose.x * 100)} ${f1(pose.y * 100)} ${f1(pose.yaw * 180 / Math.PI)} ${mode} ${action?.take ?? "-"} -55 ` +
           `${recording ? 1 : 0} ${jolt()} ${jolt()} ${jolt()} ${tilt}`);
  }, 250);
});
let line = "";
socket.on("data", (d: Buffer) => {
  for (const ch of d.toString("latin1")) {
    if (ch === "\n") { command(line); line = ""; }
    else if (ch >= " ") line += ch; // the server's heartbeat ('\0') is skipped
  }
});
socket.on("close", () => { console.log("server closed the connection"); process.exit(0); });
socket.on("error", err => { console.error(err.message); process.exit(1); });
