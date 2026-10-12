// A fake robot for trying the viewer without the robot: connects to the server as
// PicoA does (found by its announcement, or ROBOT_PORT on 127.0.0.1), prints a few
// console lines, sends where it is 4 times a second, and the snapshots of a raw
// snapshot file one after another (e.g. npm run snapreplay's), every few seconds.
// It answers "mode idle|manual" and, in manual mode, "goal x y" by driving its pose
// there at 10 cm/s, saying "Go to: ..." and "Go to done" as the robot does. Space
// stops and switches to idle.
//   npm run fakerobot -- <file.snap> [seconds between snapshots, default 4]
import net from "node:net";
import { readFileSync } from "node:fs";
import { readSnapshotFile, SNAP_MAGIC } from "./snapshot.ts";
import { ROBOT_PORT } from "./robot.ts";

const [file, every] = process.argv.slice(2);
if (!file) { console.error("usage: npm run fakerobot -- <file.snap> [seconds between snapshots]"); process.exit(2); }
const raw = readFileSync(file), snaps: Buffer[] = [];
for (let at = 8; at + 17 <= raw.length;) { // the file's snapshots as bytes, as the robot sends them
  const length = raw.readUInt32LE(at + 13);
  snaps.push(raw.subarray(at + 8, at + 8 + length));
  at += 8 + length;
}
const parsed = [...readSnapshotFile(raw)].map(s => s.snapshot);
if (!snaps.length || snaps[0].subarray(0, 4).toString("latin1") !== SNAP_MAGIC) { console.error("no snapshots in", file); process.exit(1); }

const port = Number(process.env.ROBOT_PORT ?? ROBOT_PORT);
const socket = net.connect(port, "127.0.0.1");
let mode = "idle", action = "", next = 0;
const pose = { x: parsed[0]?.pose?.x ?? 0, y: parsed[0]?.pose?.y ?? 0, yaw: parsed[0]?.pose?.yaw ?? 0 };
let goal: { x: number; y: number } | null = null;
const say = (text: string) => { socket.write(text); process.stdout.write(`robot: ${text}`); };

function sendSnapshot(): void {
  const s = snaps[next % snaps.length], p = parsed[next % snaps.length];
  next++;
  if (p?.pose && !goal) Object.assign(pose, { x: p.pose.x, y: p.pose.y, yaw: p.pose.yaw });
  for (let at = 0; at < s.length; at += 1400) { // in chunks, between the text, as wifi_console.c does
    const chunk = s.subarray(at, at + 1400), head = Buffer.from([2, chunk.length & 255, chunk.length >> 8]);
    socket.write(Buffer.concat([head, chunk]));
  }
}

socket.on("connect", () => {
  say("WiFi: connected (fake robot)\nServer: connected\n");
  sendSnapshot();
  setInterval(sendSnapshot, Number(every ?? 4) * 1000);
  setInterval(() => {
    if (goal) { // drive towards it at 10 cm/s, 4 steps a second
      const dx = goal.x - pose.x, dy = goal.y - pose.y, d = Math.hypot(dx, dy);
      pose.yaw = Math.atan2(dy, dx);
      if (d <= 0.025) { pose.x = goal.x; pose.y = goal.y; goal = null; action = ""; say(`Go to done: at (${(pose.x * 100).toFixed(0)}, ${(pose.y * 100).toFixed(0)}) cm\n`); }
      else { pose.x += dx / d * 0.025; pose.y += dy / d * 0.025; }
    }
    socket.write(`\x01pose ${(pose.x * 100).toFixed(1)} ${(pose.y * 100).toFixed(1)} ${(pose.yaw * 180 / Math.PI).toFixed(1)} ${mode} ${action || "-"}\n`);
  }, 250);
});
let command: string | null = null;
socket.on("data", (d: Buffer) => {
  for (const ch of d.toString("latin1")) {
    if (ch === "\0") continue;
    if (ch === "\x02") { command = ""; continue; }
    if (command !== null) {
      if (ch !== "\n") { command += ch; continue; }
      const c = command;
      command = null;
      const g = /^goal (-?\d+) (-?\d+)$/.exec(c), m = /^mode (idle|manual)$/.exec(c);
      if (m) { mode = m[1]; if (mode === "idle") { goal = null; action = ""; } say(`Mode: ${mode}\n`); }
      else if (g && mode !== "manual") say(`Goal (${g[1]}, ${g[2]}) cm ignored: not in manual mode (M)\n`);
      else if (g) { goal = { x: Number(g[1]) / 100, y: Number(g[2]) / 100 }; action = "goto"; say(`Go to: (${g[1]}, ${g[2]}) cm\n`); }
      else say(`Unknown command "${c}"\n`);
      continue;
    }
    if (ch === " ") { goal = null; action = ""; mode = "idle"; say("Stopped, motors off\nMode: idle\n"); }
    else say(`key ${ch}\n`);
  }
});
socket.on("close", () => { console.log("server closed the connection"); process.exit(0); });
socket.on("error", err => { console.error(err.message); process.exit(1); });
