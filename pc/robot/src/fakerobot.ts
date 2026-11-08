// A fake robot for trying the server, the page and the `robot` CLI without the robot:
// connects to the server as PicoA does (ROBOT_PORT on 127.0.0.1) and speaks the wire
// (wire.ts; picoA/app/commands.h, move.h, snapshot.h): hello first, then
//  - replays a stored boot's messages from the database (store.ts replay: moves, tof
//    frames, imu batches, statuses, snapshots, in the order they came, at their pace or
//    faster), so the page shows that drive as it was;
//  - takes command messages and answers each when it has ended, as a move with its
//    start and end (no frames, no imu batches), sends its status 4 times a second and
//    its console lines, and for `snapshot` the boot's next stored
//    snapshot, one after another, in parts as the robot does. It drives its pose at
//    the robot's speeds (10 cm/s, 29°/s); it knows no map, so it never meets an obstacle.
//   npm run fakerobot -- <boot id> [--speed=<n>] [--no-replay]   (the database: ROBOT_DB, or recordings/robot2.duckdb)
// Point it at a test server with a database of its own (README, "The page without the robot").
import net from "node:net";
import { fileURLToPath } from "node:url";
import { parseSnapshot } from "./snapshot.ts";
import { ROBOT_PORT } from "./link.ts";
import { DB_PATH, Store, type Replayed } from "./store.ts";
import {
  HAS, MOVE_KINDS, PART_MAX, TYPE, WIRE_VERSION, WireReader, decode, encodeAnswer, encodeConsole, encodeHello, encodeMapCleared,
  encodeImuBatch, encodeMoveEnd, encodeMoveStart, encodeSnapshotPart, encodeStatus, encodeTof,
} from "./wire.ts";

const STEP_S = 0.05, SPEED_MPS = 0.1, TURN_RADPS = 0.5, GOTO_MAX_M = 3, STILL_MS = 200;
const KIND: Record<string, (typeof MOVE_KINDS)[number]> = { move: "drive", turn: "turn", goto: "goto", watch: "watch" };

export interface FakeSnapshot { bytes: Buffer; seq: number; pose: { x: number; y: number; yaw: number } | null }

// The snapshots among a boot's stored messages, as the robot sends them.
export function snapshotsOf(messages: Replayed[]): FakeSnapshot[] {
  return messages.flatMap(m => {
    if (m.type !== "snapshot") return [];
    const s = parseSnapshot(m.bytes);
    return [{ bytes: m.bytes, seq: s.seq, pose: s.pose }];
  });
}

export interface FakeRobotOptions {
  port?: number;
  snapshots: FakeSnapshot[];
  replay?: Replayed[];      // a stored boot's messages, sent after hello
  speed?: number;           // the replay this many times faster than it came (Infinity: at once)
  bootId?: string;          // 8 hex digits
  version?: number;         // the wire version it says it speaks (tests: another one)
  echo?: boolean;           // print its console lines
}

export class FakeRobot {
  private socket: net.Socket;
  private opts: Required<FakeRobotOptions>;
  private pose = { x: 0, y: 0, yaw: 0 };
  private mode = "manual";
  private next = 0;
  private moves = 0;       // numbered so far (the replay's too)
  private replaying = false;
  private timers: NodeJS.Timeout[] = [];
  // The running action: what it still has to do, step by step; its command's and move's numbers.
  private action: { name: string; kind: string; n: number; move: number; steps: (() => boolean)[]; start: { x: number; y: number; yaw: number }; goal?: number } | null = null;
  readonly closed: Promise<void>;
  readonly replayed: Promise<void>; // the replay has all been sent
  private replayDone: () => void = () => {};

  constructor(opts: FakeRobotOptions) {
    this.opts = { port: ROBOT_PORT, bootId: "fa4e0001", version: WIRE_VERSION, echo: false, replay: [], speed: 1, ...opts };
    this.replayed = new Promise(resolve => { this.replayDone = resolve; });
    const first = this.opts.snapshots[0]?.pose;
    if (first) this.pose = { ...first };
    this.socket = net.connect(this.opts.port, "127.0.0.1");
    this.socket.setNoDelay(true);
    this.closed = new Promise(resolve => this.socket.on("close", () => { this.stop(); resolve(); }));
    this.socket.on("error", () => { /* "close" follows */ });
    this.socket.on("connect", () => this.connected());
    const reader = new WireReader();
    this.socket.on("data", (d: Buffer) => {
      for (const raw of reader.push(d)) {
        if (raw.type === TYPE.HEARTBEAT) continue;
        const m = decode(raw);
        if (m.type === "command") this.command(m.n, m.text);
      }
      if (reader.error) { this.say(`Server: sent something that isn't a message (${reader.error}): closing\n`); this.socket.destroy(); }
    });
  }

  stop(): void {
    for (const t of this.timers) clearInterval(t);
    this.timers = [];
    this.socket.destroy();
  }

  private connected(): void {
    this.socket.write(encodeHello({ version: this.opts.version, bootId: this.opts.bootId, build: "fake robot" }));
    this.say("WiFi: connected (fake robot)\nServer: connected\n");
    if (this.opts.replay.length) void this.replay();
    else { this.sendSnapshot(null); this.replayDone(); } // a server connected
    this.timers.push(setInterval(() => {
      if (!this.action) return;
      while (this.action.steps.length && this.action.steps[0]()) this.action.steps.shift();
      if (!this.action.steps.length) this.end("done");
    }, STEP_S * 1000));
    const status = () => {
      if (this.replaying) return; // the replay sends the stored ones
      this.socket.write(encodeStatus({ ...this.pose, mode: this.mode, action: this.action?.kind ?? "", rssi: -55 }));
    };
    status();
    this.timers.push(setInterval(status, 250));
  }

  // The stored messages in their order, each after the gap it came with (divided by speed).
  private async replay(): Promise<void> {
    this.replaying = true;
    const list = this.opts.replay, speed = this.opts.speed;
    this.say(`Replaying boot ${this.opts.bootId}: ${list.length} messages\n`);
    for (let i = 0; i < list.length && !this.socket.destroyed; i++) {
      const gap = i ? (list[i].atMs - list[i - 1].atMs) / speed : 0;
      if (gap > 0 && Number.isFinite(gap)) await new Promise(r => setTimeout(r, Math.min(gap, 10_000)));
      const m = list[i];
      if (m.type === "move_start") { this.moves = Math.max(this.moves, m.start.move); this.socket.write(encodeMoveStart(m.start)); }
      else if (m.type === "move_end") this.socket.write(encodeMoveEnd(m.end));
      else if (m.type === "frame") this.socket.write(encodeTof(m.format, m.frame));
      else if (m.type === "imu") this.socket.write(encodeImuBatch(m.batch));
      else if (m.type === "status") {
        const p = m.pose;
        this.pose = { x: p.x, y: p.y, yaw: p.yaw };
        this.socket.write(encodeStatus({ x: p.x, y: p.y, yaw: p.yaw, mode: p.mode, action: p.action, rssi: p.rssi ?? 0 }));
      } else this.sendParts(m.bytes);
    }
    this.replaying = false;
    this.say(`Replay of boot ${this.opts.bootId} done; taking commands\n`);
    this.replayDone();
  }

  private say(text: string): void {
    for (const line of text.split("\n").slice(0, -1)) this.socket.write(encodeConsole(line));
    if (this.opts.echo) process.stdout.write(`robot: ${text}`);
  }

  private answer(n: number, end: string, more: { has?: number; moved?: number; rotated?: number; goal?: number; snapshot?: number } = {},
                 at = this.pose): void {
    this.socket.write(encodeAnswer({ n, end, has: 0, x: at.x, y: at.y, yaw: at.yaw, moved: 0, rotated: 0, goal: 0, snapshot: 0, ...more }));
  }

  private end(how: string): void {
    const a = this.action;
    if (!a) return;
    this.action = null;
    const end = how === "done" ? "ok" : how, moved = Math.hypot(this.pose.x - a.start.x, this.pose.y - a.start.y), rotated = this.pose.yaw - a.start.yaw;
    this.socket.write(encodeMoveEnd({ move: a.move, end, tUs: this.timeUs(), framesSent: 0, framesDropped: 0, firstFrame: 0, lastFrame: 0, moved, rotated,
                                           imuSent: 0, imuDropped: 0 }));
    this.say(`${a.name} ${how}\n`);
    this.answer(a.n, end, { has: HAS.MOVE | (a.goal !== undefined ? HAS.GOAL : 0), moved, rotated, goal: a.goal ?? 0 });
  }

  // PicoA's clock: started 30 s before the wrap (common/clock_start.h).
  private readonly bornMs = Date.now();
  private timeUs(): number { return ((Date.now() - this.bornMs) * 1000 + 2 ** 32 - 30e6) % 2 ** 32; }

  private turn(rad: number) {
    let left = Math.abs(rad);
    return () => { const d = Math.min(left, TURN_RADPS * STEP_S); this.pose.yaw += Math.sign(rad) * d; left -= d; return left <= 0; };
  }

  private drive(m: number) {
    let left = Math.abs(m);
    return () => {
      const d = Math.min(left, SPEED_MPS * STEP_S), s = Math.sign(m);
      this.pose.x += s * d * Math.cos(this.pose.yaw); this.pose.y += s * d * Math.sin(this.pose.yaw); left -= d;
      return left <= 0;
    };
  }

  private wait(s: number) { let left = s; return () => (left -= STEP_S) <= 0; }

  // name as the robot prints it; a move: still STILL_MS before and after, as the robot's.
  private begin(name: string, take: string, n: number, param: number, steps: (() => boolean)[], goal?: number): void {
    this.end("replaced");
    if (this.mode === "idle") { this.mode = "manual"; this.say("Mode: manual\n"); }
    const kind = KIND[take], move = ++this.moves;
    this.socket.write(encodeMoveStart({ move, command: n, kind, param, tUs: this.timeUs(), preMs: STILL_MS, postMs: STILL_MS, refused: false }));
    this.action = { name, kind, n, move, steps: [this.wait(STILL_MS / 1000), ...steps, this.wait(STILL_MS / 1000)], start: { ...this.pose }, goal };
    this.say(`${name}: started\n`);
  }

  private sendParts(bytes: Buffer): void {
    for (let at = 0; at < bytes.length; at += PART_MAX) this.socket.write(encodeSnapshotPart(bytes.subarray(at, at + PART_MAX)));
  }

  private sendSnapshot(n: number | null): void {
    const snaps = this.opts.snapshots;
    if (!snaps.length) { if (n !== null) this.answer(n, "ok", { has: HAS.SNAPSHOT }); return; }
    const s = snaps[this.next++ % snaps.length];
    this.sendParts(s.bytes);
    if (n !== null) this.answer(n, "ok", { has: HAS.SNAPSHOT, snapshot: s.seq }, s.pose ?? this.pose);
  }

  private command(n: number, text: string): void {
    const num = (re: RegExp) => re.exec(text)?.slice(1).map(Number) ?? null;
    let v: number[] | null;
    if ((v = num(/^drive (-?\d+)$/))) this.begin("Move", "move", n, v[0] / 100, [this.drive(v[0] / 100)]);
    else if ((v = num(/^rotate (-?\d+)$/))) this.begin("Turn", "turn", n, v[0] * Math.PI / 180, [this.turn(v[0] * Math.PI / 180)]);
    else if ((v = num(/^watch (\d+)$/))) this.begin("Watch", "watch", n, v[0], [this.wait(v[0])]);
    else if ((v = num(/^goto (-?\d+) (-?\d+)$/))) {
      const dx = v[0] / 100 - this.pose.x, dy = v[1] / 100 - this.pose.y, d = Math.hypot(dx, dy), bearing = Math.atan2(dy, dx);
      const towards = Math.atan2(Math.sin(bearing - this.pose.yaw), Math.cos(bearing - this.pose.yaw));
      this.begin("Go to", "goto", n, d, [this.turn(towards), this.drive(Math.min(d, GOTO_MAX_M))], d);
    } else if (text === "snapshot") this.sendSnapshot(n);
    else if (text === "clear_map") { this.say("Map cleared\n"); this.socket.write(encodeMapCleared()); this.answer(n, "ok"); }
    else if (/^config (pre|post)_ms \d+$/.test(text)) { this.say(`Config: ${text.slice(7)}\n`); this.answer(n, "ok"); }
    else if (/^mode (idle|manual)$/.test(text)) {
      this.mode = text.slice(5);
      if (this.mode === "idle") this.end("stopped");
      this.say(`Mode: ${this.mode}\n`);
      this.answer(n, "ok");
    } else if (text === "stop") { this.end("stopped"); this.mode = "idle"; this.say("Stopped, motors off\nMode: idle\n"); this.answer(n, "ok"); }
    else if (text === "status") { this.say(`fake robot at (${(this.pose.x * 100).toFixed(1)}, ${(this.pose.y * 100).toFixed(1)}) cm\n`); this.answer(n, "ok"); }
    else { this.say(`Command refused: "${text}"\n`); this.answer(n, "refused"); }
  }
}

if (process.argv[1] === fileURLToPath(import.meta.url)) {
  const args = process.argv.slice(2), boot = args.find(a => !a.startsWith("--"));
  const speedArg = args.find(a => a.startsWith("--speed="))?.slice(8);
  if (!boot || !/^[0-9a-f]{8}$/.test(boot) || (speedArg !== undefined && !(Number(speedArg) > 0))) {
    console.error("usage: npm run fakerobot -- <boot id> [--speed=<n>] [--no-replay]   (the database: ROBOT_DB, default recordings/robot2.duckdb)");
    process.exit(2);
  }
  const messages = await new Store(DB_PATH).replay(boot);
  if (!messages.length) { console.error(`no messages of boot ${boot} in ${DB_PATH}`); process.exit(1); }
  const robot = new FakeRobot({ port: Number(process.env.ROBOT_PORT ?? ROBOT_PORT), bootId: boot, snapshots: snapshotsOf(messages),
                                replay: args.includes("--no-replay") ? [] : messages, speed: Number(speedArg ?? 1), echo: true });
  void robot.closed.then(() => { console.log("server closed the connection"); process.exit(0); });
}
