// The robot's side of the server (doc/ARCHITECTURE.md, the PC): announces the server on
// the home network, accepts PicoA's TCP connection and turns its messages (wire.ts)
// into typed events; sends commands back. Only this module knows the connection.
// The robot's first message is hello: a robot speaking another wire version is
// refused (closed, with a log line saying so). A heartbeat goes to the robot every
// 2 s, which it watches; its status messages (4 a second) say it is there. Nothing
// for 6 s is reported as it is (the connection open, nothing arriving: a slow link,
// or the robot gone); only after 30 s is it closed (13 Oct: closing after 6 s on a
// slow link made the robot resend everything on a new connection, and that went
// silent again). One robot at a time.
import dgram from "node:dgram";
import net from "node:net";
import os from "node:os";
import { EventEmitter } from "node:events";
import { SnapshotAssembler } from "./snapshot.ts";
import { HAS, TYPE, WIRE_VERSION, WireReader, decode, encodeCommand, encodeHeartbeat, type Hello, type Message } from "./wire.ts";

export const ANNOUNCE_PORT = 4210;
export const ROBOT_PORT = 4211;

export type RobotStatus =
  | { state: "waiting" }
  | { state: "connected"; ip: string; since: number; rssi: number | null; quietSince: number | null } // quietSince: nothing arrived since
  | { state: "lost"; ip: string; at: number };

export interface RobotLinkOptions {
  robotPort?: number;          // 0: any free port (tests)
  announcePort?: number;
  announceTo?: string[];       // default: every IPv4 network's broadcast address
  announceEveryMs?: number;
  heartbeatEveryMs?: number;  // a heartbeat to the robot this often
  quietMs?: number;           // nothing from the robot this long: reported
  dropMs?: number;            // ... this long: the connection closed
}

// Where the robot is: x, y (m) and yaw (rad, not wrapped: a full left turn adds 2π)
// in the map frame (x forward at power-up, y left).
export interface Pose { x: number; y: number; yaw: number }

// The robot's status, 4 times a second (wire.ts): where it is, its mode and running
// move ("" none), the WiFi signal (null: not known).
export interface LivePose extends Pose { mode: string; action: string; rssi: number | null; at: number }

// How command n ended (an answer message; picoA/app/commands.h): the end word, where
// the robot is (a snapshot's: where it was for it); moved, rotated (a move), goal (a
// go to), snapshot (its number) only where they apply.
export interface DoneRecord {
  n: number; end: string; pose: Pose | null; movedM: number | null; rotatedDeg: number | null; goalM: number | null; snapshot: number | null;
}

export function livePose(m: Extract<Message, { type: "status" }>, at = Date.now()): LivePose {
  return { x: m.x, y: m.y, yaw: m.yaw, mode: m.mode, action: m.action, rssi: m.rssi === 0 ? null : m.rssi, at };
}

export function doneRecord(m: Extract<Message, { type: "answer" }>): DoneRecord {
  const move = (m.has & HAS.MOVE) !== 0;
  return { n: m.n, end: m.end, pose: { x: m.x, y: m.y, yaw: m.yaw }, movedM: move ? m.moved : null,
           rotatedDeg: move ? m.rotated * 180 / Math.PI : null, goalM: m.has & HAS.GOAL ? m.goal : null,
           snapshot: m.has & HAS.SNAPSHOT ? m.snapshot : null };
}

// A message's fields without its type (the events carry wire.ts' MoveStart, MoveEnd).
function withoutType<T extends { type: string }>(m: T): Omit<T, "type"> {
  const out: Record<string, unknown> = { ...m };
  delete out.type;
  return out as Omit<T, "type">;
}

export function broadcastAddresses(): string[] {
  const out = new Set<string>();
  for (const list of Object.values(os.networkInterfaces())) {
    for (const a of list ?? []) {
      if (a.family !== "IPv4" || a.internal) continue;
      const ip = a.address.split(".").map(Number), mask = a.netmask.split(".").map(Number);
      out.add(ip.map((b, i) => (b | (~mask[i] & 255))).join("."));
    }
  }
  return [...out];
}

// Events: "status" (RobotStatus), "note" (string: what the server saw), "bytes"
// (number: a piece of the robot's stream arrived), "hello" (Hello), "text" (string:
// a console line with its "\n"), "pose" (LivePose: a status), "done" (DoneRecord:
// an answer), "mapCleared", "snapshot" (Buffer: one whole snapshot, not yet checked),
// "moveStart" (MoveStart), "moveEnd" (MoveEnd), "tof" (TofMessage: a tof frame),
// "imuBatch" (ImuBatch: up to 10 imu records); frames and imu batches are sent only during
// a move, each under its move's number. Each in the order it came.
export class RobotLink extends EventEmitter {
  status: RobotStatus = { state: "waiting" };
  private socket: net.Socket | null = null;
  private server = net.createServer(s => this.accept(s));
  private udp = dgram.createSocket("udp4");
  private timers: NodeJS.Timeout[] = [];
  private opts: Required<RobotLinkOptions>;
  private refusedVersions = new Set<string>(); // "<ip> <version>": said once, not at every reconnect

  constructor(opts: RobotLinkOptions = {}) {
    super();
    this.opts = {
      robotPort: ROBOT_PORT, announcePort: ANNOUNCE_PORT, announceTo: [],
      announceEveryMs: 1000, heartbeatEveryMs: 2000, quietMs: 6000, dropMs: 30000, ...opts,
    };
  }

  async start(): Promise<number> {
    await new Promise<void>((resolve, reject) => {
      this.server.once("error", reject);
      this.server.listen(this.opts.robotPort, () => resolve());
    });
    const port = (this.server.address() as net.AddressInfo).port;
    await new Promise<void>(resolve => this.udp.bind(() => resolve()));
    this.udp.setBroadcast(true);
    const announce = () => {
      const message = Buffer.from(`ROBOCAR-SERVER ${port}\n`);
      const targets = this.opts.announceTo.length ? this.opts.announceTo : broadcastAddresses();
      for (const to of targets) this.udp.send(message, this.opts.announcePort, to, () => { /* a network without broadcast */ });
    };
    announce();
    this.timers.push(setInterval(announce, this.opts.announceEveryMs));
    return port;
  }

  stop(): void {
    for (const t of this.timers) clearInterval(t);
    this.socket?.destroy();
    this.server.close();
    this.udp.close();
  }

  // Command n ("drive 50", printable, ≤ 46 characters); false if the robot isn't connected.
  sendCommand(n: number, text: string): boolean {
    const message = encodeCommand({ n, text }); // throws if it isn't a command
    if (!this.socket || this.status.state !== "connected") return false;
    this.socket.write(message);
    return true;
  }

  private setStatus(status: RobotStatus): void {
    this.status = status;
    this.emit("status", status);
  }

  private accept(socket: net.Socket): void {
    const ip = (socket.remoteAddress ?? "?").replace(/^::ffff:/, "");
    if (this.socket) {
      this.emit("note", "a new robot connection replaces the old one");
      this.socket.destroy();
    }
    this.socket = socket;
    socket.setNoDelay(true);
    const reader = new WireReader(), snapshots = new SnapshotAssembler();
    let lastHeard = Date.now(), quiet = false, hello: Hello | null = null;
    const current = () => this.socket === socket && this.status.state === "connected";
    const seconds = (ms: number) => (ms / 1000).toFixed(0);
    const refuse = (why: string, once?: string) => {
      if (once === undefined || !this.refusedVersions.has(once)) this.emit("note", `robot connection from ${ip} refused: ${why}`);
      if (once !== undefined) this.refusedVersions.add(once);
      socket.destroy();
    };
    const heartbeat = setInterval(() => {
      const nothingMs = Date.now() - lastHeard;
      if (nothingMs > this.opts.dropMs) {
        this.emit("note", `nothing from the robot for ${seconds(nothingMs)} s: closing the connection (the robot connects again by itself)`);
        socket.destroy();
        return;
      }
      if (!quiet && nothingMs > this.opts.quietMs) {
        quiet = true;
        this.emit("note", `nothing from the robot for ${seconds(nothingMs)} s; the connection is open (a slow link, or the robot is gone)`);
        if (current()) this.setStatus({ ...this.status, quietSince: lastHeard } as RobotStatus);
      }
      socket.write(encodeHeartbeat());
    }, Math.min(this.opts.heartbeatEveryMs, this.opts.quietMs / 3));

    // One message from the robot; the first must be a hello of our version.
    const handle = (m: Message): void => {
      if (!hello) {
        if (m.type !== "hello") { refuse(`its first message is ${m.type}, not hello`); return; }
        if (m.version !== WIRE_VERSION) {
          refuse(`it speaks wire version ${m.version}, this server ${WIRE_VERSION} (flash PicoA with this repo's firmware, or update the server; ` +
                 "said once: it reconnects every few seconds)", `${ip} ${m.version}`);
          return;
        }
        hello = m;
        this.emit("note", `robot connected from ${ip} (boot ${m.bootId}, firmware of ${m.build})`);
        this.setStatus({ state: "connected", ip, since: Date.now(), rssi: null, quietSince: null });
        this.emit("hello", { version: m.version, bootId: m.bootId, build: m.build });
        return;
      }
      if (this.socket !== socket) return; // replaced
      switch (m.type) {
        case "console": this.emit("text", `${m.text}\n`); break;
        case "status": {
          const pose = livePose(m);
          this.emit("pose", pose);
          if (pose.rssi !== null && current() && this.status.state === "connected" && this.status.rssi !== pose.rssi) {
            this.setStatus({ ...this.status, rssi: pose.rssi });
          }
          break;
        }
        case "answer": this.emit("done", doneRecord(m)); break;
        case "map_cleared": this.emit("mapCleared"); break;
        case "move_start": this.emit("moveStart", withoutType(m)); break;
        case "move_end": this.emit("moveEnd", withoutType(m)); break;
        case "tof": this.emit("tof", withoutType(m)); break;
        case "imu_batch": this.emit("imuBatch", withoutType(m)); break;
        case "snapshot_part":
          for (const snap of snapshots.push(m.bytes)) this.emit("snapshot", snap);
          if (snapshots.skipped) { this.emit("note", `snapshot parts: ${snapshots.skipped} bytes skipped (not a snapshot)`); snapshots.skipped = 0; }
          break;
        case "hello": refuse("a second hello"); break;
        default: break;
      }
    };

    socket.on("data", (chunk: Buffer) => {
      if (quiet) {
        quiet = false;
        this.emit("note", `data from the robot again, after ${seconds(Date.now() - lastHeard)} s`);
        if (current()) this.setStatus({ ...this.status, quietSince: null } as RobotStatus);
      }
      lastHeard = Date.now();
      this.emit("bytes", chunk.length);
      for (const raw of reader.push(chunk)) {
        if (raw.type === TYPE.COMMAND || raw.type === TYPE.HEARTBEAT) { refuse(`it sent a message for the robot (0x${raw.type.toString(16)})`); return; }
        let m: Message;
        try { m = decode(raw); } catch (err) { refuse((err as Error).message); return; }
        handle(m);
        if (socket.destroyed) return;
      }
      if (reader.error) refuse(`${reader.error} (not a robot speaking the wire, or an old firmware)`);
    });
    socket.on("error", () => { /* "close" follows */ });
    socket.on("close", () => {
      clearInterval(heartbeat);
      if (this.socket !== socket) return; // replaced
      this.socket = null;
      if (this.status.state === "connected") {
        this.emit("note", "robot connection closed");
        this.setStatus({ state: "lost", ip, at: Date.now() });
      }
    });
  }
}
