// The robot's side of the server (doc/ARCHITECTURE.md, the PC): announces the server on the
// home network, accepts PicoA's TCP connection, passes its console text on and
// keys and commands back, and takes the snapshots out of the stream. It writes '\0'
// every 2 s, which the robot watches; the robot's pose records (4 a second) are what
// says it is there. Nothing for 6 s is reported as it is (the connection open,
// nothing arriving: a slow link, or the robot gone); only after 30 s is it closed
// (13 Oct: closing after 6 s on a slow link made the robot resend everything on a
// new connection, and that went silent again). One robot at a time.
import dgram from "node:dgram";
import net from "node:net";
import os from "node:os";
import { EventEmitter } from "node:events";
import { StringDecoder } from "node:string_decoder";
import { SnapshotAssembler } from "./snapshot.ts";

export const ANNOUNCE_PORT = 4210;
export const ROBOT_PORT = 4211;

export type RobotStatus =
  | { state: "waiting" }
  | { state: "connected"; ip: string; since: number; rssi: number | null; quietSince: number | null } // quietSince: nothing arrived since
  | { state: "lost"; ip: string; at: number };

export interface RobotLinkOptions {
  robotPort?: number;          // 0: any free port (tests)
  dataPort?: number;           // the recording connection's, announced too (recording.ts); 0: none
  announcePort?: number;
  announceTo?: string[];       // default: every IPv4 network's broadcast address
  announceEveryMs?: number;
  heartbeatEveryMs?: number;  // '\0' to the robot this often
  quietMs?: number;           // nothing from the robot this long: reported
  dropMs?: number;            // ... this long: the connection closed
}

export const SNAP_CHUNK = 0x02; // [0x02][length u16][bytes] (picoA/app/wifi_console.c)
export const COMMAND = "\x02";   // to the robot: "\x02<command>\n" (picoA/app/debug_console.c)

// Splits the robot's stream into console text, status records ("\x01pose ...\n";
// "\x01rssi -55\n" from older firmware), which can land anywhere, also in the middle of a line, and
// snapshot chunks ([0x02][length u16][bytes]), which come between any two bytes of
// the rest. The chunks' bytes, in order, are the snapshots (snapshot.ts).
export class RecordSplitter {
  private decoder = new StringDecoder("utf8");
  private record: string | null = null;
  private chunkHead: number[] | null = null; // a chunk's header bytes so far (after 0x02)
  private chunkLeft = 0;                     // bytes of the chunk still to come

  push(chunk: Buffer): { text: string; records: string[]; snapshotBytes: Buffer[] } {
    const plain: Buffer[] = [], snapshotBytes: Buffer[] = [];
    let from = 0;
    for (let i = 0; i < chunk.length;) {
      if (this.chunkLeft > 0) {
        const n = Math.min(this.chunkLeft, chunk.length - i);
        snapshotBytes.push(chunk.subarray(i, i + n));
        this.chunkLeft -= n;
        i += n;
        from = i;
      } else if (this.chunkHead) {
        this.chunkHead.push(chunk[i++]);
        if (this.chunkHead.length === 2) { this.chunkLeft = this.chunkHead[0] | this.chunkHead[1] << 8; this.chunkHead = null; }
        from = i;
      } else if (chunk[i] === SNAP_CHUNK) {
        plain.push(chunk.subarray(from, i));
        this.chunkHead = [];
        from = ++i;
      } else i++;
    }
    if (!this.chunkHead && this.chunkLeft === 0) plain.push(chunk.subarray(from));
    let text = "";
    const records: string[] = [];
    for (const c of plain.map(b => this.decoder.write(b)).join("")) {
      if (this.record !== null) {
        if (c === "\n") { records.push(this.record); this.record = null; }
        else this.record += c;
      } else if (c === "\x01") this.record = "";
      else if (c !== "\r") text += c;
    }
    return { text, records, snapshotBytes };
  }
}

// Where the robot is now ("\x01pose <x cm> <y cm> <yaw deg> <mode> <action> <signal dBm>"),
// 4 times a second; older firmware sends no signal.
export interface LivePose { x: number; y: number; yaw: number; mode: string; action: string; rssi: number | null; at: number }
export function parsePose(record: string): LivePose | null {
  const m = /^pose (-?[\d.]+) (-?[\d.]+) (-?[\d.]+) (\S+) (\S+)(?: (-?\d+))?$/.exec(record);
  if (!m) return null;
  return { x: Number(m[1]) / 100, y: Number(m[2]) / 100, yaw: Number(m[3]) * Math.PI / 180, mode: m[4],
           action: m[5] === "-" ? "" : m[5], rssi: m[6] === undefined || m[6] === "0" ? null : Number(m[6]), at: Date.now() };
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

// Events: "text" (string), "status" (RobotStatus), "note" (string: what the server saw),
// "bytes" (number: a piece of the robot's stream arrived), "snapshot" (Buffer: one
// whole snapshot, not yet checked), "pose" (LivePose).
export class RobotLink extends EventEmitter {
  status: RobotStatus = { state: "waiting" };
  private socket: net.Socket | null = null;
  private server = net.createServer(s => this.accept(s));
  private udp = dgram.createSocket("udp4");
  private timers: NodeJS.Timeout[] = [];
  private opts: Required<RobotLinkOptions>;

  constructor(opts: RobotLinkOptions = {}) {
    super();
    this.opts = {
      robotPort: ROBOT_PORT, dataPort: 0, announcePort: ANNOUNCE_PORT, announceTo: [],
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
      const message = Buffer.from(`ROBOCAR-SERVER ${port}${this.opts.dataPort ? ` ${this.opts.dataPort}` : ""}\n`);
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

  sendKey(key: string): boolean {
    if (!this.socket || this.status.state !== "connected") return false;
    this.socket.write(key);
    return true;
  }

  // One of the robot's commands ("goal 120 -35", "mode manual").
  sendCommand(command: string): boolean {
    if (!/^[\x20-\x7e]{1,40}$/.test(command)) return false;
    return this.sendKey(`${COMMAND}${command}\n`);
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
    const splitter = new RecordSplitter(), snapshots = new SnapshotAssembler();
    let lastHeard = Date.now(), quiet = false;
    const current = () => this.socket === socket && this.status.state === "connected";
    const seconds = (ms: number) => (ms / 1000).toFixed(0);
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
      socket.write("\0");
    }, Math.min(this.opts.heartbeatEveryMs, this.opts.quietMs / 3));
    this.emit("note", `robot connected from ${ip}`);
    this.setStatus({ state: "connected", ip, since: Date.now(), rssi: null, quietSince: null });

    socket.on("data", (chunk: Buffer) => {
      if (quiet) {
        quiet = false;
        this.emit("note", `data from the robot again, after ${seconds(Date.now() - lastHeard)} s`);
        if (current()) this.setStatus({ ...this.status, quietSince: null } as RobotStatus);
      }
      lastHeard = Date.now();
      this.emit("bytes", chunk.length);
      const { text, records, snapshotBytes } = splitter.push(chunk);
      for (const r of records) {
        const m = /^rssi (-?\d+)$/.exec(r);
        if (m && this.socket === socket && this.status.state === "connected") {
          this.setStatus({ ...this.status, rssi: Number(m[1]) });
        }
        const pose = parsePose(r);
        if (pose && this.socket === socket) this.emit("pose", pose);
        if (pose && pose.rssi !== null && current() && this.status.state === "connected" && this.status.rssi !== pose.rssi) {
          this.setStatus({ ...this.status, rssi: pose.rssi });
        }
      }
      for (const b of snapshotBytes) {
        for (const snap of snapshots.push(b)) this.emit("snapshot", snap);
        if (snapshots.skipped) { this.emit("note", `snapshot stream: ${snapshots.skipped} bytes skipped (not a snapshot)`); snapshots.skipped = 0; }
      }
      if (text) this.emit("text", text);
    });
    socket.on("error", () => { /* "close" follows */ });
    socket.on("close", () => {
      clearInterval(heartbeat);
      if (this.socket !== socket) return; // replaced
      this.socket = null;
      this.emit("note", "robot connection closed");
      this.setStatus({ state: "lost", ip, at: Date.now() });
    });
  }
}
