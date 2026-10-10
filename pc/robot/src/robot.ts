// The robot's side of the server (ROBOT_WIFI.md): announces the server on the
// home network, accepts PicoA's TCP connection, passes its console text on and
// keys back, and keeps the connection alive with heartbeats. One robot at a time.
import dgram from "node:dgram";
import net from "node:net";
import os from "node:os";
import { EventEmitter } from "node:events";
import { StringDecoder } from "node:string_decoder";

export const ANNOUNCE_PORT = 4210;
export const ROBOT_PORT = 4211;

export type RobotStatus =
  | { state: "waiting" }
  | { state: "connected"; ip: string; since: number; rssi: number | null }
  | { state: "lost"; ip: string; at: number };

export interface RobotLinkOptions {
  robotPort?: number;          // 0: any free port (tests)
  dataPort?: number;           // the recording connection's, announced too (recording.ts); 0: none
  announcePort?: number;
  announceTo?: string[];       // default: every IPv4 network's broadcast address
  announceEveryMs?: number;
  heartbeatEveryMs?: number;
  silenceMs?: number;
}

// Splits the robot's stream into console text and status records
// ("\x01rssi -55\n"), which can land anywhere, also in the middle of a line.
export class RecordSplitter {
  private decoder = new StringDecoder("utf8");
  private record: string | null = null;

  push(chunk: Buffer): { text: string; records: string[] } {
    let text = "";
    const records: string[] = [];
    for (const c of this.decoder.write(chunk)) {
      if (this.record !== null) {
        if (c === "\n") { records.push(this.record); this.record = null; }
        else this.record += c;
      } else if (c === "\x01") this.record = "";
      else if (c !== "\r") text += c;
    }
    return { text, records };
  }
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
// "bytes" (number: a piece of the robot's stream arrived).
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
      announceEveryMs: 1000, heartbeatEveryMs: 2000, silenceMs: 6000, ...opts,
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
    const splitter = new RecordSplitter();
    let lastHeard = Date.now();
    const heartbeat = setInterval(() => {
      if (Date.now() - lastHeard > this.opts.silenceMs) {
        this.emit("note", `robot silent for ${this.opts.silenceMs / 1000} s, dropping it`);
        socket.destroy();
      } else socket.write("\0");
    }, Math.min(this.opts.heartbeatEveryMs, this.opts.silenceMs / 3));
    this.emit("note", `robot connected from ${ip}`);
    this.setStatus({ state: "connected", ip, since: Date.now(), rssi: null });

    socket.on("data", (chunk: Buffer) => {
      lastHeard = Date.now();
      this.emit("bytes", chunk.length);
      const { text, records } = splitter.push(chunk);
      for (const r of records) {
        const m = /^rssi (-?\d+)$/.exec(r);
        if (m && this.socket === socket && this.status.state === "connected") {
          this.setStatus({ ...this.status, rssi: Number(m[1]) });
        }
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
