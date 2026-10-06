// PicoA USB -> camera binary frames + ToF / OPT4048 JSON + status logs on one WebSocket.
import { SerialPort } from "serialport";
import { WebSocketServer, WebSocket } from "ws";
import http from "node:http";
import { readFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { FrameParser } from "./protocol.ts";

const publicDir = path.join(path.dirname(fileURLToPath(import.meta.url)), "..", "public");
const HTTP_PORT = Number(process.env.PORT ?? 8080);
async function findPicoPath(): Promise<string> {
  if (process.argv[2]) return process.argv[2];
  const ports = await SerialPort.list();
  const candidates = ports.filter(p => (p.vendorId ?? "").toLowerCase() === "2e8a");
  if (candidates.length > 1) throw new Error("Multiple Picos found. Use npm start -- /dev/tty.usbmodemXXXX (PicoA port).");
  if (!candidates.length) throw new Error("No Pico found. Connect PicoA or specify its serial port.");
  return candidates[0].path;
}
async function main(): Promise<void> {
  const portPath = await findPicoPath();
  console.log(`Opening ${portPath}`);
  const port = new SerialPort({path: portPath, baudRate: 115200});
  const clients = new Set<WebSocket>();
  const logs: string[] = [];
  let cameraSettings: string | null = null;
  function broadcast(data: Buffer | string) {
    for (const ws of clients) {
      // A slow tab must not build an unbounded queue of stale frames.
      if (ws.readyState === WebSocket.OPEN && ws.bufferedAmount < 1024 * 1024) ws.send(data);
    }
  }
  function log(text: string) {
    if (text.startsWith('CAMERA_STATE ')) {
      try {
        const state = JSON.parse(text.slice('CAMERA_STATE '.length));
        if (state.type === 'cameraSettings') {
          cameraSettings = JSON.stringify(state); broadcast(cameraSettings); return;
        }
      } catch { /* malformed diagnostic falls through to the log */ }
    }
    console.log("[pico]", text);
    logs.push(text); if (logs.length > 40) logs.shift();
    broadcast(JSON.stringify({type: "log", text}));
  }
  const server = http.createServer(async (req, res) => {
    const routes: Record<string, [string, string]> = {
      "/": ["index.html", "text/html"], "/index.html": ["index.html", "text/html"],
      "/viewer.js": ["viewer.js", "text/javascript"],
      "/tof-view.js": ["tof-view.js", "text/javascript"],
    };
    const route = routes[req.url ?? "/"];
    if (!route) { res.writeHead(404); res.end("not found"); return; }
    try {
      const data = await readFile(path.join(publicDir, route[0]));
      res.writeHead(200, {"content-type": route[1], "cache-control": "no-store"}); res.end(data);
    } catch { res.writeHead(500); res.end("Cannot read viewer file"); }
  });
  const wss = new WebSocketServer({server, maxPayload: 256});
  wss.on("connection", ws => {
    clients.add(ws);
    ws.send(JSON.stringify({type: "connection", connected: port.isOpen, port: portPath}));
    for (const text of logs) ws.send(JSON.stringify({type: "log", text}));
    if (cameraSettings) ws.send(cameraSettings);
    ws.on("close", () => clients.delete(ws));
    ws.on("error", () => clients.delete(ws));
    ws.on("message", data => {
      const bytes = Buffer.from(data as Buffer);
      const text = bytes.toString("ascii");
      const valid = (bytes.length === 1 && "SXC".includes(text)) ||
        (bytes.length === 2 && bytes[0] === 0x50 && bytes[1] <= 5) ||
        /^T (init|start|stop|\d+ \d+ \d+ \d+ \d+ \d+)\n$/.test(text) ||
        /^H (status|hold|release)\n$/.test(text) ||
        text === 'O init\n' ||
        /^[wr] [0-9a-fA-F]{1,4}( [0-9a-fA-F]{1,2})?\n$/.test(text);
      if (!valid || !port.isOpen) {
        ws.send(JSON.stringify({type: "log", text: "Command rejected: invalid command or serial port closed"}));
        return;
      }
      port.write(bytes, err => { if (err) log(`Serial write error: ${err.message}`); });
    });
  });
  const parser = new FrameParser((w, h, pixels) => {
    const head = Buffer.alloc(4); head.writeUInt16LE(w); head.writeUInt16LE(h, 2);
    broadcast(Buffer.concat([head, pixels]));
  }, log, frame => broadcast(JSON.stringify(frame)), frame => broadcast(JSON.stringify(frame)));
  port.on("data", (chunk: Buffer) => parser.push(chunk));
  port.on("error", err => log(`Serial error: ${err.message}`));
  port.on("close", () => {
    broadcast(JSON.stringify({type: "connection", connected: false, port: portPath}));
    log("Serial disconnected; reconnect PicoA and restart this app.");
  });
  port.on("open", () => {
    broadcast(JSON.stringify({type: "connection", connected: true, port: portPath}));
    console.log("Port open; sensors initializing...");
    // Queued while the Pico initializes. Camera Stop is never auto-overridden.
    setTimeout(() => { if (port.isOpen) port.write("S"); }, 300);
  });
  server.on("error", err => {
    console.error(`Web server: ${err.message}. Stop the other server or set PORT.`);
    process.exit(1);
  });
  server.listen(HTTP_PORT, "127.0.0.1", () => console.log(`Viewer at http://127.0.0.1:${HTTP_PORT}/`));
}
main().catch(err => { console.error(err.message ?? err); process.exit(1); });
