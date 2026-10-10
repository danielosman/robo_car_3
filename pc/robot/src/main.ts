// RoboCar robot server (ROBOT_WIFI.md): PicoA connects over WiFi, the browser at
// http://127.0.0.1:8080/ shows its console and sends keys. Every run writes a log
// file in logs/. PicoA's recordings arrive on a second connection and are saved in
// recordings/ (recording.ts, doc/TELEMETRY_PLAN.md).
import http from "node:http";
import { createWriteStream } from "node:fs";
import { mkdir, readFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { WebSocketServer, WebSocket } from "ws";
import { RobotLink, broadcastAddresses } from "./robot.ts";
import { RecordingServer } from "./recording.ts";

const root = path.join(path.dirname(fileURLToPath(import.meta.url)), "..");
const HTTP_PORT = Number(process.env.PORT ?? 8080);
const BACKLOG_CHARS = 256 * 1024; // what a newly opened page gets

type Item = { kind: "text" | "note"; text: string };

async function main(): Promise<void> {
  await mkdir(path.join(root, "logs"), { recursive: true });
  const stamp = new Date().toLocaleString("sv-SE").replace(/[ :]/g, "-"); // local time, sortable
  const logPath = path.join(root, "logs", `${stamp}.log`);
  const logFile = createWriteStream(logPath, { flags: "a" });

  const clients = new Set<WebSocket>();
  const backlog: Item[] = [];
  let backlogChars = 0;
  const send = (ws: WebSocket, msg: unknown) => { if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg)); };
  const broadcast = (msg: unknown) => { for (const ws of clients) send(ws, msg); };
  function add(item: Item): void {
    backlog.push(item);
    backlogChars += item.text.length;
    while (backlogChars > BACKLOG_CHARS && backlog.length > 1) backlogChars -= backlog.shift()!.text.length;
    broadcast({ type: item.kind, text: item.text });
  }
  function note(text: string): void {
    const line = `--- ${new Date().toLocaleTimeString()} ${text} ---\n`;
    logFile.write(line);
    console.log(line.trimEnd());
    add({ kind: "note", text: line });
  }

  const recordings = new RecordingServer({ dir: path.join(root, "recordings") });
  recordings.on("note", note);
  const dataPort = await recordings.start();
  const robot = new RobotLink({ dataPort });
  robot.on("text", (text: string) => { logFile.write(text); process.stdout.write(text); add({ kind: "text", text }); });
  robot.on("note", note);
  robot.on("bytes", (n: number) => recordings.consoleArrived(n));
  robot.on("status", status => broadcast({ type: "status", status }));
  await robot.start();
  note(`server started; announcing on ${broadcastAddresses().join(", ") || "no network"}; log ${path.relative(root, logPath)}`);

  const files: Record<string, [string, string]> = {
    "/": ["index.html", "text/html"],
    "/console.js": ["console.js", "text/javascript"],
  };
  const server = http.createServer(async (req, res) => {
    const file = files[req.url ?? "/"];
    if (!file) { res.writeHead(404); res.end("not found"); return; }
    try {
      const data = await readFile(path.join(root, "public", file[0]));
      res.writeHead(200, { "content-type": file[1], "cache-control": "no-store" });
      res.end(data);
    } catch { res.writeHead(500); res.end("cannot read the page"); }
  });
  const wss = new WebSocketServer({ server, maxPayload: 256 });
  wss.on("connection", ws => {
    clients.add(ws);
    send(ws, { type: "backlog", items: backlog });
    send(ws, { type: "status", status: robot.status });
    ws.on("close", () => clients.delete(ws));
    ws.on("error", () => clients.delete(ws));
    ws.on("message", data => {
      let msg: { type?: string; key?: string };
      try { msg = JSON.parse(String(data)); } catch { return; }
      // One printable character: the serial monitor's keys.
      if (msg.type !== "key" || typeof msg.key !== "string" || !/^[\x21-\x7e]$/.test(msg.key)) return;
      if (!robot.sendKey(msg.key)) note(`key ${msg.key} not sent: robot not connected`);
    });
  });
  server.listen(HTTP_PORT, "127.0.0.1", () => console.log(`Open http://127.0.0.1:${HTTP_PORT}/`));
}

main().catch(err => { console.error(err); process.exit(1); });
