// RoboCar robot server (doc/ARCHITECTURE.md, the PC): PicoA connects over WiFi, the browser at
// http://127.0.0.1:8080/ shows the robot's map, its snapshots (camera, ToF, colour,
// pose; any one of them back in time), its console, and steers it: keys, the mode,
// and in manual mode a click on the map sends it there. The console alone is at
// /console. Every run writes a log file in logs/. Snapshots arrive between the
// console's text and are saved in recordings/snapshots/ (snapshot.ts) and stored in
// DuckDB (store.ts, snapshots_api.ts). PicoA's recordings arrive on a second connection and are saved in
// recordings/ (recording.ts); each take goes into DuckDB as it
// ends (store.ts), listed at http://127.0.0.1:8080/takes and shown in 3D at /viewer
// (viewer_api.ts). Ports: PORT (HTTP 8080),
// ROBOT_PORT (4211), DATA_PORT (4212), e.g. for a second server beside a running one;
// ANNOUNCE_TO (comma-separated addresses, default every network's broadcast): a test
// server announcing only to 127.0.0.1 isn't found by the robot.
import http from "node:http";
import { createWriteStream } from "node:fs";
import { mkdir, readFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { WebSocketServer, WebSocket } from "ws";
import { ROBOT_PORT, RobotLink, broadcastAddresses } from "./robot.ts";
import { DATA_PORT, RecordingServer } from "./recording.ts";
import { Store } from "./store.ts";
import { DB_PATH } from "./import.ts";
import { handleViewerApi } from "./viewer_api.ts";
import { handleSnapshotApi } from "./snapshots_api.ts";
import { SnapshotSaver, imagePath, type Snapshot } from "./snapshot.ts";
import type { LivePose } from "./robot.ts";

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

  const recordings = new RecordingServer({ dir: path.join(root, "recordings"), port: Number(process.env.DATA_PORT ?? DATA_PORT) });
  recordings.on("note", note);
  const store = new Store(DB_PATH);
  recordings.on("take", take => {
    store.importTake(take)
      .then(n => note(`${take.name} stored: ${n.frames} frames, ${n.targets} targets, ${n.odom} odometry reports`))
      .catch(err => note(`${take.name} NOT stored: ${(err as Error).message}`));
  });
  const recordingsDir = path.join(root, "recordings");
  const snapshots = new SnapshotSaver(path.join(recordingsDir, "snapshots"));
  snapshots.on("note", note);
  snapshots.on("snapshot", (s: Snapshot, ms: number, file: string) => {
    const image = s.camera ? path.relative(recordingsDir, imagePath(path.join(recordingsDir, "snapshots"), s.bootId, s.seq)) : null;
    store.importSnapshot(s, ms, file, image)
      .then(() => broadcast({ type: "snapshot", boot: s.bootId, seq: s.seq }))
      .catch(err => note(`snapshot ${s.seq} NOT stored: ${(err as Error).message}`));
  });
  let lastPose: LivePose | null = null;
  const dataPort = await recordings.start();
  const announceTo = process.env.ANNOUNCE_TO ? process.env.ANNOUNCE_TO.split(",") : [];
  const robot = new RobotLink({ dataPort, robotPort: Number(process.env.ROBOT_PORT ?? ROBOT_PORT), announceTo });
  robot.on("text", (text: string) => { logFile.write(text); process.stdout.write(text); add({ kind: "text", text }); });
  robot.on("note", note);
  robot.on("bytes", (n: number) => recordings.consoleArrived(n));
  robot.on("status", status => broadcast({ type: "status", status }));
  robot.on("snapshot", (b: Buffer) => snapshots.arrived(b));
  robot.on("pose", (p: LivePose) => { lastPose = p; broadcast({ type: "pose", pose: p }); });
  await robot.start();
  note(`server started; announcing on ${(announceTo.length ? announceTo : broadcastAddresses()).join(", ") || "no network"}; log ${path.relative(root, logPath)}`);

  const files: Record<string, [string, string]> = {
    "/": ["app.html", "text/html"],
    "/app.js": ["app.js", "text/javascript"],
    "/console": ["index.html", "text/html"],
    "/console.js": ["console.js", "text/javascript"],
    "/takes": ["takes.html", "text/html"],
    "/takes.js": ["takes.js", "text/javascript"],
    "/viewer": ["viewer.html", "text/html"],
    "/viewer.js": ["viewer.js", "text/javascript"],
  };
  const json = (res: http.ServerResponse, code: number, body: unknown) => {
    res.writeHead(code, { "content-type": "application/json", "cache-control": "no-store" });
    res.end(JSON.stringify(body));
  };
  const server = http.createServer(async (req, res) => {
    if (await handleViewerApi(req, res, store)) return;
    if (await handleSnapshotApi(req, res, store, recordingsDir)) return;
    if (req.url === "/api/takes" && req.method === "GET") {
      try {
        json(res, 200, await store.query(`SELECT take_id, boot_id, take_no, strftime(started_at, '%Y-%m-%d %H:%M:%S') AS started,
          key, action, round(duration_s, 1) AS duration_s, end_reason, frames_whole, frames_half, frames_missing,
          datagrams, datagrams_lost, odom_reports, note FROM takes ORDER BY started_at DESC, take_no DESC`));
      } catch (err) { json(res, 500, { error: (err as Error).message }); }
      return;
    }
    const noteUrl = /^\/api\/takes\/([0-9a-f]{8}-\d+)\/note$/.exec(req.url ?? "");
    if (noteUrl && req.method === "POST") {
      let body = "";
      req.on("data", c => { if (body.length < 2000) body += c; });
      req.on("end", () => {
        store.setNote(noteUrl[1], body.slice(0, 1000)).then(() => json(res, 200, { ok: true }),
          err => json(res, 500, { error: (err as Error).message }));
      });
      return;
    }
    // three.js for the viewer, from node_modules (npm install): /vendor/three/build/…, /vendor/three/addons/…
    const vendor = /^\/vendor\/three\/(build|addons)\/([\w./-]+\.js)$/.exec(new URL(req.url ?? "/", "http://localhost").pathname);
    if (vendor && !vendor[2].includes("..")) {
      const dir = path.join(root, "node_modules", "three", vendor[1] === "build" ? "build" : "examples/jsm");
      try {
        res.writeHead(200, { "content-type": "text/javascript", "cache-control": "max-age=3600" });
        res.end(await readFile(path.join(dir, vendor[2])));
      } catch { res.writeHead(404); res.end("not found"); }
      return;
    }
    const file = files[new URL(req.url ?? "/", "http://localhost").pathname];
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
    if (lastPose) send(ws, { type: "pose", pose: lastPose });
    ws.on("close", () => clients.delete(ws));
    ws.on("error", () => clients.delete(ws));
    ws.on("message", data => {
      let msg: { type?: string; key?: string; x?: number; y?: number; mode?: string };
      try { msg = JSON.parse(String(data)); } catch { return; }
      // A goal on the map (m, map frame), the mode, or one printable character: the serial monitor's keys.
      if (msg.type === "goal" && Number.isFinite(msg.x) && Number.isFinite(msg.y) && Math.abs(msg.x!) < 300 && Math.abs(msg.y!) < 300) {
        const command = `goal ${Math.round(msg.x! * 100)} ${Math.round(msg.y! * 100)}`;
        if (!robot.sendCommand(command)) note(`${command} not sent: robot not connected`);
        return;
      }
      if (msg.type === "mode" && (msg.mode === "idle" || msg.mode === "manual")) {
        if (!robot.sendCommand(`mode ${msg.mode}`)) note(`mode ${msg.mode} not sent: robot not connected`);
        return;
      }
      if (msg.type !== "key" || typeof msg.key !== "string" || !/^[\x20-\x7e]$/.test(msg.key)) return;
      if (!robot.sendKey(msg.key)) note(`key ${msg.key} not sent: robot not connected`);
    });
  });
  server.listen(HTTP_PORT, "127.0.0.1", () => console.log(`Open http://127.0.0.1:${HTTP_PORT}/`));
}

main().catch(err => { console.error(err); process.exit(1); });
