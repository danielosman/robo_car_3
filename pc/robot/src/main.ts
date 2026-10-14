// RoboCar robot server (doc/ARCHITECTURE.md, the PC): PicoA connects over WiFi, the browser at
// http://127.0.0.1:8080/ shows the robot's map, its snapshots (camera, ToF, colour,
// pose; any one of them back in time), the log, and steers it. All steering goes
// through the command API at /api/robot/ (api/, doc/CLI_API_PLAN.md): the page's and
// the `robot` CLI's commands share one queue, and everyone sees one log (log.ts),
// also written to a file in logs/. Snapshots arrive between the
// console's text and are saved in recordings/snapshots/raw/, their pictures in
// recordings/camera/ (snapshot.ts), and stored in
// DuckDB (store.ts, snapshots_api.ts), as are the pose lines (live.ts). PicoA's recordings arrive on a second connection and are saved in
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
import { getRequestListener } from "@hono/node-server";
import { ROBOT_PORT, RobotLink, broadcastAddresses, type LivePose } from "./robot.ts";
import { DATA_PORT, RecordingServer } from "./recording.ts";
import { Store } from "./store.ts";
import { DB_PATH } from "./import.ts";
import { handleViewerApi } from "./viewer_api.ts";
import { handleSnapshotApi } from "./snapshots_api.ts";
import { SnapshotSaver, type Snapshot } from "./snapshot.ts";
import { Log } from "./log.ts";
import { LiveRobot } from "./live.ts";
import { CommandRunner, StoredSnapshots } from "./api/runner.ts";
import { CommandQueue } from "./api/queue.ts";
import { createApi } from "./api/routes.ts";

const root = path.join(path.dirname(fileURLToPath(import.meta.url)), "..");
const HTTP_PORT = Number(process.env.PORT ?? 8080);

async function main(): Promise<void> {
  await mkdir(path.join(root, "logs"), { recursive: true });
  const stamp = new Date().toLocaleString("sv-SE").replace(/[ :]/g, "-"); // local time, sortable
  const logPath = path.join(root, "logs", `${stamp}.log`);
  const logFile = createWriteStream(logPath, { flags: "a" });

  const clients = new Set<WebSocket>();
  const send = (ws: WebSocket, msg: unknown) => { if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg)); };
  const broadcast = (msg: unknown) => { for (const ws of clients) send(ws, msg); };
  const log = new Log(logFile);
  log.on("item", item => broadcast({ type: item.kind, text: item.text, command: item.command }));
  const note = (text: string) => log.note(text);

  const recordings = new RecordingServer({ dir: path.join(root, "recordings"), port: Number(process.env.DATA_PORT ?? DATA_PORT) });
  recordings.on("note", note);
  const store = new Store(DB_PATH);
  const live = new LiveRobot(store, err => note(`pose lines NOT stored: ${err.message}`));
  const stored = new StoredSnapshots();
  recordings.on("take", take => {
    store.importTake(take)
      .then(n => note(`${take.name} stored: ${n.frames} frames, ${n.targets} targets, ${n.odom} odometry reports`))
      .catch(err => note(`${take.name} NOT stored: ${(err as Error).message}`));
  });
  const recordingsDir = path.join(root, "recordings");
  const snapshots = new SnapshotSaver(recordingsDir);
  snapshots.on("note", note);
  snapshots.on("snapshot", (s: Snapshot, ms: number, file: string, image: string | null) => {
    live.setBoot(s.bootId);
    void live.flush(); // the path up to this snapshot goes in first
    store.importSnapshot(s, ms, file, image)
      .then(() => { stored.add(s.bootId, s.seq); broadcast({ type: "snapshot", boot: s.bootId, seq: s.seq }); })
      .catch(err => note(`snapshot ${s.seq} NOT stored: ${(err as Error).message}`));
  });
  const dataPort = await recordings.start();
  const announceTo = process.env.ANNOUNCE_TO ? process.env.ANNOUNCE_TO.split(",") : [];
  const robot = new RobotLink({ dataPort, robotPort: Number(process.env.ROBOT_PORT ?? ROBOT_PORT), announceTo });
  robot.on("text", (text: string) => log.text(text));
  robot.on("note", note);
  robot.on("bytes", (n: number) => recordings.consoleArrived(n));
  robot.on("status", status => { live.setStatus(status); broadcast({ type: "status", status }); });
  robot.on("snapshot", (b: Buffer) => snapshots.arrived(b));
  robot.on("pose", (p: LivePose) => { live.setPose(p); broadcast({ type: "pose", pose: p }); });
  const queue = new CommandQueue(new CommandRunner(robot, stored), (a, sender) => log.answer(a, sender));
  const api = getRequestListener(createApi({ store, live, queue, log, snapshots: stored, config: { autoSnap: true }, recordingsDir }).fetch);
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
    if (/^\/api\/robot(\/|$|\?)/.test(req.url ?? "")) { await api(req, res); return; }
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
  // The page's live feed: the log, the robot's status and pose, new snapshots. It
  // takes nothing back: the page steers through the API.
  const wss = new WebSocketServer({ server, maxPayload: 256 });
  wss.on("connection", ws => {
    clients.add(ws);
    send(ws, { type: "backlog", items: log.backlog() });
    send(ws, { type: "status", status: robot.status });
    if (live.pose) send(ws, { type: "pose", pose: live.pose });
    ws.on("close", () => clients.delete(ws));
    ws.on("error", () => clients.delete(ws));
  });
  server.listen(HTTP_PORT, "127.0.0.1", () => console.log(`Open http://127.0.0.1:${HTTP_PORT}/`));
}

main().catch(err => { console.error(err); process.exit(1); });
