// RoboCar robot server (doc/ARCHITECTURE.md, the PC): PicoA connects over WiFi, the browser at
// http://127.0.0.1:8081/ shows the robot's map, its snapshots (camera, ToF, colour,
// pose; any one of them back in time), the log, and steers it. All steering goes
// through the command API at /api/robot/ (api/, doc/CLI_API_PLAN.md): the page's and
// the `robot` CLI's commands share one queue, and everyone sees one log (log.ts),
// also written to a file in logs/. The robot's messages (wire.ts) arrive on one TCP
// connection (link.ts); each is stamped by its boot (boot.ts: the message's number,
// the time since power-up, a frame's move) and stored as it comes (store.ts: moves,
// frames, zones, status, snapshots). The camera pictures are PNGs in
// recordings/camera/<boot's folder>/ (snapshot.ts, boot_folders.ts).
// Settings (environment, or .env: .env.example): PORT (HTTP 8081), ROBOT_PORT (4211),
// e.g. for a second server beside a running one; RECORDINGS (the folder for the
// database and the pictures, default recordings/) and ROBOT_DB (the database, default
// <RECORDINGS>/robot2.duckdb); MODEL_API and MODEL (the vision model for robot
// detect); ANNOUNCE_TO (comma-separated addresses, default every network's
// broadcast): a test server announcing only to 127.0.0.1 isn't found by the robot.
import http from "node:http";
import { createWriteStream } from "node:fs";
import { mkdir, readFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { WebSocketServer, WebSocket } from "ws";
import { getRequestListener } from "@hono/node-server";
import { ROBOT_PORT, RobotLink, broadcastAddresses, type LivePose } from "./link.ts";
import { DB_PATH, RECORDINGS_DIRECTORY, Store } from "./store.ts";
import { handleSnapshotApi } from "./snapshots_api.ts";
import { CellMaps } from "./maps/cell_map.ts";
import { OutlineMaps } from "./maps/outline/index.ts";
import { ingest } from "./ingest.ts";
import { BootFolders } from "./boot_folders.ts";
import { Log } from "./log.ts";
import { LiveRobot } from "./live.ts";
import { wireCheck, type Hello, type MoveStart } from "./wire.ts";
import { CommandRunner, StoredSnapshots } from "./api/runner.ts";
import { CommandQueue } from "./api/queue.ts";
import { createApi } from "./api/routes.ts";
import { MODEL, MODEL_API, openAiChat } from "./objects/model.ts";

const root = path.join(path.dirname(fileURLToPath(import.meta.url)), "..");
const HTTP_PORT = Number(process.env.PORT ?? 8081);

async function main(): Promise<void> {
  await mkdir(path.join(root, "logs"), { recursive: true });
  await mkdir(RECORDINGS_DIRECTORY, { recursive: true });
  const stamp = new Date().toLocaleString("sv-SE").replace(/[ :]/g, "-"); // local time, sortable
  const logPath = path.join(root, "logs", `${stamp}.log`);
  const logFile = createWriteStream(logPath, { flags: "a" });

  const clients = new Set<WebSocket>();
  const send = (ws: WebSocket, message: unknown) => { if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(message)); };
  const broadcast = (message: unknown) => { for (const ws of clients) send(ws, message); };
  const log = new Log(logFile);
  log.on("item", item => broadcast({ type: item.kind, text: item.text, command: item.command }));
  const note = (text: string) => log.note(text);

  const bootFolders = new BootFolders(RECORDINGS_DIRECTORY);  // the robot's boot ids to our folder names
  const store = new Store(DB_PATH);
  store.onError = err => note(`the robot's data NOT stored: ${err.message}`);
  const maps = new CellMaps(store, note);  // the PC's cell map, built from the store when read
  const outlines = new OutlineMaps(store, maps, note);  // the PC's outline maps, brought up to each snapshot as it comes
  maps.placeBy(outlines);  // the PC's cell map placed by the corrected pose too
  const live = new LiveRobot();
  const stored = new StoredSnapshots();
  const announceTo = process.env.ANNOUNCE_TO ? process.env.ANNOUNCE_TO.split(",") : [];
  const robot = new RobotLink({ robotPort: Number(process.env.ROBOT_PORT ?? ROBOT_PORT), announceTo });

  // What the robot sends, stored in the order it came (ingest.ts); the page and the API told.
  ingest(robot, store, { recordingsDirectory: RECORDINGS_DIRECTORY, bootFolders, note, stored: s => {
    stored.add(s.bootId, s.sequence);
    void outlines.added(s.bootId, s.sequence);  // queued before the page's read of it
    broadcast({ type: "snapshot", boot: s.bootId, sequence: s.sequence });
  } });
  robot.on("hello", (h: Hello) => live.setBoot(h.bootId));
  robot.on("moveStart", (m: MoveStart) => live.setStill(m.preMs, m.postMs));
  robot.on("pose", (p: LivePose) => { live.setPose(p); broadcast({ type: "pose", pose: p }); });
  robot.on("text", (text: string) => log.text(text));
  robot.on("note", note);
  robot.on("status", status => { live.setStatus(status); broadcast({ type: "status", status }); });
  robot.on("mapCleared", () => note("the robot's map was cleared (its own 6 x 6 m map; the PC's are kept)"));
  const queue = new CommandQueue(new CommandRunner(robot, stored), (a, sender) => log.answer(a, sender));
  const chat = openAiChat(MODEL_API, MODEL, note);
  const api = getRequestListener(createApi({ store, maps, outlines, live, queue, log, snapshots: stored, config: { autoSnap: true },
                                              recordingsDirectory: RECORDINGS_DIRECTORY, bootFolders, chat, modelName: MODEL }).fetch);
  await robot.start();
  if (!wireCheck()) note("wire: the message table is wrong (two types share a number, or a ToF format is too big; wire.ts, tof_format.ts): the robot's messages can't be read");
  note(`server started; announcing on ${(announceTo.length ? announceTo : broadcastAddresses()).join(", ") || "no network"}; ` +
       `log ${path.relative(root, logPath)}; database ${path.relative(root, DB_PATH)}`);

  const files: Record<string, [string, string]> = {
    "/": ["app.html", "text/html"],
    "/app.js": ["app.js", "text/javascript"],
  };
  const server = http.createServer(async (req, res) => {
    if (/^\/api\/robot(\/|$|\?)/.test(req.url ?? "")) { await api(req, res); return; }
    if (await handleSnapshotApi(req, res, store, RECORDINGS_DIRECTORY, bootFolders, maps, outlines)) return;
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
