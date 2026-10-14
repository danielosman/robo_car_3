// The read commands (doc/CLI_API_PLAN.md, reading data): `robot <name> [arg]`. Each
// answers its data (for --json) and the same as short text. A snapshot's number is
// one of the newest boot's; without it, the newest snapshot.
import { mkdir, writeFile } from "node:fs/promises";
import path from "node:path";
import type { Store } from "../store.ts";
import type { LiveRobot } from "../live.ts";
import type { Log } from "../log.ts";
import { snapshotDetail, snapshotMap } from "../snapshots_api.ts";
import { FEATURE } from "../snapshot.ts";
import { CAMERA, MOVE } from "./firmware.ts";
import { poseText } from "./answers.ts";
import { colourName } from "./colour.ts";
import { RobotMap } from "./map.ts";
import type { CommandQueue, QueueState } from "./queue.ts";

export interface ReadContext { store: Store; live: LiveRobot; queue: CommandQueue; log: Log; recordingsDir: string }
export interface ReadResult { data: unknown; text: string }
export interface Read {
  usage: string;  // "map [n] [--png]"
  help: string;
  run(ctx: ReadContext, arg: string | undefined, flags: Set<string>): Promise<ReadResult>;
}

// A read that can't answer: the text says why; status is the HTTP status.
export class ReadError extends Error {
  readonly status: number;
  constructor(message: string, status = 404) { super(message); this.status = status; }
}

type SnapshotId = { boot: string; seq: number };

export async function findSnapshot(store: Store, arg: string | undefined): Promise<SnapshotId> {
  if (arg !== undefined && !/^\d+$/.test(arg)) throw new ReadError(`${arg}: not a snapshot number`, 400);
  return store.read(async c => {
    const newest = (await c.runAndReadAll("SELECT boot_id, seq FROM snapshots ORDER BY arrived_at DESC, seq DESC LIMIT 1")).getRowObjects()[0];
    if (!newest) throw new ReadError("no snapshots yet (robot steer s takes one)");
    const boot = String(newest.boot_id);
    if (arg === undefined) return { boot, seq: Number(newest.seq) };
    const seq = Number(arg);
    const found = (await c.runAndReadAll("SELECT 1 FROM snapshots WHERE boot_id = $1 AND seq = $2", [boot, seq])).getRowObjects();
    if (!found.length) throw new ReadError(`no snapshot ${seq} in boot ${boot} (the newest is ${newest.seq})`);
    return { boot, seq };
  });
}

export async function mapOf(store: Store, id: SnapshotId): Promise<RobotMap> {
  const view = await snapshotMap(store, id.boot, id.seq);
  if (!view?.window) throw new ReadError(`snapshot ${id.seq} has no map`);
  return new RobotMap(view);
}

const cmText = (m: number | null) => (m === null ? "none within 3 m" : `${Math.round(m * 100)} cm`);
const queueText = (q: QueueState) =>
  `running: ${q.running ? `${q.running.command} (${q.running.sender})` : "nothing"}; ` +
  `waiting: ${q.waiting.length ? q.waiting.map(w => `${w.command} (${w.sender})`).join(" ") : "nothing"}`;

// The nearest obstacle on the map ahead, left and right of the robot, from its edges
// (m), looking 3 m; null if none.
function obstacles(map: RobotMap) {
  const r = map.view.robot;
  if (!r) return null;
  const look = (turn: number, edge: number, half: number) => {
    const a = r.yaw + turn, along = map.obstacleAlong(r.x, r.y, Math.cos(a), Math.sin(a), half, 3 + edge);
    return along >= 3 + edge ? null : Math.max(0, along - edge); // nothing found: along is the most it looks
  };
  return { ahead: look(0, MOVE.edgeM, MOVE.halfWidthM), left: look(Math.PI / 2, MOVE.halfWidthM, MOVE.edgeM),
           right: look(-Math.PI / 2, MOVE.halfWidthM, MOVE.edgeM) };
}

export const READS: Record<string, Read> = {
  status: {
    usage: "status",
    help: "What the robot is doing now: where it is (its pose line, 4 a second), mode, recording, WiFi; what runs\n" +
          "and waits in the queue; the largest jolt and tilt PicoB measured since power-up.",
    async run({ live, queue }) {
      const p = live.pose, s = live.status, q = queue.state(), jolt = live.peaks.jolt.map(v => v.toFixed(2));
      const data = { connected: live.connected, status: s, pose: p, queue: q, peaks: live.peaks };
      const lines = [
        s.state === "connected" ? `robot connected (${s.ip})${p?.rssi ? `, WiFi ${p.rssi} dBm` : ""}`
          : s.state === "lost" ? `robot not connected (lost at ${new Date(s.at).toLocaleTimeString()})` : "robot not connected",
        p ? `${poseText(p)}; mode ${p.mode}${p.recording === null ? "" : `, recording ${p.recording ? "on" : "off"}`}` : "no pose yet",
        queueText(q),
        `safety since power-up: largest jolt x ${jolt[0]} g, y ${jolt[1]} g, z ${jolt[2]} g; ` +
          `largest tilt ${Math.round(live.peaks.tilt * 180 / Math.PI)}°`,
      ];
      return { data, text: lines.join("\n") };
    },
  },
  queue: {
    usage: "queue",
    help: "What runs and what waits in the queue, and who sent it (cli or page).",
    async run({ queue }) { const q = queue.state(); return { data: q, text: queueText(q) }; },
  },
  log: {
    usage: "log [lines]",
    help: "The last lines of the log everyone sees (default 20): the robot's own text, the server's notes, and every\n" +
          "command's answers with who sent it.",
    async run({ log }, arg) {
      if (arg !== undefined && !/^\d+$/.test(arg)) throw new ReadError(`${arg}: not a number of lines`, 400);
      const lines = log.tail(Math.min(1000, Number(arg ?? 20)));
      return { data: lines, text: lines.join("\n") };
    },
  },
  snap: {
    usage: "snap [n]",
    help: "A snapshot in a few lines: when, the last action and how it ended, the pose, the nearest obstacles on the\n" +
          "map ahead, left and right (from the robot's edges), the light, the camera's features.",
    async run({ store }, arg) {
      const id = await findSnapshot(store, arg);
      const [d, map] = [await snapshotDetail(store, id.boot, id.seq), await mapOf(store, id).catch(() => null)];
      const s = d!.snapshot as Record<string, unknown>;
      const pose = s.x === null ? null : { x: Number(s.x), y: Number(s.y), yaw: Number(s.yaw) };
      const near = map && obstacles(map);
      const lux = s.lux === null ? null : Number(s.lux);
      const colour = lux === null ? null : colourName(Number(s.cie_x), Number(s.cie_y), lux);
      const features = d!.features.filter(f => !(Number(f.flags) & FEATURE.LOST)).length;
      const data = { ...id, arrived: s.arrived, why: s.why, action: s.action || null, endReason: s.end_reason, mode: s.mode,
                     pose, obstacles: near, lux, colour, features };
      const text = [
        `snapshot ${id.seq} (boot ${id.boot}), ${s.arrived}, ${s.why}`,
        `last action: ${s.action ? `${s.action}, ${s.end_reason}` : "none"}; mode ${s.mode}`,
        pose ? poseText(pose) : "no pose (PicoB not connected)",
        near ? `obstacles on the map: ahead ${cmText(near.ahead)}, left ${cmText(near.left)}, right ${cmText(near.right)}` : "no map",
        lux === null ? "light: no reading" : `light: ${Math.round(lux)} lux, ${colour}`,
        `camera: ${s.image ? `${features} features` : "no picture"}`,
      ];
      return { data, text: text.join("\n") };
    },
  },
  light: {
    usage: "light [n]",
    help: "The colour sensor: lux, the colour (CIE 1931 x, y), and a rough colour name (not calibrated for the board).",
    async run({ store }, arg) {
      const id = await findSnapshot(store, arg);
      const s = (await snapshotDetail(store, id.boot, id.seq))!.snapshot as Record<string, unknown>;
      if (s.lux === null) throw new ReadError(`snapshot ${id.seq} has no colour reading`);
      const lux = Number(s.lux), x = Number(s.cie_x), y = Number(s.cie_y), colour = colourName(x, y, lux);
      return { data: { ...id, lux, cie: { x, y }, colour },
               text: `snapshot ${id.seq}: ${lux.toFixed(1)} lux, CIE x ${x.toFixed(3)} y ${y.toFixed(3)}, ${colour}` };
    },
  },
  features: {
    usage: "features [n]",
    help: `The camera's features (corners vision tracks): each one's id, its place in the ${CAMERA.width} × ${CAMERA.height}\n` +
          `picture (px; x right, y down from the top left; the middle is ${CAMERA.width / 2}, ${CAMERA.height / 2}; as the\n` +
          "robot sees it), and whether it is new, tracked (with its age in processed frames), missed in this frame or lost.",
    async run({ store }, arg) {
      const id = await findSnapshot(store, arg);
      const d = (await snapshotDetail(store, id.boot, id.seq))!;
      const state = (flags: number, age: number) => flags & FEATURE.LOST ? "lost" : flags & FEATURE.MISSED ? "missed"
        : flags & FEATURE.NEW || age === 0 ? "new" : `tracked, age ${age}`;
      const list = d.features.map(f => ({ id: Number(f.id), x: Number(f.x), y: Number(f.y), state: state(Number(f.flags), Number(f.age)) }));
      const text = [`snapshot ${id.seq}: ${list.filter(f => f.state !== "lost").length} features`,
                    ...list.map(f => `${String(f.id).padStart(5)}  (${f.x.toFixed(1)}, ${f.y.toFixed(1)})  ${f.state}`)];
      return { data: { ...id, width: CAMERA.width, height: CAMERA.height, features: list }, text: text.join("\n") };
    },
  },
  tof: {
    usage: "tof [n]",
    help: "The ToF sensor's 8 × 8 frame, cm along each ray, as the robot sees it: the top row looks highest, the bottom\n" +
          "rows (6-8) see the floor ~50, 30 and 20 cm ahead; the left column looks left. 45° across, 5.6° a zone.\n" +
          "Each zone's nearest sure target; -- nothing in range; ?N unsure (VL53 status N).",
    async run({ store }, arg) {
      const id = await findSnapshot(store, arg);
      const d = (await snapshotDetail(store, id.boot, id.seq))!;
      const cell = (z: (typeof d.grid)[number][number]) => !z ? "--" : z.sure ? String(Math.round(z.mm / 10)) : `?${z.status}`;
      const rows = d.grid.map(row => row.map(cell));
      return { data: { ...id, rows }, text: [`snapshot ${id.seq}`, ...rows.map(r => r.map(c => c.padStart(4)).join(""))].join("\n") };
    },
  },
  camera: {
    usage: "camera [n]",
    help: `The camera picture's file (PNG, ${CAMERA.width} × ${CAMERA.height}, grey, as the robot sees it).`,
    async run({ store, recordingsDir }, arg) {
      const id = await findSnapshot(store, arg);
      const s = (await snapshotDetail(store, id.boot, id.seq))!.snapshot as Record<string, unknown>;
      if (!s.image) throw new ReadError(`snapshot ${id.seq} has no picture`);
      const file = path.join(recordingsDir, String(s.image));
      return { data: { ...id, file }, text: file };
    },
  },
  map: {
    usage: "map [n] [--png]",
    help: "The map at snapshot n: the 6 × 6 m window around the robot, one character per 10 cm, x up (forward at\n" +
          "power-up), y to the left, x and y (cm) along the edges; the robot, the cell 30 cm ahead of it, the path it\n" +
          "drove since the map was last cleared, and a legend. --png: the same as a picture, answered with its file.",
    async run({ store, recordingsDir }, arg, flags) {
      const id = await findSnapshot(store, arg), map = await mapOf(store, id);
      if (flags.has("png")) {
        const file = path.join(recordingsDir, "maps", id.boot, `${id.seq}.png`);
        await mkdir(path.dirname(file), { recursive: true });
        await writeFile(file, map.png()!);
        return { data: { ...id, file }, text: file };
      }
      return { data: { ...id, ...map.view }, text: `snapshot ${id.seq}. ${map.text()!}` };
    },
  },
};
