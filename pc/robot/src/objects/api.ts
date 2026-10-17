// robot detect and robot objects (doc/OBJECTS_PLAN.md): they run at once, as the reads
// do (api/reads.ts), on any snapshot of the newest boot, with or without the robot.
//   detect [n] [--no-thinking]  the picture to the vision model (session.ts); each box
//                               placed on the map (place.ts); saved, replacing the
//                               snapshot's earlier run (db.ts)
//   objects [n] [--transcript]  what detect saved for the snapshot
// Answers in the API's units: cm and degrees; the boxes as the model gave them, on 0-1000.
import { readFile } from "node:fs/promises";
import path from "node:path";
import { ReadError, findSnapshot, type ApiContext, type Read } from "../api/reads.ts";
import { yawDeg } from "../api/answers.ts";
import type { Detection } from "./labels.ts";
import { MODEL, ModelError } from "./model.ts";
import { place, type SnapPoseLike } from "./place.ts";
import { PROMPT_VERSION, detectObjects } from "./session.ts";
import { runOf, saveRun, transcriptOf, type StoredObject } from "./db.ts";

const cm = (m: number | null) => (m === null ? null : Math.round(m * 100));

function objectOut(o: StoredObject) {
  return {
    label: o.label, confidence: o.confidence, bbox: [o.ymin, o.xmin, o.ymax, o.xmax], floor: o.floor, distance: cm(o.distance),
    shape: o.kind === null ? null : { kind: o.kind, x: cm(o.cx), y: cm(o.cy), width: cm(o.width), depth: cm(o.depth), angle: yawDeg(o.angle ?? 0) },
  };
}

async function snapshotForDetect(ctx: ApiContext, arg: string | undefined) {
  const id = await findSnapshot(ctx.store, arg);
  const [row] = await ctx.store.query(`SELECT image, x, y, yaw, pitch FROM snapshots WHERE snap_id = '${id.boot}-${id.seq}'`);
  if (!row?.image) throw new ReadError(`snapshot ${id.seq} has no picture`);
  if (row.x === null) throw new ReadError(`snapshot ${id.seq} has no pose (PicoB not connected), so nothing can be placed`);
  const pose: SnapPoseLike = { x: Number(row.x), y: Number(row.y), yaw: Number(row.yaw), pitch: Number(row.pitch ?? 0) };
  return { id, pose, file: path.join(ctx.recordingsDir, String(row.image)) };
}

const usage = (args: string[], u: string) => { if (args.length > 1) throw new ReadError(`usage: robot ${u}`, 400); };

export const OBJECT_COMMANDS: Record<string, Read> = {
  detect: {
    usage: "detect [n] [--no-thinking]",
    help: "Finds the objects in the camera picture at snapshot n with the vision model, places each on the map,\n" +
          "and saves them, replacing what detect found for that snapshot before. Takes ~15 s (the first time also\n" +
          "the model loading); --no-thinking: ~1 s, finds less. Each object: label, confidence (the model's, 0-100 %),\n" +
          "bbox as the model gave it ([ymin, xmin, ymax, xmax] on 0-1000, y first), floor, distance (cm from the\n" +
          "camera to its front edge) " +
          "and shape on the map (x, y its centre, width along the front edge, depth, angle of the front edge).\n" +
          "floor: floor (placed where its bottom edge meets the floor), cut_off (the picture's bottom cuts it:\n" +
          "at most this far), above_horizon or too_far (not placed). Labels: chair, table, sofa, door (the part\n" +
          "that swings), doorway (the opening), plant, shelf, cabinet, box, person, animal, object. result failed: why says what\n" +
          "went wrong.",
    async run(ctx, args, flags) {
      usage(args, this.usage);
      if (!ctx.chat) throw new ReadError("no vision model set up (MODEL_API)", 503);
      const { id, pose, file } = await snapshotForDetect(ctx, args[0]);
      const png = await readFile(file), thinking = !flags.has("nothinking"), started = Date.now();
      let session;
      try { session = await detectObjects(ctx.chat, png, { thinking }); }
      catch (err) {
        if (err instanceof ModelError) throw new ReadError(err.message, 502);
        throw err;
      }
      const detections: Detection[] = session.result === "ok" ? session.detections : [];
      const seconds = Math.round((Date.now() - started) / 100) / 10;
      await saveRun(ctx.store, {
        boot: id.boot, seq: id.seq, atMs: started, model: ctx.modelName ?? MODEL, promptVersion: PROMPT_VERSION, thinking,
        seconds, turns: session.turns, result: session.result, why: session.result === "failed" ? session.why : null,
        transcript: JSON.stringify(session.transcript),
      }, detections, detections.map(d => place(d, pose)));
      ctx.log.note(`detect ${id.seq}: ${session.result === "ok" ? `${detections.length} objects` : `failed: ${session.why}`} (${seconds} s)`);
      const saved = await runOf(ctx.store, id.boot, id.seq);
      return { snapshot: id.seq, result: session.result, ...(session.result === "failed" ? { why: session.why } : {}),
               seconds, turns: session.turns, objects: saved!.objects.map(objectOut) };
    },
  },
  objects: {
    usage: "objects [n] [--transcript]",
    help: "Shows what robot detect found at snapshot n (see robot detect --help), when and how. --transcript: the\n" +
          "whole conversation with the model instead (its thinking included).",
    async run(ctx, args, flags) {
      usage(args, this.usage);
      const id = await findSnapshot(ctx.store, args[0]);
      if (flags.has("transcript")) {
        const t = await transcriptOf(ctx.store, id.boot, id.seq);
        if (t === null) throw new ReadError(`detect hasn't run on snapshot ${id.seq} (robot detect ${id.seq})`);
        return { snapshot: id.seq, transcript: JSON.parse(t) };
      }
      const saved = await runOf(ctx.store, id.boot, id.seq);
      if (!saved) throw new ReadError(`detect hasn't run on snapshot ${id.seq} (robot detect ${id.seq})`);
      return { snapshot: id.seq, ...saved.run, objects: saved.objects.map(objectOut) };
    },
  },
};
