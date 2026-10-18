// robot detect, detect_save and objects (doc/OBJECTS_PLAN.md): they run at once, as the
// reads do (api/reads.ts), on any snapshot of the newest boot, with or without the robot.
//   detect [n] [--no-thinking] [--reasoning-budget=N]
//                               the picture to the vision model (session.ts), then saved
//                               as detect_save saves
//   detect_save <n> --by=<name> '<json>'
//                               boxes found elsewhere (an outside agent that read
//                               robot camera n): checked (labels.ts), each placed on the
//                               map (place.ts), saved replacing the snapshot's earlier
//                               run (db.ts)
//   objects [n] [--transcript]  what detect or detect_save saved for the snapshot
// Answers in the API's units: cm and degrees; the boxes as the model gave them, on 0-1000.
import { readFile } from "node:fs/promises";
import path from "node:path";
import { ReadError, findSnapshot, type ApiContext, type Read } from "../api/reads.ts";
import { yawDeg } from "../api/answers.ts";
import { checkReport } from "./labels.ts";
import { MODEL, ModelError, REASONING_BUDGET, parseBudget } from "./model.ts";
import { place, type SnapPoseLike } from "./place.ts";
import { PROMPT_VERSION, detectObjects, promptFor } from "./session.ts";
import { runOf, saveRun, transcriptOf, type StoredObject } from "./db.ts";

const cm = (m: number | null) => (m === null ? null : Math.round(m * 100));

// bbox: y first on 0-1000 (labels.ts); shape: the map's cm, x first.
function objectOut(o: StoredObject) {
  return {
    label: o.label, confidence: o.confidence, bbox: [o.ymin, o.xmin, o.ymax, o.xmax], floor: o.floor, distance: cm(o.distance),
    shape: o.kind === null ? null : { kind: o.kind, x: cm(o.cx), y: cm(o.cy), width: cm(o.width), depth: cm(o.depth), angle: yawDeg(o.angle ?? 0) },
  };
}

type SnapshotId = Awaited<ReturnType<typeof findSnapshot>>;

async function snapshotForDetect(ctx: ApiContext, id: SnapshotId) {
  const [row] = await ctx.store.query(`SELECT image, x, y, yaw, pitch FROM snapshots WHERE snap_id = '${id.boot}-${id.seq}'`);
  if (!row?.image) throw new ReadError(`snapshot ${id.seq} has no picture`);
  if (row.x === null) throw new ReadError(`snapshot ${id.seq} has no pose (PicoB not connected), so nothing can be placed`);
  const pose: SnapPoseLike = { x: Number(row.x), y: Number(row.y), yaw: Number(row.yaw), pitch: Number(row.pitch ?? 0) };
  return { pose, file: path.join(ctx.recordingsDir, String(row.image)) };
}

const usage = (args: string[], u: string) => { if (args.length > 1) throw new ReadError(`usage: robot ${u}`, 400); };

// What detect_save saves: the boxes as report_objects takes them, who found them, and
// how. detect fills in everything; from the CLI only objects and by come, the rest null.
export interface DetectSave {
  objects: unknown;               // [{"label": ..., "bbox": [ymin, xmin, ymax, xmax], "confidence": 0-100}, ...]
  by: string;                     // the model, or the outside agent (--by)
  promptVersion: number | null; thinking: boolean | null; seconds: number | null; turns: number | null;
  reasoningBudget: number | null;  // the thinking limit in tokens, null: none
  transcript: unknown;            // the conversation, or null
  result?: "ok" | "failed"; why?: string | null;  // only detect: a session that failed saves no boxes
}

// Checks the boxes, places each on the map and saves them, replacing the snapshot's
// earlier run. Wrong boxes: nothing saved, a 400 listing what is wrong.
export async function detectSave(ctx: ApiContext, id: SnapshotId, input: DetectSave) {
  const { pose } = await snapshotForDetect(ctx, id);
  const result = input.result ?? "ok";
  const checked = checkReport({ objects: result === "ok" ? input.objects : [] });
  if (!checked.ok) throw new ReadError(`not saved; fix these:\n${checked.errors.join("\n")}`, 400);
  const detections = checked.detections;
  await saveRun(ctx.store, {
    boot: id.boot, seq: id.seq, atMs: Date.now(), model: input.by, promptVersion: input.promptVersion, thinking: input.thinking,
    reasoningBudget: input.reasoningBudget, seconds: input.seconds, turns: input.turns, result, why: input.why ?? null,
    transcript: input.transcript === null || input.transcript === undefined ? null : JSON.stringify(input.transcript),
  }, detections, detections.map(d => place(d, pose)));
  ctx.log.note(`detect ${id.seq} (${input.by}): ${result === "ok" ? `${detections.length} objects` : `failed: ${input.why}`}`);
  const saved = await runOf(ctx.store, id.boot, id.seq);
  return { snapshot: id.seq, result, ...(result === "failed" ? { why: input.why } : {}),
           seconds: input.seconds, turns: input.turns, objects: saved!.objects.map(objectOut) };
}

// detect_save's JSON: {"objects": [...]} and, from a program, by, prompt_version,
// thinking, seconds, turns and transcript (each may be left out: null).
function parseSave(text: string, byFlag: string | undefined): DetectSave {
  let body: Record<string, unknown>;
  try { body = JSON.parse(text); }
  catch (err) { throw new ReadError(`the JSON doesn't parse: ${(err as Error).message}`, 400); }
  if (typeof body !== "object" || body === null || Array.isArray(body)) throw new ReadError('the JSON must be {"objects": [...]}', 400);
  const by = byFlag ?? body.by;
  if (typeof by !== "string" || !by.trim()) throw new ReadError("say who found the objects: --by=<name>", 400);
  const field = <T>(name: string, type: "number" | "boolean"): T | null => {
    const v = body[name];
    if (v === undefined || v === null) return null;
    if (typeof v !== type) throw new ReadError(`${name} must be a ${type}`, 400);
    return v as T;
  };
  return { objects: body.objects, by: by.trim(), promptVersion: field("prompt_version", "number"), thinking: field("thinking", "boolean"),
           reasoningBudget: field("reasoning_budget", "number"), seconds: field("seconds", "number"), turns: field("turns", "number"), transcript: body.transcript ?? null };
}

export const OBJECT_COMMANDS: Record<string, Read> = {
  detect: {
    usage: "detect [n] [--no-thinking] [--reasoning-budget=N]",
    help: "Finds the objects in the camera picture at snapshot n with the vision model, places each on the map,\n" +
          "and saves them, replacing what detect found for that snapshot before. Takes ~15 s (the first time also\n" +
          "the model loading); --no-thinking: ~1 s, finds less. --reasoning-budget=N: think at most N tokens (default\n" +
          "REASONING_BUDGET in .env; -1: no limit), then answer. Each object: label, confidence (the model's, 0-100 %),\n" +
          "bbox ([ymin, xmin, ymax, xmax] on 0-1000, y first, whatever order the model gave), floor, distance (cm from the\n" +
          "camera to its front edge) " +
          "and shape on the map (x, y its centre, width along the front edge, depth, angle of the front edge).\n" +
          "floor: floor (placed where its bottom edge meets the floor), cut_off (the picture's bottom cuts it:\n" +
          "at most this far), above_horizon or too_far (not placed). Labels: chair, table, sofa, door (the part\n" +
          "that swings), doorway (the opening), plant, shelf, cabinet, box, person, animal, object. result failed: why says what\n" +
          "went wrong.",
    async run(ctx, args, flags, req) {
      usage(args, this.usage);
      if (!ctx.chat) throw new ReadError("no vision model set up (MODEL_API)", 503);
      const model = ctx.modelName ?? MODEL;
      let prompt;
      try { prompt = promptFor(model); } catch (err) { throw new ReadError((err as Error).message, 503); }
      const id = await findSnapshot(ctx.store, args[0]);
      const { file } = await snapshotForDetect(ctx, id);
      const thinking = !flags.has("nothinking"), budgetFlag = req?.query.reasoningbudget;
      if (!thinking && budgetFlag !== undefined) throw new ReadError("--reasoning-budget with --no-thinking: there is no thinking to limit", 400);
      let reasoningBudget: number | null;
      try { reasoningBudget = thinking ? (budgetFlag === undefined ? REASONING_BUDGET : parseBudget(budgetFlag, "--reasoning-budget")) : null; }
      catch (err) { throw new ReadError((err as Error).message, 400); }
      const png = await readFile(file), started = Date.now();
      let session;
      try { session = await detectObjects(ctx.chat, png, { thinking, reasoningBudget, prompt }); }
      catch (err) {
        if (err instanceof ModelError) throw new ReadError(err.message, 502);
        throw err;
      }
      return detectSave(ctx, id, {
        objects: session.result === "ok" ? session.detections : [], by: model, promptVersion: PROMPT_VERSION, thinking, reasoningBudget,
        seconds: Math.round((Date.now() - started) / 100) / 10, turns: session.turns, transcript: session.transcript,
        result: session.result, why: session.result === "failed" ? session.why : null,
      });
    },
  },
  detect_save: {
    usage: "detect_save <n> --by=<name> '<json>'",
    help: "Saves objects found in the camera picture at snapshot n by someone other than robot detect, e.g. an\n" +
          "agent that looked at robot camera n. The JSON: {\"objects\": [{\"label\": \"chair\", \"bbox\": [ymin, xmin, ymax, xmax],\n" +
          "\"confidence\": 85}, ...]}, bbox on a 0-1000 grid, y first; confidence 0-100 %; [] if there is nothing.\n" +
          "Labels as robot detect --help lists them. --by says who found them (saved as the model). Each object is\n" +
          "placed on the map and saved, replacing what was saved for that snapshot before; the answer is robot\n" +
          "detect's. Wrong objects: nothing is saved and the answer says what to fix.",
    async run(ctx, _args, _flags, req) {
      const m = /^(\S+)\s+([\s\S]+)$/.exec(req?.body ?? "");
      if (!m) throw new ReadError(`usage: robot ${this.usage}`, 400);
      if (!/^\d+$/.test(m[1])) throw new ReadError(`${m[1]} is not a snapshot number`, 400);
      const id = await findSnapshot(ctx.store, m[1]);
      return detectSave(ctx, id, parseSave(m[2], req?.query.by));
    },
  },
  objects: {
    usage: "objects [n] [--transcript]",
    help: "Shows what robot detect or detect_save saved for snapshot n (see robot detect --help), when and how. --transcript: the\n" +
          "whole conversation with the model instead (its thinking included).",
    async run(ctx, args, flags) {
      usage(args, this.usage);
      const id = await findSnapshot(ctx.store, args[0]);
      if (flags.has("transcript")) {
        const t = await transcriptOf(ctx.store, id.boot, id.seq);
        if (t === null) throw new ReadError(`detect hasn't run on snapshot ${id.seq} (robot detect ${id.seq})`);
        if (t.transcript === null) throw new ReadError(`snapshot ${id.seq}'s objects were saved by ${t.by} without a conversation`);
        return { snapshot: id.seq, transcript: JSON.parse(t.transcript) };
      }
      const saved = await runOf(ctx.store, id.boot, id.seq);
      if (!saved) throw new ReadError(`detect hasn't run on snapshot ${id.seq} (robot detect ${id.seq})`);
      return { snapshot: id.seq, ...saved.run, objects: saved.objects.map(objectOut) };
    },
  },
};
