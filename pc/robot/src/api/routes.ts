// The command API over HTTP (doc/CLI_API_PLAN.md), under /api/robot/:
//   POST /api/robot/<command>        the body: its arguments ("d50 r30", "on", "42"); flags as query
//                                    parameters (?png, ?nowait, ?from=page)
//   GET  /api/robot/help[/<command>] the help, as text
// steer, snap, stop, clear_map, mode and config pre_ms|post_ms go into the queue (stop skips it):
// one JSON line per command streams back as each ends; ?nowait answers only what was
// queued. check, the reads (reads.ts), and detect, detect_save and objects (objects/api.ts) answer one JSON object at once. A command
// the API can't take is answered 400 ("result":"failed"); an unknown one 404.
import { Hono } from "hono";
import { commandText, parseAction, parseSteer, withSnaps, type Command, type Parsed } from "./commands.ts";
import type { Answer, Sender } from "./answers.ts";
import { checkMoves } from "./check.ts";
import { help } from "./help.ts";
import { formatJson } from "./json.ts";
import { READS, ReadError, TextAnswer, findSnapshot, type ApiContext } from "./reads.ts";
import { OBJECT_COMMANDS } from "../objects/api.ts";
import type { LogItem } from "../log.ts";

const JSON_TYPE = { "content-type": "application/json; charset=utf-8", "cache-control": "no-store" };
const json = (lines: string, status = 200) => new Response(`${lines}\n`, { status, headers: JSON_TYPE });
const line = (v: unknown) => JSON.stringify(v);

export function createApi(ctx: ApiContext): Hono {
  const app = new Hono().basePath("/api/robot");

  app.get("/help/:name?", c => {
    const h = help(c.req.param("name"));
    return new Response(`${h ?? `no help for ${c.req.param("name")} (robot --help lists the commands)`}\n`,
                        { status: h === null ? 404 : 200, headers: { "content-type": "text/plain; charset=utf-8" } });
  });

  app.post("/:name", async c => {
    const name = c.req.param("name"), arg = (await c.req.text()).trim(), flags = new Set(Object.keys(c.req.query()));
    const sender: Sender = c.req.query("from") === "page" ? "page" : "cli";
    const refuse = (why: string) => json(line({ command: `${name} ${arg}`.trim(), result: "failed", why }), 400);

    // config's auto_snap and config alone are read here; pre_ms and post_ms go to the robot.
    const queuedAction = parseAction(name, "") !== null && (name !== "config" || /^(pre|post)_ms\b/.test(arg));
    if (name === "steer" || name === "check" || name === "stop" || queuedAction) {
      const parsed: Parsed = name === "steer" || name === "check" ? parseSteer(arg)
        : name === "stop" ? (arg ? { error: "stop takes nothing" } : { commands: [{ name: "stop", values: [] }] })
        : parseAction(name, arg)!;
      if ("error" in parsed) return refuse(parsed.error);
      if (name === "check") return check(parsed.commands);
      return queued(name === "steer" ? withSnaps(parsed.commands, ctx.config.autoSnap) : parsed.commands, sender, flags.has("nowait"));
    }
    const read = READS[name] ?? OBJECT_COMMANDS[name];
    if (!read) return json(line({ error: `${name} is not a command (robot --help lists them)` }), 404);
    try {
      const answer = await read.run(ctx, arg ? arg.split(/\s+/) : [], flags, { body: arg, query: c.req.query() });
      if (answer instanceof TextAnswer) return new Response(answer.text, { headers: { "content-type": "text/plain; charset=utf-8", "cache-control": "no-store" } });
      return json(formatJson(answer));
    } catch (err) {
      if (err instanceof ReadError) return json(line({ error: err.message }), err.status);
      throw err;
    }
  });

  async function check(commands: Command[]): Promise<Response> {
    if (commands.some(cmd => cmd.name === "watch")) return json(line({ command: "check", result: "failed", why: "watch can't be checked" }), 400);
    try {
      const id = await findSnapshot(ctx.store, undefined);
      const answers = await ctx.maps.atSnapshot(id.boot, id.sequence, async (map, pose) => pose && checkMoves(commands, map, pose));
      if (!answers) return json(line({ error: "the newest snapshot has no pose (PicoB not connected)" }), 404);
      return json(answers.map(line).join("\n"));
    } catch (err) {
      if (err instanceof ReadError) return json(line({ error: err.message }), err.status);
      throw err;
    }
  }

  // A config pre_ms / post_ms the robot took: the still times now (until a move says them).
  function stillSet(a: Answer): void {
    const m = /^config (pre|post)_ms (\d+)$/.exec(a.command);
    if (!m || a.result !== "ok") return;
    const now = ctx.live.still ?? { preMs: 200, postMs: 200 }; // not said yet: the robot's power-up values
    ctx.live.setStill(m[1] === "pre" ? Number(m[2]) : now.preMs, m[1] === "post" ? Number(m[2]) : now.postMs);
  }

  // Into the queue (stop past it); each answer streams back as its command ends. A
  // report's answer carries the robot's text that came while it waited (its printout).
  function queued(commands: Command[], sender: Sender, nowait: boolean): Response {
    let open = true;
    const text: string[] = [];
    const listen = (item: LogItem) => { if (item.kind === "text") text.push(item.text); };
    const reports = commands.some(c => c.name === "report");
    if (reports) ctx.log.on("item", listen);
    const stream = new ReadableStream<string>({
      start(controller) {
        const write = (a: Answer) => {
          stillSet(a);
          if (a.command === "report") {
            ctx.log.off("item", listen);
            if (a.result === "ok") a = { ...a, printout: text.join("").split("\n").filter(l => l) };
          }
          if (open) controller.enqueue(`${line(a)}\n`);
        };
        const close = () => { if (open) { open = false; controller.close(); } };
        const finished = commands[0].name === "stop" ? ctx.queue.stop(sender, write) : ctx.queue.submit(commands, sender, write);
        if (nowait) { controller.enqueue(`${line({ queued: commands.map(commandText) })}\n`); close(); }
        else void finished.then(close);
      },
      cancel() { open = false; }, // Ctrl-C: the commands stay queued
    });
    return new Response(stream.pipeThrough(new TextEncoderStream()), { headers: JSON_TYPE });
  }

  return app;
}
