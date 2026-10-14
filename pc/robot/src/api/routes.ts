// The command API over HTTP (doc/CLI_API_PLAN.md), under /api/robot/:
//   POST /api/robot/steer           the body: steering commands ("r30d50"); answers stream back as text, one
//                                   line per command as each ends. ?check: what they would do, nothing
//                                   moves; ?nowait: only the "queued" line; ?from=page: sent by the page
//   GET  /api/robot/<read>[/<arg>]  a read (reads.ts) as text; ?json: its data; ?png: map as a picture
//   GET  /api/robot/help[/<name>]   the help
// A command the API can't take is answered 400 with "error: …"; a read that can't
// answer, 404 or 400 with why.
import { Hono } from "hono";
import { parseSteer, withSnapshots } from "./commands.ts";
import { answerLine, type Answer, type Sender } from "./answers.ts";
import { CHECKABLE, checkSteer } from "./check.ts";
import { help } from "./help.ts";
import { READS, ReadError, findSnapshot, mapOf, type ReadContext } from "./reads.ts";

const TEXT = { "content-type": "text/plain; charset=utf-8", "cache-control": "no-store" };
const text = (body: string, status = 200) => new Response(`${body.replace(/\n$/, "")}\n`, { status, headers: TEXT });

export function createApi(ctx: ReadContext): Hono {
  const app = new Hono().basePath("/api/robot");

  app.get("/help/:name?", c => {
    const h = help(c.req.param("name"));
    return h === null ? text(`no help for ${c.req.param("name")} (robot --help lists the commands)`, 404) : text(h);
  });

  app.post("/steer", async c => {
    const parsed = parseSteer(await c.req.text());
    if ("error" in parsed) return text(`error: ${parsed.error}`, 400);
    const commands = parsed.commands, sender: Sender = c.req.query("from") === "page" ? "page" : "cli";
    if (c.req.query("check") !== undefined) {
      if (commands.some(cmd => !CHECKABLE.has(cmd.name))) return text("error: --check takes only drive, rotate and goto", 400);
      const map = await mapOf(ctx.store, await findSnapshot(ctx.store, undefined));
      if (!map.view.robot) return text("error: the newest snapshot has no pose (PicoB not connected)", 404);
      return text(checkSteer(commands, map, map.view.robot).join("\n"));
    }
    const nowait = c.req.query("nowait") !== undefined;
    let open = true;
    const stream = new ReadableStream<string>({
      start(controller) {
        const write = (a: Answer) => { if (open) controller.enqueue(`${answerLine(a)}\n`); };
        const close = () => { if (open) { open = false; controller.close(); } };
        const finished = commands[0].name === "stop" ? ctx.queue.stop(sender, write)
          : ctx.queue.submit(withSnapshots(commands), sender, write);
        if (nowait) close(); // "queued" is written at once
        else void finished.then(close);
      },
      cancel() { open = false; }, // Ctrl-C: the commands stay queued
    });
    return new Response(stream.pipeThrough(new TextEncoderStream()), { headers: TEXT });
  });

  app.get("/:name/:arg?", async c => {
    const name = c.req.param("name"), read = READS[name];
    if (!read) return text(`${name}: not a command (robot --help lists them)`, 404);
    const flags = new Set(Object.keys(c.req.query()));
    try {
      const r = await read.run(ctx, c.req.param("arg"), flags);
      return flags.has("json") ? c.json(r.data as object) : text(r.text);
    } catch (err) {
      if (err instanceof ReadError) return text(err.message, err.status);
      throw err;
    }
  });

  return app;
}
