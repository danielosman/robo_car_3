// Tests for session.ts: the rules of one detection, with a scripted model. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import type { Chat, ChatMessage, ChatReply } from "./model.ts";
import { detectObjects } from "./session.ts";

const PNG = Buffer.from("not really a png");

// A model that answers from a script: each entry a tool call's objects, or text (no call).
function scripted(script: (object[] | string)[]) {
  const seen: ChatMessage[][] = [];
  const chat: Chat = async messages => {
    seen.push(messages.map(m => ({ ...m })));
    const next = script.shift();
    if (next === undefined) throw new Error("the script ran out");
    const reply: ChatReply = { reasoning: null, tokens: 1, message: typeof next === "string" ? { role: "assistant", content: next }
      : { role: "assistant", content: "", tool_calls: [{ id: `c${seen.length}`, type: "function",
                                                         function: { name: "report_objects", arguments: JSON.stringify({ objects: next }) } }] } };
    return reply;
  };
  return { chat, seen };
}
const lastTool = (m: ChatMessage[]) => String(m.filter(x => x.role === "tool").at(-1)?.content);

test("a good call is accepted at once; the picture goes with the first message", async () => {
  const { chat, seen } = scripted([[{ label: "sofa", bbox: [70, 0, 460, 1000] }]]);
  const r = await detectObjects(chat, PNG, { thinking: true });
  assert.equal(r.result, "ok");
  assert.deepEqual(r.result === "ok" && r.detections, [{ label: "sofa", bbox: [70, 0, 460, 1000] }]);
  assert.equal(r.turns, 1);
  const user = seen[0][1].content as { type: string }[];
  assert.equal(user[1].type, "image_url");
  assert.ok(!JSON.stringify(r.transcript).includes("base64"), "the transcript leaves the picture out");
});

test("a wrong label goes back with the list; the corrected call is accepted", async () => {
  const { chat, seen } = scripted([[{ label: "pouf", bbox: [100, 250, 535, 670] }], [{ label: "object", bbox: [100, 250, 535, 670] }]]);
  const r = await detectObjects(chat, PNG, { thinking: false });
  assert.equal(r.result, "ok");
  assert.equal(r.turns, 2);
  assert.match(lastTool(seen[1]), /label "pouf" is not one of: chair, table.*Use object if nothing fits/);
});

test("still wrong after 3 corrections: failed", async () => {
  const bad = [{ label: "pouf", bbox: [1, 2, 3, 4] }];
  const r = await detectObjects(scripted([bad, bad, bad, bad]).chat, PNG, { thinking: true });
  assert.equal(r.result, "failed");
  assert.match(r.result === "failed" ? r.why : "", /after 3 corrections/);
  assert.equal(r.turns, 4);
});

test("the first empty list is pushed back once; the second is accepted", async () => {
  const { chat, seen } = scripted([[], []]);
  const r = await detectObjects(chat, PNG, { thinking: true });
  assert.equal(r.result, "ok");
  assert.deepEqual(r.result === "ok" && r.detections, []);
  assert.match(lastTool(seen[1]), /An empty list is unusual/);
  const found = await detectObjects(scripted([[], [{ label: "door", bbox: [0, 0, 600, 300] }]]).chat, PNG, { thinking: true });
  assert.equal(found.result === "ok" && found.detections.length, 1);
});

test("no tool call: reminded up to 3 times, then failed", async () => {
  const { chat, seen } = scripted(["I see a sofa.", [{ label: "sofa", bbox: [70, 0, 460, 1000] }]]);
  assert.equal((await detectObjects(chat, PNG, { thinking: true })).result, "ok");
  assert.match(String(seen[1].at(-1)!.content), /Call report_objects now/);
  const r = await detectObjects(scripted(["a", "b", "c", "d"]).chat, PNG, { thinking: true });
  assert.equal(r.result, "failed");
  assert.match(r.result === "failed" ? r.why : "", /didn't call report_objects after 3 reminders/);
});
