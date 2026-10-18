// Tests for session.ts: the rules of one detection, with a scripted model. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import type { Chat, ChatMessage, ChatReply } from "./model.ts";
import { detectObjects, promptFor } from "./session.ts";
import { LABELS } from "./labels.ts";

const PNG = Buffer.from("not really a png");
const GEMMA = promptFor("gemma-4-26b"), QWEN = promptFor("qwen-3.8-27b");

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
  const { chat, seen } = scripted([[{ label: "sofa", bbox: [70, 0, 460, 1000], confidence: 70 }]]);
  const r = await detectObjects(chat, PNG, { thinking: true, reasoningBudget: null, prompt: GEMMA });
  assert.equal(r.result, "ok");
  assert.deepEqual(r.result === "ok" && r.detections, [{ label: "sofa", bbox: [70, 0, 460, 1000], confidence: 70 }]);
  assert.equal(r.turns, 1);
  const user = seen[0][1].content as { type: string }[];
  assert.equal(user[1].type, "image_url");
  assert.ok(!JSON.stringify(r.transcript).includes("base64"), "the transcript leaves the picture out");
});

test("a wrong label goes back with the list; the corrected call is accepted", async () => {
  const { chat, seen } = scripted([[{ label: "pouf", bbox: [100, 250, 535, 670], confidence: 70 }], [{ label: "object", bbox: [100, 250, 535, 670], confidence: 70 }]]);
  const r = await detectObjects(chat, PNG, { thinking: false, reasoningBudget: null, prompt: GEMMA });
  assert.equal(r.result, "ok");
  assert.equal(r.turns, 2);
  assert.match(lastTool(seen[1]), /label "pouf" is not one of: chair, table.*Use object if nothing fits/);
});

test("still wrong after 3 corrections: failed", async () => {
  const bad = [{ label: "pouf", bbox: [1, 2, 3, 4], confidence: 70 }];
  const r = await detectObjects(scripted([bad, bad, bad, bad]).chat, PNG, { thinking: true, reasoningBudget: null, prompt: GEMMA });
  assert.equal(r.result, "failed");
  assert.match(r.result === "failed" ? r.why : "", /after 3 corrections/);
  assert.equal(r.turns, 4);
});

test("an empty list is accepted at once", async () => {
  const r = await detectObjects(scripted([[]]).chat, PNG, { thinking: true, reasoningBudget: null, prompt: GEMMA });
  assert.equal(r.result, "ok");
  assert.deepEqual(r.result === "ok" && r.detections, []);
  assert.equal(r.turns, 1);
});

test("no tool call: reminded up to 3 times, then failed", async () => {
  const { chat, seen } = scripted(["I see a sofa.", [{ label: "sofa", bbox: [70, 0, 460, 1000], confidence: 70 }]]);
  assert.equal((await detectObjects(chat, PNG, { thinking: true, reasoningBudget: null, prompt: GEMMA })).result, "ok");
  assert.match(String(seen[1].at(-1)!.content), /Call report_objects now/);
  const r = await detectObjects(scripted(["a", "b", "c", "d"]).chat, PNG, { thinking: true, reasoningBudget: null, prompt: GEMMA });
  assert.equal(r.result, "failed");
  assert.match(r.result === "failed" ? r.why : "", /didn't call report_objects after 3 reminders/);
});

test("both system prompts list every label", () => {
  for (const p of [GEMMA, QWEN]) for (const label of LABELS) assert.match(p.text, new RegExp(`- \`${label}\``));
});

test("the prompt is picked by the model's name; another model is an error", () => {
  assert.equal(GEMMA.box, "y_first_1000");
  assert.equal(QWEN.box, "x_first_pixels");
  assert.equal(promptFor("Qwen-3.8-27b").file, "system_prompt_qwen.md");
  assert.throws(() => promptFor("llama-4"), /no system prompt for model llama-4/);
  assert.doesNotMatch(QWEN.text, /ymin|xmin|y first|x first|1000/);
});

test("Qwen's boxes come x first in pixels and are kept y first on 0-1000", async () => {
  const { chat, seen } = scripted([[{ label: "chair", bbox: [67.2, 37.2, 92.8, 78.6], confidence: 80 }]]);
  const r = await detectObjects(chat, PNG, { thinking: true, reasoningBudget: null, prompt: QWEN });
  assert.deepEqual(r.result === "ok" && r.detections, [{ label: "chair", bbox: [310, 420, 655, 580], confidence: 80 }]);
  assert.equal(seen[0][0].content, QWEN.text);
});

test("a wrong Qwen box gets a correction that doesn't name the order", async () => {
  const { chat, seen } = scripted([[{ label: "chair", bbox: [93, 37, 67, 79], confidence: 80 }],
                                   [{ label: "chair", bbox: [161, 37, 167, 79], confidence: 80 }],
                                   [{ label: "chair", bbox: [67, 37, 93, 79], confidence: 80 }]]);
  assert.equal((await detectObjects(chat, PNG, { thinking: true, reasoningBudget: null, prompt: QWEN })).result, "ok");
  assert.match(lastTool(seen[1]), /its left left of its right/);
  assert.doesNotMatch(lastTool(seen[1]), /ymin|xmin/);
  assert.match(lastTool(seen[2]), /inside the picture \(160 × 120 pixels\)/);
});
