// One detection session (doc/OBJECTS_PLAN.md, "What robot detect does"): the camera
// picture goes to the vision model with the system prompt, and the model answers by
// calling report_objects. Every call is checked (labels.ts); what is wrong goes back
// to the model so it can correct itself. An empty list is accepted at once.
// A turn without the tool call gets a reminder. At most 3 corrections and 3 reminders,
// then the session fails and says why. The whole conversation is kept (without the
// picture) for looking at later.
import { readFileSync } from "node:fs";
import { LABELS, checkReport, type BoxFormat, type Detection } from "./labels.ts";
import type { Chat, ChatMessage } from "./model.ts";

export const PROMPT_VERSION = 5;  // 2: door and doorway, no wall, confidence; 3: what to look for; 4: person, animal,
                                  // report everything; 5: rewritten in markdown (system_prompt_gemma4.md,
                                  // and system_prompt_qwen.md, with the model)
const MAX_CORRECTIONS = 3, MAX_REMINDERS = 3;

// A system prompt per model family, picked by the model's name (MODEL in .env), each in
// its own file to read and edit as it is. Gemma gives boxes y first on a 0-1000 grid;
// Qwen x first in the picture's pixels (160 × 120), the order shown only in its
// prompt's example. Qwen's boxes are turned y first on 0-1000 as they arrive
// (labels.ts), so everything after sees them as Gemma's.
export interface Prompt { file: string; text: string; box: BoxFormat; bboxHelp: string }
const PROMPTS: Record<string, Omit<Prompt, "text">> = {
  gemma: { file: "system_prompt_gemma4.md", box: "y_first_1000", bboxHelp: "[ymin, xmin, ymax, xmax] on a 0-1000 grid, y first" },
  qwen: { file: "system_prompt_qwen.md", box: "x_first_pixels", bboxHelp: "The object's box in the picture's pixels" },
};

export function promptFor(model: string): Prompt {
  const family = Object.keys(PROMPTS).find(f => model.toLowerCase().startsWith(f));
  if (!family) throw new Error(`no system prompt for model ${model}: its name must start with ${Object.keys(PROMPTS).join(" or ")}`);
  const p = PROMPTS[family];
  return { ...p, text: readFileSync(new URL(`./${p.file}`, import.meta.url), "utf8") };
}

const toolsFor = (prompt: Prompt) => [{
  type: "function",
  function: {
    name: "report_objects",
    description: "Reports every object found in the picture, all in one call. An empty list means nothing was found.",
    parameters: {
      type: "object", required: ["objects"],
      properties: { objects: { type: "array", items: { type: "object", required: ["label", "bbox", "confidence"], properties: {
        label: { type: "string", description: `One of: ${LABELS.join(", ")}` },
        bbox: { type: "array", items: { type: "number" }, description: prompt.bboxHelp },
        confidence: { type: "number", description: "How sure you are of this object, 0-100 (%)" },
      } } } },
    },
  },
}];

const REMINDER = "Call report_objects now with the objects you found (an empty list if there is nothing). Don't answer in text.";

export interface Turn { role: ChatMessage["role"]; text: string; tool_calls?: unknown; reasoning?: string | null }
export type SessionResult =
  | { result: "ok"; detections: Detection[]; turns: number; transcript: Turn[] }
  | { result: "failed"; why: string; turns: number; transcript: Turn[] };

export async function detectObjects(chat: Chat, png: Buffer, opts: { thinking: boolean; reasoningBudget: number | null; prompt: Prompt }): Promise<SessionResult> {
  const { prompt, thinking, reasoningBudget } = opts, tools = toolsFor(prompt);
  const image = `data:image/png;base64,${png.toString("base64")}`;
  const messages: ChatMessage[] = [
    { role: "system", content: prompt.text },
    { role: "user", content: [{ type: "text", text: "Find the objects in this picture." }, { type: "image_url", image_url: { url: image } }] },
  ];
  const transcript: Turn[] = [{ role: "system", text: prompt.text }, { role: "user", text: "Find the objects in this picture. [the picture]" }];
  const say = (m: ChatMessage, reasoning?: string | null) => {
    messages.push(m);
    transcript.push({ role: m.role, text: typeof m.content === "string" ? m.content : "", ...(m.tool_calls ? { tool_calls: m.tool_calls } : {}),
                      ...(reasoning ? { reasoning } : {}) });
  };
  let corrections = 0, reminders = 0, turns = 0;

  for (;;) {
    const reply = await chat(messages, tools, { thinking, reasoningBudget });
    turns++;
    say(reply.message, reply.reasoning);
    const calls = reply.message.tool_calls ?? [];
    if (!calls.length) {
      if (++reminders > MAX_REMINDERS) return { result: "failed", why: `the model didn't call report_objects after ${MAX_REMINDERS} reminders`, turns, transcript };
      say({ role: "user", content: REMINDER });
      continue;
    }
    let accepted: Detection[] | null = null, wrong = false;
    for (const call of calls) {
      let answer: string;
      if (call.function.name !== "report_objects") {
        answer = `There is no tool ${call.function.name}: use report_objects.`;
        wrong = true;
      } else {
        let args: unknown = null;
        try { args = JSON.parse(call.function.arguments); } catch { /* checked below */ }
        const checked = checkReport(args, prompt.box);
        if (!checked.ok) { answer = `Not accepted. Fix these and call report_objects again:\n${checked.errors.join("\n")}`; wrong = true; }
        else { answer = `Accepted: ${checked.detections.length} objects.`; accepted = checked.detections; }
      }
      say({ role: "tool", tool_call_id: call.id, content: answer });
    }
    if (accepted) return { result: "ok", detections: accepted, turns, transcript };
    if (wrong && ++corrections > MAX_CORRECTIONS) {
      return { result: "failed", why: `report_objects was still wrong after ${MAX_CORRECTIONS} corrections`, turns, transcript };
    }
  }
}
