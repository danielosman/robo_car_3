// One detection session (doc/OBJECTS_PLAN.md, "What robot detect does"): the camera
// picture goes to the vision model with the system prompt, and the model answers by
// calling report_objects. Every call is checked (labels.ts); what is wrong goes back
// to the model so it can correct itself. An empty list is accepted at once.
// A turn without the tool call gets a reminder. At most 3 corrections and 3 reminders,
// then the session fails and says why. The whole conversation is kept (without the
// picture) for looking at later.
import { readFileSync } from "node:fs";
import { LABELS, checkReport, type Detection } from "./labels.ts";
import type { Chat, ChatMessage } from "./model.ts";

export const PROMPT_VERSION = 5;  // 2: door and doorway, no wall, confidence; 3: what to look for; 4: person, animal,
                                  // report everything; 5: rewritten in markdown (system_prompt.md)
const MAX_CORRECTIONS = 3, MAX_REMINDERS = 3;

// The system prompt is in system_prompt.md, to read and edit as it is.
export const SYSTEM_PROMPT = readFileSync(new URL("./system_prompt.md", import.meta.url), "utf8");

export const TOOLS = [{
  type: "function",
  function: {
    name: "report_objects",
    description: "Reports every object found in the picture, all in one call. An empty list means nothing was found.",
    parameters: {
      type: "object", required: ["objects"],
      properties: { objects: { type: "array", items: { type: "object", required: ["label", "bbox", "confidence"], properties: {
        label: { type: "string", description: `One of: ${LABELS.join(", ")}` },
        bbox: { type: "array", items: { type: "number" }, description: "[ymin, xmin, ymax, xmax] on a 0-1000 grid, y first" },
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

export async function detectObjects(chat: Chat, png: Buffer, opts: { thinking: boolean }): Promise<SessionResult> {
  const image = `data:image/png;base64,${png.toString("base64")}`;
  const messages: ChatMessage[] = [
    { role: "system", content: SYSTEM_PROMPT },
    { role: "user", content: [{ type: "text", text: "Find the objects in this picture." }, { type: "image_url", image_url: { url: image } }] },
  ];
  const transcript: Turn[] = [{ role: "system", text: SYSTEM_PROMPT }, { role: "user", text: "Find the objects in this picture. [the picture]" }];
  const say = (m: ChatMessage, reasoning?: string | null) => {
    messages.push(m);
    transcript.push({ role: m.role, text: typeof m.content === "string" ? m.content : "", ...(m.tool_calls ? { tool_calls: m.tool_calls } : {}),
                      ...(reasoning ? { reasoning } : {}) });
  };
  let corrections = 0, reminders = 0, turns = 0;

  for (;;) {
    const reply = await chat(messages, TOOLS, opts);
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
        const checked = checkReport(args);
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
