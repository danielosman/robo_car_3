// One detection session (doc/OBJECTS_PLAN.md, "What robot detect does"): the camera
// picture goes to the vision model with the system prompt, and the model answers by
// calling report_objects. Every call is checked (labels.ts); what is wrong goes back
// to the model so it can correct itself. The first empty list is pushed back once.
// A turn without the tool call gets a reminder. At most 3 corrections and 3 reminders,
// then the session fails and says why. The whole conversation is kept (without the
// picture) for looking at later.
import { LABELS, checkReport, type Detection } from "./labels.ts";
import type { Chat, ChatMessage } from "./model.ts";

export const PROMPT_VERSION = 1;
const MAX_CORRECTIONS = 3, MAX_REMINDERS = 3;

export const SYSTEM_PROMPT = `You find objects in pictures from a small robot's camera, for a map of the flat it drives in.

The camera: 160 x 120 pixels, greyscale, 8.8 cm above the floor, looking straight ahead. The floor fills the bottom of the picture; furniture looks tall and is often cut off at the top.

Find every object that stands on the floor and report each one with the report_objects tool, once, with all of them in one call. Labels: ${LABELS.join(", ")}. Use object when nothing else fits; don't stretch a label (a pouf is an object, not a box).

Each object's bbox is [ymin, xmin, ymax, xmax] on a 0-1000 grid: 0,0 is the top left corner, 1000,1000 the bottom right; y first. Example: {"label": "chair", "bbox": [310, 420, 655, 580]}.

The bottom edge (ymax) matters most. It is used to work out how far away the object is, so put it exactly where the object touches the floor: the lowest point of its legs, base or feet, as precisely as you can, to the pixel. If the object's front doesn't reach the floor (a sofa or cabinet with a gap or a recessed base underneath), put ymax where its legs or base meet the floor, not at its lower front edge. The left and right edges give its width and direction; the top edge matters little.

One box per object (a chair's legs are one chair). An object cut off by the picture's edge: box the part that is seen. A wall or door: the box's bottom edge along where it meets the floor.

If there really is nothing, call report_objects with an empty list.`;

export const TOOLS = [{
  type: "function",
  function: {
    name: "report_objects",
    description: "Reports every object found in the picture, all in one call. An empty list means nothing was found.",
    parameters: {
      type: "object", required: ["objects"],
      properties: { objects: { type: "array", items: { type: "object", required: ["label", "bbox"], properties: {
        label: { type: "string", description: `One of: ${LABELS.join(", ")}` },
        bbox: { type: "array", items: { type: "number" }, description: "[ymin, xmin, ymax, xmax] on a 0-1000 grid, y first" },
      } } } },
    },
  },
}];

const EMPTY_PUSHBACK = "An empty list is unusual: a room in a flat almost always has something in view (furniture, a wall, " +
  "a door). Look again and report what you see. If there really is nothing, report the empty list again.";
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
  let corrections = 0, reminders = 0, pushedBack = false, turns = 0;

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
        else if (!checked.detections.length && !pushedBack) { answer = EMPTY_PUSHBACK; pushedBack = true; }
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
