// The objects a detection may name (doc/OBJECTS_PLAN.md): a fixed list, with
// `object` for anything else, and the check the model's report_objects call goes
// through. Its error messages go back to the model as they are, so they say how to
// fix the call.

// door: the part that swings; doorway: the opening in the wall (doc/OBJECTS_PLAN.md).
export const LABELS = ["chair", "table", "sofa", "door", "doorway", "plant", "shelf", "cabinet", "box", "person", "animal", "object"] as const;
export type Label = typeof LABELS[number];

// How deep an object is behind its front edge, m: the camera sees only the front.
// First guesses, to tune. Doors and doorways are lines (no depth).
export const DEPTH_M: Record<Label, number | null> = {
  chair: 0.5, table: 0.8, sofa: 0.9, door: null, doorway: null, plant: 0.4, shelf: 0.35, cabinet: 0.45, box: 0.3, person: 0.3, animal: 0.3, object: 0.2,
};

// THE BOX FORMAT, everywhere in the code and the database: [ymin, xmin, ymax, xmax],
// Y FIRST, on a 0-1000 grid over the picture (0, 0 the top left, 1000, 1000 the bottom
// right), whatever the picture's size in pixels. It is Gemma's own format, kept as it
// is. Other models' boxes are turned into it as they arrive (checkReport below: Qwen
// gives x first in pixels). Only place.ts turns it into pixels, then into the map's
// metres.
// confidence: how sure the model is of it, 0-100 %.
export interface Detection { label: Label; bbox: [number, number, number, number]; confidence: number }

export type Checked = { ok: true; detections: Detection[] } | { ok: false; errors: string[] };

// Checks report_objects' arguments: {"objects": [{"label": ..., "bbox": [ymin, xmin, ymax, xmax], "confidence": 0-100}, ...]}.
// How a model gives its boxes. y_first_1000: [ymin, xmin, ymax, xmax] on 0-1000 (Gemma,
// and what is kept). x_first_pixels: [xmin, ymin, xmax, ymax] in the picture's pixels
// (Qwen), turned into the kept form; its messages don't name the order (the Qwen
// prompt shows it only by its example).
export type BoxFormat = "y_first_1000" | "x_first_pixels";
export const PICTURE_PX = { width: 160, height: 120 };

export function checkReport(args: unknown, format: BoxFormat = "y_first_1000"): Checked {
  const xFirst = format === "x_first_pixels";
  const box = xFirst ? "[...]" : "[ymin, xmin, ymax, xmax]";
  const shape = `report_objects takes {"objects": [{"label": "...", "bbox": ${box}, "confidence": 0-100}, ...]}`;
  const objects = (args as { objects?: unknown } | null)?.objects;
  if (typeof args !== "object" || args === null || !Array.isArray(objects)) return { ok: false, errors: [shape] };
  const errors: string[] = [], detections: Detection[] = [];
  objects.forEach((o, i) => {
    const at = `objects[${i}]`;
    const { label, bbox: given, confidence } = (o ?? {}) as { label?: unknown; bbox?: unknown; confidence?: unknown };
    const { width: w, height: h } = PICTURE_PX;
    const bbox = xFirst && Array.isArray(given) && given.length === 4 && given.every(v => typeof v === "number")
      ? [given[1] * 1000 / h, given[0] * 1000 / w, given[3] * 1000 / h, given[2] * 1000 / w] : given;
    const wrong: string[] = [];
    if (!LABELS.includes(label as Label)) {
      wrong.push(`label ${JSON.stringify(label)} is not one of: ${LABELS.join(", ")}. Use object if nothing fits.`);
    }
    if (!Array.isArray(bbox) || bbox.length !== 4 || !bbox.every(v => typeof v === "number" && Number.isFinite(v))) {
      wrong.push(xFirst ? "bbox must be four numbers, as in the example" : "bbox must be four numbers [ymin, xmin, ymax, xmax]");
    } else if (bbox.some(v => v < 0 || v > 1000)) {
      wrong.push(xFirst ? `bbox must be inside the picture (${w} × ${h} pixels)` : "bbox values must be 0-1000");
    } else if (bbox[0] >= bbox[2] || bbox[1] >= bbox[3]) {
      wrong.push(xFirst ? "bbox must have its top above its bottom and its left left of its right" : "bbox must have ymin < ymax and xmin < xmax");
    }
    if (typeof confidence !== "number" || !Number.isFinite(confidence) || confidence < 0 || confidence > 100) {
      wrong.push("confidence must be a number 0-100 (how sure you are, in %)");
    }
    if (wrong.length) errors.push(...wrong.map(w => `${at}: ${w}`));
    else detections.push({ label: label as Label, bbox: (bbox as number[]).map(Math.round) as Detection["bbox"], confidence: Math.round(confidence as number) });
  });
  return errors.length ? { ok: false, errors } : { ok: true, detections };
}
