// The objects a detection may name (doc/OBJECTS_PLAN.md): a fixed list, with
// `object` for anything else, and the check the model's report_objects call goes
// through. Its error messages go back to the model as they are, so they say how to
// fix the call.

export const LABELS = ["chair", "table", "sofa", "door", "wall", "plant", "shelf", "cabinet", "box", "object"] as const;
export type Label = typeof LABELS[number];

// How deep an object is behind its front edge, m: the camera sees only the front.
// First guesses, to tune. Doors and walls are lines (no depth).
export const DEPTH_M: Record<Label, number | null> = {
  chair: 0.5, table: 0.8, sofa: 0.9, door: null, wall: null, plant: 0.4, shelf: 0.35, cabinet: 0.45, box: 0.3, object: 0.2,
};

// One box as the model gives it: [ymin, xmin, ymax, xmax] on a 0-1000 grid, y first.
export interface Detection { label: Label; bbox: [number, number, number, number] }

export type Checked = { ok: true; detections: Detection[] } | { ok: false; errors: string[] };

// Checks report_objects' arguments: {"objects": [{"label": ..., "bbox": [ymin, xmin, ymax, xmax]}, ...]}.
export function checkReport(args: unknown): Checked {
  const shape = 'report_objects takes {"objects": [{"label": "...", "bbox": [ymin, xmin, ymax, xmax]}, ...]}';
  const objects = (args as { objects?: unknown } | null)?.objects;
  if (typeof args !== "object" || args === null || !Array.isArray(objects)) return { ok: false, errors: [shape] };
  const errors: string[] = [], detections: Detection[] = [];
  objects.forEach((o, i) => {
    const at = `objects[${i}]`;
    const { label, bbox } = (o ?? {}) as { label?: unknown; bbox?: unknown };
    const wrong: string[] = [];
    if (!LABELS.includes(label as Label)) {
      wrong.push(`label ${JSON.stringify(label)} is not one of: ${LABELS.join(", ")}. Use object if nothing fits.`);
    }
    if (!Array.isArray(bbox) || bbox.length !== 4 || !bbox.every(v => typeof v === "number" && Number.isFinite(v))) {
      wrong.push("bbox must be four numbers [ymin, xmin, ymax, xmax]");
    } else if (bbox.some(v => v < 0 || v > 1000)) {
      wrong.push("bbox values must be 0-1000");
    } else if (bbox[0] >= bbox[2] || bbox[1] >= bbox[3]) {
      wrong.push("bbox must have ymin < ymax and xmin < xmax");
    }
    if (wrong.length) errors.push(...wrong.map(w => `${at}: ${w}`));
    else detections.push({ label: label as Label, bbox: (bbox as number[]).map(Math.round) as Detection["bbox"] });
  });
  return errors.length ? { ok: false, errors } : { ok: true, detections };
}
