// JSON for people and agents to read: an object one key per line; a matrix one row per
// line with its columns lined up; a list of strings or objects one item per line;
// everything else compact on its line.
const compact = (v: unknown) => JSON.stringify(v);
const isScalar = (v: unknown) => v === null || typeof v !== "object";

function value(v: unknown, indent: string): string {
  if (!Array.isArray(v) || v.length === 0) return compact(v);
  const inner = `${indent}  `;
  if (v.every(row => Array.isArray(row) && row.every(isScalar))) { // a matrix
    const cells = (v as unknown[][]).map(row => row.map(compact)), width = Math.max(...cells.flat().map(c => c.length));
    return `[\n${cells.map(row => `${inner}[${row.map(c => c.padStart(width)).join(", ")}]`).join(",\n")}\n${indent}]`;
  }
  if (v.every(isScalar) && !v.every(x => typeof x === "string")) return compact(v);
  return `[\n${v.map(x => `${inner}${compact(x)}`).join(",\n")}\n${indent}]`;
}

export function formatJson(v: unknown): string {
  if (isScalar(v) || Array.isArray(v)) return value(v, "");
  const entries = Object.entries(v as object).filter(([, x]) => x !== undefined);
  return `{\n${entries.map(([k, x]) => `  ${compact(k)}: ${value(x, "  ")}`).join(",\n")}\n}`;
}
