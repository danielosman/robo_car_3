// The take list: one row per take from /api/takes; the note is editable.
const COLUMNS = [
  ["started", "started", "l"], ["take_id", "take", "l"], ["key", "key", "l"], ["action", "action", "l"],
  ["duration_s", "s"], ["end_reason", "ended", "l"], ["frames_whole", "frames"], ["frames_half", "half"],
  ["frames_missing", "missing"], ["datagrams_lost", "dgram lost"], ["odom_reports", "odom"], ["note", "note", "l"],
];
const table = document.getElementById("takes");

async function load() {
  const res = await fetch("/api/takes");
  const rows = await res.json();
  if (!res.ok) { table.textContent = rows.error; return; }
  table.textContent = "";
  const head = table.insertRow();
  for (const [, title, cls] of COLUMNS) {
    const th = document.createElement("th");
    th.textContent = title;
    if (cls) th.className = cls;
    head.append(th);
  }
  let lastBoot = null;
  for (const r of rows) {
    const tr = table.insertRow();
    if (lastBoot !== null && r.boot_id !== lastBoot) tr.className = "boot";
    lastBoot = r.boot_id;
    for (const [key, , cls] of COLUMNS) {
      const td = tr.insertCell();
      td.textContent = r[key] ?? "";
      if (cls) td.className = cls;
      if (key === "end_reason" && r[key] === "cut") td.classList.add("cut");
      if ((key === "frames_missing" || key === "datagrams_lost") && Number(r[key]) > 0) td.classList.add("bad");
      if (key === "note") {
        td.className = "note";
        td.onclick = async () => {
          const note = prompt(`Note for ${r.take_id} (${r.key} ${r.action})`, r.note ?? "");
          if (note === null) return;
          await fetch(`/api/takes/${r.take_id}/note`, { method: "POST", body: note });
          load();
        };
      }
    }
  }
}
document.getElementById("reload").onclick = load;
load();
