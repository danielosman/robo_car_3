// The robot's viewer and steering page (doc/ARCHITECTURE.md, the PC): the map as the robot
// had it at any snapshot (its window over the whole-house map's tiles), where it was
// and what it saw then (camera with features, ToF, colour, pose), the console, and
// steering: keys, the mode, and in manual mode a click on the live map sends the
// robot there (it turns and drives; the robot works out the way from its own pose).
// Map frame: x forward at power-up, y left; drawn with x up and y to the left, as
// the robot's `m` prints it.

const $ = id => document.getElementById(id);
const CELL = 0.1, TILE = 20;
const COLORS = { 1: "--blocked", 2: "--overhang", 3: "--nofloor", 4: "--drivable", 5: "--open" };
const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const columnColor = Object.fromEntries(Object.entries(COLORS).map(([k, v]) => [k, css(v)]));
const KEYS = [
  ["Actions", [["s", "scan"], ["f", "move 50 cm"], ["t", "turn 30°"], ["a", "watch 1 min"], ["r", "direction"]]],
  ["Robot", [["p", "status"], ["S", "snapshot"], ["C", "clear map"], ["R", "recording"], ["W", "WiFi"], ["h", "help"]]],
  ["Print", [["m", "map"], ["z", "ToF"], ["c", "camera"], ["o", "ToF backgr."], ["k", "camera backgr."]]],
];
const WEAK = 2; // of 15: columns set less confidently count as unknown when "hide weak" is ticked (~10 %, as replay)
const FOOT = { front: 0.095, back: -0.095, half: 0.1175 }; // the robot's footprint, m

// --- state ---
let ws = null, robotStatus = { state: "waiting" }, livePose = null, poseFresh = 0, goal = null;
let boots = [], boot = null, snaps = [], index = -1, detail = null, mapView = null;
let image = null;
const view = { cx: 0, cy: 0, scale: 80 }; // the map's centre (m) and pixels per metre
let follow = true;

// --- the map ---
const canvas = $("map"), ctx = canvas.getContext("2d");
function resize() {
  const r = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(r.width * dpr);
  canvas.height = Math.round(r.height * dpr);
  draw();
}
const toScreen = (x, y) => {
  const dpr = window.devicePixelRatio || 1;
  return [canvas.width / 2 - (y - view.cy) * view.scale * dpr, canvas.height / 2 - (x - view.cx) * view.scale * dpr];
};
const toWorld = (px, py) => { // CSS pixels in the canvas
  const r = canvas.getBoundingClientRect();
  return [view.cx - (py - r.height / 2) / view.scale, view.cy - (px - r.width / 2) / view.scale];
};

function cellRect(cx, cy) { // world cell -> screen rect (x, y, w, h)
  const [ax, ay] = toScreen((cx + 1) * CELL, (cy + 1) * CELL), [bx, by] = toScreen(cx * CELL, cy * CELL);
  return [Math.min(ax, bx), Math.min(ay, by), Math.abs(bx - ax) + 0.5, Math.abs(by - ay) + 0.5];
}

function drawRobot(x, y, yaw, color, alpha = 1) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const corner = (f, l) => toScreen(x + c * f - s * l, y + s * f + c * l);
  ctx.globalAlpha = alpha;
  ctx.beginPath();
  for (const [f, l] of [[FOOT.front, FOOT.half], [FOOT.front, -FOOT.half], [FOOT.back, -FOOT.half], [FOOT.back, FOOT.half]]) {
    const [px, py] = corner(f, l);
    ctx.lineTo(px, py);
  }
  ctx.closePath();
  ctx.fillStyle = color + "55";
  ctx.fill();
  ctx.lineWidth = 2;
  ctx.strokeStyle = color;
  ctx.stroke();
  const [hx, hy] = corner(FOOT.front + 0.08, 0), [ox, oy] = corner(0, 0);
  ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(hx, hy); ctx.stroke();
  ctx.globalAlpha = 1;
}

function draw() {
  const W = canvas.width, H = canvas.height, dpr = window.devicePixelRatio || 1;
  ctx.fillStyle = css("--unknown");
  ctx.fillRect(0, 0, W, H);
  // A metre grid, the origin (power-up point).
  ctx.strokeStyle = "#222";
  ctx.lineWidth = 1;
  const [x0, y0] = toWorld(canvas.clientWidth, canvas.clientHeight), [x1, y1] = toWorld(0, 0);
  for (let x = Math.floor(Math.min(x0, x1)); x <= Math.max(x0, x1); x++) { const [, py] = toScreen(x, 0); ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(W, py); ctx.stroke(); }
  for (let y = Math.floor(Math.min(y0, y1)); y <= Math.max(y0, y1); y++) { const [px] = toScreen(0, y); ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, H); ctx.stroke(); }

  if (mapView) {
    const w = mapView.window;
    const inWindow = (cx, cy) => w && cx >= w.orgX && cx < w.orgX + w.cellsX && cy >= w.orgY && cy < w.orgY + w.cellsY;
    const hide = $("hideweak").checked;
    const known = i => w.columns[i] && !(hide && w.confidence[i] < WEAK);
    // The whole-house map's columns: full where the window doesn't reach, dimmed under
    // the window where it doesn't know the column now.
    for (const t of mapView.tiles) {
      for (let ix = 0; ix < TILE; ix++) for (let iy = 0; iy < TILE; iy++) {
        const c = t.columns[ix * TILE + iy];
        if (!c) continue;
        const cx = t.tx * TILE - TILE / 2 + ix, cy = t.ty * TILE - TILE / 2 + iy;
        let alpha = 1;
        if (inWindow(cx, cy)) {
          if (known((cx - w.orgX) * w.cellsY + (cy - w.orgY))) continue;
          alpha = 0.35;
        }
        ctx.globalAlpha = alpha;
        ctx.fillStyle = columnColor[c];
        ctx.fillRect(...cellRect(cx, cy));
      }
    }
    ctx.globalAlpha = 1;
    if (w) {
      for (let ix = 0; ix < w.cellsX; ix++) for (let iy = 0; iy < w.cellsY; iy++) {
        const c = w.columns[ix * w.cellsY + iy];
        if (!known(ix * w.cellsY + iy)) continue;
        ctx.globalAlpha = 0.45 + 0.55 * Math.min(1, w.confidence[ix * w.cellsY + iy] / 8);
        ctx.fillStyle = columnColor[c];
        ctx.fillRect(...cellRect(w.orgX + ix, w.orgY + iy));
      }
      ctx.globalAlpha = 1;
      const [ax, ay] = toScreen((w.orgX + w.cellsX) * CELL, (w.orgY + w.cellsY) * CELL), [bx, by] = toScreen(w.orgX * CELL, w.orgY * CELL);
      ctx.setLineDash([6 * dpr, 5 * dpr]);
      ctx.strokeStyle = "#556";
      ctx.strokeRect(Math.min(ax, bx), Math.min(ay, by), Math.abs(bx - ax), Math.abs(by - ay));
      ctx.setLineDash([]);
    }
  }
  const [ox, oy] = toScreen(0, 0);
  ctx.strokeStyle = "#666";
  ctx.beginPath(); ctx.arc(ox, oy, 4 * dpr, 0, 2 * Math.PI); ctx.stroke();

  // The path: where each snapshot was taken, up to the one shown.
  const upto = snaps.slice(0, index + 1).filter(s => s.x !== null);
  ctx.strokeStyle = "#7aa7d988";
  ctx.lineWidth = 1.5 * dpr;
  ctx.beginPath();
  for (const s of upto) { const [px, py] = toScreen(s.x, s.y); ctx.lineTo(px, py); }
  ctx.stroke();
  for (const s of upto) {
    const [px, py] = toScreen(s.x, s.y);
    ctx.fillStyle = "#7aa7d9";
    ctx.beginPath(); ctx.arc(px, py, 2.5 * dpr, 0, 2 * Math.PI); ctx.fill();
  }

  // The ToF frame of the snapshot: its sure targets, by height.
  if (detail) {
    for (const t of detail.targets) {
      if (t.x === null || ![5, 6, 9].includes(Number(t.status))) continue;
      const z = Number(t.z), [px, py] = toScreen(Number(t.x), Number(t.y));
      ctx.fillStyle = z < 0.03 ? "#6c6" : z < 0.13 ? "#f55" : z < 0.23 ? "#fa4" : "#aaf";
      ctx.fillRect(px - 1.5 * dpr, py - 1.5 * dpr, 3 * dpr, 3 * dpr);
    }
    const s = detail.snapshot;
    if (s.x !== null) drawRobot(Number(s.x), Number(s.y), Number(s.yaw), "#7aa7d9", showingLive() ? 0.5 : 1);
  }
  if (livePose && showingLive()) drawRobot(livePose.x, livePose.y, livePose.yaw, "#5fe08a");
  if (goal) {
    const [gx, gy] = toScreen(goal.x, goal.y);
    ctx.strokeStyle = "#ffd75e";
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath(); ctx.arc(gx, gy, 9 * dpr, 0, 2 * Math.PI); ctx.moveTo(gx - 14 * dpr, gy); ctx.lineTo(gx + 14 * dpr, gy);
    ctx.moveTo(gx, gy - 14 * dpr); ctx.lineTo(gx, gy + 14 * dpr); ctx.stroke();
  }
}

// Panning by dragging, zooming with the wheel; a click (no drag) steers or picks a snapshot.
let drag = null;
canvas.addEventListener("pointerdown", e => { drag = { x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy, moved: false }; canvas.setPointerCapture(e.pointerId); });
canvas.addEventListener("pointermove", e => {
  const r = canvas.getBoundingClientRect(), [wx, wy] = toWorld(e.clientX - r.left, e.clientY - r.top);
  $("maphud").textContent = `x ${(wx * 100).toFixed(0)} cm, y ${(wy * 100).toFixed(0)} cm` + (canSteer() ? "  ·  click: go there" : "");
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
  if (Math.hypot(dx, dy) > 4) drag.moved = true;
  if (drag.moved) { view.cx = drag.cx + dy / view.scale; view.cy = drag.cy + dx / view.scale; follow = false; draw(); }
});
canvas.addEventListener("pointerup", e => {
  const d = drag;
  drag = null;
  if (!d || d.moved) return;
  const r = canvas.getBoundingClientRect(), [wx, wy] = toWorld(e.clientX - r.left, e.clientY - r.top);
  clicked(wx, wy);
});
canvas.addEventListener("wheel", e => {
  e.preventDefault();
  const r = canvas.getBoundingClientRect(), before = toWorld(e.clientX - r.left, e.clientY - r.top);
  view.scale = Math.min(800, Math.max(15, view.scale * Math.exp(-e.deltaY * 0.0015)));
  const after = toWorld(e.clientX - r.left, e.clientY - r.top);
  view.cx += before[0] - after[0];
  view.cy += before[1] - after[1];
  draw();
}, { passive: false });
$("hideweak").onchange = () => draw();
$("zoomin").onclick = () => { view.scale = Math.min(800, view.scale * 1.4); draw(); };
$("zoomout").onclick = () => { view.scale = Math.max(15, view.scale / 1.4); draw(); };
$("center").onclick = () => { follow = true; centre(); draw(); };
function centre() {
  const p = showingLive() && livePose ? livePose : detail?.snapshot;
  if (p && p.x !== null && p.x !== undefined) { view.cx = Number(p.x); view.cy = Number(p.y); }
}

const showingLive = () => $("follow").checked && index === snaps.length - 1;
const canSteer = () => robotStatus.state === "connected" && livePose?.mode === "manual" && showingLive();

function toast(text) {
  const t = $("toast");
  t.textContent = text;
  t.style.opacity = 1;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => { t.style.opacity = 0; }, 2500);
}

function clicked(x, y) {
  if (canSteer()) {
    goal = { x, y };
    send({ type: "goal", x, y });
    draw();
    return;
  }
  // Not steering: pick the nearest snapshot within 15 px.
  let best = -1, bestD = 15 / view.scale;
  snaps.forEach((s, i) => { if (s.x === null) return; const d = Math.hypot(s.x - x, s.y - y); if (d < bestD) { bestD = d; best = i; } });
  if (best >= 0) { select(best, false); return; }
  if (robotStatus.state !== "connected") toast("The robot isn't connected");
  else if (!showingLive()) toast("Showing an old snapshot: tick live to steer");
  else toast("Steering by clicking needs Manual mode");
}

// --- the side panels ---
function showSnapshot() {
  const s = detail?.snapshot;
  if (!s) return;
  const ended = s.action ? `${s.action} ${s.end_reason === "done" ? "done" : `ended: ${s.end_reason}`}` : "";
  $("snaphead").innerHTML = `<b>#${s.seq}</b> of boot ${s.boot_id} · ${s.arrived} · ${s.why}` +
    (ended ? `<br>last action: ${ended}` : "") + (s.running ? `<br>running: ${s.running}` : "") + `<br>mode ${s.mode}`;
  // The camera picture, twice its size, and the features on it.
  const cam = $("camera"), c = cam.getContext("2d");
  c.imageSmoothingEnabled = false;
  c.fillStyle = "#000";
  c.fillRect(0, 0, cam.width, cam.height);
  const nf = (detail.features ?? []).filter(f => !(f.flags & 4)).length;
  $("caminfo").textContent = s.image ? `frame ${s.camera_frame_no}, ${Number(s.exposure_ms).toFixed(0)} ms × ${Number(s.gain).toFixed(1)}, ${nf} features` : "no picture";
  if (s.image) {
    const img = new Image();
    image = img;
    img.onload = () => {
      if (image !== img) return;
      c.drawImage(img, 0, 0, cam.width, cam.height);
      drawFeatures(c, cam.width / img.width);
    };
    img.src = `/snapshots/${s.boot_id}/${s.seq}.png`;
  }
  // The ToF grid as the robot sees it.
  const rows = [];
  for (const row of detail.grid) {
    const cells = row.map(z => {
      if (!z) return `<td class="none">--</td>`;
      if (!z.sure) return `<td class="unsure" title="status ${z.status}">?${z.status}</td>`;
      const hue = Math.max(0, Math.min(240, z.mm / 3000 * 240));
      return `<td style="background:hsl(${hue} 70% 32%)" title="${z.mm} mm, status ${z.status}">${Math.round(z.mm / 10)}</td>`;
    });
    rows.push(`<tr>${cells.join("")}</tr>`);
  }
  $("tof").innerHTML = rows.join("");
  // The colour reading.
  const kv = (pairs) => pairs.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
  if (s.lux !== null && s.lux !== undefined) {
    $("color").innerHTML = kv([["lux", Number(s.lux).toFixed(1)], ["CIE x, y", `${Number(s.cie_x).toFixed(3)}, ${Number(s.cie_y).toFixed(3)}`]]);
    $("swatch").style.background = swatch(Number(s.color_x), Number(s.color_y), Number(s.color_z));
  } else {
    $("color").innerHTML = kv([["", s.color_x === undefined ? "no reading" : "no valid reading"]]);
    $("swatch").style.background = "transparent";
  }
  const deg = r => (Number(r) * 180 / Math.PI).toFixed(1);
  $("pose").innerHTML = s.x === null ? kv([["", "PicoB not connected"]]) : kv([
    ["x, y", `${(s.x * 100).toFixed(1)}, ${(s.y * 100).toFixed(1)} cm`], ["yaw", `${deg(s.yaw)}°`],
    ["pitch, roll", `${deg(s.pitch)}°, ${deg(s.roll)}°`], ["", `${s.stationary ? "still" : "moving"}, motors ${s.motors_on ? "on" : "off"}`],
    ["map window", `from (${s.map_org_cx * 10}, ${s.map_org_cy * 10}) cm`], ["tiles sent", s.tiles],
  ]);
}

function drawFeatures(c, k) {
  if (!$("showfeatures").checked || !detail?.features) return;
  for (const f of detail.features) {
    if (f.flags & 4) continue; // lost
    // New green, young yellow, older red; missed (not found in this frame) grey.
    c.strokeStyle = f.flags & 2 ? "#888" : f.age === 0 ? "#5fe08a" : f.age < 10 ? "#ffd75e" : "#ff6b5e";
    c.lineWidth = 1.5;
    c.strokeRect(f.x * k - 3, f.y * k - 3, 6, 6);
    if (f.dx || f.dy) { c.beginPath(); c.moveTo(f.x * k, f.y * k); c.lineTo((f.x - f.dx) * k, (f.y - f.dy) * k); c.stroke(); }
  }
}
$("showfeatures").onchange = () => showSnapshot();

// XYZ (any scale) as a display colour of fixed brightness (sRGB, D65).
function swatch(X, Y, Z) {
  if (!(Y > 0)) return "transparent";
  const k = 0.5 / Y;
  const lin = [3.2406 * X - 1.5372 * Y - 0.4986 * Z, -0.9689 * X + 1.8758 * Y + 0.0415 * Z, 0.0557 * X - 0.204 * Y + 1.057 * Z].map(v => v * k);
  const max = Math.max(1, ...lin);
  const g = lin.map(v => Math.max(0, v / max)).map(v => v <= 0.0031308 ? 12.92 * v : 1.055 * v ** (1 / 2.4) - 0.055);
  return `rgb(${g.map(v => Math.round(v * 255)).join(",")})`;
}

// --- loading ---
async function getJson(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url}: ${r.status}`);
  return r.json();
}

async function loadBoots(prefer) {
  boots = await getJson("/api/boots");
  $("boot").innerHTML = boots.map(b => `<option value="${b.boot_id}">${b.boot_id} · ${b.first} · ${b.snapshots}</option>`).join("");
  const want = prefer ?? boot ?? boots[0]?.boot_id;
  if (want) { $("boot").value = want; await loadSnapshots(want); }
}
$("boot").onchange = () => { $("follow").checked = false; loadSnapshots($("boot").value); };

async function loadSnapshots(b) {
  boot = b;
  snaps = (await getJson(`/api/snapshots?boot=${b}`)).map(s => ({ ...s, x: s.x === null ? null : Number(s.x), y: Number(s.y), seq: Number(s.seq) }));
  $("timeline").max = Math.max(0, snaps.length - 1);
  await select($("follow").checked || index < 0 || index >= snaps.length ? snaps.length - 1 : index, false);
}

let loading = 0;
async function select(i, keepFollow = true) {
  if (!snaps.length) return;
  index = Math.max(0, Math.min(snaps.length - 1, i));
  if (!keepFollow) $("follow").checked = index === snaps.length - 1 && $("follow").checked;
  $("timeline").value = index;
  const s = snaps[index], mine = ++loading;
  const [d, m] = await Promise.all([getJson(`/api/snapshots/${boot}/${s.seq}`), getJson(`/api/snapshots/${boot}/${s.seq}/map`)]);
  if (mine !== loading) return;
  detail = d;
  mapView = m;
  if (follow) centre();
  showSnapshot();
  draw();
}
$("timeline").oninput = () => { $("follow").checked = false; select(Number($("timeline").value), true); };
$("prev").onclick = () => { $("follow").checked = false; select(index - 1, true); };
$("next").onclick = () => select(index + 1, true);
$("follow").onchange = () => { if ($("follow").checked) { follow = true; select(snaps.length - 1); } else draw(); };

// --- the robot: status, live pose, mode, keys, console ---
function send(msg) { if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg)); }
const sendKey = key => send({ type: "key", key });

$("stop").onclick = () => { sendKey(" "); goal = null; draw(); };
$("idle").onclick = () => { send({ type: "mode", mode: "idle" }); goal = null; draw(); };
$("manual").onclick = () => send({ type: "mode", mode: "manual" });

const buttons = [$("stop"), $("idle"), $("manual")];
for (const [group, keys] of KEYS) {
  const label = document.createElement("span");
  label.textContent = group;
  $("keys").append(label);
  for (const [key, what] of keys) {
    const b = document.createElement("button");
    b.innerHTML = `<kbd>${key}</kbd>`;
    b.append(what);
    b.onclick = () => sendKey(key);
    buttons.push(b);
    $("keys").append(b);
  }
}
const logToggle = document.createElement("button");
logToggle.textContent = "console ▾";
logToggle.onclick = () => { $("footer").classList.toggle("hidden"); logToggle.textContent = $("footer").classList.contains("hidden") ? "console ▸" : "console ▾"; resize(); };
$("keys").append(logToggle);
const known = new Set([" ", "M", ...KEYS.flatMap(([, keys]) => keys.map(([k]) => k))]);
document.addEventListener("keydown", e => {
  if (e.metaKey || e.ctrlKey || e.altKey || ["INPUT", "SELECT"].includes(e.target.tagName)) return;
  if (!known.has(e.key)) return;
  e.preventDefault();
  if (e.key === " ") { goal = null; draw(); }
  sendKey(e.key);
});

const MAX_CHARS = 512 * 1024;
let chars = 0;
function append(text, kind) {
  const log = $("log"), atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 30;
  const node = kind === "note" ? Object.assign(document.createElement("span"), { className: "note", textContent: text }) : document.createTextNode(text);
  log.append(node);
  chars += text.length;
  while (chars > MAX_CHARS && log.firstChild) { chars -= log.firstChild.textContent.length; log.firstChild.remove(); }
  if (atBottom) log.scrollTop = log.scrollHeight;
  // An action that ended: its goal is done.
  if (goal && /^(Go to (done|stopped)|Stopped|Go to: already there|Goal .* ignored)/m.test(text)) { goal = null; draw(); }
}

function showStatus() {
  const el = $("status"), s = robotStatus;
  el.className = `pill ${s.state}`;
  el.textContent = s.state === "waiting" ? "Waiting for robot…" : s.state === "connected"
    ? `Robot ${s.ip}${s.rssi !== null ? `, ${s.rssi} dBm` : ""}` : `Robot lost at ${new Date(s.at).toLocaleTimeString()}`;
  for (const b of buttons) b.disabled = s.state !== "connected";
  showLive();
}
function showLive() {
  const p = livePose, el = $("live");
  const fresh = p && robotStatus.state === "connected" && poseFresh > 0;
  el.textContent = fresh ? `${p.mode}${p.action ? ` · ${p.action}` : ""} · (${(p.x * 100).toFixed(0)}, ${(p.y * 100).toFixed(0)}) cm` : "";
  el.style.display = fresh ? "" : "none";
  $("manual").classList.toggle("on", !!fresh && p.mode === "manual");
  $("idle").classList.toggle("on", !!fresh && p.mode === "idle");
}
setInterval(() => { if (poseFresh > 0) poseFresh--; showLive(); }, 1000); // stale after ~3 s without a pose

function connect() {
  ws = new WebSocket(`ws://${location.host}/`);
  ws.onmessage = e => {
    const msg = JSON.parse(e.data);
    if (msg.type === "backlog") { $("log").textContent = ""; chars = 0; for (const it of msg.items) append(it.text, it.kind); }
    else if (msg.type === "text" || msg.type === "note") append(msg.text, msg.type);
    else if (msg.type === "status") { robotStatus = msg.status; showStatus(); }
    else if (msg.type === "pose") {
      livePose = msg.pose;
      poseFresh = 3;
      showLive();
      if (follow && showingLive()) centre();
      if (showingLive()) draw();
    } else if (msg.type === "snapshot") {
      if (msg.boot !== boot) { if ($("follow").checked || !boot) loadBoots(msg.boot); }
      else if ($("follow").checked) loadSnapshots(boot);
      else getJson(`/api/snapshots?boot=${boot}`).then(list => { snaps = list.map(s => ({ ...s, x: s.x === null ? null : Number(s.x), y: Number(s.y), seq: Number(s.seq) })); $("timeline").max = snaps.length - 1; draw(); });
    }
  };
  ws.onclose = () => {
    robotStatus = { state: "lost", at: Date.now() };
    $("status").className = "pill lost";
    $("status").textContent = "Server not reachable (is npm start running in pc/robot?)";
    for (const b of buttons) b.disabled = true;
    setTimeout(connect, 2000);
  };
}

window.addEventListener("resize", resize);
resize();
connect();
loadBoots().catch(err => append(`could not load the snapshots: ${err.message}\n`, "note"));
