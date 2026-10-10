// The recordings in 3D (doc/TELEMETRY_PLAN.md §4): a take's targets where they were in
// the room, the robot's path, the robot and its rays at a moment, the replayed map as
// cubes (READINGS, VOTES or where they differ), and truth boxes with each map's score.
// World frame: odometry's from power-up, x forward at power-up, y left, z up (m).
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const $ = id => document.getElementById(id);
const VALID = new Set([5, 6, 9]);
const COLUMN = { BLOCKED: 1, OVERHANG: 2 };
const G_BOTTOM = -0.07, LAYER = 0.1;
const ROW_COLOURS = [0xff5d5d, 0xffa34d, 0xffe14d, 0x9be15d, 0x4dd6c1, 0x4da6ff, 0x9d7bff, 0xff7bd5];

// --- the scene ---
const view = $("view");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
view.append(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
const camera = new THREE.PerspectiveCamera(55, 1, 0.01, 60);
camera.up.set(0, 0, 1);
camera.position.set(-1.8, 0.0, 2.0);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0.3, 0, 0);
controls.update();
scene.add(new THREE.AmbientLight(0xffffff, 1));

function grid() {
  const fine = [], coarse = [];
  for (let i = -30; i <= 30; i++) {
    const v = i / 10, list = i % 10 === 0 ? coarse : fine;
    list.push(v, -3, 0, v, 3, 0, -3, v, 0, 3, v, 0);
  }
  for (const [pts, colour] of [[fine, 0x1e1e1e], [coarse, 0x3a3a3a]]) {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
    scene.add(new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color: colour })));
  }
  scene.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.001), 0.4, 0x888888, 0.06, 0.04));
}
grid();

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);
resize();
renderer.setAnimationLoop(() => renderer.render(scene, camera));

// --- state ---
let data = null, maps = null, truth = [], frameIndex = 0, playing = null;
let pointsObj = null, shown = [], mapObj = new THREE.Group(), robotObj = new THREE.Group(), pathObj = null, truthObj = new THREE.Group();
scene.add(mapObj, robotObj, truthObj);

for (let r = 1; r <= 8; r++) {
  const l = document.createElement("label");
  l.innerHTML = `<input type="checkbox" data-row="${r}" checked> ${r}`;
  $("rows").append(l);
}

// --- colours ---
function ramp(t) { // 0..1: blue, cyan, green, yellow, red
  const c = new THREE.Color();
  c.setHSL((1 - Math.min(1, Math.max(0, t))) * 0.66, 0.9, 0.55);
  return c;
}
function colourOf(p, i, mode) {
  if (mode === "row") return new THREE.Color(ROW_COLOURS[p.row[i] - 1]);
  if (mode === "signal") return ramp(Math.log10(Math.max(1, p.signal[i])) / 3.5);
  if (mode === "height") return ramp((p.z[i] + 0.05) / 0.45);
  if (mode === "target") return new THREE.Color([0xffffff, 0x4dd6ff, 0xff6bd5, 0xffa34d][p.target[i]] ?? 0x888888);
  const s = p.status[i];
  return new THREE.Color(s === 5 ? 0x5ddc6a : VALID.has(s) ? 0xb6f09a : s === 4 ? 0xffa34d : s >= 12 ? 0xff5d5d : 0x888888);
}

// --- points ---
function frameNo(i = frameIndex) { return data.frames.frame_no[i]; }

function buildPoints() {
  if (!data) return;
  if (pointsObj) { scene.remove(pointsObj); pointsObj.geometry.dispose(); }
  const p = data.points, rows = new Set([...document.querySelectorAll("[data-row]")].filter(c => c.checked).map(c => +c.dataset.row));
  const valid = $("valid").checked, closest = $("closest").checked, mode = $("colour").value, frames = $("frames").value;
  const now = frameNo();
  shown = [];
  for (let i = 0; i < p.x.length; i++) {
    if (!rows.has(p.row[i]) || (valid && !VALID.has(p.status[i])) || (closest && p.target[i] !== 0)) continue;
    if (frames === "upto" && p.frame_no[i] > now) continue;
    if (frames === "one" && p.frame_no[i] !== now) continue;
    shown.push(i);
  }
  const pos = new Float32Array(shown.length * 3), col = new Float32Array(shown.length * 3);
  shown.forEach((i, k) => {
    pos.set([p.x[i], p.y[i], p.z[i]], 3 * k);
    const c = colourOf(p, i, mode);
    col.set([c.r, c.g, c.b], 3 * k);
  });
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  g.setAttribute("color", new THREE.BufferAttribute(col, 3));
  pointsObj = new THREE.Points(g, new THREE.PointsMaterial({ size: 0.004 * +$("size").value, vertexColors: true }));
  scene.add(pointsObj);
  buildRobot();
  status();
}

// --- the map ---
const cube = new THREE.BoxGeometry(0.095, 0.095, 0.095), tile = new THREE.BoxGeometry(0.095, 0.095, 0.004);
function instanced(geometry, colour, opacity, cells) { // cells: [x, y, z]
  const m = new THREE.InstancedMesh(geometry, new THREE.MeshBasicMaterial({ color: colour, transparent: true, opacity, depthWrite: false }), cells.length);
  const t = new THREE.Matrix4();
  cells.forEach(([x, y, z], k) => m.setMatrixAt(k, t.makeTranslation(x, y, z)));
  return m;
}
const L1_Z = G_BOTTOM + 1.5 * LAYER, L2_Z = G_BOTTOM + 2.5 * LAYER;
const solid = c => c[2] === COLUMN.BLOCKED || c[2] === COLUMN.OVERHANG;

function buildMap() {
  mapObj.clear();
  if (!maps) return;
  const variant = $("variant").value;
  if (variant === "none") return status();
  if (variant === "diff") {
    const key = c => `${c[0]},${c[1]}`;
    const r = new Map(maps.readings.filter(solid).map(c => [key(c), c])), v = new Map(maps.votes.filter(solid).map(c => [key(c), c]));
    const z = c => (c[4] === 2 ? L1_Z : L2_Z);
    mapObj.add(instanced(cube, 0x40c0ff, 0.55, [...r].filter(([k]) => !v.has(k)).map(([, c]) => [c[0], c[1], z(c)])));
    mapObj.add(instanced(cube, 0xc060ff, 0.55, [...v].filter(([k]) => !r.has(k)).map(([, c]) => [c[0], c[1], z(c)])));
    return status();
  }
  const cells = maps[variant];
  if ($("l1").checked) mapObj.add(instanced(cube, 0xe04040, 0.4, cells.filter(c => c[4] === 2).map(c => [c[0], c[1], L1_Z])));
  if ($("l2").checked) mapObj.add(instanced(cube, 0xf0a030, 0.35, cells.filter(c => c[5] === 2).map(c => [c[0], c[1], L2_Z])));
  if ($("floor").checked) mapObj.add(instanced(tile, 0x2e9d42, 0.35, cells.filter(c => c[3] === 2).map(c => [c[0], c[1], 0])));
  if ($("nofloor").checked) mapObj.add(instanced(tile, 0x777777, 0.4, cells.filter(c => c[3] === 1).map(c => [c[0], c[1], 0])));
  status();
}

async function loadMaps() {
  maps = null;
  mapObj.clear();
  status("replaying the map…");
  const res = await fetch(`/api/takes/${data.take.take_id}/maps?margin=${$("margin").value}`);
  maps = await res.json();
  if (!res.ok) { status(`map: ${maps.error}`); maps = null; return; }
  buildMap();
  loadScore();
}

// --- the robot at the slider's moment, its rays, its path ---
function buildRobot() {
  robotObj.clear();
  if (!data || !$("rays").checked || !data.frames.x.length) return;
  const f = data.frames, i = frameIndex, c = Math.cos(f.yaw[i]), s = Math.sin(f.yaw[i]);
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.16, 0.1), new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.25 }));
  body.position.set(f.x[i], f.y[i], 0.05);
  body.rotation.z = f.yaw[i];
  robotObj.add(body);
  const g = data.take.geometry ?? { sensorM: [0.025, -0.03, 0.07] };
  const o = [f.x[i] + c * g.sensorM[0] - s * g.sensorM[1], f.y[i] + s * g.sensorM[0] + c * g.sensorM[1], g.sensorM[2]];
  // A ray to each zone's closest target among the points shown (a farther target lies
  // along the same line).
  const now = frameNo(i), p = data.points, pos = [], col = [];
  const mode = $("colour").value;
  for (const k of shown) {
    if (p.frame_no[k] !== now || p.target[k] !== 0) continue;
    pos.push(...o, p.x[k], p.y[k], p.z[k]);
    const cc = colourOf(p, k, mode);
    col.push(cc.r, cc.g, cc.b, cc.r, cc.g, cc.b);
  }
  const lg = new THREE.BufferGeometry();
  lg.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
  lg.setAttribute("color", new THREE.Float32BufferAttribute(col, 3));
  robotObj.add(new THREE.LineSegments(lg, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.5 })));
  $("time").textContent = `t ${f.t_s[i].toFixed(2)} s · frame ${now}${f.zones[i] < 64 ? " (half)" : ""}`;
}

function buildPath() {
  if (pathObj) scene.remove(pathObj);
  const o = data.odom, pts = [];
  for (let i = 0; i < o.x.length; i++) pts.push(o.x[i], o.y[i], 0.003);
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
  pathObj = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0xaaaaaa }));
  scene.add(pathObj);
}

// --- truth and score ---
async function loadTruth() {
  const res = await fetch(`/api/truth?boot=${data.take.boot_id}`);
  truth = res.ok ? await res.json() : [];
  truthObj.clear();
  const table = $("truth");
  table.innerHTML = truth.length ? "<tr><th>name</th><th>x, y (m)</th><th>size (cm)</th><th></th></tr>" : "";
  for (const t of truth) {
    const box = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(t.size_x, t.size_y, t.size_z)),
      new THREE.LineBasicMaterial({ color: 0xffe14d }));
    box.position.set(t.x, t.y, t.z + t.size_z / 2);
    truthObj.add(box);
    const tr = table.insertRow();
    tr.innerHTML = `<td>${t.name}</td><td>${t.x.toFixed(2)}, ${t.y.toFixed(2)}</td>
      <td>${Math.round(t.size_x * 100)}×${Math.round(t.size_y * 100)}×${Math.round(t.size_z * 100)}</td><td><button>×</button></td>`;
    tr.querySelector("button").onclick = async () => {
      await fetch(`/api/truth/${t.truth_id}`, { method: "DELETE" });
      await loadTruth();
      loadScore();
    };
  }
}

async function loadScore() {
  if (!truth.length) { $("score").innerHTML = "<small>place truth boxes to score the maps</small>"; return; }
  const res = await fetch(`/api/takes/${data.take.take_id}/score?margin=${$("margin").value}`);
  const s = await res.json();
  if (!res.ok) { $("score").textContent = s.error; return; }
  const cell = o => `<td class="${o.found ? "yes" : "no"}">${o.found ? `yes (${o.cells})` : "no"}</td>`;
  let html = "<table><tr><th></th><th>READINGS</th><th>VOTES</th></tr>";
  s.readings.objects.forEach((o, k) => { html += `<tr><td>${o.name}</td>${cell(o)}${cell(s.votes.objects[k])}</tr>`; });
  html += `<tr><td>not explained</td><td>${s.readings.unexplained} of ${s.readings.solid}</td><td>${s.votes.unexplained} of ${s.votes.solid}</td></tr></table>
    <small>found: a blocked or overhang cell on the box (cells); not explained: such cells within 1.5 m of the start on no box</small>`;
  $("score").innerHTML = html;
}

$("tadd").onclick = async () => {
  if (!data) return;
  await fetch("/api/truth", { method: "POST", body: JSON.stringify({
    boot_id: data.take.boot_id, name: $("tname").value, x: +$("tx").value, y: +$("ty").value, z: 0,
    size_x: +$("tsx").value, size_y: +$("tsy").value, size_z: +$("tsz").value }) });
  await loadTruth();
  loadScore();
};

// --- picking: click a point for its data; shift-click the floor for truth x, y ---
const raycaster = new THREE.Raycaster();
let down = null;
renderer.domElement.addEventListener("pointerdown", e => { down = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener("pointerup", e => {
  if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4 || !data) return;
  const r = renderer.domElement.getBoundingClientRect();
  raycaster.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), camera);
  if (e.shiftKey) {
    const hit = new THREE.Vector3();
    if (raycaster.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), 0), hit)) {
      $("tx").value = hit.x.toFixed(2);
      $("ty").value = hit.y.toFixed(2);
    }
    return;
  }
  raycaster.params.Points.threshold = 0.012;
  const hits = pointsObj ? raycaster.intersectObject(pointsObj) : [];
  if (!hits.length) return;
  const i = shown[hits[0].index], p = data.points;
  const f = data.frames.frame_no.indexOf(p.frame_no[i]);
  $("info").textContent = `row ${p.row[i]}, column ${p.col[i]}, target ${p.target[i] + 1}\n` +
    `distance ${p.distance_mm[i]} mm, signal ${p.signal[i]}, status ${p.status[i]}\n` +
    `x ${p.x[i].toFixed(3)}, y ${p.y[i].toFixed(3)}, z ${(p.z[i] * 100).toFixed(1)} cm\n` +
    `frame ${p.frame_no[i]}${f >= 0 ? `, t ${data.frames.t_s[f].toFixed(2)} s` : ""}`;
  if (f >= 0) { frameIndex = f; $("slider").value = f; sliderChanged(); }
});

// --- time ---
function sliderChanged() {
  frameIndex = +$("slider").value;
  if ($("frames").value === "all") buildRobot(); else buildPoints();
}
$("slider").oninput = sliderChanged;
$("play").onclick = () => {
  if (playing) { clearInterval(playing); playing = null; $("play").textContent = "play"; return; }
  $("play").textContent = "pause";
  playing = setInterval(() => {
    if (frameIndex >= data.frames.x.length - 1) { $("play").click(); return; }
    $("slider").value = frameIndex + 1;
    sliderChanged();
  }, 1000 / 15);
};

// --- loading a take ---
function status(text) {
  if (text !== undefined) { $("status").textContent = text; return; }
  if (!data) return;
  const t = data.take, solidN = maps && $("variant").value !== "none" && $("variant").value !== "diff" ? maps[$("variant").value].filter(solid).length : null;
  $("status").textContent = `${t.take_id} · ${t.key} ${t.action} · ${t.duration_s} s · ${t.started}\n` +
    `${shown.length} of ${data.points.x.length} points shown` + (solidN !== null ? ` · ${solidN} blocked or overhang columns` : "") +
    ($("variant").value === "diff" ? "\ndifference: blue only READINGS, purple only VOTES" : "");
}

async function load(id) {
  status("loading…");
  if (playing) $("play").click();
  const res = await fetch(`/api/takes/${id}/view`);
  const d = await res.json();
  if (!res.ok) { status(d.error); return; }
  data = d;
  history.replaceState(null, "", `?take=${id}`);
  frameIndex = 0;
  $("slider").max = Math.max(0, data.frames.x.length - 1);
  $("slider").value = 0;
  $("takeinfo").innerHTML = `<small>${data.take.note ? `${data.take.note} · ` : ""}boot ${data.take.boot_id}, ended ${data.take.end_reason} ·
    <a href="/takes" target="_blank">all takes</a></small>`;
  buildPath();
  buildPoints();
  await loadTruth();
  loadMaps();
}

async function start() {
  const takes = await (await fetch("/api/takes")).json();
  for (const t of takes) {
    const o = document.createElement("option");
    o.value = t.take_id;
    o.textContent = `${t.started} · ${t.take_id} · ${t.key} ${t.action}${t.note ? ` · ${t.note}` : ""}`;
    $("take").append(o);
  }
  const wanted = new URLSearchParams(location.search).get("take");
  const id = takes.some(t => t.take_id === wanted) ? wanted : takes[0]?.take_id;
  if (!id) { status("no takes yet: record one (R, then an action)"); return; }
  $("take").value = id;
  load(id);
}

$("take").onchange = () => load($("take").value);
for (const id of ["colour", "valid", "closest", "frames", "size"]) $(id).oninput = buildPoints;
$("rows").oninput = buildPoints;
for (const id of ["variant", "l1", "l2", "floor", "nofloor"]) $(id).oninput = buildMap;
$("margin").onchange = loadMaps;
$("rays").oninput = buildRobot;
start();
