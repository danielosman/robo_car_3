import {zoneValue, sourceZone, statusNames} from './tof-view.js';
const el = id => document.getElementById(id);
const cv = el('cv'), ctx = cv.getContext('2d');
el('rotation').value = '90'; // PCB mounting orientation; reset on page startup.
let ws, image, lastCamera, lastTof, cameraAt = 0, tofAt = 0, serialConnected = false;
let lastSequence, lastTofArrival, observedHz = 0;
let lastOpt, optAt = 0, lastOptArrival, optHz = 0;
const lines = [];
function log(text) {
  lines.push(text); if (lines.length > 80) lines.shift();
  el('log').textContent = lines.join('\n'); el('log').scrollTop = el('log').scrollHeight;
  if (text.startsWith('TOF')) el('tofMessage').textContent = text;
}
function send(data) {
  if (ws?.readyState === WebSocket.OPEN && serialConnected) ws.send(data);
  else log('Not sent: serial connection unavailable');
}
function connect() {
  ws = new WebSocket(`ws://${location.host}`); ws.binaryType = 'arraybuffer';
  ws.onopen = () => { el('connection').textContent = 'WebSocket connected; checking serial…'; };
  ws.onclose = () => {
    serialConnected = false;
    el('connection').textContent = 'Disconnected — reconnecting…';
    setTimeout(connect, 1000);
  };
  ws.onerror = () => { el('connection').textContent = 'WebSocket error'; };
  ws.onmessage = event => {
    if (typeof event.data !== 'string') {
      const bytes = new Uint8Array(event.data);
      if (bytes.length < 4) return;
      const w = bytes[0] | bytes[1] << 8, h = bytes[2] | bytes[3] << 8;
      if (!w || !h || bytes.length !== 4 + w * h) return;
      lastCamera = {w, h, pixels: bytes.subarray(4)}; cameraAt = performance.now(); drawCamera();
      return;
    }
    const msg = JSON.parse(event.data);
    if (msg.type === 'log') log(msg.text);
    if (msg.type === 'connection') {
      serialConnected = msg.connected;
      el('connection').textContent = msg.connected ? `Connected to ${msg.port}` : `Serial ${msg.port} disconnected — restart host after reconnecting`;
    }
    if (msg.type === 'cameraSettings') showCameraSettings(msg);
    if (msg.type === 'tof') {
      const now = performance.now();
      if (lastTofArrival && msg.sequence !== lastSequence) observedHz = 1000 / (now - lastTofArrival);
      lastSequence = msg.sequence; lastTofArrival = now;
      lastTof = msg; tofAt = now; drawTof();
    }
    if (msg.type === 'opt') {
      const now = performance.now();
      if (lastOptArrival) optHz = 1000 / (now - lastOptArrival);
      lastOptArrival = now; lastOpt = msg; optAt = now; drawOpt();
    }
  };
}
function drawCamera() {
  if (!lastCamera) return;
  const {w, h, pixels} = lastCamera;
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; image = null; }
  image ??= ctx.createImageData(w, h);
  let lo = 0, hi = 255;
  if (el('stretch').checked) {
    lo = 255; hi = 0;
    for (const v of pixels) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
    if (hi <= lo) { lo = 0; hi = 255; }
  }
  const gamma = 1 / Number(el('gamma').value);
  for (let i = 0; i < pixels.length; i++) {
    const value = Math.pow(Math.max(0, Math.min(1, (pixels[i] - lo) / (hi - lo))), gamma) * 255;
    const o = i * 4; image.data[o] = image.data[o + 1] = image.data[o + 2] = value; image.data[o + 3] = 255;
  }
  ctx.putImageData(image, 0, 0);
}
function drawTof() {
  if (!lastTof) return;
  const {side, zones} = lastTof;
  const metric = el('metric').value, selection = el('target').value, validity = el('validity').value;
  const values = zones.map(z => zoneValue(z, metric, selection, validity));
  const scale = metric === 'distance' ? 4000 : Math.max(1, ...values.map(v => v.value ?? 0));
  const grid = el('grid'); grid.style.gridTemplateColumns = `repeat(${side}, 1fr)`;
  grid.style.gridTemplateRows = `repeat(${side}, 1fr)`;
  grid.replaceChildren();
  for (let row = 0; row < side; row++) for (let col = 0; col < side; col++) {
    const index = sourceZone(row, col, side, Number(el('rotation').value));
    const zone = zones[index], {value, target} = values[index];
    const cell = document.createElement('div'); cell.className = 'cell';
    const number = document.createElement('span'); number.textContent = value == null ? '—' : String(value);
    const sub = document.createElement('small'); sub.textContent = `z${index}` + (target ? ` · s${target.status}` : '');
    cell.append(number, sub);
    if (value != null) cell.style.background = `hsl(${Math.max(0, Math.min(1, value / scale)) * 240} 55% 25%)`;
    if (target && target.status !== 5) cell.classList.add('warn');
    cell.title = `Zone ${index}: ${zone.count} reported target(s), ambient ${zone.ambient}, SPADs ${zone.spads}\n` +
      zone.targets.slice(0, Math.min(zone.count, zone.targets.length)).map((t, i) =>
        `Slot ${i + 1}: ${t.distance}mm, status ${t.status} (${statusNames[t.status] ?? 'Unknown'}), sigma ${t.sigma}mm, signal ${t.signal}, reflectance ${t.reflectance}%`).join('\n');
    grid.append(cell);
  }
}
// Display-only color: CIE xy at Y=1 -> linear sRGB (D65), brightest channel = 1.
function xyToCss({x, y}) {
  const X = x / y, Z = (1 - x - y) / y;
  const rgb = [3.2406*X - 1.5372 - 0.4986*Z, -0.9689*X + 1.8758 + 0.0415*Z, 0.0557*X - 0.2040 + 1.0570*Z]
    .map(v => Math.max(0, v));
  const max = Math.max(...rgb) || 1;
  const [r, g, b] = rgb.map(v => { v /= max; return Math.round(255 * (v <= 0.0031308 ? 12.92*v : 1.055*v ** (1/2.4) - 0.055)); });
  return `rgb(${r} ${g} ${b})`;
}
function drawOpt() {
  if (!lastOpt) return;
  const o = lastOpt, fmt = v => v.toFixed(v < 10 ? 3 : v < 1000 ? 1 : 0);
  el('lux').textContent = o.lux == null ? '—' : fmt(o.lux);
  el('optXy').textContent = o.chromaticity ? `${o.chromaticity.x.toFixed(4)}, ${o.chromaticity.y.toFixed(4)}` : '—';
  el('optXyz').textContent = o.xyz ? [o.xyz.X, o.xyz.Y, o.xyz.Z].map(fmt).join(' / ') : '—';
  el('swatch').style.background = o.chromaticity ? xyToCss(o.chromaticity) : '#000';
  o.channels.forEach((c, i) => {
    const cell = el(`optCh${i}`);
    cell.textContent = `${c.adc} (e${c.exponent})`;
    cell.className = c.crcOk && c.rangeOk && !o.overload ? '' : 'error';
  });
}
function updateStatus() {
  const now = performance.now();
  if (lastCamera) {
    const age = (now - cameraAt) / 1000;
    el('cameraStatus').textContent = `${lastCamera.w}×${lastCamera.h} · last frame ${age.toFixed(1)}s ago${age > 2 ? ' — stopped / stale' : ''}`;
  }
  if (lastTof) {
    const t = lastTof, age = (now - tofAt) / 1000;
    el('tofStatus').textContent = `${t.side}×${t.side} · configured ${t.hz}Hz / received ~${observedHz.toFixed(1)}Hz · ` +
      `${t.temperature}°C · ${t.order === 1 ? 'closest' : 'strongest'} first · ` +
      `${t.mode === 1 ? 'continuous' : `autonomous ${t.integration}ms`} · sharpener ${t.sharpener}% · ` +
      `last frame ${age.toFixed(1)}s ago${age > Math.max(2, 2 / t.hz) ? ' — stopped / stale' : ''}`;
  }
  if (lastOpt) {
    const o = lastOpt, age = (now - optAt) / 1000;
    el('optStatus').textContent = `#${o.sequence} · ~${optHz.toFixed(1)}Hz · ${o.conversionMs}ms/channel · ` +
      `${o.overload ? 'OVERLOAD' : o.valid ? 'valid' : 'invalid (CRC/range)'} · ` +
      `last sample ${age.toFixed(1)}s ago${age > 2 ? ' — stopped / stale' : ''}`;
  }
}
el('gamma').oninput = () => { el('gammaV').textContent = Number(el('gamma').value).toFixed(2); drawCamera(); };
el('stretch').oninput = drawCamera;
function showCameraSettings(s) {
  el('exposure').textContent = `${s.exposureMs.toFixed(1)} ms · gain ×${s.gain.toFixed(2)} · ${s.fps.toFixed(1)} frames/s · ` +
    `brightness ${s.brightness} · ${s.held ? 'held' : 'adjusting'}${s.settling ? ' (changing)' : ''}`;
  el('hold').checked = s.held;
}
el('hold').onchange = () => send(el('hold').checked ? 'H hold\n' : 'H release\n');
for (const id of ['metric', 'target', 'validity', 'rotation']) el(id).onchange = drawTof;
el('resolution').onchange = () => {
  el('hz').max = el('resolution').value === '64' ? '15' : '60';
  el('hz').value = Math.min(Number(el('hz').value), Number(el('hz').max));
};
el('ranging').onchange = () => { el('integration').disabled = el('ranging').value === '1'; };
el('applyTof').onclick = () => {
  const fields = ['resolution', 'hz', 'order', 'ranging', 'integration', 'sharpener'];
  const [res, hz, order, mode, ms, sharp] = fields.map(id => Number(el(id).value));
  if (![res, hz, order, mode, ms, sharp].every(Number.isInteger) ||
      ![16, 64].includes(res) || hz < 1 || hz > (res === 64 ? 15 : 60) ||
      ms < 2 || ms > 1000 || sharp < 0 || sharp > 99) {
    log('TOF invalid settings: check resolution, rate, integration and sharpener'); return;
  }
  if (mode === 3 && ((res === 64 ? 4 : 1) * ms + 1) * hz >= 1000) {
    log('TOF integration too long: (integration × subframes + 1ms) must be less than 1000/Hz'); return;
  }
  send(`T ${res} ${hz} ${order} ${mode} ${ms} ${sharp}\n`);
  el('tofMessage').textContent = 'Settings requested; active settings appear above when new data arrives.';
};
el('startTof').onclick = () => send('T start\n'); el('stopTof').onclick = () => send('T stop\n');
el('initTof').onclick = () => { send('T init\n'); el('tofMessage').textContent = 'Reinitialization requested…'; };
el('initOpt').onclick = () => send('O init\n');
setInterval(updateStatus, 250);
connect();
