// The robot's console in the browser: the log, the connection status, and the
// serial monitor's keys as buttons (typing them works too).
const KEYS = [
  ["Robot", [["g", "motors on"], ["s", "stop"], ["p", "status"], ["l", "link"], ["w", "WiFi"], ["h", "help"]]],
  ["Map", [["n", "scan"], ["m", "map"], ["z", "ToF frame"]]],
  ["Watch", [["a", "watch"], ["v", "movement log"], ["o", "ToF backgrounds"], ["k", "camera backgrounds"], ["c", "camera frame"]]],
  ["Tests", [["q", "square"], ["d", "drift"], ["r", "turns"], ["f", "forward"], ["b", "back"], ["t", "last result"]]],
];
const MAX_CHARS = 1024 * 1024;
const log = document.getElementById("log");
const statusEl = document.getElementById("status");
const serverEl = document.getElementById("server");
const follow = document.getElementById("follow");
const buttons = [];
let ws = null, chars = 0;

for (const [group, keys] of KEYS) {
  const label = document.createElement("span");
  label.textContent = group;
  document.getElementById("keys").append(label);
  for (const [key, what] of keys) {
    const b = document.createElement("button");
    b.innerHTML = `<kbd>${key}</kbd>`;
    b.append(what);
    if (key === "s") b.className = "stop";
    b.onclick = () => sendKey(key);
    buttons.push(b);
    document.getElementById("keys").append(b);
  }
}
const known = new Set(KEYS.flatMap(([, keys]) => keys.map(([k]) => k)));
document.addEventListener("keydown", e => {
  if (e.metaKey || e.ctrlKey || e.altKey || e.target.tagName === "INPUT") return;
  if (known.has(e.key)) { e.preventDefault(); sendKey(e.key); }
});
document.getElementById("clear").onclick = () => { log.textContent = ""; chars = 0; };

function sendKey(key) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ type: "key", key }));
}

function append(text, kind) {
  const atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 30;
  const node = kind === "note" ? Object.assign(document.createElement("span"), { className: "note", textContent: text })
                               : document.createTextNode(text);
  log.append(node);
  chars += text.length;
  while (chars > MAX_CHARS && log.firstChild) { chars -= log.firstChild.textContent.length; log.firstChild.remove(); }
  if (follow.checked && atBottom) log.scrollTop = log.scrollHeight;
}

const time = ms => new Date(ms).toLocaleTimeString();
function showStatus(s) {
  statusEl.className = s.state;
  if (s.state === "waiting") statusEl.textContent = "Waiting for robot…";
  else if (s.state === "connected")
    statusEl.textContent = `Robot connected: ${s.ip}${s.rssi !== null ? `, signal ${s.rssi} dBm` : ""}, since ${time(s.since)}`;
  else statusEl.textContent = `Robot lost at ${time(s.at)} (${s.ip}); waiting for it…`;
  for (const b of buttons) b.disabled = s.state !== "connected";
}

function connect() {
  ws = new WebSocket(`ws://${location.host}/`);
  ws.onopen = () => { serverEl.textContent = ""; };
  ws.onmessage = e => {
    const msg = JSON.parse(e.data);
    if (msg.type === "backlog") { log.textContent = ""; chars = 0; for (const it of msg.items) append(it.text, it.kind); log.scrollTop = log.scrollHeight; }
    else if (msg.type === "text" || msg.type === "note") append(msg.text, msg.type);
    else if (msg.type === "status") showStatus(msg.status);
  };
  ws.onclose = () => {
    statusEl.className = "lost";
    statusEl.textContent = "Server not reachable";
    serverEl.textContent = "is npm start running in pc/robot/? retrying…";
    for (const b of buttons) b.disabled = true;
    setTimeout(connect, 2000);
  };
}
connect();
