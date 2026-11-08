// The robot's console in the browser: the log and the connection status. It only
// shows; steering goes through the API (the viewer at /, or the CLI).
const MAX_CHARS = 1024 * 1024;
const log = document.getElementById("log");
const statusEl = document.getElementById("status");
const serverEl = document.getElementById("server");
const follow = document.getElementById("follow");
let chars = 0;

document.getElementById("clear").onclick = () => { log.textContent = ""; chars = 0; };

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
let status = null;
function showStatus(s) {
  status = s;
  const quiet = s.state === "connected" && s.quietSince;
  statusEl.className = quiet ? "quiet" : s.state;
  if (s.state === "waiting") statusEl.textContent = "Waiting for robot…";
  else if (quiet) // the connection is open, nothing arrives: a slow link, or the robot gone
    statusEl.textContent = `Robot connected: ${s.ip}; nothing from it for ${Math.round((Date.now() - s.quietSince) / 1000)} s`;
  else if (s.state === "connected")
    statusEl.textContent = `Robot connected: ${s.ip}${s.rssi !== null ? `, signal ${s.rssi} dBm` : ""}, since ${time(s.since)}`;
  else statusEl.textContent = `Robot lost at ${time(s.at)} (${s.ip}); waiting for it…`;
}
setInterval(() => { if (status?.state === "connected" && status.quietSince) showStatus(status); }, 1000);

function connect() {
  const ws = new WebSocket(`ws://${location.host}/`);
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
    setTimeout(connect, 2000);
  };
}
connect();
