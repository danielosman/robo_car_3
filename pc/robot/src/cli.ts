// The `robot` command line (doc/CLI_API_PLAN.md): turns its arguments into a request to
// the robot server's command API and prints the answer. It knows no commands and
// holds no state: `robot <name> [args] [--flags]` becomes /api/robot/<name>, each
// --flag a query parameter (--no-wait: nowait). `robot steer` posts its commands and
// prints each answer as it comes. The server: ROBOT_API, default http://127.0.0.1:8080.
//   robot --help | robot steer r30d50 | robot map 42 --png | robot status --json
const API = (process.env.ROBOT_API ?? "http://127.0.0.1:8080").replace(/\/$/, "");

const args = process.argv.slice(2);
const flags = args.filter(a => a.startsWith("--")).map(a => a.slice(2).replace(/-/g, ""));
const words = args.filter(a => !a.startsWith("--"));
const [name, ...rest] = words;

let url: string, init: RequestInit = {};
if (!name || flags.includes("help") || args.includes("-h")) {
  url = `${API}/api/robot/help${name ? `/${encodeURIComponent(name)}` : ""}`;
} else {
  const query = flags.length ? `?${flags.map(encodeURIComponent).join("&")}` : "";
  if (name === "steer") {
    url = `${API}/api/robot/steer${query}`;
    init = { method: "POST", body: rest.join(" "), headers: { "content-type": "text/plain" } };
  } else {
    url = `${API}/api/robot/${encodeURIComponent(name)}${rest.length ? `/${rest.map(encodeURIComponent).join("/")}` : ""}${query}`;
  }
}

try {
  const res = await fetch(url, init);
  const out = res.ok ? process.stdout : process.stderr;
  const decoder = new TextDecoder();
  for await (const chunk of res.body ?? []) out.write(decoder.decode(chunk, { stream: true }));
  process.exitCode = res.ok ? 0 : 1;
} catch (err) {
  console.error(`robot: can't reach the robot server at ${API} (npm start in pc/robot/): ${(err as Error).message}`);
  process.exitCode = 2;
}
