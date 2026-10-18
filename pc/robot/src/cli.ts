// The `robot` command line (doc/CLI_API_PLAN.md): turns its arguments into a request to
// the robot server's command API and prints the answer as it comes. It knows no
// commands and holds no state: `robot <name> [args] [--flags]` posts the args to
// /api/robot/<name>, each --flag a query parameter (--no-wait: nowait; --by=claude-code:
// by=claude-code); --help gets the help. The server: ROBOT_API, default http://127.0.0.1:8081.
//   robot --help | robot steer r30d50 | robot snap | robot map 42 --png
//   robot detect_save 42 --by=claude-code '{"objects":[]}'
const API = (process.env.ROBOT_API ?? "http://127.0.0.1:8081").replace(/\/$/, "");

const args = process.argv.slice(2);
// A flag's name loses its dashes; a value after = is kept as it is.
const flags = args.filter(a => a.startsWith("--")).map(a => {
  const [flagName, ...value] = a.slice(2).split("=");
  const key = encodeURIComponent(flagName.replace(/-/g, ""));
  return value.length ? `${key}=${encodeURIComponent(value.join("="))}` : key;
});
const words = args.filter(a => !a.startsWith("--"));
const [name, ...rest] = words;

let url: string, init: RequestInit = {};
if (!name || flags.includes("help") || args.includes("-h")) {
  url = `${API}/api/robot/help${name ? `/${encodeURIComponent(name)}` : ""}`;
} else {
  const query = flags.length ? `?${flags.join("&")}` : "";
  url = `${API}/api/robot/${encodeURIComponent(name)}${query}`;
  init = { method: "POST", body: rest.join(" "), headers: { "content-type": "text/plain" } };
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
