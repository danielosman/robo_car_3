// The view's server: view/ is the page; it reads ../data and ../generated.
import { defineConfig } from "vite";

export default defineConfig({
  root: "view",
  server: { host: "127.0.0.1", port: 5173, fs: { allow: [".."] } },
  build: { outDir: "../dist", emptyOutDir: true },
});
