// ESLint (`npm run lint`): the recommended rules, TypeScript's for the server
// (src/), the browser's globals for the pages (public/).
import js from "@eslint/js";
import tseslint from "typescript-eslint";
import globals from "globals";

export default tseslint.config(
  { ignores: ["node_modules/", "recordings/", "logs/"] },
  js.configs.recommended,
  {
    files: ["src/**/*.ts"],
    extends: [tseslint.configs.recommended],
    languageOptions: { globals: globals.node },
  },
  {
    files: ["public/**/*.js"],
    languageOptions: { globals: globals.browser, sourceType: "module" },
  },
  {
    files: ["*.js"],
    languageOptions: { globals: globals.node, sourceType: "module" },
  },
);
