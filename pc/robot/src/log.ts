// The one log everyone sees (doc/CLI_API_PLAN.md, the log): the robot's console text,
// the server's notes, and every command's answers with who sent it. Each item goes to
// the log file, the terminal and whoever listens (the pages' WebSockets); the newest
// 256 KB are kept for a page that opens later and for `robot log`.
// Events: "item" (LogItem).
import { EventEmitter } from "node:events";
import type { WriteStream } from "node:fs";
import type { Answer, Sender } from "./api/answers.ts";

export interface LogItem { kind: "text" | "note" | "answer"; text: string; command?: string }

const BACKLOG_CHARS = 256 * 1024;

export class Log extends EventEmitter {
  private items: LogItem[] = [];
  private chars = 0;
  private file: WriteStream | null;
  private echo: boolean;

  // echo: also to the terminal.
  constructor(file: WriteStream | null, echo = true) {
    super();
    this.file = file;
    this.echo = echo;
  }

  // The robot's console text, as it comes (not always whole lines).
  text(text: string): void {
    this.file?.write(text);
    if (this.echo) process.stdout.write(text);
    this.add({ kind: "text", text });
  }

  // What the server saw: "--- 14:03:12 robot connected from … ---".
  note(text: string): void {
    const line = `--- ${new Date().toLocaleTimeString()} ${text} ---\n`;
    this.file?.write(line);
    if (this.echo) console.log(line.trimEnd());
    this.add({ kind: "note", text: line });
  }

  // '[cli] {"command":"steer d50","result":"ok",…}'.
  answer(a: Answer, sender: Sender): void {
    const line = `[${sender}] ${JSON.stringify(a)}\n`;
    this.file?.write(line);
    if (this.echo) process.stdout.write(line);
    this.add({ kind: "answer", text: line, command: a.command });
  }

  backlog(): LogItem[] { return [...this.items]; }

  // The last n lines.
  tail(n: number): string[] {
    const lines = this.items.map(i => i.text).join("").split("\n");
    if (lines.at(-1) === "") lines.pop();
    return lines.slice(-n);
  }

  private add(item: LogItem): void {
    this.items.push(item);
    this.chars += item.text.length;
    while (this.chars > BACKLOG_CHARS && this.items.length > 1) this.chars -= this.items.shift()!.text.length;
    this.emit("item", item);
  }
}
