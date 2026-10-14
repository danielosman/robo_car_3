// The one queue every action goes through, whoever sent it: one at a time, in the
// order they arrived. Anything that doesn't end ok empties it: the commands still
// waiting are answered "dropped"; a move's own snap still runs first, so the newest
// snapshot shows where the robot ended up. Stop skips the queue and empties it. Every
// answer goes to the sender and to everyone (the log).
import { commandText, isMove, type Command } from "./commands.ts";
import type { Answer, Sender } from "./answers.ts";
import type { Runner } from "./runner.ts";

type Tell = (a: Answer) => void;
interface Job { command: Command; sender: Sender; tell: Tell; settle: () => void }
export interface QueueState { running: string | null; waiting: string[] }

export class CommandQueue {
  private jobs: Job[] = [];
  private running: Job | null = null;
  private runner: Runner;
  private everyone: (a: Answer, sender: Sender) => void;

  constructor(runner: Runner, everyone: (a: Answer, sender: Sender) => void) {
    this.runner = runner;
    this.everyone = everyone;
  }

  // Queues the commands; resolves once each has been answered.
  submit(commands: Command[], sender: Sender, onAnswer: Tell): Promise<void> {
    const tell = this.teller(sender, onAnswer);
    const settled = commands.map(command => new Promise<void>(settle => this.jobs.push({ command, sender, tell, settle })));
    void this.work();
    return Promise.all(settled).then(() => {});
  }

  // Stop: at once, past whatever runs; the queue is emptied.
  async stop(sender: Sender, onAnswer: Tell): Promise<void> {
    const command: Command = { name: "stop", values: [] };
    this.dropWaiting("stop");
    this.teller(sender, onAnswer)({ command: "stop", ...(await this.runner.run(command)) });
  }

  state(): QueueState {
    return { running: this.running && commandText(this.running.command), waiting: this.jobs.map(j => commandText(j.command)) };
  }

  private teller(sender: Sender, onAnswer: Tell): Tell { return a => { onAnswer(a); this.everyone(a, sender); }; }

  private async work(): Promise<void> {
    if (this.running) return;
    let ended: string | null = null; // a move that didn't end ok: after its snap, the rest goes
    for (let job; (job = this.jobs.shift());) {
      this.running = job;
      const answer: Answer = { command: commandText(job.command), ...(await this.runner.run(job.command)) };
      this.running = null;
      job.tell(answer);
      job.settle();
      const why = answer.result === "ok" ? null : `${answer.command} ended: ${answer.result}`;
      if (why && isMove(job.command) && this.jobs[0]?.command.auto) ended = why;
      else if (why ?? ended) { this.dropWaiting((why ?? ended)!); ended = null; }
    }
  }

  private dropWaiting(why: string): void {
    for (const job of this.jobs.splice(0)) {
      job.tell({ command: commandText(job.command), result: "dropped", why });
      job.settle();
    }
  }
}
