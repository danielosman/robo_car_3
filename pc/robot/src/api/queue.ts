// The one queue every steering command goes through, whoever sent it: one at a time,
// in the order they arrived. Anything that doesn't end ok empties it: the commands
// still waiting are answered "dropped"; a move's own snapshot still runs first, so
// the newest snapshot shows where the robot ended up. Stop skips the queue and
// empties it. Every answer goes to the sender and to everyone (the log).
import { isMove, shortText, type Command } from "./commands.ts";
import { answerFor, type Answer, type Sender } from "./answers.ts";
import type { Runner } from "./runner.ts";

type Tell = (a: Answer) => void;
interface Job { command: Command; sender: Sender; tell: Tell; settle: () => void }
export interface QueueState { running: { command: string; sender: Sender } | null; waiting: { command: string; sender: Sender }[] }

export class CommandQueue {
  private jobs: Job[] = [];
  private running: Job | null = null;
  private runner: Runner;
  private everyone: Tell;

  constructor(runner: Runner, everyone: Tell) {
    this.runner = runner;
    this.everyone = everyone;
  }

  // Queues the commands; resolves once each has been answered.
  submit(commands: Command[], sender: Sender, onAnswer: Tell): Promise<void> {
    const tell = this.teller(onAnswer);
    tell({ kind: "queued", command: commands.map(shortText).join(" "), text: "", sender });
    const settled = commands.map(command => new Promise<void>(settle => this.jobs.push({ command, sender, tell, settle })));
    void this.work();
    return Promise.all(settled).then(() => {});
  }

  // Stop: at once, past whatever runs; the queue is emptied.
  async stop(sender: Sender, onAnswer: Tell): Promise<void> {
    const command: Command = { name: "stop", values: [] };
    this.dropWaiting();
    this.teller(onAnswer)(answerFor(command, await this.runner.run(command), sender));
  }

  state(): QueueState {
    const show = (j: Job) => ({ command: shortText(j.command), sender: j.sender });
    return { running: this.running && show(this.running), waiting: this.jobs.map(show) };
  }

  private teller(onAnswer: Tell): Tell { return a => { onAnswer(a); this.everyone(a); }; }

  private async work(): Promise<void> {
    if (this.running) return;
    let failed = false; // a move didn't end ok: after its snapshot, the rest goes
    for (let job; (job = this.jobs.shift());) {
      this.running = job;
      const outcome = await this.runner.run(job.command);
      this.running = null;
      job.tell(answerFor(job.command, outcome, job.sender));
      job.settle();
      if (!outcome.ok && isMove(job.command) && this.jobs[0]?.command.afterMove) failed = true;
      else if (!outcome.ok || failed) { failed = false; this.dropWaiting(); }
    }
  }

  private dropWaiting(): void {
    for (const job of this.jobs.splice(0)) {
      job.tell({ kind: "dropped", command: shortText(job.command), text: "", sender: job.sender });
      job.settle();
    }
  }
}
