// The vision model (doc/OBJECTS_PLAN.md): an OpenAI-compatible chat server, by default
// llama.cpp's router on this machine serving Gemma 4 26B. MODEL_API sets the server,
// MODEL the model. The router doesn't load a model on a request ("model is not
// loaded"): it is asked to load it, and the request goes again once it has.

export interface ToolCall { id: string; type: "function"; function: { name: string; arguments: string } }
export type Content = string | ({ type: "text"; text: string } | { type: "image_url"; image_url: { url: string } })[];
export interface ChatMessage {
  role: "system" | "user" | "assistant" | "tool";
  content: Content;
  tool_calls?: ToolCall[];
  tool_call_id?: string;
}
export interface ChatReply { message: ChatMessage; reasoning: string | null; tokens: number | null }
// One turn of the conversation: the messages so far in, the model's answer out.
export type Chat = (messages: ChatMessage[], tools: object[], opts: { thinking: boolean }) => Promise<ChatReply>;

export const MODEL_API = (process.env.MODEL_API ?? "http://127.0.0.1:8080").replace(/\/$/, "");
export const MODEL = process.env.MODEL ?? "gemma-4-26b";

// A failure talking to the model server (not the model's answer being wrong).
export class ModelError extends Error {}

const TURN_MS = 5 * 60_000, LOAD_MS = 10 * 60_000;

export function openAiChat(api = MODEL_API, model = MODEL, note: (text: string) => void = () => {}): Chat {
  const post = async (path: string, body: object, ms: number) => {
    let r: Response;
    try {
      r = await fetch(`${api}${path}`, { method: "POST", headers: { "content-type": "application/json" },
                                          body: JSON.stringify(body), signal: AbortSignal.timeout(ms) });
    } catch (err) {
      throw new ModelError(`can't reach the model server at ${api} (MODEL_API): ${(err as Error).message}`);
    }
    return { status: r.status, text: await r.text() };
  };

  async function load(): Promise<void> {
    note(`model ${model} isn't loaded: loading it (the first time takes a while)`);
    const r = await post("/models/load", { model }, 60_000);
    if (r.status >= 400) throw new ModelError(`the model server can't load ${model}: ${r.text.slice(0, 300)}`);
    const until = Date.now() + LOAD_MS;
    while (Date.now() < until) {
      try {
        const list = await (await fetch(`${api}/v1/models`, { signal: AbortSignal.timeout(10_000) })).json() as
          { data?: { id: string; status?: { value?: string } }[] };
        const state = list.data?.find(m => m.id === model)?.status?.value;
        if (state === "loaded") { note(`model ${model} loaded`); return; }
        if (state === "failed") throw new ModelError(`the model server failed to load ${model}`);
      } catch (err) { if (err instanceof ModelError) throw err; }
      await new Promise(res => setTimeout(res, 2000));
    }
    throw new ModelError(`${model} didn't load within ${LOAD_MS / 60_000} minutes`);
  }

  return async (messages, tools, { thinking }) => {
    const body = { model, messages, tools, temperature: 0, chat_template_kwargs: { enable_thinking: thinking } };
    let r = await post("/v1/chat/completions", body, TURN_MS);
    if (r.status === 400 && r.text.includes("not loaded")) { await load(); r = await post("/v1/chat/completions", body, TURN_MS); }
    if (r.status >= 400) throw new ModelError(`the model server answered ${r.status}: ${r.text.slice(0, 300)}`);
    const reply = JSON.parse(r.text) as { choices?: { message?: ChatMessage & { reasoning_content?: string } }[];
                                          usage?: { completion_tokens?: number } };
    const m = reply.choices?.[0]?.message;
    if (!m) throw new ModelError(`the model server's answer has no message: ${r.text.slice(0, 300)}`);
    const message: ChatMessage = { role: "assistant", content: m.content ?? "" };
    if (m.tool_calls?.length) message.tool_calls = m.tool_calls;
    return { message, reasoning: m.reasoning_content ?? null, tokens: reply.usage?.completion_tokens ?? null };
  };
}
