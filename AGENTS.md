# Rules for AI agents

How to work in this repo is in the README ("How we work", "Conventions", "Git
conventions"). These rules are about how to answer Daniel.

## Answers

- Answer the question first, in one line.
- Then bullet points. Prose only where a bullet can't say it.
- Short, simple sentences. Plain words, no jargon.
- Say a thing once.
- Explain in plain terms. Example: "the horizon is the middle row, moved by the
  robot's tilt", not the math behind it.
- Numbers in a small table when there are several to compare.
- End with what is open or what you need from Daniel, if anything.
- Docs follow the same rules.

## Code review

- A code review follows Daniel's guide:
  `~/code/home-page/sources/skills/strategic-code-review.md` (outside this repo).
  Read it before any review.
- In short: review as a long-term maintainer. Look at design first (deep modules,
  small interfaces, one owner per rule), then correctness and whether the tests
  check the promised behaviour.
- Also flag overkill: code for rare cases that could just return an error, be
  counted or be ignored (DATA_FLOW_PLAN.md, "Simple first").
