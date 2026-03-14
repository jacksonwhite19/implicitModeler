# AI Coding Agent Guidelines

Refer to `MasterPlan.md` before planning or implementing any change.
It is the authoritative source for project goals, scope boundaries, and architectural direction.

Do your best to minimize unnecessary context usage or token calls.

---

# Core Principles

- **Simplicity First**: Make the smallest change that works.
- **Minimal Impact**: Only modify what is necessary.
- **Fix Root Causes**: No temporary patches or surface-level fixes.
- **Follow Existing Patterns** before introducing new abstractions.
- **Prove It Works** with concrete verification.
- **No Scope Creep**: If unrelated issues are discovered, log them as follow-ups — do not expand the current task.

---

# Pre-Implementation Checklist (Mandatory)

Before writing code:

- [ ] Have I read `MasterPlan.md`?
- [ ] Is this change within project scope?
- [ ] Am I following an existing pattern?
- [ ] Is this the smallest safe change?
- [ ] Do I know exactly how I will verify this?

If any answer is unclear, stop and clarify before proceeding.

---

# Workflow Orchestration

## 1. Plan Mode Default

Enter plan mode for any non-trivial task, including:

- Multi-file changes
- Data model changes
- Public API changes
- User-visible behavior changes
- Production-impacting behavior
- Any task requiring 3+ logical steps

### Planning Rules

- Write a checklist in `tasks/todo.md`.
- Break **every task into short, manageable steps**.
- Each step must be small, atomic, and verifiable.
- Only one step may be “in progress” at a time.
- **Do not move to the next step until the current step is fully completed and verified.**
- Include verification tasks explicitly.
- If new information invalidates the plan: **stop, update the plan, then continue.**
- If the user requests, at the completion of the plan you may re-evaluate, and create an updated plan for next improvements, and then execute those.
Do not batch large changes. Do not “try to do everything at once.”

---

## 2. Subagent Strategy

Use subagents to:

- Locate relevant code paths.
- Analyze existing patterns.
- Investigate failing tests or logs.
- Research dependencies or risks.

Rules:

- One focused objective per subagent.
- Require a concrete deliverable (file list, summary, findings).
- Synthesize findings before coding.

---

## 3. Self-Improvement Loop

After any correction, mistake, or unexpected issue:

Update `tasks/lessons.md` with:

- Failure mode
- Detection signal
- Prevention rule

Review relevant lessons:
- At session start
- Before major refactors

Continuously reduce repeat mistakes.

---

## 4. Verification Before Done

Never mark a task complete without evidence.

### If tests exist:
- Run all relevant tests.

### If no tests exist:
- Add the smallest regression coverage that would have caught the issue.

### If verification cannot be automated:
- Provide deterministic manual reproduction steps.

Always document:

- What was run
- What passed/failed
- Where evidence exists

Ask:
> Would a staff engineer approve this diff and verification story?

---

## 5. Demand Elegance (Balanced)

For non-trivial changes:

- Ask: “Is there a simpler structure with fewer moving parts?”
- If a fix is hacky and a clean solution is feasible without expanding scope, implement the clean solution.
- Do not refactor unrelated areas.
- Do not over-engineer simple fixes.

---

## 6. Autonomous Bug Fixing (Guardrails Included)

When given a bug report:

1. Reproduce reliably
2. Isolate root cause
3. Fix root cause (not symptoms)
4. Add smallest regression coverage
5. Verify end-to-end

Rules:

- Use logs, errors, and failing tests as primary signals.
- Do not request hand-holding.
- Do not refactor unless required for correctness.
- Ask exactly one targeted question only if blocked, with a recommended default.

---

# Risk Awareness

Before implementing non-trivial changes:

- Identify potential blast radius.
- Identify affected modules, APIs, and data paths.
- Ensure changes are reversible when risk is high (feature flag, config gate, isolated commit).

---

# Task Management Discipline

For any non-trivial work:

1. Write plan in `tasks/todo.md`.
2. Define acceptance criteria clearly.
3. Keep exactly one step marked “in progress.”
4. Complete and verify that step before moving on.
5. Add a “Results” section summarizing:
   - What changed
   - Where
   - How it was verified
6. Update `tasks/lessons.md` when appropriate.

If scope expands unexpectedly:
- Stop.
- Re-plan.
- Do not continue blindly.

---

# Stop-the-Line Rule

If any unexpected behavior occurs:

- Stop adding features.
- Preserve evidence.
- Reproduce and isolate.
- Update plan before continuing.

No forward progress without understanding.

---

# Engineering Standards

- Respect module boundaries.
- Prefer optional parameters over duplicated logic.
- Keep error semantics consistent.
- Avoid new dependencies unless clearly justified.
- Encode invariants at boundaries.
- Treat all user input as untrusted.
- Avoid premature optimization.
- Fix obvious inefficiencies when encountered.

---

# Definition of Done

A task is done only when:

- Acceptance criteria are satisfied.
- Tests/lint/typecheck/build pass (or documented reason not run).
- Regression coverage exists when appropriate.
- Risk is controlled (feature flag or rollback path if needed).
- Code follows existing conventions.
- A clear verification story exists:
  - What changed
  - Where
  - How it was verified
## Autonomous Backlog Execution (Mandatory)

- When backlog steps remain unblocked in `tasks/todo.md`, continue directly to the next step without waiting for new user prompts.
- Do not pause after a completed step unless there is a real blocker (missing external input, broken environment, unsafe assumption).
- Status should be written to `tasks/liveStatus.md`; avoid pausing execution just to post chat updates.
- If there is no blocker, do not idle in chat; execute backlog steps continuously and only interrupt for blockers or major phase completion.
- After completing a numbered backlog step, immediately begin the next numbered step in sequence.
- Do not stop at arbitrary tranche boundaries (for example after 5-10 steps) when pending unblocked steps still exist.
- Keep chat silent during normal execution: only post here for blockers or major major milestones explicitly.
- Use `tasks/liveStatus.md` as the default progress channel for all routine updates.
