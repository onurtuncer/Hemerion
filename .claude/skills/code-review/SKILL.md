---
name: code-review
description: Rigorous structured code review for correctness, safety, performance, and maintainability. Use whenever the user asks to review code, a diff, a PR, a commit, or a file; says "review this", "check my code", "is this safe to merge", "any bugs here"; or pastes code and asks what's wrong with it. Covers modern C++ (C++17/20/23), embedded C/C++ (STM32, FreeRTOS, bare-metal), Python (scientific/numerical), CMake, and Verilog/SystemVerilog RTL. Also use before merges, releases, or when the user asks for a second pair of eyes on recently written code.
---

# Code Review

Perform a structured, severity-ranked review. Find real defects; do not pad the review with style trivia to look thorough.

## Procedure

1. **Understand intent first.** Read the surrounding code, build files, or docs as needed. If the purpose of the change is unclear, state your assumption in one line and review against it.
2. **Review in this priority order:**
   1. **Correctness** — logic errors, off-by-one, wrong math, unit/frame/sign conventions, unhandled edge cases (empty input, NaN, overflow, timezone/encoding).
   2. **Undefined behavior & memory safety** (C/C++) — lifetime bugs, dangling references, iterator invalidation, uninitialized reads, signed overflow, aliasing violations, missing rule-of-five, non-RAII resource handling.
   3. **Concurrency** — data races, missing synchronization, deadlock ordering, ISR-unsafe calls, priority inversion, non-atomic flag polling, incorrect memory ordering.
   4. **Numerical robustness** (scientific code) — catastrophic cancellation, unstable formulations, missing convergence checks, hard-coded tolerances, mixing float/double silently, non-dimensionalization errors.
   5. **Security** — injection, unvalidated input at trust boundaries, secrets in code, unsafe deserialization, path traversal.
   6. **Error handling** — swallowed exceptions, ignored return codes, error paths that leak resources or leave inconsistent state.
   7. **API & design** — leaky abstractions, God objects, hidden global state, functions doing too much, missing const-correctness.
   8. **Performance** — only flag issues that plausibly matter: accidental O(n²), copies of large objects, allocation in hot loops/ISRs, blocking calls in real-time paths.
   9. **Tests & maintainability** — missing tests for the risky paths found above, misleading names, dead code.
3. **Embedded-specific checks** (when target is MCU/RTOS/RTL): stack usage of new call paths, dynamic allocation after init, blocking in ISRs, volatile misuse vs atomics, watchdog interaction, fixed-point scaling, clock-domain crossings and reset behavior in RTL.
4. **Verify before asserting.** If you can compile, run, or test the code in the sandbox, do it rather than speculating. For claims about library/API behavior you're not certain of, check the docs.

## Output format

- **Verdict line first**: `BLOCK` (defects that must be fixed), `FIX BEFORE MERGE` (should fix now), or `LGTM with nits`.
- Then findings grouped by severity: **[critical] / [major] / [minor] / [nit]**.
- Each finding: file:line (or code excerpt), what's wrong, why it matters, and a concrete fix — show the corrected code for critical/major items.
- Maximum ~10 findings; if there are more, report the worst and summarize the rest in one line. Never invent findings to fill quota.
- If the code is genuinely clean, say `LGTM` and stop. Do not manufacture nits.

## Style

- Be direct and specific; no hedging, no praise padding.
- Distinguish facts ("this dereferences a moved-from pointer") from judgment calls ("I'd split this class") — label the latter as opinion.
- Don't restate the diff or explain what the code does unless it's needed to justify a finding.
- Respect the existing codebase conventions; don't impose a different style guide.
