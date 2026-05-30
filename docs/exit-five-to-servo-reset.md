# exitFiveToServoReset — routing the hard-exit sites through the in-process reset

**Status:** design draft (Bravo, 2026-05-30).  Follow-up to #107
(disciplineModeFsm increment #4 / binary layer).  Companion to
[`discipline-convergence-state.md`](discipline-convergence-state.md),
which introduced the in-process reset this doc generalizes.

## Why

#107 gave the engine an **in-process EKF reset** —
`DOFreqEst.reset()` + `DisciplineConvergence.reset()` — that preserves
the actuator command and keeps the engine running.  The binary layer
fires it on a gross fault (sustained `distance_to_lock = 1.0` outside
holdover).  This replaced *one* exit-5 path (the gross-fault case)
with something the engine survives: the DO keeps coasting at its last
good freq while the filter rebuilds, instead of the wrapper tearing
the process down and re-bootstrapping from scratch.

But the engine still has **five other hard-exit sites** that
`return 5` (→ `sys.exit(run(args))` → wrapper relaunch).  Some of them
detect *exactly the same condition the binary layer already handles in
process* — the servo has lost lock — and pay the full restart cost for
no reason.  Others detect conditions where a restart genuinely is
required (the position truth changed; a bootstrap precondition
failed).  This doc inventories all of them, decides which convert and
which stay, and specifies the machinery (a shared reset path + a
windowed reset budget) that makes the conversion safe.

A secondary motivation, surfaced by **#115**: the holdover path's
`servo.reset()` call was stranded when #107 changed the `reset`
signature from positional to keyword-only — the engine crashed with a
`TypeError` on every observation outage until #115 fixed it.  The
lesson — *a signature change in a callee silently stranded an old call
site* — argues for **funnelling every reset through one helper** rather
than scattering `servo.reset(...)` calls with hand-written argument
shapes across the engine.  That single helper is the natural home for
the budget logic too.

## Inventory — every `return 5` and its trigger (current `main`)

Line numbers as of `5d9e7c0`.  The runtime loop has a single physical
`return 5` (line 5431); most triggers reach it by setting
`servo_ctx['phc_diverged'] = True`, which the loop checks each epoch.

| # | Trigger | Site | How it exits | Class |
|---|---|---|---|---|
| A | 30 consecutive **catastrophic obs rejects** (residual-consistency gate, `n_used=0` with the counter advancing) | 4961 (+ sets `phc_diverged` at 4960) | direct `return 5` | obs-stream integrity |
| B | **PHC/EKF servo outlier cascade** — `|pps_err|` > `track_outlier_ns` for 30 consecutive epochs | `phc_diverged` at 8202 (via `_servo_outlier_decision`) → 5431 | `phc_diverged` | **servo lost lock** |
| C | **ClockMatrix servo outlier cascade** — same, CM path | `phc_diverged` at 7766 | `phc_diverged` | **servo lost lock** |
| D | **PHC error > `track_restep_ns`** for 3 consecutive epochs | `phc_diverged` at 8264 | `phc_diverged` | **servo lost lock** |
| E | **Freerun auto-stop** — `|pps_err|` exceeds `freerun_max_error_ns` | `phc_diverged` at 8274 | `phc_diverged` | intentional stop (not a fault) |
| F | **Position watchdog STEP auto-move** — antenna moved ≥ 1 m sustained → `mount_sn` bump + `.ppp.toml` invalidated | 5639 (+ `phc_diverged` at 5638) | direct `return 5` | **position truth changed** |
| G | **Survey-refresh STEP** — external `.survey.toml` ARP discontinuity | 4086 (`_apply_survey_refresh` returns `5` to caller) | propagated `return 5` | **position truth changed** |
| H | **TICC sanity check** at startup — first differential > 100 ms (PEROUT misaligned / PHC not bootstrapped) | 7603 | direct `return 5`, pre-loop | startup precondition |

(Note: the dayplan item that filed this work cited four sites with
drifted line numbers and labelled G as "SSR `mount_sn 0`"; the actual
SSR-side guards `return 3` (retryable) or raise at setup.  This table
supersedes that description.)

## Decision (a) — which sites are in-process recoverable

The test is: **after the trigger, are the actuator command and the
position solution both still valid?**  If yes, a filter reset that
preserves them is sufficient and the restart is wasted.  If the
trigger means the position truth changed or a bootstrap precondition
failed, the engine cannot reconstruct the needed state in process —
keep exit-5.

**Convert to in-process reset (servo lost lock; actuator + position still valid):**

- **B (PHC/EKF outlier cascade)** — this is *literally the condition
  the binary layer detects*.  The 30-outlier cascade and #107's
  gross-fault detector are two detectors of one fault with divergent
  consequences (exit-5 vs reset).  Unify them.
- **C (ClockMatrix outlier cascade)** — same fault, CM actuator path.
- **D (PHC error > restep, 3 epochs)** — a phase excursion the filter
  can recover from by rebuilding x/P at the preserved freq.

**Keep exit-5 (recovery needs state the engine can't rebuild in process):**

- **F (watchdog STEP auto-move)** — `mount_sn` was bumped and
  `.ppp.toml` invalidated; the ARP must be re-seeded from the survey
  chain at startup.  A servo reset can't re-seed position.
- **G (survey STEP)** — restart re-reads the new `.survey.toml`; same
  reasoning as F.
- **H (TICC sanity, startup)** — fires *before* the steady-state loop;
  there is no servo/filter to reset yet, and a PEROUT/PHC bootstrap
  fault needs the wrapper's full re-bootstrap.  Out of scope.
- **E (freerun auto-stop)** — not a fault at all; it's the intended
  end of a freerun measurement run.  Leave as-is.

**Boundary case — A (30 catastrophic obs rejects):** the actuator and
position are still valid, but the *obs stream* has been delivering
garbage for ~30 s (the residual-consistency gate rejected every
epoch).  This overlaps `_enter_obs_holdover`, which already handles obs
*absence*; catastrophic rejects are obs *present-but-unusable*.

- **Recommendation:** convert to **reset-into-holdover** (coast the
  actuator at last adjfine, rebuild the filter, re-acquire) **gated by
  the reset budget** in decision (c).  A transient cause (a multipath
  storm passing, a brief sky obstruction) recovers softly; a
  persistent cause (antenna fell, cable kicked) burns the budget and
  falls through to exit-5 for a full wrapper restart.  This is the
  most consequential call in this doc and the one most worth pinning
  with **lab data** — clkPoC3 is the host that generates these
  overnight (day0529 / day0530 captures), so it's both the motivating
  case and the validation bed.

## Decision (b) — reset scope per converted site

Not every reset is the binary layer's gentle filter-only reset.  Scope
to what actually diverged:

- **B / C / D (servo lost lock):** the binary-layer scope is exactly
  right — `servo.reset()` (re-init EKF x/P + innovation history,
  **preserve** the actuator) + `DisciplineConvergence.reset()` +
  scheduler latch reset.  **Do not** touch the PPP/position filter; the
  position solution is unaffected by a servo-side phase excursion.
- **A (catastrophic obs rejects), if converted:** heavier — "reset
  everything but the actuator."  The thing that diverged is the
  obs-side path, so the reset must also clear the PPP filter's
  `_consecutive_catastrophic_rejects` counter, purge stale PPS state
  (`_purge_pps_state`), and enter holdover (coast the actuator) so the
  loop doesn't immediately re-trip while re-acquiring.  This is closer
  to `_enter_obs_holdover` + a filter rebuild than to the binary
  layer's gentle reset.

The shared helper (below) takes a `scope` argument so each call site
declares how much to tear down, with the actuator command preserved in
every case.

**B can also fire non-transiently** (Charlie, #116 review).  The
PHC/EKF outlier cascade isn't always a recoverable phase excursion —
it can be chi²-gate over-rejection driven by a wrong `sigma_DO` (the
`clkpoc3GateOverRejectsBeforeLock` flavour).  A reset won't fix a gate
*configuration* problem; it just hands the loop another window to
over-reject, and the budget (decision c) eats the resets in ~minutes.
That's *correct* — it's exactly what the budget is for — but it means
the operator needs to *see* the pattern.  So the `[SERVO_RESET]` log
line must emit the **per-reason counter alongside the cumulative
count** ("3 resets this window, all reason=B") rather than a bare
total, so a gate-misconfig thrash is diagnosable from one log line
without trawling.  This is the concrete shape of the per-reason
counter extension noted under the refactor below.

## Decision (c) — repeated-reset budget (the safety net for the safety net)

In-process reset must not become an infinite loop that hides a broken
host.  A host that needs 5 resets in 5 minutes is broken in a way the
wrapper's full re-bootstrap (re-survey, re-init PEROUT, fresh state)
may handle better than endless in-process resets.

`BinaryLayer` already counts `n_gross_fault_resets` cumulatively but
has **no time window**.  Add a **windowed reset budget**, shared across
*all* in-process-reset sources:

- Record each reset's `CLOCK_MONOTONIC` timestamp (per
  [`stream-timescale-correlation.md`](stream-timescale-correlation.md)
  — never wall-clock; survives NTP steps and is the engine's one
  reliable timescale).
- When **> N resets occur within W seconds**, the next reset request
  returns **"exit 5"** instead of resetting in process — fall through
  to the wrapper.
- The budget is **shared** across the binary layer, the converted
  outlier cascades (B/C/D), and the converted catastrophic-reject path
  (A).  These are symptoms of one illness; a host resetting N times via
  *any mix* of detectors is broken.  A per-detector budget would let a
  host thrash N×(detectors) times before giving up.

Suggested defaults (tunable): `--reset-budget-n 3`,
`--reset-budget-window-s 300` (3 resets in 5 min → exit 5).  Defaults
chosen so a single transient event (one reset, re-acquire, hours of
quiet) never trips it, while a genuine thrash escalates within minutes.

## The unifying refactor — one reset path

Extract a single helper that every detector calls instead of either
`servo.reset()` or `return 5`:

```python
def _request_servo_reset(ctx, reason: str, scope: str = "servo",
                         *, now=time.monotonic) -> str:
    """Consult the shared reset budget and either reset in process or
    signal exit-5.  Returns "reset" (done in process) or "exit5"
    (budget exhausted — caller should return 5).

    scope="servo"   → DOFreqEst.reset() + DisciplineConvergence.reset()
                      + scheduler latch reset (B/C/D, binary layer).
    scope="obs"     → the above + PPP catastrophic-counter clear +
                      _purge_pps_state + enter holdover (A).
    Preserves the actuator command in every scope.

    The budget is SHARED across every reset reason — B/C/D outlier
    cascades, the A obs-reject path, and the #107 binary layer all
    draw from one window.  These are symptoms of one illness; a
    per-detector budget would let a host thrash N×(detectors) times
    before the wrapper gets a chance.  (Bake this rationale into the
    real docstring so the next reader doesn't suggest splitting them.)
    """
```

**Inject the clock** (Charlie, #116 review): the budget reads time via
the `now` keyword (default `time.monotonic`) so sim/unit tests can
drive the N-in-W window deterministically instead of sleeping `W`
seconds or monkey-patching a global — the same time-injection pattern
the #111 binary-layer test uses.  The CLOCK_MONOTONIC requirement from
decision (c) is satisfied by the `time.monotonic` default.

(Return-type nit, non-blocking: a `ResetOutcome` enum or a bool reads
slightly cleaner at the call site than the `"reset"`/`"exit5"` strings
— `if not _request_servo_reset(...): return 5`.  Either is fine;
settle it at implementation.)

Benefits:

- **One place** performs `servo.reset()`, so a future signature change
  can't strand a caller (the #115 failure mode).  The detectors become
  *inputs* — "request a reset, reason=X" — not call sites that each
  hand-write the reset mechanics.
- **One budget** is consulted in one place (decision c is enforceable,
  not scattered).
- The binary-layer block at engine:8446 becomes a call to this helper;
  the cascades at 7766 / 8202 / 8264 replace
  `ctx['phc_diverged'] = True` with
  `if _request_servo_reset(ctx, reason) == "exit5": phc_diverged`.
- `n_gross_fault_resets` / `max_consec` diagnostics extend naturally to
  per-reason counters for the `[GROSS_FAULT_RESET]` / new
  `[SERVO_RESET]` log lines.

Sites F / G / H keep their `return 5` untouched.

## Lint follow-up — catch the next stranded call site (Charlie's #115 note)

Add a contract check (a unit test, or a pre-commit hook) that fails if
any `.reset(` call on the servo object appears with a **positional**
argument — the only allowed shapes are `reset()` and
`reset(initial_freq=..., ...)`.  This pins the keyword-only contract at
the *call-site* level, complementing
`test_enter_obs_holdover_reset.py::test_reset_is_keyword_only` which
pins it at the API level.

A bare `grep` works but is fragile (matches strings, comments,
unrelated `.reset(` on other objects).  Prefer a small **AST-based
check** (Charlie, #116 review) — ~10 lines of `ast.parse` + `ast.walk`
gated on the file basename `peppar_fix_engine.py`, flagging
`Call` nodes whose receiver is a known servo name and whose `args`
(positional) is non-empty.  Low priority; rolls in with the refactor.

## Test plan

- **Per-converted-site unit tests** (B/C/D): trigger the cascade
  condition; assert `_request_servo_reset` returns `"reset"`, the
  actuator command is unchanged across the reset, and no `return 5`.
  The outlier path already has `_servo_outlier_decision` extracted
  (tests/test_servo_outlier_decision.py) — extend it to assert the
  reset-vs-exit5 routing.
- **Catastrophic-reject path (A):** trigger 30 rejects; assert
  reset-into-holdover, PPP counter cleared, actuator held, no exit-5
  until the budget is spent.
- **Shared budget:** synthetic failure-then-recovery loop crossing
  N-in-W from a *mix* of reasons; assert the (N+1)th request returns
  `"exit5"`.  Monotonic-time injection (no wall-clock in the test).
- **No-regression:** F / G / H still `return 5`; binary-layer behavior
  (#107 sim A/B in #111) unchanged when the budget isn't exhausted.

### Sim harness shape (Charlie, #116 review — mirrors the #111 binary-layer A/B)

`SimConfig` knobs:

- `reset_budget_n: int = 3`, `reset_budget_window_s: float = 300.0` —
  defaults match decision (c); tests override to small values for
  short runs.
- `induced_drop_windows: list[tuple[float, float]] = []` — a **list**
  (not a single tuple) so a test can stack N+1 fault windows
  back-to-back to exhaust the budget.  The existing single
  `drop_measurements_window_s` becomes the head of this list.
- Optional `induced_outlier_windows` / `induced_obs_reject_windows` to
  drive B and A from different sources and **prove the budget is
  shared**, not per-detector.

Sim-side instrumentation:

- `reset_events: list[tuple[float, str, str]]` on the sim result —
  `(t_s, reason, outcome)`, `outcome ∈ {"reset", "exit5"}` — mirroring
  `gross_fault_events` from #111.
- On `"exit5"`, the sim **stops** the run (or marks `result.exit5_at =
  t_s`): the helper's "wrapper would have re-bootstrapped" semantic.
  Don't keep pumping the loop past the request, or budget bugs hide.

Test cases (slot into the existing `test_servo_sim.py` patterns):

1. **Sub-budget recovery** — N faults inside W → all reset; assert
   `len(reset_events) == N`, all `"reset"`, actuator preserved across
   each.
2. **Budget exhaustion** — N+1 faults inside W → (N+1)th outcome is
   `"exit5"` and the sim terminated there.
3. **Recovery after quiet** — N faults inside W, then > W quiet, then
   1 fault → outcome `"reset"` (budget recovered).
4. **Shared-not-per-detector** — alternating reasons (B / A / B within
   W) → (N+1)th still `"exit5"`.  *The* load-bearing test for
   decision (c).
5. **Default-off byte-identical** — feature flag off (or
   `reset_budget_n=∞`) → outcomes never `"exit5"`; existing #111
   binary-layer A/Bs unchanged.

## Lab plan

clkPoC3 — the host generating exit-5s overnight in the day0529 /
day0530 captures — with `--gross-fault-reset` plus the converted
cascades:

1. Confirm B/C/D recover in process (log `[SERVO_RESET] reason=...`,
   no wrapper relaunch, no actuator step at the reset epoch).
2. Confirm the budget falls through to exit-5 under a *persistent*
   induced fault (e.g. a sustained outlier injection), and the wrapper
   re-bootstraps cleanly.
3. Decide A's conversion (reset-into-holdover vs keep exit-5) from the
   catastrophic-reject rate and whether the in-process re-acquire
   actually beats a restart on this host.

## PR sequencing (sim-then-lab cadence)

1. **This design doc** → review (Main + Charlie).
2. Extract `_request_servo_reset` + the shared `ResetBudget`; convert
   B / C / D.  (Bravo writes; Main labs.)
3. **Sim A/B** in Charlie's `closedLoopServoSim`: induce repeated
   faults, confirm budget fall-through and actuator preservation —
   the same harness shape #111 used for the binary layer.  This step
   **lands before** the clkPoC3 lab run (4): even though lab data
   decides A's conversion, the budget mechanics must be pinned by sim
   first so a lab anomaly can't be misread as a budget-logic bug
   (Charlie, #116 review).
4. **Lab A/B** on clkPoC3.
5. Decide and (if confirmed) land **A**'s reset-into-holdover
   conversion from lab data.
6. Optional: the stale-positional `.reset(` lint.

Only after step 2 lands is the disciplineModeFsm design doc's claim
"replacing exit-5" actually complete for the recoverable sites.
