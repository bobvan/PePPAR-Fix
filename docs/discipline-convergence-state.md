# Discipline convergence state (reframes "disciplineModeFsm")

**Status:** design. Supersedes the `disciplineModeFsm` framing (dayplan
2026-05-28) per Bob's 2026-05-29 design preference. Validate every
increment in `closedLoopServoSim` (PR #81) before hardware.

## The problem

Discipline state is **scattered** across stages that don't coordinate:

- `DOFreqEst` `freq_verifying → tracking` mini state machine
- `DisciplineScheduler._converging` latch (1 → max_interval cliff)
- `OcxoTrustedGate.min_age_s` (engages by **age**, not lock state)
- exit-5 watchdog relaunch (out-of-process "reset")
- bootstrap glide

Same "uncoordinated state across stages" SatPulse 0.2 cited when it
rewrote its PHC discipline. It's also the root of the v2 regime mess:
we tried to detect acquiring-vs-locked *inside* the gate (innovation
bias-ratio) instead of having one coherent convergence signal the gate
just reads. And `clkpoc3GateOverRejectsBeforeLock` is the direct
symptom — the gate engaged by age while the loop was still far from
lock, over-rejected, starved, never locked.

## The reframe — derived continuous, not a latched FSM

Bob (2026-05-29), consistent with the position filter (AntPosEst):
**prefer an emergent / derived continuous state over an event-driven
latched FSM.** Convergence is a gradient recomputed each epoch from the
EKF, not a threshold we cross. So:

> Make the **policies** continuous functions of one **derived
> convergence signal**, recomputed every epoch. No latched modes.

A continuous policy of a derived metric **cannot chatter** on
enter/exit/re-enter the way a thresholded state can — it sidesteps the
hysteresis contrivance the v2 dwell-latch fell into.

This is not a rejection of the `reset/converging/tracking/holdover`
carve-up (Charlie confirmed it's the right decomposition) — it's a
statement about *representation*: those are **regions of one continuous
axis**, not discrete states with edge transitions. The
`longTauGnssCoupling` taper-not-cliff and the `√(P22)` coast-cap are
already exactly this; this doc extends it to the whole discipline loop.

## The single derived signal

`distance_to_lock` ∈ [0, 1] (0 = locked, 1 = far), already built and
tested as `discipline.normalized_distance_to_lock(metric_ns,
converged_ns, far_ns)` (PR #86). The raw `metric_ns` is the EKF's own
honest uncertainty:

- `√(P22)` — the DO-phase-state standard deviation, **honest only with
  Q-from-char** (qFromCharPerActuator, #83); with an inflated Q it
  reads converged too early (the same overconfidence that defeated the
  coast-cap and the gate).
- or `σ_total = √(P22 + measurement R)` when a fused output bound is
  wanted.

**One signal, read by all parts** (the centralization win SatPulse
cited). Computed once per epoch, in one place, and handed to every
policy. This is the part worth keeping a small home for — a
`DisciplineConvergence` object that owns the signal and the thin binary
layer below.

## Policies as continuous functions of `distance_to_lock`

| policy | today (latched / fixed) | continuous function of `m = distance_to_lock` |
|---|---|---|
| coast interval | `1 if converging else τ` cliff | `graded_interval(τ, m)` — **built (#86)**, taper IS the converging→tracking transition |
| coast-cap | n/a | `coast_cap_from_p22/tdev` on honest √(P22) — **built (#86)** |
| outlier gate strength | OCXO gate on by `min_age` | gate **off** at `m≈1` (converging: let the loop catch up), **on** at `m≈0` (tracking); strength scales with `m` — no age trigger |
| source weighting | fixed R per arm | soft-weight via fusion **R** as a function of `m`, not a hard latched switch |
| sawtooth/qErr correction | always | skip while `m` high (converging), apply when low |

`converged_ns` / `far_ns` are derived from `phase_error_budget_ns`, not
magic constants (e.g. converged at the budget, far at a small multiple
chosen in-sim) — to be pinned by the sim sweep, not guessed.

## Per-arm gating — Charlie's caveat (load-bearing)

The OCXO gate gates **only Arm 4 (TICC)**. The sim shows a host locks
**iff x[2] has a non-TICC carrier when TICC is gated**: PiFace-v1
(EXTINT carries x[2]) locks; clkPoC3 (TICC-primary) starves. So
"tracking ⇒ gate on" must **not** be a blanket gate — it must be
**per-arm**, keyed on which state each arm carries, leaving x[2] a
fallback (EXTINT) when TICC is rejected. A blanket tracking-gate
re-creates the clkPoC3 starvation.

Implication: the gate strength is `f(m)` **per arm**, and when an arm's
state has no alternate carrier the gate must degrade to soft-weighting
(inflate that arm's R) rather than hard-reject — never leave x[2]
uncarried. This dovetails with `routedQErrArm` (#79): the TICC arm
already routes to external/internal/raw; gating is the same idea —
admit, down-weight, or reject per arm.

## The thin binary layer (the honest exceptions)

A *little* machine still fits where the world is genuinely binary —
**not** gradients:

- **GNSS present / absent** → holdover entry/exit. A real discrete
  event (signal there or not), not a convergence gradient.
- **Gross fault → reset.** Replaces the exit-5 out-of-process relaunch
  with an **in-process** reset triggered by a `√(P)` sentinel (the
  coast-overconfidence divergence the sim reproduces). Low-risk,
  testable in-sim.

Everything else is the continuous axis. Where a policy *must* be binary,
prefer soft-weighting (fusion R) over a hard latched switch.

## Mapping existing logic (consolidate, don't rebuild)

- `DisciplineScheduler._converging` latch → delete; the interval is
  `graded_interval(τ, m)`.
- `OcxoTrustedGate.min_age_s` → delete the age trigger; gate strength is
  `f(m)`, per arm, tracking-region only.
- `DOFreqEst freq_verifying → tracking` → its job is "is x[3] trusted
  yet"; express as a region of `m`.
- exit-5 relaunch → in-process gross-fault→reset (binary layer).
- bootstrap glide → the `m≈1` end of the axis (already continuous via
  the landed glide).

## Naming (per Bob — `disciplineModeFsm` is itself a misnomer)

It is **mostly a derived continuous convergence signal** with a thin
discrete layer for binary world-conditions — not a "mode FSM."
Proposed: `DisciplineConvergence` (owns `distance_to_lock` + the binary
link/fault layer); retire the `Fsm` name. Logged in `docs/misnomers.md`
when the identifier lands.

## Implementation increments (each sim-validated)

1. **Centralize the signal + wire the deferred taper.** Add
   `DisciplineConvergence` computing `distance_to_lock` from the servo's
   √(P22); pass it as `distance_to_lock` into
   `compute_adaptive_interval` (the hook left unwired in #86). Smallest
   step; directly exercises the taper the sim already wants.
2. **Scheduler latch → continuous.** Remove `_converging`; interval and
   re-converge behavior become `f(m)`.
3. **Gate → per-arm, `f(m)`, tracking-region.** Replace `min_age` and
   the v2 regime hack; keep x[2] always carried (soft-weight when no
   alternate).
4. **Binary layer.** GNSS present/absent holdover hook + in-process
   gross-fault→reset (√(P) sentinel) replacing exit-5.

Gating: each increment is default-off / behind the same opt-in as the
coast-cap until the sim A/B clears it, then lab A/B (Main coordinates).
The honest small Q (#83) is the prerequisite that makes `m` truthful.
