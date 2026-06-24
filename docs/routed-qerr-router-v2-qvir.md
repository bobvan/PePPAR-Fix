# Routed-qErr router v2 — qVIR-routed two-candidate

> **RETIRED 2026-06-22 (retireQerrMatchingCode, I-064807).** The qVIR-gated
> v2 router and the matched v1 router were removed from the runtime once the
> comparative-gate latest-qErr path (latestQErrChiSelect) became the default
> and only routed qErr path. qVIR survives only as an offline diagnostic
> (`scripts/tdev_qerrab.py`, `scripts/proto_latest_qerr_chi.py`). This
> document is kept for historical/design context only — it no longer
> describes live code. See [`latest-qerr-chi-select.md`](latest-qerr-chi-select.md).

**Status:** RETIRED (was: design). Successor to the merged v1 router (PR #79). Open
question: drop the `internal` candidate? Raised by Charlie (#79 review,
2026-05-28) and Main (#79 review, 2026-05-29 18:47); validated by my
own routedQErrArm coupling finding. Validate any change in
`closedLoopServoSim` (Charlie) before hardware.

## Recap — v1 today

`OcxoTrustedGate` aside, `_route_ticc_arm` (do_freq_est.py) picks one
of three candidates per TICC edge, priority-ordered, first to pass
chi² ≤ 100:

| candidate | z | h | H | R | role |
|---|---|---|---|---|---|
| **external** | `ticc_diff + ext_qerr` | `−x[2]` | `[0,0,−1,0]` | `R_base` | clean x[2]-only, no x[0] coupling |
| **internal** | `ticc_diff` | `−x[2] − qerr(x[0])` | `[−1,0,−1,0]` | `R_base + R_lin` | today's coupled behavior; uses the FILTER's qerr(x[0]) estimate |
| **raw** | `ticc_diff` | `−x[2]` | `[0,0,−1,0]` | `R_base + tick²/12` | sawtooth-variance floor, always-accepted |

Lab- and sim-validated: PR #79 merged at 10a9500.

## Why revisit

Three observations the v1 router doesn't address well:

1. **The `internal` candidate IS the rx-TCXO leakage path.** Its
   `H = [−1,0,−1,0]` is exactly the coupling that bleeds `x[0]`'s
   uncertainty into `z_ticc`. Main's MadHat finding (day0529): without
   routing, `innov_ticc rms = 11 ns` and `lag-1 autocorr = +0.90`
   (red) — the leak signature; with routing forcing 97% to `raw` (via
   F10T qErr-match-miss), autocorr collapses to −0.08 (white). The
   structural fix is "stop using H=[−1,0,−1,0]," and v1 keeps it as
   the middle priority.

2. **chi² couples to filter state.** χ² = innov²/S where
   `S = HPHᵀ + R` — so the gate's threshold depends on the filter's
   own confidence. Two mistuning modes are now documented (Main's
   qFromCharPerActuator deeper analysis): an inflated Q over-grows
   P[2,2] and the chi² gate relaxes too far; the honest-Q floor
   under-grows it and the chi² gate over-rejects. The router's
   correlation-quality verdict is bracketed by Q-mistuning — exactly
   the same Q the longTauGnssCoupling coast-cap is sensitive to.

3. **qVIR is a validated measurement-quality signal already in the
   system.** `RunningVarianceWindow` lives in
   `peppar_fix_engine.py:774`; `qvir = Δvar(raw) / Δvar(raw+qerr)` is
   computed per epoch (`servo_ctx['qvir']`), and the project already
   uses the `qVIR > 1.5` litmus operationally (memory:
   project_ticc_qerr_epoch_matching, lab validation at 165×).  qVIR
   reads the actual variance of the corrected vs uncorrected stream —
   exactly the signal the router is trying to infer from chi²
   indirectly.

## Proposal — v2: two-candidate, qVIR-routed

Drop the `internal` candidate entirely. Two candidates per edge,
gated on the windowed qVIR:

| candidate | z | h | H | R |
|---|---|---|---|---|
| **external** | `ticc_diff + ext_qerr` | `−x[2]` | `[0,0,−1,0]` | `R_base` |
| **raw** | `ticc_diff` | `−x[2]` | `[0,0,−1,0]` | `R_base + tick²/12` |

Selector (per edge):

```
if ext_qerr is available and qvir_window > QVIR_THRESHOLD:
    candidate = external
else:
    candidate = raw
```

Both candidates have `H = [0,0,−1,0]` — neither couples `x[0]`.  The
rx-TCXO leakage path is structurally gone.  qVIR (windowed
measurement-quality) replaces chi² (filter-state-coupled
plausibility).  `QVIR_THRESHOLD` is the project's already-validated
litmus (1.5 today).

## The per-edge vs windowed trade-off (the main subtlety)

v1's chi² is per-edge.  qVIR is inherently windowed (N samples). Three
implications:

- **Onset of degradation lags.**  A receiver that starts to drift
  (e.g. F9T qErr-pipeline issue mid-run) takes ~N epochs of qVIR
  window to fall below threshold and trigger the route flip.  v1's
  chi² flips on the FIRST bad edge.  **Mitigation:** short rolling
  window (e.g. 60 s = 60 samples at 1 Hz) — fast enough to react,
  long enough to be statistically meaningful.
- **Single-bad-edge tolerance is actually a feature.**  A clean F9T
  with one bad sub-tick per minute looks bad to chi² each occurrence;
  the window-averaged qVIR correctly says "still mostly clean,
  keep ext."  Per-edge chi² over-rejects in that regime.
- **Bootstrap (window not yet full).**  Until enough samples,
  default-fall-to-raw: it's the always-safe candidate (no x[0]
  coupling, always-accepted with sawtooth-variance R).  v2 graceful
  degradation matches v1's "raw is the floor."

## Sole-observer behavior

The disciplineModeFsm increment #3 observability principle
(sole-carrier arms can't be soft-weighted) is unchanged.  Both v2
candidates use `H = [0,0,−1,0]`, so neither carries `x[0]` — the
sole-observer logic (force-admit on TICC-primary hosts) applies to
the Arm 4 update regardless of which v2 candidate is chosen.  No
interaction.

## What the existing v1 router got right (preserve)

- Default-off; opt-in via `--routed-qerr-arm`.
- `last_ticc_route` field + cumulative counters (PR #97) + arm-state
  CSV + `[ROUTED_QERR]` log.  All apply to v2 unchanged; the route
  set just collapses from {ext,int,raw} to {ext,raw}.
- Routes-to-raw automatically when qErr correlation fails (no
  per-receiver flag) — kept; qVIR is the new trigger.
- Sawtooth-variance R on raw (`tick²/12`, not `(tick/2)²`).

## Implementation sketch (when ready)

1. Add a per-edge qVIR signal source to `DOFreqEst.update`.  v1
   already accepts `ticc_qerr_ns`; add a `ticc_ext_correlated: bool`
   kwarg with the qvir-vs-threshold decision made engine-side.  The
   EKF stays oblivious to the windowing mechanism.
2. **Reuse `servo_ctx['qvir']` — the existing `RunningVarianceWindow`
   at `peppar_fix_engine.py:774` is the single source of truth for
   "is qErr correlated right now" (Main #98 review).  Do NOT
   instantiate a separate window in the router — saves cycles and
   prevents two-windows-disagreeing bugs.
3. New `_route_ticc_arm_v2(...)` (or fold into v1 behind a config):
   two candidates, return `'ext'` or `'raw'`.  No chi² internal to
   the router; the router's job is candidate SELECTION, the
   measurement update's own filtering happens downstream.
4. Engine: thread `ticc_ext_correlated = (servo_ctx['qvir'] > 1.5)`
   into `servo.update`.  `last_ticc_route` reports `'ext'`/`'raw'`
   only; route counters update accordingly.
5. Flag: keep `--routed-qerr-arm` (semantics unchanged: "use the
   external-qErr router") with a new `--router-version {v1,v2}` or
   `--router-qvir` knob.  Default v1 until v2 is sim-validated.

The two-candidate router collapses to ~30 lines vs v1's ~50.

### Legacy non-routed path — long-term fate

Main flagged this on #98 review: PR #97's observability noted "legacy
single-candidate path counts as `int` for parity," meaning hosts
running without `--routed-qerr-arm` (the current default) still
execute the internal-coupled `H = [−1,0,−1,0]` model — i.e., the
leakage path is renamed out of the router menu but lives on as the
default code path.  **v2 deprecates that path too:** the long-term
migration is

> v2 ships with `--router-qvir` opt-in → sim A/B clears → lab A/B
> clears → flip `--routed-qerr-arm` default to on → after a stage
> window, decommission the v1 router AND the legacy non-routed code
> path together.

Internal is truly gone only at the last step.  Until then, calling v2
"the fix" without addressing the default code path overstates the
shipped state.

## Validation plan

1. **Sim A/B in `closedLoopServoSim` (Charlie's PR #96 harness)** —
   sweep `ext_qerr_noise_ns` across F9T (~0.1 ns), F10T (~3 ns), and
   pathological (~tick) regimes.  v1 vs v2 metrics: `innov_rms`,
   `autocorr`, `x[2]_std`, `x[3]_std`, route fractions.  v2 must
   match v1 on all three regimes — clean ext → 100% ext, noisy ext
   → graceful raw fallthrough, pathological → 100% raw.  If any
   regression: don't ship.
2. **Bootstrap convergence-time check** (Main #98 review): on a
   clean-F9T config, the first ~60 s of the qVIR window are not yet
   full, so v2 routes to `raw` (safe, always-accepted) while qErr
   correction is delayed.  The bootstrap glide actuator is in
   initial-acquisition anyway, so this should be invisible — but
   measure bootstrap convergence time in the sim A/B and assert no
   regression vs v1.
3. **Lab A/B on MadHat** (Main coordinates).  Same hosts and
   procedure as #79's lab validation.
4. **Honesty check:** qVIR threshold sweep (1.5 today; what's the
   sensitivity?).

## Open questions

- **Hysteresis on the qVIR threshold?**  A receiver hovering at
  qVIR ≈ 1.5 could flap.  Cheap fix: hysteresis (require qVIR > 2.0
  to go ext-on, qVIR < 1.5 to go ext-off).  Or accept the flap (the
  two candidates differ only in R; the state-space trajectory should
  be smooth).
- **Per-receiver thresholds?**  F9T is reliably ~165×; F10T is ~1.0.
  A single 1.5 threshold works.  But a host-specific override might
  be useful for misbehaving receivers.  Probably wait until needed.
- **What about chi² as a SECOND gate, after the v2 routing?**
  Possible: route by qVIR, then chi²-gate the chosen candidate.
  Defends against per-edge outliers the window can't see.  Worth
  considering but adds back the filter-state coupling we just
  removed.  Hold for v2.1 if real data motivates.
- **Decommissioning v1.**  Stage-gated: v2 ships with `--router-qvir`
  opt-in; v1 stays default.  After 2-4 weeks of operational
  confidence, flip default; remove v1 code one release later.  Match
  the v2-regime-gate / disciplineModeFsm migration cadence.

## Recommendation

**Pursue v2.**  Three independent signals (Charlie, Main, my own
coupling test) agree the `internal` candidate is the leakage source
the router should bypass, not include.  qVIR is the correct quality
signal — already in the system, already validated, decoupled from
filter Q-mistuning.  The structural simplicity (two candidates, both
with H=[0,0,−1,0]) is its own justification.

Sequence: this note → sim A/B (Charlie) on the v2 selector vs v1 in
parallel → lab A/B → stage-gate `--router-qvir` default.
