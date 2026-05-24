# TDCP holdover anchor — Phase D design

Status: v2 — revised per charlie + main reviews (bravo, 2026-05-23).
Open questions resolved; σ-growth analysis added; holdover_max_s
replaced with σ-threshold transitions.  No code in flight.

## Motivation

Today's "holdover" support in `peppar-fix` is minimal:

- On Arm 3 (EXTINT) or Arm 4 (TICC) loss, the existing `holdover`
  ctx slot trips; the engine logs `[STATE] DOFreqEst: tracking →
  holdover` and **preserves the last applied adjfine** while the
  observation stream is absent.  See `feedback_temp_freq_holdover.md`
  and `docs/future-work.md`.
- No software-driven phase prediction during holdover — the DO
  just runs open-loop at its last commanded frequency.
- Effective holdover budget today is ~60 s (per the SatPulse-style
  dual-EMA design noted in future-work) before adjfine error
  exceeds an acceptable per-clock budget.

Bob's framing in the PR #61 thread (2026-05-23 conversation):

> if we don't have a phase reference like EXTINT or TICC, can we
> ever build a software phase reference?  ... am I right in thinking
> we always need Arm 3 or Arm 4 to get the hardware phase reference
> into the filter, at least at startup?  And if that's true, then
> the only possible benefit I see of building an anchor from TDCP
> is holdover.

Answer:
- **Yes, software cannot construct an absolute DO phase reference
  without at least one hardware observation of DO PPS.**  The DO
  is a physical oscillator separate from the rx-TCXO; software can
  *propagate* phase forward via a frequency model, but the initial
  value and any long-term anchor must come from hardware (Arm 3,
  Arm 4, or an equivalent path).
- **Holdover is the main use case for a TDCP-derived phase anchor.**
  Secondary benefits (bias cross-check, earlier servo activation,
  PPP-trip resilience) exist but are smaller.

Phase D builds the infrastructure to use Arm 5's TDCP signal as a
long-term phase anchor *during holdover* — i.e., when Arms 3 and
4 are temporarily unavailable.

## Goals + non-goals

**Framing**: we will do well for short holdover.  We will fail for
infinite holdover.  The job is to know *how well we're doing* as
holdover progresses, so we can signal our confidence downstream:

- PTP `clockClass` transitions at defined σ thresholds
- Future hardware: mute DO PPS OUT when confidence is gone

We do NOT promise a fixed holdover budget ("X hours").  We report
σ(t) honestly and let downstream consumers decide their tolerance.

**In scope**:

- Extended holdover from ~60 s today to **minutes on OCXO-class,
  degrading gracefully** — for failure modes where TDCP is alive.
- Honest σ(t) reporting: the engine must know and expose how its
  holdover phase uncertainty grows with time (see σ-growth analysis
  below).
- PTP clockClass transition thresholds tied to σ(t) — today
  HOLDOVER → clockClass 7 immediately; Phase D makes that
  transition σ-aware, and adds a later transition to clockClass 52
  (FREERUN) when σ crosses a harder threshold.
- Smooth holdover entry / exit (no servo punch on either boundary).
- Self-rcal: the calibration that lets us propagate DO phase
  during holdover updates *continuously* while Arms 3/4 are healthy,
  so it's fresh when holdover begins.
- L1+L2+L3 protection extends naturally — same slip-defense layers
  protect the TDCP path during holdover as during normal operation.
- **Future**: DO PPS OUT muting when σ exceeds the excursion bound
  (requires hardware gate on PPS OUT, not available on current lab
  hosts).  Phase D builds the σ-tracking infrastructure; threshold
  policy is separate.

**Out of scope**:

- GNSS sky loss.  TDCP depends on the sky.  When sky is gone,
  Arms 1, 3, 5 all die simultaneously; only Arm 4 (TICC chA-chB)
  can keep running, and even then only synchronizing DO to
  rx-TCXO (both unanchored).  This case is the territory of the
  temperature-compensated holdover (`project_temp_freq_holdover.md`),
  not TDCP.
- Arm 1 (PPP) replacement.  TDCP-integrated phase could in principle
  substitute for PPP `dt_rx`, but that's a different architectural
  question — Phase E or later.
- Long-term bias removal of TDCP itself (cable bias, ephemeris
  ambiguity).  Best handled as an innovation-based R-rcal in a
  separate workstream once we have enough live data to characterize.

## Failure-mode coverage

| Failure | Arms 1 / 3 / 4 status | TDCP (Arm 5)  | Phase D helps? |
|---|---|---|---|
| DO PPS cable break | A3 silent, A4 silent | alive | **yes** |
| TICC failure (`/dev/ticc1` unplug) | A4 silent | alive | **yes** |
| F9T sky loss | A1 + A3 + A5 silent | dead | no (temp-compensated holdover) |
| PPP filter trip / ZTD storm | A1 unreliable | alive | yes (continuous use) |
| Bootstrap re-bootstrap (exit code 5) | all reset | alive after fresh seed | yes (faster re-anchor) |
| Engine restart | all reset | alive after fresh seed | partial (continuous capture across restart) |

The two big wins are the first two rows: hardware-side faults that
take out Arms 3/4 but leave the F9T's sky-side observations intact.

## State machine

Four operating modes for the DO discipline path:

```
                ┌────────────────────────────────┐
                │ TRACKING                       │
                │ Arms 3/4 healthy, x[2] from    │
                │ hardware; TDCP runs but Arm 5  │
                │ only updates x[1].  Capture    │
                │ rolling DO-vs-rx_TCXO offset.  │
                └─────────────┬──────────────────┘
                              │ Arms 3 + 4 absent for > T_enter (e.g., 5 s)
                              ▼
                ┌────────────────────────────────┐
                │ HOLDOVER                       │
                │ x[2] propagated from TDCP-     │
                │ integrated rx_TCXO phase + the │
                │ last captured DO-vs-rx_TCXO    │
                │ offset.  L1+L2+L3 active.      │
                └─────────────┬──────────────────┘
                              │ Arms 3 or 4 return + healthy
                              ▼
                ┌────────────────────────────────┐
                │ REACQUIRED                     │
                │ Hardware obs streaming again.  │
                │ Blend TDCP-derived x[2] toward │
                │ hardware-observed x[2] with    │
                │ time-constant T_blend.         │
                └─────────────┬──────────────────┘
                              │ Blend converged (< σ_blend)
                              ▼
                ┌────────────────────────────────┐
                │ TRACKING (steady)              │
                └────────────────────────────────┘
```

Transitions are observable in the engine log via existing
DOFreqEstState transitions; this design adds two sub-states
(`HOLDOVER`, `REACQUIRED`) under the umbrella state.

## Architecture

### New components

1. **`DoVsRxTcxoOffset`** (new file
   `scripts/peppar_fix/do_vs_rxtcxo_offset.py`)

   While in TRACKING:
   - Every servo epoch, capture the *current* DO-vs-rx_TCXO offset:
     `offset = x[2] - x[0]` (DO phase from GPS minus rx_TCXO phase
     from GPS).
   - Smooth via an EMA with τ ~ 60 s.  Long enough to absorb
     Arm-3/4 noise; short enough to track real drift.
   - Expose: `last_offset_ns`, `last_offset_sigma_ns`, `last_capture_t`.

2. **`TdcpPhaseIntegrator`** (new file
   `scripts/peppar_fix/tdcp_phase_integrator.py`)

   Maintains an integrated rx_TCXO phase from TDCP `df_f` deltas:
   - Seeded at HOLDOVER entry from the filter's current `x[0]`
     (which during TRACKING is anchored by Arm 1 PPP and/or
     informed by Arm 5 via covariance).
   - Each epoch: `phase += df_f * dt`.
   - Drops dt > `max_dt_s` (gap protection — re-seed at next
     hardware re-acquisition).

3. **`HoldoverActor`** (state-machine glue, in `do_freq_est.py` or
   a sibling file)

   Owns the TRACKING / HOLDOVER / REACQUIRED transitions and
   bridges the existing DOFreqEstState machinery.

### State propagation during HOLDOVER

When in HOLDOVER, the EKF runs as today but with Arm 3 and Arm 4
inputs absent.  Arm 5 still feeds `x[1]`.  We synthesize a
*pseudo-observation* on `x[2]`:

```
x[2]_pseudo = TdcpPhaseIntegrator.phase + DoVsRxTcxoOffset.last_offset
σ_x[2]_pseudo² = TDCPIntegrator drift variance(dt_holdover)
                + DoVsRxTcxoOffset.σ²
                + crystal-drift uncertainty growth × dt²
```

This pseudo-observation enters as a synthetic "Arm 6" (or as a
re-purposed Arm 3 with widened σ — implementation detail).  σ
grows with holdover duration per the analysis below; the engine
reports σ(t) continuously and ties downstream signals (PTP
clockClass, future PPS muting) to threshold crossings.

### REACQUIRED → TRACKING blend

When Arms 3 or 4 return, we don't snap directly back — that would
inject a step in `x[2]`.  Instead:

- For T_blend (default 30 s), accept both the hardware
  observation (Arms 3/4) and the integrator's pseudo-observation
  into the EKF with their respective σs.  The Kalman fusion
  naturally weights them by R.
- σ of the integrator grows during HOLDOVER, so by the time
  REACQUIRED begins, the hardware observation typically
  dominates.  Blend is a few-epoch transition, not a hard switch.
- Exit REACQUIRED when `P[2,2]` returns below `σ_blend_done`
  (default ~1 ns² for OCXO hosts).

## Calibration: what gets captured during TRACKING

The DO-vs-rx_TCXO offset is the load-bearing calibration.  It's
the difference between two phase reference points:

- `x[0]` = rx_TCXO phase from GPS time (anchored by Arm 1 PPP)
- `x[2]` = DO phase from GPS time (anchored by Arms 3 or 4)

In TRACKING, both anchors are alive, so `offset = x[2] − x[0]`
is well-defined.  Drift sources on this offset:

- **DO-side cable bias drift**: PPS cable length changes (thermal
  expansion).  ~ps/°C on coax; days-to-weeks timescale.
- **F9T internal rx_TCXO drift**: ~3 ppb/hr on F9T-20B per our
  2026-05-23 measurement.  Integrates into `x[0]`, but Arm 1 keeps
  `x[0]` anchored in TRACKING.  In HOLDOVER (Arm 1 still alive),
  Arm 5 keeps `x[1]` tight and the integrated x[0] tracks GPS.
- **GPS broadcast eph error**: bounded, sub-meter; impacts
  multiple SV residuals symmetrically.  Bundled into TDCP's
  intrinsic noise floor; not a holdover-specific concern.

The EMA's τ ~ 60 s smooths out epoch-to-epoch noise.  This is the
calibration that gets *frozen* at HOLDOVER entry.

## Holdover σ growth analysis

We will do well for short holdover.  We will fail for infinite
holdover.  This section makes explicit *where the crossover is*
for each DO class, so we don't oversell.

### Error sources during holdover

Three independent contributions to `σ(x[2]_pseudo)`:

**S1 — TDCP integrated phase noise (random walk).**  TDCP gives
per-epoch frequency estimates with noise σ_y.  Integrating
frequency to maintain rx_TCXO phase accumulates as:

```
σ_tdcp(T) = σ_y × √T     [ns, with σ_y in ppb, T in seconds]
```

From the 2026-05-23 validation gate (PiFace/TimeHat/clkPoC3 from
the Phase A morning gate; MadHat from Main's separate 1.02 h
F10T offline replay at 09:27 CDT):

| Host | σ_y (ppb) | σ_tdcp(60 s) | σ_tdcp(300 s) | σ_tdcp(3600 s) |
|---|---|---|---|---|
| PiFace (F9T-20B) | 0.026 | 0.20 ns | 0.45 ns | 1.56 ns |
| TimeHat (F9T-10) | 0.051 | 0.40 ns | 0.88 ns | 3.08 ns |
| clkPoC3 (F9T-20B) | 0.057 | 0.44 ns | 0.99 ns | 3.43 ns |
| MadHat (F10T) | 0.065 | 0.50 ns | 1.12 ns | 3.90 ns |

**Caveat**: `σ_y × √T` assumes white-FM noise, which matches the
prototype's short-τ TDEV slope.  At longer τ the prototype showed
TDEV ∝ τ^1.6 (from 15 ps at 1 s to 2.3 ns at 30 s — steeper than
the τ^0.5 white-FM prediction of 82 ps at 30 s).  This implies a
flicker-FM or RW-FM component that grows faster than √T.  S1 is
therefore a **lower bound** on TDCP integrated noise at long T;
actual could be 20-50% higher at T > 300 s.  This doesn't change
the 354 ps crossover (S3 dominates there anyway) but means the
1-hour S1 column is optimistic by up to 50%.

This is the *minimum* contribution — TDCP integration noise alone,
even if the frozen offset were perfect.

**S2 — Frozen offset uncertainty at holdover entry.**  The EMA
captures `offset = x[2] − x[0]` with noise dependent on EMA τ and
Arm 3/4 observation noise:

```
σ_offset ≈ √P[2,2] at holdover entry    [typically 0.1-1 ns]
```

This is constant (doesn't grow with T) — it's the initial condition
uncertainty.  Negligible relative to S1 beyond ~30 s.

**S3 — Differential DO-vs-rx_TCXO frequency drift (systematic).**
The frozen offset assumes the DO and rx_TCXO maintain the same
frequency relationship they had at holdover entry.  They don't:

- The DO runs at fixed adjfine.  Its actual frequency drifts from
  temperature changes and aging.
- The rx_TCXO drifts independently (different crystal, different
  thermal mass, different tempco).
- TDCP tracks the rx_TCXO's motion (that's what Arm 5 measures)
  but is blind to the DO's drift.

This produces a *systematic* phase error that grows linearly (or
worse, if temperature is changing):

```
Δx[2]_drift(T) = δf_differential × T    [ns, with δf in ppb]
```

where `δf_differential` is the unmeasured differential frequency
walk between DO and rx_TCXO during holdover.

Bounding δf_differential is the hard part:
- OCXO in a stable thermal environment: δf ≈ 0.01-0.05 ppb/min
  (dominated by rx_TCXO tempco; OCXO tempco is 10-100× lower)
- OCXO during a thermal transient: δf ≈ 0.1-1 ppb/min
- TCXO-class DO: δf ≈ 0.5-5 ppb/min (both crystals responding
  to temperature with comparable magnitude but different time
  constants)

### Combined σ(t) — OCXO-class hosts (PiFace, clkPoC3)

Conservative case (δf_differential = 0.05 ppb/min = 0.83 ppt/s,
thermally quiet lab):

| T (holdover) | S1 (TDCP noise) | S3 (drift) | Combined σ | vs 354 ps budget | vs 1 ns bound |
|---|---|---|---|---|---|
| 30 s | 0.14 ns | 0.03 ns | **0.14 ns** | within | within |
| 60 s | 0.20 ns | 0.05 ns | **0.21 ns** | within | within |
| 2 min | 0.28 ns | 0.10 ns | **0.30 ns** | within | within |
| 5 min | 0.45 ns | 0.25 ns | **0.51 ns** | **EXCEEDS** | within |
| 10 min | 0.64 ns | 0.50 ns | **0.81 ns** | exceeds | within |
| 30 min | 1.10 ns | 1.50 ns | **1.86 ns** | exceeds | **EXCEEDS** |
| 1 hr | 1.56 ns | 3.00 ns | **3.38 ns** | exceeds | exceeds |

Aggressive case (δf_differential = 0.2 ppb/min, thermal transient):

| T | S1 | S3 | Combined σ | vs 354 ps | vs 1 ns |
|---|---|---|---|---|---|
| 60 s | 0.20 ns | 0.20 ns | **0.28 ns** | within | within |
| 2 min | 0.28 ns | 0.40 ns | **0.49 ns** | **EXCEEDS** | within |
| 5 min | 0.45 ns | 1.00 ns | **1.10 ns** | exceeds | **EXCEEDS** |
| 10 min | 0.64 ns | 2.00 ns | **2.10 ns** | exceeds | exceeds |

**Headline**: on an OCXO-class host in a quiet lab, TDCP holdover
stays within the moonshot per-clock budget (354 ps) for ~2-5 minutes
and within the shared-antenna excursion bound (1 ns) for ~10-30
minutes.  These ranges compress during thermal transients.

Charlie's estimate of "5-15 minutes before exceeding 354 ps" is
confirmed — the dominant term beyond ~2 min is S3 (differential
drift), not S1 (TDCP noise).

### Combined σ(t) — TCXO-class hosts (TimeHat, MadHat)

For TCXO-class DOs, both the DO and rx_TCXO have comparable tempco
(~1e-9), so δf_differential can be larger (0.5-5 ppb/min) and S1
is also noisier.  Budget for the 354 ps moonshot bound is already
unachievable for TCXO hosts (per CLAUDE.md), so holdover buys little.
Best-effort only; might extend the 60 s open-loop to ~2-3 minutes.

### PTP clockClass and PPS muting thresholds

The σ(t) analysis gives us three natural transition points:

| σ threshold | PTP clockClass | PPS action | Meaning |
|---|---|---|---|
| σ ≤ 354 ps | 6 (locked, high accuracy) | PPS OUT active | within moonshot per-clock budget |
| 354 ps < σ ≤ 1 ns | 7 (holdover) | PPS OUT active | degraded but within excursion bound |
| 1 ns < σ ≤ 10 ns | 52 (holdover, degraded) | PPS OUT active (future: consider muting) | useful for loose-tolerance consumers |
| σ > 10 ns | 248 (FREERUN) | **PPS OUT muted** (future hardware) | no better than uncorrected |

Phase D v1 implements σ(t) tracking and log reporting.  Phase D v2
wires it to PTP clockClass transitions via `ptp4l-supervision.md`.
PPS muting requires hardware support (GPIO gate on PPS OUT, not
present on current lab hosts).  The threshold policy will be tuned
from live holdover cable-pull experiments.

The old `holdover_max_s = 7200` hard cap is replaced by σ-threshold
transitions — there is no fixed timeout.  The engine stays in
HOLDOVER as long as TDCP is alive, reporting σ honestly; downstream
consumers decide their tolerance.

### Prerequisite: TDCP bias measurement during TRACKING

(Charlie's concern #2, accepted.)

In TRACKING, any sustained TDCP bias is masked by Arms 3/4 — the
Kalman fusion weights them appropriately and the bias stays buried.
In HOLDOVER, the TDCP path becomes load-bearing: a sustained bias
of 0.01 ppb integrates to 36 ns over 1 hour.

Before Phase D code lands, we need an empirical measurement:
compare Arm 5's TDCP-derived x[1] against Arm 4's TICC-derived x[1]
over a 4+ hour TRACKING window on each host, and extract the mean
bias and its temporal structure.  Sources of bias include:

- Residual ionospheric delay in undifferenced carrier phase
- Ephemeris errors that don't fully cancel under time-differencing
- Satellite geometry-dependent systematic residuals

If the bias is ≤ 0.001 ppb (integrates to 3.6 ns over 1 hr), it's
below S3 and doesn't change the analysis.  If it's larger, it
becomes a floor on σ(x[2]_pseudo) during HOLDOVER.

The measurement uses existing infrastructure: `--arm-state-log`
records per-epoch arm contributions to x[1]; a simple offline
script computes mean(Arm5_x1 − Arm4_x1) over the capture.  No
new code required, just lab time.

## Interaction with existing systems

- **Temperature memory** (`project_temp_freq_holdover.md`): the
  DO-vs-rx_TCXO offset has a strong temperature dependence (cable
  thermal expansion).  Future v2 can record `(temperature, offset)`
  pairs and predict the offset during HOLDOVER if temperature is
  changing.  Out of scope for Phase D v1.
- **clockClass reporting**: today HOLDOVER → ptp4l clockClass 7
  (via `ptp4l-supervision.md`).  Phase D replaces the
  fixed-timeout clockClass cascade with σ-threshold transitions
  (see "PTP clockClass and PPS muting thresholds" above).  The
  engine reports current σ(t) in its UDS interface; the ptp4l
  supervision layer maps σ to clockClass.
- **PHC step / re-bootstrap (exit code 5)**: orthogonal — the
  `HoldoverActor` resets at engine restart.  Capture continuity
  across restart requires persisting `DoVsRxTcxoOffset.last_offset`
  to disk (e.g., `state/dos/<do_uid>.holdover.json`) — Phase D v2.

## Validation

Following the same pattern as Phases B and C:

- **Unit**: synthetic timeline (clean → HOLDOVER trigger → clean
  return).  Verify (1) `x[2]` continuity at HOLDOVER entry,
  (2) drift rate during HOLDOVER matches the crystal-drift model,
  (3) no `x[2]` step at REACQUIRED entry, (4) blend converges in
  T_blend, (5) σ(t) reporting grows at the predicted rate.
- **σ characterization**: 4+ hour TRACKING capture on PiFace and
  clkPoC3 with `--arm-state-log`, measuring TDCP-vs-Arm4 bias
  (Phase D prerequisite).  Extract δf_differential empirically
  rather than relying on the estimates above.
- **Integration**: deliberately pull the DO PPS cable on PiFace
  while the engine is running.  Verify the engine reports σ(t)
  accurately and stays within budget for the duration predicted by
  the σ-growth table.  Compare against Phase D off (today's
  behavior) — should hit ~60 s before divergence.
  Expected: TDCP holdover extends useful operation to ~5 min
  (within 354 ps budget) to ~30 min (within 1 ns) on OCXO-class
  in a quiet lab.  Do NOT claim "hours" without validating S3
  empirically.
- **Threshold validation**: verify PTP clockClass transitions
  fire at the correct σ crossings during the cable-pull test.

Required hardware: any single host with Arms 3 or 4 (PiFace,
TimeHat, clkPoC3).  Two hosts for cross-host pair-excursion check
during HOLDOVER (the real moonshot test for this design).

## Tuning knobs

| Parameter | Default | Notes |
|---|---|---|
| `T_enter` | 5 s | Arm 3/4 absent for this long → HOLDOVER |
| `τ_offset_ema` | 60 s | DO-vs-rx_TCXO offset smoothing |
| `max_dt_s` | 1.5 s | Integrator gap-protection (drop diff if exceeded) |
| `T_blend` | 30 s | REACQUIRED → TRACKING blend |
| `σ_blend_done` | 1 ns² | REACQUIRED exit condition |
| `σ_clockclass_7` | 354 ps | σ(t) above this → clockClass 7 (holdover) |
| `σ_clockclass_52` | 1 ns | σ(t) above this → clockClass 52 (holdover, degraded) |
| `σ_freerun` | 10 ns | σ(t) above this → clockClass 248 (FREERUN) |
| `σ_pps_mute` | 10 ns | σ(t) above this → mute PPS OUT (future hardware) |

All exposed as engine CLI flags so we can iterate without code
changes.  The previous `holdover_max_s = 7200` hard cap is removed;
σ-based transitions handle quality degradation continuously.

## Resolved design questions

(Closed by main + charlie reviews, 2026-05-23.)

1. **Single Arm 5 σ for both TRACKING and HOLDOVER.**
   Charlie's argument: let the integrator's process noise model
   carry the uncertainty growth, not a manually-widened R.  If
   Arm 5 is genuinely noisier during HOLDOVER, measure it.
   Main concurred with separate-σ but the σ-growth analysis above
   makes the point moot — `σ(x[2]_pseudo)` already grows via S1+S3
   regardless of Arm 5's R.  **Decision: single σ.**

2. **`HoldoverActor` in a sibling file**
   (`scripts/peppar_fix/holdover_actor.py`).  Both reviewers agreed.

3. **`DoVsRxTcxoOffset` v1 in-memory only.**  Disk persistence
   deferred to v2 after we know whether engine-restart-during-
   HOLDOVER is a real operational need.  Both reviewers agreed.

4. **4-PR stack**: `DoVsRxTcxoOffset` → `TdcpPhaseIntegrator` →
   `HoldoverActor` → engine wiring.  Matches A/B/C precedent.
   Both reviewers agreed.

## Cross-references

- `docs/tdcp-servo-integration.md` — Phase B/C reference design
- `docs/future-work.md` — earlier holdover sketch + SatPulse pointer
- `feedback_temp_freq_holdover.md` — Bob's note on preserving last
  adjfine
- `docs/clock-state-modeling.md` — how holdover falls out of the
  EKF predict step naturally
- `docs/architecture-vision.md` — the broader holdover framing
- `docs/ocxo-platform-matrix.md` — DPLL-level holdover for the
  E810 case (different beast — hardware-side DPLL holdover; this
  doc is about software-side)
