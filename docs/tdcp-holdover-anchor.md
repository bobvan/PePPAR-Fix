# TDCP holdover anchor — Phase D design

Status: draft for review (bravo, 2026-05-23).  No code in flight.

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

**In scope**:

- Extended holdover budget: from ~60 s today to **hours**, for the
  failure modes where TDCP is still alive.
- Smooth holdover entry / exit (no servo punch on either boundary).
- Self-rcal: the calibration that lets us propagate DO phase
  during holdover updates *continuously* while Arms 3/4 are healthy,
  so it's fresh when holdover begins.
- L1+L2+L3 protection extends naturally — same slip-defense layers
  protect the TDCP path during holdover as during normal operation.

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
grows with holdover duration; eventually σ exceeds the moonshot
per-clock budget and the engine should transition to a noisier
fallback (or alarm).

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

## Interaction with existing systems

- **Temperature memory** (`project_temp_freq_holdover.md`): the
  DO-vs-rx_TCXO offset has a strong temperature dependence (cable
  thermal expansion).  Future v2 can record `(temperature, offset)`
  pairs and predict the offset during HOLDOVER if temperature is
  changing.  Out of scope for Phase D v1.
- **clockClass reporting**: today HOLDOVER → ptp4l clockClass 7
  (via `ptp4l-supervision.md`).  Phase D's HOLDOVER preserves this
  but lengthens the duration before clockClass falls to FREERUN.
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
  T_blend.
- **Integration**: deliberately pull the DO PPS cable on a TimeHat
  or PiFace test host while the engine is running.  Verify the
  engine remains within budget for ~10 minutes and recovers cleanly
  when the cable is reconnected.  Compare against the same scenario
  with Phase D off (today's behavior) — should hit ~60 s before
  divergence.
- **Long-duration**: 1-hour cable-pull, verify the disciplined-DO
  output stays within the moonshot per-clock budget (354 ps RMS
  at all τ from 0.1 s to 1000 s).

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
| `holdover_max_s` | 7200 s | Hard cap; beyond this, drop to clockClass FREERUN |

All exposed as engine CLI flags so we can iterate without code
changes.

## Open questions for reviewers

1. **Single Arm 5 σ for both TRACKING and HOLDOVER, or separate?**
   - During TRACKING, Arm 5 mostly tightens `x[1]`.
   - During HOLDOVER, the integrated phase is the load-bearing
     metric.  Conservative σ during HOLDOVER (say 2× TRACKING)
     biases toward keeping the predict-step's process noise visible.
2. **Where does the `HoldoverActor` live?**
   - Inside `do_freq_est.py` keeps it close to the EKF/LQR but
     bloats that file.
   - Sibling file is cleaner but the wiring at the engine layer
     becomes more complex.
   - Lean: sibling file `scripts/peppar_fix/holdover_actor.py`.
3. **Should we persist `DoVsRxTcxoOffset.last_offset` to disk?**
   - Lets HOLDOVER survive engine restart.  Useful for crash
     recovery + scheduled reboots.
   - Adds a state file + cache-coherency concerns.
   - Lean: v1 in-memory only; v2 add disk persistence after
     measurement experience.
4. **Phase D as one PR or stack?**
   - One PR: easier to review the whole design.  Big diff.
   - Stack: `DoVsRxTcxoOffset` → `TdcpPhaseIntegrator` →
     `HoldoverActor` → engine wiring.  4 small PRs in dependency
     order.
   - Lean: stack.  Aligns with Phases A/B/C precedent.

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
