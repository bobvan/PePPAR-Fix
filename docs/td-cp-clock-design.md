# Time-differenced carrier-phase clock estimation

**Status**: design doc, draft 2026-05-05
**Owner**: main
**Related**: I-145915 (PR/CP residual-gating port — landed today),
docs/time-filter-pr-cp-port.md (the related fault-detection port),
docs/qerr-correlation.md (qErr precision when correlated correctly),
docs/dofreqest-sign-chain.md (sign convention in the existing EKF)

## Why this exists

The time filter (`FixedPosFilter` in `solve_ppp.py`) estimates
receiver clock and ZTD residual from a mix of pseudorange (PR)
observations and time-differenced carrier-phase (TD-CP)
observations.  PR has metre-scale noise; TD-CP has sub-cm noise.
PR carries persistent per-SV biases (SSR phase-bias substitution,
antenna PCV, multipath, code-bias drift) that the filter has no
clean way to subtract — they leak into clock state.  TD-CP, by
construction, **cancels every constant per-SV bias** between
consecutive epochs and exposes only the *change* in clock + tropo +
geometry over that interval.

The PR/CP residual-gating port (I-145915, landed) addresses how the
*watchdog* detects faults — gate trips on TD-CP residual rather
than the PR-dominated mixed RMS.  This doc proposes the
*measurement-side* counterpart: clock estimation should be driven
by TD-CP after cold-start, with PR used only for the one-shot
absolute-clock seed at the first epoch.

The goal is sub-ns cross-host PPS agreement.  Tonight (post-servo
target=0 + pinned-position) the cross-host TICC pairs sit at
~30-50 ns std at τ=1 s with a ~9 ns mean.  That floor reflects
PR-domain noise leaking through the existing FixedPosFilter into
clock state.  TD-CP-only clock estimation pushes the floor toward
TICC's measurement noise (60 ps single-shot, ~0.18 ns at τ=1 s)
plus TIM-TP qErr correlation noise (~ns) plus rx-TCXO drift
between ties.

## Today's architecture (recap)

```
FixedPosFilter states:  [clock, clock_rate, ZTD_resid, ISB_GAL, ISB_BDS]

Per-SV per-epoch H rows:
  PR:    h[clock]=1, h[ZTD]=m_wet, h[ISB]=1     σ ≈ 1 m
  TD-CP: h[clock]=1, h[ZTD]=m_wet                σ ≈ 0.3 m weighted, sub-cm fit
         (TD differencing cancels per-SV ambiguity & ISB)

Output to DOFreqEst:  dt_rx_ns (the absolute clock state),
                      dt_rx_sigma_ns (its sigma)

DOFreqEst (4-state EKF) consumes dt_rx + raw TICC chA-chB to drive
adjfine via LQR.
```

The absolute clock state is updated by both PR and TD-CP rows.
PR-domain noise dominates the clock-state covariance and propagates
through the LQR into adjfine.

## The proposal: TD-CP-only clock estimation after cold-start

```
At cold-start (first epoch only):
  PR observations seed clock state via the existing median-of-PR-residual
  path (solve_ppp.py:1210-1240).  P[clock] is reset to (50 m)² or
  spread² — same as today.
  ISBs seeded from per-constellation PR median.

After cold-start (every subsequent epoch):
  PR rows are NOT used in the Kalman update.  Clock state is
  updated only by TD-CP rows.
  
  TD-CP observation per SV:
    Δφ_IF(t) - Δφ_IF(t-1)
      = Δrho_geo (computable, ~100 m/s × dt)
      + Δsat_clk (subtracted from corrections)
      + Δtropo = m_wet × ΔZTD (small, mm-scale per second)
      + Δrx_clk
      + noise (sub-cm per measurement)
  
  After subtracting computed terms, the residual is dominated by
  Δrx_clk plus tropo motion.  Filter assigns the change to clock
  state via h[clock]=1, h[ZTD]=m_wet.
  
  Slips are handled the same way as today: SV's prev_geo entry is
  cleared on cycle slip, so its TD-CP row is omitted from the next
  epoch's update.  Existing detectors (mw_jump, gf_jump, arc_gap,
  ubx_locktime_drop) cover this.
```

The clock state still tracks absolute clock — it just drifts from
the cold-start seed at the TD-CP rate-accuracy floor instead of
being repeatedly nudged by PR-domain noise.

## State design

States stay the same as today: `[clock, clock_rate, ZTD_resid,
ISB_GAL, ISB_BDS]`.  No removals; no additions.  What changes is
*which observations update which states*:

| State | Cold-start seed | Steady-state update |
|------|----------------|--------------------|
| clock | PR median (no change) | TD-CP only (new) |
| clock_rate | predict from clock-rate model + cold-start zero | TD-CP rate-of-change of innovation (effectively) |
| ZTD_resid | METAR target (I-024942) | TD-CP weak signal + METAR tie (I-024942) |
| ISB_GAL / ISB_BDS | PR median per-constellation | **frozen at cold-start value** (TD differencing cancels them, so no information from TD-CP) |

The ISB freeze is the cleanest part of the proposal: once ISBs are
pinned at the cold-start PR-median estimate, they don't drift on
PR-domain noise.  At the cost of not adapting to inter-system
clock-bias drift, which is typically << 1 ns per hour and below
our other noise floors.

## Cold-start handoff

Question: when does the engine flip from "PR + TD-CP" to "TD-CP only"?

Three triggers, OR-combined:
1. Σ_clock from FixedPosFilter < threshold (default 50 m → 5 m, say)
2. N consecutive epochs with TD-CP innovation count ≥ 4 (enough geometric diversity for clock-rate observability)
3. T elapsed since first epoch (sanity floor: 30 s minimum)

Once the flip happens, it doesn't unflip.  If a watchdog re-seeds
FixedPosFilter, the flip resets too (back to PR + TD-CP until the
next handoff condition).

## ZTD interaction

TD-CP barely sees ZTD: ΔZTD per second is mm-scale, m_wet ranges
1-5, so per-SV ΔZTD contribution is mm × m_wet ≈ a few mm per
second.  Below the TD-CP single-measurement noise (~5-10 mm with
elevation weighting).  Effectively, TD-CP-only operation makes ZTD
weakly observable from observations alone.

The existing METAR tie (I-024942, every 60 s, σ_target 30 mm)
becomes the dominant ZTD constraint.  This is fine — METAR is
authoritative atmospheric truth at the pressure-and-temperature
level, and the MULTI-host ZTD agreement we measured tonight (15 mm
spread across hosts) is already at the METAR uncertainty floor.

ZTD-target-unification (I-132038) becomes more important under
TD-CP-only: when both filters use METAR truth as the ZTD target,
ZTD becomes a known constant + small residual that the
clock-domain math doesn't need to disambiguate.

## ISB interaction

TD-CP cancels per-constellation ISBs in the difference, so ISB
states get **no information** from TD-CP rows.  Three options:

A. **Freeze ISBs at cold-start values** (recommended).  PR
   provides a one-shot estimate at the cold-start; thereafter the
   states are fixed, P[ISB] = 0, no Q.  Simple, low-risk.

B. **Apply ISBs as known biases** in observation construction
   (subtract from each SV's row at observation time).  Equivalent
   to (A) numerically; slightly more code; cleaner separation of
   "estimated states" vs "applied corrections".

C. **Re-seed ISBs periodically** by injecting a few PR rows every
   N minutes (every 60 s, like the METAR tie).  Catches ISB drift
   if there is any.  Adds complexity; uncertain whether the drift
   is real.

A is the right starting point.  Promote to C only if cross-host
PPS comparison reveals an ISB drift signature.

## Cycle-slip handling

TD-CP requires a SV's carrier phase from two consecutive epochs.
A slip in epoch N invalidates the (N-1, N) TD-CP row but does not
invalidate (N, N+1) — by epoch N+1 the SV has resumed clean phase
locks.  The existing `prev_geo[sv]` clear-on-slip pattern in
`FixedPosFilter.update` handles this: a missing prev_geo entry
omits the TD-CP row for that SV.

What changes under TD-CP-only: a slip-storm that knocks out many
SVs simultaneously could leave the filter with too few TD-CP rows
to observe clock state for an epoch.  Today's filter would still
have PR rows to fall back on; the new design must.  Two options:

A. **Reactivate PR rows when n_td < 4** for a single epoch.
   Conservative; preserves observability under sparse-CP.

B. **Predict-only when n_td < 4**, no measurement update that
   epoch.  Clock drifts at process-noise σ × dt for that epoch;
   acceptable for short gaps.

A is safer; B is cleaner.  Recommend A for the first deployment,
revisit after we have data on slip-storm sparsity.

## Validation plan

1. **Unit tests** in `test_fixedpos_td_cp.py`:
   - Cold-start seed unchanged (regression)
   - Steady-state update with PR rows turned off — clock state
     evolves at process-noise + TD-CP measurement rate; PR-only
     spikes don't affect clock
   - Slip handling: slipped SV's TD-CP row omitted; remaining SVs
     still update clock
   - n_td < 4 fallback path

2. **A/B replay** on a host's RAWX capture:
   - Run today's FixedPosFilter and the proposed FixedPosFilter
     side-by-side
   - Compare clock-state TDEV(τ) per host across τ ∈ [1, 10000] s
   - Expected: TD-CP-only sub-ns at τ=1 s vs current ~1 ns; bigger
     improvement at long tau

3. **Live A/B**: one host on TD-CP-only, two on current build.
   Cross-host TICC stability over an overnight should improve on
   the TD-CP host's pairs.  Compare derived "TD-CP host − control
   host" stability against derived "control host − control host".

Success criteria:
- Clock-state TDEV(1 s) drops by 5× or more
- Cross-host PPS agreement (std of 30-second running mean) drops
  from current ~30-45 ns to single-digit ns
- No regressions in convergence time or watchdog robustness

## Risks and open questions

- **Clock-rate observability under TD-CP only**: TD-CP measures
  Δrx_clk per epoch, which is the rate × dt.  Should be plenty
  of signal for clock_rate state, but we should verify the
  innovation covariance doesn't blow up at startup before the
  rate state has a baseline.

- **The cold-start handoff threshold**: σ_clock < 5 m is
  intuitive but not data-driven.  May need tuning.

- **Multi-second epoch gaps**: if the engine misses a few epochs
  (USB hiccup, e.g.), TD-CP over Δt > 1 s adds geometric
  uncertainty (SV motion × Δt).  Existing prev_geo pattern handles
  this by clearing prev_geo when too much time has elapsed (the
  arc_gap detector).  Need to verify the threshold is consistent
  with TD-CP-only operation.

- **DOFreqEst expects today's signal shape**: the EKF in
  do_freq_est.py was designed against the current FixedPosFilter
  output (dt_rx with σ from PR + TD-CP mix).  Under TD-CP-only,
  dt_rx_sigma will be much tighter; the EKF's R_ppp scaling needs
  to track this correctly or it'll over-trust the new dt_rx.

- **Interaction with I-145915 watchdog port**: the new watchdog
  trips on TD-CP residual.  Under TD-CP-only clock estimation,
  TD-CP residuals ARE the residuals — so the watchdog gates on
  the same signal that drives the filter.  This is desirable
  (single source of truth) but means watchdog-trip semantics
  shift slightly: a tripping TD-CP residual now means *the filter
  itself is making bad updates*, not just *PR is noisy and we're
  ignoring it*.

## Sequencing

This work depends on / interacts with:

- **I-145915** (PR/CP watchdog port) — landed today.  No conflict;
  the watchdog change moves to TD-CP-residual-driven trips, which
  is forward-compatible with the proposed TD-CP-only filter.

- **I-132038** (ZTD-target unification) — open.  Should land
  before TD-CP-only because ZTD becomes weakly observable under
  TD-CP and we need both filters agreeing on the same target.

- **I-023008** (servo error-source hierarchy) — open.  Should
  land after TD-CP-only because it changes what feeds the servo;
  doing both at once compounds the validation question.

Recommended landing order:
  1. (Today) I-145915 watchdog port — landed
  2. Tomorrow / week: I-132038 ZTD unification
  3. Later: this design (TD-CP-only) — implementation behind a
     feature flag with the cold-start handoff defaulting OFF;
     enable on one host first
  4. After stability validation: I-023008 servo hierarchy

## Implementation sketch

`scripts/solve_ppp.py FixedPosFilter`:
- New attribute `self._td_only_active: bool` (default False)
- New method `self._maybe_handoff_to_td_only()` — checks σ_clock,
  n_td, t_elapsed, sets `self._td_only_active = True` once
- In `update()`, after first-epoch seed and before building H_rows:
  if `self._td_only_active`, build only TD-CP rows from this
  epoch's observations (skip the PR-row construction loop body
  for the absolute-clock contribution).  PR rows still consumed
  for ISB freeze book-keeping if option (B) is chosen.

`scripts/peppar_fix_engine.py`:
- New CLI `--td-cp-clock` (default off).  When on, plumbs the
  flag into FixedPosFilter at construction time.

`scripts/peppar_fix/test_fixedpos_td_cp.py`: new test file with
the unit tests listed in the validation plan.

Estimated scope: ~50 LOC in solve_ppp.py + ~30 LOC of tests.
Half a day of work plus a couple of overnight runs to validate.
