# TDCP servo integration — design

> *Filed 2026-05-23 in response to dayplan `tdcpServoIntegration-bravo`
> review (main ✓, charlie ✓ in spirit).  Companion to the
> read-only prototype on branch `bravo/tdcp-prototype` and the
> findings memo at `prototypes/tdcp-findings-2026-05-22.md`.
> Phase A code does not start until this doc is reviewed.*

## TL;DR

Add time-differenced carrier phase (TDCP) as **Arm 5** of the existing
DOFreqEst measurement ladder ([dofreq-est-measurement-ladder.md]).  It
observes the same state qErr already does — `x[1] = rx_TCXO frequency
offset` — but with measurement noise ~1-2 orders cleaner.  The Kalman
fusion handles the weighting; running both arms in production is the
"competition" Bob asked for, with the K-gain log-line as the
running scoreboard.

```
EKF state vector (unchanged):
  x[0] = rx_tcxo phase offset from GPS time   (ns)
  x[1] = rx_tcxo frequency offset from GPS    (ppb)   ← TDCP observes
  x[2] = DO phase offset from GPS time        (ns)
  x[3] = DO frequency offset from GPS         (ppb)

Existing arms (after 2026-05-22 overnight tuning):
  z_ppp     :  observes  x[0]                       σ ~ 0.1 ns
  z_qerr    :  observes  x[1]                       σ ~ 0.3 ppb
  z_extint  :  observes  x[2]                       σ ~ 5–10 ns
  z_ticc    :  observes  −x[0]·∂qerr/∂x − x[2]      σ ~ 60 ps

New arm (this doc):
  z_tdcp    :  observes  x[1]                       σ ~ 0.026 ppb
```

## Why this slots in cleanly as Arm 5

The prototype's measured TDEV(1s) ≈ 15 ps is the σ of the
ensemble-derived **phase** at τ = 1 s.  To use it as Arm 5's
R-matrix entry we need σ_y, the fractional-frequency noise.  For
white-FM-dominated noise (the regime TDCP measures on F9T data):

  TDEV(τ) ≈ σ_y · √(τ / 3)   ⇒   σ_y = TDEV(τ) · √(3 / τ)

  At τ = 1 s:  σ_y = 15e-12 · √3 ≈ **2.6e-11 ≈ 0.026 ppb**

(An earlier draft used the MDEV/τ ratio of 0.015 ppb, missing the
√3 noise-model factor.  Fixed; the headline survives at a
12–15× ratio against qErr.)

qErr today sits at "sub-ppb" — call it 0.3 ppb effective σ after
the time-averaging window.  So z_tdcp is **~12× more precise than
z_qerr on the same state**.

When two arms observe the same state with different R values, the
Kalman fusion gives an optimal-weighted estimate.  K-gain for the
sharper arm dominates.  No design choice required at integration
time — the math sorts it out.  This is exactly the competition
Bob asked for, and we can read the scoreboard directly:

  * Log K_5[1] vs K_2[1] per epoch.  Ratio ≫ 1 ⇒ TDCP carries it.
  * Run `--no-tdcp-arm` to ablate and re-measure chA TDEV.
  * Differential analysis: does adding Arm 5 change chA TDEV(τ)?
    By how much?  At which τ?  That's the answer.

(For the per-SV-σ-from-MAD path — 4.2 mm = 14 ps per SV, ensemble
σ = 14 / √16 = 3.5 ps — that's the *estimator's* noise floor, well
below the 15 ps the prototype actually observed.  The 15 ps is real
rx-TCXO motion on top.  The right number to feed R is therefore
the observed TDEV converted via the formula above, not the
estimator floor.)

## What TDCP is NOT measuring (Charlie's reframe)

**The 15 ps frequency-observation σ_x1 and the 79 ps chA TDEV are
different physical quantities.**  15 ps is TDCP's measurement
noise on `x[1]` (rx-TCXO frequency, expressed as the equivalent
τ = 1 s phase deviation).  79 ps is the phase TDEV of the
disciplined DO output as measured on TICC chA.  The latter is what
downstream users see; the former is what TDCP improves as an
observation of one of the model's internal states.

In the CLAUDE.md "two oscillators bound the result" framing, the
15 ps is the **RX TCXO measurement-chain ceiling** — the second of
the two limits, the one bounding how cleanly we can observe GPS
time from the receiver's perspective.  It is **not** a
disciplined-DO output target.  Future readers should not anchor
chA expectations to "15 ps".

What TDCP buys us is **cleaner observation of x[1]**.  Whether
that translates to chA improvement depends on whether x[1] is on
the critical path for chA today.  After main's 2026-05-22 finding,
chA discipline is carried by Arm 4 (TICC chA-chB + qErr) which
observes a function of `x[0]` and `x[2]`.  TDCP enters through
`x[1]` — see the next section for the propagation path.

## How an x[1] observation reaches x[3] / adjfine

The DOFreqEst predict step couples states over time:

```
x[0]_{k+1} = x[0]_k + x[1]_k · Δt + w_phase     (rx-TCXO phase ← integrates freq)
x[1]_{k+1} = x[1]_k             + w_freq        (rx-TCXO freq, RW with Q_clk_rate)
x[2]_{k+1} = x[2]_k + x[3]_k · Δt + w_do_phase  (DO phase ← integrates DO freq)
x[3]_{k+1} = x[3]_k             + w_do_freq     (DO freq, RW with Q_do_rate)
```

The measurement model of Arm 4 (TICC chA-chB + qErr) couples
`x[0]` and `x[2]` in a single innovation: `z_ticc ≈ −x[0]·∂qerr/∂x
− x[2]`.  Adjfine reads `x[3]`.

The chain by which Arm 5's tightening of `x[1]` propagates to
`x[3]`:

1. **Predict step**: tighter posterior on `x[1]` ⇒ tighter
   predicted `x[0]` at the next step (because `x[0]_{k+1}` depends
   on `x[1]_k`).
2. **Arm 4 update**: the next z_ticc arrives.  Its innovation is
   processed against the (now tighter) prior in `x[0]`.  The
   Kalman gain `K_4 = P H^T (HPH^T + R_ticc)^{-1}` reads the
   off-diagonal `P[3, 2]` and `P[3, 0]` covariances — tighter
   `x[0]` prior implies the actual TICC innovation is attributed
   more cleanly to `x[2]` and (through `P[3, 2]`) to `x[3]`.
3. **Adjfine**: LQR reads the updated `x[3]`.

Two-step indirect path, gated on Q's process-noise injection
between epochs.  The expected size of the improvement at chA is
not large in magnitude on top of today's TICC-only floor (79 ps);
the value is in (a) headroom for better-than-OCXO DOs, (b)
robustness when TICC degrades and Arm 4 has to lean harder on
priors, and (c) cleaner `x[1]` for downstream consumers (e.g.,
the slow-loop / drift-trim path).

Phase B validation measures the size of (a)+(b) in practice.  If
it turns out chA TDEV is indistinguishable from TICC-only at all
τ we care about — Arm 5 still earns its keep through (c) and as
free instrumentation of the rx-TCXO behavior.

## Architecture choice — three options considered

### Option A (recommended): Arm 5 of DOFreqEst

Add `z_tdcp` as a new measurement arm of the existing 4-arm Kalman.
~50 LOC change to `scripts/peppar_fix/do_freq_est.py` + a new
`scripts/peppar_fix/tdcp_estimator.py` that ports the prototype.

Pros:
- Minimal surgery.  Existing Q tuning + R-rcal stay valid;
  Arm 5 just adds a measurement update.
- Kalman fusion is the native "competition" mechanism Bob wants.
- Natural fallback: TDCP arm degrades (sky-loss, slip storm) →
  Kalman down-weights it via innovation-based R-rcal → smoothly
  reverts to qErr.
- Each arm independently gateable: `--no-tdcp-arm` (and `--no-qerr-arm`
  / `--no-extint` / `--no-ticc` already exist) lets ablation match
  main's 2026-05-22 methodology one-for-one.

Cons:
- Couples TDCP into the DOFreqEst Kalman.  A calibration bug in
  TDCP could contaminate `x[1]` and propagate to `x[3]` / adjfine.
  Mitigated by conservative initial R + innov-based outlier
  rejection.
- Q tuning may need a light re-sweep with 5 arms (likely benign;
  Q operates on states, not arms).

### Option B: Replace z_ppp (Arm 1)

Tempting, since main's 2026-05-22 finding was "Arm 1 PPP dt_rx is
the dominant short-tau noise source on chA, remove it."  But Arm 1
observes `x[0]` (rx_tcxo *phase*), not `x[1]` (rx_tcxo
*frequency*).  TDCP observes `x[1]` directly.  Mathematically these
are different observations — not a drop-in replacement.

We *could* derive a phase observation from cumulative TDCP, but
that re-introduces all the ambiguity-drift problems we got to
escape by going differential.  Don't do this.

### Option C: Sidecar actor / new input source

A `tdcp_estimator` actor that emits frequency-error onto a queue
that DOFreqEst consumes as a "new input source" parallel to the
existing arms.  Charlie's instinct, also reasonable.

Pros:
- More decoupled — TDCP estimator is its own thread with its own
  failure model.

Cons:
- The "new input source" still has to enter DOFreqEst somehow.  In
  practice it ends up being Arm 5 by another name, plus extra
  plumbing (queue, actor lifecycle, joint-shutdown logic).
- Doesn't get the native Kalman-fusion competition for free.

If Option A turns up a clean failure in validation we can pivot to
C, but A is the lower-friction first cut.

## Slip protection — three layers (per Charlie's PR-thread spec)

Cycle slips become frequency steps in a pure-TDCP loop.  The
existing 4-arm fusion absorbs slip events through Arm 1's PPP-side
covariance; with Arm 5 added, a missed slip on one SV becomes a
~10 cm ensemble-median step → ~330 ps frequency punch per epoch.
Multi-SV co-slips (day0419h sunrise: 115 slips in 2 hours) defeat
a naïve MAD filter because the outliers co-move.

Three layered defenses, all required:

**L1 — per-SV gating (in `tdcp_estimator.py`)**
- UBX `cpValid` bit (already in the prototype).
- UBX `locktime` monotonicity (already in the prototype).
- GF (geometry-free) phase jump check ported from the existing
  `scripts/peppar_fix/cycle_slip.py` infrastructure.
- IF (ionosphere-free) phase jump check, same source.

**L2 — per-epoch ensemble-MAD gate on the TDCP observation
(in `tdcp_innov_gate.py`)**
- Sliding-window MAD-based anomaly check on the raw TDCP
  `df_f` *observation*, not the Kalman innovation `(z − Hx̂)`.
- Reject if `|z − running_median| > N × max(1.4826·MAD, σ_floor)`.
- Default `window=30 s`, `n_sigma=5`, `sigma_floor_ppb=0.01`.
- Decoupled from filter state by design (Phase B v2 revision):
  - A Kalman-innovation gate would compare against the filter's
    prior `Hx̂`.  If the filter belief is itself contaminated
    (e.g., upstream slip not yet detected, or a prior bad-arm
    update), the Kalman gate accepts the new bad observation as
    "consistent with the prior" — failing exactly when we need
    it most.
  - The observation-domain gate uses a robust running median of
    recent observations as its reference.  A single outlier
    moves the median negligibly; sustained drift moves the
    median over `window`-many epochs (adaptive); a transient
    spike is rejected without polluting future statistics.
- Trade-off vs Kalman: a legitimate rapid shift in `x[1]` (e.g.,
  thermal step) is rejected for ~window epochs until the median
  catches up.  Acceptable because (a) real rx-TCXO steps are rare
  and (b) holding the prior x[1] for 30 s is cheaper than a
  multi-σ punch leaking through.

**L3 — actuator rate limit (in `do_freq_est.py`)**
- Cap the per-epoch adjfine delta via the `max_step_ppb`
  constructor parameter on `DOFreqEst`.  Engine flag
  `--max-adjfine-step-ppb`.  None = disabled (preserves
  pre-Phase-C behavior exactly).
- Applies AFTER the existing `±max_ppb` absolute clamp.
- Belt-and-suspenders defense against ANY arm that produces a
  punch — not Arm-5 specific.  E.g., an outlier TICC sample
  through Arm 4 or an EXTINT spike through Arm 3 also get
  capped here.
- Engaged from the second servo epoch onward (first epoch's
  adjfine is unconstrained because there's no prior to clamp
  against).
- Suggested 10-50 ppb on OCXO-class hosts.  Well above
  legitimate convergence rates (~0.01-0.1 ppb/s) and well
  below the worst-case multi-cycle co-slip punch (~600 ppb).

The day0419h sunrise event is a good fault-injection fixture for
all three layers.  See `prototypes/tdcp-findings-2026-05-22.md`.

## Phasing

### Phase 0 — this design doc

Sign-offs from main + charlie.  No code lands.

### Phase A — standalone estimator (no servo wiring)

- New file: `scripts/peppar_fix/tdcp_estimator.py`.  Ports the
  prototype's core estimator into engine-facing shape (consumes the
  same per-epoch obs dict the engine's serial_reader already
  produces).
- L1 slip-layer integration with the existing `cycle_slip.py`
  detectors.
- Unit tests on a stored RAWX fixture (from `--ubx-out` capture,
  PR #58).  Deterministic TDEV regression: each commit verifies
  the prototype's 15 ps result on the same fixture.
- `--print-tdcp` debug output to the engine log — read-only, does
  not steer.

No actuator risk.  Lands behind the existing `--no-ppp-arm` style
flag plumbing (no flag of its own yet).

### Live-data confirmation gate (between Phase A and Phase B)

Per charlie's recommendation, this is a HARD gate before any
write-side wiring lands:

- 1-hour live capture on TimeHat + PiFace (both have TICC + good
  antenna access), `--ubx-out` enabled.
- Run Phase A estimator offline against each capture.  TDEV(1s)
  must be ≤ 35 ps on each host.  Headroom rationale (revised
  2026-05-23 after the morning gate ran): the prototype hit 15 ps
  on one TimeHat-old capture from 2026-03-23, but the gate run on
  three lab hosts revealed PiFace F9T-20B 15.1 ps / TimeHat F9T-10
  (TIM 2.20) 29.6 ps / clkPoC3 F9T-20B 32.9 ps — fleet-wide F9T
  unit-to-unit rx-TCXO variation is ~2× the original fixture's
  TCXO, not the "few ps" originally allowed for.  35 ps absorbs
  the observed unit variation while still leaving 5–30× headroom
  over qErr (200–960 ps on the same hosts).  Pair-differential
  TDEV(1s) matched the incoherent-sum √(σ_a² + σ_b²) prediction
  within 0.6 ps across all three pairs on the gate run, proving
  the estimator is correct and the per-host floors are independent
  rx-TCXO noise.
- Pair-excursion (integrated phase) is **not** the right gate
  metric here.  Free-running rx-TCXOs have linear frequency drift
  (thermal + aging) of order 1e-12/s that integrates to hundreds
  of ns of differential phase over an hour — the moonshot's 1 ns
  bound applies to the **disciplined-DO output** (Phase B's own
  downstream gate via TICC chA differential), not to the
  measurement-side rx-TCXO motion TDCP observes.

If the per-host TDEV(1s) bound fails on any host, debug Phase A
before Phase B.

### Phase B — actuator wiring (Arm 5 of DOFreqEst)

- Add z_tdcp arm to `do_freq_est.py`.  Initial R sized at
  σ = 0.13 ppb (~5× the prototype floor σ_y ≈ 0.026 ppb).
  Conservative starting value lets innov-rcal tighten from there
  with low risk of contaminating `x[1]` before adaptation kicks
  in.  Cost of "too loose" is slower adaptation; cost of "too
  tight" is contamination — asymmetric.
- `--servo-input tdcp` opt-in.  When set, Arm 5 is enabled; when
  unset, the engine behaves exactly as today.
- `--no-tdcp-arm` kill switch (parallel to `--no-qerr-arm`).
- L2 innovation gate.

#### Phase B default-config recommendation

Main raised this explicitly in PR #59: with Arm 5 added, what are
the *other* arms during the Phase B opt-in window?  Three options:

  (a) **All 5 arms enabled** (additive Arm 5 on top of today's
      Arm 1+2+3+4).  Maximum K-gain-scoreboard signal between
      Arm 2 (qErr) and Arm 5 (TDCP); but Arm 1 is the proven
      noise source from 2026-05-22, and adding Arm 5 on top of a
      contaminated baseline confounds the validation.
  (b) **Match yesterday's TICC-only operational config + Arm 5**:
      `--no-ppp-arm --no-qerr-arm --no-extint` + new Arm 5.
      Cleanest A/B against the proven-good 79 ps freerun-floor
      baseline.  But Arm 5 is the *only* x[1] observation in this
      config — we lose the Arm-2-vs-Arm-5 competition.
  (c) **TICC-only + Arm 2 (qErr) + Arm 5**: keep Arm 2 enabled
      so we get the Arm-2-vs-Arm-5 K-gain comparison, drop the
      proven-noise Arm 1 and the no-value-add Arm 3.  This is
      the cleanest "competition" config and the cleanest A/B
      against TICC-only.

**Recommendation: (c)**.  Phase B default startup:

  `--no-ppp-arm --no-extint --servo-input tdcp`

This keeps Arm 4 (TICC) as the proven primary discipline path,
keeps Arm 2 (qErr) so it can compete with Arm 5 on `x[1]`, and
drops Arm 1 (proven contaminator) + Arm 3 (no chA value-add per
2026-05-22 ablation).  Validation A/B: this config vs today's
`--no-ppp-arm --no-qerr-arm --no-extint` (TICC-only).  Difference
isolates Arm 5's contribution.

#### Phase B validation

- chA TDEV(τ) regression on PiFace.  Must not degrade the 79 ps
  freerun-floor result from 2026-05-22.
- A/B: 30-min runs with and without `--servo-input tdcp`, plot
  TDEV(τ).  Predicted improvement is modest at chA (the
  propagation path through `x[1] → x[3]` is two-step; main
  effect is headroom + robustness, not a big chA TDEV delta at
  the OCXO floor we already hit).
- K-gain log: `K_5[1]` vs `K_2[1]` per epoch confirms Arm 5
  carries the `x[1]` competition.

### Phase C — production polish

- L3 actuator rate limit verification.
- Cycle-slip frequency-step injection tests.
- Promote `--servo-input tdcp` from opt-in to default (only after
  a week of all-host opt-in runs show no regression).
- Failure-mode hardening: all-SV dropout, sky-loss coast policy,
  recovery from extended outages.

## Failure modes + rollback

- **Calibration bias in TDCP**.  If TDCP carries a constant
  frequency bias (e.g., from a wrong wavelength constant or sat
  ephemeris drift), it would push `x[1]` by that bias.  Arm 5
  innovation-based R-rcal catches this within a few minutes
  (`scripts/peppar_fix/r_calibration.py` +
  `scripts/peppar_fix/fit_r_calibration.py` infrastructure already
  exists); worst case the user runs `--no-tdcp-arm` to revert.

- **Slip storm**.  If L1+L2 both miss a co-slip, the actuator gets
  a frequency punch.  L3 (rate limit) caps the worst-case damage
  to whatever the LQR's per-epoch step bound is.  Recovery is
  next-epoch when L1's GF/IF detectors catch up.

- **All-SV dropout**.  Arm 5 produces no measurement; DOFreqEst
  predicts forward on `x[1]` using the existing random-walk
  process model (`Q_clk_rate` injects per-epoch process noise so
  `P[1,1]` grows unboundedly during a sustained outage).  As
  `P[1,1]` grows, `K_2[1]` (qErr's gain on `x[1]`) auto-increases
  to fill the gap — smooth fallback to qErr-only behavior for
  `x[1]`, no special handling required.  Same coast behavior as
  today on the other states.

- **Hard rollback**.  `--no-tdcp-arm` reverts to today's exact
  behavior.  Adding the flag is part of Phase B; verify it works
  via integration test.

## Open questions for review

1. **R-matrix sizing**.  Plan: start at **5× prototype σ_y floor**
   (0.13 ppb starting σ; up from the original 3×/0.05 ppb after
   charlie's review note that "too tight before innov-rcal kicks
   in" has asymmetric cost vs "too loose").  Innov-rcal tightens
   from there.  Open: is 5× enough headroom for cross-host F9T
   variation, or should it be 10× (0.26 ppb starting σ)?  Will
   measure host-to-host variation in the validation gate.
2. **Default opt-in vs opt-out**.  Charlie + main both say
   opt-in.  Agreed; flip after 1 week of all-host clean runs.
3. **Cross-host calibration**.  Different F9T units may have
   slightly different rx_TCXO behavior.  Does Arm 5's R need
   host-specific tuning, or is the global default fine?  Probably
   global is fine — the prototype showed 15 ps across constellation
   subsets on one host; cross-host variation should be similar
   magnitude.  Will measure in the validation gate.
4. **--rinex-out style token expansion** for `--ubx-out`
   (PR #58).  Currently bare path.  Worth adding `{date}`/`{host}`?
   Small follow-up, not blocking.

## Open in dayplan, not here

Anything that's actually a dayplan discussion belongs on
`tdcpServoIntegration-bravo`, not in this doc.  This doc is the
"how" for an agreed "what".  When the answer to an open question
needs commitment, file an amend or discuss on the bead.
