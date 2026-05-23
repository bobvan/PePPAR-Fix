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
  z_tdcp    :  observes  x[1]                       σ ~ 0.015 ppb
```

## Why this slots in cleanly as Arm 5

The prototype's measured TDEV(1s) ≈ 15 ps is the σ of the
ensemble-derived **phase** signal at τ = 1 s.  Converting:

  σ_x1 = TDEV(1s) / τ = 15e-12 s / 1 s = **1.5e-11 = 0.015 ppb**.

That's the measurement-noise floor of TDCP's observation of `x[1]`.
qErr today sits at "sub-ppb" — call it 0.3 ppb effective σ after the
time-averaging window.  So z_tdcp is ~20× more precise than z_qerr
**on the same state**.

When two arms observe the same state with different R values, the
Kalman fusion gives an optimal-weighted estimate.  K-gain for the
sharper arm dominates.  No design choice required at integration
time — the math sorts it out.  This is exactly the competition Bob
asked for, and we can read the scoreboard directly:

  * Log K_5[1] vs K_2[1] per epoch.  Ratio ≫ 1 means TDCP carries it.
  * Run `--no-arm-5` to ablate and re-measure chA TDEV.
  * Differential analysis: does adding Arm 5 change chA TDEV(τ)?
    By how much?  At which τ?  That's the answer.

## What TDCP is NOT measuring (Charlie's reframe)

**15 ps is the F9T rx-TCXO's own motion as seen through carrier
phase.**  It is **not** a disciplined-DO output target.  In the
CLAUDE.md "two oscillators bound the result" framing, this is the
**RX TCXO measurement-chain ceiling** — the second of the two
limits, the one that bounds how cleanly we can characterize GPS
time from the receiver's perspective.

The disciplined OCXO output (chA TDEV via TICC) sits *below* 15 ps
on PiFace + clkPoC3 today (79–84 ps was the 2026-05-22 night result,
and OCXO freerun is ~85 ps).  Wait — 79 ps is *less* than 15 ps?
No: 79 ps is the **DO PPS** measured through TICC; 15 ps is the
**rx-TCXO frequency observation noise** at 1 s.  Different units and
different physical signals.  Future readers should not anchor chA
expectations to "15 ps".

What TDCP buys us is **cleaner observation of x[1]**.  Whether
that translates to chA improvement depends on whether x[1] is on the
critical path for chA.  Today (after main's overnight) x[2] (DO
phase via TICC) carries the chA discipline directly.  TDCP enters
through x[1] which influences x[3] (DO frequency) through the
prediction model, and x[3] sets adjfine.  So TDCP's improvement
propagates to chA through one prediction step.  Validation
measures the size of that propagation.

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
- Each arm independently gateable: `--no-arm-5` (and `--no-qerr-arm`
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

**L2 — per-epoch ensemble innovation gate (in `do_freq_est.py`)**
- Reject the whole epoch's Arm 5 measurement if
  `|z_tdcp − Hx̂| > N × σ_innov`.
- Catches multi-SV co-slips because L1 is per-SV but L2 treats the
  whole epoch as suspect when the ensemble disagrees with the
  filter's prior.
- Conservatively N = 4 (white-noise excess is rare at N > 3).

**L3 — actuator rate limit (in the LQR controller)**
- Cap the per-epoch adjfine delta independent of what DOFreqEst
  asks for.  Belt-and-suspenders.
- Already partially in place; verify it remains enforced when
  Arm 5 is the dominant innovation source.

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
- Run Phase A estimator offline against both captures.  TDEV(1s)
  must be ≤ 25 ps on each host (1.5× headroom over the prototype's
  15 ps).
- Cross-host pair-excursion: PiFace ↔ TimeHat differential
  rx_TCXO frequency must agree to ≤ 1 ns at 95% over 1 hour.
  This is the moonshot's per-pair short-tau bound and is the
  reason this whole work exists.

If either bound fails, debug Phase A before Phase B.

### Phase B — actuator wiring (Arm 5 of DOFreqEst)

- Add z_tdcp arm to `do_freq_est.py`.  Initial R sized
  conservatively at σ = 0.05 ppb (3× the prototype floor).
- `--servo-input tdcp` opt-in.  When set, Arm 5 is enabled; when
  unset, the engine behaves exactly as today.
- `--no-arm-5` kill switch (parallel to `--no-qerr-arm`).
- L2 innovation gate.
- Validation: chA TDEV(τ) regression on PiFace.  Must not degrade
  the 79 ps freerun-floor result from 2026-05-22.
- A/B: 30-min runs with and without Arm 5 enabled, plot TDEV(τ).
  Expected outcome: small improvement at sub-1 s τ where x[1]
  precision propagates to x[3]; no change at τ ≫ servo time
  constant.

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
  ephemeris drift), it would push x[1] by that bias.  Arm 5
  innovation-based R-rcal catches this within a few minutes
  (`projet/.../r_calibration` infrastructure already exists);
  worst case the user runs `--no-arm-5` to revert.

- **Slip storm**.  If L1+L2 both miss a co-slip, the actuator gets
  a frequency punch.  L3 (rate limit) caps the worst-case damage
  to whatever the LQR's per-epoch step bound is.  Recovery is
  next-epoch when L1's GF/IF detectors catch up.

- **All-SV dropout**.  Arm 5 produces no measurement; DOFreqEst
  predicts forward on x[1] using the existing process model.
  Same coast behavior as today minus one observation.  qErr arm
  (Arm 2) keeps pulling.

- **Hard rollback**.  `--no-arm-5` reverts to today's exact
  behavior.  Adding the flag is part of Phase B; verify it works
  via integration test.

## Open questions for review

1. **R-matrix sizing**.  Plan: start at 3× prototype floor
   (0.05 ppb) and innov-rcal from there.  Concerns?
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
