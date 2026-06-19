# τ-aware EXTINT/TICC fusion — design notes

**Status:** design / backlog (dayplan `tauAwareExtintTiccFusion`).
**Owner:** charlie. **Gated on:** clean EXTINT (`ExtintLateEdgeFilter`, PR #198).
**Motivating data:** clkPoC3 three-arm control, 2026-06-18/19
(`scripts/tdev_threearm_clkpoc3.py`, plot
`gt:~/gt/threearm-analysis/clkpoc3-tdev-3arm.png`).

## The problem in one sentence

The DOFreqEst fuses its arms with **one fixed measurement variance R per
arm**, so the relative weighting of EXTINT (Arm 3) vs TICC (Arm 4) is the
same at every τ — but the measurement we *should* trust is τ-dependent, and
the current (TICC-dominated) weighting throws away a 2–3× short-τ win.

## What the control measured

clkPoC3 has a clean EXTINT input (no ringing), so it isolates servo-input
behavior from the PiFace ringing pathology. chA TDEV (detrended, the metric
per CLAUDE.md), steady-state, sub-decade τ:

| τ (s) | default (TICC+EXT) | no-extint (TICC) | no-ticc (EXTINT) |
|---:|---:|---:|---:|
| 1   |  93 |  96 | **44** |
| 5   |  55 |  52 | **24** |
| 100 | 1264 | 1078 | **417** |
| 300 | 1255 | 1237 | **1045** |
| 1000 | 925 | 720 | 1262 |

- **Short/mid τ (1–~300 s): EXTINT-only wins 2–3×**, statistically robust
  (tight, non-overlapping χ² bands; edf ≳ 100 below τ=100 s).
- **Long τ (≥ ~500 s): trend flips** (TICC anchors to the GPS-PPS hardware
  edge; EXTINT-only drifts up) but the bands overlap — **not yet resolved**
  in a 3 h run. Needs a >12 h clean-host capture before tuning a crossover.
- **default ≈ no-extint**: the current fusion is **TICC-dominated** and does
  not capture EXTINT's short-τ advantage.

Independent corroboration from the actuator log (same run): with `--no-ticc`
the adaptive scheduler is far more **patient** — it coasts on 38% of epochs
(vs ~4% for the TICC arms), holds the actuator up to 15 s (vs 3–5 s), and
makes 7× smaller corrections. That is the soft-loop signature.

## Why — the mechanism (R vs observable *content*)

Two distinct "noises" must be kept separate:

1. **Instrument noise R** (sets the Kalman gain / loop stiffness): EXTINT
   ~8 ns, TICC ~60 ps, TDCP ~20 ps. Lower R ⇒ higher gain ⇒ stiffer loop.
2. **Observable content** (what real-world signal the arm carries):
   - EXTINT (Arm 3, H=[0,0,1,0]) references the **carrier-smoothed nav-time**
     view of the DO PPS — clean.
   - TICC (Arm 4, H=[−1,0,−1,0]) measures **chA−chB = DO − GNSS-PPS**, so its
     content carries the raw GNSS-PPS edge: the **8 ns quantization sawtooth
     *and* the receiver TCXO's analog phase noise**.
   - TDCP (Arm 5) observes **x1 = rx frequency**, *not* the DO — it can't
     steer DO phase; it de-contaminates the rx-clock state the DO arms lean on.

The short-τ outcome is the interaction of the two: EXTINT pairs a **soft
loop** (high R) with **clean content**, so the OCXO flywheel defines short-τ
stability (24 ps). TICC pairs a **stiff loop** (low R) with **dirty content**
(chB), so the loop works hard to null chA−chB and **drags the DO to chase the
8 ns GNSS-PPS noise** — worst of both for short τ. At long τ the roles invert:
the GPS-PPS hardware edge is the better *absolute* anchor, while EXTINT's
nav-time reference wanders.

## The qErr connection (why this matters for receiver choice)

qErr — hardware (TIM-TP, F9T only) or **synthetic** (we already compute it in
the TICC arm: `z_ticc = −φ_do − qerr(φ_rx)`, the 125 MHz sub-tick residual
from the modeled rx phase; `docs/pps-ppp-error-source.md`) — removes only the
**quantization** half of chB's contamination. It does **not** remove the
receiver TCXO's **analog** phase noise. EXTINT (carrier-smoothed) escapes
both, so τ-aware fusion can beat even a working-hardware-qErr F9T at short τ.

Measurement caveat from the motivating run: **neither host had usable
hardware qErr** (PiFace F9P emits qErr=0; clkPoC3 F9T threw
`qErrInvalid=1` and dropped every sample). So the TICC penalty above is the
**synthetic-qErr-only** condition. Three complementary levers fall out:

| Lever | Keeps F9P? | Removes quantization | Removes analog rx noise |
|---|---|---|---|
| τ-aware EXTINT/TICC fusion | ✅ | n/a (sidesteps chB) | ✅ (via EXTINT) |
| Better synthetic qErr (improve x0) | ✅ | ✅ | ❌ |
| Hardware qErr (fix F9T `qErrInvalid`) | ❌ | ✅ | ❌ |

Fusion is the only lever that also escapes the analog floor *and* keeps the
cheaper F9P — hence the priority. The qErr experiments (below) bound the
other two.

## Design options for the τ-aware weighting

The knob is per-arm effective R as a function of the regime. Candidates,
roughly increasing complexity:

1. **τ-banded R schedule (static).** Inflate TICC R (soften it) at short τ
   and EXTINT R at long τ, with a fixed crossover (~300–500 s from the data).
   Simplest; but the EKF runs at 1 Hz — "τ" must be expressed as a loop
   bandwidth / scheduler-patience setting, not a literal per-sample switch.
2. **Complementary filter on the two DO-phase arms.** Low-pass the TICC
   estimate of x2 and high-pass the EXTINT estimate (or vice-versa by τ),
   blending at the crossover. Classic two-sensor fusion; needs the crossover
   pinned by a long capture first. **Caveat:** a single fused EKF exposes only
   one x2 — there is no separate "TICC estimate of x2" vs "EXTINT estimate of
   x2" to blend. This option therefore requires **parallel per-arm estimators**
   (or an equivalent decomposition), which is materially heavier than options
   (1) and (3); it is listed for completeness, not as the cheap path.
3. **Confidence-driven dynamic R.** Drive each arm's R from a live noise
   estimate (e.g. EXTINT's running jitter post-filter, TICC's qVIR/chB
   activity) so the fusion adapts per-host without a hard-coded crossover.
   Most general; most to get wrong.

Recommendation: prototype (1) in the closed-loop sim (`scripts/servo_sim.py`)
to find the crossover and confirm it beats both single arms, then validate on
clkPoC3 before considering (3).

## Prerequisites & risks

- **Clean EXTINT is mandatory.** The whole result inverts on a ringing input
  (PiFace EXTINT-only diverged to 308 µs). Gated on `ExtintLateEdgeFilter`
  (#198, live A/B in progress) and/or the hardware level-shifter fix.
- **Confirm the long-τ crossover** with a >12 h clean-host capture before
  fixing the crossover point — the 3 h run does not resolve τ ≥ 500 s.
- **Don't regress long-τ anchoring.** Over-weighting EXTINT long-τ would let
  the DO walk; the fusion must hand authority back to TICC past the crossover.
- TDCP belongs in the picture as the **x1 (rx-freq) pin**, not a DO arm —
  include it so neither DO arm is fooled by rx-clock motion.

## Validation plan

1. >12 h clean-host (clkPoC3) capture → resolve the long-τ crossover with
   tight χ² bands.
2. Sim sweep of crossover/blend in `servo_sim.py`; target: beat both single
   arms at *every* τ (the moonshot "best-of-both" curve).
3. Lab A/B: fusion vs default vs no-ticc, three-arm style, chA-TDEV metric.
4. Cross-check on a ringing host *after* #198 to confirm fusion is safe there.

## Related

- `docs/pps-ppp-error-source.md` — the synthetic-qErr (125 MHz tick) model.
- `docs/ticc-vs-extint-do-observer-experiment.md` — the per-clock-TICC vs
  EXTINT vs TDCP arm comparison this builds on.
- `scripts/tdev_threearm_clkpoc3.py` — the χ²-banded per-arm TDEV analysis
  (ships with these notes; hardcodes `gt:~/gt/...` paths — one-off artifact).
- `scripts/analyze_extint_ringing.py` — the EXTINT ringing characterization.
  **Lands with the filter in PR #198, not on main yet** — present once #198
  merges.
