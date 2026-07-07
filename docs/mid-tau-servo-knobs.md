# Mid-τ servo knobs — servo_sim cross-check of delta's hardware findings

**I-084500 (midTauLoopResonanceHump), bravo × delta, 2026-07-07.**
Parallel to delta's live GNSSDO+ hardware arms: use servo_sim (which
drives the *real* `DOFreqEst`) as a fast, deterministic test bed to
pressure-test delta's proposed mid-τ fixes before more overnight hardware
time is spent. This doc records what the sim showed — including two places
where it **disagrees with or fails to reproduce** the hardware story, which
matters more than a confirmation would.

## Background — delta's hardware findings (this morning)

From delta's dayplan posts (05:30, 08:20) on the single-oscillator GNSSDO+
(SXT-D Rakon OCXO, Mosaic-T receiver):

1. `--lqr-phase-gain` (LQR `L[2]`) is the **wrong** mid-τ knob: a 10×
   change barely moved `TDEV(100 s)` (172 vs 187 ps).
2. The real levers are the **EKF freq process-noise Q**, **dt_rx
   innovation gating**, and an **anti-windup slew limit** — with the
   claim that the *measured* (tight) Q is "the right direction" but
   "rings without the gating."
3. The observed instability was an **actuator rail**: `x[3]` diverged,
   `freq +227 ppb`, `word → rail`, attributed to *"x[3] diverging on a
   swallowed dt_rx glitch"* (`u = -(L2·φ + 1.0·x[3])`, the freq term
   ungated).

## What servo_sim shows

Test rig: `servo_sim` on the `piface-current` preset (re-grounded to
current PiFace hardware) plus an invented **stable-OCXO** model
(`rwfm=0.005, wfm=0.01`) to stand in for delta's much quieter SXT-D.
Two new deterministic injection hooks were added to probe delta's
mechanism (kept as tooling):

* `dt_rx_glitch = (t_s, Δns)` — one-shot glitch on the emitted PPP `dt_rx`.
* `do_obs_bias_ramp = (t0, ns/s)` — a growing bias on the DO-phase arms
  (EXTINT + TICC), modelling an rx-coupled PPS drift reaching the
  observation the servo actually steers on.

### Finding 1 — the mid-τ hump lever is Q, and **looser lowers it** on *both* DO models

`TDEV`/excursion (settled, skip 300 s) vs `sigma_do_freq_ppb`:

| DO model | loose Q=0.03 | char-ish | tight Q=8.4e-5 |
|---|---|---|---|
| piface (wandering, rwfm=0.05) | exc **6.4 ns** | 9.8 ns (Q=0.009) | **131 ns** (rings) |
| stable OCXO (rwfm=0.005) | exc **1.5 ns**, rms 0.34 ns | — | 6.2 ns, rms 3.47 ns (Q=8.4e-5) |

Looser Q lowers the mid-τ excursion **regardless of DO stability** — even
on the stable-OCXO model, loose Q beats the char-matched tight Q by ~10×.
This **disagrees with delta's "measured-Q is the right direction."**
Physically: tight Q makes the filter over-trust its freq prediction and
lag the DO's real motion (the hump *is* that lag); looser Q raises the
effective loop bandwidth and tracks it out. Under tight Q the ring shows
up as the **Arm-4 TICC chi² gate firing continuously** (|innov| 24→38 ns
and growing) — the stiff freq state can't track, phase error blows up, the
DO-phase observer gates itself out, and the loop free-runs.
`soft_ticc_gate` (R-inflation instead of hard reject) only partially helps
(131 → 84 ns); it doesn't fix the Q-induced lag.

> ⚠️ Caveat: `piface-current` over-produces the hump *amplitude* ~6× (the
> preset's own note) and the stable-OCXO model is invented, not
> characterized — so treat the **direction** as robust and the absolute
> numbers as indicative only. **Recommendation: settle the Q direction
> with a controlled hardware Q A/B (loose 0.03 vs char-matched) on the
> SXT-D before committing to measured-Q.**

### Finding 2 — delta's `dt_rx → x[3] → rail` mechanism does **not** reproduce

Driving the identical `DOFreqEst`, **no** modeled perturbation rails the
actuator:

* **dt_rx (PPP) glitch is decoupled from DO steering.** A −12 ns (even
  −1000 ns) `dt_rx` glitch moves the rx-clock state `x[0]`, but the DO
  output is **unchanged to machine zero** — the perturbation is below the
  adjfine quantum. Mechanism: the DO-phase observation is **GPS-referenced**
  (EXTINT reports `φ_do`; TICC reports `−φ_do − qErr(edge)`), so the rx
  clock enters only through the sub-tick qErr sawtooth — because the F9T
  PPS is GPS-locked, not free-running on the rx TCXO. `dt_rx` (the PPP
  rx-clock offset, `x[0]`) is a *different* quantity and does not touch the
  DO-phase arms. **So "dt_rx innovation gating" targets the wrong arm.**

* **DO-phase-obs corruption coasts, it doesn't rail.** A bias ramp on the
  DO-phase arms *does* grow the DO output (9.8 → 595 ns at 0.5 ns/s), but
  by gating **both** DO-phase arms out (chi²) → the servo loses its phase
  observer and **coasts** → the DO free-runs. The **actuator stays bounded**
  (|x3| ≤ 1.6 ppb, |adj| ≤ 1.9 ppb) at every ramp rate tried (0.02–1.0
  ns/s). This is the *opposite* of delta's `x[3] → +386 ppb` rail.

Conclusion: the sim's EKF + existing gates are **robust against the
actuator rail**. delta's hardware rail comes from a coupling or input the
sim does not model (candidates: the Mosaic-T's PPS/RxClkBias behavior
feeding the DO-phase observation, or an actuator→measurement feedback path).
**A captured hardware repro is needed to root-cause it** — the sim cannot
currently serve as its test bed.

## Code shipped

Gated, **default-off**, byte-identical when off:

* **`do_freq_clamp_ppb`** (`DOFreqEst`, + `servo_sim` passthrough) — clamp
  `x[3]` to ±clamp of its bootstrap nominal (a DO has a bounded pull range;
  `x[3]` physically cannot wander further). A **safety seatbelt** peer to
  the existing `max_step_ppb`, bounding a freq-state windup *inside* the
  gross ±1e6 ppb sanity bound. Honestly labeled **unvalidated in sim**
  (the sim never rails `x[3]`); it's a hardware backstop delta can enable
  on the SXT-D while the rail is root-caused, not a proven fix.
* **`dt_rx_glitch`, `do_obs_bias_ramp`** (`servo_sim`) — the mechanism-probe
  injection hooks above.

**Not shipped (deliberately reverted):** the `dt_rx_innov_gate_nsigma`
prototype. The sim shows the PPP arm it gates is decoupled from DO
steering, so it cannot address the mid-τ hump or the rail — shipping it
would be fixing a disproven mechanism.

## Recommendations for delta

1. **Q direction**: run a controlled hardware A/B (loose Q≈0.03 vs
   char-matched) on the SXT-D. The sim says looser is better even on a
   stable OCXO; if hardware disagrees, the reconciling factor is a
   measurement noise the sim treats as too clean (real outliers /
   multipath / Mosaic PPS) — worth identifying.
2. **The rail**: capture a hardware repro (the instrumented arm). The rail
   is not a generic `dt_rx → x[3]` coupling; the sim points at the
   DO-phase observation path (Mosaic PPS). Root-cause *that* before coding
   a servo fix.
3. Enable `do_freq_clamp_ppb` (set to the SXT-D's measured pull range) as
   a seatbelt against catastrophic rail-to-word-limit while investigating.
