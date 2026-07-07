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

### Finding 2 — the rail is a **single-oscillator** phenomenon the (two-oscillator) sim can't yet model

> **Corrected after delta's PR #300 review (topology mismatch).** The first
> cut of this finding said "the rail doesn't reproduce / dt_rx gating targets
> the wrong arm." That is a **two-oscillator statement** and is right *only*
> for the topology servo_sim models. delta's correction and a follow-up
> sole-observer experiment sharpen it.

**servo_sim models the two-oscillator topology** (separate F9T rx TCXO + a
disciplined DO; GPS-locked PPS). In that topology:

* A `dt_rx` (PPP, Arm 1 → `x[0]`) glitch **is** decoupled from DO steering:
  −12 ns (even −1000 ns) moves `x[0]` but the DO output is unchanged to
  machine zero (below the adjfine LSB), because the DO-phase observation is
  GPS-referenced (EXTINT = `φ_do`; TICC = `−φ_do − qErr(edge)`; rx enters
  only via the sub-tick qErr). `dt_rx_glitch` injects `meas['dt_rx_ns']` =
  the Arm-1 PPP feed → this decoupled arm.
* A DO-phase-obs bias ramp grows the DO output (9.8 → 595 ns at 0.5 ns/s)
  but by gating **both** DO-phase arms out (chi²) → the servo **coasts** →
  DO free-runs; the **actuator stays bounded** (|x3| ≤ 1.6 ppb, |adj| ≤ 1.9
  ppb) at every rate 0.02–1.0 ns/s. **This coast-not-rail behaviour is
  exactly why delta's knob 1 works** (see below).

**The GNSSDO+ is single-oscillator** (SXT-D OCXO clocks the Mosaic-T
receiver). There the mosaic `dt_rx` **is** the DO-phase observation: the
engine feeds it to the `x[2]` arm (`_dt_rx_servo_epoch`, PPP arm off), and
that arm is the **sole** observer — no TICC, no EXTINT. delta's knob 1 (soft
NIS innovation gate in `_kalman_linear_update`, covering all linear arms)
adds to that sole observer the gate my TICC-based loop already has — which
is precisely the "gate-out → coast, don't rail" defense of Finding 2b.
Hardware confirms: with knob 1, freq bounded ±0.1 ppb where ungated railed
to +386 ppb.

**But the sim still can't reproduce the rail — even sole-observer.** A
follow-up (`use_extint`-only, no TICC/PPP, tight measured-Q, glitch/ramp on
the DO-phase obs) still does **not** rail: a one-shot glitch is absorbed,
and a sustained ramp is *tracked stably* (|x3| ≤ 2 ppb, the DO faithfully
follows the spurious ramp). The reason is structural: in the single-
oscillator design **the DO is the receiver's clock**, so steering the DO
*moves the observed quantity* (`dt_rx`) — an **actuator→observation feedback
loop** that, with the gain-1.0 `x[3]` term and an ungated sole observer,
closes into a self-consistent runaway. servo_sim's two-oscillator model
observes the DO's *absolute* phase (`φ_do`) independently of the fact that,
on a GNSSDO+, the same oscillator drives the observer — so the positive-
feedback path that rails is simply absent.

**Conclusion:** the rail is a property of the **single-oscillator topology**,
not of `DOFreqEst` in isolation. To reproduce it (and to serve future
single-oscillator designs) servo_sim needs a **single-oscillator mode** —
see below.

## Single-oscillator mode — the servo_sim gap

servo_sim was built for the two-oscillator world (rx TCXO ≠ DO). Single-
oscillator GNSSDO designs (GNSSDO+ SXT-D today; more coming) differ
structurally, and the sim does not yet model them:

| | two-oscillator (F9T + DO) | single-oscillator (GNSSDO+) |
|---|---|---|
| rx clock vs DO | **separate** oscillators | **same** oscillator |
| `dt_rx` / PPP arm | rx TCXO offset (`x[0]`), decoupled | **is** the DO-phase obs (→ `x[2]`) |
| DO-phase observers | TICC + EXTINT (gated) | mosaic `dt_rx`, **sole**, (pre-knob-1) ungated |
| actuator → observation | open (DO obs is GPS-referenced) | **closed** (steering DO moves `dt_rx`) |
| failure mode | coast on gate-out (bounded) | self-consistent **rail** |

**`single_oscillator=True` — BUILT (I-122448, PR stacked on #300).** The
mode collapses the rx-TCXO+DO into one oscillator, routes the DO-phase truth
into the `x[2]` arm as the sole observer (via `extint_phase_ns` =
functionally delta's do_phase Arm 8), and closes the actuator→observation
loop (the plant already does this). Two hardware-realism knobs: `actuator_
gain_true` (imperfectly-known steering gain) and `single_osc_obs_lag_s` (the
mosaic's internal clock-filter lag — the cascaded-loop dynamic that is
genuinely single-oscillator).

**Honest result — the mode does NOT reproduce delta's rail.** Driving the
real `DOFreqEst`, no ingredient tried (dt_rx/do_phase glitch, `actuator_gain_
true` 0.5–3×, `single_osc_obs_lag_s` 0–40 s, coast 1–60 s) reproduces delta's
**tight-Q, normal-cadence** rail. Over a 12-combo {lag × gain} sweep at normal
cadence, **0** show delta's tight-Q-rails pattern; the instabilities the mode
*does* exhibit are the **opposite** Q-dependence — looser Q (higher loop BW)
rails under observation lag (textbook delay-instability), and any topology
rails under an extreme (≥45 s) coast. That long-coast rail is **not**
single-oscillator-specific (the two-oscillator model rails there too).

What this implies (a useful cross-check, not a dead end):
1. delta's rail is almost certainly **perturbation-triggered**, not pure
   tight-Q underdamping — consistent with knob 1 (a *gate*) fixing it, since
   a gate cannot fix an underdamped loop. The trigger was the concurrent
   `mosaic RxClkBias` ramp / dt_rx glitch through the sole ungated observer.
2. Reproducing it in sim needs the **hardware-specific dynamics** the model
   still idealizes: the mosaic's actual clock-filter response, the SXT-D's
   measured actuator gain, and the real adaptive-scheduler cadence. Next
   step is to **parameterize the mode from the SXT-D characterization**
   (joint bravo↔delta) and re-check.

The mode still delivers (b) the standing test bed for the single-oscillator
designs on the roadmap and (c) — once SXT-D-parameterized — the place to
validate knob 1 / knob 2 / `do_freq_clamp` before hardware.

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
