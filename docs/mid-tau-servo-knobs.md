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

**Reproducing delta's rail — it took the mosaic clock filter.** The *bare*
topology (servo observing the DO phase directly) does **not** rail on any
perturbation — a glitch is absorbed, a processing stall is absorbed, the loop
is well-damped. That is itself a finding: delta's rail is **not** tight-Q
underdamping (both class-default Q=0.01 *and* measured Q=8.4e-5 railed on
hardware — confirmed by delta), and it is **not** a generic long-coast
instability (that rails the two-oscillator model too).

The load-bearing single-oscillator dynamic is the **mosaic's own clock
filter**. delta's captures prove `our_dt_rx_ns == mosaic_rxclkbias_ns` and
`mosaic_rxclkdrift_ppb ≈ our_freq_ppb` — the servo does not observe the DO
directly; it observes the **bias state of the mosaic-T's internal clock
estimator**, a *second* filter (bias + drift, α-β) in series with our servo.
`single_osc_mosaic_filter=True` models it. With the cascade in place, a short
**processing stall** (`processing_stall`, delta's `max_stall_s≈2 s` trigger)
tips the marginal two-filter loop, and the mosaic filter's bandwidth sets how
far it goes:

| mosaic filter (α) | outcome | matches delta capture |
|---|---|---|
| fast (α≥0.2) | stable — stall absorbed | (clean lock) |
| α≈0.1 | bounded **limit cycle** (freq ±~12 ppb) | knob1 (±1700 ns / ±7 ppb) |
| slow (α≤0.05) | **rail** | abA1 (+386 ppb) |

So the sim now reproduces delta's full **stable → limit-cycle → rail
spectrum**, and it confirms the mechanism: a cascaded mosaic+servo loop made
marginal by the mosaic filter, tipped by a short stall/glitch through the
sole ungated observer. This is exactly why **knob 1 (a gate) works** — it
caps the tip — and why it's necessary-but-not-sufficient (the *loop* is still
marginal; knob 2 / damping is the cure).

Remaining (a quantitative fit, not a mechanism gap): matching delta's *exact*
limit-cycle amplitude (±1700 ns) and period (~10–15 min) needs (a) the SXT-D
DAC **word-limit** (delta railed at +386 ppb where the control word
saturated; the sim's `max_ppb` is far higher, so its rail overshoots) and
(b) fitting α/β to the knob1 CSV trajectory. The knobs (`mosaic_alpha`,
`mosaic_beta`, `processing_stall`, `actuator_gain_true`) are wired for that
fit. The mode is now the standing test bed for the single-oscillator roadmap
and — with delta's knob-1 gate rebased on top — the place to validate knob 1
/ knob 2 / `do_freq_clamp` in sim before hardware.

### Correction (2026-07-07) — the second filter is OURS, not the mosaic's

The "mosaic clock filter" framing above is **mis-attributed**. After Bob's
prompt and a read of the Septentrio mosaic reference guide + our own code, the
physics and the lever conclusions stand, but the *owner* of the in-loop
clock-smoothing filter is us, not the mosaic:

- **We consume raw `MeasEpoch` carrier phase, never `RxClkBias`.** A repo grep
  finds zero consumers of `RxClkBias`/`PVTCartesian`/`PVTGeodetic`. `RxClkBias`
  is *"the clock bias term computed in the PVT solution"* — the mosaic's own
  Kalman PVT clock, and it is exactly what the **SparkFun reference GNSSDO
  servos on** (its ESP32 reads `RxClkBias` → steers the OCXO). Servoing on
  `RxClkBias` would be "asking the mosaic's filtered opinion" — the wrong
  question. Our ESP32 firmware **replaced** that SparkFun loop with our own
  actuation (SparkFun `$W`), and we read raw phase, so that filter is out.

- **With the external OCXO the mosaic cannot filter the frequency at all.**
  The OCXO drives the mosaic's time base via `REF IN` (10 MHz; confirm with
  the `EXT_FREQ` status bit). §2.3's free-running/steered clock modes are
  *internal-oscillator* framing: "clock steering" physically tunes the mosaic's
  own crystal — impossible for an external reference it only receives. So the
  carrier phase, timestamped on the OCXO, is an inherently **raw** measure of
  OCXO-vs-GPS phase. The mosaic's only clock operation is the discrete **1 ms
  time-of-day jumps** (applied to code + phase, deterministic, removable via
  the `CumClkJumps` field in MeasEpoch) — not a filter.

- **So the only clock-smoothing filter in our loop is our own `FixedPosFilter`**
  (its `random_walk` clock model turns the raw phase into `dt_rx`). That is
  what `single_osc_mosaic_filter` / `mosaic_alpha`/`mosaic_beta` actually
  model — the α-β stand-in for our FixedPosFilter's clock smoothing, *not* a
  mosaic filter. (The knob names are kept for code continuity; read them as
  "the in-loop clock filter.") The cascade is `FixedPosFilter → DOFreqEst` —
  **both ours** — which is why lever (c) = `--clock-model wno` (collapse the
  FixedPosFilter half) is the full receiver-side fix, and **no mosaic
  reconfiguration is needed or even possible on the frequency.**

Hardware checks to *verify* this holds (cheap, no change): (1) `ReceiverStatus`
**`EXT_FREQ` set** (mosaic locked to the external 10 MHz, not its internal
clock); (2) **nothing on `TimeSync`** — a 1 PPS there makes the mosaic sync its
time base to that pulse, an extra path; (3) the engine **un-wraps
`CumClkJumps`** so a 1 ms jump isn't misread as a phase glitch.

### Knob-2 lever sweep — which fixes the in-loop clock filter (aiming delta's next arm)

delta's decisive question: *does lowering OUR loop bandwidth alone stabilize
the cascade, or must we touch the mosaic filter?* Swept the three levers at
the SXT-D operating point (ungated-rail regime, `mosaic_alpha≈0.05`), stall-
perturbed, measuring stability + mid-τ TDEV(100 s). **Absolute mid-τ runs ~6×
delta's hardware (the preset's amplitude over-production) — read the levers
RELATIVELY.**

| lever | stabilizes? | mid-τ TDEV(100 s) |
|---|---|---|
| **(a)** lower our BW (tighter Q) | only at Q ≤ 1e-3 | **5.6–20.8 ns** (worst) |
| **(b)** widen the in-loop clock filter (α≥0.2, faster FixedPosFilter) | yes | **0.74 ns** (best) |
| **(c)** bypass cascade (`--clock-model wno`, single loop) | yes, all Q | **~0.83 ns** |

**Answer: lowering our bandwidth alone does NOT solve it.** Lever (a)
stabilizes only by going to very tight Q, and that gives the *worst* mid-τ
(the cascade forces a bad stability↔mid-τ tradeoff). You **must collapse the
in-loop clock filter** — which, per the correction above, is *our own*
FixedPosFilter, so we fully own the fix: (c) **`--clock-model wno`** collapses
it (delta owns it, no new code) — this is the decisive lever. Lever (b)
("raise the filter bandwidth") is nearly equivalent and slightly better mid-τ
in sim, but it's just a different way to widen the *same* FixedPosFilter, not
a mosaic change. Recommended next hardware arm: **tight-Q + `--clock-model
wno`**. There is no residual *mosaic*-side smoothing to chase — with the
external OCXO the mosaic can't filter the frequency; the only receiver-side
artifact is the removable 1 ms `CumClkJumps` (verify the engine un-wraps it).

This is robust to the **Q-direction discrepancy** (my sim: tighter Q
stabilizes lever a; delta's hardware: measured tight-Q *railed*) because
lever (a) loses either way — it either doesn't stabilize (hardware) or
stabilizes at unusable mid-τ (sim). The discrepancy itself flags that the
cascade model reproduces the *spectrum* and the *tradeoff* but not the exact
tight-Q hardware behaviour (candidate: word-limit windup under tight-Q); it
does not change the decisive lever.

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

### Actuator settling-noise term — the short-τ penalty (charlieActuatorNoise)

charlie's OTC hardware Q sweep confirmed #300's mid-τ win but exposed a
short-τ COST of looser Q the excursion metric had folded away: a TDEV bump
that **peaks at τ≈4 s and grows with looser Q** (loose Q=0.03: TDEV rises
1s=0.46 → 4s=1.02 then falls; tight Q=1.4e-4 stays flat-low). The sim's clean
frequency actuator never reproduced it — a frequency-steering servo can only
inject noise that integrates to phase over τ, so it shows at *long* τ, never
at 1–4 s.

`actuator_noise_gain` / `actuator_noise_tau_s` model the missing mechanism: a
**bounded AR(1) PHASE transient** (DAC-write + DPLL settling/ringing) kicked
by each adjustment, kick std = `gain·|Δapplied|`, correlation time `tau_s`.
Phase (not frequency) because charlie's TDEV *rises then falls* — the
signature of a bounded/stationary phase process; the peak sits at τ≈`tau_s`.
Because the kick scales with the per-epoch adjustment magnitude, looser Q
(bigger adjustments) grows the bump — reproducing the Q-dependence. Default
`gain=0` → byte-identical.

Status: the term reproduces the qualitative signature (bump at τ≈`tau_s`,
grows with adjustment magnitude / Q, absent at tight Q). Precise magnitude +
the scaling law (linear vs superlinear in `|Δapplied|`) need charlie's
5-point TDEV-vs-Q calibration — 2 endpoints can't pin it, and the sim's
loose/tight adjustment ratio (~7×) is smaller than charlie's actuator-
contribution ratio (~20×), which itself is a clue the scaling may be
superlinear.

**Early two-timescale finding (needs the calibration to confirm):** the naive
"loose estimator + `max_step_ppb` slew" does **not** work — a slew tight
enough to quiet the short-τ bump (it caps `|Δapplied|`) also caps the loop's
tracking and **winds up / diverges** at long τ. The real fix is a *low-pass*
on the actuation (smooth the per-epoch command jitter without capping the mean
rate) + anti-windup, not a hard rate limit. Testable in sim once the term is
calibrated.
