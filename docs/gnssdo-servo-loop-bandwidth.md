# Servo loop bandwidth for a low-noise single-oscillator DO (GNSSDO+)

**Goal:** let the beautifully low-noise Rakon OCXO in the GNSSDO+ "do its
thing" below τ ≈ 1000 s — the discipline loop should not touch it there — and
only steer to GPS at longer τ, ever so gently. This is the moonshot's core
("at short τ the better oscillator's noise floor shines through unmolested by
the loop; at long τ the DO tracks GPS; the loop guides the transition ever so
gently — no servo-induced noise", `CLAUDE.md`).

## Why the GNSSDO+ is the clean bench

Bravo's mid-τ-rise work concluded the rise was at least partly the servo
interacting with the **rx TCXO** (a second oscillator: the F9T's receiver
clock, separate from the steered DO). The **GNSSDO+ has only one oscillator** —
the mosaic-T runs off the steered OCXO — so that interaction is **ruled out**.
Any mid-τ TDEV above the DO's free-run floor here is the **servo's own
self-noise**, measurable directly. This makes it the ideal bench to isolate and
tune the servo contribution to zero.

## The evidence: the loop is *adding* noise below 1000 s

Free-run vs disciplined chA TDEV (measured 2026-07-06/07, de-glitched of the
GNSSDO PPS cycle-slips — see `gnssdo-plus-pps-alignment.md`):

| τ | free-run (Rakon) | disciplined | disciplined ÷ free-run |
|---|---|---|---|
| 1 s | 45 ps | 53 ps | 1.2× |
| 100 s | 54 ps | 187 ps | **3.5×** |
| 300 s | 170 ps | 380 ps | 2.2× |
| 1000 s | 690 ps | 902 ps | 1.3× |

The **free-running OCXO beats the disciplined output at every τ out to 1000 s.**
So below ~1000 s the loop is dragging the DO toward the noisier GPS/`dt_rx`
reference — in a band where the DO is *better than the reference*. The
crossover where GPS finally wins (ADEV floor 1.2e-12 for this OCXO) is **beyond
1000 s**.

## The mechanism: loop bandwidth set ~50× too high

The DO-frequency command is an LQR feedback on the DO phase/frequency states
(`peppar_fix/do_freq_est.py`):

```
u = −(L · x),   L = [0, 0, L2, 1.0]      # L2 = phase gain (default −0.05)
```

So each epoch the loop applies `u = |L2|·φ_do` of frequency correction per ns of
phase error. That's a first-order phase loop with **time-constant ≈ 1/|L2|
epochs** and **corner ≈ |L2|/2π Hz**. At the default `L2 = −0.05` and 1 Hz
actuation the corner is at **τ ≈ 20 s** — the loop tracks the reference for all
τ > 20 s, which is exactly why the reference's mid-τ noise (atmosphere,
multipath, orbit/clock corrections, PPP per-epoch noise) shows up on chA at
100 s.

**To let the OCXO free-run below ~1000 s, push the corner out ~50×:**
`L2 ≈ −0.001` → τ-constant ≈ 1000 epochs ≈ 1000 s. Below the corner the loop
gain < 1 (DO free-runs, its own noise passes through); above it the loop tracks
GPS. In closed-loop-transfer terms: **H ≈ 0 below the corner, H ≈ 1 above** —
the DO is high-passed, the reference low-passed, crossover at the corner.

### The oscillation is the same knob

The 2026-07-07 runaway (measured tight Q → growing ~120 s oscillation → rail)
was **estimation-lag-induced under-damping**: a stiff `sigma_do_freq` made the
frequency estimate `x[3]` slow to track a real DO frequency change, so phase
accumulated and the fixed high phase gain `L2 = −0.05` over-corrected → ring.
**Lowering `|L2|` fixes both problems at once** — it lowers the bandwidth (the
mid-τ-noise fix) *and* restores phase margin (the stability fix). Low bandwidth
and good damping are the same move here.

## Knobs (all per-host, GNSSDO+ wants very different values than DAC/PHC hosts)

- **`--lqr-phase-gain` / `lqr_phase_gain`** (new) — sets `L[2]`. Primary
  bandwidth knob. Default −0.05; sweep toward −0.001 for the GNSSDO+.
- **`--max-interval` / `max_interval`** (existing) — the Goldilocks/coast
  scheduler ceiling (default 120 s). Raise it so the loop can coast long
  between corrections; the DO's measured `coast_tdev` (45 ps @ 1 s, slope 0.82
  — an excellent OCXO) tells the scheduler it *can* coast far. Interval and
  gain both lower the effective bandwidth; use them together.
- **`coast_tdev`** (measured, in `state/dos/<uid>.toml [freerun_noise]`) — the
  DO's honest short-τ floor that bounds the coast. Reinstate the measured Q
  **only with a matched low `|L2|`** (the tight Q alone rings at the default
  gain — that is the whole point above).

## The A/B (this is what "the host is yours to learn from" buys us)

Sweep `--lqr-phase-gain` across a few settings (e.g. −0.05 baseline, −0.01,
−0.003, −0.001), each disciplined for a couple hours on the GNSSDO+, and
measure the **disciplined chA TDEV** (`char_gnssdo_freerun.py`, which
de-glitches the PPS cycle-slips) against the **free-run floor** (54 ps @ 100 s).

**Winner = the lowest-bandwidth setting that**
1. **stays stable** — no ringing, dt_rx bounded, zero servo resets over the run;
2. drives the disciplined chA TDEV **down onto the free-run OCXO curve below
   ~1000 s** (the mid-τ 3.5× gap → ~1×); and
3. still nulls the long-τ drift (disciplined TDEV stays flat where free-run
   rises past the crossover).

Because there is no second oscillator, the disciplined-minus-free-run TDEV gap
**is** the servo self-noise — the A/B tunes it to zero directly. Findings and
the chosen per-host `lqr_phase_gain` (+ any `max_interval`) land back here and
in `config/madhat-sxtd.toml`.

## A/B RESULT (2026-07-07): `L2` is the wrong knob — the frequency term governs both

Ran −0.05 (baseline) and −0.005 back-to-back, class-default Q, 2 h each,
`--no-antposest --known-pos`. **Both falsified the loop-bandwidth hypothesis:**

| gain L2 | corner (theory) | TDEV(100 s) disciplined | stable? |
|---|---|---|---|
| −0.05 | ~20 s | 187 ps | ran away @ ~2.8 h (measured-Q) |
| −0.005 | ~200 s | **172 ps** | stable 54 min, then **ran away to the rail** |

Two observations, one cause:

1. **Lowering L2 10× barely moved the mid-τ** (187 → 172 ps, still 3.3× the
   54 ps free-run floor). If L2 set the bandwidth, τ = 100 s (well inside the
   −0.005 corner) should have dropped toward the floor. It didn't.
2. **−0.005 ran away just like −0.05** — stable ~54 min, then a perturbation
   at 06:59 (our `dt_rx` glitched −12 ns while the mosaic `RxClkBias` ramped
   +53 → +68 ns), the loop recovered `dt_rx` to 0, then a **steady +3 ns/epoch
   phase ramp** wound `word` down to the rail (freq +386 ppb).

**Why L2 can't fix either:** the LQR command is

```
u = −(L · x) = −( L2·x[2] + 1.0·x[3] )      # x[2]=phase, x[3]=DO-freq estimate
```

The **frequency-cancellation term `−1.0·x[3]` has gain 1.0 and is independent of
`--lqr-phase-gain`.** It dominates the loop. So:

- The **effective loop bandwidth is set by how fast the EKF updates `x[3]`**
  (its frequency Kalman gain / `sigma_do_freq` process noise), **not by L2.**
  That is why the mid-τ TDEV didn't move — the L2 phase term is a minor
  contributor.
- The **wind-up is `x[3]` diverging**: a `dt_rx` perturbation gets mis-attributed
  into the frequency estimate, `u = −x[3]` drives the DO the wrong way, which
  produces more phase error, which pushes `x[3]` further — positive feedback on
  the gain-1.0 term. Lowering L2 leaves it untouched, so −0.005 railed too.

This also corrects §"The oscillation is the same knob" above: the earlier
measured-Q ring and this runaway are **both** the `x[3]` term, not L2.

### The real knobs (next experiments — NOT another L2 value)

1. **Robust innovation gating on `dt_rx`** so a carrier-phase glitch / receiver
   clock perturbation cannot corrupt `x[3]`. This is the stability fix — the
   runaway starts at a `dt_rx` perturbation the EKF swallows into frequency.
   **IMPLEMENTED (`--innov-gate-nsigma` / `innov_gate_nsigma`, 2026-07-07):** a
   soft NIS cap in every linear arm (`_kalman_linear_update`). An arm whose
   normalized innovation exceeds `N·√S` has its influence capped to `N` sigma by
   inflating `S` — the measurement is **down-weighted, never rejected**, so
   acquisition (legitimately large `dt_rx`) is untouched while a spike cannot
   punch `x[3]`. The gate is **inherently adaptive** (`S` contains `P`): a big
   innovation while `P` is large — acquisition — reads as few sigma and passes;
   the same jump at lock (small `P`) is capped. Note the runaway ramp itself is
   *self-consistent* (each epoch's innovation is small — the wrong `x[3]`
   predicts the drift it causes), so the gate does not catch the ramp; it
   catches the **seeding glitch** (the 06:59 −12 ns jump = 75σ at σ≈0.16 ns) so
   the ramp never starts. `madhat-sxtd.toml` sets `N = 5`. Per-arm
   `arm_gate_trips` counts trips (a healthy `dt_rx` arm trips ~never); the
   DT_RX log line shows `gate_trips=` once non-zero.

   **Hardware result (2026-07-07, 3 h, default gain −0.05 + gate=5):** knob 1
   converted the **rail into a bounded limit cycle**. No rail (freq bounded
   ±7 ppb vs the prior +386 ppb runaway), and **67 min of perfect lock**
   (phase ±0.2 ns, gate frozen at 19) — past the ~54 min point where the
   ungated −0.005 arm railed. Then a 2–3 s processing stall kicked the
   marginally-stable loop into a ±1700 ns limit cycle (freq ±7 ppb, our
   `dt_rx` and the mosaic `RxClkBias` swinging *together* → the DO physically
   oscillates; gate firing continuously, 19→6092 trips). **Clean-window
   mid-τ TDEV:** short-τ on the OCXO floor (1 s = 49 ps, 5 s = 27 ps vs 45 ps
   free-run) but the mid-τ hump persists (30 s = 274 ps, 100 s = 238 ps,
   300 s = 276 ps, ~5× the floor) — **unchanged by knob 1, as expected.**
   Verdict: knob 1 is **necessary but not sufficient** — it removes the
   catastrophic rail and buys clean lock, but the mid-τ resonance / limit
   cycle is a **loop-dynamics** problem (knob 2). Note the gate is
   double-edged in a *real* excursion: the large innovations are genuine (the
   DO truly is off), so capping them under-corrects and may prolong the
   oscillation — the clean cure is damping the loop (knob 2) so it never
   excurses, leaving the gate to fire only on true glitches.
2. **Slow the EKF frequency dynamics** (tighter `sigma_do_freq` / lower
   frequency Kalman gain) — *this* is the actual mid-τ bandwidth knob. The
   measured Q (tight `sigma_do_freq`) is the right direction for mid-τ; it was
   reverted only because it rang **at the default L2** — i.e. it hit the same
   `x[3]` instability. Tight-Q **plus knob 1** is the coherent experiment.
3. **Anti-windup / slew limit on `u`** as a backstop so a mis-estimated `x[3]`
   can't rail the actuator.

### The `ClockSyncThreshold, usec500, on` finding

The Mosaic-T is configured to steer/jump its own clock when `RxClkBias` exceeds
500 µs. At the 68 ns bias here it did **not** fire, so it was **not** this
runaway's trigger — but it is a latent hazard: if a future run lets the bias
grow unbounded, the Mosaic will apply a **1 ms jump** that would catastrophically
wind up the loop. For a DO we steer externally, evaluate setting the Mosaic to
**not** steer its own clock (report-only), so there is exactly one control loop.

**Do not run further L2 arms** (−0.001 etc.) until knobs 1–3 are addressed — the
sweep is measuring a term that isn't in control.

## DAC-host result (2026-07-25): on PiPuss + PiFace, L2 *is* an effective bandwidth knob — optimum −0.01

The §"L2 is the wrong knob" result above is **GNSSDO+-specific** (single
oscillator, pre-`innov_gate`, the `x[3]` runaway dominating). On the
DAC-actuator, **two-oscillator** lab clocks — with `innov_gate_nsigma` in place
so `x[3]` no longer diverges — sweeping `lqr_phase_gain` gives a clean,
reproducible bandwidth response, and L2 governs exactly the mid-τ-vs-long-τ
trade the theory predicts.

**PiPuss** (IsoTemp OCXO + F9T-20; 5 cycles × {−0.05, −0.01, −0.001}, 45-min
interleaved arms, detrended chA TDEV):

| gain L2 | corner | mid-τ (τ 16–32 s) | long-τ (τ 256–512 s) | whole-run RMS |
|---|---|---|---|---|
| −0.05 | ~20 s | 1.33 / 1.42 ns | 2.16 / 1.76 ns | 4.0 ns (2.0–**9.5**) |
| **−0.01** | **~100 s** | **1.28 / 1.33 ns** | **1.54 / 1.26 ns** | **2.6 ns (2.3–3.2)** |
| −0.001 | ~1000 s | 1.24 / 1.27 ns *(best)* | 2.09 / **3.25** ns | 6.7 ns (4.8–8.2) |

**PiFace** (IsoTemp OCXO131-100 + F9T-20B — same class as PiPuss; 3 cycles ×
{−0.05, −0.01, −0.003, −0.001}): same shape — −0.001 wins mid-τ (τ 64 s = 0.89 ns
vs −0.05's 1.29) but −0.003 and −0.001 **blow up at long τ** (τ 512 s RMS 13–14 ns
vs −0.01's 2.3 ns); −0.01 is the balance.

**Reading it:**

- **−0.01 (corner ~100 s) is the optimum for both**, now committed to
  `config/pipuss.toml` and `config/piface.toml`. It is ~20–30 % better than the
  −0.05 default at τ 64–512 s and far more consistent — it kills −0.05's
  occasional excursion-tracking blow-ups (the 9.5 ns RMS arm), which is what the
  two-clock **excursion** bound actually cares about.
- **−0.001 proves L2 is a real bandwidth knob here.** It drives the mid-τ down
  (the flywheel) but **blows up at long τ**, because the loop then tracks GPS too
  weakly and the DO drifts off it. That long-τ failure *is* the loop bandwidth
  acting — the opposite of the GNSSDO+'s "L2 barely moved anything."
- **Both hosts land at −0.01.** They are the same class — IsoTemp OCXO131-100 +
  F9T-20 on the shared UFO1 antenna — so −0.01 reproduces across two units, and
  the corner is pinned by the **shared long-τ GPS-tracking constraint** (same
  antenna + corrections + rx-TCXO class), not by unit-to-unit oscillator scatter.
  The mid-τ hump floor (~1.05–1.25 ns TDEV @ τ 32 s) is
  the DO free-run itself; only ~0.15 ns of it is removable servo self-noise. See
  the DO-ADEV ↔ two-clock-p95-agreement relationship (`σ_y(τ) ≤ 0.625·P/τ`) for
  why the DO free-run — not the servo — is the binding floor here.

**Open reconciliation with the GNSSDO+ finding:** the GNSSDO+ has **not been
re-swept since `innov_gate` landed**. The clean hypothesis is that the gate
stabilizes `x[3]`, restoring L2 as the effective bandwidth knob the theory
expected — a GNSSDO+ L2 re-sweep *with the gate active* is the test. Until then:
the Rakon's −0.001 target in this doc was calibrated for a **quieter
single-oscillator** DO; our two-oscillator hosts want **−0.01**.

## Watch-outs

- **NAV2 watchdog** still runs under `--no-antposest` and trips on a diverged
  clock (it caught the runaway). It is a backstop, not a nuisance — but for a
  clean long run pin the ARP well and keep the clock in-bounds.
- A very low bandwidth means **slow initial acquisition** — the loop takes
  ~1 corner-time to pull in. Bootstrap the DO frequency (drift seed) so it
  starts near-locked, or start at a higher `|L2|` and lower it once locked
  (gain scheduling — a later refinement).
- The measured OCXO `coast_tdev`/Q is authoritative for *this* Rakon; other
  GNSSDO+ units get their own characterization.
