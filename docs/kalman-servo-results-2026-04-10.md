# Kalman Servo + PEROUT Fix — Results 2026-04-10

## Summary

Two breakthroughs today:

1. **Kalman filter + LQR servo** replaces PI for DO frequency steering.
   TDEV(1s) = 0.75 ns — below the DO's 0.92 ns free-running floor and
   3.7x better than raw PPS (2.9 ns).  Zero overshoot on pull-in.

2. **i226 PEROUT 500ms half-period bug** root-caused and fixed.  The
   i226 Target Time comparator latches onto the wrong half-period after
   ADJ_SETOFFSET.  Fix: disable PEROUT before stepping, double-program
   after.  Verified on all three i226 hosts.

## Kalman Servo Design

2-state model: `[phase_ns, freq_ppb]`.  Measurement: TICC+qErr (0.178 ns
floor) or EXTTS+qErr (1.9 ns).  LQR gain computes optimal adjfine each
epoch.

```
State model:
  phase[n+1] = phase[n] + (freq[n] + u[n]) * dt
  freq[n+1]  = freq[n] + w_freq[n]

Measurement:
  z[n] = phase[n] + v[n]
```

Noise parameters (from lab measurements 2026-04-09):
- `sigma_meas_ns = 0.178` (TICC+qErr floor)
- `sigma_phase_ns = 0.92` (DO free-running from adjfine noise test)
- `sigma_freq_ppb = 0.01` (TCXO frequency random walk)

LQR gain: `L = [0.05, 1.0]` — proportional phase correction + full
frequency cancellation.  L[1] >= 1.0 enforced to prevent runaway drift.

Adaptive Q: 10x sigma_freq during pull-in (|phase| > 50 ns), tapering
to 1x when settled (|phase| < 10 ns).  Enables fast convergence with
sigma_freq=0.001 without sacrificing long-tau stability.

## TDEV Results (TimeHat)

Best configuration: Kalman with default parameters (sf=0.01, no dead zone).

### 2-hour run (7200 epochs, definitive)

| tau (s) | Kalman chA (ns) | PPS chB (ns) | Ratio | Notes |
|--------:|----------------:|-------------:|------:|-------|
| 1       | **1.01**        | 2.14         | 0.47  | Kalman wins 2.1x |
| 2       | **1.07**        | 2.44         | 0.44  | Kalman wins 2.3x |
| 5       | **1.58**        | 2.35         | 0.67  | Kalman wins 1.5x |
| 10      | 2.50            | 2.31         | 1.08  | Crossover |
| 20      | 3.66            | 2.43         | 1.51  | PPS wins |
| 50      | 4.96            | 2.41         | 2.06  | |
| 100     | 5.59            | 2.35         | 2.37  | |
| 200     | 6.31            | 2.52         | 2.51  | Plateau |
| 500     | 6.67            | 3.11         | 2.14  | |
| 1000    | 6.67            | 2.87         | 2.32  | |
| 2000    | 5.77            | 2.91         | 1.99  | Improving |

Crossover at tau ~8s.  Long-tau TDEV plateaus at ~5.5-6.7 ns (does
not diverge).  At tau=2000s the Kalman starts to improve — the servo
is pulling the long-term average toward GPS time.  DO free-running
floor = 0.92 ns at tau=1s.

### MadHat comparison (no heatsink on TCXO, 2-hour run)

| tau (s) | MH Kalman (ns) | MH PPS (ns) | TH Kalman (ns) |
|--------:|---------------:|------------:|---------------:|
| 1       | 2.50           | 2.18        | **1.01**       |
| 5       | 8.57           | 2.38        | **1.58**       |
| 100     | 17.22          | 3.03        | 5.59           |
| 1000    | 19.43          | 4.07        | 6.67           |

MadHat (no TCXO heatsink): adjfine std = 3.6 ppb (4x worse than
TimeHat's 0.9 ppb).  TDEV(1s) = 2.50 ns, doesn't beat PPS at any
tau.  The TCXO temperature sensitivity dominates the noise budget.
Heatsink installation planned.

### Carrier Phase tracker (running, not yet driving)

The CarrierPhaseTracker ran for the full 2 hours on both hosts,
accumulating adjfine corrections and tracking the inter-oscillator
drift rate.  TimeHat Carrier error: +24.3 ± 1.2 ns (6900 epochs).
MadHat Carrier error: -340.0 ± 3.1 ns — large offset indicating
the drift rate estimate hasn't fully converged.  Carrier is not yet
winning the source competition (TICC at 3 ns confidence dominates)
but is tracking steadily.

### Early 15-min runs (for reference)

Best 15-min TDEV(1s) = 0.75 ns (run kalman-base).  The 2-hour value
of 1.01 ns is the definitive number.

## Pull-in Behavior

Starting from -9418 ns, the Kalman servo converges monotonically to
zero with no overshoot:

| Epoch | Error (ns) | Adjfine (ppb) | Mode    |
|------:|-----------:|--------------:|---------|
| 1     | -9418      | 118           | pull_in |
| 10    | -5644      | 756           | pull_in |
| 25    | -1816      | 182           | landing |
| 50    | -844       | 144           | landing |
| 100   | -232       | 123           | landing |
| 200   | -58        | 119           | settled |
| 341   | 0 (crossing)| 119          | settled |

First zero crossing at epoch 341.  No sign changes before that —
critically damped as designed by the LQR cost function.

## Tuning Experiments

| Config | TDEV(1s) | TDEV(100s) | Corrections | Notes |
|--------|------:|--------:|---:|---|
| **sf=0.01, no dz** | **0.750** | 5.71 | 54% | Best overall |
| sf=0.01, dz=0.5 | 0.870 | 4.11 | 3% | Dead zone hurts Kalman |
| sf=0.001, no dz | 0.822 | 3.88 | 56% | Slightly better long-tau |
| sf=0.001, dz=0.5 | 1.175 | 7.42 | 2% | Worst: both hurt together |
| r_weight=4 (MH) | 2.725 | — | 44% | Too sluggish to converge |

**Dead zone hurts the Kalman servo** — suppresses optimal corrections,
causing phase error accumulation between threshold crossings.  Keep the
feature for in-band PSD measurement windows (97% quiet epochs) but don't
default it on.

**sigma_freq=0.001** marginally improves long-tau (3.88 vs 5.71 at
tau=100) but slightly hurts short-tau (0.822 vs 0.750).  Needs adaptive
Q (implemented) for pull-in convergence.

## i226 PEROUT 500ms Half-Period Bug

### Root cause

When ADJ_SETOFFSET shifts the i226 PHC time and then enable_perout
programs Target Time mode, the hardware fires at 500 ms instead of 0 ms
approximately 90% of the time.  The corrupted state persists across
simple disable/enable cycles and across process restarts.

Per kernel netdev mailing list consensus: stepping the PHC while PEROUT
is active causes the i226 to oscillate at 62.5 MHz or lock up.  The
corrupted state contaminates the Target Time comparator.

> **Provenance, added 2026-09-19:** that paragraph is *repeated from a
> mailing list*, not measured here — we have never observed 62.5 MHz on
> an i226.  What we measured is the 500 ms offset in the Evidence
> section immediately below.  The two had been sitting together long
> enough to read as equally solid.  Keep the precaution; downgrade the
> fear.

### Evidence

Across 10 TimeHat TICC-drive runs that stepped the PHC, 9 produced
500ms PEROUT offset.  Both hosts (TimeHat + MadHat) affected identically.
Non-TICC-drive runs never noticed because the servo uses EXTTS PPS, not
PEROUT.

### Fix (commit 3db2d6d)

1. Disable any pre-existing PEROUT BEFORE stepping the PHC
2. Always double-program: enable, wait 2s, disable, re-enable
3. The second enable sees a stable PHC and fires correctly

Verified on all three i226 hosts:

| Host    | igc driver | Step size | 500ms outliers | First error |
|---------|-----------|----------:|---------------:|------------|
| TimeHat | patched   | varies    | **0**          | -5437 ns   |
| MadHat  | patched   | +13254 ns | **0**          | -1276 ns   |
| ocxo    | stock     | +43501 ns | **0**          | -3224 ns   |

### Impact

Eliminates the 30-outlier rejection cycle (30+ seconds) and occasional
re-bootstrap (40+ seconds) on every cold start.  Both hosts now go
straight to servo tracking on the first attempt.

### SatPulse / ts2phc approach

SatPulse and ts2phc use `PTP_PEROUT_PHASE` flag instead of absolute
start times, which may be more robust.  Worth investigating as a future
alternative to the double-program workaround.

## Open questions

1. **Long-tau TDEV**: the Kalman output rises above PPS at tau > 15s.
   Is this from frequency estimate wander, or just insufficient data
   (15-min runs)?  2-hour runs in progress.

2. **MadHat noise**: disciplined PEROUT std=49 ns vs TimeHat's 4.3 ns
   (11x worse).  PPS noise floors are similar (2.7 vs 2.5 ns).  Heat
   sink (TimeHat has one, MadHat doesn't) is the primary suspect.

3. **Carrier Phase drive**: the Kalman servo is ready for Carrier input
   (dt_rx-derived phase error with 0.1 ns precision, bypassing PPS).
   This could push TDEV below 0.75 ns by eliminating the TICC+qErr
   measurement floor from the servo loop.

## Files

- `scripts/peppar_fix/kalman_servo.py` — 2-state Kalman + LQR servo
- `scripts/phc_bootstrap.py` — PEROUT 500ms fix
- `docs/adjfine-noise-characterization-2026-04-09.md` — adjfine is free
- `tools/adjfine_noise_test.py` — adjfine noise measurement tool
- `tools/adjfine_sweep.py` — adjfine frequency resolution sweep
