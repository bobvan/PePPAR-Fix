# The PEROUT sub-tick offset — our 1 Hz PPS OUT is in the degenerate family

**2026-09-20.  Mechanism reported by BBGresearch; arithmetic and
applicability to our fleet worked out here.  NOT yet tested against our
data — §4 is the test, and it runs on captures we already have.**

## 1. The mechanism

An i210/i226 PEROUT edge is placed by comparing a **target-time register
against SYSTIM**, and SYSTIM advances in **8 ns ticks**.  A requested period
that is not an exact integer number of ticks therefore lands the edge on a
sawtooth: the fractional residue accumulates and resets.

The tick is not exactly 8 ns.  Per the i210 datasheet (TIMINCA, §8.15.11),
SYSTIM advances by `8 ns ± Incvalue × 2⁻³² ns` per cycle, so the **effective
tick is `8(1 + adj)`** where `adj` is the frequency correction in use — which
for a disciplined clock is, in steady state, the crystal's own error.

The sawtooth frequency is therefore

```
    f_m = f_out · frac( period_ns / (8 · (1 + adj)) )
```

**BBGresearch's key finding: the `frac = 0` family is the dangerous one.**
Its null sits at `adj = 0`, so as the oscillator or the servo gets *better*
the sawtooth slows toward a stop and degenerates into a **static half-tick
offset — up to ±4 ns — that no amount of averaging removes.**  Better
discipline, worse artefact.  Their measured 4395 Hz dither averages 2.31 ns
rms down to ~53 ps in a 0.437 s capture; a static 4 ns offset stays 4 ns.

## 2. We are in that family

Our PPS OUT is a **1 Hz PEROUT off the PHC**:

```
    1 s / 8 ns = 125,000,000 ticks   exactly.   frac = 0.
```

So `f_m = frac(125×10⁶ · adj)` Hz, and the whole behaviour is governed by
where `125e6 · adj` falls relative to an integer.

## 3. Four regimes — and the worst one is the MIDDLE, not the static end

**Correction 2026-09-20 (BBGresearch, `subtick-sawtooth-regimes.md`).  An
earlier version of this doc framed `f_m → 0` as the hazard.  That is wrong:
a truly static offset is at least calibratable in principle.**  Classifying it
needs *two* timescales — `T_obs` (one observation) and `T_run` (the whole
comparison) — with `N = f_m · T`:

| regime | condition | what you see | severity |
|---|---|---|---|
| **dithers** | `N_obs ≫ 1` | averages away inside each observation | benign |
| **noisy** | `N_run ≫ 1 ≫ N_obs` | parked per observation, different each time → ~2.3 ns white, no bias | tolerable |
| **masquerades** | **`N_run ≈ 1–10`** | **sweeps once across the run — looks exactly like clock wander** | **worst** |
| **parks** | `N_run ≪ 1` | one static offset, uniform in ±4 ns, different per host | bias, not noise |

The middle case is **too slow to average and too fast to calibrate, and it
arrives wearing the costume of the thing we are trying to measure.**  Against
1 ns and 2 ns cross-host targets, an 8 ns pk-pk sweep across a run is not a
perturbation — it is the answer.

## 4. A 1 Hz output is a special case, and it is not about `adj`

`f_m` aliases at `f_out / 2`.  **For a 1 Hz output that is 0.5 Hz, so
`N_obs ≤ 0.5` at one-second sampling for *every possible value of `adj`*.**

> **A PPS generated from a PHC is always in a parked or noisy regime at 1 s.
> There is no oscillator good enough, or bad enough, to dither it away.**

This corrects the host table this doc previously carried, which described
PiFace (`f_m` 0.125 Hz) and clkPoC3 (0.375 Hz) as *"dithers, averages."*  They
do not average **within** an epoch — at 1 s sampling those are `N_obs` of 0.125
and 0.375.  They average over a long enough **run**, which puts them in the
*noisy* row (≈2.3 ns white, no bias), not the benign one:

| adj | 125e6·adj | f_m | N_obs @1 s | regime at T_run = 100 s |
|---|---|---|---|---|
| +1.2 ppm (TimeHat free-run) | 150.000000 | 0 Hz | 0 | **parks** |
| +1.2000001 ppm | 150.000013 | 12.5 µHz | ~0 | **parks** |
| −135 ppb (PiFace) | −16.875 | 0.125 Hz | 0.125 | noisy (N_run 12.5) |
| +21 ppb (clkPoC3) | +2.625 | 0.375 Hz | 0.375 | noisy (N_run 37.5) |

Escaping into *dithers* needs `T_obs ≫ 2 s`.  This is why the same mechanism
is a curiosity for a 32 kHz comb (`N ≈ 1900` in a 0.44 s capture) and an error
budget for a 1 PPS: **the output rate, not the oscillator.**

## 5. Nulls every 8 ppb — and SHORT runs are the dangerous ones

`f_m = 0` whenever `125e6 · adj` is an integer, i.e. **every 8 ppb of `adj`**.
A servo wandering at the ppb level visits a null *periodically, not rarely*.

Taking "fails to average over the run" as `N_run < 10`:

| run | `f_m` needed | half-width in `adj` | duty in the bad zone |
|---|---|---|---|
| 100 s | 0.10 Hz | 0.80 ppb | **20 %** |
| 1000 s | 0.010 Hz | 0.080 ppb | 2.0 % |
| 10000 s | 0.0010 Hz | 0.0080 ppb | 0.20 % |
| 1 day | 0.00010 Hz | 0.0009 ppb | 0.02 % |

**Short runs are the dangerous ones**, which inverts the usual intuition and
is worth knowing before anyone "fixes" an anomaly by shortening a comparison.

## 5a. The test, with a free discriminator

`f_m` is computable from logged `adjfine`: `f_m = frac(125e6 · adj)`.  So on an
existing capture: compute `f_m(t)`, find the windows where `N_run` is small,
and ask whether TICC chA parks or sweeps in exactly those windows.

**The discriminator that makes it a one-pass test:** this signature correlates
with **`f_m`, i.e. servo state**, and *not* with temperature, load, or link
activity.  A thermal explanation and this one make **opposite predictions about
which windows are bad**, so a single analysis separates them — which matters,
because we have attributed excursions to thermal causes before
([[feedback_do_discontinuity_means_external_force]]).

## 6. What would remove it

- **Choose a period that is not an integer number of ticks** — deliberately
  land in a fast-dither regime.  The arithmetic above says the healthy case is
  a large `frac`, not a small one.  For a 1 Hz output this is not available
  without changing the output rate.
- **A phase interpolator rather than a counter comparison.**  See the general
  form in §7.
- **Measure and correct per-host**, accepting that the correction is only
  valid while `adj` stays away from a degenerate point.
- **Fit and subtract it in software — probably the right answer.**  The
  sawtooth has **known amplitude** (one tick) and **known frequency** (from
  logged `adjfine`); only its **phase** is unknown.  That is a *one-parameter
  fit* over a run.  Subtracting a fitted sawtooth is far cheaper than escaping
  it in silicon, and a defect this well characterised is a candidate for
  correction rather than replacement.  Derived, not yet demonstrated on either
  project's data — but worth trying before anyone specifies a clock generator
  on account of this.

## 7. The general question, worth applying to any candidate part

> Is the output edge placed by a **counter compared against a target in a
> clocked domain**, or by a **genuine phase interpolator**?

If it is a counter — which includes an FPGA signal generator *and* a plain
integer output divider on a clock chip — the tick is present, equal to that
domain's clock period, and this whole analysis applies with 8 ns replaced by
whatever that period is.

## 8. A boundary that must NOT be carried across

Everything above is for a **time-domain** estimator — our TICC.  A *spectral*
estimator behaves oppositely: the sawtooth puts **discrete sidebands** at
`±k·f_m` on every harmonic, and a fitter does not average them, it *competes*
with them.  Longer integration does not help at all.

> **Time-domain: the enemy is `f_m` being too *slow*.
> Frequency-domain: the enemy is the sideband being too *strong*, and `f_m`
> only sets where it sits.**

Same artefact, opposite remedies.  Our instinct to make `f_m` large is right
for us and would be irrelevant to a spectral measurement.

## Related

- `docs/two-clock-agreement-forward-model.md` §4.1 — the *input*-side twin of
  this arithmetic: 1 s being an exact multiple of 8 ns is also why EXTINT
  quantisation stops dithering when the receiver is clocked from the DO.
  Same number, same cause, opposite direction of signal flow.
- `docs/i226-perout-500ms-bug.md` — a different, larger PEROUT defect.
- `docs/peroutVS-phc-time.md` — measured PEROUT-vs-PHC offsets (µs-scale,
  a separate mechanism).
