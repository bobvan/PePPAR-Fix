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
| **noisy** | `N_run ≫ 1 ≫ N_obs` | parked per observation, different each time — **deterministic staircase, NOT white** (§5b) | tolerable, but *identifiable* |
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

## 5b. The "noisy" regime is a DETERMINISTIC STAIRCASE, not white noise

**Correction 2026-09-20 (BBGresearch, `ca19b51`), and it is in our favour.**
An earlier version of this doc called the noisy regime "~2.3 ns white."  It is
not white at all.  SYSTIM fires at the first tick at or past the target, so the
residue is

```
    e_k = T · ( 1 − frac( k·P / T ) )
```

— **deterministic in the cycle index `k`**.  Sampling once per output cycle
does not randomise it; it reproduces the sawtooth exactly.

For a **1 Hz output the per-cycle phase advance is `φ = frac(−adj_ppb / 8)`
exactly**, so an integer-ppb `adj` gives an **8-second periodic staircase of 8
levels, 1 ns apart, spanning the full tick.**  Verified arithmetically; both
our disciplined hosts land there:

| host | adj | φ | `e_k` sequence (ns) | period |
|---|---|---|---|---|
| clkPoC3 | +21 ppb | 3/8 | 8, 5, 2, 7, 4, 1, 6, 3, … | 8 s |
| PiFace | −135 ppb | 7/8 | 8, 1, 2, 3, 4, 5, 6, 7, … | 8 s |

**Three consequences, all of which make this easier to catch, not harder:**

1. **The detection test must predict a FREQUENCY, not an amplitude.**  Look
   for a **spectral line at the predicted `f_m`** in the phase residuals — not
   elevated scatter in small-`f_m` windows.  A matched periodicity is far
   stronger evidence than a matched magnitude, and it can only be passed by
   the right mechanism.  It also sharpens the thermal separation: **a ceiling
   fan does not produce an 8-second line at a frequency computable from
   `adjfine`.**

2. **The ADEV/TDEV signature is a set of NULLS at `τ = n/f_m` — not a bump.**
   *(Corrected 2026-09-20, `bdb2b70`; an earlier version of this doc said
   "bump near τ ≈ 1/(2 f_m)" and **looking there gives a false negative** —
   that τ is just the top of the envelope.)*  A periodic phase term gives an
   envelope falling as ~1/τ punctuated by minima at `τ = n/f_m`, going to an
   **exact zero** wherever `n/f_m` lands on a sampled τ.  Verified numerically
   on the staircases above:

   ```
   tau:      1      2      3      4      5      6      7      8      9     16
   clkPoC3 4.899  2.001  0.943  1.414  0.566  0.666  0.700  0.000  0.544  0.000
   PiFace  2.828  1.999  1.633  1.414  0.980  0.667  0.404  0.000  0.314  0.000
   ```

   **Both hosts null at τ = 8 s and 16 s.**  Not a coincidence: since
   `φ = frac(−adj_ppb/8)`, an *odd* integer-ppb `adj` makes the staircase
   repeat in exactly 8 samples whatever its value; an even one halves the
   period.  Either way the nulls land on multiples of a small integer number
   of seconds — squarely inside the decade we routinely plot.

   **Nulls are a far stronger test than a peak.**  Noise does not make minima,
   and a coincidence cannot match a *set* of null positions computed in
   advance.

   **Sensitivity caveat (ours):** in real data the staircase is one component
   among several.  The null is in the *staircase term only*, so the observable
   is a local minimum in the **total** curve, visible only where the staircase
   is a meaningful fraction of the total at that τ.  Absence of a visible dip
   bounds the term's size; it does not disprove the mechanism.

   **The caveat inverts into a measurement method (BBGresearch, `0f63c8f`).**
   Variances add, and the staircase contributes **exactly zero at the nulls**
   — so the total curve reads off `σ_other` *directly* there.  Interpolate
   that baseline across the nulls and subtract in quadrature at the
   intervening τ:

   ```
   σ_stair(τ) = sqrt( σ²_total(τ) − σ²_other,interp(τ) )
   ```

   **Interpolate in log–log, not linear** — `σ_other` falls steeply across
   that decade and a linear baseline biases the recovery high.

   So the test has **three outcomes and none of them is "inconclusive"**:
   deep nulls → *measure* the term; shallow nulls → measure it less
   precisely; no dip → quote an **upper bound**, which is a number that can go
   in a budget.

   **Precision collapses onto ONE parameter:** `r = σ_stair(τ) / σ_other(τ)`
   at the τ being recovered.  Propagating a fractional baseline error ε
   through `σ_stair = √(σ²_total − σ²_other)` gives

   ```
   fractional error in recovered amplitude  ≈  ε / r²
   ```

   **Verified here across 9 cells spanning 77× in `r`.**  Raw error spans
   0.2 % → 400 % (a factor of 2000); `err · r²` spans 11.9 % → 38.8 % (a factor
   of 3.3).  So the **`r²` scaling is real and is the right parameterisation** —
   amplitude and background do not enter separately.

   | r | recovery | branch |
   |---|---|---|
   | ≥ 3 | 1–2 % | a measurement |
   | ~1 | 10–20 % | an estimate |
   | < 0.5 | > 40 % | **bound only** |

   ⚠⚠ **SUPERSEDED — the prefactor difference was τ PLACEMENT, not setup.**
   *(2026-09-20, BBGresearch.)*  The `ε/r²` law is a law about an
   **extrapolated** baseline.  Both our τ sets put recovery points *below the
   first null*, where `σ_other` is extrapolated rather than interpolated.
   Recover **strictly between two measured nulls** and the picture changes
   completely — verified here, same 9 cells, same backgrounds:

   | | extrapolated (τ = 2,4,6) | **interior (τ = 9,12,14)** |
   |---|---|---|
   | r = 1.79 | 12.1 % | **0.1 %** |
   | r = 0.53 | 92.3 % | **0.8 %** |
   | r = 0.18 | 403 % | **14.2 %** |

   **Interior recovery is 30–120× better at the same `r`**, and the `r²`
   scaling stops being the binding term.

   > **THE RULE: recover only at τ strictly between two *measured* nulls.
   > Never below the first or above the last.**

   Interior-only branches:

   | r | error | verdict |
   |---|---|---|
   | ≥ 1 | < 0.5 % | a measurement |
   | ~0.5 | ~1–3 % | good estimate |
   | ~0.15 | ~11–14 % | estimate |
   | < 0.1 | > 50 % | bound only |

   **This moves the bound threshold from `r ≈ 1` down to `r ≈ 0.1`.**  A PHC
   on a plain XO near `r ≈ 1` is therefore in the **measurement** branch, not
   the estimate branch — our earlier conclusion that "the bound is the branch
   that matters" was too pessimistic by about an order of magnitude in `r`.

   **Two practical notes.**  The *measured* nulls are only those τ where
   `n/f_m` lands on a **sampled** τ — for clkPoC3's `f_m = 0.375 Hz` that is
   8, 16, 24, **not** 2.67 or 5.33 — so the usable interior is τ ≈ 9…23.  And
   within that span `σ_stair` still dips at the *unsampled* nulls (10.67,
   13.33), so choose recovery τ away from those, where `r` is largest.

   **Negative result, recorded so nobody re-tries it:** BBGresearch first
   guessed the prefactor was simply `ε` — the baseline interpolation's own
   fractional error — and tried estimating it by leave-one-null-out
   cross-validation.  It **failed**, spreading by a factor of 144.  The
   prefactor is not the baseline's self-consistency; it is *whether you are
   interpolating or extrapolating at all*.

   The superseded analysis, kept as the instance:

   ⚠ **The exponent transfers; the prefactor does not.**  BBGresearch measured
   `err·r²` ≈ **10.4 %** (spread 7.7–12.4 %); we measure **29.7 %** (spread
   11.9–38.8 %) on the same law.  The difference is in setup details — how `r`
   is defined, which τ are averaged, interpolation specifics — so **calibrate
   the prefactor on your own synthetic data before quoting an accuracy.**
   Note the recursion: reparameterising to remove a frame-dependent *table*
   left a frame-dependent *constant*.  The law also degrades below `r ≈ 0.5`,
   where clipping and the √ nonlinearity bite.

   **Operationally this is worse than "estimate `r` first": precision degrades
   as the SQUARE, so 3× more background costs 10× in recovered accuracy.**  For
   a PHC on a plain XO `r` may sit near 1 — the *estimate* branch, which makes
   the **bound** the branch that matters for us.

   The underlying per-cell measurements, kept as the instance:

   | background | 8.0 ns pk-pk | 2.4 ns | 0.8 ns |
   |---|---|---|---|
   | light (white 0.2 ns + RW 0.01) | 0.2 % | 2.7 % | 20 % |
   | moderate (white 0.5 ns + RW 0.05) | **1.5 %** | **13 %** | 90 % |
   | heavy (white 1.5 ns + RW 0.25) | 12 % | 90 % | 400 % |

   The middle row reproduces BBGresearch's quoted 1.3 % / 13 % / 77 % almost
   exactly, which identifies the background their table assumes.  **Do not
   quote those figures as a property of the technique** — under a heavier
   background the same method is ten times worse.  Estimate our own ratio
   before deciding what a null-depth measurement can resolve.

   **Governing caveat (theirs, and it governs the whole test):**
   `d f_m/d adj = 125×10⁶`, so **1 ppb of servo motion moves every null.**
   Compute `f_m` from the `adjfine` logged *during that run*, never from
   today's value.  A null at today's predicted τ in a run taken at a different
   `adj` is a coincidence; a null at the "wrong" τ is not a refutation.  And
   if `adj` moved appreciably *within* a run, `f_m` swept and the nulls smear
   into **elevated broadband TDEV across a decade** — which is **consistent
   with the mechanism, not evidence against it.**  Segment long runs by `adj`
   before looking.

   **That gives a split prediction, which is stronger than either half:** a
   run with *stable* `adj` should show clean nulls at computable τ; a run with
   a *moving* servo should show a raised, featureless decade.  If our archive
   contains both kinds and they split that way, that is a better result than
   either observation alone.

3. **It is a ZERO-parameter prediction, not a one-parameter fit.**  `e_k`
   depends only on `k` and `T`, and we log both — `T` from `adjfine`, `k` from
   the cycle count.  Reserve one phase parameter in practice to absorb the
   counter origin, and accumulate `φ` across the log rather than assuming it
   constant (since `adj` moves).  But the ambition is **predict and subtract**,
   not fit and subtract — which matters because **the residual after
   subtraction is then a test of the model rather than a consequence of having
   fitted to the data.**

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
- **PREDICT and subtract it in software — probably the right answer.**  Per
  §5b this is a *zero*-parameter prediction: amplitude is one tick, frequency
  comes from logged `adjfine`, and the sequence is determined by the cycle
  index.  (An earlier version of this doc called it a one-parameter fit; that
  undersold it.)  Subtracting a fitted sawtooth is far cheaper than escaping
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
