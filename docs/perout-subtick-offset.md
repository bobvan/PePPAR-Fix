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

## 3. What that gives on our actual hosts

| adj | 125e6·adj | f_m | sawtooth period |
|---|---|---|---|
| +1.2 ppm (TimeHat free-run) | 150.000000 | **0 Hz** | never — **static** |
| +1.2000001 ppm | 150.000013 | 12.5 µHz | 80,000 s — static in practice |
| −135 ppb (PiFace) | −16.875 | 0.125 Hz | 8 s — dithers, averages |
| +21 ppb (clkPoC3) | +2.625 | 0.375 Hz | 3 s — dithers, averages |
| 0 (perfect) | 0 | **0 Hz** | never — **static** |

**The sensitivity is the story: `d f_m / d adj = 125×10⁶`, so a 1 ppb change
in `adj` sweeps `f_m` by 0.125 Hz.**  A disciplined servo moving `adj` at the
ppb level therefore sweeps `f_m` across the whole 0–0.5 Hz range continuously
— and **sweeps through the near-degenerate values where `f_m → 0` and the
offset parks quasi-statically at up to ±4 ns.**

So this is not a constant bias we could calibrate out once.  It is an
**intermittent, servo-state-dependent, quasi-static offset** whose duration
depends on how slowly `adj` happens to be crossing a degenerate point.

## 4. The test — retrospective, on data we already have

`f_m` is **computable from logged `adjfine`**: `f_m = frac(125e6 · adj)`.  We
log `adjfine` every epoch and TICC chA every second.  So:

1. For an existing capture, compute `f_m(t)` from the servo log.
2. Find the windows where `f_m` is small (say < 10 mHz — the offset is then
   static over minutes).
3. Ask whether TICC chA shows an unexplained quasi-static offset in exactly
   those windows, and dithers in the others.

**A positive result would explain an intermittent ±4 ns excursion class that
appears and disappears with no external cause** — a signature we have chased
before and attributed elsewhere.  A negative result bounds the effect and
retires it.  Either way it costs one analysis pass, no hardware.

## 5. Why it matters

Our cross-host agreement targets are **1 ns (shared antenna)** and **2 ns
(separate antennas)**, 95 %.  A per-host quasi-static offset of up to ±4 ns
would dominate both — and because it depends on each host's own `adj`, it
does **not** cancel between hosts.

## 6. What would remove it

- **Choose a period that is not an integer number of ticks** — deliberately
  land in a fast-dither regime.  The arithmetic above says the healthy case is
  a large `frac`, not a small one.  For a 1 Hz output this is not available
  without changing the output rate.
- **A phase interpolator rather than a counter comparison.**  See the general
  form in §7.
- **Measure and correct per-host**, accepting that the correction is only
  valid while `adj` stays away from a degenerate point.

## 7. The general question, worth applying to any candidate part

> Is the output edge placed by a **counter compared against a target in a
> clocked domain**, or by a **genuine phase interpolator**?

If it is a counter — which includes an FPGA signal generator *and* a plain
integer output divider on a clock chip — the tick is present, equal to that
domain's clock period, and this whole analysis applies with 8 ns replaced by
whatever that period is.

## Related

- `docs/two-clock-agreement-forward-model.md` §4.1 — the *input*-side twin of
  this arithmetic: 1 s being an exact multiple of 8 ns is also why EXTINT
  quantisation stops dithering when the receiver is clocked from the DO.
  Same number, same cause, opposite direction of signal flow.
- `docs/i226-perout-500ms-bug.md` — a different, larger PEROUT defect.
- `docs/peroutVS-phc-time.md` — measured PEROUT-vs-PHC offsets (µs-scale,
  a separate mechanism).
