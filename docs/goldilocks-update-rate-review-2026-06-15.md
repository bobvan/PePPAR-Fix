# Goldilocks vs. faster update rate — premise review (2026-06-15)

A second-look at the `fasterUpdateRate` (I-fasterUpdateRate-main) effort,
questioning whether running the loop above 1 Hz is consistent with the
premises the adaptive Goldilocks scheduler is built on. **Conclusion: for
current plants it is largely contrary, and the effort's payoff is conditional
on a low-σ_q actuator we don't have yet. Refocus on measuring σ_q.**

## The conflation

"Update rate" bundled two different levers the Goldilocks work treats
separately:

- **Observation (measurement) rate** — how often we *estimate*. More obs →
  better estimate; **no actuator cost**.
- **Actuation rate** — how often we *correct the DO*. **Each actuation injects
  the actuator quantization noise σ_q.**

The Goldilocks premise is *only* about actuation: τ_opt is the coast where the
DO's free-running phase wander over τ equals σ_q. **Below τ_opt, σ_q dominates
→ correcting more often is net-worse.** That is the entire reason the adaptive
scheduler exists (TimeHat: 1 Hz was *worse* than coasting because σ_q is
ns-class, not the LSB floor — see adaptive-scheduler-motivation memory).

## The crux datum

The scheduler computes, consistently across every clkPoC3 run:

```
scheduler: predicted Goldilocks τ=1.453s (σ_q=0.0231 ns, k·σ_DO(1s)=0.0189 ns, slope=0.53)
```

**τ_opt ≈ 1.45 s > 1 s.** So at 1 Hz the scheduler already wants to coast
~1.45 s (every 1–2 epochs) — it is *near* Goldilocks-optimal, not
measurement-rate-limited.

## Is faster *observation* contrary to the premise? No.

Faster observation is orthogonal (better estimate, no σ_q). It only *matters*
if τ_opt < 1 s (you'd want to actuate between measurements but can't). It is in
fact the Goldilocks-*consistent* way to reach a sub-second τ_opt — which is
exactly what **mode A (rate-aware adaptive)** does. Mode A can only match or
beat 1 Hz; on clkPoC3 it would just confirm τ_opt ≈ 1.45 s and change nothing.

## Is mode B (forced fast actuation) contrary? Yes — provably, here.

`--fire-every-epoch` actuates at 0.2 s regardless of τ_opt. With τ_opt = 1.45 s
that is **~7× over-actuation**: ~7× the σ_q-injection events to buy only
√(0.2/1.45) ≈ 0.37× of the DO-wander reduction. Below τ_opt, σ_q dominates by
construction → net-worse. **Mode B disables the very logic the scheduler is
built on**, and is provably contrary to the Goldilocks premise for any plant
with τ_opt > 1/rate — which clkPoC3's own scheduler says is the case.

Corollary for the mid-τ bump (τ32 = 270 ps, τ64 = 527 ps) we were trying to
shrink: at τ < τ_opt the loop is *intentionally coasting*, so that bump is the
**DO's own free-running wander, not a loop-bandwidth artifact.** Faster
actuation would *replace* DO-coast-wander with σ_q-injection — it cannot shrink
the bump unless σ_q ≪ DO-wander, which is the opposite of the premise.

## Reasons to question the premise itself

The *conclusion* (coast; don't over-actuate) is robust for current hardware,
but two assumptions are genuinely open:

1. **σ_q is unmeasured and assumed fixed-per-actuation.** τ_opt = 1.45 s uses
   the *theoretical LSB* σ_q (0.0231 ns); the real σ_q is likely ns-class. A
   bigger real σ_q → *larger* τ_opt → faster is even more contrary. **But the
   deeper question:** if σ_q scales with correction *magnitude* (smaller steps
   = less noise — plausible for a DAC with relative/slew-dependent error), then
   smaller-more-frequent corrections could carry *less total* injected noise,
   the breakeven model is wrong, and faster could win. Only answerable by
   **measuring σ_q as a function of step size.** This is the "unknown is the
   actuator noise."
2. **The model ignores rx-reference-noise tracking.** A higher-bandwidth loop
   tracks *more* rx-TCXO/GNSS reference noise into the DO, threatening the
   moonshot's "preserve the DO's superior short-term stability." Goldilocks
   balances DO-wander vs σ_q but does not account for this at all — faster
   could hurt short-τ for a reason the model never sees.

## Verdict & refocus

- For **today's plants** (clkPoC3 OCXO+AD5693R, TimeHat TCXO) τ_opt ≥ 1.45 s →
  1 Hz is already near-Goldilocks-optimal; faster actuation is contrary and
  faster measurement is marginal. The bump is the DO, not the loop.
- The effort's real payoff is **conditional on a low-σ_q actuator** (τ_opt < 1 s)
  — i.e. it is an argument *for* the AD5781 18-bit-DAC upgrade
  (two-site-sync-budget.md §DAC), not for faster rate on current hardware.
- **Refocus:** measure σ_q empirically — and its step-size dependence (premise
  #1) — then recompute τ_opt with the real σ_q. Faster rate is warranted only
  if τ_opt < 1 s (or σ_q shrinks with step size).
- The `meas_rate_hz` plumbing + rate-aware adaptive (mode A) is kept (1 Hz
  byte-identical, ready for a future low-σ_q plant). **Mode B
  (`--fire-every-epoch`) is a σ_q diagnostic, not a deployment mode.**

## Connection to the oscillator-characterization-consistency review

τ_opt, the DOFreqEst Q, and holdover all read the *same* DO characterization
(σ_DO, σ_q, the freerun TDEV). The `sigma_do_phase` override seen at startup
("0.0105 ns/√s, was 0.9200") is an ~88× correction — the kind of inconsistency
that, if a wrong value reaches the scheduler/Q, makes a host fail to lock (cf.
PiFace, I-100945-main). **You cannot trust τ_opt or measure σ_q meaningfully
until characterization is done one consistent way.** So the σ_q-measurement
refocus is the first concrete step of a broader characterization-consistency
review (filed separately).
