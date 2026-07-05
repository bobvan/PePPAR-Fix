# Mid-τ loop-resonance hump — servo_sim diagnostic (I-084500)

**Owner:** bravo · **Date:** 2026-07-05 · **Status:** diagnostic complete,
levers scoped, two-clock harness prerequisite identified.

Picks up `midTauLoopResonanceHump` (I-084500-main → bravo). The diagnosis
that the τ≈10–30 s hump is a *realization* gap (not the reference) was
already established (PR #220 de-sawtooth; overnight cal A/B showed the
timestamper cal is orthogonal to the hump). This doc is the **lever
investigation on `servo_sim`** — the go/no-go rig — before any lab A/B.

## TL;DR

1. **The hump reproduces on `servo_sim`** and is faithful to the field
   number: settled clkpoc3 shows a mid-τ TDEV hump ≈ **394 ps @ τ≈32–64 s
   vs the 150 ps budget = 2.6×** (dayplan/softGate analysis quotes
   "clkPoC3 ~3×"), phase excursion ≈ 2.7 ns (the dayplan's "~5 ns").
2. **Detrend/acquisition artifact caveat (load-bearing):** computing TDEV
   over the *whole* run — including the 200 ns bootstrap pull-in — inflates
   the apparent hump ~2.5× and moves its peak (966 ps @ τ=8 s vs the true
   394 ps @ τ=64 s). **Always grade on the settled portion (skip ≥ 300 s),
   reset-free, matching the lab A/B protocol.** The first-pass "levers are
   invariant" conclusion was this artifact and is retracted.
3. **The dayplan's lever #2 ("stiffer-Q / coast — trust the DO flywheel at
   mid-τ") is half-wrong.** *Coasting is counterproductive for the mid-τ
   hump*: coast_interval 8 s → hump 1091 ps, 32 s → 4726 ps; coast-cap
   barely helps. Coast reduces τ=1 s (fewer actuations = less injected
   actuation noise) but the DO free-runs un-corrected exactly at the τ where
   the hump lives. Only the *stiffer Q[3,3]* half helps, and modestly
   (394→340 ps).
4. **The most effective single lever is `soft_ticc_gate` (#217 softGateMidTau)
   — on clkpoc3.** It selectively down-weights the bad-innovation
   (sawtooth-crossing / jitter-tail) TICC epochs (always-admit R-inflation)
   instead of coasting through them: hump 394→329 ps, rise 3.5×→2.7×, τ=1 s
   preserved, excursion 2.7→2.0 ns. Best combo: **softgate + Q[3,3]=0.003 →
   300 ps @64 s, rise 2.8×, τ=1 s 108 ps, excursion 1.84 ns.**
5. **Grade on the two-clock p95 excursion, not TDEV or single-clock
   max.** With the now-faithful `run_two_clock` (§ below), the acceptance
   metric (both clocks get the lever; p95 over 6 independent pairs,
   settled) says:

   | lever (both clocks) | clkpoc3 p95 | piface-v1 p95 |
   |---|---|---|
   | baseline | 1.81 ns | 3.53 ns |
   | softgate | 1.56 ns (−14%) | 3.53 ns (inert) |
   | **softgate + Q[3,3]=0.003** | **1.45 ns (−20%)** | **3.23 ns (−9%)** |
   | Q[3,3]=0.003 only | 1.67 ns (−8%) | 3.23 ns (−8%) |
   | coast 8 s (dayplan lever #2) | 5.14 ns (**+184%**) | 8.40 ns (**+138%**) |

   `softgate + Q[3,3]=0.003` is the best lever on **both** hosts; coast is
   catastrophic on both. On piface softgate alone is inert (its OCXO gate
   at 0.54 ns is the active gate, so the internal soft-gate never engages)
   and the Q[3,3] term does the work. **Metric caveat:** an earlier pass
   reported stiffer Q[3,3] "blowing piface excursion to 21 ns" — that was
   *single-clock absolute* `max_excursion` (heavy-tailed, dominated by rare
   pull-in spikes), the wrong metric. The differential p95 *improves* and
   even the differential max (5.51 vs 5.77 ns) does not blow up. Stiffer Q
   can raise a high-drift host's *absolute* holdover excursion while
   improving the *differential* — grade the shared-antenna target on the
   differential.

6. **No lever reaches PASS (≤1 ns) alone.** The sim baseline is already
   1.8–3.5 ns p95; the best lever buys 10–20%, not the 2–3.5× needed. This
   matches `two-site-sync-budget.md` §3.2.1/§4.2: σ_servo_residual is ~7×
   over budget and needs *architecture* (soft-gate + slow innovation-mean
   integrator + per-host bias calibration), not tuning alone. The mid-τ
   levers are a real but partial improvement to be *stacked on* the §8.1
   gate-architecture work, not a substitute for it.

## Method

`scripts/peppar_fix/servo_sim.py` (`ClosedLoopSim`) drives the *real*
`DOFreqEst` closed-loop. Presets `clkpoc3` (clean-lock) and `piface-v1`
(gated, high-drift) are the two hosts with timestamper cals, so both
express their DO vs the same GNSSDO+ truth (acceptance now unblocked).
Protocol: duration 7000 s, **TDEV on t ≥ 600 s** (settled), 4–5 seed
average to beat down realization scatter. Hump metric = max TDEV over
τ∈{8,16,32,64}. Scratch harnesses: `repro_hump.py`, `isolate_hump.py`,
`skip_acq.py`, `lever_settled.py`, `confirm_xhost.py`.

## Injection-path isolation (why gain levers alone don't close it)

Single-arm settled runs localize the hump to the **DO-phase measurement
arms**:

| Arms fed | mid-τ hump | note |
|---|---|---|
| PPP only | none (91 ps @8 s, monotone) | receiver-clock arm is clean |
| PPP + qErr | none (184 ps) | same |
| TICC only | unstable (44 ns @32 s) | ns-jitter dominates, can't hold loop |
| EXTINT only | 1519 ps @16 s | 8 ns quantization |
| all four (baseline) | 394 ps @64 s (settled) | the operational config |

The hump enters through TICC/EXTINT (the DO observers), which carry the rx
sawtooth (qerr) + PPS jitter. But it **persists with zero PPS jitter and
rx_f0=0** — so it is not purely the sawtooth; it is the loop's realization
of the residual structured noise on those arms. Uniform gain levers (flat
R-inflation, Q) trade τ=1 s against mid-τ but don't remove the structured
mid-τ residual; `soft_ticc_gate` helps *because* it is selective
(per-epoch χ²-weighted), not uniform.

## Faithful `run_two_clock` (LANDED this arc)

The acceptance metric is the **two-clock p95 excursion**, and findings 5–6
show single-clock TDEV / max are not sufficient proxies. `run_two_clock`
was v1/non-faithful (`share_gnss=True` reseeded B with A's *entire*
realization, DO noise included → collapsed the differential toward zero,
and desynced when arm configs drew a different number of randoms).

The docstring's proposed "ONE shared rx/GNSS realization + independent
per-DO noise" is itself **contradicted by `two-site-sync-budget.md` §2**:
on a shared antenna the rx TCXO, DO free-running noise, *and* disciplining-
loop noise are all in the *independent* column — only sky-side terms
(orbit/clock, iono, tropo, multipath) cancel, and the sim injects no
sky-side term (`phi_rx` *is* the per-host rx TCXO). So the faithful
shared-antenna model is **two fully independent sims with independent
seeds**, giving σ²_Δ ≈ σ²_clock,A + σ²_clock,B (= the budget's 2σ²_clock).
There is no correlated stream, so the whole v1 desync class is gone.

Landed: independent-seed `run_two_clock` + `two_clock_excursion_stats`
(settled p50/p95/max/rms) + tests (independence, √2 scaling, deprecation
warning). CLI `scripts/servo_sim.py --two-clock A B` now prints the real
verdict — two clkpoc3 clocks p95 = 1.88 ns FAIL, same order + FAIL as the
lab §4.4 result. This is the go/no-go gate the lab A/B needs.

## Recommendation

- Carry **`soft_ticc_gate` (#217)** forward as the lead lever for
  TICC-primary hosts (clkpoc3-class); pair it with a *modest* stiffer
  Q[3,3] (≈0.003), not coast.
- **Do not** pursue coast / flywheel-trust as a mid-τ hump fix — it is a
  short-τ lever and worsens the hump.
- Treat piface-class (gated, high-drift, EXTINT-carried) separately — its
  hump is resolution-limited, not sawtooth-injection; the fix there is a
  cleaner mid-τ DO-phase observer (TDCP), not gate/Q tuning.
- Build the faithful two-clock harness before the lab A/B so the gate is
  the real acceptance metric.
