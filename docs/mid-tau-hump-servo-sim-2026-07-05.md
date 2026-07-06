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

## Update 2026-07-05 — re-grounded to CURRENT hardware (supersedes the stale-preset grade)

The grade above was on the pre-London presets. A parallel PiFace+PiPuss
lab capture (`gt:~/gt/mtbaseline-20260705/`, `project_mtbaseline_current_hw`)
shows those presets misdescribe the current hardware, and **re-grounding
flips the lever**:

- **The hump is real on current hardware** — PiFace settled chA TDEV humps
  **484 ps @ τ=32 s** (52 ps @ 1 s clean; 3.2× the 354 ps budget). PiPuss
  is 4050 ps @ 16 s, dominated by a chA/DO-PPS edge-jitter fault
  (I-105924), not servo dynamics.
- **Mechanism changed:** current hosts show **0 % OCXO-gate rejection**
  (vs `piface-v1`'s 99.6 % lockout). So `soft_ticc_gate` (#217) targets a
  pathology that isn't occurring — it is **moot** on current PiFace. The
  live hump is an open-gate loop resonance: the IsoTemp OCXO's frequency
  RWFM (DO char `coast_tdev_ref`=0.070 ns@1s, slope 1.118) showing through
  above loop BW (shape = bump-and-*recover*, not the monotone rise DO-noise
  alone would give).
- **New preset `piface-current`** encodes the measured params (do_f0=+2.30,
  no gate, real Q σ_do_phase=0.0564 / σ_do_freq=0.009, routed-qErr, DAC LSB
  0.026). **Amplitude caveat:** the sim reproduces the hump *shape* but
  over-produces its *amplitude* ~6× (the DO mid-τ freerun the coast char
  extrapolates to is unpinned) — use for **relative** lever direction only;
  grade absolute numbers by lab A/B.
- **Re-grade verdict (relative, on `piface-current`):**

  | lever | hump(16–64 s) | two-clock p95 | vs base |
  |---|---|---|---|
  | baseline (σ_do_freq=0.009) | 3071 ps | 12.6 ns | — |
  | **looser Q[3,3] σ_do_freq→0.03** | **1464 ps** | **6.24 ns** | **−52 %** |
  | stiffer Q[3,3] →0.003 | 5532 ps | 23.9 ns | +80 % |
  | coast 8 s | 5365 ps | 21.3 ns | +75 % |
  | Q[2,2] (phase) ±5× | ~3050 ps | ~12.7 ns | ±4 % (moot) |

  **The lever is *looser* Q[3,3] (σ_do_freq), not stiffer** — it lets the
  freq loop track the OCXO's RWFM above BW instead of trusting a stale
  flywheel estimate. This **reverses** the stale-preset grade (which said
  *stiffer* Q[3,3] + softgate) and the dayplan's "stiffer-Q/coast" lever
  #2. Grading on stale presets would have sent the lab A/B the wrong way.

## Recommendation (current)

- **Lab A/B on PiFace** (only — PiPuss blocked by I-105924): interleaved,
  reset-free, `σ_do_freq` = 0.009 (current) vs ~0.03 (looser), grade on the
  real chA TDEV(32 s) + two-clock p95. The sim gives the *direction*
  (looser Q[3,3]); the lab gives the *magnitude*.
- **Do not** pursue coast/flywheel-trust or softgate for the current PiFace
  hump — coast worsens it, softgate is moot at 0 % gate reject.
- The pre-London `soft_ticc_gate`/stiffer-Q grade applies only to a
  gate-lockout regime that current hardware is not in; keep it filed for
  hosts that re-enter that regime, not as the live lever.

## Lab A/B result (2026-07-05 evening) — the sim lever is REFUTED

Ran the A/B on PiFace: A = σ_do_freq 0.009 (default, 3 h reset-free) vs
B = σ_do_freq 0.03 (`--sigma-do-freq-override`, 50 min), warm-started,
settled chA TDEV:

| τ (s) | 1 | 8 | 16 | **32** | 64 | 128 |
|---|---|---|---|---|---|---|
| A (0.009) | 57 | 167 | 361 | **454** | 420 | 318 |
| B (0.030) | 44 | 193 | 393 | **450** | 391 | 493 |
| B/A | −23% | +16% | +9% | **−1%** | −7% | +55%¹ |

¹ low-confidence (B is 50 min). **At the τ=32 s hump: −1 %. The sim's
−52 % prediction did not transfer.** Looser Q[3,3] gives only a modest
short-τ win (τ=1 s −23 %) and trends worse at long τ; it does **not** move
the hump.

**Conclusion: the real 484 ps @ 32 s hump is insensitive to Q[3,3] over a
3.3× change — its mechanism is NOT the freq-loop-vs-OCXO-RWFM the
re-grounded sim modeled.** The sim mispredicted the lever in *both*
regimes (stale → stiffer-Q/softgate; re-grounded → looser-Q; lab → neither
moves the hump). It reproduces the hump's *shape* but not the physics that
sets its amplitude or sensitivity, so **servo_sim is not a reliable
predictor for this hump even directionally** — the mid-τ lever search must
be **hardware-driven**. The harness/re-grounding tooling in this PR remains
valid; only the sim's *lever recommendation* is refuted.

Next hardware candidates (Q[3,3]-insensitivity points away from the freq
loop): loop **bandwidth** via actuation cadence, Q[2,2]/measurement
weighting, or the hump as a fixed loop resonance set by the 1 Hz cadence +
gain. Data: `gt:~/gt/mtbaseline-20260705/{piface-qfreq-A-c1,piface-qfreq-B}`.

## Cadence A/B (2026-07-06) — the hump IS loop-bandwidth-limited (fix found)

PiFace, σ held at 0.009, cadence the only variable: FAST = A-c1 (1 Hz,
3 h) vs SLOW = coast every 8 s (`--min-interval 8 --max-interval 8`, 44 min,
reset-free, verified dt_actual ≈ 8 s once converged):

| τ (s) | 1 | 16 | **32** | 64 | 128 |
|---|---|---|---|---|---|
| FAST 1 Hz | 57 | 361 | **454** | 420 | 318 |
| SLOW 8 s | 52 | 387 | 688 | **978** | 893 |
| S/F | −9% | +7% | +51% | +133% | +181% |

**FAST peaks 454 ps @ 32 s; SLOW peaks 978 ps @ 64 s** — coasting *grew*
the hump ~2× and *pushed it to longer τ*. So the mid-τ hump **is a
loop-dynamics feature set by the actuation cadence / loop bandwidth**, not a
fixed DO/measurement floor. This **reconciles the Q[3,3] null**: the hump is
bandwidth-limited by the **1 Hz cadence**, not the Kalman gain — Q[3,3] over
3.3× did nothing, cadence over 8× moved it decisively.

**Fix direction: *faster* actuation / higher loop BW** (slower worsened it +
lengthened τ, so faster shrinks it and pushes it below the τ where DO noise
bites). 1 Hz measurement is the bottleneck → next experiment: a **faster
measurement rate** (`--measurement-rate-ms 500` = 2 Hz, if the F9T-20B
supports it) or **TDCP at the full RAWX rate** (`two-site-sync-budget.md`
§6.3). Sim, physics, and lab all agree here — unlike the Q[3,3] lever.
Data: `gt:~/gt/mtbaseline-20260705/piface-cadence-slow8`.
