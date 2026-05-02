# AC Datum Mixing — When SSR Sources Can Combine

*2026-05-02 — written after the I-122350-main P3 twin-WHU run on
TimeHat + MadHat surfaced the dual-mount merge bug.  The smoking
gun was a 0.93 m disagreement in the GAL E1C phase bias applied
for SV E10 between the two hosts on a shared antenna, traced to
last-write-wins routing flipping between CNES and WHU values.*

## TL;DR

Phase biases from different ACs are **calibrations in different
datums, not noise around a common truth**.  Mixing them is OK for
float-PPP and forbidden for AR.  Concretely:

  - **For each (SV, signal) ambiguity**, AR-fixable iff the bias
    source is in the same datum as the orbit/clock source.
  - Cross-datum bias is fine for float-PPP because the real-valued
    float ambiguity absorbs the persistent ~0.1–1.5 m datum offset.
  - Cross-datum bias on AR-attempted signals breaks LAMBDA: the
    closest integer is wrong, the correct integer isn't anywhere
    in the search space, and the ratio test either fails or
    accepts a wrong fix.

For our lab today: **CNES SSRA00CNE0 is the AR-eligible primary
(orbit + clock + code bias + GPS L1/L2 + GAL E1/E5a phase biases).
WHU OSBC00WHU1 is gap-fill secondary (GPS L5Q + BDS B2a-I + GLO
phase + code biases that CNES leaves blank).**

## Why phase biases are not "the bias"

A satellite phase bias `b_sat(SV, signal)` is the satellite-side
fractional cycle that must be subtracted from the observed phase
so the residual decomposes cleanly into (range + clock +
atmosphere + integer cycles).  AC analysts derive it by holding a
network of reference receivers fixed and pinning some subset of
ambiguities to integer to define the bias scale.

The choice of **which ambiguities are pinned to define the scale**
is the AC's "datum."  Different ACs pin different subsets of
ambiguities, so they publish different numerical values for the
same `b_sat(SV, signal)`.  Both are correct in their own datum.

Empirically, for a given (SV, signal):

  - The **difference** between two ACs is approximately constant
    over hours-to-days.  See `docs/l5i-l5q-phase-bias-empirical.md`
    (mean −0.73 m, SD 1.46 m across 19 GPS SVs in a single arc
    for L5I−L5Q).
  - **Each AC's value alone** is internally consistent with that
    AC's orbit + clock + other phase biases.

So for a given SV E10 GAL E1C phase bias, expecting CNES and WHU
values to agree to mm scale is the wrong mental model.  ~1 m
persistent offset is the **expected** behaviour, not noise to be
averaged out.

## Mathematical consequence — why mixing breaks AR

The phase observation:

```
Φ_obs = ρ + c·δt_rx − c·δt_sat + T + I + λ·N + b_rx + b_sat^datum + ε
```

The receiver applies `b_sat_applied` (whatever the engine looked up
from the bias cache):

```
Φ_residual = Φ_obs − b_sat_applied
           = ρ + c·δt_rx − c·δt_sat + T + I + λ·N + b_rx
             + (b_sat^datum − b_sat_applied) + ε
```

Two regimes:

1. **`b_sat_applied = b_sat^datum`** (single AC, internally consistent).
   The residual datum offset is zero.  `λ·N` is a clean integer
   multiple of the wavelength (modulo the receiver-side `b_rx`
   absorbed elsewhere).  AR can find the integer.

2. **`b_sat_applied = b_sat^other_AC`** (mixing).  The residual
   datum offset is a non-zero, persistent ~0.1–1.5 m systematic.
   `N` absorbs that into a non-integer float value.  Float-PPP is
   fine — it just sees `λ·N + δ_AC` as the float estimate and
   carries on.  But LAMBDA's integer search is built around
   `λ·N` being an integer multiple of `λ`.  With the residual
   `δ_AC` mixed in, the closest integer is wrong and the correct
   integer is outside the search space.

The same argument applies to MW combinations, IF combinations,
and any downstream consumer that treats float ambiguities as
candidates for integer fixing.

## What "AR-eligible" means per (SV, signal)

For each (SV, signal) ambiguity in the filter:

  - **AR-eligible** = bias source is in the same datum as the
    orbit/clock source.  In our setup: CNES O/C + CNES bias.
  - **Float-only** = bias source is a different AC than O/C.  In
    our setup: CNES O/C + WHU bias on a GPS L5Q signal.

A single run can hold both classes simultaneously.  E.g., MadHat
on UFO1 with CNES + WHU dual-mount has:

  - GAL E1+E5a ambiguities: AR-eligible (CNES O/C + CNES bias).
  - GPS L1+L2 ambiguities: AR-eligible (CNES O/C + CNES bias).
  - GPS L5Q ambiguities: float-only (CNES O/C + WHU bias).
  - BDS B2a-I ambiguities: float-only (CNES O/C + WHU bias).
  - GLO L1+L2 ambiguities: float-only (CNES O/C + WHU bias).

The float-only signals contribute extra observations to the
position solution (more SVs in the IF combination, better
geometry) without poisoning the AR-eligible signals.

## Implementation — gap-fill via static allow-list

The simplest and lowest-risk implementation:

  - The primary mount writes to the bias cache **for any signal**.
  - The secondary mount writes **only for signals on the
    `GAP_FILL_SIGNALS` allow-list**.  All other writes are
    dropped at intake.

This makes the cache **single-source-per-(SV, signal) by
construction** — last-write-wins becomes a no-op because the
cache key collisions don't happen.  The merge bug from the P3
twin-WHU run becomes structurally impossible.

The allow-list is in `scripts/ssr_corrections.py`:

```python
GAP_FILL_SIGNALS = frozenset({
    # GPS L5 — CNES SSRA00CNE0 omits L5; WHU OSBC00WHU1 provides.
    ('G', 'C5Q'), ('G', 'L5Q'),
    ('G', 'C5X'), ('G', 'L5X'),
    # BDS B2a-I — CNES omits, WHU provides.
    ('C', 'C5X'), ('C', 'L5X'),
    ('C', 'C5P'), ('C', 'L5P'),
    # GLONASS — CNES does not publish GLO biases.
    ('R', 'C1C'), ('R', 'L1C'),
    ('R', 'C1P'), ('R', 'L1P'),
    ('R', 'C2C'), ('R', 'L2C'),
    ('R', 'C2P'), ('R', 'L2P'),
})
```

The `bias_only=True` path in `realtime_ppp.py` passes
`gap_fill_only=True` to `SSRState.update_from_rtcm`, which
filters `_parse_code_bias` and `_store_phase_bias` against
`GAP_FILL_SIGNALS`.

When the primary AC adds a signal to its product (rare; happens
on ~yearly cadence as ACs catch up to receiver firmware), the
corresponding tuple is removed from `GAP_FILL_SIGNALS`.  When the
secondary AC drops a signal, the corresponding tuple is also
removed (the gap-fill no longer fills).  This list is the **AC
coverage frontier**, not a config switch.

## What this design does NOT do

  - **Does not give AR fixes on gap signals.**  L5Q phase
    observations are added to the float-PPP solution but their
    ambiguities won't reach LAMBDA's ratio threshold because the
    bias is in WHU's datum and LAMBDA expects integer in CNES's.
    That's OK; partial-AR is supported (LAMBDA already handles
    the case where some SVs aren't candidates).
  - **Does not handle primary-AC outages.**  If CNES drops L1
    bias for SV G05 mid-arc, our code stops applying any L1 bias
    for G05 — there's no fallback to WHU because L1 is not in
    `GAP_FILL_SIGNALS`.  The G05 L1 ambiguity goes float and
    drifts on its uncorrected hardware bias.  Acceptable: CNES
    rapid product outages are <1% per month, and the alternative
    (auto-failover with hysteresis) introduces datum flips that
    cost more in re-convergence than the gap costs in occasional
    SV loss.
  - **Does not enable mid-arc AC switches.**  Switching between
    AC sources for the same (SV, signal) ambiguity invalidates
    the float ambiguity (datum just changed; the existing
    estimate is in the old datum).  Treated as a cycle slip.  We
    don't switch in normal operation; this is documented for
    correctness.

## Verifying via bias_diff overlay

The `scripts/overlay/bias_diff.py` overlay (commit 2603d71) cross-
diffs two hosts' [CB_APPLIED] / [PB_APPLIED] streams.  After the
gap-fill filter is in place, on shared antenna (twin-host setup):

  - **value mismatch on shared signals**: should be 0.  Anything
    > 0 is a routing bug.
  - **TimeHat-only / MadHat-only rows**: legitimate hardware
    asymmetry (TIM 2.20 doesn't track BDS-3 / L5).  Not a bug.
  - **src mismatch (same value, different src)**: usually 0.
    Non-zero means the two hosts ended up in different AC
    sources for the same (SV, signal), which shouldn't happen
    under the static gap-fill design — investigate.

A 5-min twin-host smoke run + bias_diff is the recommended
acceptance test for any future AC-mixing change (new AC, new
signal coverage, gap-fill list edit).

## References

- `docs/ssr-mount-survey.md` — survey of SSR mounts; introduces
  WHU OSBC00WHU1 as gap-fill candidate.
- `docs/ssr-cross-ac-diagnostic-2026-04-25.md` — earlier 2x2
  isolation testing that motivated the dual-mount design.
- `docs/l5i-l5q-phase-bias-empirical.md` — empirical proof that
  AC datum offsets are persistent, not noise.
- `dayplan I-122350-main` 2026-05-02 — the twin-WHU run that
  surfaced the merge bug and motivated this doc + the gap-fill
  filter.
