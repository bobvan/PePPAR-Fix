# Single-Frequency Float-PPP Extension

*Design for I-182352-charlie, started 2026-05-02 afternoon after the
gap-fill twin-WHU run on TimeHat + MadHat surfaced TimeHat's inert
BDS B1I observations (TIM 2.20 firmware doesn't track BDS B2a-I
dual-freq; without an IF combination, the engine drops the
observations).*

## TL;DR

Extend the float-OK / AR-not-OK framing from
`docs/ac-datum-mixing.md` one axis: **AR-eligible iff signal has a
dual-frequency IF combination available.**  Single-frequency
observations contribute to float-PPP geometry via Klobuchar-iono
correction; they never reach LAMBDA because MW requires dual-freq
by construction.

The PPPFilter currently drops single-freq observations at the
realtime_ppp.py obs-build step.  This proposal threads single-freq
observations through to a new PPPFilter update path that adds a
Klobuchar-modelled iono-delay term to the predicted range.

## Why

Concrete benefits today:

  - **TimeHat BDS B1I**: TIM 2.20 firmware ACK's
    `CFG_SIGNAL_BDS_B2A_ENA` but doesn't actually output B2a-I
    observations.  Confirmed empirical 2026-05-02 (`dual=N
    f2=?:0.0`).  With this extension, BDS B1I single-freq
    becomes geometry contribution; without it, the observations
    are inert.
  - **F9P GLO**: F9P tracks GLONASS L1+L2 dual-freq, but CNES
    has no GLO O/C and WHU's GLO biases land in WHU's datum
    (gap-fill list catches this).  Each GLO SV's ambiguity is
    structurally float-only.  Currently the engine still tries
    to advance these to NL and LAMBDA fails the ratio test.
    With this extension, GLO SVs are explicitly float-only; no
    LAMBDA attempts.
  - **Future-firmware quirks**: the F9T/F9P family sometimes
    drops a band on cold-start until reconfigured.  Graceful
    degradation rather than fix-set holes.

## Architecture

The existing engine pipeline produces, per SV per epoch:

```
Dual-freq path:
  realtime_ppp builds obs = {sv, sys, pr_if, phi_if_m, ...}
                         ^   ^      ^ ^
                         |   |      | └ wl_f1·cp_f1·a1 - wl_f2·cp_f2·a2 (m)
                         |   |      └ a1·pr_f1 - a2·pr_f2 (m)
                         |   └ system prefix
                         └ sat name (e.g. G05)
  PPPFilter.update(obs) consumes pr_if + phi_if_m, forms innovation
  MW tracker forms wide-lane combination from cp_f1, cp_f2
  NL resolver runs LAMBDA on dual-freq ambiguities
```

The single-freq path adds:

```
Single-freq path:
  realtime_ppp builds obs = {sv, sys, pr_l1, phi_l1_m, freq_hz,
                              single_freq: True, signal_code, ...}
  PPPFilter.update_singlefreq(obs):
    iono = klobuchar.slant_iono_delay_m(freq_hz, ...)
    pr_pred  = ρ + clk + ISB + tropo + ZTD·m_wet + iono
    phi_pred = ρ + clk + ISB + tropo + ZTD·m_wet - iono + λ·N
                                                  ^^^^
                                  iono cancels for IF; doesn't here
    R_pr  = (σ_pr_singlefreq)²  ≈ (3 m)²   ← inflated for iono residual
    R_phi = (σ_phi_singlefreq)² ≈ (3 m)²   ← same: phase carries iono too
  MW tracker: SKIP single-freq SVs (no f2; can't form WL)
  NL resolver: SKIP single-freq SVs (never reach CONVERGING)
```

Note the phase observation: in IF, iono cancels because the L1
and L5 phases each pick up `+I` and the IF coefficients cancel
them.  For single-freq L1, the phase carries the FULL ionospheric
delay with the OPPOSITE sign from PR (since iono advances phase
and delays code).  So `phi_pred` subtracts iono where `pr_pred`
adds it.

## Ambiguity state for single-freq SVs

Two design choices:

**Option A — single ambiguity slot per single-freq SV.** Same as
existing dual-freq SVs but the ambiguity absorbs Klobuchar
residual + receiver-side L1 bias.  Won't reach integer (residual
is non-integer m-scale); the MW tracker and NL resolver are
configured to skip these SVs entirely.

**Option B — pseudorange-only single-freq path, no ambiguity slot.**
Skip carrier-phase observations entirely for single-freq SVs.
Easier to plumb but loses ~half the geometry contribution
(carrier phase is the much more precise observable).

**Recommendation: Option A.**  Carrier phase is what makes PPP
better than SPP; dropping it is half the value.  The ambiguity
slot is cheap (already exists structurally as `sv_to_idx[sv]`);
it just needs to be marked as "float-only, never AR-eligible."

## Per-SV ar_eligible flag

Existing fix-set lifecycle (per `docs/sv-lifecycle-and-pfr-split.md`):

```
TRACKING → FLOATING → CONVERGING → ANCHORING → ANCHORED
                  ^         ^           ^
            (admit)   (MW converged)   (NL fix)
```

For single-freq SVs, the path stops at FLOATING.  No MW (requires
dual-freq), so no CONVERGING.  `SvAmbState` natively handles this
— the engine just never advances the SV past FLOATING.  No new
flag needed.

But for completeness, the cohort-comparing monitors
(`FixSetIntegrityMonitor`, `SettingSvDropMonitor`,
`AnchoringSvPromoter`) need to know the SV is in the float pool,
not the AR pool.  Today these monitors look at `SvAmbState` —
SVs in FLOATING aren't candidates for ANCHORING promotion or
fix-set membership.  So this is implicit.

The only place we need an explicit flag is the **iono-source
attribution** in logs and per-SV diagnostics.  For SVs corrected
via Klobuchar, the bias source attribution should make the
single-freq distinction visible (e.g., `iono_src=Klobuchar` vs
`iono_src=IF` in `[CB_APPLIED]/[PB_APPLIED]` logs).  This is a
nice-to-have, not load-bearing.

## Observation noise

For single-freq, the observation-noise floor is dominated by
Klobuchar's residual (~1–3 m RMS depending on solar activity)
rather than receiver thermal noise (~0.3 m for PR, ~0.003 m for
phase).

Two parameter choices:

  - `SIGMA_PR_SINGLEFREQ`  ≈ 3 m  (vs `SIGMA_P_IF` ~0.3 m)
  - `SIGMA_PHI_SINGLEFREQ` ≈ 3 m  (vs `SIGMA_PHI_IF` ~0.003 m)

The phase noise is dominated by Klobuchar residual when ranging
single-freq.  Dropping σ_phi to receiver-thermal-only would make
the float ambiguity converge to a wrong (Klobuchar-biased)
integer that LAMBDA would then accept; we don't want that.
Keeping σ_phi inflated keeps the ambiguity float-only naturally.

## Frequency-aware bias lookup

The existing CB_APPLIED / PB_APPLIED path looks up biases by
RINEX signal code (e.g., `C1C`, `L1C`, `C5Q`, `L5Q`).  For
single-freq, we need:

  - The signal code we're using (e.g., `C2I` and `L2I` for BDS B1I)
  - The frequency in Hz for Klobuchar scaling
  - The bias source mount (already plumbed via `src_mount`)

Frequency-Hz can be derived from the RINEX signal code via a
lookup table — already exists in `solve_ppp.py` for the IF
combination math.

## Implementation plan

```
Stack:
  (1) ✓ Klobuchar module (docs/single-freq-float-ppp.md spec)
                          scripts/klobuchar.py (commit landed)
  (2)   realtime_ppp.py: build single_freq obs when f2 missing
        — add 'single_freq' branch in the per-SV obs-build loop
        — populate {sv, sys, pr_l1, phi_l1_m, freq_hz,
                     signal_code, src_mount, single_freq=True}
        ~80 LOC
  (3)   solve_ppp.py: PPPFilter.update single-freq branch
        — detect obs['single_freq'] flag
        — compute Klobuchar iono via scripts/klobuchar.py
        — innovation: dz_pr = pr_l1 - (rho_pred + iono)
        — innovation: dz_phi = phi_l1_m - (rho_pred - iono + λ·N)
        — H rows analogous to IF, no bias-state contribution
        ~100 LOC
  (4)   sv_state / mw_tracker / nl_resolver: skip single-freq SVs
        — MW tracker: in observation-ingest, check obs['single_freq']
                      and skip the WL combination
        — NL resolver: candidate filter checks SvAmbState; SVs
                       stuck at FLOATING aren't candidates anyway
        Nothing structural; just ensuring no MW/NL crash on
        missing f2 fields.  ~30 LOC
  (5)   Tests:
        - PPPFilter.update with mixed dual+single observations
        - Single-freq SV stays at FLOATING
        - σ improvement metric vs PR-only baseline
        ~80 LOC
  (6)   Lab smoke:
        - TimeHat re-enable BDS in --systems
        - Verify BDS B1I SVs reach FLOATING (not stuck at TRACKING)
        - σ improvement vs BDS-off baseline (shipped today, ac7c595)
        - MadHat dual-freq AR pipeline metrics unchanged
```

## Acceptance criteria

  1. PPPFilter.update tolerates a mix of dual-freq + single-freq
     observations in the same epoch (no crash).
  2. Single-freq SVs reach `SvAmbState.FLOATING`; do NOT advance
     to `CONVERGING` or `ANCHORING`.
  3. NL resolver never attempts LAMBDA on single-freq SVs (verified
     via NL_DIAG log output: 0 single-freq candidates).
  4. TimeHat with BDS re-enabled measurably improves σ compared
     to the BDS-off baseline (ac7c595, captured at
     `TimeHat:~/peppar-fix/data/day0502-twin-whu-gapfill-bds-off-timehat.log`).
     Magnitude: low (BDS B1I single-freq adds modest geometry);
     directional improvement is the test.
  5. MadHat metrics unchanged from gap-fill baseline (no
     regression on dual-freq AR pipeline).

## What this doesn't do

  - **Doesn't add ionosphere-aware AR.**  LAMBDA still requires
    dual-freq.  Future work to use ionosphere-fixing AR (TEC-
    constrained integer search) would build on this scaffold.
  - **Doesn't handle ionosphere storms.**  Klobuchar misses ~50%
    of iono variation; storm-time residual is ~10 m or worse.
    The σ_PR_SINGLEFREQ = 3 m floor doesn't capture storm
    conditions; integrity-trip path (existing) catches the
    resulting position drift.
  - **Doesn't enable single-freq receivers without phase.**  This
    is for receivers that have phase + PR on a single band; pure
    code-tracking GNSS receivers (e.g., legacy NAVSPARK boards)
    aren't in scope.

## References

  - ICD-GPS-200N §20.3.3.5.2.5 (Klobuchar algorithm definition)
  - `docs/ac-datum-mixing.md` (architectural framing — float-OK
    / AR-not-OK applies here too)
  - `docs/sv-lifecycle-and-pfr-split.md` (SvAmbState lifecycle —
    single-freq SVs stay at FLOATING by construction)
  - `docs/f9t-firmware-capabilities.md` (TIM 2.20 BDS B2a-I
    behaviour: ACK's enable, doesn't track — empirical
    2026-05-02)
  - I-182352-charlie dayplan proposal (this work)
