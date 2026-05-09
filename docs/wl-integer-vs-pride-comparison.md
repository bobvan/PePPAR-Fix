# Engine WL integers vs PRIDE float WL — bias diagnosis

**Issue:** I-155354 (systematic-bias investigation in float/WL position
solutions). Hypothesis #1: the streaming MW tracker fixes WL
ambiguities to wrong integers; PRIDE-PPP's batch lsq↔redig
produces tighter float estimates from the same RINEX, and the
engine's integer is off by multiple cycles for some SVs.

**TL;DR:** Confirmed. On MadHat F9T DOY 126 (2026-05-06), the
engine fixed E10 to 15 while PRIDE's float WL estimate was
+36.96 ± 0.06 cycles (round → 37) — engine wrong by **22 cycles
= 16.5 m** of WL geometric range error. E19: engine -89, PRIDE
float -34.24 ± 0.06 (round → -34) — **55 cycles wrong = 41.3 m**.
Non-confident PRIDE arcs (where σ > 0.25 or float-residual > 0.25)
show engine integers tens of cycles off PRIDE's float for E04,
E11, E12, E33 too — the tracker is structurally unstable, not
just occasionally wrong.

This explains the I-155354 motivating observation: WL-only
AntPosEst converges to a basin 6–8 m off NAV2 with ZTD residual
+1146 mm. The 6–8 m position bias and the ZTD-absorbing-PR-residual
~1.1 m are both downstream of these multi-meter WL-integer errors.

## Method

The engine's MW tracker emits `[WL_FIX_LIFE] event=enter sv=Gxx
n_wl=N elev=E ...` log lines whenever an SV transitions into
WL-fixed state. The `int_history` field records the running list
of WL integer fixes the tracker has produced for that SV — this
should be stable if the WL is correctly fixed.

PRIDE-PPP's `amb_<DOY>_<site>` output contains per-arc float WL
ambiguities (`WLamb`, in cycles) with sample standard deviation
(`SigWL`). When σ_WL is small (< 0.25 cycles) and the float is
close to integer (residual < 0.25 cycles), PRIDE's float pinpoints
the integer unambiguously — and the engine's integer should match.

Comparison: `scripts/replay/wl_vs_pride.py`
- Parses both inputs
- Matches per-(SV, arc-window): finds the engine's `n_wl` that was
  active during the PRIDE arc (most recent enter before arc-end
  with no exit before arc-start)
- Computes Δcycles and position-bias contribution (Δcycles × λ_WL)

WL wavelength: 75.1 cm (GAL E1/E5a), 86.2 cm (GPS L1/L2),
102.5 cm (BDS B1I/B3I).

## Inputs

- **RINEX**: `~/peppar-fix/data/rinex/MadHat-2026126.obs` (24h
  capture, 30 s decimation, RINEX 3.04, F9T-20B at TIM 2.25 on
  CHOKE1 antenna)
- **Engine log**: `day0506pm-madhat-arm14.log` from MadHat —
  same engine instance that wrote the RINEX
- **PRIDE output**:
  `/home/bob/pride-choke1-doy126/A-madhat-f9t/2026/126/amb_2026126_ufo1`
  — pre-existing PRIDE run on the same RINEX (15-min window
  21:22-21:37 UTC). AR fixing was disabled in this run; the WL
  float column is what we compare against.

A 24h PRIDE-PPP run on the same RINEX with AR enabled would
strengthen the diagnosis (more SVs, more arcs, integer-fix
confidence). Filed as a follow-up below.

## Result

```
 sv          arc           pride_wl   σ_WL   int   eng_n  Δcyc   bias_m   elev  conf
 E04   21:22:21-21:37:51  +27.2602  0.153   +27     31    -      -        33.0  ???
 E10   21:22:21-21:37:51  +36.9569  0.062   +37     15  -22   -16.53      62.3  PRIDE
 E11   21:22:21-21:36:51  +23.0016  0.393   +23     29    -      -        35.4  ???
 E12   21:22:21-21:37:51   +7.6640  0.050    +8     76    -      -        75.1  ???
 E19   21:22:21-21:37:51  -34.2387  0.063   -34    -89  -55   -41.33      58.3  PRIDE
 E33   21:22:21-21:36:51  -22.6394  0.100   -23     14    -      -        29.1  ???
```

`conf=PRIDE` when PRIDE's float is within 0.25 cycles of an
integer AND σ_WL < 0.25 — both conditions for a confident integer
anchor. Two of six arcs cleared this bar (E10, E19); both engine
integers are wildly wrong.

The other four arcs (`???`) had PRIDE float estimates that were
not themselves near-integer (e.g., E12 = +7.66 with σ 0.05 — a
near-half-cycle residual that's consistent with a real WL
ambiguity *between* integers, suggesting an unfixed-by-PRIDE
condition rather than noise). Even there, the engine's integer
diverges by tens of cycles from PRIDE's float — engine n_wl=76
vs PRIDE float +7.66. **The engine isn't producing integers in
the same ballpark as PRIDE's LSQ-derived floats**, which is the
load-bearing diagnosis.

## Worked example: E10

Engine `int_history` for E10 (excerpted from the log around the
arc window):

```
21:20:40  enter n_wl=28   int_history=[37, 44, 35, 28]
21:22:00  exit
21:23:57  enter n_wl=15   int_history=[44, 35, 28, 15]
```

E10's tracker held a 37 at one point (matching PRIDE's float
+36.96 → integer 37). It then drifted: 44 → 35 → 28 → 15. PRIDE
float σ_WL = 0.06 cycles — true integer is 37 with no
ambiguity. The MW tracker has lost its integer lock and is
producing values 9 to 22 cycles off the true integer.

WL geometric error per cycle = 75.1 cm (E5a band).
- 22 cycles = **16.5 m**
- 9 cycles = 6.8 m

If even half of the engine's "fixed" WL integers are similarly
displaced, the LSQ position solution lands in a basin
multiple meters from the surveyed truth — exactly the I-155354
motivation.

## Cycle-slip audit (2026-05-09 follow-up)

A second audit (`scripts/replay/slip_vs_wl_audit.py`) on the same
day0506 MadHat log reveals two layered failures, both pointing at
PR-noise contamination of the MW formula.

### Audit (1): the slip detector fires conf=LOW on mw_jump alone

```
reason                                count  conf=HIGH  conf=LOW  carrier?
mw_jump                                1275          0      1275  no
gf_jump,ubx_locktime_drop               595        595         0  YES
arc_gap                                 276          0       276  YES
arc_gap,mw_jump                         163        163         0  YES
gf_jump                                 143          0       143  YES
gf_jump,mw_jump                          87         87         0  YES
gf_jump,mw_jump,ubx_locktime_drop        66         66         0  YES
ubx_locktime_drop                        64          0        64  YES
```

**47.8 % of slip events (1275/2669) are mw_jump-only**, no
carrier-phase corroboration.  These are flagged conf=LOW by the
detector — but the engine still calls `flush_sv_phase()` for
every slip regardless of confidence
(`peppar_fix_engine.py:2696`), which calls
`mw_tracker.reset(sv)` and wipes the WL fix.

The mw_jump signal includes pseudorange (Melbourne-Wübbena
combines code and phase).  PR noise spikes shift the MW
running mean; a few-meter PR noise event maps to several
cycles of MW jump even when the underlying carrier phase
hasn't moved.  These are PR-noise false positives, not real
slips.  Charlie's I-194752 work already established that
MW-residual rolling-mean is uncorrelated with real slips
(Z = -0.17, p = 0.86) — exactly this concern, scaled up.

### Audit (2): even CARRIER-confirmed slips produce wrong-integer re-fixes

```
=== Post-slip WL re-fix outcomes (window 300s) ===
category     no_refix   same_int   DIFF_int  notes
CARRIER           524          3        712  |Δcyc| median=38  max=276
mw-only           285         12        934  |Δcyc| median=30  max=298
```

Of slips with a post-slip re-fix on the same SV:
- CARRIER-confirmed: **712/715 = 99.6 %** re-fix to a DIFFERENT
  integer; median |Δcyc| = 38 cycles ≈ 28 m
- mw-only:           **934/946 = 98.7 %** re-fix to a DIFFERENT
  integer; median |Δcyc| = 30 cycles ≈ 22 m

This is the deeper finding: **the MW tracker's re-fix process is
structurally unable to anchor to consistent integers** — even
across legitimate carrier-phase slips that reset only the slip-
of-cycles count, the new integer differs by tens of cycles.  A
real cycle slip changes the carrier-phase ambiguity by O(1) cycle,
not O(30).

### Per-SV drift extent (top 10 by mw-only slip count)

```
sv     carrier_slips  mw_only_slips  fixes   min_n_wl  max_n_wl   range
C33               28             55     46      -233       +33     266
C38               24             47     45      -100      +270     370
E36               26             46     37      -106      +120     226
G20              128             37     16       -88      +191     279
C36               16             35     26      -143       +71     214
C31               15             35     25      -183       +74     257
E11               14             34     32       -61      +102     163
C34               20             34     37       -98       +82     180
E29               25             32     20       -94      +100     194
C23                8             32     25      -135       +82     217
```

Several SVs show 200+ cycle ranges in their fix history — orders
of magnitude beyond what real cycle slips could explain.  The MW
re-fix lands in a different basin almost every time.

## Diagnosis (corrected from initial mechanism hypothesis)

My initial hypothesis was "the cycle-slip detector silently misses
slips, the tracker absorbs them by re-fixing."  Audit shows the
opposite: the detector fires aggressively (often falsely on
mw_jump alone), AND even the legitimate-slip re-fixes land on
wrong integers.

The MW (Melbourne-Wübbena) tracker computes
`MW = (φ1·λ1 - φ2·λ2)·(f1+f2)/(f1-f2) − (P1·f1 + P2·f2)/(f1+f2)`
which is geometry-free and ionosphere-free, but **NOT
pseudorange-free**.  Expected value: `λ_WL × N_WL + sub-cycle
bias`.  The PR contribution `(P1·f1 + P2·f2)/(f1+f2)` is the
narrow-lane (NL) of pseudorange, which inherits PR-side noise:
~30 cm code noise floor, multipath spikes of meters, signal-
code mismatch biases.

Two-layer failure mode:

1. **PR-noise contamination of MW signal**: meter-scale PR
   noise events shift MW by several λ_WL.  The slip detector
   sees these as `mw_jump` and flushes the tracker —
   conf=LOW, carrier-not-confirmed, mostly false positives.
   47.8 % of slip events fall in this category.

2. **Post-slip re-fix uses fresh MW samples that still carry
   the same PR-noise bias** that triggered the false positive.
   The new running mean settles to a DIFFERENT integer with
   ~99 % probability (median 30-38 cycles different).  Multiple
   consecutive false-positive flushes drift the integer
   monotonically — the per-SV `range` column above shows
   200+ cycles of drift on several SVs.

Charlie's I-194752 GF_STEP detector replaced MW-residual-based
slip detection because Z=-0.17 showed MW-residual is uncorrelated
with real slips.  This audit confirms the corollary: **the same
PR-noise contamination that makes MW-residual a bad slip detector
also makes MW a bad WL ambiguity estimator**.  The WL integer
isn't actually being constrained by carrier-phase observations
in the streaming path — it's being driven by PR noise plus a
rounding step.

## Next steps

In order of expected leverage:

1. **Gate `flush_sv_phase()` on slip confidence.**  Treat
   conf=LOW (mw_jump-only) slip events as warnings, not
   ambiguity-resetting events.  ~5 LOC change at
   `peppar_fix_engine.py:2696` to skip `mw_tracker.reset()`
   when `ev.confidence == 'LOW'`.  Eliminates the 1275
   false-positive flushes per day.  Doesn't fix layer 2 (the
   re-fix on real slips is still wrong) but stops the bleeding
   from layer 1.

2. **Rebuild WL re-fix to anchor against PRIDE-grade ambiguity
   resolution**, not the streaming MW running-mean.  Options:
   - LAMBDA over a multi-SV ambiguity vector (orders of
     magnitude tighter than per-SV rounding)
   - Wait-and-PRIDE: defer streaming WL fix until PRIDE batch
     produces an integer for the arc, then anchor the streaming
     tracker to PRIDE's value.  Loses streaming-low-latency
     property; gains correctness.
   - Drop streaming WL entirely (the I-125649 `--ar-mode none`
     path).  Position filter goes float-only; ARP-motion
     detection runs on float-σ basis.  Float σ ≈ 1-3 m which
     is still inside the 1-m budget Bob wants for ARP detection.

3. **Run PRIDE 24-h with AR enabled** on MadHat-2026126.obs
   for comprehensive per-SV validation across the full SV
   cohort.  Quantitative bound on per-SV bias magnitudes
   across diurnal cycle.  ~few-hour wall time.

4. **Bracket against Charlie's hypothesis #2** (L5 phase-bias
   mismatch).  If L5-band SVs have higher mw-only slip rates
   or larger drift ranges than other bands, phase-bias is
   contributing.  If similar across bands, the PR-noise-
   contamination story is the primary mechanism.

(1) is the cheapest immediate fix; (3) goes from "WL is broken"
to "drop WL streaming entirely" depending on the leverage from
(1).  Recommend landing (1) in parallel with the ar-mode wl
deployment (I-125649).

## Post-fix audit (2026-05-09 bravotest, fix-I + fix-H)

After fix-I (PR-only-slip flush gate) and fix-H (Hatch-smoothed
PR feeding MW) landed and Main test-merged into the I-125649
ar-mode-wl branch, MadHat ran a 2.5 h cold-start
(`day0509-madhat-bravotest.log`, 12:56-15:23 UTC, --ar-mode wl,
--position-blend-source antposest, --systems gps,gal).  Main's
verdict: "necessary, but not sufficient."  This section
quantifies the residual.

### What the fixes did

| metric                                | pre-fix (day0506) | post-fix (day0509)         |
|---------------------------------------|-------------------|----------------------------|
| mw_jump-only events (Layer 1)         | 1275 / 2669 (47.8%) | 11689 / 12203 (95.8%)    |
| mw_jump-only events that flushed WL   | 1275              | ~414 (96.5% gated)         |
| AntPosEst-vs-truth basin              | 6-8 m             | 1.5-4 m                    |
| AntPosEst σ_pos (converged)           | 0.27-0.50 m       | 0.147 m                    |
| `[FIXEDPOS_ZTD]` (time filter)        | n/a               | -157 ± 35 mm (clean)       |
| `[AntPosEst]` ZTD                     | +1146 mm trapped  | -1770 to +6205 mm (volatile) |
| Carrier-confirmed slip wrong-re-fix   | 712/715 (99.6%)   | 139/143 (97.2%)            |

**Layer 1 (Fix-I):** working as designed.  11283/11664 = 96.7% of
mw_jump-only slips no longer trigger flush.  The remaining ~414
that DID flush are events where the slip detector's reason set
also included a corroborator within the same epoch — those are
no longer "PR-only" by `is_pr_only_slip()`.  Empirically Main
counted 47 cycle-slip flush events in 30 min, ZERO with reason
`mw_jump` alone.

**Layer 2 (Fix-H):** working but small effect on this dataset.
Carrier-confirmed slip wrong-re-fix dropped from 99.6% → 97.2%.
Per-SV WL drift ranges across multiple fix events still span
50-157 cycles (G13 121 cyc, E10 102 cyc, E31 157 cyc).  The
Hatch smoother is firing — but on this site, 62% of WL fixes
happen with `n_epochs ≤ 90` (60 = `min_epochs` floor; smoother's
α = 1/60 ≈ 1.7% leaves substantial PR noise).  The smoother
needs longer arcs to deliver its full noise-floor reduction.

### What's still wrong

The two ZTD numbers tell the story.  Time filter
`[FIXEDPOS_ZTD]` is healthy at **-157 ± 35 mm** (PRIDE on the
same antenna at DOY 126 reports **-61 mm**, also mm-scale —
same family).  Position filter `[AntPosEst]` ZTD is volatile,
**range -1770 to +6205 mm**, mean +88 mm.

This split is exactly the per-filter signature Charlie laid
out in I-175645 part 2:

> MISS phase bias is fatal for position math but invisible to
> TD-CP time math.  TD-CP differences cancel constant offsets
> between epochs; raw IF residuals + MW/WL math do not.

Smoking gun in the bravotest log: **8.5% of `[PB_APPLIED]`
events have BOTH bands MISS** (G13 80 events, E11 105 events,
E28 2 events; G20 has f1 MISS, f2 OK = partial-correction
case, 107 events).  These SVs are processed without
phase-bias correction by either band:

```
[PB_APPLIED] G13 f1=GPS-L1CA→C1C val=MISSm src=? f2=GPS-L5Q→C5Q val=MISSm src=? avail=[]
[PB_APPLIED] E11 f1=GAL-E1C→C1C val=MISSm src=? f2=GAL-E5aQ→C5Q val=MISSm src=? avail=[]
```

The audit's per-SV cycle-drift ranges align with the MISS-PB
SVs:

| SV  | mw_only_slips | fixes | n_wl range | PB coverage |
|-----|---------------|-------|------------|-------------|
| G13 | 1597          | 3     | +16 to +137 (121) | **MISS both** |
| E11 | (not in top)  | 1+    | -          | **MISS both** |
| G20 | 1198          | 3     | -5 to +24 (29)    | f1 MISS, f2 OK |
| E10 | 722           | 7     | -119 to -17 (102) | both OK     |
| E31 | 874           | 4     | -7 to +150 (157)  | both OK     |

G13 with both-band MISS is in the worst-drift cohort.  G20
with f1 MISS shows narrower range — partial corruption.
Note E10 / E31 with full PB coverage still drift large
ranges, so MISS-PB is not the only mechanism — the streaming
MW arc-length ceiling (Layer 2 residual) accounts for the
rest.

### Conclusion: necessary, not sufficient — three layers

1. **PR-noise false positives** (Layer 1) — Fix-I closes.
2. **Short-arc smoother convergence** (Layer 2) — Fix-H helps,
   doesn't fully close on a site with high cycle-slip rate.
   Streaming MW running mean fundamentally bounded by
   arc-length distribution.
3. **MISS phase bias on partial AC coverage** (Layer 3) —
   *not addressed by fix-I or fix-H*.  Charlie's I-175645
   gate-widening at `realtime_ppp.py:1030-1035` is the
   load-bearing fix for this layer.  Per-filter design (drop
   from PPPFilter, keep in FixedPosFilter) preserves time
   filter benefit while removing position-side bias.

The PRIDE same-antenna comparison cleanly separates "engine
model gap" from "data signal."  PRIDE on this antenna sees
mm-scale ZTD; engine position filter sees multi-meter ZTD
volatility.  The bias is in the engine.  Layers 1+2 are
structural in the streaming PPP filter; Layer 3 is a
correctness gap that I-175645 closes.

### Recommended next moves

In order of expected leverage on the residual:

1. **Land Charlie's I-175645** (engine MISS-bias gate widening)
   per-filter.  Closes Layer 3 on `realtime_ppp.py:1030-1035`.
   ~25-40 LOC + tests; biggest single improvement to AntPosEst
   ZTD on this dataset.

2. **CN0/elev-weighted MW samples** (originally fix-L on the
   I-155354 menu).  Down-weight low-quality observations
   feeding MW running mean.  Sharpens fix-H.  ~30 LOC.

3. **Wait-and-fix hold for fresh arcs** (originally fix-K).
   Don't fix WL until smoother has at least N converged
   epochs (e.g., n ≥ 120).  Sacrifices fix latency for
   integer correctness on short arcs.  ~30-50 LOC.

Tier 3 LAMBDA (fix-J) remains research-only for now —
none of the post-fix-I/H residuals require LAMBDA-grade
decorrelation.  The Layer-3 phase-bias gap dominates.

### Architectural framing — Bob's "margin of safety"

ARP-motion detection budget under post-fix WL with Layer 3
still open:

- AntPosEst σ_pos = 0.147 m (converged) — λ_WL/8 ≈ 9 cm
  precision well within Bob's 1-m budget.
- AntPosEst basin offset = 1.5-4 m — exceeds the 1-m budget.
  Until I-175645 closes Layer 3, AntPosEst-as-blend-source
  costs the budget.
- Time filter `FIXEDPOS_ZTD` σ = 35 mm — clean throughout.
  Stage 4 architectural decoupling means the position
  basin doesn't leak into time.

Production setting under Stage 4 = Stage 5: NAV2-blend remains
the right default until I-175645 lands.  After I-175645 +
fix-I + fix-H + (likely) fix-L: AntPosEst-blend becomes
viable, σ_pin can drop below NAV2 hAcc, and the system enters
Bob's "moonshot" regime where AntPosEst beats NAV2 precision
without any cross-host or longer-arc dependency.

## References

- I-155354-main (parent investigation)
- I-125649-main (the strip-NL refinement that surfaced the
  WL-only basin offset)
- I-175645-charlie (engine MISS-phase-bias gate widening,
  per-filter design — Layer 3 fix)
- docs/l5i-l5q-phase-bias-empirical.md (CNES L5I vs WHU L5Q
  bias mismatch, related)
- docs/bds-b2a-phase-bias-survey-2026-05-09.md (Charlie's
  AC-coverage survey)
- docs/streaming-ekf-ceiling.md (why streaming PPP-AR can't
  reach IGS-class — same family of issues)
- scripts/replay/wl_vs_pride.py (engine-vs-PRIDE harness)
- scripts/replay/slip_vs_wl_audit.py (cycle-slip / WL-refix
  audit harness)
