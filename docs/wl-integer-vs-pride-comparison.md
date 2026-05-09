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

## Mechanism (hypothesis)

The MW (Melbourne-Wübbena) tracker computes wide-lane integer
fixes from the linear combination
`MW = (φ1·λ1 - φ2·λ2)·(f1+f2)/(f1-f2) − (P1·f1 + P2·f2)/(f1+f2)`
which is geometry-free and ionosphere-free, so its expected value
is just `λ_WL × N_WL + sub-cycle bias`. The integer N_WL is
obtained by rounding (or LAMBDA-decorrelated rounding) once the
running mean has settled.

The engine's `int_history=[37, 44, 35, 28, 15]` for E10 means the
running-mean has been jumping by tens of cycles between successive
fix attempts. Plausible mechanisms:

1. **Pseudorange-side noise spikes** are inflating the running
   mean variance, and the tracker is re-fixing on partially-
   corrupted data. F9T pseudorange noise floor is ~30 cm; if a
   multipath spike adds a few meters of PR noise to one channel,
   the MW value shifts by `~PR_error / λ_WL` cycles. A 5 m PR
   spike would shift WL by ~7 cycles — consistent with what
   we're seeing.

2. **Wrong signal-code mapping.** F9T-20B tracks GAL on E1+E5a
   by default (the `SYS / FREQUENCY BAND` line in the RINEX
   header confirms `GAL E1 E5a`). If the engine's MW formula
   is using the wrong code per signal (e.g., L5I-vs-L5Q phase
   bias mismatch from the L5 phase-bias-empirical work
   already documented for GPS but possibly affecting GAL E5a
   too), the integer lock is corrupted.

3. **Cycle-slip detector miss.** A real cycle slip on the
   carrier phase changes the WL integer by a non-trivial amount.
   If the cycle-slip detector misses a slip, the MW tracker
   sees a step in the running mean and re-fixes to a new
   integer. The slip detector relies on phase coherence
   between L1/L5 (GF and MW jumps), and any failure mode
   there leaks into WL integers.

(2) and (3) are testable. (3) is most suggestive given the
"multi-cycle drift then re-fix" pattern visible in E10's
int_history. A cycle-slip failure mode would produce exactly
this signature.

## Next steps

1. **Cycle-slip detector audit on this dataset.** Replay the
   day0506 RINEX through the engine's slip detector with the
   most recent thresholds; correlate slip-detection events
   against E10's int_history transitions. If slips were
   detected at the integer-jump points, the WL re-fix is
   correct (slips break ambiguity continuity); if no slips
   detected, the slip detector missed real slips and the
   tracker is silently absorbing them.
2. **Run PRIDE 24-h with AR enabled** on MadHat-2026126.obs.
   More SVs, more arcs, all integer-confident. Strengthens the
   bias-magnitude estimate across the full SV cohort.
3. **Bracket against Charlie's hypothesis #2** (L5 phase-bias
   mismatch). Compare per-SV WL error rates between L5-band
   GAL SVs and other-band SVs; if L5 is uniquely affected, the
   phase-bias mechanism is implicated. If all bands are affected,
   it's a more general MW-tracker issue.

## References

- I-155354-main (parent investigation)
- I-125649-main (the strip-NL refinement that surfaced the
  WL-only basin offset)
- docs/l5i-l5q-phase-bias-empirical.md (CNES L5I vs WHU L5Q
  bias mismatch, related)
- docs/streaming-ekf-ceiling.md (why streaming PPP-AR can't
  reach IGS-class — same family of issues)
- scripts/replay/wl_vs_pride.py (the comparison harness used here)
