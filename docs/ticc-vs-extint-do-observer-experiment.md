# Experiment design: TICC vs EXTINT as the DO-PPS phase observer

**Decision this informs:** when building new DO clocks, is a **per-clock
TICC** worth its cost (board, serial port, the DTR-reset gotcha) over the
**free EXTINT** path the F9T/PHC already provides? Both observe DO PPS
phase; TICC (~60 ps) is far more accurate than EXTINT (~5–10 ns), but TDCP
(~15–40 ps) — which needs *no* TICC — is more accurate still. So the real
question has a punchline: **if TDCP-discipline matches/beats a TICC, you
may need only one *shared* TICC for validation, not one per clock.**

Status: **design + first-overnight results** (2026-06-08, see Results
below — preliminary/inconclusive on the noise floor; needs a redo).

## What each chain observes (the servo arms)

From [dofreq-est-measurement-ladder.md](dofreq-est-measurement-ladder.md),
the DOFreqEst Kalman fusion arms:

| Arm | Observable | σ | Role in this experiment |
|---|---|---|---|
| Arm 1 (PPP `dt_rx`) | rx-TCXO phase x[0] | — | rx-clock anchor — **kept on, identical, in every arm** |
| Arm 2 (qErr) | rx-TCXO freq x[1] | — | rx-clock anchor — **kept on, identical** |
| **Arm 3 (EXTINT / TIM-TM2)** | DO phase x[2] | **~5–10 ns** | the "EXTINT" observer |
| **Arm 4 (TICC chA−chB)** | couples x[0]+x[2] | **~60 ps** | the "TICC" observer |
| Arm 5 (TDCP) | rx clock (carrier) | ~15–40 ps | ceiling reference (no TICC needed) |

Only the **DO-phase observer** (Arm 3 vs Arm 4 vs both vs Arm 5) is varied
between arms; the rx-clock anchors (Arms 1+2) stay identical so the *only*
variable is how the loop sees the DO.

## The metric and why it's fair

**TDEV of TICC chA (DO PPS, detrended)** — the lab convention (chA alone,
*not* chA−chB; see CLAUDE.md "TICC stability metric"). chA is the absolute
phase stability a downstream consumer actually sees; chA−chB would measure
servo *tracking*, not output quality.

**Measurement independence (the crux), handled by the engine:** every arm
runs with `--ticc-port` wired, so the TICC physically logs chA in *all*
arms. `--no-ticc` gates the TICC out of the *servo* but **keeps logging it**
("TICC measurements are still logged… not fed to the servo"). So:

- EXTINT-only & TDCP arms: servo input is independent of the TICC → chA is a
  fully independent reference-grade metric. ✓
- TICC-only & both arms: the metric TICC is also the servo TICC. Mild
  coupling, but the servo minimizes chA−chB (tracking) while the metric is
  chA detrended (absolute) — different quantities — and the TICC reference
  noise is a **common floor across all arms**, so cross-arm *differences*
  remain valid. (Gold-plate option: a *second*, independent TICC for the
  metric in these two arms, if one is free.)

τ range: **1 s to ~1000 s.** Headline comparisons at **τ = 1 s** (the
per-clock 1 Hz servo budget — see [[position-accuracy-target-10cm]]'s
sibling, the two-site sync budget) and 10 s / 100 s. ⚠ At τ = 1000 s a 2 h
run gives only ~7 samples → wide error bars; treat τ ≥ 300 s as
indicative, not final, unless runs are extended.

## The arms — exact engine invocation

Common to every run (time-only, true position seeded):
```
--no-antposest --known-pos <ufo1 ITRF2020 truth>   # pin at truth; clock-only
--servo /dev/<ptp> --pps-pin <n>                    # DO discipline
--ticc-port /dev/ticcN                              # TICC ALWAYS wired (metric)
--systems gps,gal --duration 7200
```
(known-pos from `timelab/antennas.json`; `--no-antposest` keeps the position
solution from contaminating the clock — exactly the time-mission mode.)

| Arm | Add to the common flags |
|---|---|
| **EXTINT-only** | `--no-ticc` (Arm 4 off, Arm 3 on; TICC still logged) |
| **TICC-only** | `--no-extint` (Arm 3 off, Arm 4 on) |
| **both** | *(neither flag — Arms 3+4 both feed the servo)* |
| **TDCP (ceiling)** | `--servo-input tdcp --no-ticc --no-extint` |

## Host requirements & selection

A clean run needs all three on one **OCXO-class** DO (TCXO-class DOs are
best-effort per CLAUDE.md and their noise floor can mask the observer
difference — see below): **OCXO DO**, **TICC** wired chA = DO PPS / chB =
F9T PPS, and **DO PPS → F9T EXTINT** pin.

| Host | DO | TICC | EXTINT | Verdict |
|---|---|---|---|---|
| **MadHat** | OCXO-33 + DAC | yes (chA/chB) | **missing** (TICC-primary host) | **Primary** — add a DO-PPS→F9T-EXTINT jumper (split the existing OCXO-PPS→TICC-chA line into the F9P EXTINT pin). One-time wiring; also literally the capability being evaluated. |
| TimeHat | i226 TCXO | yes (#1) | yes (EXTTS) | **Secondary / cross-DO check** — has all chains but DO is TCXO; runs the same schedule to test whether the TICC>EXTINT trend holds on a second (noisier) DO. |
| ocxo (E810) | OCXO | #3 | EXTTS | Avoid: F9T PPS not externally TICC-accessible (no chA−chB), EXTTS 8 ns quant. |
| otcBob1 / ptBoat | OCXO (ClockMatrix) | — | on-chip TDC | Different architecture; out of scope here. |

**Pre-flight: confirm/add MadHat's EXTINT wiring** (the [[sole-observer-arm-cannot-be-deweighted]] note records MadHat as "no EXTINT").

## Overnight schedule (interleaved, reproducible)

One DO = arms must be **sequential** to keep the DO constant — but
sequential arms are vulnerable to overnight thermal/sky drift (the
matched-pair lesson from the position investigation). So **interleave and
repeat**, don't block:

```
Cycle A (≈6 h):  EXTINT(2h) → TICC(2h) → both(2h)
Cycle B (≈6 h):  both(2h)  → TICC(2h) → EXTINT(2h)   # reversed order
```
≈12 h → **2 runs per arm in different time-windows** (reproducibility +
de-confounds the time-trend, since each arm samples both early and late
night). TDCP either replaces one cycle's slot or gets its own night.

**Secondary host (TimeHat) runs the same interleave in parallel** → the
TICC-vs-EXTINT *trend* is checked on a second, independent DO at no extra
wall-clock cost.

Between arms: ~2–3 min settle after re-launch (warm start; discard the
first few minutes of each segment from the TDEV computation).

## Reset-free is a hard requirement

A mid-run reset re-seeds the servo and **destroys the TDEV** of that
segment. So:
- **Mitigate:** warm start (drift/freq seed), `--ocxo-trusted-gate` where
  applicable, loosened NAV2/watchdog thresholds, and the exit-5→servo-reset
  mitigations already in main.
- **Detect & discard:** count `=== Phase 2` relaunches per segment; any
  segment with a relaunch is dropped and re-run, not analyzed. Report the
  reset rate alongside the TDEV (a chain that resets more is itself a mark
  against it).
- Log **case/OCXO temperature** throughout (the foam-removal lesson —
  thermal hunting masquerades as servo noise).

## Analysis & decision framework

For each arm: overlay TDEV(chA, detrended) across its repeat runs (show the
run-to-run spread as the reproducibility band), then compare arms at τ = 1,
10, 100 s.

| Outcome | Decision |
|---|---|
| TICC TDEV ≪ EXTINT at the budget τ | **TICC earns its place per clock** |
| TICC ≈ EXTINT | EXTINT (free) suffices — **skip the per-clock TICC** |
| both ≈ TICC | EXTINT is redundant when a TICC is present (no fusion gain) |
| both ≪ TICC | fusion helps — keep both chains |
| **TDCP ≲ TICC** | **discipline by TDCP (no TICC needed operationally) → one *shared* TICC for validation, not one per clock** ← the likely punchline |

The TCXO secondary (TimeHat) calibrates interpretation: if the observer
difference is visible even on a noisy TCXO DO it's robust; if TICC≈EXTINT on
the TCXO but TICC≪EXTINT on the OCXO, that confirms the value is DO-floor-
dependent (a better DO makes a better observer matter more — directly
relevant to "building OCXO clocks").

## Pre-flight checklist

- [ ] MadHat DO-PPS→F9T-EXTINT jumper installed & TIM-TM2 emitting.
- [ ] TICC chA = DO PPS, chB = F9T PPS confirmed (per-host); HUPCL handled
      (`scripts/ticc.py`).
- [ ] `--known-pos` = current ufo1 ITRF2020 from `timelab/antennas.json`.
- [ ] Warm-start state present (no cold-start transient inside a 2 h window).
- [ ] Free-run DO floor characterized first (so we know how much headroom
      the observer even has to matter).
- [ ] Analysis: `allantools` TDEV on chA detrended; reset/temperature logs.

## Results — first overnight (2026-06-08)

Run staggered 2 h arms × 4 windows on 3 OCXO hosts (03:38–11:56 UTC).
Data: `gt:/home/bob/gt/peppar-fix-data/ticcexp-20260608/`. **The yield was
heavily degraded — read the verdict with that in mind:**

- **MadHat logged chB only** (DO PPS not wired to TICC chA) → no metric, and
  its TICC arm (chA−chB) had no DO-phase input; all MadHat arms stuck at
  `DOFreqEst=phase` (compounded by its range-starved OCXO). **Fully
  excluded. ACTION: wire MadHat DO-PPS → TICC chA.**
- **Single-observer arms cascaded** (exit-5 "servo lost control / 30
  consecutive outliers", excluded): TICC-only on clkPoC3 (31×), TDCP on
  clkPoC3 (7×) and PiFace (9×); clkPoC3-extint took 1 reset (phase-jump,
  excluded). "both" never cascaded.
- **PiFace TICC is on a free-running OCXO reference** → its long-τ TDEV is
  ref-wander-dominated (relative-only; short τ usable).
- **One run per arm per host, in different windows** → the long-τ arm
  comparison is window/ref confounded.

Net clean dataset: the PiFace τ=1 s triad (relative) + clkPoC3 "both"
(absolute). TDEV of TICC chA (detrended), ns:

| host (ref) | arm | τ=1 s | τ=10 s | τ=100 s |
|---|---|---|---|---|
| clkPoC3 (Rb, **absolute**) | both | **0.105** | 0.100 | 2.24 |
| PiFace (OCXO, **relative**) | extint | **0.141** | 11.3* | 70* |
| PiFace (OCXO, **relative**) | both | 0.341 | 16.4* | 51* |
| PiFace (OCXO, **relative**) | ticc | 0.428 | 5.3* | 8.1* |

\* long-τ ref-wander/window confounded — not a clean DO measure.

**Reads:**
- **clkPoC3 "both" is a clean, healthy disciplined-OCXO output**:
  TDEV(1 s)=105 ps, flat to ~10 s, 2.2 ns @100 s (Rb-referenced, absolute).
- **At τ=1 s (least ref-confounded): EXTINT (141 ps) ≤ both (341) ≤ TICC
  (428).** Suggestive — and *opposite* the "better observer → better output"
  prior — that the tight 60 ps TICC observer **over-actuates** and injects
  more short-τ noise than the gentle ~ns EXTINT. Single-run, relative →
  suggestive, not conclusive.
- Long-τ ordering (ticc < both < extint) *hints* the tight observer tracks
  GPS better at long τ, but it's ref/window-confounded — set aside.

**Robustness — the strongest finding:** "both" (TICC+EXTINT fused) was the
**only arm that stayed locked + clean on every host where the OCXO could
lock.** Every single-observer arm cascaded somewhere. Redundant observation
buys servo stability.

**Preliminary verdict:** this run does **not** make the noise-floor case for
a per-clock TICC — at the budget-relevant short τ, EXTINT is at least as
good (likely better; TICC over-actuates), and the long-τ TICC edge is
confounded. The robustness case favors **"both"**. So: **don't add a
per-clock TICC on noise-floor grounds yet** — the fused/EXTINT path is
competitive. A conclusive decision needs a redo with the fixes below.

**Protocol fixes for a conclusive redo:**
1. Wire MadHat DO-PPS → TICC chA (or drop MadHat).
2. ≥2 repeats per arm per host (reproducibility + window de-confounding).
3. Rb-referenced TICC on the comparison host(s) for clean long-τ (PiFace's
   free OCXO ref destroys long τ).
4. Fix/prevent the single-observer cascades (retune the exit-5 outlier gate)
   so TICC-only and TDCP actually yield data.
5. 4 h+ windows for solid τ ≥ 100 s.

## Results — second overnight (2026-06-09, after the servo fix)

Re-run after fixing the outlier doom loop (`servoOutlierDoomLoop` →
re-acquire on a sustained ramp, not let-through; main `0af1a3e`). Data:
`gt:/home/bob/gt/peppar-fix-data/ticcexp3-20260609/`. **Caveats:** clkPoC3
was the only host with a usable metric — MadHat's DO-PPS→TICC-chA cable was
dead all night (chA=0 every window), and PiFace drifted on *every* arm
(−2.5 to −14.7 ppb, host-specific, excluded). W4 was truncated to ~64 min
(operator killed it ~56 min early); τ ≤ 100 s still valid there.

**Crucial method fix:** verify lock by **chA−chB drift**, not
`DOFreqEst=tracking` — the buggy let-through (the first version of the doom-
loop fix) drifted silently because the filter state *and* the detrended-TDEV
metric are both blind to frequency drift. Caught on the scope.

clkPoC3 4-arm (Rb-ref, absolute), TDEV of chA detrended (ns):

| arm | re-boots (exit-5) | τ=1 s | τ=10 s | τ=100 s | status |
|---|---|---|---|---|---|
| **EXTINT-only** | **0** | **0.052** | 0.037 | 0.91 | clean, locked |
| **both** | **0** | 0.086 | 0.121 | 2.09 | clean, locked |
| TDCP | 3 | — | — | — | light sawtooth (TDEV step-contaminated) |
| TICC-only | 34 | — | — | — | heavy sawtooth (TDEV step-contaminated) |

**Findings:**
- **Drift regression fixed:** all four arms now *lock* (chA−chB drift ≈ 0),
  vs the buggy let-through's silent drift.
- **Robustness (re-bootstrap count): EXTINT (0) = both (0) < TDCP (3) ≪
  TICC-only (34).** The single TICC observer is the *least* robust (its
  large innovations are chi²-gated hardest, forcing constant re-acquire);
  EXTINT-only is as robust as the fused "both".
- **Noise floor (clean, 0-re-boot arms): EXTINT-only (52 ps @1 s) < both
  (86 ps @1 s) at every τ.** The gentle single observer gives the cleanest
  output; the second observer ("both") adds robustness margin at a small
  TDEV cost. Consistent with the first overnight (extint ≤ both ≤ ticc at
  τ=1 s). The "tight TICC observer over-actuates" reading now holds on two
  hosts/receivers.
- TICC-only and TDCP TDEV are **step-contaminated** by their re-bootstraps,
  so their *intrinsic* noise floor is still not measured — the in-process
  re-acquire never engaged (reset budget exhausted → full re-bootstrap each
  time). A fair TICC-only floor needs the reset budget loosened.

**Verdict (strengthened, with one caveat):** a **per-clock TICC is not
justified** by this data — EXTINT (free) is the cleanest at short τ *and*
as robust as "both", while TICC-only is the least stable arm. Favor
**EXTINT, or "both" when you want robustness margin**; a TICC is better
deployed as a *shared validation* instrument than one-per-clock. Caveat:
TICC-only's true floor is unmeasured (the reset-budget bug makes it
sawtooth), so the head-to-head isn't fully apples-to-apples until that's
fixed.

**Blockers for a fully clean head-to-head (next attempt):**
1. **Reset budget** — let the in-process re-acquire engage so single-
   observer arms lock *gently* (no re-bootstrap steps) → clean TICC-only /
   TDCP TDEV.
2. **PiFace** host-specific drift (drifts on every arm; not the doom loop).
3. **MadHat** DO-PPS→TICC-chA cable (dead all night → no metric).

## Open questions

- Is τ ≥ 300 s worth longer runs (4–8 h) on a follow-up night, or is the
  1–100 s region (where the servo loop + DO floor live) sufficient for the
  build decision?
- Does TDCP-discipline hold its short-τ advantage over a full 2 h without
  the rx-clock holdover (Arm 6) degrading at gaps? (NB: TDCP cascaded on 2/2
  hosts that ran it this round — robustness first.)
- Gold-plate: is a second independent TICC available to remove the
  metric/servo coupling in the TICC and both arms?
