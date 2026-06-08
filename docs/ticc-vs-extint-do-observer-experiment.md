# Experiment design: TICC vs EXTINT as the DO-PPS phase observer

**Decision this informs:** when building new DO clocks, is a **per-clock
TICC** worth its cost (board, serial port, the DTR-reset gotcha) over the
**free EXTINT** path the F9T/PHC already provides? Both observe DO PPS
phase; TICC (~60 ps) is far more accurate than EXTINT (~5–10 ns), but TDCP
(~15–40 ps) — which needs *no* TICC — is more accurate still. So the real
question has a punchline: **if TDCP-discipline matches/beats a TICC, you
may need only one *shared* TICC for validation, not one per clock.**

Status: **design** — not yet run.

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

## Open questions

- Is τ ≥ 300 s worth longer runs (4–8 h) on a follow-up night, or is the
  1–100 s region (where the servo loop + DO floor live) sufficient for the
  build decision?
- Does TDCP-discipline hold its short-τ advantage over a full 2 h without
  the rx-clock holdover (Arm 6) degrading at gaps?
- Gold-plate: is a second independent TICC available to remove the
  metric/servo coupling in the TICC and both arms?
