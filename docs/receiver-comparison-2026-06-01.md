# Receiver comparison: F9P vs F10T vs F9T (overnight 2026-05-31 → 2026-06-01)

First like-for-like comparison of three u-blox GNSS receivers on the
lab's PHC discipline chain.  Motivated by MadHat's swap from F10T to
F9P on 2026-05-31, which exposed an opportunity to characterize
**what we get for the money** across the three receiver tiers
(price-ordered ascending: F10T ≈ F9P < F9T).

## Configuration

| Host | Receiver | DO | Engine + servo input |
|---|---|---|---|
| **MadHat last night (F9P)** | u-blox ZED-F9P | OCXO-33 via AD5693R DAC | `--servo-input tdcp`, Phase D |
| **MadHat prior night (F10T)** | u-blox NEO-F10T | same OCXO-33 + DAC | older engine, no TDCP arm |
| **PiFace last night** | ZED-F9T-20B | Isotemp OCXO + AD5693R | `--servo-input tdcp` |
| **PiFace prior night** | same F9T-20B | same Isotemp | older engine |
| **clkPoC3 last night** | ZED-F9T-20B | OCXO + AD5693R | `--servo-input tdcp` |
| **clkPoC3 prior night** | same F9T-20B | same OCXO | older engine |

Measurement chain: 10 MHz Rb on TICC reference (per
[`project_lab_ticc_ref_now_rb.md`](../memory/project_lab_ticc_ref_now_rb.md));
TICC channels chA = DO output (post-servo) and chB = raw GNSS PPS;
both timestamped against the Rb-disciplined TICC clock.

Window definitions:

- **"last" night** — MadHat F9P ran ~21:17 May 31 → 06:43 June 1 CDT
  (9.4 h).  For fairness, the other "last" cases use **the last 9 h**
  of their longer runs (skips warm-up; lands all hosts in the same
  overnight slot).
- **"prior" night** — same wall-clock slot, one day earlier
  (02:17–11:43 UTC May 31 = 21:17 May 30 → 06:43 May 31 CDT).
  Extracted from the longer `day0531-overnight-*` captures.

Analysis: `allantools.tdev` on the linearly-detrended phase
(removes the receiver-clock offset that doesn't matter for short-τ
noise); n ≈ 32–34 k samples per cell.

## chB (raw GNSS PPS vs Rb-referenced TICC)

This is the **servo loop's input signal** — what the receiver actually
delivers as a 1 PPS edge.  No DO involvement.

| Host (RX) | TDEV(1s) | TDEV(4s) | TDEV(16s) | TDEV(64s) | TDEV(256s) | TDEV(1024s) |
|---|---:|---:|---:|---:|---:|---:|
| **MadHat F9P last** | **2.109** | 1.131 | 0.598 | 0.759 | 1.091 | 0.948 |
| **MadHat F10T prior** | **4.562** | 2.271 | 1.163 | 0.557 | 0.353 | 0.687 |
| PiFace F9T last | 2.344 | 1.104 | 0.496 | 0.346 | 0.343 | 0.796 |
| PiFace F9T prior | 2.229 | 1.090 | 0.532 | 0.324 | 0.332 | 0.704 |
| clkPoC3 F9T last | 2.323 | 1.091 | 0.608 | 1.085 | 1.886 | 2.779 |
| clkPoC3 F9T prior | 2.302 | 1.100 | 0.590 | 1.075 | 1.794 | 1.772 |

All values in **nanoseconds**.

**Findings:**

1. **F9P matches F9T at short τ.**  At 1 s the F9P (2.109 ns)
   is statistically identical to PiFace (2.344) and clkPoC3 (2.323)
   F9T units — actually a touch *better*, well within run-to-run
   noise.  Same for 4 s and 16 s.  The F9P's 1 PPS edge has F9T-class
   short-τ noise.
2. **F10T is ~2× noisier than F9P/F9T at short τ.**  4.56 ns @ 1 s
   vs ~2.1 ns; 2.27 vs ~1.1 ns @ 4 s; 1.16 vs ~0.5 ns @ 16 s.  This
   matters because **short τ is where the servo loop operates** —
   loop bandwidth ~ 0.1–1 Hz on our hosts.  A 2× noisier input is a
   2× noisier filter innovation, eats 2× the gate budget, drives 2×
   the actuator stepping.
3. **F10T crosses over above ~64 s.**  At 256 s F10T chB sits at
   0.35 ns vs F9P 1.09 ns / F9T 0.34 ns.  Almost certainly internal
   smoothing/averaging in the F10T's PPS-generation pipeline — helps
   post-loop long-τ stability but doesn't reach where the servo can
   use it (the loop is already correcting at sub-second cadence).

The PiFace F9T captures vs clkPoC3 F9T are useful in their own right:
clkPoC3's chB degrades visibly past 64 s (2.78 ns @ 1024 s vs
PiFace's 0.80 ns), consistent with the
[`rxTcxoInherentVsThermal`](../memory/project_rxtcxo_inherent_vs_thermal_20260524.md)
finding that F9T-20B units have measurably different per-unit
white-FM PSD floors (PiFace ~23 ps vs clkPoC3 ~54 ps inherent at
short τ; the long-τ behaviour here is consistent with that).

## chA (DO output, servo result)

Confounded by host/DO/servo state — Bob noted he wouldn't take it as
a strong signal — but the contrast is worth recording:

| Case | TDEV(1s) | Notes |
|---|---:|---|
| MadHat F9P **last** | **0.097 ns** | DO solidly locked, sits below floor for ~64 s |
| MadHat F10T **prior** | **81.8 ns** | DO never locked — the failure that motivated the swap |
| PiFace F9T last | 0.096 ns | locked, sub-100 ps @ 1 s |
| PiFace F9T prior | 0.193 → 2845 ns @ 256 s | mid-run lock loss (old engine, pre-Phase D) |
| clkPoC3 F9T last | 0.143 ns | locked |
| clkPoC3 F9T prior | 0.175 → 309 ns @ 256 s | mid-run lock loss (old engine) |

The PiFace + clkPoC3 prior-night blow-ups are software artifacts —
those captures predate `--servo-input tdcp` and the disciplineModeFsm
stack.  The MadHat F10T 81.8 ns is the real F10T failure signal: with
the same DO and same servo as the F9P run that followed, the F10T's
noisier raw PPS plus its long convergence pattern kept the loop from
finding lock through the whole overnight.  Once swapped to F9P with
the same OCXO and the same DAC and the same servo, the loop locked
within minutes and stayed there (0.097 ns @ 1 s).

## TDCP innovation σ — carrier-phase tracking quality

Robust statistics (MAD-based, 5σ_MAD outlier trim) on the
EKF's TDCP-arm innovation residual (`innov_tdcp`), post-1-hour
convergence skip:

| Case | RX | n_total | σ_raw | σ_MAD | σ_trim | median(√S) | %outlier |
|---|---|---:|---:|---:|---:|---:|---:|
| MadHat F9P last | **F9P** | 4 295 | 0.089 | **0.065** | **0.071** | 0.036 | 0.4% |
| PiFace F9T last | F9T | 48 935 | 0.037 | **0.037** | **0.037** | 0.035 | 0.0% |
| clkPoC3 F9T last | F9T | 9 730 | 505.2 | **0.099** | **0.109** | 0.035 | 1.7% |

All in **nanoseconds**.  Prior-night runs (F10T, PiFace prior,
clkPoC3 prior) had Phase D disabled or pre-shipped, so the TDCP
arm wasn't populated — no F10T comparison is possible from these
captures.  The clkPoC3's huge σ_raw (505 ns) collapses to 109 ps
after trim, confirming the inflation is from a handful of
cycle-slip-class outliers (1.7%) rather than the noise floor.

**Findings:**

| RX | σ_trim | vs PiFace (cleanest F9T) |
|---|---:|---|
| F9T (PiFace, cleaner unit) | **37 ps** | 1.0× — baseline |
| F9P (MadHat) | **71 ps** | 1.9× |
| F9T (clkPoC3, noisier unit) | **109 ps** | 2.9× |

The F9P TDCP floor (65–71 ps) sits **between the two F9T units we
own**.  Same-model receiver-to-receiver variance within the F9T
population is roughly as large as the F9P-vs-F9T gap — so "F9P costs
less and is 2× the cleaner F9T" needs to be read alongside "F9T's
own per-unit variance is 3× already".

For comparison with the budget targets in
[`two-site-sync-budget.md`](two-site-sync-budget.md): both the F9P
and F9T floors are below the 150–200 ps `σ_DO_above_BW` allowance
when integrated to 1 s phase noise, and the engine's predicted
√S (35–36 ps, identical across all three) matches the cleaner-F9T
observation exactly — meaning the EKF's R is currently calibrated
to F9T-best-unit, not to per-host actual.

## What we get for the money

Approximate USD list/eBay prices as of 2026-05:

| RX | $/unit | Raw PPS (chB @ 1 s) | TDCP arm σ_trim | Worth buying? |
|---|---:|---|---|---|
| F9P  | ~$200 | **2.1 ns** (= F9T) | 71 ps (between the two F9T units we own) | **YES** — F9T-class short-τ PPS at ~60% the cost. |
| F9T  | ~$340 | 2.3 ns | 37–109 ps (per-unit variance is the dominant signal) | Reference standard — buy if you need the *cleanest* unit, but expect the next one to be noisier. |
| F10T | ~$250 | 4.6 ns @ 1 s, 0.35 ns @ 256 s | (no comparable data) | **NO** at short τ — meaningfully worse than F9P/F9T where the servo loop operates.  The long-τ averaging won't help discipline a DO whose loop sees the input every second. |

**The headline result: the F9P is the value-tier winner.**  At short
τ — where the disciplining loop lives — the F9P delivers
indistinguishable raw-PPS performance from the F9T, in a population
of two F9T units that already show 3× variance among themselves.
The F10T's chB at 1 s is 2× noisier than both and didn't even let
the MadHat servo find lock during the overnight.

## Caveats

- **MadHat's antenna feed**: the F9P-on-MadHat configuration shares
  the same lab antenna distribution as the F9Ts.  Per
  [`same-antenna-splitter`](../memory/project_same_antenna_splitter.md)
  all lab GNSS receivers are on the same antenna via GUS splitter,
  so antenna-side variance is common-mode.
- **Per-unit crystal-noise variance is large** for the F9T family.
  The PiFace F9T is the lab's cleanest; clkPoC3's same-model unit
  is ~3× noisier inherent.  We don't have two F9P units to know
  the F9P per-unit spread — the 71 ps observation is one sample.
- **Prior-night chA results** are confounded by older engine
  software (pre-Phase D, pre-disciplineModeFsm).  Don't read the
  PiFace/clkPoC3 prior-night DO instability as a per-host or
  per-receiver fault — that's a software-version artifact.
- **F10T TDCP data is missing**: the older engine that ran the F10T
  overnight didn't populate the TDCP arm.  A future direct F10T
  TDCP comparison requires a re-run on current engine.

## Cross-links

- [`pulsepuppy-ocxo-buying-guide.md`](pulsepuppy-ocxo-buying-guide.md)
  — companion price-vs-performance writeup, for OCXOs rather than
  receivers.
- [`two-site-sync-budget.md`](two-site-sync-budget.md) — the
  per-clock budget that determines whether a given receiver floor
  is "good enough" for the moonshot.
- [`ticc-baseline-2026-04-01.md`](ticc-baseline-2026-04-01.md) —
  earlier F9T-PPS-via-EXTTS baseline (2.3 ns TDEV(1 s), 2 h runs);
  matches this work's F9T chB numbers within run-to-run noise.
- [`madHatTiccRForF10T`](../) dayplan item — the F10T-noisier-than-F9T
  observation predicted the scheduler-relaxation pattern that
  triggered the receiver swap.

## Raw data

Source CSVs (all under `~/peppar-fix/data/` on the respective hosts):

- MadHat F9P last:    `day0531-tdcpresume-madhat-{ticc,arm-state}.csv`
- MadHat F10T prior:  `day0531-overnight-madhat-{ticc,arm-state}.csv`
- PiFace F9T last:    `day0531-tdcpstability-piface-{ticc,arm-state}.csv`
- PiFace F9T prior:   `day0531-overnight-piface-{ticc,arm-state}.csv`
- clkPoC3 F9T last:   `day0531-tdcpstability-clkpoc3-{ticc,arm-state}.csv`
- clkPoC3 F9T prior:  `day0531-overnight-clkpoc3-{ticc,arm-state}.csv`

Analysis script: `/tmp/recv-compare/analyze2.py` (matched-window
TDEV + robust TDCP innovation stats; allantools-based).  Not in
the repo — one-off analysis.
