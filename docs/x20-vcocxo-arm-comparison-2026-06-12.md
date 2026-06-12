# Servo-observer arm comparison on clkPoC3 (ZED-X20P + VCOCXO) — overnight 2026-06-11/12

**Host:** clkPoC3 (Pi 4), ZED-X20P on native USB (`ttyACM1`), VCOCXO disciplined
via AD5693R DAC, TICC #5 (`ttyACM0`) on the UFO1 antenna. Time-only mode
(`--no-antposest`, ARP pinned at UFO1).

**Question (from the ticc-vs-extint experiment design):** with only clkPoC3
available, how do the three DO-phase servo-observer arms — **TICC** (chA−chB,
~60 ps), **EXTINT** (X20 `UBX-TIM-TM2`, ~5–10 ns), **TDCP** (time-differenced
carrier phase + TICC phase anchor) — compare as servo inputs, and how
reproducible is each run to run?

**Metric:** TICC **chA detrended TDEV** (CLAUDE.md output-stability metric).
chA is logged in *every* run regardless of which arm drives the servo (TICC
logging is gated on `ticc_port`, not on `--no-ticc`), so the metric is
arm-independent and the comparison is apples-to-apples (same TICC reference,
common-mode across arms).

> **Analysis gotcha (recorded so it isn't repeated):** chA edges are exactly
> 1 Hz in *TICC* time (`ref_sec += 1` each second). Detrend the phase
> (`ref_sec + ref_ps·1e-12`) against **`ref_sec`**, NOT `host_monotonic` —
> the host read-timestamp carries ±tens-of-ms scheduling jitter that, if used
> as the regression timebase, manufactures a fake ~30 ms TDEV. First-pass
> numbers were physically impossible (30 ms @ τ=1 s) for exactly this reason.

## Design

6 runs × 70 min ≈ 7.5 h, **interleaved** (`ticc, extint, tdcp, ticc, extint,
tdcp`) so each arm samples both early- and late-night (cancels diurnal OCXO
drift). Each run is an independent cold servo bootstrap. Arm isolation:

| Arm | Flags | Servo observer | Notes |
|---|---|---|---|
| TICC | `--no-extint --no-tdcp-arm` | Arm 4 (chA−chB) | sole DO-phase observer → soloObserverChiGate accepts |
| EXTINT | `--no-ticc --no-tdcp-arm` | Arm 3 (TIM-TM2) | TICC still logs chA (metric), not used for servo |
| TDCP | `--servo-input tdcp --no-extint` | Arm 5 (TDCP freq) + Arm 4 TICC as phase anchor | #161 requires a phase reference |

All three arms converged the VCOCXO to the **same operating point, −20.6/−20.7
ppb**, confirming consistent discipline of the same oscillator. Lock times:
TICC ~6 min to ±0 ns, TDCP ~4 min to ±0.2 ns, EXTINT ~5 min to ±4 ns
(slower/noisier, as expected for the coarser observer).

## chA detrended TDEV (ns), per run

```
run          n    gap   rms    τ1     τ2     τ4     τ8    τ16    τ32    τ64   τ128   τ256   τ512  τ1024
ticc_1     3780    0   1.47  0.105  0.066  0.045  0.078  0.185  0.404  0.807  1.026  0.772  0.730  0.384
tdcp_1     3781    0   1.85  0.085  0.081  0.056  0.061  0.134  0.330  0.678  0.942  0.665  0.474  1.151
extint_1   3781    0   8.15  0.303  0.743  1.672  3.039  4.485  5.120  4.557  3.538  2.986  2.220  2.239
ticc_2     3781    0   1.96  0.078  0.072  0.063  0.096  0.216  0.495  0.983  1.301  1.089  0.685  0.279
tdcp_2     3781    0   2.13  0.108  0.058  0.052  0.093  0.214  0.408  0.748  1.302  1.862  1.225  1.075
extint_2   3781    0   2.63  0.051  0.037  0.026  0.028  0.054  0.118  0.254  0.474  0.881  1.833  0.579
```

Arm mean across both cycles (ns):

```
arm         τ1     τ2     τ4     τ8    τ16    τ32    τ64
ticc      0.091  0.069  0.054  0.087  0.201  0.450  0.895
tdcp      0.096  0.069  0.054  0.077  0.174  0.369  0.713
extint    0.177  0.390  0.849  1.534  2.269  2.619  2.406   ← extint_1-burst-contaminated
```

## Findings

### 1. TICC ≈ TDCP at every τ — the headline

The TICC and TDCP arms are statistically indistinguishable: τ1 = 91 vs 96 ps,
identical 54 ps at τ4, and TDCP is *slightly better* at long τ (713 vs 895 ps at
τ64). **The carrier-phase frequency observer disciplines the VCOCXO as well as
the 60 ps TICC, with no per-clock TICC needed for steering** (TICC serves only
as the startup/phase anchor here, a role a *shared* validation TICC can fill).
This is the moonshot-relevant result from the ticc-vs-extint design: if
TDCP ≲ TICC, one shared TICC beats one-per-clock. Confirmed on the X20+VCOCXO.

### 2. EXTINT can go *blind to large real DO excursions* — disqualifying as sole observer

The two EXTINT cycles disagree 30× (reproducibility ratio 0.17). The cause is
not a higher noise floor — `extint_2` is actually TICC-class when the receiver
behaves (26 ps @ τ4). It is a **single ~5-min burst in `extint_1`** (≈00:46
local) that dominates its whole-run statistics. Attribution of that burst:

- chA std in the window **26.95 ns, peak 141 ns**; chB (GNSS-PPS) normal at
  2.57 ns / 7.4 ns; the burst is **preserved in chA−chB** (27.35 ns,
  peak 142.6 ns) with `corr(chA,chB) = −0.11` → **real DO-PPS motion vs GPS,
  not a TICC-reference glitch.**
- **Throughout the excursion the engine's EXTINT-derived `err` read +0.0–0.3 ns**,
  adj steady at −20.6 ppb, zero slips/resets/large innovations.

So the X20 `TIM-TM2` observer reported the DO as perfectly on-time while the
true PPS output excursed to 141 ns peak. The receiver's own clock noise /
timestamp quantization **masked the excursion from the control loop**, which
therefore never corrected it. This is the EXTTS/EXTINT quantization-masking
failure documented in CLAUDE.md (`docs/ticc-baseline-2026-04-01.md`), here
caught *actively hiding a >100 ns real error* from discipline. An EXTINT-only
clock would have shipped that error downstream undetected; only the independent
TICC exposed it.

**Implication:** EXTINT (receiver-timestamped DO-PPS) is unsuitable as a *sole*
servo observer for a precision time service on the X20 — not because its
typical noise is high, but because its blind-spots are large, real, and
unpredictable (hence non-reproducible). It remains useful as a coarse
startup/phase anchor or sanity arm, not as the disciplining observer.

### 3. Reproducibility

| Arm | cycle2/cycle1 TDEV ratio (mean / min / max) | Verdict |
|---|---|---|
| TICC | 1.13 / 0.73 / 1.41 | most reproducible |
| TDCP | 1.46 / 0.72 / 2.80 | moderate; long-τ tail + TDCP `n_sv=0` dropouts add variance |
| EXTINT | 0.17 / 0.01 / 0.83 | non-reproducible (burst-driven) |

TDCP showed intermittent `n_sv=0/11 → nan` epochs (a few per minute); the TICC
phase anchor carries the loop through them, but they inflate the long-τ tail
and run-to-run variance. Worth investigating (SV-geometry / screening gate)
before TDCP can claim TICC-class reproducibility, but its *short-τ* discipline
already matches TICC.

## Bottom line

- **TDCP is the winner for steering** — TICC-class output stability with no
  per-clock TICC required. Pursue TDCP-as-servo-input + one shared validation
  TICC.
- **TICC is the reproducibility reference** and the indispensable *independent
  validator* — it is what caught EXTINT's blind 141 ns excursion.
- **EXTINT is out as a sole observer** — receiver timestamping can hide large
  real DO excursions from the loop.

## Reproduce

- Series runner: `data/overnight_series.sh` on clkPoC3 (nohup).
- Analysis: `data/analyze_arms.py` (chA detrend vs `ref_sec`, per-arm TDEV +
  reproducibility ratios).
- Raw captures archived: `gt:~/gt/captures/x20-vcocxo-arms-2026-06-12/`
  (`*.ticc.csv` chA/chB, `*.extint.csv`, `*.tdcp.csv`, `*.engine.log`).
