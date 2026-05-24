# TDCP Prototype — Findings 2026-05-22

Read-only proof-of-concept for time-differenced carrier-phase (TDCP)
clock-frequency estimation as a candidate servo input.  Question being
answered: can a simple ensemble TDCP estimator reach a short-tau
stability floor competitive with the moonshot's 350 ps/clock budget,
without PPP / AR / position estimation?

**Headline: TDEV(1s) ≈ 15 ps, measured on F9T 1-Hz raw carrier
phase from a 91-epoch capture (TimeHat 2026-03-23).**

## What the prototype does

- Reads carrier phase + sat ephemeris from any of:
  - RINEX 3.x OBS file (via `scripts/regression/rinex_reader.py`)
  - Raw UBX byte-stream log (via `pyubx2` + `RXM-RAWX`)
- Per epoch: for each tracked SV with continuous lock, computes the
  TDCP residual `r(sv) = Δφ_meas − Δρ_geom + c·Δdt_sat` where the
  right-hand side is the broadcast-ephemeris-predicted change in
  range plus satellite clock motion.  All in metres.
- Ensemble across SVs: median, with 3-MAD outlier rejection.
- Outputs `c·Δdt_rx` (m) and `df/f` (dimensionless) per epoch.
- Postprocess: TDEV(τ) via `allantools` on the df/f series.

No state, no Kalman filter, no ambiguity resolution, no SSR, no ZTD,
no AR.  Single-frequency L1.  ~600 lines total including the RINEX
NAV parser.

## Test data

- **UBX 1 Hz**: `/home/bob/gt/lab-archive/timehat-old-clone-2026-04-08/qerr-ticc-run/peppar_20260323T011546_raw.ubx`
  - TimeHat F9T (TIM 2.20 era), CHOKE1 antenna, 91 epochs (~1.5 min)
- **RINEX 30 s**: `/home/bob/git/PePPAR-Fix-bravo/data/madhat-bravotest/MadHat-2026129.obs`
  - MadHat F9T-20B, 294 epochs (~2.5 h)
- **Broadcast ephemeris**: IGS `BRDC00IGS_R` for days 082 and 129 2026,
  downloaded from `igs.bkg.bund.de` (no Akamai gate).

## TDEV results — 1 Hz UBX (TimeHat 2026-03-23)

| Constellation | N SVs | TDEV(1s) | TDEV(10s) | TDEV(30s) |
|---|---|---|---|---|
| GPS only | 9 (med) | 15.5 ps | 280 ps | 2.30 ns |
| GAL only | 6 (med) | 15.4 ps | 279 ps | 2.28 ns |
| GPS + GAL | 16 (med) | 15.2 ps | 279 ps | 2.29 ns |

**Key observation: TDEV(1s) is essentially independent of SV count.**
If the estimator were measurement-noise limited, adding SVs would
reduce TDEV as 1/√N; we'd see 15 ps → 20 ps when going from 16 to 9
SVs, and to 25 ps at 6.  We see ~15.5 ps regardless.  Conclusion:

> The 15 ps TDEV(1s) is real receiver-clock motion correlated across
> all SVs, not estimator noise.  The estimator's own noise floor is
> well below 15 ps.

This is exactly the regime we want: a measurement chain that delivers
real clock state with negligible noise added on top.

## Cross-check: per-SV residuals

- Per-SV TDCP residual MAD per epoch: **2.8 mm** (median).
- Implied per-SV σ (≈ MAD × 1.4826): **4.2 mm** → 14 ps per single SV.
- Ensemble of 16 SVs: predicted σ_ens = 4.2/√16 = 1.05 mm → **3.5 ps**.
- Measured TDEV(1s) = 15 ps ≫ 3.5 ps prediction.
- Gap factor 4× = correlated component (real clock motion within 1 s).

Per-SV carrier-phase noise of ~3 mm matches u-blox F9 datasheet
spec (≤ 1 mm/√Hz × √(1 Hz × 9) integration window ≈ few mm).

## Comparison with current servo inputs

| Input | TDEV(1s) | TDEV(10s) | TDEV(30s) |
|---|---|---|---|
| PPS raw | ~3 ns | (qual.) | (qual.) |
| PPS + qErr | ~250 ps | — | — |
| FixedPosFilter dt_rx (current) | ~450 ps | — | — |
| **TDCP ensemble (this prototype)** | **15 ps** | **280 ps** | **2.3 ns** |

The 30 ms→s region is where the current filter wastes the carrier-phase
floor.  TDCP recovers nearly all of it.

## TDEV growth with τ — drift dominates, not noise

TDEV grows roughly as τ^1.6 in the 1–30 s range.  That's faster than
white frequency noise (τ^0.5) and slower than pure drift (τ^1.0 → τ^2
depending on integration window).  Interpretation: in this short
window, the F9T's internal TCXO is doing its normal frequency drift
under its slow disciplining loop, and the drift dominates over the
measurement noise.

This is the regime the two-loop architecture targets: a slow PPS-anchored
loop with a long time constant (100–1000 s) trims the drift, while the
fast TDCP loop tracks the noise-free instantaneous frequency.

## What the long-tau result on RINEX (30 s) data showed

| τ | TDEV |
|---|---|
| 30 s | 2.21 ns |
| 60 s | 6.19 ns |
| 120 s | 18.5 ns |
| 300 s | 103 ns |
| 600 s | 381 ns |
| 1200 s | 1.39 µs |

τ^1.5 growth — this is real TCXO drift over the 2-hour MadHat capture.
At these τ values the residual is dominated by oscillator drift, not
measurement chain.  Numbers will look very different once we close the
servo loop and replace the TCXO drift with a disciplined-DO output.

## Limitations & known issues

1. **91 epochs is short** for confident long-tau TDEV.  Need live
   capture (lab hosts) for 1-hour-class runs.  At τ=1s the 91-sample
   estimate has tight confidence interval; longer τ values are less
   trustworthy.
2. **Single host** — should reproduce on at least 2 hosts.  PiFace,
   MadHat would do.
3. **Broadcast ephemeris** — sub-meter position SD per SV.  Switching
   to precise SP3 + clock would tighten the per-SV residual slightly
   but the 15 ps floor is real-clock-limited, not measurement-limited,
   so this would not change the headline.
4. **BDS support written but untested** (BDT/GPST handling sketchy).
   Stick to GPS+GAL until verified.
5. **No actuator wired.**  This is purely measurement.  Real servo
   integration is the next step.
6. **MadHat 30 s data didn't surface 1Hz floor** — only the UBX
   capture from 2026-03-23 has true 1 Hz.

## Recommended next steps

In order of yield-for-effort:

1. **Live 1 Hz capture** on TimeHat + PiFace (when lab access opens),
   1-2 hour run.  Confirm 15 ps holds across host and across time of
   day (multipath, ionosphere quiet vs active).  TICC chA log
   alongside as ground truth on PHC behavior.
2. **Cross-host TDCP comparison.**  If we run the same prototype on
   two hosts seeing the same SVs (shared antenna via splitter), the
   common-mode SV-side effects cancel and we see only host-pair
   differential.  This would let us bound how much of the 15 ps is
   per-host RX-TCXO motion vs SV-side common mode.
3. **TDCP servo path** (write side).  Adapt the engine to consume
   TDCP-derived frequency error as a new servo input alongside the
   existing PPS/PPS+qErr/FixedPosFilter options.  Plumb through
   `--servo-input tdcp`.
4. **Two-loop architecture**: fast TDCP frequency loop +
   slow PPS phase loop.  Cascaded loop-bandwidth design.
5. **Slip handling hardening**.  Current prototype uses UBX `cpValid`
   bit + locktime monotonicity.  Production servo would need
   median-deviation slip detection too (cross-SV consensus catches
   missed locktime-only slips).

## Files

- `prototypes/tdcp_proto.py` — the prototype (~600 lines).
- `prototypes/tdcp-findings-2026-05-22.md` — this writeup.
- `/tmp/tdcp/results_ubx.csv` — per-epoch outputs from the 1 Hz run.
- `/tmp/tdcp/BRDC_082.rnx`, `BRDC.rnx` — IGS broadcast nav for the
  two test days (kept locally for repro).

## Reproducing the headline number

```
cd /home/bob/git/PePPAR-Fix-bravo
/home/bob/git/PePPAR-Fix/venv/bin/python prototypes/tdcp_proto.py \
    --obs /home/bob/gt/lab-archive/timehat-old-clone-2026-04-08/qerr-ticc-run/peppar_20260323T011546_raw.ubx \
    --nav /tmp/tdcp/BRDC_082.rnx \
    --systems GE
```
