# Regression Harness Plan

*Design doc 2026-04-20.  Goal: build a regression test that runs
PePPAR Fix against a published PPP-AR dataset with independent
ground truth, so architectural changes like Path A can be validated
against something other than our own lab.*

## Why

Day0419i-k data shows real position drift (altitude on MadHat swung
±13 m) that survived even after structural fixes (elev-stratified
squelch, admit path, Δaz 8°).  We can see *that* something's pulling
the filter, but without an independent reference we can't tell:

- Whether the drift is caused by our code, our lab's multipath, our
  choice of corrections AC, or some combination.
- Whether a proposed architectural change (Path A: exclude
  NL_SHORT_FIXED from the filter) fixes the problem or just
  shifts the drift elsewhere.

A regression test against a *published* reference station with
a *published* ITRF coordinate gives us an absolute yardstick.

## What we'll use

**PRIDE-PPPAR example data** (GPL-3, bundled in
`github.com/PrideLab/PRIDE-PPPAR`):

| Scenario | Station | Date | Duration | Use |
|---|---|---|---|---|
| static 24h | **ABMF** (Guadeloupe IGS) | 2020 DOY 001 | 24 h | primary regression |
| tropo | ABPO (Madagascar) | 2020 DOY 003 | 24 h | tropo stress |
| kinematic 1h LAMBDA | CCJ2 (Japan) | 2021 DOY 210 | 1 h | kinematic smoke |
| multipath | WUH2 (Wuhan) | 2023 DOY 002 | 24 h | multipath (BDS-3 modern signals) |

**Ground truth**: station coordinates from the IGS weekly SINEX
files — independent of PRIDE itself.  PRIDE's bundled
`results_ref/pos_2020001_abmf` gives
`(2919785.79086, -5383744.95943, 1774604.85992) m ECEF` which
matches the published ABMF MJD 58849 coordinate within mm.

**Pass gate**: converged 24h static solution within
5 mm horizontal / 1 cm vertical of the ITRF14 coordinate.

## Scope

Four independent pieces:

### 1. RINEX observation reader ✅ (this PR)

`scripts/regression/rinex_reader.py` — parses RINEX 3.x OBS files
and produces per-SV dual-frequency observation dicts compatible with
the PPP filter's expected input.  Supports both L2 and L5 receiver
profiles via `L5_PROFILE` / `L2_PROFILE` signal-code maps.

Synthesizes `lock_duration_ms` from the RINEX loss-of-lock
indicator (not as precise as UBX hardware counter, but enough for
cycle-slip detection in static scenarios).

### 2. Broadcast ephemeris reader (follow-up)

`scripts/regression/rinex_nav_reader.py` — parse the `brdm*.*p` /
`brdc*.*n` broadcast NAV files that PRIDE ships alongside obs data.
Must emit records compatible with our `BroadcastEphemeris` class
(`scripts/broadcast_eph.py`) which today parses RTCM 1019 (GPS),
1045/1046 (GAL), 1042 (BDS), 1020 (GLO).  Two options:

a. **Direct object creation**: build in-memory records that match
   our class's internal state, bypassing RTCM parsing.
b. **RINEX→RTCM conversion**: transcode to RTCM 1019/1045/1042
   messages in memory and feed through the existing parser.  More
   complex but preserves the exact code path.

Recommendation: option (a) for first cut — easier to debug.

### 3. Correction source (follow-up)

Two choices per the discussion:

**Option 1**: `osb_bias_sinex_reader.py` — parse Wuhan University's
WUM OSB Bias-SINEX static files.  PRIDE's `install.sh` downloads
them; we can cache a copy.  Format is ASCII with per-SV per-signal
phase+code biases valid for the full day.

**Option 2**: `cnes_ssr_replay.py` — download archived CNES
SSRA00CNE0 RTCM stream from IGN (`ftp://igs.ign.fr/pub/igs/products/
mgex/`) and replay through the existing `RealtimeCorrections` /
`SSRState` parsers.  For old dates (2020) the stream archive may
have gaps; 2021-2023 should be reliable.

Recommendation: **do both**.  Option 2 exercises our runtime code
path exactly; option 1 is a cleaner independent reference (since
bias-SINEX values don't have the time-evolution uncertainty of a
live stream).  If both pass against the same truth, our correction
pipeline is validated.  If one passes and the other fails, we've
isolated where a bug lives.

### 4. Harness runner (follow-up)

`scripts/regression/run_regression.py` — orchestrates:

1. Load RINEX obs + NAV.
2. Load correction source.
3. Iterate epochs, call PPP filter exactly like the engine's
   AntPosEstThread does.
4. At the end (or periodically), extract `filt.x[:3]` ECEF.
5. Compare to truth coordinate; report per-axis error, RMS, %
   within tolerance.

Usage:
```sh
python scripts/regression/run_regression.py \
    --obs /path/to/abmf0010.20o \
    --nav /path/to/brdc0010.20p \
    --corrections-osb /path/to/WUM...BIA \
    --truth "2919785.79086,-5383744.95943,1774604.85992" \
    --tolerance-horizontal 0.005 \
    --tolerance-vertical 0.01 \
    --profile l5
```

Exit 0 on pass, non-zero on fail; stdout carries the metrics.

CI-friendly once scaffolded.

## Dependencies on main engine

The regression harness shouldn't need to start the full engine
(threading, serial ports, TICC, DO servo).  It just needs:

- `PPPFilter` (from `solve_ppp.py`)
- `MelbourneWubbenaTracker`, `NarrowLaneResolver` (from `ppp_ar.py`)
- `SvStateTracker` + monitors (from `peppar_fix/`)
- `BroadcastEphemeris` + `SSRState` (from `broadcast_eph.py`,
  `ssr_corrections.py`)

Scope: a ~200-line "regression-mode" wrapper that threads obs +
corrections + ephemeris through these classes epoch by epoch.
Most of the logic already exists inside `AntPosEstThread.run()`;
the wrapper extracts the core loop.

## Ordering

Small PRs, shippable independently:

1. **This PR** (`regression-harness`): RINEX obs reader + tests.
   Foundation; unblocks everything else.
2. **Next**: broadcast NAV reader + tests.
3. **Next**: OSB Bias-SINEX reader OR CNES SSR replay (pick one
   to start; the other is a follow-up).
4. **Last**: harness runner; end-to-end pass gate against ABMF
   2020 DOY 001.

Each step testable in isolation (unit tests against synthetic and
integration tests against the real PRIDE-bundled dataset when
`PRIDE_DATA_DIR` env var is set).

## What this enables

Once the harness exists:

- Every structural PR runs regression before deploy.  Tighter
  false-fix thresholds, Path A architectural change, any new
  LAMBDA tuning — all get a pass/fail gate independent of lab.
- Distinguishes "code bug" from "lab-specific issue".
- Validates our implementation against an external reference.

And importantly: if we *pass* ABMF 2020 but *fail* our lab, we
know the problem is lab-environmental (multipath, correction
source age, antenna quality) rather than code.  That's a
diagnostic tool we don't have today.
