# Regression harness

End-to-end test that runs PePPAR Fix's PPP pipeline against published
RINEX datasets and checks the final ECEF position against an
independently-published truth coordinate.

## What's here

| File | Purpose |
|---|---|
| `rinex_reader.py` | RINEX 3.x OBS file → per-SV dual-frequency observation dicts |
| `rinex_nav_reader.py` | RINEX 3.x NAV file → BroadcastEphemeris records |
| `bias_sinex_reader.py` | IGS Bias-SINEX 1.00 OSB file → per-SV per-signal bias lookup |
| `run_regression.py` | Harness runner: orchestrates the above through `PPPFilter` |
| `test_*.py` | Unit + integration tests (synthetic + PRIDE-bundled) |

## Quick start

The PRIDE-PPPAR repo bundles a 24h RINEX OBS file for the IGS MGEX
station ABMF on 2020 DOY 001 plus an expected ECEF truth.  We use
that as the primary regression target.

### 1. Get PRIDE-PPPAR's example data

```sh
git clone --depth=1 https://github.com/PrideLab/PRIDE-PPPAR.git
export PRIDE_DATA_DIR=$PWD/PRIDE-PPPAR/example/data
```

This gives you:

- `2020/001/abmf0010.20o` — 24h ABMF observations, 27 MB
- `2023/{wuh20010.23o,wuh20020.23o,brdm0010.23p,brdm0020.23p}` —
  multipath scenario station + broadcast NAV

### 2. Get a broadcast NAV file for ABMF 2020 DOY 001

PRIDE doesn't bundle a NAV file for the 2020 dataset; their
`pdp3` script downloads one on-demand from CDDIS.  You can fetch
manually from any of:

- **CDDIS** (Earthdata login required): <https://cddis.nasa.gov/archive/gnss/data/daily/2020/001/20p/brdc0010.20p.gz>
- **IGN** (FTP, may be blocked by some firewalls): <ftp://igs.ign.fr/pub/igs/data/daily/2020/001/20p/brdc0010.20p.gz>
- **BKG** (HTTPS): <https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2020/001/BRDC00WRD_S_20200010000_01D_MN.rnx.gz>

Decompress; the resulting file is plain ASCII RINEX 3.x NAV.

### 3. (Optional) Get the WUM Bias-SINEX OSB file

Adds satellite-side phase + code bias corrections so the harness
can exercise the full PPP-AR-ready pipeline.  Without this the run
is float-PPP only with broadcast orbits — converges to ~1 m, not
cm.

- Wuhan WUM (PRIDE's source): `ftps://bdspride.com/wum/2020/001/`
  `WUM0MGXRAP_20200010000_01D_01D_OSB.BIA`
- CNES at IGN: `ftp://igs.ign.fr/pub/igs/products/mgex/2089/`
  `GRG0MGXFIN_20200010000_01D_01D_OSB.BIA` (week 2089 = 2020 DOY 001)

### 4. Run the regression

Float-only (broadcast orbits, no SSR), loose tolerance:

```sh
PYTHONPATH=scripts python3 scripts/regression/run_regression.py \
    --obs $PRIDE_DATA_DIR/2020/001/abmf0010.20o \
    --nav /path/to/brdc0010.20p \
    --truth "2919785.79086,-5383744.95943,1774604.85992" \
    --tolerance-m 5 \
    --max-epochs 200 \
    --profile l5
```

Expected: converges to within a few meters of truth after a few
hundred epochs (30 s × 200 = ~1.7 hr of float convergence).

With Bias-SINEX OSB and longer runtime:

```sh
PYTHONPATH=scripts python3 scripts/regression/run_regression.py \
    --obs $PRIDE_DATA_DIR/2020/001/abmf0010.20o \
    --nav /path/to/brdc0010.20p \
    --bia /path/to/WUM0MGXRAP_20200010000_01D_01D_OSB.BIA \
    --truth "2919785.79086,-5383744.95943,1774604.85992" \
    --tolerance-m 0.05 \
    --profile l5
```

Expected (once SSR-bias application is wired into the runner —
follow-up PR): cm-level static fix matching the IGS-published
coordinate.

## Run unit tests

```sh
PYTHONPATH=scripts python3 -m unittest discover scripts/regression -v
```

Optional integration tests:

```sh
# RINEX OBS / NAV integration
PRIDE_DATA_DIR=/path/to/PRIDE-PPPAR/example/data \
  PYTHONPATH=scripts python3 -m unittest discover scripts/regression -v

# Real Bias-SINEX file
REGRESSION_BIA=/path/to/file.BIA \
  PYTHONPATH=scripts python3 -m unittest regression.test_bias_sinex_reader
```

## Architecture

See `docs/regression-harness-plan.md`.

## Known limitations (current state)

- `run_regression.py` does **float-PPP only** — no MW tracker, no
  LAMBDA, no SvAmbState monitors.  Intended as the foundational
  validation that the position-computation pipeline (filter +
  broadcast-ephemeris orbits + RINEX obs ingest) produces a
  reasonable answer.  Adding AR is a follow-up.
- Phase-bias and code-bias application from the Bias-SINEX file
  is not yet wired into the runner (the parser reads them; the
  runner doesn't apply them yet).  Follow-up PR will integrate.
- Pass-gate target of 5 mm horizontal / 1 cm vertical is the
  long-term aspiration; current runner with broadcast-only orbits
  expects single-meter accuracy.

## Pass gate goals (in order of difficulty)

1. **Now**: float-PPP with broadcast orbits → < 5 m of truth
2. **Next**: float-PPP with broadcast + SSR biases → < 1 m
3. **Goal**: PPP-AR (LAMBDA + state machine) → < 5 mm horizontal,
   < 1 cm vertical.  Same target IGS uses for static MGEX
   solutions.
