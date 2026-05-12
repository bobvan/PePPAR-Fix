# Wrong-integer basin investigation — 2026-05-11

**Status**: investigation findings + overnight plan.
**Lab**: MadHat + TimeHat on shared CHOKE1 antenna via splitter,
both pinned to surveyed truth via `--pin-position --known-pos`.
**Branch**: `charlie/secondOpinionPinPos` (fixes landed; not yet
PR'd as of this writeup).

## TL;DR

1. **Cure-worse-than-disease bug fixed** (commit `d427d4e`):
   `SECOND_OPINION_POS` watchdog was unconditionally resetting the
   filter to NAV2's biased position even when `--pin-position` was
   set. Now it resets to the surveyed pin. Validated by 47+ trips
   across both hosts overnight today — all `pin=N nav2=0`.

2. **Disease persists after the fix is in place.** Filter still
   drifts off truth, oscillating wildly (±15-25m altitude over
   minutes, ZTD inflating 6-10x baseline).

3. **Constellation/signal elimination ruled out**:
   BDS off, GPS-only, GAL-only, ar-mode full, WHU dual-mount off,
   no-phase-bias — none of these is the unique disease driver.
   Disease is independent of phase-bias source.

4. **Disease is in slip-handling**: ~10 cycle slips/hour on the
   antenna, 2-epoch GF_STEP detection latency means each slip
   injects 2 epochs of bad phase observations into the float NL
   state before the SV is flushed. Filter can't damp fast enough
   between slips. Other position solvers (PRIDE, NAV2) don't
   have this vulnerability:
   - PRIDE: batch LS, excludes slip epochs during data cleaning
   - NAV2: code-only, no integer ambiguities to corrupt
   - Ours: continuous Kalman with float NL absorbs slip energy
     for 2 epochs before recovery

5. **Frame mismatch is a separate ~1.7m contribution** —
   `--known-pos` supplied in NAD83(2011), SSR products in ITRF20.
   Not the primary disease driver, but accounts for ~1m of the
   "steady-state residual when the filter isn't actively being
   hit by slips."

## Day's experimental sequence

| time | host(s) | config | result |
|---|---|---|---|
| pre-fix | MadHat | gps,gal,bds, CNES+WHU, --pin-position | SO_POS reset filter to NAV2 (~4m bias) every few min; basin pull |
| 15:43 | MadHat + TimeHat | + fix `charlie/secondOpinionPinPos` | every SO_POS trip resets to known_pos (validated) |
| 16:42 | both | (baseline) | ~7m cross-host alt differential on same antenna; ±15-25m oscillation; 1.5/5min trip rate |
| 17:32 | MadHat | --systems gps,gal (no BDS) | same disease as baseline → BDS not driver |
| 17:42 | MadHat | --systems gal (alone) | too few SVs to converge — dead test |
| 17:46 | MadHat | --systems gps (alone) | too few SVs — dead test |
| 17:49 | MadHat | --no-ssr-phase-bias | all obs route away from position filter via Fix #2 MISS-bias gate — dead test |
| 17:54 | MadHat | --ar-mode full | LAMBDA never fires (n_screened<4 always); identical to wl mode at convergence |
| 18:02 | MadHat | CNES-only (WHU dual-mount disabled in TOML) | reduced SV count, much steadier (45 min at -1.2m below truth) before slip event hits at 19:35 → +13m drift |

PRIDE-PPPAR (pdp3 v3.2) on the captured baseline RINEX (4h21m DOY
130 file): truncated to 17m30s of usable data after auto-cleaning,
GAL-only sub-arc, no AR fix attempted (insufficient arc length),
returned 0.91m 3D residual vs ITRF20 truth on the 17-min window.

## Disease characterization

**Cycle-slip rate**: 10.7/hour measured via `[GF_STEP]` events on
both hosts' fix-branch runs, matching the 192-events-in-18h
baseline log from yesterday. Same rate independent of constellation
choice or WHU dual-mount.

**Slip magnitudes**: 0.04m to 15.8m geometry-free delta. Most are
3-10m. Each "real" slip is several L5 cycles of carrier-phase jump.

**Detection latency**: `GfStepMonitor` requires 2 sustained epochs
above threshold (default 0.040m, sustained=2). 2 epochs of bad
data per slip × 10 slips/hr = 20 bad-data epochs per hour
absorbed by the filter, mostly via float NL → bleeding into ZTD
+ position.

**Filter response signature**:
- WL count: tracks well (5-12 fixed steady-state across configs)
- NL count: always 0 (`--ar-mode wl` skips NL; `--ar-mode full`
  blocked by `n_screened<4 & p_bootstrap_ib<0.99` pre-screen)
- ZTD: monotonically inflates from ~33mm cold-start to 200+mm
  over hours (real wet ZTD changes are ~5mm/hour at most;
  this is bias absorption, not weather)
- σ: filter underestimates true error by 10-100× during basin
  excursions (claims σ=0.13m while 25m off truth)
- Cross-host: agrees with itself sometimes (filters wander
  into same attractor), disagrees by 7m+ at others

**Direction of drift**: not monotonic. Random-walk-like
oscillation around truth with growing variance. Bob's earlier
"pulling away from truth" was a sampling artifact — over the
full session, the filter has visited both above-truth and
below-truth basins, transiting through near-truth.

## What "fixes" the disease

| candidate | how | leverage |
|---|---|---|
| (a) Pre-detect slips via UBX RXM/RAWX trkStat | gate phase obs by per-SV lock-loss bit at ingest | high; structural |
| (b) Tighten GF_STEP latency: sustained=2 → 1 | 1-line config + risk tolerance for occasional FP | medium-low; band-aid |
| (c) Reduce slip rate at the antenna | hardware: RF chain, splitter, GUS, RFI | high if rate is abnormal |
| (d) Tune process noise on N_IF state | bigger Q lets ambiguity absorb without committing wrong long-term | low; doesn't address root cause |
| (e) ar-elev-mask raise (30°+) | exclude low-elev SVs where most slips occur | medium; cheap to test, reduces SVs |

Open hardware questions (asked Bob, awaiting):
1. UFO1 (pre-CHOKE1) slip rate as calibrated baseline
2. Any RFI changes in lab today
3. GUS supply / splitter / cable changes since 2026-05-05 CHOKE1
   swap-in

## Overnight 2026-05-11/12 plan

Goal: characterize whether elevation-mask raising (cheap test of
hypothesis (e)) reduces slip-induced disease amplitude.

### A/B setup

| host | config | role |
|---|---|---|
| TimeHat | baseline: gps,gal,bds, CNES+WHU, ar-mode wl, ar-elev-mask=20° (default) | control — wild oscillation baseline |
| MadHat | same as TimeHat BUT --ar-elev-mask 30 | treatment — exclude low-elev SVs where slips concentrate |

Both pinned to `--known-pos = CHOKE1 NAD83 truth`. Both on
`charlie/secondOpinionPinPos` so SO_POS resets to pin (fix
in place; no cure-worse-than-disease).

Restore MadHat config (re-enable WHU dual-mount via
`ssr_bias_mount = "OSBC00WHU1"`) for the overnight to match
TimeHat. Only `--ar-elev-mask` differs.

### Predictions

- If elevation-mask raising significantly reduces disease
  amplitude (e.g., MadHat oscillation < 30% of TimeHat's),
  low-elev SVs are the dominant slip source.
- If similar to control, slips are distributed across the sky,
  and we need (a) pre-detection (UBX RAWX trkStat) or (c)
  hardware investigation.

### Capture

- Engine logs: `data/day0511overnight-{madhat,timehat}.log`
- TICC + EXTTS + qErr CSVs (all hosts)
- RINEX (MadHat writes 30s decimated, TimeHat doesn't)

### Watch dimensions

- `[GF_STEP]` event count by host
- `[STATUS]` σ trajectory (does treatment have lower σ
  excursions?)
- ZTD time series (does treatment inflate ZTD less?)
- Cross-host altitude differential

### Decision criteria for morning

- IF MadHat slip rate is ≥30% lower than TimeHat: elevation-
  driven hypothesis confirmed; next: per-SV elev histogram of
  slip events, propose production elev-mask raise.
- IF slip rates similar: slips are NOT elevation-concentrated;
  pivot to (a) pre-detection via UBX RAWX trkStat as the
  next-day work.
- Either way: capture rates for `[SLIP_FILTERED]` (PR-only,
  carrier preserved) vs `[GF_STEP]` (carrier slip, ambiguity
  flushed) to see if the *type* of slip differs by elevation.

## Remaining open items

- PR for `charlie/secondOpinionPinPos` (commits `d427d4e` SO_POS
  pin-target fix + `ad1a74b` RINEX header for PRIDE compat).
  Both small, both validated empirically today. Should land.
- METAR-fetch-fail handling: today's gps,gal experiment cold-
  started with ZTD=0 σ=200mm because METAR fetch timed out, and
  the cold-start trajectory was much worse than usual. No retry
  or cache in the current code path. File separately.
- FixedPosFilter status thread silence on yesterday's MadHat
  baseline run (10:57 onward) — correlates with TICC CSV
  truncation at 10:57. Probably Bob's lab rewiring per his
  comment. Noted, not pursued.
- PRIDE on a 4+ hour clean fix-branch arc (vs the 17-min sub-arc
  used today). Tomorrow when overnight RINEX is in hand.
