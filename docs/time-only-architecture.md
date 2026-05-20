# Time-only architecture — `--no-antposest`

## Summary

`--no-antposest` is an opt-in engine flag that **skips the position
filter entirely** and delegates the antenna reference point (ARP) to
[`peppar-survey`](peppar-survey-install.md).  The engine still does its
time-transfer job at full precision; it just doesn't try to refine the
position online.

In time-only mode:

| Component | Status |
|---|---|
| `FixedPosFilter` (time-side, mode=pinned) | **Active** — estimates `dt_rx + ZTD` at the pinned position, drives PHC |
| Phase-1 PPPFilter bootstrap | **Refused** — engine exits 1 if no seed available |
| `AntPosEstThread` (position filter + AR machinery) | **Not spawned** |
| `AntPosEstWatchdog` (running-mean ARP refinement) | Inert (no `ape_thread` → no updates) |
| Position blend (`--position-blend-source`) | Forced to `none`; `--pin-position` forced on |
| NAV2 store + watchdog (gross-move detector at 10 m) | **Active** |
| RINEX writer | **Active** — captures observations for offline re-survey |
| `peppar-survey` (OPUS / PRIDE / RTKLIB / CORS) | **Load-bearing** for ARP acquisition |

## Why

`peppar-fix`'s mission is time transfer.  The position filter
(`PPPFilter`, AR machinery, anchoring state machine) was always a means
to an end: get an accurate enough ARP so `FixedPosFilter` can compute
`dt_rx + ZTD` correctly.  If `peppar-survey` delivers a survey-class
pin offline (OPUS-Static σ ≈ 12 mm, PRIDE-PPP-AR sub-cm, RTKLIB via
`surveyRtklibBackend-main`), the runtime position filter is just
surface area for known bugs:

- biased-equilibrium traps (day0422 + day0519 confirmed)
- false-fix lock-in (`--ar-mode full` pathology, observed 2026-05-20)
- ZTD/clock/ambiguity coupling under disturbance (10-round bisect)
- ~30 % of engine complexity that doesn't serve time output

The time-side `FixedPosFilter` has been silently correct throughout
every bias-hunt round — `pos_sigma` stays at 0.005 m on the seed,
`dt_rx + ZTD` outputs are unaffected by AntPosEst's drift.

This mode is the architectural conclusion: **let `peppar-survey` own
position, let the engine own time.**

## What it costs

1. **No sub-cm runtime ARP refinement.**  You get whatever
   `peppar-survey` produced — typically ~12 mm from OPUS-Static.  You
   can't tighten that during the engine run.  Honestly, you couldn't
   tighten OPUS-Static σ = 12 mm with float PPP anyway (the historic
   AntPosEst running mean was meter-class on our lab), so this isn't a
   regression in practice.

2. **NAV2 10 m threshold for ARP-move detection.**  Sub-meter antenna
   shifts (mast wobble, ground heave) won't trigger.  NAV2's
   1.5–4 m systematic bias on our receivers means anything tighter
   than 10 m generates false positives anyway, so this is the natural
   limit.

3. **No cm-class real-time position output.**  For applications where
   `peppar-fix` would be a positioning node (moving platform, real-time
   surveying), this mode is the wrong choice.  Out of scope.

## Startup dependency change

Time-only mode reframes `peppar-survey` from "nice diagnostic" to
**load-bearing infrastructure**.  Fresh deployment becomes:

1. Install `peppar-fix`
2. Install `peppar-survey` (`scripts/install_peppar_survey.sh`)
3. Run a survey to produce `state/positions/<uid>.survey.toml` (or
   provide `arp_label` pointing at `timelab/antennas.json`)
4. Start the engine with `--no-antposest`

The default (without `--no-antposest`) still works as before: NAV2
seed → Phase-1 bootstrap → AntPosEst → running-mean refinement.  Time-
only mode is explicit opt-in.

## Seed sources accepted

The engine refuses to bootstrap in time-only mode and instead exits
with code 1 if none of these is available:

| Source | Priority | Typical use |
|---|---|---|
| `--known-pos` operator CLI | 1 | one-off runs |
| `state/positions/<uid>.survey.toml` | 2 (σ < trust threshold) | normal lab path — written by `peppar-survey` |
| `state/positions/<uid>.ppp.toml` | 2 (σ ≤ trust threshold) | from prior engine convergence (now unused by default in time-only mode) |
| `timelab/antennas.json[arp_label]` | 2 | lab-shared antenna database |

The same trust + mount_sn gates as the default mode apply.  If the
selected seed has σ > the trust threshold (`_TRUSTED_POSITION_SIGMA_M`
in `peppar_fix_engine.py`), the engine logs and falls through to the
LS-validation path — which is also refused under `--no-antposest`.

## Operational example

A representative lab host wanting time-only behavior:

```bash
# 1. One-time survey (offline)
./scripts/peppar-survey --pride --receiver <uid> \
    --rinex-dir data/rinex --output state/positions/<uid>.survey.toml

# 2. Run the engine, time-only
./scripts/peppar-fix --no-antposest \
    --systems gps,gal --nav-sig-gate \
    --servo /dev/ptp_i226 --pps-pin 1 \
    --log-out data/engine.log
```

The engine reads the `.survey.toml` at startup, pins the position,
spins up `FixedPosFilter` + NAV2 watchdog + RINEX writer, and starts
PHC discipline.  No AntPosEst messages appear in the log.  No NL_DIAG
events.  No FALSE_FIX / IF_STEP / ANCHORING transitions.

## What's unchanged

- PHC bootstrap / DO characterization / servo loop
- Clock state machine (`DOFreqEst`)
- Servo glide-slope, dead-zone, gain scheduling
- Watchdog actor (NAV2-driven slew/step at 10 m)
- TICC / EXTTS / qErr correlation
- RINEX writer (PR #42 fixes still in effect)
- Wrapper-respawn on exit 5

## Validation plan

The unit tests in `tests/test_no_antposest.py` cover:

1. Flag parses and defaults to False
2. When set, derived defaults force `pin_position = True` and
   `position_blend_source = 'none'`
3. Pre-bootstrap gate refuses to proceed without a seed
4. Engine `--help` advertises the flag (catches accidental rename /
   removal)

Lab validation worth doing before declaring this production-ready for
any host:

- Start an engine with `--no-antposest` + a valid `.survey.toml` seed.
  Confirm no `AntPosEst*` messages in the log; `FixedPosFilter`
  emits `[FIXEDPOS_ZTD]` and `[FIXEDPOS_RESID]` normally.
- Compare 1-hour TDEV on TICC chA between `--no-antposest` and the
  default `--ar-mode wl` baseline.  Should be statistically
  indistinguishable (clock side is unaffected).
- Synthetic ARP move (e.g., temporarily replace the seed in
  `state/positions/<uid>.survey.toml` by 15 m and restart):
  `WatchdogActor` should trip on NAV2 disagreement.
- Engine refuses to start when given an empty receiver state
  directory + no `arp_label` + no `--known-pos`.

## Open follow-ons

- `WatchdogActor`'s slew/step paths consult `ape_thread` for the
  refined ARP target.  In time-only mode, slew/step is effectively
  disabled (only step at 10 m via NAV2 disagreement remains).  This
  is documented behavior, not a bug — but if a future deployment
  wants sub-meter ARP-move tracking without AntPosEst, a separate
  pathway would need to be designed.
- `peppar-mon` may need a tweak to handle missing AntPosEst data
  cleanly.  Today it logs a zombie-SV warning when `n=` is missing;
  in time-only mode no `[AntPosEst N]` lines are emitted so that
  warning may fire spuriously.  TBD whether this needs adjustment or
  if peppar-mon's existing graceful handling already covers it.

## Related

- Dayplan: `timeOnlyArchitecture-main`
- Bias hunt that motivated this: `docs/position-bias-hunt-2026-05-20.md`
- WL-only foundation (parallel architectural simplification on the AR
  side): `docs/wl-only-foundation.md`
- The companion `surveyRtklibBackend-main` extends `peppar-survey`
  with a CORS / RTK backend, broadening the seed-source menu.
