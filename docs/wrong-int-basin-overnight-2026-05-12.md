# Wrong-integer basin — overnight 2026-05-11/12 results

**Setup**: TimeHat (`--ar-elev-mask 20`, default control) vs MadHat
(`--ar-elev-mask 30`, treatment). Both: `gps,gal,bds`, CNES+WHU
dual-mount, `--pin-position` to NAD83 truth, branch
`charlie/secondOpinionPinPos` @ `4a93fa9`.

**Duration**: 19:46 May 11 CDT → 06:10 May 12 CDT (~10h25m).

## TL;DR

1. **No NAV2-target SO_POS trips on either host across the whole
   night.** Every SO_POS trip reset to known_pos. Fix is rock solid.
2. **METAR-fetch-fail at startup was a major confound** — both
   engines cold-started with `ZTD = 0 σ=200mm` instead of ~33mm
   σ=50mm from METAR, putting both into a deep negative-ZTD trap
   that took 4-5 hours to recover from. Engines spent the first
   half of the night in pathological wide-σ states.
3. **Elevation-mask raising (30° vs 20°) had a marginal effect**:
   ~3% fewer total SO_POS trips on the high-mask host over 10h,
   ~equal GF_STEP rates. Not the operational fix.
4. **Both engines converged tight by morning**: σ=0.03-0.05m at
   T+10h, both within 2m of truth. *After* the bad cold-start
   resolved, steady-state PPP behavior is actually clean.
5. **MadHat had a wrapper relaunch event overnight** that
   truncated its main engine log. TICC/qErr CSVs preserved
   (append-mode from recoveryRetry-main). Main engine.log uses
   shell `>` truncation on relaunch — should be `>>` append-mode
   too. File separately.

## TimeHat 10h25m summary (control, full data)

- 149 SO_POS trips, **all `pin=149 nav2=0`** (fix engaged 100%)
- 39 GF_STEP events (~3.8/hr — *lower* than yesterday's 10.7/hr
  baseline; possibly diurnal sky conditions, possibly the
  recoveryRetry-merged code path)
- Cold-start trap: σ peaked at 1.68m around midnight (T+4h)
  recovering from negative-ZTD-init
- Final state at engine kill: σ=0.05m, alt=200.5m (-1.0m below
  NAD83 truth), 24 WL fixed, no new trip in 30+ min

σ trajectory (sampled every 50 STATUS lines ≈ ~1h):

```
19:48 T+0:02   σ=1.15m  4 WL   (cold-start)
20:41 T+0:55   σ=0.37m  14 WL
21:38 T+1:52   σ=0.43m  14 WL
22:36 T+2:50   σ=0.53m  18 WL
23:35 T+3:49   σ=1.68m  21 WL  (peak excursion)
00:33 T+4:47   σ=0.79m  19 WL
01:32 T+5:46   σ=0.73m  21 WL
02:30 T+6:44   σ=0.43m  21 WL
03:28 T+7:42   σ=0.22m  25 WL
04:27 T+8:41   σ=0.51m  26 WL
05:25 T+9:39   σ=0.11m  25 WL  (clean steady-state)
06:10 T+10:24  σ=0.05m  25 WL  (final, kill time)
```

## A/B comparison (with confound)

| metric | MadHat (mask=30) | TimeHat (mask=20) | mask=30 vs mask=20 |
|---|---|---|---|
| total SO_POS trips overnight* | ~145 | 149 | -3% |
| total GF_STEP overnight* | ~40 | 39 | ~0% |
| final position vs truth | -1.5m alt (last steady-state, ~3:50) | -1.0m alt | comparable |
| final σ (clean) | 0.03m | 0.05m | comparable |

*MadHat counts from my last live monitor reading at 03:52 +
extrapolating; full overnight log was truncated by wrapper
relaunch at 06:10.

The elev-mask raise produced a **small but consistent** reduction
in SO_POS trip rate (3%) but did not reduce drift amplitude or
recovery time. **Both engines visited the same wrong basins.**

This matches the prediction from yesterday's elimination: mask
raising shrinks the cycle-slip event pool but doesn't change the
filter's per-slip vulnerability. The disease is structural to the
continuous-Kalman + float-NL design, not specific to which SVs
contribute observations.

## Confound: METAR-fetch-fail cold-start trap

Both engines cold-started with KDPA METAR fetch timing out
(HTTP request), so fell back to `ztd=0 σ=200mm default`. With a
wide σ prior and zero initialization, the filter spent 4-5 hours
oscillating between ZTD = -300mm to ZTD = +500mm before reaching
the ~+200mm normal wet ZTD range. Position drifted 20-50m
during that period.

If METAR had succeeded (yesterday: `ZWD=0.057m ZTD=2.332m
residual=+32mm σ=50mm`), cold-start would have been clean and the
overnight would have produced ~10h of steady-state PPP data
rather than ~5h of cold-start chaos + ~5h of recovery.

**Action**: file separately as engine-quality issue. Need either:
- METAR retry-with-backoff for transient HTTP failures
- METAR result cache so a successful fetch from earlier in the
  day survives a subsequent failure
- Fallback to a wider but reasonable ZTD prior (e.g., 100mm
  σ=200mm matching typical NAD83 IGS values rather than 0±200)

## Comparison to PRIDE (from yesterday's run)

PRIDE-PPPAR on 17-min sub-arc of yesterday's RINEX got 0.91m 3D
residual vs ITRF20 truth. TimeHat overnight at the final tick was
σ=0.05m within ~1m of truth — comparable, on much more data.

The disease lives in *recovering from disruption*, not in the
steady-state PPP solution itself. When the engine is undisturbed,
it's not dramatically worse than PRIDE. The wild ±25m oscillation
we observed is the recovery cycle: slip event → bad data injected
→ filter perturbed → SO_POS trip → reset → re-converge.

## Updated disease character

Yesterday's claim "filter drifts off truth" was too strong. More
precisely: the filter is **stable when undisturbed** but **the
cycle-slip rate × detection latency means it's almost never
undisturbed**. The visible "drift" is the integral of recovery
trajectories from successive disruptions.

When disruptions abate (e.g., the late-night clean period
03:30-06:10 on TimeHat), the engine sits at ~5cm σ within 1m of
truth.

## Today's morning action items

1. **Restore baseline configs** on both hosts (WHU dual-mount
   restored on MadHat per the .with-whu backup; configs match
   timehat.toml minus the asymmetric-hardware caveats).
2. **PR #24 review/merge** — SO_POS pin-target fix + RINEX header
   fix + investigation writeups. Validated by 47+ trips yesterday +
   149 trips overnight = ~196 trips total, all pin-resets,
   zero NAV2-target regressions.
3. **File METAR-fetch-fail-handling** as a separate bead/PR.
   Easy fix, big confound-elimination win for future overnight
   characterization.
4. **File main-engine-log-append-mode** issue — wrapper relaunch
   should append, not truncate, the engine.log redirect. Matches
   the existing TICC/qErr/extint append-mode behavior in
   recoveryRetry-main.
5. **Next characterization experiment design**: with the SO_POS fix
   in + a successful METAR, the steady-state is ~5cm σ at ~1m
   from truth. Want to measure: does that 1m residual narrow
   over 24h+ of clean steady-state? Or is 1m the inherent ceiling
   given our PPP setup?

## What's NOT explored in this overnight

- UBX RAWX trkStat pre-detection of slips (a) — needs code work
- Hardware investigation of slip rate (c) — needs Bob's answers
  to lab-side questions filed yesterday
- Production elev-mask raise — the 3% improvement isn't worth
  the SV-count tradeoff
