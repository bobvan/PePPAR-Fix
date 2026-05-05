# PR/CP residual-gating port: position filter → time filter

**Status**: design doc, draft 2026-05-05
**Owner**: main, with proposed help from bravo / charlie (see end)
**Related**: I-142553-main (consistency-vs-ARP framing), I-123122-main
(MadHat over-tripping)

## Problem

Last night the time filter's `PositionWatchdog` re-seeded
`FixedPosFilter` 19 times across three hosts (TimeHat 3, PiFace 6,
MadHat 10).  All re-seeds came from the same trigger: PR-residual
RMS exceeded `max(baseline×3, baseline+10 m)` for ≥10 consecutive
epochs.  Each re-seed contaminated the cross-host TICC
measurements with multi-µs bursts.

The position filter has been through this exact pattern multiple
times and accumulated three layers of fix.  The time filter inherits
none of them today.

## The pattern, in two lines

A residual indicator that mixes PR-domain and CP-domain rows is
dominated by PR-domain noise (m-scale floor) and trips on PR-domain
disturbances (multipath, code-bias drift, SSR product latency)
while the CP-domain residuals (sub-cm) still say the filter is
healthy.  Re-seeding the filter on this signal throws away cm-grade
state because of m-grade noise.

## Position-filter fix inventory

The position filter accumulated three layers of fix for this
pattern:

1. **`SettingSvDropMonitor` elevation-ceiling gate** (commit 5bffc81,
   2026-04-22; memory `project_landed_20260422_setting_sv_drop_fix`).
   PR-residual-based SV drops only fire in the *setting band*
   (18–30° elevation).  Above the ceiling, residual issues are
   `FalseFixMonitor`'s domain (stricter 2.0 m base, designed for
   wrong integers) or filter-health monitors.  Below the mask, the
   absolute low-elev trigger handles it.  The middle band is where
   real "SV is setting and the residual is degrading"
   classification happens; nowhere else.

2. **`FixSetIntegrityMonitor` IF-domain residual gate** (per
   misnomers.md "residual-domain mismatch" entry).  The integrity
   monitor's `window_rms` trip uses IF residuals — the
   ionosphere-free combination, computed in carrier-phase domain.
   Raw PR-domain noise can no longer trip it.

3. **`WlDriftMonitor` empirical reframe** (charlie's 2026-04-28
   memo `project_wl_drift_smooth_float_signal_20260428`).  Despite
   its name and original framing as a wrong-WL-integer detector,
   WL_DRIFT was empirically reacting to PR-domain noise (MW residual
   = phase − pseudorange, dominated by the PR side) while BNC's
   IF combination saw smooth float drift.  Lever: integer-history
   class adaptive thresholding (Pop A relaxed, Pop B stays sensitive)
   so PR-multipath spikes on stable SVs don't demote them.

The pattern across all three: **route fault detection through
CP-domain residuals where possible; when PR-domain is the only
signal, gate it on context (elevation band, integer-history class)
so it doesn't trip on the noise floor.**

## Time-filter vulnerability map

`scripts/peppar_fix/watchdog.py:PositionWatchdog`:

```python
def update(self, residuals_rms, n_used):
    ...
    limit = max(self._baseline_rms * 3.0,
                self._baseline_rms + self.threshold_m)
    if residuals_rms > limit:
        self._bad_count += 1
        if self._bad_count >= self.alarm_count:
            self._alarmed = True
```

Caller (engine `run_steady_state`):

```python
n_used, resid, n_td = filt.update(observations, ...)
resid_rms = float(np.sqrt(np.mean(resid ** 2)))
watchdog.update(resid_rms, n_used)
```

`resid` is the post-fit residual vector that **concatenates** PR
rows (`SIGMA_P_IF` ≈ 1 m) and TD-CP rows (`σ_td` ≈ 0.3 m
weighted, but residuals usually sub-cm).  RMS over this mixed
vector is dominated by PR.  Last night's trips show the watchdog
tripping at residual ≈ 11 m (= baseline+10 m) while the engine
keeps logging healthy `WL fixed: …` lines from the AntPosEst PPPFilter
on the same host — the PPP-AR side sees a fine sky.

`FixedPosFilter._first_epoch_seed` (post-cold-start clock seed
from PR median): vulnerable to PR multipath, but only on the
*first* epoch and not subject to repeated tripping.  Out of scope
for this port.

`DOFreqEst` (servo): receives raw TICC chA-chB + PPP `dt_rx` from
PPPFilter.  TICC chA-chB is sub-ns measurement, not subject to PR
domain.  The PPP `dt_rx` it consumes is filtered through
PPPFilter's existing PR/CP gating, so this path is already
protected.  Out of scope.

The single point of leak in the time filter is `PositionWatchdog`'s
mixed-row RMS.

## Mapping: which fixes apply

| Position-filter fix | Time-filter port |
|---------------------|------------------|
| `SettingSvDropMonitor` setting-band gate | n/a — time filter doesn't drop SVs by per-SV residual |
| `FixSetIntegrityMonitor` IF-domain gate | **direct port** — replace `resid_rms` with TD-CP-row RMS in the watchdog |
| `WlDriftMonitor` integer-history adaptive | n/a — time filter has no per-SV WL state |

Net: one targeted change, plus a separate diagnostic for PR-domain
disturbances so we still see them.

## Proposed implementation

**Engine change** (`scripts/peppar_fix_engine.py:run_steady_state`):

The `filt.update()` already returns `n_pr` and `n_td` counts
separately (line 1212 in `solve_ppp.py`).  Plumb the residual
*splits* alongside, then feed PR-row RMS and TD-CP-row RMS to the
watchdog as separate signals.  Trip on TD-CP-row RMS exceeding the
limit; log a `[PR_DISTURBANCE]` WARNING when PR-row RMS exceeds
limit but TD-CP is fine (so we still see the multipath/SSR/code-bias
events for postmortem).

```python
# inside FixedPosFilter.update — already computes residuals per-row
# expose them via the return tuple:
return n_pr, post_resid_pr, n_td, post_resid_td  # was: n_pr, post_resid, n_td

# in the engine:
n_pr, resid_pr, n_td, resid_td = filt.update(...)
resid_pr_rms  = float(np.sqrt(np.mean(resid_pr  ** 2))) if len(resid_pr)  else 0.0
resid_td_rms  = float(np.sqrt(np.mean(resid_td  ** 2))) if len(resid_td)  else 0.0
watchdog.update(resid_td_rms, n_used)        # CP-domain witness drives the trip
if resid_pr_rms > resid_pr_disturbance_threshold:
    log.warning("[PR_DISTURBANCE] resid_pr_rms=%.1f m  resid_td_rms=%.3f m "
                "(PR multipath/code-bias; not tripping watchdog)",
                resid_pr_rms, resid_td_rms)
```

**Watchdog change** (`scripts/peppar_fix/watchdog.py`): rename
internal field `threshold_m` to `threshold_td_m` for clarity (the
threshold is now in TD-CP-residual domain).  Default tightens
substantially since TD-CP noise floor is sub-cm — try 0.1 m for
the additive component, see how it lands.  The base × 3 multiplier
stays for relative robustness.

**No filter math changes.**  The filter still runs PR + TD-CP rows
exactly as it does today; only the *trip indicator* changes.

## Relationship to other in-flight work

- **TD-CP clock-estimation design (task #65)**: complementary, not
  redundant.  TD-CP design is about **what the filter measures**
  (replace absolute CP+ambiguities with TD-CP for clock).  This
  port is about **how the watchdog detects faults** (gate trips on
  TD-CP residual not PR residual).  This port can ship before the
  TD-CP design is finalised, and benefits even if we never do the
  larger redesign.

- **MadHat-tripping investigation (I-123122-main)**: this port is
  the most likely fix.  Validation directly answers I-123122's
  open question.

- **Trip-reframing (I-142553-main)**: this port is a prerequisite.
  The reframing splits trips into ARP-movement vs internal-
  consistency; the internal-consistency category needs CP-vs-PR
  separation before the reframing can land cleanly.

## Validation plan

1. **Replay test** — re-run the engine on last night's RAWX
   captures (one per host) with a debug flag that logs PR-row
   RMS and TD-CP-row RMS at every epoch.  Map those onto the 19
   actual re-seed events: at how many of the 19 was TD-CP RMS
   *also* high?
2. **Live test on MadHat** — restart MadHat with the new code,
   keep TimeHat and PiFace on the current code as control.
   Overnight: expect MadHat re-seeds drop from ~10 to TimeHat /
   PiFace level (3-6).  Cross-host TICC bursts disappear.
3. **PR_DISTURBANCE log volume** — should be non-zero (we know
   multipath happens) but not catastrophic.  If we see hundreds
   per hour, the threshold needs tightening.

Success criteria:
- MadHat re-seeds reach TimeHat/PiFace level
- TICC #4 + TICC #5 1-second TDEV drops from 5+ µs (re-seed
  contaminated) to the derived-PiFace-TimeHat level (~1 ns 1s,
  ~5 ns 1hr)

## What I'm asking from bravo / charlie

This work decomposes cleanly:

- **bravo: build the replay harness** + run it on last night's
  three captures.  You wrote the FixedPosFilter ZTD seed plumbing;
  you know the filter's residual columns.  The replay harness
  reads RAWX from a single host's session, runs `FixedPosFilter`
  forward with current code, and records `(epoch, resid_pr_rms,
  resid_td_rms, watchdog_alarmed)` for each epoch.  Output: a CSV
  + a 1-page summary of how many re-seeds would have stayed vs
  gone away under the new gate.  Estimated 2-3 hours.

- **charlie: own the watchdog change**.  You have the strongest
  context on the position-filter `wl_drift` PR-vs-CP work
  (project_wl_drift_smooth_float_signal_20260428).  Update
  `scripts/peppar_fix/watchdog.py` + the engine's wiring, write
  unit tests, validate against bravo's replay output before
  pushing.  Estimated half a day.

I'll review both, then we co-deploy on MadHat for an overnight
arm.  This should land today.
