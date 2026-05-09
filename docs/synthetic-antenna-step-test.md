# Synthetic 1 m antenna-step validation test

Procedure for validating the I-125649 cold→warm-float→warm-WL→hot
state machine end-to-end with a synthetic 1 m position step.  Per
Main's I-125649 refinement 3 — exercises the position filter's
ARP-motion detection AND the time filter's NAV2-blend stability
when the surveyed truth changes mid-run.

**Owner:** main runs the procedure on lab hardware.
**Reviewer:** bravo (writer of this plan + analysis script).
**Status:** ready for execution post-I-125649 merge to main.

## Hypothesis

After the cold→warm-float→warm-WL→hot transitions stabilize, a
sub-meter ARP shift should:

1. **Position filter** raises ARP-motion alarm within 1 minute
   (single-host WL σ ≈ 30 cm; 1 m step is ~25σ at 60 s averaging
   per Bob's 2026-05-09 framing).
2. **Time filter** PPS OUT preserves sub-ns TDEV through the
   step — it's pinned to the surveyed ARP, not the AntPosEst
   estimate, so a position-side basin shift doesn't leak into
   PPS phase.  PPS TDEV(1 s) should stay at the host's
   noise-floor (e.g., 0.1 ns on PiFace OCXO).
3. **Warm-WL re-converge** within ~10 minutes after the step,
   showing the position filter recovers a coherent ARP estimate
   at the new location.
4. **Cross-host TICC differential** (when 2+ hosts on the same
   antenna feed) shows the step host diverges from the unstepped
   host(s) by exactly the synthetic offset (1 m × c ≈ 3.3 ns
   if the antenna feed is the same — but for true antenna
   motion, the cross-host PPS differential should NOT shift
   because all hosts see the same antenna).  This validates
   the test mechanism: a synthetic --known-pos override is
   NOT a real antenna move, so the differential should hold.

## Test variant

The procedure as designed uses a **synthetic CLI override** via
`--known-pos`.  This injects a step into the engine's ARP belief
without physically moving the antenna.  Limitations:

- The receiver hardware doesn't see a real antenna motion, so
  the carrier-phase observations don't have the corresponding
  geometry change.  The position filter's residuals will grow
  by ~1 m on every SV — that's the synthetic step manifesting
  as residual.
- A REAL antenna motion would also shift the receiver's
  internal NAV2 fix; the synthetic override doesn't.
- This test validates the engine's RESPONSE to a believed-ARP
  shift, not a true antenna move.  For the latter, a separate
  physical test (move antenna by 1 m, re-survey) is required.

The synthetic test is the right shape for validating the engine's
state-machine response in isolation.  Physical tests come later.

## Procedure

Use a host running --ar-mode wl on a stable surveyed ARP.
Recommend MadHat or PiFace (CHOKE1 antenna feed).  Configure
the cohort with a second host (TimeHat) NOT receiving the step
for cross-host differential.

### Phase 0 — Pre-flight

```bash
# Verify the host's drift file is fresh (last hour) and the
# state/dos/<uid>.json reflects the current build.
cd ~/peppar-fix
git status                  # confirm tip
git log -3                  # confirm intended commits landed
ls -la state/dos/*.json     # confirm DAC + DO state present
```

Establish the surveyed truth ARP from `timelab/antPos.json`
(host's antenna entry, current OPUS mean) and pick a step
direction.  Recommend +1 m east in ENU.  Convert to ECEF /
LLA with `pyproj` (or `ecef_to_enu` helpers under
`scripts/peppar_fix/...`); ±10 cm error in the step
direction is fine — the test signal is 1 m.

For the procedure below, refer to the truth as `LAT_TRUE,
LON_TRUE, ALT_TRUE` and the +1 m east step as `LAT_STEP,
LON_STEP, ALT_STEP`.  Do not commit literal lat/lon — the
public repo enforces this via pre-commit hook.

### Phase 1 — Baseline (30 min, surveyed ARP, --pin-position)

```bash
./scripts/peppar-fix \
    --ticc-log data/$(date +%Y%m%d_%H%M%S)-step-baseline-ticc.csv \
    --known-pos LAT_TRUE,LON_TRUE,ALT_TRUE \
    --pin-position \
    --ar-mode wl \
    > data/$(date +%Y%m%d_%H%M%S)-step-baseline.log 2>&1 &
echo $! > /tmp/step-test-pid
sleep 1800
kill $(cat /tmp/step-test-pid)
wait
```

This is the "before-step" reference.  Expected: clean lock,
ZTD residual within ±300 mm, no ARP-motion alarms, PPS TDEV
on noise floor.

### Phase 2 — Step injected (30 min, ARP_STEP, --pin-position)

```bash
./scripts/peppar-fix \
    --ticc-log data/$(date +%Y%m%d_%H%M%S)-step-stepped-ticc.csv \
    --known-pos LAT_STEP,LON_STEP,ALT_STEP \
    --pin-position \
    --ar-mode wl \
    > data/$(date +%Y%m%d_%H%M%S)-step-stepped.log 2>&1 &
echo $! > /tmp/step-test-pid
sleep 1800
kill $(cat /tmp/step-test-pid)
wait
```

(The lat/lon above is approximate +1 m east of the surveyed
truth.  Use precise pyproj-derived coords for the actual run.)

Expected: position filter sees ~1 m residual on every SV, raises
ARP-motion alarm within 1 minute.  Time filter (pinned)
continues steering to the (now-wrong) ARP — the disciplined
PPS receives a ~3.3 ns offset until the operator notices.
This is the failure-mode validation: alarm rate vs basin
absorption.

### Phase 3 — Recovery (30 min, surveyed ARP again, --pin-position)

Same as Phase 1.  Engine restarted with --known-pos at the
true ARP.  Expected: clean recovery to baseline behavior; no
residual basin trap from the previous step.

## Observables

The combined log + TICC capture from all three phases enables
the analysis script (`scripts/replay/antenna_step_analysis.py`)
to compute:

| Observable | Phase 1 (baseline) | Phase 2 (stepped) | Phase 3 (recovery) |
|---|---|---|---|
| ARP residual mean | ≈ 0 | ≈ 1 m | ≈ 0 |
| ARP residual σ over phase | < 30 cm | > 30 cm initially | < 30 cm |
| ARP-motion alarm count | 0 | ≥ 1 (within first 60 s) | 0 |
| PPS OUT TDEV(1s) | host floor | host floor | host floor |
| PPS OUT TDEV(100s) | host floor | host floor + offset | host floor |
| WL re-converge time | n/a | < 10 min from step | n/a |
| ZTD residual range | ±300 mm | ±300 mm (pinned) | ±300 mm |
| Cross-host TICC chA-chB Δ | step host = unstepped (within ns) | step host shifted ~3.3 ns | step host = unstepped |

## Pass / Fail

PASS if:

- Phase 2 raises ≥ 1 ARP-motion alarm within the first 60 s.
- Phase 2 PPS OUT TDEV(1s) within 2× the Phase 1 baseline (the
  time filter shouldn't degrade short-term stability — pin
  isolates).
- Phase 3 returns to baseline ARP residual + alarm count
  within 5 minutes (no persistent basin from prior step).
- Cross-host TICC differential during Phase 2 shows the step
  host shifted by ~3.3 ns vs the unstepped host (the
  expected pinning-to-wrong-ARP signature).

FAIL if:

- No alarm fires in Phase 2 (motion detection broken).
- Phase 2 PPS OUT TDEV(1s) > 2× Phase 1 (pin isolation broken).
- Phase 3 ZTD residual or position-residual basin persists
  > 5 min beyond the recovery start (state contamination).
- Cross-host TICC shows Phase 1 differential ≠ Phase 3
  differential (recovery introduced a new bias).

## Expected operational interpretation

The synthetic test won't replace a real antenna move (the
hardware doesn't see geometry change), but it does validate
that:

1. The engine's ARP-motion alarm logic correctly fires on a
   1 m believed-ARP shift.  If it doesn't, the position filter's
   threshold is wrong.
2. The pinned time-filter is properly isolated from the
   AntPosEst pathologies (per I-125649 Stage 4 — the time
   filter takes its ARP from NAV2 running mean now, not
   AntPosEst).  The synthetic step doesn't perturb NAV2 fix
   directly, so PPS should be steady.
3. The transition between "alarm raised" and "operator action"
   is clean — recovery in Phase 3 confirms no state pollution.

If all three pass, the cold→warm-float→warm-WL→hot state
machine is robust to the most likely production failure mode
(operator changes --known-pos to wrong value) without
contaminating the time signal.

## Analysis script

`scripts/replay/antenna_step_analysis.py` consumes the three
phase logs + TICC chA captures and emits a per-observable
table aligned with the pass/fail criteria above.  Run as:

```bash
scripts/replay/antenna_step_analysis.py \
    --baseline-log data/<TIMESTAMP>-step-baseline.log \
    --stepped-log  data/<TIMESTAMP>-step-stepped.log \
    --recovery-log data/<TIMESTAMP>-step-recovery.log \
    --baseline-ticc data/<TIMESTAMP>-step-baseline-ticc.csv \
    --stepped-ticc  data/<TIMESTAMP>-step-stepped-ticc.csv \
    --recovery-ticc data/<TIMESTAMP>-step-recovery-ticc.csv
```

Output: per-phase summary table + verdict line + HTML for
visual inspection of the observables across phases.

## Cross-host extension

For the cross-host TICC differential, run a SECOND host
(TimeHat) on the SAME --known-pos as the step-host's Phase 1
ARP throughout — i.e., never injecting the step on the
second host.  Then:

```bash
scripts/replay/antenna_step_analysis.py ... \
    --cohort-host-log data/<TIMESTAMP>-timehat.log \
    --cohort-ticc-log data/<TIMESTAMP>-timehat-ticc.csv
```

Cross-host comparison surfaces whether the step-host's PPS
shifted relative to the unstepped host — the operationally
meaningful signal for downstream PPS consumers.

## References

- I-125649-main (parent — cold→warm-float→warm-WL→hot transitions)
- I-145846 (runtime ARP transition, option A: restart on
  σ_arp threshold crossing — the design this test exercises)
- docs/runtime-arp-transition.md (the full state-survival table)
- Bob's 2026-05-09 framing: 1 m budget for ARP-motion detection,
  margin of safety on top
