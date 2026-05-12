# Slip Detection & Surgical Handling — design rev 2

**Status**: design rev 2, 2026-05-12. (Rev 1 paralleled the receiver's
own logic with our own per-SV continuity check; rev 2 just reads the
receiver's verdict.)

**Motivation**: clkPoC3 day0511night had 11 catastrophic-reject cascades
in 14 hours; PiFace (same antenna, same SSR stream, F9T receiver) had 0.
Investigation revealed:

1. The F10T's own NAV2 solution stays rock-steady through every cascade.
2. So the F10T already has internal SV-exclusion machinery that rejects
   the bad observations from its nav solution.
3. We can *read* its verdict directly via UBX-NAV-SIG instead of
   building our own parallel per-SV continuity check.

The cycle/code slip dichotomy collapses: the receiver doesn't care
about either label — it tracks each signal's `prUsed` / `crUsed` /
`doUsed` per epoch based on its internal RAIM and signal-quality logic.
We follow.

## Empirical evidence — F10T NAV2 unaffected by 21 ms chip slip

Cascade #6 on clkPoC3 day0511night (22:36 CDT, median |PR| = 6,295,637 m
= exactly 21,488 L1 chips = 21 ms):

```
22:36:17  [NAV2 1600] lat=<ARP> alt=<ARP_h+0.6m> hAcc=0.740m sv=20 pDOP=1.07
22:36:18  [CATASTROPHIC_REJECT] median |PR|=6,295,636.8m  (consecutive: 1)
22:36:27  [NAV2 1610] lat=<ARP> alt=<ARP_h+0.5m> hAcc=0.744m sv=20 pDOP=1.07
            ← unchanged through the cascade
```

Δlat between the two NAV2 reports ≈ 1 µdeg (~11 cm).  pDOP stable at
1.07, hAcc stable at 0.74 m, sv count stable at 20.  The F10T's
internal solver is unaware of any problem because it's already
excluded the offending signal(s).

Equivalent stability holds across all 11 cascades.  We do not have a
single case where F10T's NAV2 was destabilized by the chip slip.

## Plan

Replace the binary "30 rejects → exit 5" with a graduated response
that *starts from the receiver's verdict*:

```
L0  NAV-SIG-based exclusion          NEW    skip obs where receiver says prUsed=0
L2  per-SV mute (defensive)          NEW    catch cases where receiver re-admits too early
L3  mode-classified cat-reject       NEW    if a cascade still happens, classify and contain
L4  state-flush                      NEW    reset GNSS state, preserve clock + actuator
L5  re-bootstrap (exit-5)            keep   last resort
```

L1 (persistence escalator from rev 1) is dropped — the receiver's
internal logic handles persistence implicitly.

### L0: NAV-SIG consumer

UBX-NAV-SIG (msg class 0x01, ID 0x43) emits per-signal status every
nav epoch:

| Field        | Meaning                                                            |
|--------------|--------------------------------------------------------------------|
| `prUsed`     | Pseudorange used in nav solution (bit 0 of `flags`)                |
| `crUsed`     | Carrier-range (phase) used (bit 1)                                 |
| `doUsed`     | Doppler used (bit 2)                                               |
| `prSmoothed` | Code smoothing applied (bit 3)                                     |
| `prCorrUsed` | Corrections applied to PR (bit 4)                                  |
| `crCorrUsed` | Corrections applied to carrier (bit 5)                             |
| `doCorrUsed` | Corrections applied to doppler (bit 6)                             |
| `health`     | 0=unknown, 1=healthy, 2=unhealthy (bits 8-9 of `flags`)            |
| `cno`        | dBHz                                                               |
| `prRes`      | PR residual in nav solution (m, 0.1m resolution, signed)           |
| `ionoModel`  | Iono correction model used                                         |
| `qualityInd` | Signal tracking quality (0-7)                                      |

Available on both ZED-F9T-20B (TIM 2.25) and NEO-F10T-00B-01 per
u-blox interface descriptions.

**Engine integration**:

```python
# In the obs admission path (PPP filter + AntPosEst):
sig_status = nav_sig_store.get(sv, signal_band, max_age_s=2.0)
if sig_status is None:
    # Receiver hasn't reported this signal recently; safer to skip
    skip(reason="nav_sig_stale")
elif not sig_status.pr_used:
    skip(reason="nav_sig_pr_excluded")
elif sig_status.health == "unhealthy":
    skip(reason="nav_sig_unhealthy")
else:
    use(obs)
```

A new `Nav2SignalStore` mirrors `Nav2PositionStore` — receives parsed
NAV-SIG payloads from the serial reader, stores a `(sv, sig_id) →
SigStatus` map indexed by latest receive monotonic timestamp.

**Composition with our existing exclusions**:

`prUsed=0` is necessary-but-not-sufficient.  We still apply our own
masks for things the receiver doesn't know about:

- Missing phase bias (`PB_GAP_DROP` — our SSR-mount-coverage gap)
- Elevation mask (below 10°)
- ZTD-suspect SVs (when ZTD state is volatile)
- AR-side ambiguity health (per the AntPosEst state machine)

Composition: `use_sv = receiver_allows AND we_allow`.

### L2: per-SV mute (defensive)

If the receiver re-admits an SV (`prUsed: 0 → 1`) sooner than our own
ambiguity-recovery wants, hold off using it until our state catches up.

Lean on the existing `SvAmbState.WAITING → FLOATING` machinery: when a
SV transitions `prUsed 0 → 1`, treat it as `arc_gap` re-acquisition
(force the full WAITING → FLOATING → CONVERGING ladder, don't shortcut).

### L3: mode-classified cat-reject

If L0+L2 doesn't catch a failure (e.g. the receiver itself has a state
bug, or its `prUsed` flag lags reality), cat-reject still fires.
At consecutive ≥ 5, classify the last 5-10 rejected magnitudes:

```python
def classify(recent_magnitudes_m):
    mean = statistics.mean(recent_magnitudes_m)
    sd   = statistics.stdev(recent_magnitudes_m)
    cv   = sd / mean if mean > 0 else float('inf')
    chips = mean / 293.05
    if cv < 0.05 and mean > 100 and abs(chips - round(chips)) < 0.5:
        return "STATIONARY_CHIPSLIP"
    if cv > 0.20:
        return "DRIFTING_OUTLIER"
    return "UNKNOWN"
```

For STATIONARY_CHIPSLIP: HOLD state, don't increment consecutive
counter, re-engage when median drops below threshold for ≥5 clean
epochs.  Timeout 120 s to L4.

For DRIFTING_OUTLIER: exclude worst-PR-contributor SV (largest
|residual / σ|), re-check median.

For UNKNOWN: behave like current code (consecutive=30 → L4).

### L4: state-flush (without restart)

Flush all per-SV ambiguity state; preserve clock state, actuator,
subsystems.  ~30 LOC.  Avoids the 6-second exit-5 overhead.

### L5: re-bootstrap (current exit-5)

Unchanged.  Fires only after L4 timeout.

## Implementation footprint (rev 2)

| Component | Location | LOC | Notes |
|-----------|----------|-----|-------|
| NAV-SIG message subscribe + parser | `scripts/peppar_fix/receiver.py` + pyubx2 | 20 | Add to REQUIRED_MESSAGES; ack via CFG-MSG |
| `Nav2SignalStore` | new `scripts/peppar_fix/nav_sig_store.py` | 60 | Mirror Nav2PositionStore; thread-safe map |
| Engine consumer (obs admission) | `scripts/peppar_fix_engine.py` admission path | 30 | Skip when prUsed=0 / stale / unhealthy |
| L2 mute integration | existing SV state machine | 20 | Trigger WAITING→FLOATING on prUsed 0→1 |
| L3 mode classifier | `scripts/solve_ppp.py` cat-reject path | 50 | + recent-magnitudes ring buffer |
| L4 state-flush | `scripts/peppar_fix_engine.py` | 30 |   |
| Bonus 10-LOC win | cat-reject log line | 10 | Print chips + ms equivalents |
| Tests (synthetic injectors + replay) | scripts/peppar_fix/test_*.py | 150 |   |

Total: ~220 LOC + ~150 LOC tests = 4-5h focused work.

## Validation phases

**Phase A — logger only, no behavior change.**

Subscribe to NAV-SIG, parse it, log a summary every epoch:
```
[NAV-SIG 12345] G07: prUsed=1 health=1 cno=44.0 prRes=+0.3m
                E19: prUsed=0 health=1 cno=39.0 prRes=+412.7m  ← receiver excluded
                ...
```

Run on the CURRENT engines (PiFace + clkPoC3 still running day0511 data
into day0512).  No exclusion behavior change yet.  Capture:
- Distribution of `prUsed=0` events per SV.
- Correlation between `prUsed=0` and our cat-reject events.
- Whether the F10T's chip-slip cascades have `prUsed=0` set on the
  cascading SVs.

If the correlation is strong (≥80% of our cat-rejected epochs have the
cascading SV at `prUsed=0`), we have empirical proof.

**Phase B — NAV-SIG-based exclusion, after F10T swap to MadHat.**

Switch the engine to skip obs where the receiver says `prUsed=0`.
Validation overnight on MadHat (F10T host).  Targets:
- Exit-5 events: 0 expected
- L3 mode-classified cat-rejects: ≤ 1 / night (only for receiver-side
  state bugs or NAV-SIG lag)
- TICC chA TDEV(1s) on MadHat: should approach F9T-class values
  (PiFace ~0.4 ns) since the bad data is now filtered out at the
  obs-admission boundary.

If Phase A shows weak correlation (< 50% of cat-rejects have prUsed=0),
that's surprising and reveals something about F10T NAV-SIG behavior
we don't yet understand — fall back to the rev-1 per-SV continuity
check approach, treat the receiver verdict as advisory not authoritative.

## Open questions for review

1. **NAV-SIG cadence on F10T**: u-blox default is 1 Hz.  Is that fast
   enough?  RAWX is 1 Hz too, so the timescales match.  But if NAV-SIG
   lags by 1 epoch, we'd be excluding based on slightly-stale info.
   Need to measure empirically.

2. **belt-and-suspenders question**: do we also want the rev-1 PR-rate
   continuity check (predict ρ_k from ρ_{k-1} + range-rate, flag |residual| > 50 m)?
   Pros: defends against any case where NAV-SIG lags or has a state bug.
   Cons: complexity, potential false positives during real maneuvers.
   Lean: defer to Phase B results; if NAV-SIG-only catches everything,
   skip the continuity check.

3. **L4 state-flush scope**: just ambiguities, or also dt_rx / ZTD?
   Default proposal: just ambiguities (preserves the most-trustworthy
   state).  Flipping ZTD or dt_rx loses information we don't want to lose.

4. **What if NAV-SIG isn't being emitted?**  Fail-safe behavior:
   - If NAV-SIG has been stale for >5s across all signals: assume
     receiver is degraded, fall back to *not* requiring `prUsed=1`
     (i.e., behave like current code without NAV-SIG dependency).
     Log a warning.

## Why this matters

You wrote: "we are trying to parallel what it's doing internally".
Exactly.  The receiver vendor has spent years on per-signal trust
logic and exposes the result via a standard message.  Reading their
answer is cheaper, more robust, and free of our own bias toward what
"normal" looks like.  Our remaining responsibility is everything they
don't know about: phase-bias mount coverage, AR readiness, our own
ZTD stability monitor, the elevation mask we tune for our environment.

Rev-1 framed this as building our own slip detector.  Rev-2 inverts:
*use the receiver's verdict as primary input, layer our own
domain-specific exclusions on top.*
