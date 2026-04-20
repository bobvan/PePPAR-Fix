# State Machine Refactor Plan

**Date**: 2026-04-15, updated 2026-04-19 with pointer to per-SV state machine
**Status**: Phase 2 in progress; **per-SV states added as a separate orthogonal machine** (see `docs/sv-lifecycle-and-pfr-split.md`)

## Scope note

This document covers the **host-level** state machines for
`AntPosEst` (position) and `DOFreqEst` (clock discipline).  The
host answers "is *the receiver's* position / clock trustworthy?"
— one state per scope for the whole host.

A **per-SV integer-fix state machine** (`SvAmbState`: TRACKING →
FLOAT → WL_FIXED → NL_SHORT_FIXED → NL_LONG_FIXED, plus
SQUELCHED) is defined separately in
`docs/sv-lifecycle-and-pfr-split.md`.  That machine answers "does
*this specific satellite's* integer ambiguity contribute to the
fix set?" — one state per (system, PRN).  The two machines are
orthogonal; they interact only via the RESOLVED transition
(position-solution CONVERGING → RESOLVED requires ≥ N SVs as
long-term members in NL_LONG_FIXED).

Don't confuse them.  Routine sky motion (SVs dropping out of the
fix set, new SVs rising, short-term members promoting to long-term)
does not flip position-solution state.  Solution transitions are
for antenna-position and antenna-movement events, not for per-
satellite bookkeeping.

## Goal

Replace the ad hoc `run_bootstrap()` / `run_steady_state()` split
with explicit state machines for both AntPosEst and DOFreqEst, as
specified in `docs/architecture-vision.md`.  The engine should log
state transitions as structured events, making the system's behavior
observable from the log alone.

## Current code structure

```
run_bootstrap()        →  run_steady_state()
  PPPFilter                 FixedPosFilter + DOFreqEst EKF
  AR (MW+NL)                No AR
  Exits on convergence      Runs until duration/signal
```

Problems:
1. PPPFilter is discarded after bootstrap — no background position
   refinement, no AR in steady state
2. The boundary is a function call, not a state transition
3. No explicit state logging
4. `run_steady_state()` conflates AntPosEst's post-bootstrap
   states with DOFreqEst's tracking states

## Target structure

Two independent state machines running concurrently.  AntPosEst
gates DOFreqEst startup (DOFreqEst blocks in UNINITIALIZED until
AntPosEst reaches VERIFIED).  After that, they run independently.

### AntPosEst states

```
UNSURVEYED    No seed, no observations yet.
              → VERIFYING when receiver state has a position
              → stays here for cold start, runs PPPFilter

VERIFYING     Seed loaded from receiver state or known_pos.
              Sanity checking against live observations.
              → VERIFIED when sanity check passes (sigma < 10m)
              → UNSURVEYED if seed is clearly wrong

VERIFIED      Position accepted. DOFreqEst may start.
              → CONVERGING immediately (start background PPPFilter)

CONVERGING    Background PPPFilter running with decimated observations.
              MW+NL accumulating. Position refining.
              → RESOLVED when ≥ N SVs reach NL_LONG_FIXED (per-SV state)
                and host sigma < threshold

RESOLVED      AR-fixed antenna position at cm level.
              Phase bias to GPS < 100 ps.
              Individual SVs cycle FLOAT ↔ WL_FIXED ↔ NL_SHORT_FIXED ↔
              NL_LONG_FIXED constantly as geometry evolves — that's
              routine and does not flip solution state.
              → CONVERGING only if NL_LONG_FIXED count drops below N
              → MOVED if consensus detects displacement

MOVED         Antenna displacement detected (NAV2 consensus).
              DOFreqEst enters HOLDOVER.
              → UNSURVEYED (re-bootstrap from scratch)
```

### DOFreqEst states

```
UNINITIALIZED  Waiting for AntPosEst VERIFIED.
               No corrections applied.  clockClass: 248.

PHASE_SETTING  Stepping DO phase to GPS second boundary.
               PHC: ADJ_SETOFFSET.  DAC: TADD ARM.
               → FREQ_VERIFYING on completion

FREQ_VERIFYING Checking drift file against PPS measurement.
               Measuring DO frequency via timestamper.
               → TRACKING when base_freq determined

TRACKING       EKF + LQR servo running. Corrections applied.
               clockClass: 6 (after settling).
               → HOLDOVER on measurement loss

HOLDOVER       No usable measurements. Coast on last adjfine.
               clockClass: 7.
               → TRACKING when measurements resume
               → UNINITIALIZED if AntPosEst enters MOVED
```

## Implementation plan

### 1. State classes (`scripts/peppar_fix/states.py`, new file)

```python
class AntPosEstState(enum.Enum):
    UNSURVEYED = "unsurveyed"
    VERIFYING = "verifying"
    VERIFIED = "verified"
    CONVERGING = "converging"
    RESOLVED = "resolved"
    MOVED = "moved"

class DOFreqEstState(enum.Enum):
    UNINITIALIZED = "uninitialized"
    PHASE_SETTING = "phase_setting"
    FREQ_VERIFYING = "freq_verifying"
    TRACKING = "tracking"
    HOLDOVER = "holdover"
```

Each state machine has a `transition(new_state, reason)` method
that logs the transition and emits a structured event:

```
[STATE] AntPosEst: VERIFYING → VERIFIED (seed σ=0.04m, LS check 12m)
[STATE] DOFreqEst: UNINITIALIZED → PHASE_SETTING (AntPosEst VERIFIED)
[STATE] DOFreqEst: TRACKING → HOLDOVER (no TICC for 30s)
```

### 2. Periodic status line

Every N epochs (configurable, default 60), emit a one-line summary:

```
[STATUS] AntPosEst=CONVERGING(σ=0.12m, 6/14 WL, 0 NL)
         DOFreqEst=TRACKING(adj=+153.1ppb, err=+1.5ns, interval=3)
         SVs=28(dual=16) TICC=ok qVIR=2.8
```

This replaces the scattered `log.info` calls with a structured,
parseable summary.  Postmortem analysis can grep `[STATUS]` for a
complete timeline.

### 3. AntPosEst as persistent background thread

The key architectural change: keep the PPPFilter alive after
bootstrap, running in a background thread.

```python
class AntPosEstThread(threading.Thread):
    """Background position refinement with AR."""

    def __init__(self, ppp_filter, obs_queue, corrections,
                 beph, ssr, position_callback):
        ...

    def run(self):
        # Decimated observation loop (every 5-10 epochs)
        # MW+NL runs here
        # Calls position_callback(ecef, sigma) when improved
```

DOFreqEst's FixedPosFilter receives position updates via the
callback, blending them exponentially into its phase reference.

### 4. Engine refactor

Replace `run_bootstrap()` + `run_steady_state()` with a single
`run_engine()` that manages both state machines:

```python
def run_engine(args, ...):
    ape = AntPosEstStateMachine()
    dfe = DOFreqEstStateMachine()

    # Phase 0: receiver config, NTRIP startup
    ...

    # AntPosEst: load seed or start unsurveyed
    if receiver_state_has_position(uid):
        ape.transition(VERIFYING)
        # sanity check...
        ape.transition(VERIFIED)
    else:
        ape.transition(UNSURVEYED)
        # run PPPFilter until converged...
        ape.transition(VERIFIED)

    # Start background AntPosEst thread
    ape.transition(CONVERGING)
    ape_thread = AntPosEstThread(ppp_filter, ...)
    ape_thread.start()

    # DOFreqEst: bootstrap and track
    dfe.transition(PHASE_SETTING)
    # ... bootstrap ...
    dfe.transition(FREQ_VERIFYING)
    # ... measure frequency ...
    dfe.transition(TRACKING)

    # Main loop: DOFreqEst epoch processing
    while not stop_event.is_set():
        # ... existing servo loop ...
        pass
```

### 5. Auto-discovery fallback

When `serial` port fails, use `discover_receivers()` to find the
receiver by `unique_id` from receiver state.  This handles USB
re-enumeration (MadHat's ACM1→ACM2 issue).

### Scope estimate

| Component | Lines | Risk |
|-----------|-------|------|
| states.py (new) | ~80 | Low — just enums + logging |
| Status line | ~30 | Low — replaces existing log.info |
| AntPosEstThread | ~150 | Medium — threading + observation decimation |
| Engine restructure | ~200 changed | High — touches the main loop |
| Auto-discovery | ~30 | Low — uses existing discover_receivers() |

Total: ~500 lines touched/added.  The engine restructure is the
risky part — the main loop is deeply nested and any regression
breaks all three hosts.

### Phasing

1. **States + logging** (low risk): ✓ Done (commit dca8398).
   State enums, transition logging, periodic [STATUS] line.

2. **AntPosEstThread** (medium risk): ✓ Done.  PPPFilter kept alive
   after bootstrap as a background thread.  Steady-state loop forwards
   decimated observations (every Nth epoch).  MW+NL run in thread.
   Position callback on improvement.  State machine driven:
   CONVERGING → RESOLVED on AR fix, back on loss.  Warm start creates
   fresh PPPFilter at known position when bootstrap was skipped.

3. **Engine restructure** (high risk): replace run_bootstrap +
   run_steady_state with unified run_engine.  Should come last,
   after phases 1-2 validate the state model.

4. **Auto-discovery** (low risk): can land independently at any time.
