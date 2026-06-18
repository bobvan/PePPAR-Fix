# TICC-Drive Cleanup Plan

**Date**: 2026-04-14
**Status**: Planned for morning session

## Background

The TICC-drive path was bolted on when TICCs were measurement-only.
It has its own mode management, gain adaptation, and error source
selection — all designed for the old PI servo, all unnecessary with
DOFreqEst.  The result: clkPoC3 (TICC-drive + DAC) diverges while
TimeHat (non-TICC-drive + PHC) works fine.

## Quick fixes applied (revert during cleanup)

These are band-aids that should be removed when the full refactor
lands.  They're marked with TODO comments in the code.

1. **Force interval=1 in TICC-drive mode** (commit d1b2e81)
   - File: `scripts/peppar_fix_engine.py` ~line 3528
   - What: Replaced mode-specific scheduler intervals (5/1/variable)
     with always-1
   - Why: DOFreqEst diverges with dt=5 prediction steps
   - Revert: the full cleanup removes TICC-drive mode management
     entirely, making this moot

2. **DAC reset to center before TICC measurement** (commit 90045d1)
   - File: `scripts/peppar_fix_engine.py` ~line 2218
   - What: Creates a temporary DacActuator to write center code before
     the bootstrap TICC frequency measurement
   - Why: stale DAC setting from prior run corrupts frequency estimate
   - Revert: the full cleanup should unify the DAC lifecycle so the
     actuator is set up BEFORE the frequency measurement, not after
   - ⚠️ **noMagicCenterCode (2026-06-18)**: "reset to center" bakes in
     the magic-center-code error — midscale is the actuator's *neutral*
     command (↔ `adjfine = 0`), not a meaningful frequency reference.
     On an asymmetric DAC (PiFace) parking at midscale sits the DO far
     from GNSS.  The unified lifecycle should park at the **GNSS-matching
     code** (or last-known code), not midscale.  See
     `docs/do-characterization-architecture.md` §"Code symmetry vs pull
     symmetry".

## Full refactor plan

1. **Add TICC as a source in `compute_error_sources()`**
   - `error_sources.py`: add TICC source with confidence from
     ticc_measurement.confidence
   - TICC competes with Carrier, PPS+qErr, PPS on equal terms
   - Delete `ticc_only_error_source()`

2. **Remove TICC-drive mode management**
   - Delete `_update_ticc_tracking_mode()` and related state
   - Delete pull_in/landing/settled state machine
   - Delete mode-specific scheduler intervals
   - Delete `--ticc-drive` flag and all conditionals on it
   - TICC is just another source — if `--ticc-port` is set, TICC
     is available

3. **Unify DAC lifecycle**
   - `DacActuator.setup()` before TICC frequency measurement
   - Single actuator instance from bootstrap through servo
   - Remove the temporary DAC reset hack

4. **Unify error source selection path**
   - Delete the `if args.ticc_drive` / `else` split in _servo_epoch
   - Single `compute_error_sources()` call for all hosts
   - DOFreqEst always gets `pps_err_ticc_ns` when available (already
     the case)

5. **Clean up gain adaptation**
   - Remove PI-era gain scaling for TICC-drive (lines 3639-3652)
   - DOFreqEst has its own LQR gain — no external scaling needed
   - Keep confidence-based gain scaling for the scheduler only

Estimated scope: ~200 lines removed from engine, ~30 lines in
error_sources.py, ~10 CLI args deleted.
