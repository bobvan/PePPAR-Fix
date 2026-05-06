# no-gnss-pps experiment

Branch: `no-gnss-pps`. Purpose: prove (or disprove) that the engine
does not need the F9T PPS edge — neither raw EXTTS PPS, nor PPS+qErr,
nor TICC chA-chB qErr-corrected — for clock discipline. Carrier-phase
clock (PPP `dt_rx`, exposed through the existing "Carrier" source) is
the only steering input.

This is a **single-experiment branch**. It merges to `main` on success
or is abandoned on failure. No CLI flag is added; the lockout is
hardwired in `compute_error_sources` so the experiment runs from any
engine launch on this branch.

## What "success" means

Cold-boot a host with the F9T PPS cable physically disconnected from
its TICC chB and demonstrate that the host's DO PPS phase agreement
against an external GPS reference (otcBob1 PPS, on TICC #4 chB) stays
within a sanity bound (~100 ns) over a multi-hour window.

The static PiFace-DO-vs-otcBob1 offset (currently ~40 ns, observed up
to ~60 ns historically) does **not** need to be zero. The goal is for
it to be **constant** — calibration to zero is a follow-on once cable
lengths and receiver delays are characterized.

Intermediate milestones, in increasing difficulty:

1. **Hot boot.** Engine restarted on a host with PHC already
   disciplined and Carrier converged. F9T PPS still cabled, lockout
   in code. Verify Carrier wins and DO PPS stays within bound.
2. **Warm boot.** Engine restarted with a known position pin
   (CHOKE1) but no prior PHC discipline. Carrier needs to converge
   (~90 s) before the servo has a useful input. Verify the bound
   still holds after convergence.
3. **Cold boot with F9T PPS unplugged.** As above plus the F9T PPS
   cable physically removed from TICC chB. Carrier is the only
   measurement reaching the engine. Verify the bound still holds.

If milestone 3 succeeds, the branch merges to main and the deprecated
sources can then be removed entirely.

## What "failure" means

Carrier alone cannot keep DO PPS within ~100 ns of otcBob1 over a
multi-hour stationary run. Causes that would show up as failure
include (in roughly increasing severity):

- Carrier source has a slow, large calibration drift
  (carrier_tracker offset wanders > 100 ns over hours)
- Carrier sigma routinely exceeds `carrier_max_sigma` (default 50
  ns) so the source disappears from competition periodically — at
  which point the lockout produces "no useful steering input" and
  the DO drifts with its TCXO
- A mismatch (e.g. a 77 ns offset between Carrier and PPS+qErr we
  observed in 2026-05-05 overnight data) is a Carrier-side bias,
  not a PPS+qErr bias

## Mechanism (this branch's only behavioral change)

`scripts/peppar_fix/error_sources.py:compute_error_sources` was
modified so PPS, PPS+qErr, TICC, and PPS+PPP sources now return
hardwired confidence `_GNSS_PPS_SUPPRESS_NS = 1e6` instead of their
calibrated values. They stay in the source list for downstream
bookkeeping (source-change logging, scheduler ingest, status emit)
but cannot win the competition while Carrier is available.

If Carrier is unavailable (uninitialized, sigma too high, ppp_cal not
calibrated), the best-by-confidence source is suppressed at 1e6 ns —
which the scheduler treats as effectively unusable. The engine's
existing `TRACK_OUTLIER_NS` and consecutive-outlier exit paths handle
the no-steering case, same as today.

## Lab setup

Per `timelab/topology.md` change-log entry 2026-05-06:

- TICC #4 chA: PiFace DO PPS (i226 PEROUT)
- TICC #4 chB: otcBob1 PPS OUT (Timebeat OCXO, F9T-disciplined GPS
  reference, scope-cleaner than M600)
- Differential measurement (chA − chB), no TICC-timescale
  correlation needed

Other lab hosts continue to be observed via their existing TICCs.
The cross-host PPS agreement metric is the differential between two
hosts on a single shared-reference TICC — same approach as the
position cross-host metric (consensus across hosts, then deviation
per host).

## Two-dimensional analysis (parallel to position)

Per Bob's framing (2026-05-06):

| Dimension  | Position side                              | Time side                                |
|------------|--------------------------------------------|------------------------------------------|
| Stability  | AntPosEst σ                                | per-host TICC chA detrended TDEV          |
| Truth      | Surveyed ARP (UFO1, CHOKE1)                | TICC differential vs otcBob1 PPS          |

Overnight 2026-05-05 baseline (before this branch's lockout was
deployed) is captured in `data/no-gnss-pps-baseline-2026-05-05.md`
(to be written from the `/tmp/no-gnss-pps-analysis` artifacts) for
post-experiment comparison.

## Success → merge plan

If milestones 1-3 all pass, the merge to main does:

1. Remove `_GNSS_PPS_SUPPRESS_NS` and the suppressed source branches
   from `compute_error_sources` entirely (no longer needed).
2. Stop subscribing to TIM-TP, EXTTS PPS, and TICC chA-chB diff in
   the engine — those code paths are dead.
3. Remove the `qErr` correlation pipeline (FIFO matching,
   `match_pps_mono`, qVIR computation) since nothing consumes it.
4. Update CLAUDE.md and `docs/full-data-flow.md` to reflect the
   simpler architecture.
5. Update `docs/visual-stories.md` with the deprecation rationale.

## Failure → abandon plan

Branch stays unmerged. The investigation pivots to fixing Carrier
calibration / sigma behavior so PPS can eventually be removed in a
later iteration. The 2026-05-05 overnight evidence (Bug 1 hardwired
PPS+qErr confidence, Bug 2 source flip without hysteresis, Bug 3 77
ns Carrier-vs-PPS+qErr disagreement) becomes the bug list for that
follow-on.
