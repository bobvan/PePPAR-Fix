# Runtime ARP transition

How the engine consumes a freshly-tightened ARP from the orchestrator
without going off-the-rails or waiting for the next operator-driven
restart.

**Status**: design adopted, prototype owned by Charlie.
**Decision**: option A (engine restart on σ_arp threshold crossing).
**Scope**: this document covers only how the engine *receives* a new
ARP. Producing the ARP is owned by the daily PRIDE pipeline
(I-013307); arbitration of when an ARP becomes canonical is owned by
the orchestrator (I-013239).

## Problem

Two operational regimes have different mechanics for getting position
truth into the FixedPosFilter:

- **Cold-start regime**: `AntPosEst` PPP-AR runs free; once
  ANCHORED at sub-cm σ, position trickles into FixedPosFilter via
  the position-blend path (`engine.py:3917-3926`, α≈0.001/epoch).
  Slow but sufficient — the time filter converges to AR truth over
  ~1 000 epochs.
- **Pinned-mode regime**: `--pin-position` skips the trickle entirely
  and reads `known_ecef` from the orchestrator at engine launch.

When σ_arp tightens (daily PRIDE updates `state/arp/<uid>.json`, or
a multi-day OPUS-Static comes in, or a watchdog event invalidates and
a re-survey re-converges), the engine in pinned mode has no in-process
mechanism to pick that up — orchestrator + engine restart is the only
path today.

## Decision: option A — restart on threshold crossing

The orchestrator restarts the engine when σ_arp crosses the pin
threshold (or when the running mean ECEF moves by more than a
configurable epsilon). The engine reads the new ARP at launch and
proceeds normally.

**Why A over B (hot-swap) and C (glide-slope)** at this point in the
project:

- PRIDE post-processing runs once a day (per I-013307). σ_arp
  crossings are inherently low-cadence — a few per *month*, not per
  hour. The cost of restart-cadence is bounded.
- Engine restart is already on the well-trod path. Servo state
  loss is bounded to a few seconds; PPS goes briefly to clockClass
  248 then re-locks. The post-restart `--pin-position` cold-start
  is well-validated by tonight's overnight (the I-145915 + I-024942
  + bake-in story).
- B (hot-swap) has subtle correctness pitfalls — the new ARP is
  sub-cm from the old, so geometry change per-SV is sub-ns, but
  the FixedPosFilter's `prev_geo` cache holds *previous-epoch*
  values that no longer match the new `pos`. Time-differenced
  carrier-phase residuals (`solve_ppp.py:1268-1282`) would briefly
  misbehave. The fix is to invalidate `prev_geo` on hot-swap, which
  forces a TD-CP-row warm-up window — equivalent in user-visible
  effect to a restart anyway.
- C (glide-slope) is overkill at daily cadence and reintroduces the
  exact mechanism (slow blend) that I-013342 just removed for
  pin-mode.

A defaults to operationally simpler for the scope where it's
correct, and we can graduate to B if measured impact justifies it.

## Triggers

The orchestrator restarts the engine when one of:

1. **σ_arp threshold first-cross** — `running_mean.sigma_3d_m`
   transitions from ≥ `arp_pin_threshold` to < `arp_pin_threshold`
   (default 0.10 m per I-013239). First time only — subsequent
   tightening does not re-trigger.
2. **Mean drift** — `running_mean.ecef_m` differs from the
   currently-pinned `known_ecef` by more than a configurable epsilon
   (default 50 mm 3D). This is rare in well-behaved deployments;
   typical day-to-day variation is sub-cm.
3. **Mount-id increment** (operator action) — antenna moved; new
   `mount_id` in `state/arp/<uid>/history.jsonl`. Engine must
   restart even before σ_arp re-converges, because the previously-
   pinned ECEF is now wrong by the move distance.

(2) and (3) are explicit operator-or-cron-driven events; (1) is the
common case of σ_arp passing the threshold for the first time.

## What survives a restart

Most of what matters is *recoverable*, not *preserved*:

| State | Survives? | Reasoning |
|---|---|---|
| Surveyed ARP (`state/arp/<uid>.json`) | Yes | On disk, source of truth. |
| `state/receivers/<uid>.json` (frequency) | Yes | DOFreqEst seed. |
| Drift file | Yes | PHC bootstrap continues. |
| `prev_geo` cache (per-SV TD-CP) | No | Rebuilt over ~1 epoch. |
| Filter clock state | No | Re-seeded from PR; ~10 s convergence. |
| ZTD residual state | Mostly | METAR seed (I-024942) lands within
                                 ±50 mm of physical truth at epoch 1. |
| AR (PPP-AR ambiguity set) | No | Cold-start re-resolution; ~5-15 min. |
| MW wide-lane tracker state | No | ~30-300 s to FLOAT, longer to FIXED. |
| TICC capture | Yes (separate process) | TICC is independent of engine. |
| ptp4l clockClass | Briefly degraded | Goes to 248 during restart, re-locks. |
| Cross-host TICC measurements | Brief gap | One missing 1 s sample. |

The PPS-OUT signal is briefly absent or unsynchronized during the
restart window. For the disciplined-clock use case this is bounded
by the clockClass-248 announcement so downstream consumers know to
hold over.

## What would re-open option B

Switch to in-process hot-swap if any of these become measurable
problems:

- σ_arp crossing cadence rises beyond ~weekly (e.g. when continuous
  multi-host PRIDE is wired up and meaningful epsilon-drift triggers
  fire daily).
- The PPS gap during restart causes downstream alarms or
  measurement-noise contamination beyond what the I-145915 PR/CP
  gate work + TICC chA capture can absorb.
- A second, finer-grained ARP-update path lands (e.g. real-time
  consensus from peer hosts) where second-cadence updates are
  expected.

The hot-swap implementation is a small extension of existing code:
re-seed `filt.pos`, invalidate `filt.prev_geo`, leave clock + ZTD +
ISB states alone. Estimated 20-40 LOC + a TD-CP-warmup-window test.

## What would re-open option C

Glide-slope only re-opens if the disciplined-PPS noise floor becomes
sensitive to sub-mm ECEF steps. We're nowhere near that floor today
(target 3 ns ≈ 1 m ECEF). It's a non-event for the foreseeable.

## Engine-side surface area

Charlie's mtime-watcher prototype (per the dayplan allocation)
restarts the engine via a clean shutdown signal when `state/arp/<uid>.json`
mtime advances. The engine has nothing new to do for option A —
it already reads `--surveyed-position-from <uid>` (per I-013342) at
launch.

The only engine-side polish worth landing:

1. Log on startup the σ_arp value and `mtime` of the state file
   that gated the launch, so post-hoc analysis can correlate
   restart events with ARP updates.
2. On `clean-shutdown` signal: emit a final
   `[FIXEDPOS_RESID_SUMMARY]` (and the analogous summary lines from
   other I-145915-class shadow gates) so the new-engine restart
   doesn't lose the prior session's bake-in numbers.

Both are ≤ 10 LOC; can ship alongside Charlie's mtime-watcher in
the same PR or as a separate cosmetic follow-up.

## Test plan

A is mostly tested by existing engine restart machinery. Two new
checks:

- **Unit**: orchestrator decides restart iff (1) σ_arp crosses, or
  (2) mean ECEF drifts > epsilon, or (3) mount_id increments. A
  small state machine is the obvious test target.
- **Integration**: simulate a daily PRIDE solution that crosses the
  σ_arp threshold; orchestrator triggers engine restart; engine
  launches with new `known_ecef`; FixedPosFilter cold-starts to
  ANCHORED again within the standard convergence window.

The integration test belongs in Charlie's mtime-watcher prototype
since that's where the restart trigger lives.

## References

- I-145846 (this item, owner: bravo for design / charlie for
  prototype)
- I-013239 (orchestrator state file + watchdog, owner: main)
- I-013307 (daily PRIDE pipeline, owner: main)
- I-013342 (engine `--surveyed-position` pinned mode, owner: main)
- I-200645 (PRIDE solution parser + ARP running-mean accumulator,
  owner: bravo, landed)
- I-024942 (METAR-seeded ZTD prior, owner: bravo, landed)
