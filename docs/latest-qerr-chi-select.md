# latestQErrChiSelect — let the fusion filter judge qErr, drop the matching machinery

**Status:** design / backlog (dayplan `latestQErrChiSelect`).
**Owner:** charlie. **Origin:** Bob, 2026-06-20.
**Relationship:** simplifies the qErr path that `--routed-qerr-arm` / `--router-qvir`
+ `qVIR` currently guard; see [`qerr-correlation.md`](qerr-correlation.md).

## The problem with the status quo

qErr↔PPS correlation has been a **perennial** pain. The TIM-TP qErr message
must be matched to the *same PPS edge* the TICC measured, via CLOCK_MONOTONIC
read-timestamp matching with an expected-offset window (`docs/qerr-correlation.md`,
`docs/stream-timescale-correlation.md`). Off-by-one-edge matching makes TDEV
*worse* than raw, so we built **qVIR** (`Δvar(raw)/Δvar(raw+qErr) > 1.5`) as a
gate to catch broken correlation before it poisons a run. That whole stack —
queueing, expected-offset windows, qVIR gating — exists to answer one question:
*"is this qErr message correct for this edge right now?"*

But qVIR predates the **fusion filter**. We now have an EKF that already judges
every measurement's consistency per-epoch via its normalized innovation (χ²).
We're doing correlation bookkeeping *outside* the filter to decide what the
filter is perfectly capable of deciding *inside*.

## The idea

Drop the matching/queue/qVIR machinery on the qErr path. Per epoch, form the
**latest-qErr-corrected** TICC candidate using whatever the most recent TIM-TP
qErr message was (no edge matching, no window), and let the EKF's **χ²
consistency test select** between it and the raw TICC candidate:

- qErr is *correct* for this edge → corrected candidate has a small innovation →
  selected (effectively trusted, low residual).
- qErr is stale / off-by-one / from a CPU-scheduling hiccup → corrected
  candidate has a large innovation → **rejected**, fall back to raw.

The filter's own innovation test does continuously, per-edge, what qVIR tried to
do with a windowed variance ratio — and it needs no host-specific tuning.

## Two things that must be right (or it backfires)

### 1. Do NOT feed raw AND corrected as two independent arms

This is the "two arms measuring the same thing" trap. Raw and corrected differ
*only* by the qErr term — they share the entire TICC reading, so they are ~100%
correlated. Feeding both as independent measurements **double-counts** the shared
reading: the EKF covariance collapses too far → overconfident → sluggish to real
drift (the opposite of robust). **Feed exactly one per epoch** — the χ²-selected
candidate. One measurement in, no double-count. (This is already how the v1
router behaves: it picks one of ext/internal/raw and feeds that single value.)

### 2. A fixed R does NOT "trust less" on its own

A vanilla EKF weights by the **R you give it**; a fixed-R corrected arm applies
full weight even to a garbage corrected value. "The filter just trusts less"
only happens through an **innovation-driven** mechanism — gate/weight the
corrected candidate by its own normalized innovation. That is the active
ingredient; without it, a stale qErr is trusted at full weight. So this is
*not* "feed corrected with a fixed R and hope" — it's "select/weight by χ²."

## Implementation — a small change to the existing v1 router

`--routed-qerr-arm` (v1, χ²) in `peppar_fix/do_freq_est.py` already does per-edge
χ² selection between `ext` (external qErr), `internal` (qerr(x0) model), and
`raw`, feeding one. The change:

1. Supply the **latest** TIM-TP qErr as the `ext` candidate's correction —
   **no edge matching, no expected-offset window, no queue**. Just "most recent
   message received."
2. Keep the χ² selection that already exists. (Optionally retire the `internal`
   candidate or keep it as a third option — it costs nothing.)
3. Retire the qVIR gate and the qErr matching/queue code from the qErr path once
   this is validated. (qVIR remains useful as an *offline diagnostic* — it's how
   we'd still characterize a host's qErr quality.)
4. CLI: `--qerr-latest-chi` (default off initially) for clean A/B vs the matched
   path.

## Why it's attractive now

- **Auto-handles cross-host qErr variation** with zero per-host debugging: a host
  whose qErr doesn't correlate (for any reason — matching, firmware, or a clean
  PPS with no sawtooth to remove) simply has its corrected candidate lose the χ²
  test → raw is used → no benefit, no harm. No "why is qVIR 1.15 on this host"
  investigation needed for *operation*.
- **Removes the most fragile code on the qErr path** (the queueing + offset-window
  matching that off-by-one bugs live in).
- **Robust by construction** to startup transients and CPU-scheduling jitter (the
  cases where the latest qErr is briefly wrong): the filter just rejects it those
  epochs.

## Honest caveats / what it does NOT fix

1. **Systematic** mis-matching (always off-by-one, not noise) → the corrected
   candidate *always* loses χ² → no benefit on that host (but no harm). You trade
   "try hard to match" for "use it when it fits." Given the matching pain, likely
   a good trade — but it is a trade, not a free recovery.
2. If a host's TICC chB **isn't sawtooth-dominated** (e.g. an intrinsically clean
   F9T-20B PPS), there's no quantization to remove and selection correctly yields
   ~no benefit — not a failure of this scheme, just nothing to gain.
3. χ² selection with a too-tight R on the corrected candidate can wrongly accept a
   close-but-wrong qErr. R for the corrected candidate must still budget the qErr
   message's own ~ns-class noise; this isn't "trust corrected to sub-ns."
4. Selection (hard pick) vs soft mixture: start with hard χ² selection (matches
   the v1 router); a soft innovation-weighted blend is a possible refinement but
   adds the double-count risk back if done carelessly — defer.

## Validation plan

1. **`servo_sim` first** (deterministic, closed-loop — the right venue): inject
   correct / stale / off-by-one / dropped qErr streams and verify
   (a) χ²-selection recovers the full qErr benefit when the qErr is matchable,
   (b) it cleanly ignores a wrong qErr with **no covariance collapse** (assert
   P stays sane; no overconfidence), (c) exactly one candidate is fed per epoch
   (no double-count).
2. **Replay** the MadHat / PiFace four-arm `ticc-hwqerr` captures (which logged
   the matched-router decision) — does latest-qErr-χ² reach the same routing /
   chA-TDEV as the matched path? (MadHat qVIR 75–130 and PiFace 28–80 are the
   matchable references; both should be recovered.)
3. **Lab A/B**: `--qerr-latest-chi` vs the matched `--router-qvir` path on PiFace
   and MadHat (F9T, valid qErr); metric = detrended chA TDEV. Expect parity
   where matching already works, and graceful raw-fallback where it doesn't.

## Related

- [`qerr-correlation.md`](qerr-correlation.md) — the matching design this retires
  (and the qVIR rationale, retained as an offline diagnostic).
- [`stream-timescale-correlation.md`](stream-timescale-correlation.md) — the
  CLOCK_MONOTONIC correlation principles; this proposal moves the qErr decision
  from that layer into the EKF.
- `peppar_fix/do_freq_est.py` — `_route_ticc_arm` (v1, the χ² selector to reuse).
