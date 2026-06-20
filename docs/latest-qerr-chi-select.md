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

**Why pure-latest is enough (Bob, 2026-06-20):** at the 1 Hz PPS cadence,
applying the most recent qErr message to the *following* PPS edge is right the
large majority of the time — call it ~99%. Much of the existing queue/correlation
machinery was built to eke the last bit of information out of *buffered* qErr
readings during the cases where "latest" is briefly wrong: **startup** (no edge
correlated yet) and **CPU starvation** (the read thread fell behind and the
"latest" message is stale by one or more edges). Those are exactly the cases the
χ² test rejects on its own — so we don't need to *match* through them, we just
need to *not trust* the corrected candidate during them. Pure-latest + χ²
selection covers the 99% directly and degrades gracefully on the ~1%.

### qErr path scope — TICC arm only (NOT the EXTINT/EXTTS path)

This changes the **TICC-arm** qErr (the `ext` candidate in `_route_ticc_arm`,
where qErr removes the 8 ns sawtooth from a 60 ps-resolution chA−chB reading —
the qVIR-meaningful path; MadHat 75–130, PiFace 28–80). It does **not** touch the
**EXTINT/EXTTS-path** qErr. The "EXTTS qVIR = 0.00" seen on hosts (incl. MadHat
F9T-20B, 2026-06-20) is a *different path* and is expected, not broken: EXTINT/
EXTTS are themselves ~8 ns-quantized, so qErr can't reduce their variance and
qVIR≈0 there on every host. #203 neither addresses nor needs that.

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

## Implementation — change the existing v1 router, including its gate

`--routed-qerr-arm` (v1, χ²) in `peppar_fix/do_freq_est.py` (`_route_ticc_arm`)
already does per-edge selection between `ext` (external qErr), `internal`
(qerr(x0) model), and `raw`, feeding one. The change:

1. Supply the **latest** TIM-TP qErr as the `ext` candidate's correction —
   **no edge matching, no expected-offset window, no queue**. Just "most recent
   message received."

2. ⚠️ **Make `ext` acceptance COMPARATIVE, not absolute (Main's must-resolve,
   PR #203 review).** The current v1 gate is *absolute*:
   `if name == 'raw' or chi2 <= _CHI2_GATE_THRESHOLD` (~100) — it accepts `ext`
   merely for clearing 100, it does not require `ext` to *beat* `raw`. That is
   fine for the **matched** router (matching keeps `ext` almost always correct,
   so it rarely sees a mis-correlated `ext`), but it **fails for pure-latest**:
   an off-by-edge qErr is the difference of two ±4 ns sawtooth values ≈ **3.3 ns
   RMS** (up to 8 ns). At lock √S ≈ 0.8 ns on the small-`R_base` `ext` candidate
   → `chi² ≈ (3.3/0.8)² ≈ 17`, reaching ~100 only at the 8 ns worst case. So a
   mis-correlated latest-qErr **passes the absolute-100 gate most of the time** →
   `ext` accepted → poisons `z` → the exact qVIR=0 / "TDEV worse than raw"
   failure. The no-harm property therefore **requires** the gate change: `ext`
   wins only if `chi²(ext)` beats `chi²(raw)` by a margin; otherwise route `raw`.

   | candidate | innovation | R | √S | χ² | vs raw |
   |---|---|---|---|---|---|
   | ext, correct | ~0.1 ns | `R_base` | ~0.8 ns | ~0.02 | **wins** ✅ |
   | raw | ~2.3 ns (sawtooth) | `R_base + tick²/12` | ~2.4 ns | ~0.9 | — |
   | ext, wrong | ~3.3 ns | `R_base` | ~0.8 ns | ~17 | **loses → raw** ❌ |

   Without this change "reuse v1 as-is" is unsafe; *with* it, the comparative
   χ² is the whole no-harm mechanism. Also fix the `_route_ticc_arm` docstring,
   which currently overstates the absolute gate's robustness
   ("conspicuous chi² outlier" — true vs the 8 ns worst case, false at the ~3.3
   ns typical mis-correlation that lands at χ²≈17 < 100).

3. Keep the `internal` (qerr(x0)) candidate as a third option — it costs nothing
   and gives a no-PPP-free fallback.
4. Retire the qVIR gate and the qErr matching/queue code from the qErr path once
   the A/B proves parity. (qVIR remains useful as an *offline diagnostic* — it's
   how we'd still characterize a host's qErr quality.)
5. CLI: `--qerr-latest-chi` (default off initially) for clean A/B vs the matched
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
   (no double-count). **The off-by-one injection is the key test of the
   comparative gate** (item 2 in Implementation): with an absolute-100 gate the
   poisoned `ext` is accepted (χ²≈17) and TDEV degrades; with the comparative
   gate `ext` loses to raw and TDEV is unharmed. This is the *proof* of no-harm,
   not just recovery-of-benefit.
2. **Replay both halves** (Main's ask — don't only prove recovery):
   - *Recovery:* the MadHat / PiFace four-arm `ticc-hwqerr` captures (matched,
     qVIR 75–130 / 28–80) — latest-qErr-χ² should reach the same routing /
     chA-TDEV as the matched path.
   - *Graceful fallback:* a **genuinely-broken** qErr case — synthetic off-by-one
     in the sim (above) and/or a host where the TICC qErr does not correlate —
     must route to raw with **no TDEV penalty vs raw-only**. "No harm when
     matching fails" must be *demonstrated*, not assumed.
3. **Lab A/B**: `--qerr-latest-chi` vs the matched `--router-qvir` path on PiFace
   and MadHat (F9T, valid qErr); metric = detrended chA TDEV. Expect parity
   where matching already works, and graceful raw-fallback where it doesn't.

## Optional middle ground — coarse time-of-week pick (no queue)

Pure-latest gives up only the *systematically* off-by-one host (caveat 1). The
TIM-TP message carries its own edge time-of-week; a single coarse compare of
that field against the expected PPS second (no queue, no CLOCK_MONOTONIC window,
no expected-offset tuning) could pick the right edge for those hosts nearly free
— a middle ground between full-queue matching and pure-latest. Optional;
pure-latest + comparative-χ² is a fine v1, and this can be layered on later if a
systematically-off host turns up.

## Related

- [`qerr-correlation.md`](qerr-correlation.md) — the matching design this retires
  (and the qVIR rationale, retained as an offline diagnostic).
- [`stream-timescale-correlation.md`](stream-timescale-correlation.md) — the
  CLOCK_MONOTONIC correlation principles; this proposal moves the qErr decision
  from that layer into the EKF.
- `peppar_fix/do_freq_est.py` — `_route_ticc_arm` (v1, the χ² selector to reuse).
