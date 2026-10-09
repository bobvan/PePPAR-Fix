# TIM-TM2 message latency is BIMODAL — and it invalidated two analyses

**2026-10-08.  Measured on archived PiFace/F9T EXTINT captures after
BBGresearch retracted a −900 ms reading of the same effect.**

## The finding

Per-message TM2 latency — `host_wall − (edge instant from the message's own
`wn`/`towMsR`/`towSubMsR`)` — has **two regimes about one second apart**, and
our captures sit roughly half in each:

| capture | on-time (<500 ms) | late (≥500 ms) |
|---|---|---|
| `extint.csv` (7,215 rows) | n=4,056, med **84 ms** | n=3,159, med **1086 ms** |
| `extint.csv` (10,796) | n=5,764, med 83 ms | n=5,032, med 1080 ms |
| `piface-default-extint.csv` (10,793) | n=7,029, med 74 ms | n=3,764, med 1097 ms |
| `piface-no-ticc-extint.csv` (10,138) | n=5,034, med 83 ms | n=5,104, med 1078 ms |

BBGresearch measured the same two regimes on an independent ZED-F9T
(`TIM 2.20`, PROTVER 29.20) — ~40 ms and ~1,040 ms, with **544 switches**
between them in one log.  So this is a receiver-family property, not one
host's fault.

## What it did to the +100 ms stray analysis

A rare sample reads `phase = normal + exactly 100.000000 ms`, replacing that
second's normal sample.  **Relative to its immediate neighbours, every one of
our 18 strays is a LATE-regime event whose latency is ~100 ms SHORT:**

```
   stray_lat      prev      next  stray-prev  neighbours
       996.9    1100.9    1087.1      -103.9  LATE both
      1004.7    1106.6    1104.7      -101.9  LATE both
       ... 18 of 18, every one "LATE both", stray-prev in [-110, -90]
```

**18 of 18.** BBGresearch predicted exactly this before I ran it.

So the stray's TOW is **+100 ms relative to its neighbours** — which is what
the raw phase said all along.  The original observation was right.

## The error, and it was made twice in one analysis

I compared stray latency against a **pooled per-file median** of normal
samples and got +885…+916 ms, reporting it as independent confirmation of a
−900 ms TOW error.  It was not.  The pooled median lands in **whichever mode
is more populous**, so the comparison measures the mode ratio, not the stray.

Same error, second instance, in the same analysis: I flagged
`piface-no-ticc-extint.csv` as a possible "sustained fault" because its
*normal* samples pooled to 1,057 ms.  That file simply has late (5,104)
marginally outnumbering on-time (5,034), so its pooled median fell in the late
mode.  Nothing special about the capture at all.

**The general form, which we already have written down twice:** comparing a
single event against a pooled statistic of a **bimodal** population.  The
pooled statistic describes neither mode.  See
[[feedback_ask_what_the_number_could_not_have_produced]] — a pooled median of
a 50/50 bimodal distribution *could not have produced* a meaningful per-event
delta — and the sibling rule about comparisons whose arms differ in something
other than the variable under test.

**The fix is the one BBGresearch used: compare each event to its immediate
neighbours, never to a pooled summary.**

## Consequence for OUR Arm 3 — the off-by-one-edge hazard, in a new place

Derived from the measured latency, **not observed in our output** — recorded so
someone measures it rather than assumes either way.

`ExtintReader.consume_latest()` returns `(phase_ns, acc_est)` and **discards
`recv_mono`**, using it only for a freshness check.  The phase itself is the
*receiver's* timestamp of the edge, so **a delivery delay does not corrupt the
measurement.**  That part is safe.

Two things are not:

**1. `max_age_s = 2.0` has ~1 s of headroom, not 2.**  A late-regime message
arrives ~1.04 s after its edge, so it passes a 2 s window with under a second
to spare.  Nobody chose 2.0 knowing the latency was bimodal with a ~1 s mode —
another parameter nobody argued about
([[feedback_audit_the_parameters_you_did_not_argue_about]]).  If the late mode
ever lengthens, or the engine's poll loop adds delay, Arm 3 samples start being
dropped as stale, silently.

**2. In the late regime we are probably feeding the EKF the PREVIOUS second's
edge.**  The engine consumes "latest" once per epoch
(`peppar_fix_engine.py:10186`).  On-time (~40–80 ms) that is the current
edge; late (~1.04 s) it is one epoch old.  **And the regime switches —
~2.2/hour on our hosts, ~24/hour on BBGresearch's — so Arm 3's observation
alternates between current and one-epoch-stale, unannounced.**

That is precisely the hazard CLAUDE.md already warns about for qErr matching:
*"Off-by-one edge makes TDEV worse than raw PPS (3.3 ns vs 2.1 ns — confirmed
2026-04-11)."*  Same error, different arm, and this one has no gate watching
for it.  Magnitude is `DO drift × 1 s` — sub-ns to a few ns at ppb-level
drift, which is small but not negligible against a 1–2 ns excursion budget.

**How to check it cheaply:** `consume_latest()` already has `recv_mono` in
hand.  Returning it, or logging `now_mono − recv_mono` per consumed sample,
makes the regime visible at the point of use — and correlating that against
Arm 3's chi² gate events would say whether the stale samples are already being
rejected (in which case we are quietly running without Arm 3 half the time,
which is its own finding) or accepted (in which case they are biasing x[2]).

## What is now open

- **The bimodality itself is unexplained** and is the bigger finding.  Two
  regimes ~1 s apart with hundreds of transitions, on two independent F9Ts.
- `CFG-RATE-MEAS = 100 ms` is **excluded** — BBGresearch read NewHat's RAM
  layer: `CFG-RATE-MEAS 1000`, `CFG-RATE-NAV 1`, `CFG-RATE-TIMEREF 0 (UTC)`.
- The next discriminator is **`count` continuity across an on↔late switch**:
  does the receiver think it emitted an extra edge, or re-reported one?
  BBGresearch began logging raw flags, `count`, `wnR` and `towMsR` per edge
  from 2026-10-08 19:07Z.
- Our own `flags` column was a constant zero until `e2c1e45`, and our `wn`
  column mixes 10-bit-rolled and full week numbers — so our archive cannot
  contribute flag or absolute-time evidence, only neighbour-relative latency.
