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

## The +100 ms stray: TM2 carries both edges, so one pulse width is in reach

Checked against pyubx2's payload definition rather than assumed — **every
TIM-TM2 message carries two complete timestamps**:

```
ch  flags{mode,run,newFallingEdge,timeBase,utc,time,newRisingEdge}  count
wnR  wnF  towMsR  towSubMsR  towMsF  towSubMsF  accEst
```

`wnR/towMsR/towSubMsR` is the last **rising** edge, `wnF/towMsF/towSubMsF` the
last **falling** edge, each with its own validity flag.  So any consumer that
reads the F fields, or reads "whichever edge is new", is wrong by **exactly one
pulse width** — not by a jitter-like amount, by a hardware constant.

This matters because the F9T's factory timepulse width is **100 ms**, which is
the stray's magnitude.  It also reproduces the regime-dependence BBGresearch
established empirically (strays require the late latency regime) with no
walking boundary of any kind: the on-time regime's 74–84 ms latency is *less*
than a 100 ms pulse, so those messages are emitted **before the falling edge
exists** and there is nothing to confuse; the late regime's ~1080 ms is after
it, so both edges are new.

**But the pulse width that matters is the width on the EXTINT pin, not the
F9T's timepulse-output default** — and on neither fleet is the F9T's own
timepulse the signal being timestamped:

- **NewHat** (BBGresearch) is fed from an i226 SDP0 PEROUT.  Our
  `ptp_device.py` documents, verified empirically and cross-checked against
  SatPulse, that **igc uses a 50% duty cycle regardless of the duty-cycle
  flag** — a requested 1 ms ON time is ignored.  At 1 Hz that is a **500 ms**
  pulse, which predicts a +500 ms stray.  Not what they see.
- **PiFace** (ours) is fed from a **TADD-2 Mini** divider (10 MHz → 1 PPS,
  `config/piface.toml`).  Its output width has never been measured here.  We
  already owe that board a scope probe for an unrelated reason — whether it
  drives 5 V into a 3V3 EXTINT — so width goes on the same probe.

So the hypothesis reduces to one read-only measurement with the verdict fixed
in advance: scope the EXTINT pin.  100 ms ⇒ live; 500 ms ⇒ dead; anything not
equal to the stray ⇒ dead.  This is worth spending *before* the 200 ms PEROUT
offset test, which needs code and moves the watermark.

**Excluded on our side regardless:** nothing in this repo reads the
falling-edge fields — `extint_reader.py` takes `wnR/towMsR/towSubMsR` only, and
a repo-wide grep for `towMsF`/`towSubMsF`/`wnF` finds no other consumer.  Our
18 strays therefore cannot be reader-side field confusion; the mechanism would
have to be the receiver populating `towMsR` from a falling edge.

### Are the two fleets' strays even one phenomenon?

Open, and currently unarguable.  Different EXTINT sources (PEROUT vs TADD-2),
different receivers, and MadHat turns out to run a **ZED-X20P**, not the F9T
that timelab's table claimed — which voided the two-host firmware comparison as
cross-model.  If the pin widths differ and the stray is 100 ms on both, that is
two causes wearing one number, and pooling the evidence would hide it.

### A note on what the ±60/+30 ns phase scan could not have shown

That scan (~120k samples, late fraction flat at 46–51%) cleanly killed the
"boundary sits at the GPS second" model.  It cannot speak to a boundary at the
receiver's own measurement epoch, which sits ~2 ms out — about 20,000× outside
the scanned span.  Falsifying one model is not support for the next by
elimination.

And the receiver-epoch model has its own problem: NewHat's RAWX `rcvTow`
fraction reads **+0.002000000 s exactly**.  Exactly-round values are scheduled
or quantised, never the residue of a free-running clock bias — so the boundary
is *stationary*, and a stationary boundary cannot produce a regime that switches
24×/hour.  The hypothesis needs `rcvTow` to be observed moving before it earns
any instrumentation.  (Our engine notes ~N.997 as the F9T norm at
`peppar_fix_engine.py:9236`, where `round()` instead of `floor()` was once a
real +1 s bug; the X20P reads 0.993.  NewHat's +0.002 is the outlier against
our own baseline — why *that* differs is the prior question to why it walks.)

### `ch` and `count`: what each branch can and cannot conclude

`ch` is recoverable retrospectively from raw TM2 frames (it is the first
payload byte), so BBGresearch can read it for the in-gate strays without new
instrumentation.  Measured on NewHat as of 03:03Z: every frame `ch=0`, all
outside the 07–15Z gate, no strays yet.

The two branches are **not symmetric**, and the asymmetry has to be registered
before the read-out or the result will be over-read:

- **stray at `ch=1`** ⇒ a second signal on a second input.  Conclusive.
- **stray at `ch=0`** ⇒ refutes a second *input*, and nothing more.  It does
  **not** refute TP1 coupling, because crosstalk onto EXTINT0 itself lands on
  `ch=0` exactly like a genuine edge.

For the `ch=0` branch the discriminator is **`count`**, which is already
logged.  A stray second shows:

- **`count` +2** ⇒ two edges were detected — the real one *and* a spurious
  extra.  The stray is an additional capture, not a mistimed one.
- **`count` +1** ⇒ one edge, reported at the wrong time.  A mistiming.

### If TP1 is the source, the stray is its FALLING edge

TP1's rising edge is aligned to the second and is therefore indistinguishable
from the real PPS edge.  Its falling edge sits at **+100 ms** — exactly the
stray.  So "TP1 reaching an EXTINT input" is specifically *TP1's falling edge
being captured*, which carries three consequences worth pre-registering:

1. **The 100 → 10 ms VALSET predicts strays at +10 ms.**  If they stay at
   exactly +100 ms after the width change, TP1 is not the source at all.
2. **Capacitive coupling differentiates**, so a fast falling transition appears
   on the victim input as a transient whose *recovery* is a rising edge — which
   is how a falling-edge event gets captured by a rising-edge timestamper, and
   why marginal coupling would fire intermittently (~50%) rather than every
   second.
3. It predicts **`count` +2**, per the branch table above: the real edge plus
   the coupled one.

Note this mechanism is distinct from the both-edges one killed above.  That one
needed TM2's own `towMsF` fields to be read (they are not, by either codebase);
this one needs no software error at all — a second physical edge arrives and is
correctly timestamped.

### Difference `count` per unit TIME, not per message

My absolute branch table above (+2 = extra capture, +1 = mistiming) does not
transfer to a 50%-duty EXTINT.  BBGresearch corrected it: on NewHat both edges
are counted, so normal steps are *already* +1 or +2, and an extra capture shows
as "one more than the normal step".  Measured on 10-09: +1 ×31,287 and
+2 ×19,005 normally, with +2 ×900 and +3 ×266 into strays.

But the relative form inherits a confound, and their own numbers expose it:

```
advance per message = (31287·1 + 19005·2) / 50292 = 1.3779
  count = rising only, no message loss        -> 1.0000
  count = both edges, 1 msg/s, no loss        -> 2.0000
  observed                                    -> 1.3779   <- neither
```

So the +1/+2 mix is not explained by edge-counting alone.  The other reading is
**message loss**: `count` is a hardware counter that does not drop events even
when the message carrying it is dropped, so a lost message makes the *next*
message's per-message delta larger.  37.8% of steps being +2 is then a ~38%
loss rate, not an edge-counting property.

The two readings are not distinguishable per-message, which is the problem: a
"+3 into strays" could be one extra edge on a normal +2, or a lost message on a
normal +2, or two lost messages.  Differencing per message conflates a property
of the *signal* with a property of the *message stream*.

**The fix removes the confound rather than modelling it.**  Edges arrive at a
hardware-determined rate — 2/s for a 50%-duty 1 Hz pulse if both edges count,
1/s if rising only.  Over any window:

```
expected count advance = (edges per second) x window_seconds
```

An extra captured edge raises that **rate**.  A lost message does not change it
at all — the counter kept counting while the message was missing.  So the
per-second count rate is immune to delivery loss, where the per-message delta is
not.  It also measures the edge rate directly, which settles "both edges or
rising only" as a by-product: ≈2.0/s ⇒ both, ≈1.0/s ⇒ rising only.

### A consequence worth testing before anything else

If +2 steps really are lost messages at ~38%, that sits close to the 46–51%
late fraction — and "late by ~1000 ms" would then have a mundane explanation:
**the message for that second was lost, and what arrives ~1 s later is the
next epoch's message.**  That would dissolve the latency bimodality into
message loss rather than a latency mechanism, and it would make the
off-by-one-edge hazard in Arm 3 (documented above) concrete rather than
hypothetical.

Two checks, both on data already logged: on a late-regime message, (a) does
`count` show the extra advance of a missed predecessor, and (b) does `towMsR`
correspond to the *previous* second rather than the current one?  Both yes ⇒
message loss, not latency.
