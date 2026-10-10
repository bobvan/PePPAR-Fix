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

### Message loss refuted; the count rate is 1.44/s and unexplained

Measured on NewHat with the per-unit-time method, deltas unwrapped mod 65,536
(`count` is U2 and wrapped in the long log):

| Window | Span | Delivered | count rate | late |
|---|---|---|---|---|
| 10-09 15:06Z → 10-10 03:16Z | 43,007 s | 99.49% | 1.441/s | 50.3% |
| from 10-10 03:16Z | 1,214 s | 100% | 1.432/s | 0% |

**Message loss is refuted as the late regime** — 0.5% missing against 50% late.
The late messages are delayed, not substituted for lost predecessors.  That
retires the hypothesis in the previous section, in one line, from data already
on disk.

What remains unexplained is **1.44/s on a 1 Hz input, in both regimes** —
neither rising-only (1.0) nor both-edges (2.0).  Read as an edge rate it means
EXTINT sees ~0.44 extra counted transitions per second.

#### The counter/reporter rate mismatch is expected, not evidence against extras

BBGresearch note the extras are far more frequent than the strays (~16× by our
arithmetic: 0.441/s vs 1166 events / 43,007 s = 0.027/s) and read that as "most
extras don't produce one".  But **a counter sees every event while TM2 reports
at most one rising and one falling edge per message**, so count rate ≫ stray
rate is the predicted behaviour of the pair, not a problem for the coupling
picture.  The mismatch carries no information either way.

#### The real constraint is phase concentration

What *does* constrain it: if extras occurred at random phases, then whenever an
extra happened to be the last rising edge of its second, the reported residual
would land at that random phase — so reported residuals would be **scattered
across the second**.  They are not; they are bimodal at ≈0 and +100 ms.

So either the extras are phase-concentrated (a deterministic aggressor, which
is what the TP1 story predicts — see the falling-edge section above), or **they
are not EXTINT edges at all** and `count` is incrementing on something internal.

A phase histogram of *all* reported residuals, not just the classified ones,
separates those: scattered ⇒ random coupling; clean bimodal ⇒ deterministic
aggressor or a non-edge counter.

#### Test whether `count` is an edge counter before buying 0.44 extra edges

Cheaper than a scope and it decides whether a scope would even be measuring the
right thing: **change the input rate.**  Take the PEROUT to 2 Hz.

- count rate → ~2.88/s (scales with input) ⇒ `count` tracks edges, and the
  0.44/s excess is real extra transitions.
- count rate stays ~1.44/s ⇒ `count` is counting something internal, the
  "extras" are not edges, and the whole 0.44/s line of inquiry is void.

#### Do not read 44.1% ≈ 50.3% as an identity

0.441 extras/s is a 44.1% per-second probability, close to the 50.3% late
fraction.  **That resemblance is not evidence.**  One message earlier in this
same investigation we matched a 37.8% implied loss rate against the same 46–51%
late fraction and it was refuted outright by the delivery measurement.  Two
aggregate rates being close is the weakest form of agreement available.

The test that would make it more than coincidence is **per-second pairing**:
build the 2×2 contingency table of (count advanced above the normal step) ×
(message was late), second by second.  Aggregate rates matching is compatible
with the two phenomena being independent; a per-second correlation is not.
Same rule as everywhere else here — compare each event to its neighbours, never
to a pooled statistic of a population you already know is bimodal.

### The 1.44/s extras are EXTINT input ringing — already characterized here

NewHat's remaining measurements (10-09 15:06Z → 10-10 03:16Z, consecutive-TOW
pairs) resolve this:

- **Per-second 2×2**: late +1 12,016 / +2 9,523 (P(+2) 44.2%); on-time +1
  11,780 / +2 9,248 (44.0%).  **Independent** — so the 44.1%-vs-50.3%
  resemblance warned about above was pure coincidence, as predicted.
- **All 42,788 reported residuals are within ±1 µs**, with nothing between
  1 µs and 1 ms.  The extras never surface at a random phase.

Extra counted transitions that are phase-concentrated at the real edge is the
signature of **a second threshold crossing on one physical transition** — and
that is characterized in this repo already, on PiFace
(`scripts/peppar_fix/extint_reader.py:41`,
`docs/piface-shorttau-glitches-2026-05-29.md`, analyzer
`scripts/analyze_extint_ringing.py`): the EXTINT input is overdriven, the edge
rings, and when the ring dips below logic-low and re-crosses the receiver sees a
second edge — **+180–400 ns on 24.8% of edges**.

NewHat: extras within ±1 µs on **44.1%** of seconds.  Same signature, same
order of rate.

**±1 µs is too coarse to see it.**  A 180–400 ns second mode lives entirely
inside ±1 µs, so that bound — which reads like it rules out a distinct extra
edge — could not have detected the mechanism that best fits the count rate.
The test is a **ns-resolution histogram of the residuals inside the ±1 µs
band**, predicting: bimodal; primary mode at the fixed input-network delay;
secondary mode a few hundred ns later; secondary fraction ≈ 44%, matching the
count excess.  Unimodal at ns resolution ⇒ ringing is out and `count` is not
counting edges, which is then the argument for the 2 Hz test.

#### Why this outranks the stray question

TM2 reports the later edge, and a store that keeps the freshest sample lets it
**trump** the real one.  The EKF reads the DO PPS as late → concludes the DO
slowed → cranks the actuator frequency **up**.  On PiFace that produced
measured DO excursions of **29 µs on chA, and 308 µs with EXTINT as the sole
observer** — a real clock error, not a counter cosmetic.  Any consumer using
EXTINT as a servo input with last-write-wins semantics has that exposure at the
observed extra-edge rate.  Mitigation here is `ExtintLateEdgeFilter`
(`reject_ns=50.0, window=16, warmup=4, max_consecutive_rejects=8`).

**Cause does not transfer, only mechanism.**  PiFace's driver is a TADD-2 Mini
whose PIC defaults to 5 V, overdriving a 3V3 input.  An i210 SDP0 is a 3.3 V
output, so overdrive is probably not NewHat's cause — but impedance mismatch,
an unterminated line, cable length, or a slow rise time all produce a second
crossing too.  This makes the queued scope *more* useful, with a specific
target: ringing on the rising transition, not a second signal at +100 ms.

#### Two phenomena, separately docketed

That window contains **no strays at all** — nothing between 1 µs and 1 ms in
42,788 residuals.  **A window with zero strays cannot discriminate stray
mechanisms**, so this result is evidence about extras only and says nothing
either way about the +100 ms strays.  Keep them apart:

| | rate | phase | status |
|---|---|---|---|
| extras | ~44% of seconds | at the real edge (≲1 µs) | ringing, characterized |
| strays | ~0.027/s, in-gate only | +100 ms | open; TP1 falling edge predicted |

Different rates, different phases, and now probably different causes.
