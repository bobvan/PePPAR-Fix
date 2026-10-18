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
walking boundary of any kind: our PiFace on-time latency of 74–84 ms is *less*
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

### Ringing refuted on NewHat — and their clean EXTINT is the control we lacked

Measured, same 12 h window, 50 ns bins:

```
-100..-50 ns      63   0.147%
 -50..0       20,569  48.072%
   0..+50     22,114  51.683%
 +50..+100        42   0.098%
+100 ns..+1 µs     0   0.000%
```

Unimodal, 99.755% within ±50 ns, **zero beyond ±100 ns**.  Nothing in the
predicted +180–400 ns band, so **ringing is refuted on NewHat** and bbgld's
EXTINT has no late-edge servo exposure.

That negative result is worth more to us than the hypothesis was.  We had no
clean-EXTINT control: every EXTINT phase distribution we have characterized is
PiFace's, which carries the defect.  NewHat's i210-driven pin at ±50 ns
retro-supports something we had only inferred — PiFace's +180–400 ns late edges
on 24.8% of edges are specific to the **TADD-2 Mini's 5 V overdrive into a 3V3
input**, not a generic property of EXTINT inputs, of TIM-TM2, or of our
measurement chain.  Cite NewHat's histogram as the control when that question
comes up again.

### The +1/+2 structure rules out both readings of `count`

Per-second deltas are **exactly +1 or +2 — never 0, never 3**: 23,796 vs
18,771 (55.9% / 44.1%), mean 1.4410/s.  That excludes both candidates:

- **rising-only** ⇒ every second +1.  Observed +2 on 44%.
- **both-edges hardware counter** on a 1 Hz 50%-duty pulse ⇒ every second +2.
  Observed +1 on 56%.

So `count` increments on something occurring one or two times per second, which
on a 500 ms pulse points at *both edges, but only those captured within that
message's window*.

**Next test needs no hardware change, no 2 Hz, no watermark disturbance:
correlate delta=+2 against the `newFallingEdge` flag.**  If +2 ⟺
`newFallingEdge` set, the extras are simply falling edges, the 0.44/s puzzle
dissolves with no extra physical edges, and the 2 Hz test buys nothing.
Second confirmation in the same frames: `towMsF`/`towSubMsF` should place the
falling edge at ≈+500 ms, the 50% duty point.

### Method note: twice now the discriminator was already on disk

**Neither codebase reads `towMsF`** (verified repo-wide here; confirmed on
theirs), exactly as neither logged `ch`.  Both times the field that
discriminates the live hypotheses was present in every frame and undecoded
while we reasoned about what the data *could* be.  Decode the fields the
message already carries before modelling what it might mean.

## RESOLVED: the extras are a sub-tick re-crossing, and they bias the clock

BBGresearch decoded `towMsF`/`towSubMsF` on 2,342 raw frames (NewHat, from
03:03Z 2026-10-10).  The falling-edge timestamp separates the two populations
cleanly:

| `count` step | fall − rise | n | reading |
|---|---|---|---|
| +1 | **−500.000 ms** | 1,336 | normal — the *previous* pulse's falling edge |
| +2 | **−8 ns** (min −8, max −7) | 1,012 | the line rose, dipped and rose again **inside one 8 ns tick**, and TM2 reported the LAST rise |

8 ns is the F9T's timestamper tick (125 MHz), so the two events are one tick
apart — the minimum resolvable separation.  **`newFallingEdge` was set on every
frame (0xcd), so no flag distinguishes them; only the timestamps do.**

### It is not cosmetic — it biases the disciplined clock

Dense log, same 12 h window:

| population | mean phase | sd | n |
|---|---|---|---|
| clean (+1) | **−7.05 ns** | 13.4 | 23,796 |
| glitched (+2) | **+9.22 ns** | 14.2 | 18,771 |

The two sit **16.3 ns apart**, and bbgld steers the *mixture* to zero — so
NewHat's PHC actually sits ~7 ns early on clean seconds, and much of the ~12 ns
EXTINT scatter is just the two populations superposed rather than noise.

> ⚠ **A retracted claim, kept because the error is instructive.**  I presented
> `−p·Δ = −7.17 ns` against the measured clean-second mean of `−7.05 ns` as an
> independent quantitative confirmation of the mechanism, "predicted, not
> fitted".  **It is neither independent nor a prediction** — BBGresearch caught
> it.  The mixture mean is `p·m₂ + (1−p)·m₁` *by construction, from the same
> samples*, so `m₁ = mix − p·Δ` is a rearrangement of that definition.  The
> residual is exactly the mixture mean, identically:
>
> ```
> (−p·Δ) − m₁ = −mix = −0.1247 ns      <- the "0.12 ns agreement", algebraically
> ```
>
> So the check restates only that bbgld holds **what it sees** near zero.  True,
> and worth stating as a servo fact, but it tests nothing about the mechanism.
> This is precisely the failure mode recorded in
> [[feedback_audit_the_parameters_you_did_not_argue_about]] — agreement between
> two quantities coupled by construction is not corroboration — committed here
> one message after warning them about the weaker aggregate-rate version of it.

**What actually carries the mechanism** is per-sample and deterministic, not
aggregate:

- `fall − rise = −8 ns` on **every** count+2 frame and `−500.000 ms` on
  **every** count+1 frame, across 2,342 frames.  No overlap, no distribution.
- the 16.3 ns phase split **keyed to that same per-frame flag**, so the two
  populations are identified by a signal independent of their phases.
- the TP1 null.

The servo fact, stated without overreach: the mixture mean is +0.125 ns, so
bbgld steers the *mixture* to zero.  Given the split is real — which the
per-frame evidence above establishes — the clean population therefore sits
~7 ns early.  The bias follows from the split being real, not from the algebra.

### Why our filter would not have caught it, and our archive cannot test it

Same class as the PiFace TADD-2 hazard, at **16 ns instead of 180–400 ns** —
and that difference is exactly what makes it dangerous:
`ExtintLateEdgeFilter`'s default `reject_ns=50` blocks the TADD-2 edges and
**passes a 16 ns split unchallenged**.  Below anything a ±100 ns look resolves.

We also never logged TM2's falling-edge fields, so **no capture in our archive
can answer whether PiFace has a sub-tick sibling** — the same gap as `ch`, for
the third time in one investigation.  Added 2026-10-09: `wn_f`, `tow_ms_f`,
`tow_sub_ms_f_ns` and the derived `fall_minus_rise_ns` now go to
`--extint-log`, with `fall_minus_rise_ns` returning `None` across a GPS-week
rollover rather than guessing.

**Open on our side — BLOCKED as of 2026-10-10.**  PiPuss was accidentally
overvolted (extent undiagnosed) and PiFace is powered down, so neither host can
supply the capture.  Nothing below is actionable until that changes, and the
PiPuss half additionally needs a full DO re-characterization first — its stored
steering describes pre-overvolt hardware (see the banner in
`config/pipuss.toml`).  Needs a fresh PiFace + PiPuss capture:

1. Does PiFace show a `fall_minus_rise_ns ≈ −8 ns` population, and at what `p`?
2. Does the clean/glitched mean phase split, and by how much?
3. Does `(p_PiFace − p_PiPuss) · Δ` account for any of the measured
   **p95 |Δ| = 8.04 ns** cross-host disagreement, which
   `two-clock-agreement-forward-model.md` currently attributes entirely to
   σ_ref noise?  A static per-host offset and white noise are not
   distinguishable by `sync_agreement_model.py backfit`, so that doc now
   carries a caveat.

Credit: mechanism and measurement BBGresearch; the falling-edge pointer came
from this side after `towMsF` turned out to be the field neither codebase read.

## Aftermath: thermal gating, the TP1 confound, and the first independent test

### Thermal evidence (BBGresearch, NewHat, 5 mornings)

| KDPA in-gate minimum | strays |
|---|---|
| 8.3 °C | yes |
| 8.9 °C | yes |
| 9.4 °C | yes |
| 11.1 °C | none |
| 14.4 °C | none |

10-10 was a **valid** zero-stray day: the late latency regime was present
throughout the gate (08Z 100%, 11Z 99%, 14Z 99%), so the absence is not an
instrumentation gap.  The guarded 11:00Z reset stood down untriggered.

Their statistic is right and correctly hedged: under the null that strays are
independent of temperature, P(the stray-free days are exactly the warmest) =
`1/C(5,3) = 1/10`.  Suggestive, not a finding.

**Ranks throw away the informative part.**  The cold cluster spans 1.1 °C while
the gap to the next day is **1.7 °C** — wider than the cluster itself — so the
data are consistent with a *threshold* somewhere in (9.4, 11.1) °C rather than
a gradual response.  That converts the observation into a forecast, which is
worth more than the p-value: the next sub-9.5 °C morning should have strays and
any >11 °C morning should not.  Budgeting from `1/C(n,k)`, two more
correctly-predicted mornings reach **p ≈ 0.03** (n=7, k=4); four more reach
**p ≈ 0.008**.

### TP1 is void on this day for the right reason, but the cells matter

10-10 changed **two** variables at once relative to the stray mornings — TP1
width (100 → 10 ms) *and* temperature (14.4 vs 8–9 °C).  A zero-stray day
cannot separate them, so "TP1 void" rests on the pre-registered table, not on
this day's evidence.  The design is a 2×2:

| | TP1 = 100 ms | TP1 = 10 ms |
|---|---|---|
| cold (<9.5 °C) | **strays** (3 days) | ← **the decisive pending cell** |
| warm (>11 °C) | ? — *was the 11.1 °C day at 100 ms?* | no strays (10-10) |

The warm/100 ms cell **is** filled: 10-07 (11.1 °C, no strays) ran TP1 at
100 ms — the 10 ms write was 10-09 15:06Z.  (Caveat theirs: 10-07 was also the
day after a boot.)

> ⚠ **I misread what that cell buys, and the error is necessary-vs-sufficient.**
> I argued that warm/100 ms = "none" makes temperature *sufficient* and voids
> TP1.  It does not — BBGresearch corrected it.  The cell shows cold is
> **necessary**, and the conjunction **"cold AND TP1's falling edge"** fits all
> four filled cells exactly as well as "cold alone" does.  The filled cells
> cannot discriminate; only **cold / 10 ms** can.

Pre-registered outcomes for that cell:

| cold + TP1 = 10 ms gives… | conclusion |
|---|---|
| strays at **+10 ms** | TP1 **is** the source — magnitude tracks the width |
| strays at **+100 ms** | TP1 **void** — magnitude ignored the width |
| **no strays** | TP1 was **necessary** (the conjunction holds) |
| strays at **some other magnitude** | neither — the magnitude tracks something else |

That fourth row is added here deliberately: a registered table without a
"none of the above" row is the one most likely to be quietly reinterpreted
after the fact.

### The bbgld glitch correction is the first chance at an independent test

Bob had bbgld correct the glitch (deployed 01:55Z 10-11): a learned population
delay seeded at 16.3 ns, so a live `+15 ns` sample with `F−R = −8` was consumed
as `−1`.

This is the out-of-sample test the retracted `−p·Δ` check could never be.
Before the fix the servo steered the *mixture* to zero, leaving the clean
population at −7.05 ns; after it, corrected samples read like clean ones, so
the servo steers *those* to zero and the clock's physical phase should move by

```
step = +p·Δ = +7.17 ns   (clean seconds move from -7.17 to ~0)
```

**But only an observer outside the loop can see it.**  A servo drives its own
input to zero by construction, so EXTINT will read ~0 after the fix whether or
not the clock moved — the servo's own input cannot measure this step.  It is
visible only to a TICC channel, a scope, or **a second clock**.  Which ties
their fix directly to our open question: a per-host bias of this size is
precisely what `(p_A − p_B)·Δ` contributes to cross-host disagreement, so the
natural instrument for confirming their fix is the same two-clock comparison
that our 8.04 ns caveat needs.

#### The common-view observer is under-powered, and noise is not the limit

Their outside-loop observer is **pn511 common view** — both sites time the same
ch19/24/33 stations, so NewHat's step should appear in the cross-site
station-phase difference at 01:55Z.  At 100 ns per session and 45 samples/h
(15 sessions/h × 3 channels):

| window | σ_step | SNR vs 7.17 ns |
|---|---|---|
| 4 h/side | 10.54 ns | 0.68 |
| 12 h/side | 6.09 ns | 1.18 |
| 24 h/side | 4.30 ns | 1.67 |
| 36 h/side | 3.51 ns | 2.04 |
| 78 h/side | 2.39 ns | 3.00 |

"Several hours each side" is SNR ≈ 0.7 — under-powered, not merely marginal.
2σ needs ~35 h/side, 3σ ~78 h/side.

**And the binding constraint is drift, not noise.**  The step sits at a known
instant, so it competes with slow drift across that instant — and averaging
longer to beat the noise accumulates more drift.  At 35 h/side the window spans
~3 days, and this host has a *demonstrated diurnal thermal term*, which aliases
straight into a two-block-mean difference.  Two analysis-only fixes: **fit** a
drift-plus-step-at-known-t0 model rather than differencing (the drift/step
covariance then tells you whether the step is identifiable at all), and run
**placebo steps** at times where nothing happened to measure the method's
false-positive rate on its own data.

**A better instrument may be on our bench — but not right now.**  If PiFace
carries the same sub-tick population, the step can be measured with a **TICC**
(60 ps) instead of a 100 ns/session link — three orders of magnitude better,
and it would make the clean confirmation cheaper here than there.  **Deferred
2026-10-10**: PiFace is powered down and PiPuss was accidentally overvolted, so
this route is unavailable and the common-view attempt is the only one standing.

Two hazards worth watching on a *learned* delay:

1. **Seeded at 16.3 ns from one host on one day.**  If the delay tracks the
   physical dip, it may differ per host and move with temperature — and the
   whole phenomenon already looks thermal.  Learning is right; the seed is a
   prior that must be allowed to move, and monitored when it does.
2. **The correction is only as good as the `F−R` classification.**  A sample
   mis-classified as glitched gets 16.3 ns subtracted for nothing, which is a
   16 ns error in the opposite direction.  The classifier is `F−R ≈ −8 ns`
   against `−500 ms` — a 6-decade separation, so this is safe today, but it is
   the thing to assert in a test rather than assume.

### Nobody else looks at the falling edge either

BBGresearch checked **SatPulse 0.2** (our kindred PHC-sync project, see
[[reference_satpulse_kindred_project]]) at Bob's suggestion: its tracking mode
accepts any sample within 50 ns, its dual-edge filter works at 100–200 ns
tolerances on NIC EXTTS, and it has **no TM2 falling-edge field at all**.  So a
16 ns sub-tick population would pass SatPulse unchallenged exactly as it passes
our `reject_ns=50`.  This is not a defect peculiar to our filter — the whole
small-ecosystem convention is to threshold the rising-edge series and never
decode `towMsF`.

## A candidate for the latency bimodality itself: the epoch output batch

Surfaced 2026-10-18 while answering a BBGresearch config audit (Bob suspected
PePPAR-era message output was delaying their timing messages).

**Bandwidth is not the mechanism.**  Our full enabled set
(`receiver.py::configure_messages`) is ~1824 B/s at 30 SV — 16% of even a
115200 UART, negligible on USB CDC-ACM.  That version of the worry is dead.

**Batch position is a live mechanism.**  TIM-TM2 is an *event* message that
queues behind the epoch batch, and u-blox emits NAV, then RXM, then TIM — so
the entire epoch's output precedes a 36-byte TM2:

| | bytes ahead of TM2 |
|---|---|
| ordinary epoch | 1688 B |
| every 5th epoch | 2188 B (**+30%**) |

NAV-SIG alone is 496 B, 29% of the ordinary batch.

> ⚠ **Whose latency?  The 74–84 ms figure in this document is OURS (PiFace),
> and I quoted it at BBGresearch's hosts three times before they caught it.**
> Their measured on-time latencies are **NewHat 35 ms** and **MadHat 20 ms** —
> half to a third of ours.  The numbers in the table at the top of this file
> come from `piface-default-extint.csv` and `piface-no-ticc-extint.csv`; they
> describe *our* host and were never evidence about theirs.
>
> The both-edges argument above survives the correction only by luck: it needed
> on-time latency to fall *below* a 100 ms pulse width, and 35 ms clears that
> bar more easily than 74–84 ms did.  Reasoning from the wrong host's data and
> getting away with it is not rigor.  **Any cross-fleet claim in this document
> needs its host named** — two fleets, two receivers, two EXTINT sources, and
> now demonstrably different latencies.

### The free test: period-5 structure

NAV-SAT, NAV-TIMEGPS and NAV2-PVT are all configured at **rate 5**, so every
5th epoch carries ~500 B more ahead of TM2.  **If batch-drain sets TM2 latency,
latency must run ~30% higher on every 5th epoch** — a mod-5 signature already
present in every capture taken on a peppar-configured receiver, testable with
no config change, no reset, and no hardware access.

Flat across epoch mod 5 ⇒ batch-drain refuted.  Present ⇒ confirmed, and it
would explain both regimes: on-time latency is drain time, and the late regime
is a batch that failed to drain before the next epoch's was queued.

### Falsifiable prediction for the NAV-SIG arm

If drain time is linear in batch size, removing NAV-SIG (29%) should cut
on-time latency by ~29% — which on their measured baselines means
**35 → ~25 ms (NewHat)** and **20 → ~14 ms (MadHat)**.  (My original form of
this prediction said "74–84 → 52–59 ms", using *our* PiFace baseline for
*their* hosts; see the correction above.)

**Guard the arm at the signal, not the procedure.**  A RAM-only VALSET is the
right safety choice, but a power cycle restores NAV-SIG=1 from Flash — and a
power cycle is also the event most likely to silently invalidate a long run
(TimeHat's igc wedge was an under-supplied 5 V rail).  Verify NAV-SIG is
*absent from the stream* continuously, not that the command was sent.

### Our config persisted to Flash

Worth recording because it bears on every reset discussion in this document:
`send_cfg()` defaults to `layers=7` = RAM|BBR|**Flash**.  Every
`CFG_MSGOUT_*` key we set is in Flash, so **no reset — including the guarded
hot reset weighed earlier — clears the message configuration.**

Also from the audit: a sniffed `0x29-07` is **ours** (class 0x29 = NAV2,
id 0x07 = PVT, rate 5, plus the global `CFG_NAV2_OUT_ENABLED`), while
**NAV-TIMEUTC and NMEA GNZDA are not ours** and our own **NAV-CLOCK was absent
from their sniff** — so that receiver's config is a mixture and cannot be
attributed wholesale to either side.

### Batch drain: REFUTED

Measured by BBGresearch 2026-10-18, TM2 latency binned by the edge's own GPS
second (`towMsR`) mod 5:

| host | window | on-time median | late fraction |
|---|---|---|---|
| NewHat | 6 h before 19:07Z | **34.9 ms in all five bins** (p90 39.1–39.5) | 37.4–38.3% every bin |
| NewHat | after 19:07Z | 33.6–34.2 ms | 94.0–95.4% |
| MadHat | 6 h | **19.7–19.8 ms in all five bins** | 0.2–0.5% |

The prediction was +30% on every 5th epoch — **+10 ms on NewHat, +6 ms on
MadHat**.  The bins differ by **≤0.3 ms**.  Refuted with a ~30× margin, and at
better than 0.5 ms resolution.

**One caveat is load-bearing, and they raised it themselves:** the test assumes
the rate-5 messages were still emitting.  The 10-10 sniff saw NAV-SAT,
NAV-TIMEGPS and NAV2-PVT at 0.2/s on both hosts, but that was eight days
earlier.  If any had been disabled since, the test had **no independent
variable** and would read flat trivially — a flat result would then be
uninformative rather than refuting.  One sniff closes it.

### RESOLVED: the late fraction is window-dependent, not inconsistent

The three values reconcile, and my observer-effect concern was wrong.  The two
19:07Zs are **different days**: raw-frame logging began 2026-10-08 19:07Z,
while 2026-10-18 19:07:06Z is a latency step in bbgld's journal
("TM2 LATENCY +1022 ms (#1)").  Nothing in their instrumentation changed at
today's step.

The underlying structure is **one two-state receiver regime with episodes
lasting hours** (per-file late-row counts: 10-11→12 29,673; 10-12→13 36,353;
10-13→18 209,103; 10-18 16:46 file stale from bbgld start to 17:42:09Z; 10-18
17:49 file on time until 19:07:06Z).  Any fixed window therefore reads a
different mix — 50% over 10-09/10, 38% in the 6 h before today's step, 94%
since.  **The 50.3% and 44.1% figures stand for their windows; state the
window with any of them.**

#### The sign of `fall − rise` IS the latency classifier

A geometric result worth having explicitly.  On a 50%-duty 1 Hz pulse:

```
emitted in (T, T+500 ms) -> latest fall is the PREVIOUS pulse at T-500 -> -500 ms = on-time
emitted after T+500 ms   -> latest fall is THIS pulse                  -> +500 ms = late
```

So "late" means exactly "emitted more than 500 ms after the rise", pinning the
late regime's latency to >500 ms with no clock involved — and their measured
+1022 ms sits inside it.  BBGresearch report every late row at +500 ms and
every on-time row at −500 ms, consistent with this.

#### Open: so where is the −8 ns population?

The 10-10 frame decode found **three** values of `fall − rise`, not two:
`−500.000 ms` on count+1 rows and **`−8 ns` on count+2 rows, 44.1%**.  "Every
late +500, every on-time −500" cannot be a complete description alongside that.
Three readings, with different consequences:

1. **Shorthand** — the −8 ns rows coexist at ~44%.  The glitch is still live.
2. **The glitch stopped** — genuinely absent from 10-11 on.  Then the 16.3 ns
   split and −7.17 ns bias describe a condition that no longer obtains.
3. **The correction masks it** — bbgld's fix deployed 01:55Z 10-11, exactly
   where their per-file counts begin.  If the logged `fall − rise` is
   *post*-correction, the glitch is invisible by construction.

Reading 3 is the hazard: **a correction that rewrites the logged value destroys
the evidence needed to monitor the thing it corrects.**  The alarm they added
for the learned delta wandering from 16.3 ns cannot see that drift if its input
sits downstream of the correction.  Log raw *and* corrected; alarm on raw.

#### A config readback does not test emission

Their plan is a paginated VALGET to close the mod-5 caveat, as "a stronger
check than a sniff".  For *configuration* it is.  For "were the rate-5 messages
**emitting** during the test window" it is strictly weaker — and NewHat is its
own counterexample: we set `CFG_MSGOUT_UBX_NAV_CLOCK_USB = 1` unconditionally
whenever SFRBX is on, their 10-10 sniff saw SFRBX, and it **did not see
NAV-CLOCK**.  Config and emission demonstrably disagree on that receiver.

Better than either, and retroactive: both a sniff and a readback measure *now*,
while the test window was hours earlier.  If bbgld's capture from that window
logs UBX arrivals, check whether NAV-SAT / NAV-TIMEGPS / NAV2-PVT appear at
0.2/s **inside the window** — evidence about the right interval, with no
receiver access and no bbgld stop.


