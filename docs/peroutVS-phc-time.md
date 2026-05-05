# PEROUT edge vs PHC counter — what "GPS-aligned" means in PePPAR-Fix

The system has multiple distinct points where time can be observed
along the chain from the PHC's internal counter out through the SDP
pin and onto a cable to a downstream consumer.  These points are
**not the same** — they're separated by hardware-fixed delays of
hundreds of ns to a few microseconds, depending on the PHC's PEROUT
implementation and the cable run.

This doc spells out which point we are *aligning to GPS top-of-second*,
and what that means for code that observes time at one of the other
points (in particular, kernel timestamping of incoming packets via
the PHC's internal counter).

## The chain

```
F9T's GNSS antenna
     │
     │ (RF + sat geometry — gives us GPS time at the antenna)
     ▼
F9T receiver, internal clock model
     │
     │ (F9T's PPS edge, quantised to TCXO tick — qErr says where GPS
     │  actually fell within that tick)
     ▼
F9T PPS pin → splitter → TICC chB
                       → i226 EXTTS pin (or TADD-2 SYNC_IN on PiFace)

PHC counter (i226 internal timer / OCXO via DAC on PiFace)
     │
     │ (PHC integer second tick.  PHC counter value at this moment is
     │  what gets stamped on incoming packets via SO_TIMESTAMPING /
     │  PTP HW timestamps.)
     ▼
PHC → SDP pin (PEROUT internal hardware latch)
     │
     │ (SDP pin asserts at integer second.  i226 introduces a fixed
     │  ~µs latency here; TADD-2 dividers introduce their own residual.)
     ▼
SDP pin → splitter → TICC chA
                   → external PPS distribution
```

## What we align

PePPAR-Fix's discipline loop drives **TICC chA − TICC chB → 0**.  That
is, the rising edge of PEROUT *as observed at the local TICC chA input*
is aligned to the rising edge of F9T PPS at TICC chB (which is GPS
top-of-second modulo qErr).

In other words, our reference moment "GPS top-of-second" is **the
arrival of the rising edge of the PEROUT pulse at TICC chA**.  Anything
downstream of that point (other consumers of the PEROUT signal via the
splitter, networked PPS distribution, etc.) sees GPS time *at the moment
the cable delivers the edge to them* — which means cable propagation
matters per-consumer, but every consumer downstream of the same
splitter sees the same edge.

## What we do *not* align

The PHC's internal counter ticks at the same rate as PEROUT but
**reads a different value at the moment of GPS top-of-second**.
Concretely:

```
PHC counter at GPS top-of-second  =  N (integer second)  −  Δ_hw
                                                         ─────
                                              PHC→PEROUT→cable delay
```

where Δ_hw is the per-host hardware latency from "PHC counter equals
integer second" to "TICC chA sees the rising edge."  Tonight's
measurements (on UFO1's L5 fleet, fresh-stepped):

| Host | architecture | local PEROUT − F9T_PPS |
|------|--------------|------------------------|
| TimeHat | i226 PHC + SDP0 PEROUT | +2.21 µs |
| MadHat  | i226 PHC + SDP0 PEROUT | +1.91 µs |
| PiFace  | OCXO via DAC + TADD-2 divider | +1.42 µs |

These numbers are with `--ticc-target-ns 0` (servo actively pulls
chA-chB to zero).  Once the loop converges, chA-chB ≈ 0 and the
*PEROUT edge* is GPS-aligned.  But the *PHC counter* at that moment
reads `GPS_top - Δ_hw`, where Δ_hw is the values shown above.

## Implications for downstream code

**Anywhere we timestamp a packet or other event using the PHC counter
directly** — kernel SO_TIMESTAMPING, ts2phc reads, PTP timestamping,
RXM-RAWX `rcvTow` — the timestamp is **offset from the GPS-aligned
PEROUT edge** by the PHC→PEROUT→cable delay.

For most use cases this doesn't matter: the offset is a constant
per-host and cancels in any time-difference computation.  But for
absolute timestamping where the consumer expects "this counter value
== GPS time," the constant Δ_hw must be added.

**Per-host calibration**: Δ_hw is a function of (PHC implementation,
SDP-pin/divider electronics, cable length to TICC chA).  It changes
only if the cable is re-routed or the hardware is replaced.  Once
characterised, it can be stored alongside `feedline_ns` in the
receiver/DO state file.

## Why this convention

The alternative would be to align "PHC counter == GPS top-of-second"
directly.  That would require *measuring* and *correcting for* the
PHC→PEROUT→cable delay continuously, which we cannot do — we have no
oracle for the PHC counter's relationship to GPS independent of the
PEROUT chain it generates.  TICC chA is the closest point in the chain
that we can directly measure with sub-ns precision.

So we align what we can measure.  The PHC counter then *trails* GPS
top-of-second by a per-host fixed offset.  Code that needs GPS-aligned
timestamps from the PHC counter must apply that offset.

## See also

- `tools/adj_setoffset_relative_precision.py` — characterises the PHC
  step itself (sub-200 ns p95 on i226 ADJ_SETOFFSET, post-propagation)
- `docs/phc-bootstrap.md` — bootstrap step-and-glide algorithm
- I-013346-main (dayplan) — fix `phi_0` verify to use PPS edge + qErr
  rather than CLOCK_REALTIME, so the engine's reported residual matches
  the actual step precision instead of the verify-drift artifact
