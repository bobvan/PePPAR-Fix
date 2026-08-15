# Building a multi-channel TICC equivalent from TDC Click boards

Research note, 2026-08-15.  Question asked: the MIKROE-4770 carries the
same TDC chip as our TAPR TICCs — could we put one or more on a
mikroBUS host board and get a two-or-more-channel TICC equivalent?

**Short answer: yes, and more cheaply and simply than first thought.
The Click is $18 and the two-socket Pi Pico shield is $54, so two
channels of hardware is ~$90.  The only real risk left is whether the
RP2040 can be clocked coherently from the house 10 MHz.**

> **CORRECTION 2026-08-15** — the first version of this note named the
> *stop gate* as the hard part and the main reason a Click build would
> be non-trivial.  That was wrong, and wrong in the expensive direction:
> it implied per-channel glue hardware that does not need to exist.  The
> TDC7200 has the gate **built in** as the `CLOCK_CNTR_STOP_MASK`
> registers — "all STOP signals occurring before the value set by the
> CLOCK_CNTR_STOP_MASK registers will be ignored" (§8.3.3.3).  At 10 MHz
> a mask of 3 gives the same 300 ns blanking the TICC builds in
> hardware, for the cost of a register write.  §2 and §3 below are
> updated; the reasoning about *why the TICC needs external hardware
> anyway* is in §2.1.

---

## 1. What MIKROE-4770 actually is

TDC Click, a size-M (42.9 × 25.4 mm) mikroBUS add-on carrying the **TI
TDC7200** — the same converter used in the TAPR TICC (which carries two,
one per channel).  So the fundamental single-shot resolution, ~55–60 ps,
is *identical to the TICC's by construction*.  Nothing about the Click
degrades it.

| Signal | Where it goes |
|---|---|
| SPI (CS / SCK / MISO / MOSI) | mikroBUS SPI, up to 20 MHz |
| `EN` (enable / reset all digital) | mikroBUS **RST** |
| `TRG` (trigger, start measurement) | mikroBUS **PWM** |
| `INTB` (measurement complete) | mikroBUS **INT** |
| `OEN` (onboard oscillator enable) | mikroBUS **AN** |
| START, STOP | dedicated onboard signal connectors |
| CLOCK | onboard 8 MHz crystal **or** external via a miniature coax connector |

Supply 3.3 V.

The two features that matter most, and that I did not expect to find:

1. **A `CLK SEL` jumper with `INT` / `EXT` positions, and a miniature
   coaxial (N.FL-series) connector for an external reference clock.**
   This is decisive.  The TDC7200 datasheet is blunt that "measurement
   accuracy is heavily dependent on the external CLOCK accuracy" — the
   reference is what the ring oscillator is calibrated against on every
   single measurement.  With `CLK SEL = EXT` we can drive the Click from
   the lab's house 10 MHz exactly as the TICC does.  Had the board been
   hard-wired to its onboard 8 MHz crystal, it would have been useless
   for our purposes and this note would end here.
2. **START and STOP are brought out as separate signal connectors.**
   That is precisely the topology a TICC needs: DUT pulse → START, gated
   coarse clock → STOP.  A board that only generated its own pulses for
   time-of-flight demos could not be used this way.

**One item to confirm physically when a board is in hand:** sources
describe the START/STOP connectors as SMA and the clock connector as
N.FL.  Three SMAs on a 42.9 × 25.4 mm board would be crowded, and I
could not reach MikroElektronika's schematic (mikroe.com and
download.mikroe.com both return 403 to automated fetches).  Assume
U.FL/N.FL pigtails may be needed until the board is inspected.  This
changes cabling, not feasibility.

Cached locally: `doclib show tdc7200`, `doclib show A700000008924024`.

---

## 2. What a TICC actually does

This is the part worth internalizing before designing anything, because
the TDC7200 **is a stopwatch, not a timestamper**.  It measures START →
STOP and nothing else.  Everything that makes a TICC a *timestamping
counter* is built around it.

The TICC's architecture (from the TAPR manual, cached as
`doclib show TICC-Manual`):

```
EXT_REF 10 MHz ──┬──────────────────────────► TDC7200 CLOCK  (both channels)
                 │
                 └──► ÷1000 divider ──► COARSE_CLOCK 10 kHz (100 us)
                                            │
                                            ├──► host interrupt ──► PICcount++
                                            │
DUT pulse ──► TDC7200 START                 │
      │                                     │
      └──► [ STOP GATE ] ◄──────────────────┘
                 │
                 ├──► TDC7200 STOP
                 └──► host interrupt ──► latch PICstop = PICcount

INTB ──► host reads TIME1/TIME2/CLOCK_COUNT1 + calibration over SPI ──► tof

timestamp = (PICstop × 100,000,000 ps) − tof
```

The subtraction is not a typo: `PICstop` is the coarse tick *after* the
event, so the fine measurement is removed from it.

Numbers that pin the design down:

- **Reference: 10 MHz.**  Coarse tick: **10 kHz / 100 µs.**  In the
  TICC this divider is a PIC running Tom Van Baak's `PD15` firmware.
- **TDC7200 Measurement Mode 2** (range 250 ns – 8 ms).  Mode 1 has a
  12 ns minimum but tops out near 500 ns — far too short to span a
  100 µs coarse tick.
- Mode 2 imposes a **minimum START→STOP of ~200 ns**.
- Resulting time-of-flight range: **~200 ns to 100.3 µs**, comfortably
  inside Mode 2 at both ends.
- The TDC self-calibrates its ring oscillator against the reference at
  the end of *every* measurement (default 20 reference periods), which
  is what makes a free-running ~17 GHz ring usable.
- Throughput: **>600 timestamps/s single channel**; ~1130/s in binary
  mode at 230400 baud.  For 1 Hz PPS work this is not a constraint.

### The stop gate — the piece that does not exist on the Click

`COARSE_CLOCK` cannot be wired straight to `STOP`.  If a coarse tick
happens to land less than 200 ns after the DUT pulse, the TDC7200's
minimum START→STOP is violated and that measurement is invalid.  Over a
long run this *will* happen — with a 100 µs tick and an asynchronous
DUT, roughly 1 measurement in 500 lands in the forbidden window.

The TICC's solution: a **stop gate clocked by EXT_REF that blocks
COARSE_CLOCK from reaching STOP for 3 EXT_REF cycles (300 ns) after the
START pulse.**  After the gate opens, the next coarse tick stops the
timer.  So the tof range becomes just-over-200 ns to 100.299… µs —
always legal.  The same gated pulse is what interrupts the host to latch
`PICstop`, which is also why the host isn't taking a 10 kHz interrupt on
every coarse tick per channel.

### 2.1 …but we do not need to build one

The TDC7200 already implements this internally.  Per §8.3.3.3, the
Clock Counter starts on the first rising CLOCK edge after START, and
**all STOP signals arriving before `CLOCK_CNTR_STOP_MASK` clock cycles
are ignored**.  At 10 MHz (100 ns/cycle) a mask of 3 reproduces the
TICC's 300 ns blanking exactly.  The register is 16-bit
(`_H × 2^8 + _L`), so the window can run to 6.55 ms; the one constraint
is that `CLOCK_CNTR_OVF` must remain **above** the mask value, or the
overflow interrupt halts the measurement before the window expires.

So why does the TICC build the gate in hardware?  Because its gate does
**two** jobs: it blanks early STOPs, *and* the gated pulse is what
interrupts the Arduino to latch `PICcount`.  The second job is what
requires a physical signal.  In a design where the host itself generates
the coarse clock, the host already knows the count — the second job
disappears, and with it the need for external gating.

**Consequence for wiring: STOP is a single free-running coarse clock,
fanned out to every channel's STOP input.**  No per-channel logic.

---

## 3. Gap analysis

| Needed | On the Click? |
|---|---|
| TDC7200, 55 ps resolution | **Yes** — identical to TICC |
| External 10 MHz reference into CLOCK | **Yes** — coax + `CLK SEL=EXT` |
| START and STOP as separate inputs | **Yes** — signal connectors |
| SPI + INTB + EN + TRG to host | **Yes** — via mikroBUS |
| 10 MHz → 10 kHz coarse divider | **No** |
| Stop gate (per channel) | **Not needed** — `CLOCK_CNTR_STOP_MASK` does it in-chip (§2.1) |
| Coarse counter + STOP-edge latch | **No** (host's job) |
| Input conditioning for 2–5 V DUT pulses | **No** — 3.3 V logic input |
| 50 Ω termination on the reference input | **Unknown** — verify |
| Coherent clock distribution across channels | **No** |

Two entries deserve emphasis:

**Input conditioning.** The TICC accepts 2–5 V pulses and triggers on
the leading edge with defined input impedance.  The Click's START/STOP
go to a 3.3 V part.  Our PPS sources are not all 3.3 V-friendly, and a
5 V PPS into a 3.3 V input is a slow way to destroy a board.  Any real
build needs comparators or at minimum level translation and protection.

**Reference termination.** The TICC manual is emphatic, from experience:
without the 50 Ω termination jumper, ringing on a square-wave 10 MHz
source caused double-clocking and timestamps advancing at *twice* real
time.  That failure is dangerous precisely because it looks like data
rather than like a fault.  Whatever we build, terminate the reference
and verify the coarse count rate against wall clock before trusting a
single timestamp.

---

## 4. The host is the real architectural decision

Every channel must share **the same 10 MHz and the same coarse clock**,
or timestamps from different channels are not on a common timescale and
differencing them is meaningless.  Beyond that, the host must maintain
the coarse counter and latch it at each STOP edge.

The TICC does this with an ATmega2560 at 16 MHz using hardware
interrupts.  **A Raspberry Pi running Linux cannot do this job** — there
is no way to service a 10 kHz interrupt with bounded jitter under a
general-purpose kernel, and jitter here lands directly in the coarse
half of every timestamp.  This is why the TAPR multi-TICC uses a Pi only
as a *data collector* over USB, with each TICC keeping its own coarse
counter on its own Arduino.

Three viable host paths:

**(a) ATmega2560, TICC-style.**  Proven, and the TICC firmware is
open-source BSD and already knows how to do the arithmetic (including
the 64-bit-avoidance accumulator trick that keeps throughput up).
Lowest risk; also lowest ceiling.

**(b) RP2040 / RP2350 with PIO — the most interesting option.**  The PIO
state machines can plausibly implement *all three* missing pieces in
deterministic hardware: the ÷1000 divider, the stop gate (a 3-cycle
inhibit after START is a trivial PIO program), and the coarse counter
with a hardware latch on the STOP edge.  That would be **architecturally
better than the TICC**, not merely equivalent — it removes ISR latency
jitter from the coarse path entirely, and removes the external divider
and gate chips from the BOM.

The thing to verify before committing: the PIO must be clocked coherently
with EXT_REF, or the divider and gate drift relative to the TDC's own
reference.  The RP2040 can be driven from an external clock source, so
this looks achievable, but **confirm that a 10 MHz external clock can
drive the system/PIO clock cleanly before designing around it.**  If it
can't, fall back to an external ÷1000 and let PIO do only the gate and
counter.

MikroElektronika makes a **Click Shield for Pi Pico with two mikroBUS
sockets** — a two-channel prototype needs no custom PCB at all.

**(c) Per-channel MCU, Pi as collector.**  The multi-TICC pattern.
Scales indefinitely, at the cost of one MCU per two channels and a
cross-unit synchronization problem you must then solve.

---

## 5. TAPR already solved the multi-channel problem

Before building anything, read **TAPR App Note 2020-01, "multi-TICC"**
(cached: `doclib show multi-TICC-App-Note-2020-01`).  It documents
**four TICCs → eight channels**, collected by a Raspberry Pi and served
over the network as a "timestamp appliance" — telnet in, pull live data
from any channel, hot-plug inputs without disturbing the others.

The synchronization method is the one we'd have to reinvent otherwise:
one **primary** board generates COARSE_CLOCK and a startup sync pulse;
**secondary** boards have their divider PIC (IC10) *removed* and take
both signals from the primary over a 3-pin header (pin 1 sync, pin 2
ground, pin 3 COARSE_CLOCK), with firmware set to Secondary mode.  A
gotcha worth stealing: shorting `DISABLE AUTO-RESET` (JP1) on every
board, because otherwise opening a serial port resets that Arduino and
desynchronizes it.  (Note the parallel with our own TICC DTR/HUPCL
problem in CLAUDE.md — same class of bug, different layer.)

The multi-TICC is explicitly **not** a TAPR product: material cost and
assembly labor were judged too high for a niche device.  That is the gap
a Click-based build would be filling.

---

## 6. Options, honestly compared

| | Design risk | Per-channel cost | Ceiling |
|---|---|---|---|
| **A. More TICCs / build a multi-TICC** | None — documented | Highest (TICC kit + Arduino Mega per 2 ch) | 8 ch, proven |
| **B. Clicks + Pico + PIO** | Highest | Lowest | Best — no ISR jitter |
| **C. Clicks + external divider/gate board + Mega** | Medium | Low-medium | TICC-equivalent |
| **D. Custom board, bare TDC7200s** | High (PCB + TSSOP) | Lowest at volume | Best, most work |

**Recommendation: prototype B at two channels, on a Click Shield for Pi
Pico with two TDC Clicks.**  Rationale: no custom PCB for the prototype,
it directly tests the one genuinely uncertain thing (can PIO be clocked
coherently from the house 10 MHz and implement the stop gate), and if
PIO works it is a better machine than the TICC rather than a clone.  If
PIO clocking turns out not to work, the same two Clicks drop into option
C with an external divider and a flip-flop.

**Validation is non-negotiable and we already own the instrument.**  Any
build gets checked against a real TICC: same PPS into both, compare
timestamp series, and confirm the noise floor.  We have TICC #1, #2, #3,
and #5, plus documented calibration procedure in
`docs/ticc-calibration-2026-03-19.md` and `timelab/calibration.md`.  A
new timestamper that hasn't been differenced against a known-good TICC
is a source of confident wrong numbers.  Specific checks:

1. Coarse count rate vs wall clock — catches the termination/ringing
   failure mode before it poisons anything.
2. Same-signal-both-channels ("zero baseline") — should show the
   instrument's own noise floor and nothing else.
3. Common PPS into new build and real TICC — TDEV of the difference
   should sit at the RSS of the two instruments' floors, ~60–100 ps.
4. Deliberately verify the stop gate by looking for outliers at the
   ~1-in-500 rate a missing gate would produce.

**I have not verified current pricing** for the TDC Click or the TICC
kit; the cost column above is relative, not absolute.  Check both before
choosing — if a TICC kit is close in price to a two-Click build, option
A wins on risk alone.

---

## 7. Why PePPAR-Fix would want this

Not "a cheaper TICC" — more channels *on one timescale*.

CLAUDE.md's cross-host PPS agreement goal ("any pair of PePPAR Fix
clocks must produce PPS OUT edges that agree in phase, with a
probability-bounded maximum excursion") is measured today by connecting
two PPS OUT signals to chA and chB of a shared TICC, so the differential
is immune to that TICC's own reference noise.  That works for **a
pair**.  With six or eight channels on one coarse timescale, the entire
fleet — PiPuss, PiFace, ptBoat, otcBob1, plus a reference — could be
compared *simultaneously*, and every pairwise difference falls out of
one dataset instead of requiring a separate cabling session per pair.

That directly serves the two-clock excursion bound work in
`docs/two-site-sync-budget.md`, and it would make three-cornered-hat
separation of per-clock noise routine rather than an expedition.  It
also removes a real scheduling constraint: TICC channels are currently a
contended lab resource (`hw:` labels exist precisely because of this).

Worth noting what it does **not** buy: the instrument's reference still
sets the floor in timestamp mode.  A multi-channel timestamper does not
relax the need for a good house standard — it just stops us from having
to choose which two clocks to look at.

---

## 8. Open questions

1. **Can the RP2040/RP2350 be clocked coherently from the house
   10 MHz?**  Now the *only* significant risk, and it carries all the
   weight: if the Pico's coarse clock is not phase-locked to the same
   reference the TDCs calibrate against, the two timescales drift and
   the timestamps are worthless.  If yes, this build is simpler than a
   TICC.  If no, add one external ÷1000 divider — still no per-channel
   hardware.  **Answer this before designing anything else.**
2. **Physical connectors on the Click** — SMA vs U.FL/N.FL for
   START/STOP, and whether the reference input is 50 Ω terminated.
   Needs a board in hand or the schematic.
3. **Input conditioning design** — what our PPS sources actually present
   (levels, impedance, rise time), and therefore what comparator front
   end is needed.  The TICC's front end is a reasonable template.
4. **Does `OEN` on the AN pin need driving** when `CLK SEL = EXT`, or
   does the jumper alone isolate the crystal?  Affects whether the
   onboard 8 MHz can inject spurs into a design running on house 10 MHz.
5. **Price of both paths**, per section 6.

## References

All cached in `doclib` (group `timelab`) — `doclib search "<query>" --group timelab`:

- TI TDC7200 datasheet (SNAS647D) — `doclib show tdc7200`
- TDC Click MIKROE-4770 datasheet — `doclib show A700000008924024`
- TAPR TICC Operation Manual (rev. 2025-12-15) — `doclib show TICC-Manual`
- TAPR App Note 2020-01 multi-TICC — `doclib show multi-TICC-App-Note-2020-01`
- TICC source: <https://github.com/TAPR/TICC> (BSD)
- PIC divider firmware: <http://www.leapsecond.com/pic/picdiv.htm>


---

## 9. Wiring, resolved (2026-08-15)

Confirmed prices: TDC Click **$18** each, Click Shield for Pi Pico
**$54** → **~$90 for two channels**.  Clock input is **u.FL**.

STOP is an **input to the TDC**, driven by the Pico — nothing flows from
the Click's STOP pin back to the host.

| Signal | From | To |
|---|---|---|
| 10 MHz house reference | 3-way fan-out | Click A clock (u.FL), Click B clock (u.FL), Pico clock input |
| Coarse clock, 10 kHz | one Pico GPIO | 2-way fan-out → Click A STOP, Click B STOP |
| PPS channel A | DUT | Click A START |
| PPS channel B | DUT | Click B START |
| `TRIGG` A / B | mikroBUS **PWM**, per socket | Pico GPIO (interrupt) |
| `INTB` A / B | mikroBUS **INT**, per socket | Pico GPIO (interrupt) |
| SPI + per-socket CS | mikroBUS | Pico SPI |

Both Clicks set to `CLK SEL = EXT`.  Series resistor on the Pico's
coarse-clock output.

**Which coarse tick stopped a given channel** is recovered from
`TRIGG`, which the Click routes to mikroBUS PWM.  Datasheet measurement
sequence step 5: *"After receiving a START, the TDC resets the TRIGG
pin"* — so TRIGG's falling edge marks START arrival on a pin already
present in the mikroBUS connector, and **no PPS fan-out to the Pico is
required**.  The host snapshots its coarse count on that edge; combined
with the measured `tof`, the tick is unambiguous (ticks 100 µs apart,
`tof` known to picoseconds, so tens of µs of slop in the TRIGG
timestamp are harmless).  At 1 Hz PPS this is an ordinary GPIO
interrupt — **no PIO and no determinism needed on this path.**  The only
path that must be hardware-deterministic is generation of the coarse
clock itself.

Between measurements the free-running coarse clock on STOP is harmless:
step 8 of the sequence disables the START, STOP and TRIGG pins once a
measurement completes, so stray ticks are ignored until the host re-arms
via `START_MEAS`.

**Open cabling question:** the clock input is u.FL — are START and STOP
also u.FL, or SMA?  If u.FL throughout, two channels need six pigtails
plus two fan-outs, which is now the fiddliest part of the build.
