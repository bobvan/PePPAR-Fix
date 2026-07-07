# GNSSDO+ PPS-to-DO-edge alignment — bias + boundary-step concerns

The SparkFun SXT / SXT-D (GNSSDO / GNSSDO+) has a hardware feature that
**delays the mosaic-T PPS until the next disciplined-oscillator (10 MHz)
rising edge** before it leaves the box as PPS OUT. It is **on by default and
overridable**. Almost certainly a D-flip-flop (or equivalent retiming latch)
clocked by the 10 MHz DO, with the mosaic-T PPS as the data input, so the
output PPS edge is quantized to a DO clock edge.

Purpose: hand downstream consumers a PPS that is phase-coherent with the box's
10 MHz output (the two agree to the DO's edge), instead of a PPS whose edge can
sit anywhere within a 10 MHz period relative to the 10 MHz.

This note records two concerns (Bob, 2026-07-07) and the bench test that
resolves them. **Status: plan agreed; scope test pending lab access.**

## Concern 1 — the alignment adds a bias

Retiming to the next DO rising edge delays the PPS by the time from the
mosaic-T PPS edge to that DO edge — somewhere in **[0, 100 ns)** (one 10 MHz
period). So the box PPS OUT is offset from the mosaic-T's own (GPS-aligned) PPS
by a fixed **alignment bias** = that delay.

Consequences:
- **Downstream absolute timing**: a consumer of the box PPS OUT sees GPS ToS +
  bias. Calibratable, but real, and it is *not* the mosaic-T's PPS.
- **Cross-host PPS agreement** (the moonshot's 1 ns / 2 ns two-clock excursion
  bound, `docs/two-site-sync-budget.md`): two GNSSDO+ boxes can settle at
  *different* alignment biases (their DO edges fall at different phases vs GPS
  ToS), so their PPS OUTs differ by a constant offset up to ~100 ns even when
  both clocks are perfectly disciplined. Constant and calibratable, but it must
  be measured per box, not assumed zero.
- **Our discipline loop is unaffected**: PePPAR-Fix disciplines the DO from the
  mosaic-T's carrier-phase `dt_rx`, upstream of this circuit. The alignment
  bias is downstream of the loop — it shifts the PPS OUT, not `dt_rx`.

## Concern 2 — boundary metastability → 100 ns steps *while disciplined*

If the alignment bias ever sits **near a 100 ns boundary** — i.e. the mosaic-T
PPS edge arrives within the flip-flop's setup/hold window of a DO clock edge —
then noise/jitter can push it across the edge from epoch to epoch, so the
retiming catches *this* DO edge on some seconds and the *next* one on others.
The PPS OUT then **steps by 100 ns** intermittently, even though the DO
frequency is perfectly disciplined. (Worse, right at the window it can go
metastable and produce a runt/late edge.)

This is not hypothetical for a disciplined GNSSDO+. The loop drives `dt_rx → 0`,
aligning the DO's timescale to GPS. **If that also places a DO 10 MHz edge at
(or very near) GPS top-of-second, then the mosaic-T PPS lands right on a DO
clock edge — the metastable boundary.** Whether the disciplined operating point
sits there, or comfortably inside the 100 ns window, is a property of the
mosaic-T's PPS generation vs the DO phase, and is exactly what the scope test
below measures.

### Distinction from the free-run cycle-slips

When the DO **free-runs** (e.g. `gnssdo_freerun_hold.py` for characterization),
it drifts against GPS ToS and *deterministically* crosses 100 ns boundaries as
it drifts — producing occasional clean 100 ns PPS steps (~0.4/h at a held word
near GPS frequency; see `docs/gnssdo-plus-integration.md §8`). That is expected
and handled in analysis by `freerun_analysis.deglitch_cycle_slips`
(GNSSDO-specific, opt-in). Concern 2 is different: it is 100 ns steps **while
disciplined**, from *jitter across a fixed boundary* rather than *drift across
boundaries* — a real defect in the PPS OUT, not an analysis artifact, and not
something to de-glitch away.

## The bench test (once back in the lab)

Open the box and scope, on a shared trigger:

1. **Raw mosaic-T PPS OUT** (before the alignment latch — probe the mosaic-T PPS
   pin / the flip-flop data input), and
2. **Box PPS OUT** (after the latch), and ideally
3. the **10 MHz DO** (the flip-flop clock),

with the DO **disciplined** (ESP32 or PePPAR-Fix). Measure the delay from (1)
to (2) and where the mosaic-T PPS edge falls relative to the DO edge (3).

### Acceptance criteria

**Leave the alignment feature ON** if the box PPS OUT shows:
- a **constant bias** relative to the raw mosaic-T PPS (stable, not stepping),
  **and**
- that bias is **not near the boundary** — the mosaic-T PPS edge sits well away
  from the DO clock edge, with margin > the flip-flop's setup time (i.e. the
  operating delay is comfortably inside `(0, 100 ns)`, away from both
  `≈ 0` and `≈ 100 ns − t_setup`).

**Otherwise** (bias near the boundary, or observed 100 ns steps / metastable
edges while disciplined): **override the alignment** (disable the latch → PPS
OUT = raw mosaic-T PPS, no bias, no boundary steps, but the PPS OUT is no longer
phase-coherent with the 10 MHz), or otherwise move the operating point off the
boundary.

### In-situ cross-check (no box-opening needed)

The disciplined grade run (`gnssdo-disc2-ticc`, 2026-07-07) captures the box
PPS OUT on the TICC while disciplined. **A non-zero 100 ns cycle-slip rate on
that disciplined chA is Concern 2 manifesting** — the operating point is near
the boundary. A ~zero disciplined slip rate is evidence the operating point is
safely inside the window. `tools/char_gnssdo_freerun.py` reports the slip
count; the scope test is still the authoritative check (it sees the raw
mosaic-T PPS and the setup-time margin directly).

## Override

The alignment is overridable (ESP32 console / mosaic-T config — mechanism TBD;
check the SXT firmware's PPS setup and the mosaic-T `setPPSParameters` /
alignment option). Disabling it trades DO-edge phase-coherence of the PPS OUT
for a bias-free, boundary-step-free PPS that is the mosaic-T's directly. The
decision is downstream-consumer-dependent and deferred to the scope result.
