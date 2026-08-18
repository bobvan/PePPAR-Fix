# Antenna Position and Timing Calibration Plan

Establish cm-level antenna position, sub-ns cable delay, and F9T
observation noise floor using zero-baseline and short-baseline
techniques with independent confirmation from local CORS.

## Goals

1. **Antenna position** to ±1 cm horizontal, ±2 cm vertical
2. **Cable delay** to ±1 ns
3. **F9T carrier-phase noise floor** (isolated from all common errors)
4. **PPS alignment to GPS time** — absolute timing to ±2 ns requires
   knowing antenna position + cable delay + receiver internal delay
5. **Independent confirmation** — cross-check against multiple CORS
   stations at known positions

## Equipment

| Item | Role |
|------|------|
| Choke ring antenna (survey-grade) | Primary antenna — published PCO/PCV, multipath rejection |
| Leica GRX 1200 GG Pro | Zero-baseline reference receiver (GPS+GLONASS L1/L2) |
| u-blox ZED-F9T (TimeHat) | Receiver under test |
| Splitter (Wilkinson or SMA tee) | Feed both receivers from one antenna |
| TICC #1 | PPS timing comparison (Leica PPS vs F9T PPS) |
| Local CORS (Naperville, DuPage) | Short-baseline position validation |

## Antenna Selection

The choke ring antenna is ideal for calibration:

- **Published phase center model**: IGS maintains absolute antenna
  calibration files (ANTEX format) for survey-grade choke ring
  antennas.  Look up the antenna model at
  https://files.igs.org/pub/station/general/igs20.atx
  The phase center offset (PCO) and phase center variation (PCV)
  are specified per frequency, per elevation/azimuth.
- **Multipath rejection**: the choke ring's ground plane suppresses
  multipath from below, giving cleaner carrier-phase observations.
- **Stable phase center**: unlike patch antennas, the choke ring's
  phase center is well-defined and stable across elevation angles.

Before starting, identify the antenna's IGS model name and verify
it appears in the ANTEX file.  If it doesn't, the antenna needs
to be calibrated (send to a calibration lab, or use the zero-baseline
method to characterize it relative to a known antenna).

## Experiment 1: Zero-Baseline (F9T + Leica on One Antenna)

### Setup

```
Choke ring antenna
      |
    Splitter (Wilkinson preferred — better isolation)
    /         \
F9T (TimeHat)  Leica GRX 1200
   |              |
   PPS OUT        PPS OUT (10 MHz if available)
   |              |
 TICC chA      TICC chB
```

Both receivers see identical signals.  All common-mode errors
cancel in the double difference:
- Ionosphere: identical (same signal path)
- Troposphere: identical
- Satellite clocks: identical
- Multipath: identical (same antenna)
- Antenna phase center: identical

Residual = receiver noise only.

### Procedure

1. **Connect** choke ring to splitter.  Splitter output 1 → F9T
   antenna input.  Splitter output 2 → Leica antenna input.  Note
   cable lengths (measure with tape, refine with TDR if available).

2. **Configure Leica** for RTCM3 or RINEX logging via Ethernet.
   Set to 1 Hz, GPS+GLONASS, all available signals.  Log raw
   observations.  If the Leica can output PPS, connect to TICC chB.

3. **Configure F9T** for RAWX at 1 Hz.  PPS to TICC chA.

4. **Start capture**: run both receivers + TICC simultaneously for
   **24 hours**.  The long duration gives:
   - Complete sky coverage (all azimuths sampled)
   - PPP convergence to mm level
   - Temperature cycle coverage (day/night thermal effects)

5. **Collect from local CORS**: stream NAPERVILLE-RTCM3.1-MSM5
   (port 12054) for the same 24 hours.  Also stream any DuPage
   stations within 10 km.

### Analysis: F9T Carrier-Phase Noise

Form GPS double-differences between F9T and Leica for each satellite
pair.  At zero baseline, the DD residual is:

    DD_residual = noise_F9T + noise_Leica + splitter_imbalance

The Leica's carrier-phase noise is specified at ~1 mm (L1) and ~1.5 mm
(L2) from the manufacturer.  The F9T noise is unknown — this experiment
measures it.

Expected F9T carrier-phase noise: 2-5 mm based on similar consumer
dual-frequency receivers.  If significantly worse, it limits the PPP
filter's achievable clock precision.

### Analysis: PPS Timing

TICC chA - chB gives the PPS timing difference between F9T and Leica.
This includes:
- Receiver internal PPS generation delay (different for each receiver)
- Cable delay difference (splitter to each receiver)
- Receiver clock offset (F9T TCXO vs Leica OCXO drift)

The Leica GRX 1200's internal PPS delay is published in its
specifications.  Subtracting this gives the F9T's internal delay.

### Analysis: Antenna Position

Run 24-hour PPP on the F9T observations (or the Leica observations
if tools are available).  Cross-check:
- F9T PPP position vs Leica PPP position (should agree within noise)
- Both vs CORS baseline solution (NAPERVILLE at ~5 km)

The 24-hour PPP solution gives absolute position in ITRF2020 to
~5 mm horizontal, ~15 mm vertical.  Apply the antenna PCO/PCV from
the ANTEX file to get the ARP (antenna reference point).

Measure the ARP height above the monument (or ground mark) with a
tape measure to ±1 mm.


## Experiment 2: Short-Baseline to CORS

### Why Multiple CORS Stations

Using multiple CORS stations within 10 km provides:

- **Redundant position check**: if all baselines agree on your
  position to ±1 cm, the position is trustworthy.
- **Troposphere validation**: at <10 km, troposphere is nearly
  common.  Any residual troposphere shows up as a vertical bias
  that's consistent across all baselines.
- **Error detection**: if one baseline disagrees, it flags a
  problem with that CORS station (antenna change, coordinate
  error) rather than with your setup.

### Available CORS (within ~15 km of lab)

Check which stations are closest and which have the best sky
coverage.  From the NTRIP sourcetable:

**Port 12055 (DuPage)**:
- WHEATON-RTCM3 — closest? Check coordinates
- NAPERVILLEPD-RTCM3
- ELMHURST-RTCM3
- WOODDALE-RTCM3
- HANOVERPARK-RTCM3
- KNOLLWOOD-RTCM3

**Port 12054 (ISTHA)**:
- NAPERVILLE-RTCM3.1-MSM5 — ~5 km, GPS+GLO+GAL
- BOLINGBROOK-RTCM3.1-MSM5
- BENSENVILLE-RTCM3.1-MSM5
- SCHAUMBURG-RTCM3.1-MSM5

Use 2-3 closest stations.  Naperville (ISTHA) is preferred because
it provides MSM5 (full carrier-phase, multi-constellation).

### Procedure

1. **Collect simultaneous observations** from your F9T and 2-3
   CORS stations for 24 hours.
2. **Process baselines** using RTKLIB (or similar) in static
   post-processing mode.
3. **Compare positions**: all baselines should give the same
   antenna position (in the same reference frame) to ±1 cm.
4. **Absolute position**: use the CORS published coordinates
   (in NAD83 or ITRF) plus your computed baseline to derive
   your antenna position in the same frame.

### Baseline Processing Notes

- At 5-10 km baseline, L1/L2 iono-free combination resolves
  integer ambiguities within minutes.
- Fix rate should be >99% for a 24-hour session at this baseline.
- The Leica's L1/L2 observations (from the zero-baseline experiment)
  can also be used for baseline processing, providing a second
  independent check.


## Experiment 3: Cable Delay Measurement

### Method A: PPS Comparison (preferred)

With the zero-baseline setup, the TICC measures:

    TICC_diff = PPS_F9T - PPS_Leica
              = (cable_F9T + delay_F9T) - (cable_Leica + delay_Leica)

If both cables are the same length (same splitter output):

    TICC_diff ≈ delay_F9T - delay_Leica

The Leica's PPS delay is published.  Solving for `delay_F9T` gives
the F9T's total delay (internal + cable).

If cables are different lengths, measure the difference with a TDR
or swap cables and re-measure (the cable difference reverses, the
receiver difference doesn't).

### Method B: Cable-Swap

1. Measure PPS_F9T on TICC with cable A
2. Swap to cable B (different length)
3. The TICC difference = cable_B - cable_A
4. Measure cable_A and cable_B independently (TDR or VNA)
5. Cross-check: does the measured cable difference match the
   TICC difference?

### Method C: TDR/VNA

If a time-domain reflectometer or vector network analyzer is
available, measure each cable's electrical length directly.
Typical coax velocity factor: 66% (RG-58) to 85% (LMR-400).
A 10m cable at 66% VF has 50.5 ns delay.

### Target

Know the total signal path delay (antenna → receiver PPS output)
to ±1 ns.  This sets the accuracy of the PPS alignment to GPS
time.  The components:
- Cable delay: 3-5 ns/m × cable length
- Receiver internal delay: ~28 ns (F9T, see reference below)
- Antenna phase center: ~mm level, negligible at ns scale
- Splitter: ~1 ns insertion delay (characterize from VNA S21)

**Do this per signal, not once for the chain.**  Record the delay
separately for each code we track (L1 C/A, L5Q, E1, E5a) and derive the
inter-frequency difference.  The common part of a delay error passes
through the ionosphere-free combination at ×1; the inter-frequency part
is amplified ×2.26 (L1/L5) to ×2.98 (L1/L2).  A single chain-total
figure hides the term that actually propagates.  See "Dual-frequency
amplifies the DIFFERENTIAL part of your calibration" below.

### Independent F9T delay reference

Ricardo Píriz at GMV (Madrid) published F9T timing calibration
measurements on LinkedIn (2019-2020):

- **F9T device internal delay: ~28 ns**
- Full chain (antenna 16 ns + cable 52 ns + device 28 ns): 95.9 ns
- Day-to-day repeatability: 0.3 ns (1σ) over one week
- PPS jitter: ±4 ns (larger than the older M8F's ±2 ns)
- More rigorous calibration (April 2020): 93.9 ns total chain
- **Antenna: Harxon CHX600A** (low-cost multi-band), GMV rooftop —
  the same antenna fed both the F9T and the reference receiver

The ~28 ns internal delay is our benchmark.  Our zero-baseline
experiment should produce a consistent value when we subtract the
known cable and antenna delays.

**Note the 2 ns spread between his own two numbers** (95.9 ns Aug 2019
→ 93.9 ns Apr 2020).  That is the same magnitude as the stated
uncertainty of the relative-calibration method below, and is the
realistic floor for any delay figure adopted from publication rather
than measured against a reference in hand.

### The peer-reviewed companion — and the method that matters

The LinkedIn posts are the accessible summary; the substantive
write-up is **Píriz, Garbin, Díaz & Defraigne, "Scalable, Traceable
Time for Datacenters Using GNSS and White Rabbit," Inside GNSS**.
Defraigne is at ROB (the BIPM-side timing authority), which is where
the traceability claim comes from.  It is also, structurally, the
architecture this project is heading for: GNSS-disciplined units with
White Rabbit distribution.

Two calibration methods, and the distinction is load-bearing:

| method | how | uncertainty |
|---|---|---|
| **Absolute (AKAL)** | GNSS signal simulator for the receiver, VNA for antenna/cable, Compact Antenna Test Range | ~1 ns, whole chain |
| **Relative (practical)** | Two receivers on one rooftop, time-interval counter on the two 1PPS outputs, known cable delays subtracted | ~2.0 ns |

Chain delays reported there (a different receiver from the F9T — do
**not** conflate with the 93.9 ns above): antenna + receiver 90 ns,
receiver alone 68 ns, antenna 22 ns.  White Rabbit distribution over
multi-hop fibre: ~1 ns error, 33 ps standard deviation.

**The relative method is Experiment 1 of this document** — same
geometry, same instrument, same arithmetic.  What we lack is its
premise: *an absolutely calibrated reference receiver*.

### Adopting a published delay — what it does and does not buy

Replicating Píriz's chain (CHX600A + F9T, matched signals) does not
make our receiver calibrated.  It makes our chain the same *shape* as
his, so his published delay can be adopted as a transfer standard —
calibration by design similarity.  That is a legitimate technique and
it is the cheapest route to a lab reference without a national-lab
visit, but the uncertainty is strictly worse than his 2 ns because it
inherits, on top of his:

1. **Per-signal group delay.**  Receiver delay is code-dependent.  The
   adopted number is only valid for the exact signal/code set he used;
   an L1+L5 chain is not an L1 C/A chain.  Record the signal set
   alongside any delay figure — a bare "28 ns" is not a calibration.
2. **Unit-to-unit variation.**  His 0.3 ns is *day-to-day
   repeatability of one unit*, not spread across units.  Our own
   [receiver-comparison-2026-06-01.md](receiver-comparison-2026-06-01.md)
   found F9T per-unit variance exceeding the F9P-vs-F9T gap.
3. **Firmware.**  His work is 2019-2020 (TIM 2.20 era); F9T-20B ships
   TIM 2.25.
4. **Cable and splitter.**  Measure, do not assume — velocity factor
   varies by part.

Once one receiver carries an adopted delay, the rest of the fleet is
relatively calibrated against it by Experiment 1, and the fleet is
self-consistent even where the absolute figure is off.  **For
two-clock PPS agreement a common bias cancels exactly** (see
[two-clock-agreement-forward-model.md](two-clock-agreement-forward-model.md)
§5), so the absolute term only matters for outward-facing time
delivery — where ~2 ns is comfortably inside the 50-300 ns of
uncalibrated PHY asymmetry that PTP-over-copper carries anyway.  A
national-lab absolute calibration becomes worth its cost in the White
Rabbit era, not before.

References:
- [Testing the new ublox F9T (part 2)](https://www.linkedin.com/pulse/testing-new-ublox-f9t-part-2-ricardo-p%C3%ADriz) — Aug 2019
- [Calibrating mass-market GNSS timing receivers](https://www.linkedin.com/pulse/calibrating-mass-market-gnss-timing-receivers-ricardo-p%C3%ADriz) — Apr 2020
- [Ublox F9T: testing in the lab](https://www.linkedin.com/pulse/f9t-testing-lab-ricardo-p%C3%ADriz/) — Píriz
- [Scalable, Traceable Time for Datacenters Using GNSS and White Rabbit](https://insidegnss.com/scalable-traceable-time-for-datacenters-using-gnss-and-white-rabbit/) — Píriz, Garbin, Díaz, Defraigne, Inside GNSS


### Dual-frequency amplifies the DIFFERENTIAL part of your calibration

Píriz's conclusion — that "the usage of dual-frequency GPS for timing is
not extremely advantageous" because "receiver noise and multipath error
gets amplified by a factor of 3 in the iono-free combination", and that
"the residual GPS calibration error is roughly doubled" — is exactly
right for his problem and is the single most useful thing to carry into
ours.  It changes *what* we record, not *whether* we run dual-frequency.

#### Where the 3× comes from

The ionosphere-free combination is a weighted difference chosen so the
1/f² iono terms cancel:

```
IF = α·P₁ + β·P₂        α = f₁²/(f₁²−f₂²)       β = −f₂²/(f₁²−f₂²)
```

Independent errors of equal σ on the two frequencies add in quadrature,
so `σ_IF = σ·√(α²+β²)`.  It is a lever-arm extrapolation to a virtual
frequency *outside* the interval between the two, and lever arms
amplify.  **L1/L5 is materially cheaper than L1/L2** because L5 is
farther from L1 — relevant to the L5-tracking fleet:

| pair | α | β | independent-error amplification |
|---|---|---|---|
| GPS L1/L2 | +2.546 | −1.546 | **×2.978** |
| GPS L1/L5, GAL E1/E5a | +2.261 | −1.261 | **×2.588** |

#### Why calibration only *doubles* — and the consequence for this plan

**α + β = 1 exactly**, by construction (the combination must preserve
true range).  So the amplification depends entirely on the *correlation
structure* of the error, not on its size:

| error structure | amplification (L1/L2) |
|---|---|
| independent per-frequency | ×2.98 |
| **perfectly common-mode** | **×1.00 — not amplified at all** |
| L1-only | ×2.55 |
| L2-only | ×1.55 |

Píriz's "roughly doubled, depending on the calibration technique
employed" is the signature of *partially correlated* error.  Whatever is
shared between the two frequencies — the antenna, the cable, the
connector, the measurement technique — passes through at ×1.  What gets
amplified is the **inter-frequency (DCB-like) residual**.

**Therefore: record per-frequency delays, never just the chain total.**
A bare "28 ns" is not a calibration.  Two F9Ts with identical absolute
delay but a 1 ns difference in their L1−L5 delay disagree by ≈2.3 ns in
the IF solution.  That differential is also the part most likely to vary
unit-to-unit and across firmware, because it lives in the correlator and
filter chains rather than the shared RF front end — where the absolute
delay mostly lives.  This is the sharpest form of the per-signal caveat
in the previous section.

#### Why we stay dual-frequency anyway

The 3× is applied to whatever you feed it, and we feed it carrier phase:

| observable | σ | after ×2.98 |
|---|---|---|
| raw code | 40 cm | 1.19 m = **3.97 ns** |
| smoothed code | 10 cm | 30 cm = **0.99 ns** |
| **carrier phase** | 2 mm | 6 mm = **0.02 ns** |

Amplified code noise *is* Píriz's error budget; amplified phase noise is
~30 ps against our 354 ps per-clock budget — a rounding error.  Beyond
that, his figure of merit is *absolute* uncertainty vs UTC, where a bias
is the error; ours is differential and stability-shaped, and a common
bias contributes exactly zero to two-clock p95 |Δ| (see
[two-clock-agreement-forward-model.md](two-clock-agreement-forward-model.md) §5).

And the reasons we need a second frequency are mostly not about the
ionosphere at all:

- **Cycle-slip detection** — the GF detector is inherently dual-frequency.
- **Wide-lane / MW → PPP-AR** — dual-frequency by definition.  No second
  frequency, no ambiguity resolution, no cm-class ARP.
- **SSR and broadcast clocks are IF-referenced.**  Going single-frequency
  does not dodge the problem; it forces a TGD/DCB conversion back to
  single-signal, which is another calibration term with its own error —
  and one we already know is fragile (see
  [bds-ppp-integration.md](bds-ppp-integration.md) for the B3I-referenced
  BDS clock, and the GPS L5 code-bias gap-fill).

#### A caveat on his caveat

His closing "should be taken with caution... depending on the solar
activity" carries more weight than it appears to.  Those articles are
**Aug 2019 and Apr 2020**, and the solar cycle 24/25 minimum was
**December 2019** — he measured at essentially the most favourable moment
in an 11-year cycle for single-frequency timing.  Cycle 25 peaked around
late 2024.  "Moderate solar activity" in his data is quieter than
anything a box deployed now will see.

## Experiment 4: Antenna Delay

### Why you can't just look it up

The absolute group delay of a *specific antenna model*, as a number in
ns, is essentially never published.  This is structural, not an
oversight: the CGGTTS/BIPM framework **defines the antenna into the
receiver**.  From PTB's 2022 BIPM calibration report:

> The sum XR + XS represents the "INT DLY" field in the CGGTTS header:
> XR represents the receiver hardware delay… [XS is the] antenna delay,
> between the phase center and the antenna cable connector at the
> antenna.

So the hundreds of public BIPM Group-1/Group-2 calibration reports each
give a number valid only for *that antenna bolted to that receiver*.
The antenna is never broken out.  Same reason Píriz reports 93.9 ns
(F9T) and 77.9 ns (mosaic) as chain totals and attributes the 16 ns
difference to the receivers — "same antenna used".

Two sources look like the answer but are not:

- **Absolute GDV catalogs** (Geo++ robot calibrations, IfE/Hannover,
  Wübbena et al., IGS Repro3) give antenna group delay vs
  elevation/azimuth — but as *variations*, mean removed.  The constant
  we want is exactly what's been differenced away.
- **IGS/NGS ANTEX** (`igs20.atx`, NGS ANTCAL) is PCO/PCV in mm.  No
  group delay at all.

### Published measured values

The best free source is **JRC EUR 40306**, Tegedor & Sgammini (2025),
*Absolute and relative calibration of GNSS timing receiver chains*.
Its Table 8 is a straight absolute antenna calibration (ns; 3 m
far-field, 10.01 ns propagation term removed, tare ≈ 33 ns):

| Rx antenna | L1/E1 | L2 | E5a/L5 |
|---|---|---|---|
| Horn (control) | 0.09 | 0.18 | 0.58 |
| BP2G | 21.47 | 15.81 | 20.47 |
| JRCT | 19.32 | 15.27 | 19.50 |
| BP2D | 17.69 | 20.49 | 24.99 |

The antennas are identified by station name, not model — JRCT is
described only as "a high-end choke-ring antenna" feeding a PolaRx5TR.
Three conclusions matter more than the specific models:

1. **Magnitude: 15–25 ns**, dominated by the LNA + SAW filter.  Píriz's
   antenna term (16 ns, above) sits in the same family.
2. **Strongly frequency-dependent.**  BP2D swings 17.69 → 24.99 ns
   between L1 and L5 — 7.3 ns across the bands we combine iono-free.
   A single scalar "antenna delay" is not a sufficient model for us.
3. **Same class ≠ same number.**  Three choke-ring-class chains spread
   17.7–21.5 ns at L1.  Buying "the model someone else measured" only
   helps if it's the identical model, and unit-to-unit spread within a
   model is unquantified anywhere we could find.

Manufacturer datasheets almost never print the number.  The one
exception found: Calian/Tallysman **TW3742AJ** specifies "Group Delay:
17 ns @ GPS-L1 | <1.0 ns @ GLONASS-G1".  Plausible against the JRC
table, but type-typical rather than unit-specific, no traceability or
uncertainty statement, L1 only, and the second figure is clearly ripple
rather than absolute — the two aren't the same quantity.  Sanity check,
not a calibration.

### The horn-pair method — no calibrated reference antenna needed

Píriz measured his antenna against a previously calibrated reference.
The JRC method needs no such prerequisite — it bootstraps absolute
delay from geometry plus a symmetry assumption.  Anechoic chamber + VNA,
three steps:

1. Connect the RF cables directly through a transition → **tare delay**
   (cables + bias-tee).
2. Install **two identical horns** as Tx and Rx at 3 m (far-field).
   Each horn's delay = (total − tare − 3 m propagation) / 2.
3. Swap the antenna under test in as Rx.  Its delay = total − tare −
   3 m propagation − horn delay.

The horn-vs-horn control row (0.09–0.58 ns) is the self-consistency
check.  NIST's Plumb & Larson use the same chamber+VNA approach with a
flat metal sheet for the reflected-peak reference, quoting 0.2 ns
single-frequency uncertainty — and explicitly flag as a weakness that
"only one antenna was calibrated", recommending all antennas used for
geodetic time transfer be done.  Two decades on that's still largely
unfulfilled, which is why the lookup table doesn't exist.

Both variants need anechoic chamber access, which the lab does not have.

### What we can measure without a chamber — and why it's enough

The moonshot target is **two-clock PPS agreement**, not UTC
traceability.  For that, absolute antenna delay is common-mode and
cancels when both boxes run the same antenna model; what actually
threatens the 1 ns / 2 ns excursion budget is the **difference** between
the two boxes' antennas.  That we can measure with no external
calibration at all:

    Same receiver, same cable, same sky.  Measure PPS vs the TICC with
    antenna A, swap to antenna B, re-measure.  Receiver and cable terms
    cancel; the residual is the antenna-to-antenna delta.

Run it as an A/B/A to cancel DO drift across the swap, and take the
delta per band (L1 and L5 separately — see the 7.3 ns frequency spread
above).  The GUS-splitter shared-antenna config gives the
zero-difference control case: two receivers on one antenna must show a
delta consistent with zero.

This bounds the term that matters for cross-host agreement, works on any
antenna we can get, and needs no chamber, no reference antenna, and no
calibration lab.  Absolute delay only becomes load-bearing if we want
PPS OUT traceable to UTC — a separate mission.

### HX-CHX600A

An **HX-CHX600A** is on order for the lab (~$200, 2026-08).  This is the
antenna Píriz used, so it lets us reproduce his *chain* calibration
end-to-end (F9T + HX-CHX600A + measured cable → compare against his
93.9 ns) as an independent check on our own delay budget.  It does not
hand us an antenna constant — he didn't publish one separable from his
reference chain.  Its practical value here is as the **second antenna**
in the swap measurement above: HX-CHX600A vs CHOKE1/UFO1 gives the delta
directly, and the same swap against a future second unit would give the
unit-to-unit spread the literature is silent on.

References:
- [JRC EUR 40306 — Absolute and relative calibration of GNSS timing receiver chains (2025)](https://publications.jrc.ec.europa.eu/repository/bitstream/JRC139176/JRC139176_01.pdf)
- [PTB G1G2_1201_2022 BIPM calibration report](https://webtai.bipm.org/ftp/pub/tai/publication/gnss-calibration/group2/2022/1201-2022/PTB_G1G2_1201_2022_Calibration_Report_V1.pdf)
- [BIPM guidelines for GNSS equipment calibration](https://webtai.bipm.org/ftp/pub/tai/publication/gnss-calibration/guidelines/archive/annex-1_operational-procedures-20210216.pdf)
- [Plumb & Larson, *Absolute Calibration of a Geodetic Time Transfer System*](https://www.kristinelarson.net/wp-content/uploads/2015/10/paperIrevise2.pdf)
- [Garbin, Defraigne, Krystek, Píriz, Bertrand, Waller — Metrologia 56(1), 2018](https://iopscience.iop.org/article/10.1088/1681-7575/aaf2bc) (paywalled; peer-reviewed backing of Píriz's method)
- [Estimation of absolute GNSS satellite antenna GDV — GPS Solutions (2021)](https://link.springer.com/article/10.1007/s10291-021-01137-8)
- [Tallysman TW3742AJ datasheet](https://www.tallysman.com/app/uploads/2021/06/Tallysman%C2%AE-TW3742AJ-Datasheet-Rev.-202208.pdf)


## Data Products

After all three experiments:

| Product | Source | Accuracy |
|---------|--------|----------|
| Antenna ARP position (ITRF) | 24h PPP + CORS baseline | ±1 cm H, ±2 cm V |
| Antenna PCO/PCV | ANTEX file (or zero-baseline cal) | ±1 mm |
| F9T carrier-phase noise | Zero-baseline DD residual | measured |
| F9T PPS internal delay, **per signal** (L1 C/A, L5Q, E1, E5a) | PPS comparison vs Leica | ±2 ns |
| F9T **inter-frequency** delay (L1−L5) | difference of the above | ±1 ns target — amplified ×2.26 in IF |
| Cable delay (ant → F9T) | TDR or PPS swap | ±1 ns |
| Antenna-to-antenna delay delta | A/B/A antenna swap (Exp 4) | TICC-limited, per band |
| Absolute PPS-to-GPS alignment | position + cable + internal | ±2 ns |

These calibration products enable:
- Correct PPS-to-TAI alignment for time transfer
- Accurate PPP filter initialization
- Confidence in TDEV measurements (known systematic biases)


## Quick Position Check: RTKLIB PPP on a Pi

Experiments 1–3 above are comprehensive calibration campaigns.  For
a quicker **second-opinion position check** (±5 mm H, ±15 mm V) using
only the existing F9T and a Raspberry Pi, RTKLIB can do standalone PPP
post-processing without any reference station or Leica.

### Software

Use the [rtklibexplorer fork](https://github.com/rtklibexplorer/RTKLIB)
(RTKLIB-EX 2.5.0, successor to "demo5").  It has the best u-blox
dual-frequency support.

```bash
git clone https://github.com/rtklibexplorer/RTKLIB.git
cd RTKLIB/app/convbin/gcc && make        # UBX → RINEX
cd ../../rnx2rtkp/gcc && make            # post-processing PPP
```

Both build cleanly on aarch64 Raspberry Pi OS.

### Procedure

1. **Log 2–4 hours of raw observations** from the F9T.  peppar-fix
   already captures RXM-RAWX; alternatively use `str2str` or a simple
   UBX logger.  Longer is better — 24h gives mm-level, 2h gives ~1 cm.

2. **Convert to RINEX**:

       ./convbin -r ubx -o obs.rinex raw_log.ubx

3. **Download IGS precise products** for the observation date from
   [CDDIS](https://cddis.nasa.gov/archive/gnss/products/) or
   [IGN](http://igs.ign.fr/products):
   - **SP3** (precise orbits): `igsWWWWD.sp3` or rapid `igrWWWWD.sp3`
   - **CLK** (precise clocks): `igsWWWWD.clk` or rapid `igrWWWWD.clk`
   - Final products appear ~12–18 days after observation; rapid
     products in ~1 day.  For a quick check, rapid is fine.

4. **Run PPP-Static**:

       ./rnx2rtkp -p 7 -sys G,E obs.rinex nav.rinex \
           -sp3 igsWWWWD.sp3 -clk igsWWWWD.clk -o ppp_result.pos

   `-p 7` selects PPP-Static.  `-sys G,E` processes GPS+Galileo
   (matching our F9T signal config).

5. **Compare** the RTKLIB solution against peppar-fix's
   `data/position.json`.  Agreement within 2–3 cm confirms both
   are correct.  A larger discrepancy warrants investigation
   (antenna PCO difference, troposphere model, observation quality).

### What this gives vs doesn't give

| Provides | Does not provide |
|----------|------------------|
| Independent position in ITRF (±5 mm H / ±15 mm V with 24h) | Cable delay |
| Cross-check of peppar-fix PPP position | F9T internal delay |
| Completely different software + correction products | PPS alignment to GPS time |
| No additional hardware needed | Carrier-phase noise floor (need zero-baseline) |

This is not a substitute for the full calibration campaign — it only
validates the antenna position.  But it's a useful quick sanity check
that can be done today with existing hardware.

### Notes

- RTKLIB uses IGS final/rapid products (orbit + clock files), which
  are different from the SSR corrections peppar-fix uses in real time.
  This makes the position fix genuinely independent.
- The F9T's L5 signal config (GPS L1+L5, GAL E1+E5a) is fully
  supported by RTKLIB-EX.
- `rtkrcv` can also do real-time PPP if you feed it the F9T's serial
  stream + an NTRIP SSR correction stream, but post-processing with
  final products gives a better position.


## Dependencies

- [ ] Identify choke ring antenna model, verify ANTEX entry
- [ ] Set up Leica GRX 1200 Ethernet access, test RTCM output
- [ ] Acquire splitter (Wilkinson preferred for isolation)
- [ ] Verify Leica PPS output availability and connector type
- [ ] Determine closest CORS stations and coordinates
- [ ] Install RTKLIB on a lab Pi for baseline processing and PPP
- [ ] HX-CHX600A on order (~$200, 2026-08) — Píriz chain cross-check +
      second antenna for the Experiment 4 swap
