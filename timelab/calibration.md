# Calibration & Measurement Validation

Track work planned and completed to quantify reproducibility and error bounds
of timelab measurements.

## References

- [DCC 2016: The TAPR TICC Counter](https://files.tapr.org/meetings/DCC_2016/DCC2016-The_TAPR_TICC_Counter_Measuriing_Trillionths_of_a_Second_with_an_Arduino_John_Ackerman_N8UR.pdf) —
  John Ackermann N8UR's original presentation. TI TDC7200-based, <60 ps
  single-shot resolution, ~70 ps jitter, Arduino Mega 2560.
- [TICC detailed measurements (leapsecond.com)](http://leapsecond.com/pages/ticc/) —
  Tom Van Baak's TICC performance characterization (site intermittently down).
- [TICC product page (febo.com)](https://www.febo.com/pages/TICC/) —
  N8UR's TICC page. 17-day test: no outliers >500 ps across 1.5M measurements.
- [multi-TICC Application Note 2020-01](https://files.tapr.org/tech_docs/multi-TICC_App_Note_2020-01.pdf) —
  John Ackermann's multi-TICC note. **Key finding**: unterminated TICC clock
  inputs caused reflections that translated into jitter when reference was
  daisy-chained via SMA tee connectors. Fix: drive each TICC from its own
  cable from a distribution amplifier, and install the termination jumper on
  each TICC to ensure good impedance match.
- [TICC firmware (GitHub)](https://github.com/TAPR/TICC) —
  Open source firmware. multi-ticc/ directory has the app note and config.
- [TICC Operation Manual (2025)](https://web.tapr.org/~n8ur/TICC_Manual.pdf) —
  Current manual (revised December 2025).

### Relevance to our setup

Our TICCs are each driven by individual buffered outputs from the SV1AFN
distribution amplifier via radial RG-316 runs — not daisy-chained via tees.
This matches the multi-TICC app note's recommended configuration. The
termination jumpers should be verified on each TICC.

## Measurement noise floor

### TICC single-shot resolution
- TAPR TICC specified: <60 ps single-shot, ~70 ps jitter
- 10 MHz reference: FE-5680A Rb standard via SV1AFN distribution amplifier
  (was Geppetto GPSDO OCXO until 2026-05-07; Rb wired in 2026-05-10 — Rb is
  undisciplined/slightly off 10 MHz but far better short-term TDEV than the OCXO)
- Both TICC channels share the same reference, so cross-channel measurements
  cancel reference oscillator noise

### Known noise contributors
- **TICC quantization**: ~60 ps (spec)
- **Reference oscillator drift**: cancels in cross-channel (chA−chB) but
  appears in individual channel TDEV at long τ. Both TICCs share the same
  10 MHz chain. Observed: ~1.7 ns TDEV at τ=500-1000s (common to both channels)
- **Cabling**: RG-316 + SMA throughout, but signals connect to bare PCBs
  (TICC Arduino, TimeHAT, RPi5). No shielding at board-to-board interfaces.
- **EMI**: Bare RPi5 within inches of bare TICC. Pi5 runs at 2.4 GHz with
  active cooling fan. Potential coupling into TICC analog front-end or
  PPS signal paths.
- **PPS signal integrity**: SDP0/SDP1 are 3.3V LVCMOS from the i226, going
  through SMA J4 → RG-316 → TICC SMA input. No termination or level shifting.

### Noise floor measurement plan

1. **TICC self-consistency (same signal, both channels)**
   - Split a single PPS source into both TICC channels via a tee
   - The resulting TDEV(τ) is the TICC + cabling noise floor
   - Do this for each TICC independently
   - Expected: dominated by TICC quantization (~60 ps) and cable mismatch

2. **Cross-TICC agreement (same signal, different TICCs)**
   - Feed one PPS source to chA of TICC #1 and chA of TICC #3
   - Compare individual-channel TDEV — should agree if reference chain is clean
   - Differences reveal reference distribution path noise or EMI pickup
   - **Priority**: explains the TICC #1 vs #3 discrepancy (0.98 ns vs 3.40 ns
     for the same disciplined PHC PPS)

3. **Cable delay calibration**
   - Measure mean chA−chB offset with a known zero-delay configuration
     (same source, matched cables)
   - Current measured offset: ~29 ns (PHC PPS vs F9T PPS on TimeHat)
   - Separating cable delay from PHC offset requires a common-source test

## Calibration Run C1 — Self-consistency (2026-03-16)

**Setup**: Single F9T PPS → TADD-2 3-way active buffer → Wilkinson divider
(out1) + 2 SMA tees (out2, out3) → all 6 TICC channels. Same PPS signal
on every channel. 10-minute capture, all 3 TICCs simultaneous.

### Differential TDEV(1s) — the self-consistency metric

| TICC | Host | diff TDEV(1s) | diff ADEV(1s) | chA−chB std | chA−chB mean |
|------|------|---------------|---------------|-------------|--------------|
| **#2** | **Onocoy (Pi 4)** | **0.49 ns** | **0.85 ns** | **0.52 ns** | -27.1 ns |
| #1 | PiPuss (Pi 5) | 5.63 ns | 9.74 ns | 5.59 ns | -79.7 ns |
| #3 | TimeHat (Pi 5+HAT) | 5.17 ns | 8.95 ns | 5.17 ns | +52.9 ns |

### Key finding: TICC unit-to-unit variation dominates, not the Pi

Initial C1/C3 results suggested Pi 5 EMI was the cause (both Pi 5 hosts
showed ~5 ns vs 0.5 ns on Pi 4). However, a **swap test** (S1) where
TICC #2 and #3 exchanged hosts proved otherwise:

| TICC | C1 host (diff TDEV 1s) | S1 host (diff TDEV 1s) | Follows |
|------|------------------------|------------------------|---------|
| #2 | Onocoy/Pi4 (0.49 ns) | TimeHat/Pi5 (1.08 ns) | **TICC** |
| #3 | TimeHat/Pi5 (5.17 ns) | Onocoy/Pi4 (5.47 ns) | **TICC** |
| #1 | PiPuss/Pi5 (5.63 ns) | PiPuss/Pi5 (5.30 ns) | control |

**The noise follows the TICC unit, not the Pi host.** TICC #3 is ~5 ns
regardless of which Pi it's on. TICC #2 is sub-ns on either host.

TICC #2 did degrade slightly on the Pi 5 (0.49 → 1.08 ns), suggesting a
modest Pi 5 contribution (~0.5 ns), but the dominant factor is **TICC
unit-to-unit variation** — likely TDC7200 chip tolerance, solder quality,
component matching, or Arduino crystal quality.

This means the TICC spec of "<60 ps single-shot" is a best-case; real
units may vary by 10× or more in differential measurements. TICC #2 is
the only trustworthy unit for sub-ns measurements in this lab.

### Full rotation matrix (C1, C3, S1, S2)

Every TICC tested on multiple hosts. Swap tests S1 and S2 moved TICCs
between hosts to separate TICC-unit effects from host-environment effects.

**diff TDEV(1s) — same PPS to both channels:**

| TICC | Onocoy (Pi 4) | TimeHat (Pi 5) | PiPuss (Pi 5) |
|------|---------------|----------------|---------------|
| **#2** | **0.49 ns** (C1) | **1.08 ns** (S1) | **0.97 ns** (S2) |
| #3 | 5.47 ns (S1) | 5.17 ns (C1) | 5.21 ns (S2) |
| #1 | — | 5.74 ns (S2) | 5.63 ns (C1) |

C3 (repeat of C1) confirmed reproducibility within ±11% statistical bounds.

**Conclusion**: The noise follows the TICC unit, not the host. TICC #2 is
consistently sub-1.1 ns on every host. TICCs #1 and #3 are ~5 ns everywhere.
A modest Pi 5 contribution exists (~0.5 ns, visible in TICC #2: 0.49 on
Pi 4 vs ~1.0 on Pi 5) but is 10× smaller than the unit-to-unit variation.

Possible causes for unit variation: TDC7200 chip lot, solder quality,
decoupling capacitor values, Arduino crystal, or ground loop susceptibility.

### Battery power test (BAT1) — TICC #3 on PiPuss

Powered TICC #3 from 12V battery pack via barrel jack (bypasses USB 5V
regulator). USB data cable still connected (ground path intact).

| Power source | diff TDEV(1s) | chA−chB std |
|--------------|---------------|-------------|
| USB (S2 baseline) | 5.21 ns | 5.15 ns |
| Battery (BAT1) | 4.46 ns | 4.34 ns |

~15% improvement — modest. USB power noise contributes some, but is not
the dominant source. The remaining ~4.5 ns is either intrinsic to the TICC
board or enters through SMA signal/ground paths.

Next: USB isolator (arriving tomorrow) to break the USB ground loop entirely.
If that doesn't help, the noise is intrinsic to TICC #3's hardware.

### Wilkinson vs SMA tee comparison (WIL1)

Swapped the Wilkinson divider from TADD-2 out1 to out3, so TICC #2's
channels swap which splitter type they go through.

| Splitter on TICC #2 chA/chB | chA−chB mean | std | diff TDEV(1s) |
|------------------------------|-------------|-----|---------------|
| Wilkinson / tee (USB3) | -27.8 ns | 1.55 ns | 1.52 ns |
| tee / Wilkinson (WIL1) | -43.7 ns | 1.49 ns | 1.49 ns |

**The 16 ns mean shift** is the Wilkinson's extra propagation delay from its
meander structure. The **noise is identical** — Wilkinson and SMA tee perform
the same. The isolation resistor neither helps nor hurts at our noise levels.

Conclusion: for PPS splitting, SMA tees and Wilkinson dividers are
interchangeable. The dominant noise source remains the TICC unit itself.

### Same-leg test (LEG1) and tee-tree test (TEE1) — TADD-2 is the culprit

Fed both channels of TICC #2 from the same TADD-2 output (LEG1), then
bypassed the TADD-2 entirely and fed all 6 channels through a tree of
SMA tees from a single PPS source (TEE1).

**LEG1: TICC #2 both channels from same TADD-2 output**

| Config | chA−chB std | diff TDEV(1s) |
|--------|-------------|---------------|
| Different TADD-2 outputs | 1.49 ns | 1.49 ns |
| Same TADD-2 output | **0.050 ns** | **0.050 ns** |

30× improvement. The 1.5 ns was TADD-2 output-to-output jitter, not the TICC.

**TEE1: All 6 channels from single PPS through tee tree (no TADD-2)**

| TICC | With TADD-2 | Tee tree | Improvement |
|------|-------------|----------|-------------|
| #2 (Onocoy/Pi4) | 1.49 ns | **52 ps** | 29× |
| #3 (PiPuss/Pi5) | 5.21 ns | **55 ps** | 95× |
| #1 (TimeHat/Pi5) | 4.13 ns | **783 ps** | 5× |

**TICCs #2 and #3 both measure 50-55 ps — right at the published TICC spec.**
They were never "dirty" units. The entire 5 ns attributed to TICC unit
variation was actually the modified TADD-2 buffer's output-to-output jitter.

TICC #1 at 783 ps is still elevated. Possible causes: position in tee tree
(more downstream reflections), a genuine per-unit hardware difference, or
its host (TimeHat/Pi5) adding noise. Further testing needed.

**Revised conclusion**: The TADD-2 modification (3 individual outputs instead
of paralleled outputs) introduces ~1-5 ns of output-to-output jitter. This
is unsuitable as a PPS distribution buffer for sub-ns calibration work.
Simple passive SMA tee trees work better for self-consistency testing
where all channels need the same signal.

### Per-channel TDEV(1s) — reference oscillator wander

Individual channel TDEV(1s) is ~31-35 ns on all three TICCs, dominated by
the reference oscillator's short-term phase noise (common to both channels,
cancels in the differential measurement).

> **STALE — re-measure.** These ~31-35 ns single-channel numbers are from the
> Geppetto GPSDO OCXO era.  The TICC reference is now the FE-5680A Rb standard
> (since 2026-05-10), whose short-term TDEV is far lower, so per-channel
> (single-shot, absolute) TDEV should now be much better — re-run the
> per-channel survey against the Rb.  Differential (chA−chB) numbers are
> unaffected (the reference cancels either way).

### Implications for servo measurements

The overnight M5 servo TDEV(1s) of 3.30 ns was measured on TICC #3
(5 ns self-consistency floor). The measurement is noise-floor-limited —
the true servo TDEV(1s) may be significantly better than 3.30 ns.

To get a trustworthy servo TDEV(1s), we need to measure with TICC #2
(1 ns floor on Pi 5, 0.5 ns on Pi 4). TICC #2 should be the primary
instrument for all sub-ns servo characterization.

### Previous comparison (pre-calibration, different signals)

Earlier 5-minute runs with the servo running (different PPS on each channel):

| Metric | TICC #3 (TimeHat) | TICC #1 (PiPuss) | Notes |
|--------|-------------------|-------------------|-------|
| TDEV(1s) chA (PHC PPS) | 3.30 ns | 0.98 ns | Same PPS, different TICC |
| TDEV(1s) chB (raw F9T) | 1.97 ns | 2.15 ns | Different F9T, different antenna |

The 0.98 ns on TICC #1 was measured during a period when EMI was lower
(different processes running, different CPU load). The calibration run
shows the typical Pi 5 noise is ~5 ns.

## Overnight A/B servo comparison (2026-03-16)

**Metric of interest: individual chA TDEV** — this is the disciplined PHC PPS
stability as measured by the TICC. The differential (chA−chB) TDEV is not a
good servo quality metric because it includes the raw PPS noise we're filtering
out; it measures the measurement system, not the servo.

### Run A: baseline gains (kp=0.3, ki=0.1)
- Duration: 3.94h, 14191 paired seconds
- chA−chB mean: +29.2 ns, std: 9.4 ns
- **Individual chA TDEV(1s): 3.30 ns** (disciplined PHC)
- Individual chB TDEV(1s): 1.97 ns (raw F9T-3RD GPS PPS)
- Individual chA TDEV(1000s): 1.75 ns
- Individual chB TDEV(1000s): 1.77 ns (both reference-limited)

### Run B: low-noise gains (kp=0.1, ki=0.03, EMA=0.3, adaptive)
- Duration: 3.94h, 14191 paired seconds
- chA−chB mean: +29.2 ns, std: 29.1 ns (3× worse)
- **Individual chA TDEV(1s): 2.87 ns** (slightly better short-τ)
- Individual chA TDEV(5s): 17.4 ns, TDEV(10s): 30.6 ns (severe resonance)
- Individual chA TDEV(1000s): 1.85 ns

### Full TDEV table (Run A — recommended baseline)

| τ (s) | chA disciplined | chB raw GPS |
|-------|-----------------|-------------|
| 1 | 3.30 ns | 1.97 ns |
| 2 | 4.71 ns | 1.60 ns |
| 5 | 7.24 ns | 1.16 ns |
| 10 | 6.00 ns | 0.88 ns |
| 20 | 2.78 ns | 0.63 ns |
| 50 | 1.23 ns | 0.50 ns |
| 100 | 0.89 ns | 0.65 ns |
| 200 | 1.08 ns | 0.98 ns |
| 500 | 1.66 ns | 1.65 ns |
| 1000 | 1.75 ns | 1.77 ns |

### Conclusion
Run A gains (kp=0.3, ki=0.1) are near-optimal. Run B's lower gains reduce
TDEV(1s) slightly (2.87 vs 3.30 ns) but create a severe mid-tau resonance.
The disciplined PHC shows a characteristic servo hump at τ=2-10s (PI response)
before settling below the raw GPS PPS at τ>50s. Both channels converge to
~1.7 ns at long τ, limited by the TICC reference oscillator.

The raw F9T PPS TDEV(1s) of ~2 ns without qErr correction is consistent with
carrier-phase reasoning: the PPP filter absorbs the F9T TCXO's phase noise
entirely, making qErr correction unnecessary.

## Calibration test plan

### Goal
Measure the noise floor and cross-TICC reproducibility of the three TICCs,
focusing on TDEV(1s). Published TICC specs: <60 ps single-shot, ~70 ps jitter.
Expected TDEV(1s) noise floor for white phase noise: ~35-40 ps.

### Run duration
**10 minutes per run** is sufficient for short-τ characterization.
With 600 samples at 1 Hz, the 95% confidence interval on TDEV(1s) is ±11%.
This resolves differences down to ~8 ps between TICCs — our actual
discrepancies are in the nanosecond range (3.30 vs 0.98 ns), so even
shorter runs would suffice. We use 10 minutes for margin and to get
clean TDEV out to τ~100s.

### Equipment
- **PPS source**: One EVK-F9T in timing mode (clean, survey-in complete)
- **Splitter**: Modified TAPR TADD-2 (active 3-way PPS buffer)
  - Original TADD-2 outputs are wired in parallel for better rise time;
    Bob's modified unit has 3 individually buffered outputs instead.
  - Each output drives one SMA tee → 2 copies each → 6 total.
  - Active buffering eliminates reflection coupling between TICCs (the main
    concern with passive tee splitting, where reflections from one TICC
    input travel back and appear at other inputs as correlated timing errors).
  - TADD-2 propagation delay/jitter is common-mode → cancels in all comparisons.
  - **Wiring constraint**: for self-consistency (chA vs chB on same TICC),
    the two channels must come from different TADD-2 outputs so tee-induced
    correlation doesn't fake good agreement.
- **Optional: Wilkinson divider as tee alternative** (on hand, worth testing):
  - Multi-section broadband PCB Wilkinson (0.5-4 GHz, SMA, 3-port).
  - Despite the RF frequency spec, there IS a clear DC path between all ports —
    the microstrip traces are just copper at low frequencies. The quarter-wave
    impedance transformation is irrelevant below 500 MHz.
  - At PPS frequencies: behaves as a low-loss 2-way split with a ~100Ω isolation
    resistor bridging the outputs. For a common-mode input (same signal to both
    outputs), the resistor carries zero current → essentially lossless.
  - The isolation resistor damps reflections between outputs — potentially better
    than a bare SMA tee for suppressing correlated timing errors.
  - **Plan**: use one Wilkinson alongside SMA tees in calibration runs to compare.
    If the Wilkinson-fed pair shows tighter cross-channel agreement, the
    isolation resistor helps.
  - Meander structures add ~1-2 ns propagation delay, but common-mode → cancels.
- **Alternatives considered and rejected**:
  - *Passive SMA tees only (as sole splitter)*: reflection coupling between 4+
    inputs creates correlated timing errors in the regime we're measuring.
    SMA tees are fine as a secondary split from buffered TADD-2 outputs.
  - *Resistive Y dividers (50Ω, -6 dB)*: halves PPS to ~1.65V, marginal for
    TDC7200 VIH (1.7V); reduced slew rate increases jitter. Cascading two
    for 4-way gives 0.8V — won't trigger.
- **10 MHz reference**: Existing radial RG-316 runs from SV1AFN dist amp
  (already matches the multi-TICC app note's recommended configuration).
  Verify 50Ω termination jumper installed on each TICC before running.
- **TICCs**: #1 (PiPuss), #2 (Onocoy), #3 (TimeHat)

### Pre-flight checklist
- [ ] Verify 50Ω termination jumper installed on all 3 TICCs
- [ ] Measure and record cable lengths (PPS source → each tee output → each TICC)
- [ ] Verify F9T PPS source is in timing mode (fixType=5) with survey-in complete
- [ ] Stop any running servo, SatPulse, or other processes using TICC/PPS ports

### Test matrix

All three TICCs measured simultaneously using the TADD-2 3-way buffer.
Each TADD-2 output drives one SMA tee → two TICC channels.

**Baseline wiring (runs C1, C3):**
```
F9T PPS ──→ TADD-2 input
              ├── out1 ──→ tee ──→ TICC #1 chA + TICC #2 chA
              ├── out2 ──→ tee ──→ TICC #1 chB + TICC #3 chA
              └── out3 ──→ tee ──→ TICC #2 chB + TICC #3 chB
```

Self-consistency pairs (same TICC, different TADD-2 outputs — no shared tee):
- TICC #1: chA (out1) vs chB (out2)
- TICC #2: chA (out1) vs chB (out3)
- TICC #3: chA (out2) vs chB (out3)

Cross-TICC same-tee pairs (share a tee — may have correlated errors):
- TICC #1 chA ↔ TICC #2 chA (both from out1)
- TICC #1 chB ↔ TICC #3 chA (both from out2)
- TICC #2 chB ↔ TICC #3 chB (both from out3)

Cross-TICC different-tee pairs (independent paths — no shared artifacts):
- TICC #1 chA (out1) ↔ TICC #3 chB (out3)
- TICC #2 chA (out1) ↔ TICC #3 chB (out3)
- etc.

**Rotated wiring (run C2):**
```
              ├── out1 ──→ tee ──→ TICC #1 chA + TICC #3 chB
              ├── out2 ──→ tee ──→ TICC #2 chA + TICC #1 chB
              └── out3 ──→ tee ──→ TICC #3 chA + TICC #2 chB
```
Rotation changes which TICC pairs share a tee. If same-tee pairs show
tighter agreement than different-tee pairs in C1 but not C2, the tee
is adding correlated noise.

| Run | Wiring | Duration | Purpose |
|-----|--------|----------|---------|
| C1 | Baseline | 10 min | Full matrix: 3 self-consistency + 6 cross-TICC |
| C2 | Rotated | 10 min | Different tee pairings, tests for tee artifacts |
| C3 | Baseline | 10 min | Repeat of C1 for reproducibility |

**Total: 3 runs × 10 min = 30 min capture + cabling time between runs.**

If C1 and C3 agree within statistical uncertainty (±11%), the measurement is
reproducible and no further runs are needed. If they disagree, add runs with
individual cable swaps to isolate the cause.

### Analysis per run
For each run, compute:
1. **Self-consistency**: TDEV(τ) of chA−chB within each TICC
   - This is the TICC noise floor (both channels see the same signal)
   - Expected: ~35-40 ps TDEV(1s) for white phase noise at 60 ps single-shot
2. **Cross-TICC agreement**: TDEV(τ) of individual channels compared across TICCs
   - TICC_X chA vs TICC_Y chA (same cable path → same signal)
   - Differences reveal reference distribution, EMI, or ground noise
3. **Per-channel TDEV(τ)**: individual channel stability relative to TICC reference
   - Should be identical across all channels if TICCs are equivalent

### Expected outcomes
- If all TICCs agree at ~35-40 ps: noise floor is the TICC itself, our
  measurement setup is clean, and the 3.3 ns TDEV on TICC #3 during servo
  operation is real servo/PPS noise (not TICC artifact).
- If TICC #3 shows elevated noise even with a clean PPS: the TimeHat
  environment (EMI, ground, USB) is contaminating the measurement.
- If specific channels are noisy but others aren't: per-channel issue
  (bad SMA connector, TDC7200 variation, or cable problem).

### Noise types and slopes (reference)

For interpreting results:

| Noise type | ADEV slope | TDEV slope | Typical source |
|------------|-----------|-----------|----------------|
| White phase (WPM) | τ^(-1) | τ^(-1/2) | TICC quantization, thermal noise |
| Flicker phase (FPM) | τ^(-1) | τ^0 (flat) | Servo injection, 1/f electronics noise |
| White frequency (WFM) | τ^(-1/2) | τ^(+1/2) | Oscillator white FM, random walk of phase |
| Flicker frequency (FFM) | τ^0 (flat) | τ^(+1) | TCXO aging, temperature wander |

From overnight Run A data:
- Raw F9T PPS (chB): ADEV slope -0.95 (WPM), TDEV slope -0.38 (WPM). Clean.
- Disciplined PHC (chA): ADEV slope -0.61, TDEV slope +0.03 (flat = FPM).
  The servo converts white PPS noise into flicker phase noise in the PHC.

### Planned work (ordered by priority)

- [ ] Run C1-C5 calibration matrix (see above)
- [ ] Cable delay calibration (zero-delay reference measurement)
- [ ] EMI investigation: move TICC #3 away from RPi5, remeasure
- [ ] Reference chain validation: compare FE-5680A Rb 10 MHz at each TICC input
- [ ] Overnight PiPuss TICC #1 run for long-τ comparison with TICC #3
