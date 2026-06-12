# Visual Stories — TDEV Plots for PePPAR Fix

Each plot tells one story.  No overloading.


## Servo Drive Taxonomy

PePPAR Fix supports four progressively more precise ways to drive
the discipline servo.  The first two require precise timestamps of
physical PPS edges; the last two derive the TCXO-to-GPS relationship
from carrier-phase observations and need no edge timestamps at all.

| Servo Drive | Observation Source | Precision (1s) | Edge Timestamp Required |
|---|---|---|---|
| **PPS Phase** | PPS edge via EXTTS/TICC | ~2.3 ns (~2300 ppb) | Yes |
| **PPS+qErr Phase** | PPS edge + firmware sawtooth correction | ~0.2 ns (~200 ppb) | Yes |
| **PPP Carrier Phase** | Float PPP dt_rx from dual-frequency carrier-phase | ~0.1 ns (~0.1 ppb) | No |
| **PPP-AR Carrier Phase** | Ambiguity-resolved PPP dt_rx | ~0.01 ns (~0.01 ppb) | No |

**PPS Phase** and **PPS+qErr Phase** measure the phase error of a
physical PPS edge.  The servo input is a phase measurement; the
instrument (EXTTS, TDC, TICC) limits the achievable precision.
qErr is literally a correction applied to the PPS edge — the
F9T firmware reports the sub-cycle error between the ideal and
quantised PPS edge.

**PPP Carrier Phase** and **PPP-AR Carrier Phase** estimate the
receiver clock offset (dt_rx) from GPS time using carrier-phase
observations from multiple satellites.  This is an independent
measurement of the TCXO-to-GPS relationship — no PPS edge
timestamps are involved.  PPS edges are still needed for initial
phase alignment (bootstrap), but the steady-state servo runs
entirely on dt_rx.

The "Carrier Phase" label reflects the GNSS measurement technique:
dual-frequency carrier-phase observations provide sub-wavelength
(sub-cm → sub-ns in time) precision.  PPP-AR further resolves
integer cycle ambiguities for an additional ~10x improvement.

On platforms with a ClockMatrix (Timebeat OTC), the servo steers
the fractional output divider via FCW.  On PHC-only platforms
(TimeHat, E810), the servo steers PHC adjfine.  The servo drive
taxonomy is independent of the actuator.


## Plot 1: PePPAR Fix vs a Commercial GPSDO

**Story**: At long tau, all GPSDOs converge to the same GNSS-derived
stability.  Differentiation happens at short tau, where the local
oscillator and discipline algorithm matter.  PePPAR Fix should compete
with or beat a high-quality commercial GPSDO.

**Traces**:
- PePPAR Fix disciplined PPS OUT (TICC chA), warm start
- Published TDEV from a reference GPSDO (HP/Agilent 58503B or
  Symmetricom/Microsemi SA.45s — find a published plot with
  comparable tau range)
- Optional: raw F9T PPS (undisciplined) as a "without discipline"
  reference

**X-axis**: tau from 1s through 10,000s (showing the crossover to
GNSS).

**Y-axis**: TDEV (ns).

**Key visual**: both curves converge at long tau (GNSS limit).  At
short tau, PePPAR Fix should be comparable or better, proving the
PPP Carrier Phase servo drive is competitive.

**Shading**: one-sigma confidence band on PePPAR Fix trace.  The
commercial GPSDO likely shows only a nominal curve from the datasheet.

**Data needed**:
- [x] Disciplined PePPAR Fix run on TimeHat, TICC chA
      (15 min runs collected 2026-04-02; need 2+ hour for long tau)
- [ ] Published TDEV data from HP 58503B, Symmetricom SA.45s, or
      similar (digitize from datasheet or find tabulated data)

**Reference GPSDO candidates**:
- HP/Agilent 58503B: classic telecom GPSDO, OCXO, widely published
  TDEV curves.  Typical TDEV(1s) ~1 ns.
- Symmetricom/Microsemi SA.45s CSAC: chip-scale atomic clock with
  GPS discipline.  Published ADEV curves available.
- Jackson Labs Fury: OCXO GPSDO, spec sheet has ADEV curves.
- SRS FS740: GPS-referenced frequency standard with published TDEV.

Any of these would make a credible comparison.  The HP 58503B is the
most widely recognized benchmark.


## Plot 2: EXTTS Cannot Accurately Measure Raw F9T TDEV

**Story**: neither the i226 nor the E810 EXTTS path faithfully
reproduces the true F9T PPS jitter.  The TICC is the ground truth.
The gap between TICC and EXTTS is measurement error, not signal.

**Traces**:
- F9T PPS TDEV measured on TICC (ground truth, solid black)
- F9T PPS TDEV measured on i226 EXTTS (orange)
- F9T PPS TDEV measured on E810 EXTTS (purple)

**Shaded regions**:
- Between TICC and E810 EXTTS (purple shading):
  "Actual F9T TDEV unreported by E810 — quantization flatness
  (77% identical adjacent timestamps, ~8 ns effective resolution)"
- Between TICC and i226 EXTTS (orange shading):
  "i226 EXTTS measurement noise — 8 ns tick quantization adds
  ~2.9 ns RSS to the measurement"

**No qErr on this plot.**  This plot is purely about measurement
fidelity.

**X-axis**: tau 1s to ~100s (short tau where EXTTS limitations
are most visible).

**Data needed**:
- [x] TICC chB 2h baseline (data/ticc-baseline-2h-1.csv)
- [x] i226 EXTTS freerun 2h (data/freerun-timehat-2h.csv)
- [x] E810 EXTTS freerun 2h (data/freerun-ocxo-2h.csv)

**Note**: the E810 EXTTS trace is BELOW the TICC (falsely low TDEV)
— shade downward from TICC to E810.  The i226 EXTTS trace is ABOVE
the TICC (added noise) — shade upward from TICC to i226.


## Plot 3: Free-Running Oscillator Comparison (TCXO vs OCXO)

**Story**: the E810's OCXO should beat the i226's TCXO at all
taus.  This sets the discipline floor — the servo can't make the
PHC quieter than its oscillator at short tau.

**Traces**:
- i226 TCXO free-running PEROUT (TICC chA, TimeHat)
- E810 OCXO free-running PEROUT (TICC chA, ocxo)

**Both measured by TICC** — no EXTTS in this plot.  Pure oscillator
comparison at 60 ps resolution.

**X-axis**: tau 1s to 2000s.

**Shading**: one-sigma confidence band on each trace.

**Data needed**:
- [x] TimeHat TICC chA 2h (data/ticc-baseline-2h-1.csv)
- [x] ocxo TICC chA 2h (data/ticc-ocxo-2h.csv)

**Note from initial data**: surprisingly, the E810 OCXO PEROUT
showed 2.78 ns TDEV(1s) vs the i226 TCXO at 1.17 ns.  The E810
PEROUT appears coupled to the F9T PPS sawtooth internally.  If
this holds, the story becomes: "the E810's OCXO is excellent, but
its PEROUT output doesn't reflect the OCXO's true stability."
Investigate whether the E810 PEROUT is phase-locked to the PPS
rather than free-running from the OCXO.


## Plot 4: Disciplined PPS OUT — Four Servo Drives

**Story**: each servo drive type improves the disciplined output.
The progression from PPS Phase through PPP-AR Carrier Phase shows
orders-of-magnitude improvement in discipline precision.

**Servo drive types** (see "Servo Drive Taxonomy" below for details):

| Servo Drive | Observation Source | Precision (1s) | Edge Timestamp Required |
|---|---|---|---|
| PPS Phase | PPS edge via EXTTS/TICC | ~2300 ppb (2.3 ns) | Yes |
| PPS+qErr Phase | PPS edge + firmware sawtooth correction | ~200 ppb | Yes |
| PPP Carrier Phase | Float PPP dt_rx from carrier-phase observations | ~0.1 ppb | No |
| PPP-AR Carrier Phase | Ambiguity-resolved PPP dt_rx | ~0.01 ppb | No |

**One plot per host** (TimeHat and ocxo, if both have TICC on chA).

**Traces** (all measured on TICC chA = disciplined PEROUT):
- Disciplined with PPS Phase servo drive
- Disciplined with PPS+qErr Phase servo drive
- Disciplined with PPP Carrier Phase servo drive

**All warm start** — after servo has settled (skip first 5-10 minutes
of convergence).

**X-axis**: tau 1s to crossover point with GNSS (~1000s), or a
little beyond.

**Shading**: one-sigma confidence band on each trace.  Shade the
improvement region between PPS Phase and the best drive type.

**Data needed**:
- [x] TimeHat disciplined run, PPS Phase, 15 min, TICC chA
- [x] TimeHat disciplined run, PPS+qErr Phase, 15 min, TICC chA
- [x] TimeHat disciplined run, PPP Carrier Phase, 15 min, TICC chA
- [x] ocxo PPS Phase + PPP Carrier Phase, 15 min, TICC chA
      (qErr not reliably delivered on E810 I2C — 28% coverage)
- [ ] 2+ hour versions of the above for long-tau confidence

**Note**: these are disciplined runs, not freerun.  The servo must
be actively applying corrections.  The TICC chA measures the result
on the PEROUT.


## Plot 5: Our Measurements Are Valid

**Story**: our best measurement (TICC+qErr corrected F9T PPS) is
well above the noise floor of the measurement setup.  We are
measuring real signal, not instrument noise.

**Traces**:
- TICC+qErr corrected F9T PPS (our best result, 170 ps at τ=1s)
- TICC+GPSDO OCXO measurement noise floor (gray shaded)
  - Component: TICC 60 ps white phase noise (τ⁻¹)
  - Component: Geppetto GPSDO OCXO stability (rises at long τ)

**Shading**:
- Gray: measurement floor region (everything below this is
  instrument noise)
- One-sigma confidence band on the TICC+qErr trace

**X-axis**: tau 1s to 600s.

**Key visual**: clear daylight between the green trace and the gray
floor at all taus.  At τ=1s, the measurement (170 ps) is 5× above
the floor (35 ps).

**Data needed**:
- [x] TICC+qErr v2 30m (data/ticc-qerr-v2-30m.csv)
- [x] GPSDO OCXO ADEV estimate (5×10⁻¹²/√τ, 5×10⁻¹⁴ flicker)

**Note**: the floor at long τ is estimated from the GPSDO spec, not
measured independently.  The uncommissioned rubidium oscillators
would provide a tighter floor characterization.


## Common conventions across all plots

### One-sigma shading

Every TDEV trace should show a one-sigma confidence band.  For
overlapping Hadamard (ohdev) estimators, the confidence interval
is approximately:

    σ_TDEV ≈ TDEV / sqrt(N_effective)

where N_effective ≈ (total_samples - 3*tau) / tau for non-overlapping,
or the EDF (equivalent degrees of freedom) for overlapping.  A
simpler approximation for plotting:

    upper = TDEV * (1 + 1/sqrt(N_eff))
    lower = TDEV * (1 - 1/sqrt(N_eff))

Use translucent fill matching the trace color.

### Duration matching

When comparing two traces on the same plot, use the same observation
duration.  The F9T PPS sawtooth has observation-length-dependent TDEV
(2h gives 2.3 ns, 30 min gives 1.0-1.4 ns).  Mismatched durations
produce misleading comparisons.

### Tau range

Most plots should focus on tau 1s to 1000s.  Beyond 1000s, all
GPS-disciplined sources converge to the same GNSS limit and the
differentiation story ends.  Exception: Plot 1 (commercial GPSDO
comparison) should extend to 10,000s to show the convergence.

### Detrending

Freerun data (Plots 2, 3) must be linearly detrended to remove the
oscillator's frequency offset before computing TDEV.  Disciplined
data (Plots 1, 4) should not be detrended — the servo's residual
frequency error is part of what we're measuring.


## Plot 6: SSR Corrections — Magnitude and Timescale

A different *class* of visual from Plots 1–5: not a TDEV plot, but a
time series of the SSR corrections themselves.  Time on the x-axis,
correction magnitude on the y-axis **in nanoseconds** — because the
audience cares about time, and ns is the common currency that lets
orbit, clock, code bias, and phase bias be compared on one scale
(`1 m = 3.3356 ns`; `value_ns = value_m / c * 1e9`).

All SVs × all signals would be a solid band of spaghetti, so we
cherry-pick a newer-generation SV or two per constellation.

### Story A — component contrast (magnitude × timescale)

The strongest narrative, and it explains real stream engineering: the
four components separate cleanly by *how fast they move*, not just by
size.

| Component | Magnitude (LOS) | Natural timescale | Trace shape |
|---|---|---|---|
| **Clock** | 1–10 ns (peaks ~20) | seconds–minutes | the busy one — all the high-frequency content |
| **Orbit** (radial) | ~1–3 ns | orbital period (~12–14 h) + 2-h ephemeris steps | slow sinusoid on a sawtooth |
| **Code bias** | ~1–5 ns | days–weeks | flat horizontal offsets |
| **Phase bias** | 0–0.6 ns (fraction of an L1 cycle = 0.635 ns) | hours, rare cycle steps | nearly flat with occasional integer jumps |

Punchline: **clock is the only fast-moving correction.**  That is *why*
real-time SSR streams update clock every ~5 s, orbit every ~60 s, and
biases rarely — the plot makes the stream-design rationale visible.
The 2-hourly ephemeris-update discontinuities (orbit + clock both step
when a new IODE is issued) are a secondary feature worth calling out.

### Story B — constellation contrast (clock-oscillator)

The clock panel, one SV-pair per constellation, ranks visually by the
onboard frequency standard: **Galileo PHM (passive H-maser) smooth and
quiet ≪ GPS Rb (wander, ~12-h periodic signature on IIF) < GLONASS Cs
(jumpiest, plus FDMA-driven code-bias spread).**  Block-IIF-vs-III GPS
SVs give a within-GPS generational sub-story.

### Per-component windowing — there is no single window

Each component lives on a different timescale, so plot each in a window
matched to it and **decimate/low-pass to the feature being shown**
(this is what avoids the solid-noise-band problem):

| Component | Window (a few cycles) | Sample/decimate to |
|---|---|---|
| Clock — overview | 24–48 h | ~1 min (running mean) |
| Clock — zoom inset | 10–30 min | native (5–30 s) |
| Orbit | 48–72 h | ~5 min |
| Code bias | 1–7 days | ~hourly |
| Phase bias | ~24 h | ~1–5 min |

Don't try to show seconds and 48 h on one axis — the clock gets a
multi-day overview *plus* a minutes-scale zoom inset.

### Data source and tooling

- **Source:** CNES `SSRA00CNE0` (`products.igs-ip.net:443`) — carries
  all four components including **phase bias**.  Note the operational
  `SSRA00BKG0` stream has **phase bias = 0**, so the phase-bias trace
  needs a CNES/WHU capture.
- **Logger:** `scripts/log_ssr_corrections.py` reuses the engine's
  `NtripStream` + `SSRState` to snapshot all four components per SV into
  a long-format CSV (metres + ns) every `--interval` seconds.
- **Plotter:** `scripts/plot_ssr_corrections.py` renders **four separate
  figures** — `ssr_clock.png`, `ssr_orbit.png`, `ssr_codebias.png`,
  `ssr_phasebias.png` — each:
  - **detrended** (per-SV mean removed) so the variance is symmetric about
    zero and shows *character*, not the per-SV datum offset (raw SSR clock
    `c0` carries tens-of-ns per-SV offsets that are absorbed by the receiver
    clock + inter-system bias and would otherwise read as alarming biases);
  - on a **common ±10 ns scale** so magnitude can be compared *across* the
    four figures (the contrast is the story: code bias ≈ 0 ≪ clock < orbit;
    a trace exceeding ±10 ns is clipped and its peak is named in the title —
    e.g. GLONASS orbit ±40 ns, BeiDou phase ±18 ns);
  - **gap-broken** for clock & orbit (`_break_stale` NaNs values held past the
    ~10 s cadence, so CNES's intermittent GLONASS coverage shows as gaps, not
    held-value flats).  Code/phase bias are genuinely quasi-static, so a
    value-change detector can't distinguish "held fresh" from "stale" — they
    are **not** gap-broken by value-change.  The logger now records `age_s`
    (staleness = snapshot − rx_time) and `disc` (bias discontinuity counter),
    so the next capture can age-gap the biases and flag the per-SV phase-bias
    re-reference jumps (the BeiDou discontinuities) directly.
- **Cherry-picked SVs:** GPS `G24` (IIF) vs `G04` (III); Galileo `E26`,
  `E36` (FOC); GLONASS `R02`, `R20`; BeiDou-3 MEO `C21`, `C36` — all eight on
  every figure, coloured by constellation.
