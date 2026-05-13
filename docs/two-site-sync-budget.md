# Two-site clock-sync budget — design doc

**Status**: design / moonshot specification.  Drafted 2026-05-13 in
response to Bob's observation that *"TDEV(1s)=250 ps doesn't mean
two clocks stay sub-ns — they can drift 1 ns in 4 s."*  This doc does
the math for what TDEV/ADEV regime, servo bandwidth, actuator
resolution, and DO class are actually needed to hit a probability-
bounded **excursion** target, separately for the shared-antenna and
two-site cases, then lays out the hardware path implied.

The moonshot in CLAUDE.md (updated in lockstep with this doc) is the
operational statement of the targets derived here.

---

## 1. Why TDEV is not the right success metric on its own

TDEV(τ) is a statistical *variance* metric: roughly,
`TDEV(τ) ≈ σ(time error)` computed over a triangular window of half-
length τ.  Two clocks each with TDEV(τ)=σ have an instantaneous
phase-difference variance of `2σ²` and an instantaneous σ_diff of
`σ·√2`.  But "instantaneous" is the load-bearing word.

The trajectory of `Δ(t) = clock_A(t) − clock_B(t)` over an interval
is a stochastic process whose statistics depend on the **dominant
noise type**.  In the standard Allan classification:

| Slope on TDEV log-log | Noise type | σ_phase(τ) scaling | Two-clock divergence |
|---|---|---|---|
| −3/2 | White phase modulation (WPM) | ∝ 1/√τ | benign — averaging helps |
| −1 | Flicker phase modulation (FPM) | ∝ const | stationary |
| −1/2 | White frequency modulation (WFM) | ∝ √τ | random-walk apart |
| 0 | Flicker frequency modulation (FFM) | ∝ const | persistent drift |
| +1/2 | Random-walk frequency (RWFM) | ∝ τ | unbounded divergence |

Free-running OCXOs and TCXOs typically show **WPM below ~0.1 s** and
**WFM/FFM above** ~1-10 s before flattening into the flicker floor
near τ=100-1000 s.  So in the τ-regime that matters for "two clocks
stay sub-ns at all times," the noise model is closer to WFM/FFM than
WPM, and **σ_diff(τ) grows with τ between servo corrections**.

This is why Bob's intuition is right: two TDEV(1s)=250 ps clocks
*can* drift to ~700 ps in 4 s if the noise is white-frequency
dominant in that band — and exceed 1 ns within seconds if the loop
isn't pulling them back.

**Operational consequence**: the moonshot's success metric must be
a **probability-bounded excursion**, not a TDEV value.  TDEV is
inferred from the budget; it isn't the budget.

---

## 2. Budget allocation — common shape

For both shared-antenna and separate-antenna cases, the two-clock
phase-difference variance decomposes into:

```
σ²_Δ(τ) = σ²_A,vs_GPS(τ) + σ²_B,vs_GPS(τ) − 2·cov(A,B,vs_GPS)(τ)
```

The covariance term captures **common-mode** error sources that
cancel when the two clocks see the same reference signal.  Three
classes of error live there:

| Source | Shared antenna | Separate antennas |
|---|---|---|
| Satellite orbit + clock | full cancel | full cancel (PPP) |
| Ionospheric delay | full cancel | partial (∝ baseline) |
| Tropospheric delay | full cancel | partial (∝ baseline + weather) |
| Antenna multipath | full cancel | independent |
| Receiver thermal noise | independent | independent |
| Receiver clock | independent | independent |
| Cable/connector phase | independent | independent |
| Disciplining loop noise | independent | independent |
| DO free-running noise | independent | independent |

Define **per-clock σ_clock(τ)** = the part of σ_vs_GPS(τ) that
*doesn't* cancel with a co-located twin.  This is what the
servo-plus-DO chain owns; it's the same in both cases.  Define
**σ_atm(τ)** = the residual atmospheric+multipath disagreement
between the two sites.  Then:

```
shared antenna:  σ²_Δ(τ) ≈ 2 · σ²_clock(τ)
separate sites:  σ²_Δ(τ) ≈ 2 · σ²_clock(τ) + σ²_atm(τ)
```

For "95% of excursions stay within bound B," using a Gaussian
assumption, B ≈ 2 · σ_Δ.  So σ_Δ ≤ B/2.  Substituting:

```
shared antenna:  σ_clock ≤ B_shared / (2√2)         (B_shared = 1 ns)
                          ≤ 354 ps RMS at every τ
separate sites:  √(2σ²_clock + σ²_atm) ≤ B_sep / 2  (B_sep = 2 ns)
                          ≤ 1.0 ns RMS combined
```

The combined bound is **the same RMS as if we only allowed σ_clock**;
the second site's atmospheric residual has to fit within the
additional 1 ns we relax to.

---

## 3. Per-clock budget — what σ_clock(τ) ≤ 354 ps actually demands

σ_clock(τ) decomposes further into:

```
σ²_clock(τ) = σ²_meas(τ)  +  σ²_servo_residual(τ)  +  σ²_DO_above_BW(τ)  +  σ²_actuator_q(τ)
              ──────┬─────     ─────────┬────────     ──────────┬────────     ──────┬──────
                    │                   │                       │                  │
              TD-CP per-epoch     loop dynamics fail        free-running        DAC LSB
              precision           to pull error to 0        DO noise above       ÷ pull range
              ~5-10 ps            at τ                      servo BW             quantization
                                                            (depends on DO
                                                            class)
```

Each term needs to fit inside ≤ 354 ps (so the four together stay
under the budget at all τ).  Work through:

### 3.1 Measurement floor σ_meas(τ)

L1 carrier wavelength = 19.0 cm.  Per-epoch carrier-phase precision
in a clean PPP filter is **1-2 mm range-domain ≡ 3.3-6.7 ps
time-domain**.  TD-CP epoch-to-epoch is at this floor.

**σ_meas ≈ 5-10 ps.** ✓ Well under the 354 ps per-clock budget;
fundamentally not the limiter.

### 3.2 Servo residual σ_servo_residual(τ)

The loop's ability to drive innovation to zero at τ.  For a
well-tuned LQR/Kalman servo (which we have — `kalman_servo.py`,
`do_freq_est.py`), this is set by `(σ_meas / SNR_loop) · (τ_loop / τ)`.
At τ=1 s, with loop time-constant ~1 s, this is dominated by
σ_meas — i.e., ~10 ps.  Fine.

The concern is **plant-model error** — if `dac_ppb_per_code` is
mis-scaled by factor α, the EKF systematically under- or
over-corrects, and σ_servo_residual carries a bias term `∝ u·(1−α)`.
That term is detected by the **innov-vs-control monitor** filed as
`innovControlMonitor-main` (2026-05-13).  Treat it as a separate
hazard with its own gate, not an additive σ here.

**σ_servo_residual ≈ 10-20 ps**, assuming plant model is correct. ✓

### 3.3 Free-running DO noise above servo BW — σ_DO_above_BW

This is the wall.

For actuation cadence T_act and effective loop bandwidth
f_c ≈ 1/(2π·T_act), the free-running DO noise from f_c to ∞ is
unfiltered.  Approximating the DO as integrated white-frequency
noise with one-sided spectral density h_y(f) yields:

```
σ_phase(above f_c) ≈ σ_y(τ=1/f_c) · (1/f_c)
                   = ADEV(τ=T_act·π·2) · T_act·π·2 · approximation factor
```

In practical numbers, for the three DO classes:

| DO class | typical ADEV(1s) | σ_phase, T_act=1s | σ_phase, T_act=0.1s |
|---|---|---|---|
| i226 TCXO (TimeHat) | 1e-10 | ~100 ps | ~10 ps |
| Premium TCXO (CTS GTXO-92) | 3e-11 | ~30 ps | ~3 ps |
| Hobbyist OCXO (Isotemp/CTI) | 1e-11 | ~10 ps | ~1 ps |
| Premium OCXO (Microchip OX-249) | 2e-13 | ~0.2 ps | ~0.02 ps |
| Cs/Rb reference | 1e-12 | ~1 ps | ~0.1 ps |

The **354 ps per-clock budget** allows σ_phase_above_BW up to roughly
half of it (the rest goes to measurement + servo residual +
actuator), so ~150-200 ps.  **All OCXO classes meet this at T_act=1s.**
i226 TCXO does **not** — at 100 ps it already eats the whole budget
on its own.

**Conclusion**: OCXO-class DO is sufficient at 1 Hz actuation cadence.
TCXO-class is borderline-to-inadequate.

This matches CLAUDE.md's updated moonshot — TCXO hosts are **best
effort, not part of the sync target**.

### 3.4 Actuator quantization — σ_actuator_q(τ)

The DAC commands frequency in discrete LSBs of size Δf_LSB ppb.
Between corrections, the DO accumulates phase at the residual rate
`u_target − round(u_target)` × actuation period.  Statistically,
σ_q ≈ (Δf_LSB / √12) · T_act in ns/s × s.

| DAC | Δf_LSB (over ±2 ppm) | σ_q at T_act=1s |
|---|---|---|
| 16-bit (AD5693R, current) | 0.061 ppb/code | 18 ps |
| 18-bit (AD5781) | 0.015 ppb/code | 4 ps |
| 20-bit (AD5791) | 0.004 ppb/code | 1 ps |
| Internal 20-24 bit (digital OCXO) | 0.001-0.005 ppb/code | 0.3-1 ps |

The current 16-bit AD5693R's 18 ps quantization noise is already a
meaningful fraction of the 354 ps budget — not enough to disqualify
the host, but enough that **upgrading to 18-bit DAC removes the
quantization term entirely** (4 ps is negligible against the
~150-200 ps DO + measurement headroom).

**Conclusion**: 16-bit is OK but suboptimal; 18-bit eliminates the
quantization concern; 20-bit+ (digital OCXO with internal DCO) is
overkill but harmless.

---

## 4. Shared-antenna case — math, achievability

### 4.1 Budget summary

```
shared antenna, 95% excursion ≤ 1 ns:
  σ_Δ(τ) ≤ 500 ps      at all τ from 0.1s out to 1000s
  → σ_clock(τ) ≤ 354 ps per clock
```

### 4.2 Per-term allocation (OCXO + 18-bit DAC + 1 Hz servo)

| Term | Allocation | Actual | Headroom |
|---|---|---|---|
| σ_meas (TD-CP) | 100 ps | 10 ps | huge |
| σ_servo_residual | 100 ps | 20 ps | comfortable |
| σ_DO_above_BW | 200 ps | 10 ps (Isotemp OCXO) | huge |
| σ_actuator_q | 50 ps | 4 ps (AD5781) | huge |
| **RSS** | **354 ps** | **~25 ps** | **fits in 0.07× budget** |

**Verdict**: shared-antenna 1 ns @ 95% is comfortably reachable with
**OCXO + 18-bit DAC + 1 Hz servo**.  The actual sub-ns floor of the
chain in this configuration looks more like **~50-100 ps**, leaving
ample margin for plant-model/loop imperfections.

### 4.3 Risks

- **Plant-model error not caught**.  `innovControlMonitor-main`
  closes this.
- **Servo oscillation / limit cycle**.  The clkPoC3 day0512 TDEV
  hump at τ=128s (63.5 ns) is exactly this kind of signature.
  Diagnostic on the table; not a fundamental limit.
- **Local TICC drift contaminating chA-alone TDEV measurement** —
  measurement-side artifact, not in σ_Δ.

### 4.4 Validation

Two PiFace-class hosts (OCXO + DAC + same firmware) on the UFO1
splitter, both running the disciplining loop.  Measure σ_Δ directly
via TICC differential (chA-chB on a shared TICC, eliminating that
TICC's own reference noise).  Expect σ_Δ ≤ 200 ps RMS, max
excursion in 1 h ≤ 1 ns.

---

## 5. Separate-antennas case — math, achievability

### 5.1 What goes into σ_atm

For two sites at baseline distance D, the differential atmospheric
delay decomposes into:

```
σ_atm(τ) ≈ σ_ZHD(D)  +  σ_ZWD(D)  +  σ_ion(D, sky)  +  σ_mp_indep
            ──┬──        ──┬──         ──┬──         ──┬──
              │            │             │             │
          dry tropo      wet tropo    ionosphere    multipath
          ~1-3 mm        ~3-10 mm     ~0-3 mm at L1  ~3-10 mm
          per 10 km      per 10 km    (modeled out   per receiver
                                       in PPP IF)    (independent)
```

Converting mm → ps via c⁻¹: 1 mm ≈ 3.3 ps.  So per 10 km baseline,
worst-case differential atmospheric delay is roughly:

- Dry tropo:   ~3 mm    →  10 ps
- Wet tropo:   ~10 mm   →  33 ps
- Iono (after IF combo): ~3 mm  →  10 ps
- Multipath (per-site, independent): ~10 mm  →  33 ps

For two sites at ≤10 km, **σ_atm ≈ 50 ps** is plausible with PPP +
shared-sky.  Multipath dominates — could blow up to 100-300 ps if
either site has bad sky.  For a "two-site sub-2-ns" goal, σ_atm
budget is the residual after accounting for two clocks' σ_clock:

```
separate sites, 95% excursion ≤ 2 ns:
  σ_Δ ≤ 1.0 ns
  → √(2σ²_clock + σ²_atm) ≤ 1.0 ns
  → if σ_clock = 354 ps (same as shared), σ_atm ≤ 880 ps
```

**Verdict**: 2 ns @ 95% on separate antennas is **reachable** if
sites have clean multipath environments (rooftop or open-field
antennas, no urban canyons).  Lab-style sites with antenna ledges
near walls/buildings may show σ_atm > 1 ns, pushing total above
the 2 ns bound.  The atmospheric budget is the limiter, not the
clock chain.

### 5.2 Subtleties

- **Different antenna types** introduce per-receiver group-delay
  differences that look like a constant offset, calibratable but
  must be applied.
- **Time-of-day effects** — diurnal tropo cycling, ionospheric peaks
  near local solar maximum.  May need to widen the 95% probability
  window across daily cycles, or specify "95% within any 1-hour
  window."
- **Common SSR analysis center** — required for AR.  Mixing AC
  datums (e.g., CNES+WHU) is fine for float PPP but disqualifies
  AR per `docs/ac-datum-mixing.md`.
- **Receiver firmware differences** — F9T-10 (TIM 2.20) vs F9T-20B
  (TIM 2.25) may exhibit different cycle-slip / signal-tracking
  behavior at low elevation.  Empirical, characterizable.

### 5.3 Validation

Two PiFace-class hosts on separate rooftop antennas at ≤5 km
baseline.  Same SSR mount, same firmware revision, same DAC
revision.  TICC at each site monitoring its own DO PPS; cross-
correlation via post-processing (timestamps shared via network).
Or: a single TICC with two BNC feeds (one from each site over
fiber to eliminate cable delay variation).  Expect σ_Δ ≤ 500 ps
RMS, max excursion in 1 h ≤ 2 ns.

---

## 6. Hardware path implications

The math above resolves to a parts-list:

### 6.1 DO class

**Best:** Premium OCXO at ADEV(1s) ≤ 1e-11.  CTI OS-class, Isotemp
OCXO118, or similar.  Bob has these.

**Acceptable:** Standard OCXO at ADEV(1s) ≤ 3e-11.  Most COTS
through-hole OCXOs in the $40-100 range.  Margin tightens but
shared-antenna 1 ns still fits.

**Inadequate:** TCXO at ADEV(1s) ≥ 1e-10 (includes i226 internal
TCXO).  **Designated best-effort.** Not part of sync target.

### 6.2 Actuator resolution

**Recommended:** 18-bit external DAC (Analog Devices AD5781 or
AD5791) + LTC6655 or ADR4525 voltage reference.  ~$80-100 in parts.
LSB at ±2 ppm pull ≈ 15 ppt — well under any OCXO's intrinsic
noise floor.

**Current:** 16-bit AD5693R is the baseline.  Marginal but
operationally OK.  18-bit is the upgrade path.

**Future option:** Native digital interface in the OCXO itself
(20-24 bit internal DAC, see §7).

### 6.3 Servo cadence

**Current:** 1 Hz actuation (matches PPS/TICC measurement cadence).
Adequate for OCXO-class DO.

**Future option:** Higher cadence (5-10 Hz) via TD-CP at the
receiver's full RAWX rate.  Would buy 5-10× margin against
σ_DO_above_BW but is more important for TCXO hosts (which are
designated best-effort anyway).  Lower priority than DAC upgrade.

### 6.4 Measurement chain

**Current:** PPP + TD-CP at 1 Hz.  Carrier-phase ambiguity-fixed.
σ_meas ~5-10 ps, well within budget.

**No upgrade needed** for the target.  The measurement chain is
not the limiter.

---

## 7. Digital OCXO (DC-OCXO) survey

Bob asked: would buying a digitally-controlled OCXO with built-in
high-resolution DCO replace the external DAC architecture?  Survey
results (2026-05-13):

### 7.1 Findings

The "DC-OCXO" market is smaller and pricier than expected.  Three
practical classes:

**(a) Telecom-grade analog OCXO + internal DAC, digital I²C/SPI
interface.**  Vendors: IQD IQOV-200/IQOV-300 series, Vectron OCS,
Microchip MV-DOCSO, Rakon RFPO45.  Specs: ADEV(1s) 1e-12 to
5e-12, 20-bit-equivalent internal DAC, I²C or SPI.  Pricing:
$200-500 typical at qty-1, but stock is often "factory order"
with 12-20 week lead.  Not casually available on Mouser/DigiKey.

**(b) MEMS Super-TCXO with native digital interface.**  Best
example: **SiTime SiT5358** — I²C, **5 ppt resolution**,
ADEV(10s) = 1.5e-11 (≈ ADEV(1s) ~5e-11).  Cheap (~$50-80) and
available.  **Caveat**: MEMS short-term ADEV is ~10× worse than
premium analog OCXO.  For the moonshot's TDEV requirements,
SiT5358 is borderline.  Better fit for "fleet expansion at modest
cost" than for "best DO in the lab."

**(c) Mil-grade / space OCXOs.**  Microchip OX-249, Symmetricom
SA45.  ADEV(1s) 2e-13.  Pricing $1000-5000.  Out of hobbyist
budget unless surplus.

**(d) Connor-Winfield DOCAT/DOCSC series.**  Despite the "D"
prefix, these are **analog VCO-EFC parts** with internal *digital
temp compensation* but external *analog* steering.  They do not
help with our DAC-resolution problem.  Mis-categorized by some
distributors.

### 7.2 Comparison vs upgrade-existing-OCXO-with-18-bit-DAC

| Path | Up-front cost | ADEV(1s) | Resolution | Availability |
|---|---|---|---|---|
| Current: existing OCXO + AD5693R 16-bit | $0 | 1e-11 | 0.061 ppb | done |
| Upgrade: existing OCXO + AD5781 18-bit + LTC6655 | ~$80/host | 1e-11 | 0.015 ppb | yes |
| Upgrade: existing OCXO + AD5791 20-bit + LTC6655 | ~$120/host | 1e-11 | 0.004 ppb | yes |
| Native: SiT5358 MEMS Super-TCXO | ~$50/host | 5e-11 | 5 ppt | yes |
| Native: IQD IQOV-200 telecom DC-OCXO | ~$250-400/host | 5e-12 | <1 ppt | factory-order |
| Premium: Microchip OX-249 | $1000+/host | 2e-13 | unknown | quote |
| Already deployed: Renesas 8A34002 ClockMatrix + OCXO | $0 (otcBob1, ptBoat have it) | inherits OCXO | sub-ppt FCW | done |

### 7.3 Recommendation

**For the sync moonshot, OCXO + 18-bit DAC is the best
price-performance path.**  ~$80/host parts cost, uses existing
OCXOs Bob has on the shelf, four extra bits of resolution clears
the quantization budget item entirely.  No new vendor relationship
needed.

The Renesas 8A34002 ClockMatrix architecture (otcBob1, ptBoat)
**already is a DC-OCXO architecture** — FCW writes steer the
attached OCXO at sub-ppt resolution.  Bringing more hosts onto
ClockMatrix is a parallel option; bears against integration
complexity vs the simpler DAC-upgrade path.

If Bob wants to *test* the native-digital-OCXO architecture with
minimal investment, **two SiT5358 eval boards (~$100-150 total)**
gets the I²C-DCO experience in our pipeline.  Comparison run
against the AD5781 + OCXO host tells us empirically whether the
MEMS Super-TCXO's intrinsic ADEV penalty is offset by the cleaner
actuator path.  Likely no (5e-11 ADEV vs 1e-11 is a real
deficit), but a clean experiment is cheap.

Premium parts (Microchip OX-249, IQD IQOV-200 telecom-grade) are
defensible but require direct vendor contact and aren't
cost-effective for hobbyist deployment.  Skip unless one falls
into the lab via surplus.

---

## 8. Path forward (work items)

1. **Land `innovControlMonitor-main`** — closes the load-bearing
   diagnostic for plant-model error.  Tells us empirically whether
   the current servo is actually hitting the per-clock budget
   above, regardless of theoretical analysis here.

2. **Land `dacPpbSignClean-main`** — eliminates the most-likely
   source of plant-model error.  Pair with #1's monitor as the
   validation gate.

3. **Upgrade clkPoC3 + PiFace + (one more) to 18-bit DAC
   architecture.**  AD5781 + LTC6655.  Lab-host hardware work, not
   software; file as `hw:`-labeled item once electronics ordered.
   Validation: σ_Δ measurement vs 16-bit AD5693R baseline.

4. **Run shared-antenna two-clock σ_Δ measurement** as the
   moonshot's primary acceptance test.  Pair of OCXO+18-bit-DAC
   hosts on UFO1 splitter, TICC differential, 24h run, statistical
   summary.

5. **(Stretch) Two-site separate-antenna run** — needs Bob's other
   antenna site to be production-ready.  PR run with PPP-AR + same
   SSR mount.

6. **(Optional experiment) SiT5358 eval-board pair** — characterize
   native-digital-OCXO architecture as an alternative path.

---

## 9. Open questions

- **Empirical TDEV slope** for our current OCXO+AD5693R hosts at
  τ=1-100s — is it actually WFM (slope −1/2) or closer to FFM
  (slope 0)?  Re-analysis of clean day0512 PiFace data can answer.
  Matters for whether the 95%-excursion math is conservative or
  optimistic.

- **Loop dynamics** at servo BW — is the LQR gain critically damped
  in practice?  The clkPoC3 day0512 hump at τ=128s suggests
  maybe not.  Innov monitor will help.

- **Differential atmospheric model fidelity** for our specific
  lab/rooftop site pair — needs empirical data from a parallel run
  on the two-site case.

---

## References

- `docs/asd-psd-servo-tuning.md` — Allan-deviation / power-spectral
  density theory + servo BW relationships for our hosts.
- `docs/clock-state-modeling.md` — EKF state model and where each
  noise term enters.
- `docs/ac-datum-mixing.md` — why SSR-AC choice matters for the
  separate-antennas case.
- `docs/ticc-baseline-2026-04-01.md` — TICC noise floor + EXTTS
  quantization limits.
- Memory item `freerun_floors_2026_05_07` — empirical ADEV/TDEV of
  i226 PHC vs F9T PPS vs PPP dt_rx, with GNSS-vs-DO crossover
  at τ≈2-3s on i226 PHC.

---

*Drafted: 2026-05-13 (main).  Acks pending bravo + charlie.*
