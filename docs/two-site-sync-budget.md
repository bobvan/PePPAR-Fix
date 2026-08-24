# Two-site clock-sync budget — design doc

**Status**: design / moonshot specification.  Drafted 2026-05-13 in
response to Bob's observation that *"TDEV(1s)=250 ps doesn't mean
two clocks stay sub-ns — they can drift 1 ns in 4 s."*  Substantially
revised 2026-06-01 after first clean shared-antenna acceptance-test
run revealed σ_servo_residual is the binding constraint via
chi²-gate lockout, not the actuator/DO terms originally predicted.
This doc does the math for what TDEV/ADEV regime, servo bandwidth,
actuator resolution, and DO class are actually needed to hit a
probability-bounded **excursion** target, separately for the
shared-antenna and two-site cases, then lays out the hardware path
implied.

The moonshot in CLAUDE.md (updated in lockstep with this doc) is the
operational statement of the targets derived here.

**Running this backwards?**  See
[`two-clock-agreement-forward-model.md`](two-clock-agreement-forward-model.md)
— the same math solved for p95 given a parts list, plus
`tools/sync_agreement_model.py` to sweep it.  This doc is the one to read when
grading a run; that one is the one to read when speccing hardware.

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

#### 3.1.1 The Rb-reference floor — single-clock vs differential

The TICC timestamps every PPS edge against the lab Rb standard
(FE-5680A) as its timebase, so the Rb's own instability enters the
measurement — but **only in single-clock mode, not in the differential
the budget is actually written against** (§2).  This distinction
governs how we can measure the random-walk budget.

**Single-clock (chA vs Rb).**  The Rb's wander adds in quadrature, so a
clock's random walk is attributable only *above* the Rb's TDEV at that
τ.  For the FE-5680A (≈ white-FM `ADEV ≈ 2e-11/√τ` out to ~100 s, a
flicker floor of ~1-3e-12 at 100-1000 s, then its own RWFM + drift
rising beyond ~1000-3000 s), with `TDEV(τ) ≈ τ·ADEV(τ)/√3`:

| τ | Rb TDEV (single-clock floor) | per-clock budget |
|---|---|---|
| 1 s | ~12 ps | 354 ps |
| 10 s | ~37 ps | 354 ps |
| 100 s | ~120 ps | 354 ps |
| 1000 s | ~0.3-1 ns (RWFM emerging) | 354 ps |

So single-clock chA-vs-Rb can verify the per-clock budget at short/mid
τ with healthy margin, and — because the FE-5680A has *no RWFM of its
own below ~1000 s* — it is just good enough to confirm the budget's
**"no positive-slope (RWFM) region below τ ≈ 1000 s"** requirement: any
positive slope you see below 1000 s is the clock's, not the reference's.
But past ~1000 s the Rb's own RWFM/drift climbs to and beyond the
budget, so a long-τ chA TDEV bulge there **cannot be cleanly attributed**
to the clock vs the Rb.  *Corollary: do not grade the moonshot on
single-clock chA-vs-Rb TDEV at long τ — you would be measuring the
FE-5680A as much as the clock.*

**Differential (chA − chB on one shared TICC).**  The Rb is the common
timebase for *both* channels, so its instability — **including its
random walk** — is a common-mode term (§2) that **cancels** in
chA − chB.  The shared-reference differential is therefore *immune to
the Rb's random walk*: its floor is `σ_meas ≈ 5-10 ps`, not the Rb's
~12 ps→1 ns TDEV.  The Rb only has to hold over the sub-second interval
between the two edges the TICC timestamps within a given second, where
it is sub-picosecond, so the cancellation is essentially perfect.

**This is the reason the budget is written as σ_Δ measured on a shared
TICC** (§6.3 / §7): the random-walk budget is measurable down to the
carrier-phase floor *regardless of the reference's own random walk*.
The Rb's long-term stability is therefore almost a red herring for the
budget — it would become the limiter only if we tried to characterize
one clock's *absolute* random walk past ~1000 s, which the differential
budget never asks for.

### 3.2 Servo residual σ_servo_residual(τ)

The loop's ability to drive innovation to zero at τ.  For a
well-tuned LQR/Kalman servo (which we have — `kalman_servo.py`,
`do_freq_est.py`), this is set by `(σ_meas / SNR_loop) · (τ_loop / τ)`.
At τ=1 s, with loop time-constant ~1 s, this is dominated by
σ_meas — i.e., ~10 ps.  Fine.

The concern is **plant-model error** — if `dac_ppb_per_code` is
mis-scaled by factor α, the EKF systematically under- or
over-corrects, and σ_servo_residual carries a bias term `∝ u·(1−α)`.
That term is detected by the **innov-vs-control monitor**
(`innovControlMonitor-main`, landed 2026-05-13, code in
`scripts/peppar_fix/innov_control_monitor.py`).  Treat it as a
separate hazard with its own gate, not an additive σ here.

#### 3.2.1 Gate-lockout failure mode (revised 2026-06-01)

The 10-20 ps estimate above assumes the loop is *actually closed* —
i.e., the arms supplying load-bearing phase information are being
admitted to the state update.  In practice we observe the opposite:

The TICC arm carries a hard chi² gate at `K=100` (≈10σ).  During
real-world transients — physical phase steps from cable disturbance,
multi-second EKF state drift after a long coast, even just a
brief PPP outlier feeding back through the coupled `H_ticc =
[-1, 0, -1, 0]` — the arm's innovation lands above threshold and is
*entirely* rejected.  The EKF then runs on TDCP frequency only,
with no phase observation.  The DO drifts open-loop on phase at
its free-running rate (mid-tau OCXO drift is real once you remove
the phase feedback).

When the gate finally relents (which it does as predicted variance
S grows during the coast, eventually crossing the innovation), the
EKF snaps to the accumulated TICC error in a single epoch.  That
snap is the disturbance the cold-start cascade dance (Bob's "4
wrong starts before lock" observation) is built from, and the
mid-tau TDEV bulge it leaves behind is the budget failure.

**Empirical evidence (overnight 2026-05-31 → 2026-06-01, three
OCXO-class hosts, full moonshot servo stack with --router-qvir
--coast-cap --max-adjfine-step-ppb 2.0 --servo-input tdcp):**

```
TDEV(τ) [ps]      τ=1s   τ=10s   τ=50s   τ=100s  τ=200s  τ=500s
PiFace             88     309    506     607     631     589
clkPoC3           148     270   1655    3537    3457    2821
MadHat            103     129    944    1471    1846    1250
budget target     354    354    354     354     354     354
```

PiFace is at most 2× over the per-clock target and shows a gentle
monotonic positive slope.  clkPoC3 hits **10× over** at τ=100 s.
MadHat is **5× over** at τ=200 s.  Same OCXO class, same SSR mount,
same servo flags — but a 5× spread in mid-tau noise.  The delta
between PiFace and the other two is gate-rejection behavior, not
DO noise floor (per §3.3, the DO floor is ~10 ps — three orders of
magnitude below the budget).  The TimeHat 1.85 µs excursion on
2026-06-01 02:54 UTC after a 345 ns TICC step is the same
mechanism in extremis: 70 s of chi²-locked coast, ~1.5 µs of
open-loop drift, then a corrective snap.

**Updated σ_servo_residual model.**  When the rejection rate of
load-bearing phase arms exceeds a threshold (empirically: above
~90 % over a window > the DO's mid-tau coherence time), σ_servo_
residual is no longer bounded by `(σ_meas / SNR_loop)`.  It is
bounded above by the *integrated DO free-running phase drift over
the coast window*:

```
σ_servo_residual(τ) ≤ σ_y(τ_DO) · τ_coast     for τ > τ_coast
```

where `τ_DO` is the DO's intrinsic coherence time and `τ_coast` is
the typical lockout duration (50-200 s in observed data).

Implication: **the per-term decomposition in §3 is correct only
under a soft-gate or always-admit architecture.**  Under hard chi²
rejection at K=100, σ_servo_residual swallows the entire 354 ps
budget on hosts where the gate rejects more than the lockout
threshold.

**Architectural fixes on the table (`docs/pull-attribution-mid-
tau-2026-06-01.md` for the analysis, PR #118 for the diagnostic
schema):**

- **Soft gate**: R-inflation by `max(1, (innov/(K·√S))²)` instead
  of binary reject.  Admits all samples; the EKF's own variance
  weighting handles outliers proportionally.  Most-promising
  single fix.
- **Slow innovation-mean integrator**: feed `mean(innov/√S)` from
  the existing `InnovControlMonitor` window back into a slow
  state-correction term.  Captures the long-term mean of
  rejected samples (which still carries information; only the
  per-epoch high-frequency content is noisy).  Independent of
  the gate; complementary to soft-gate.
- **Per-arm pull-attribution logging** (PR #118): observability
  prerequisite.  Without per-arm `K · innov` logged, no way to
  validate either fix or to diagnose which arm's information is
  being lost on a given host.

**σ_servo_residual under the current hard-gate architecture:
50-3500 ps depending on host and τ.  Under the proposed soft-gate
+ slow-integrator architecture: projected back to the original
10-20 ps estimate, *to be empirically validated*.**

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

**Priority note (revised 2026-06-01).**  The 18-bit DAC upgrade was
ordered as work item #3 in §8 before mid-tau data was clean.  With
2026-06-01 overnight data showing σ_servo_residual blown out by
~100× from gate-rejection behavior (§3.2.1), the 14 ps quantization
delta between 16-bit and 18-bit is *not the current limiter on any
host*.  The DAC upgrade remains a good investment for headroom but
is no longer the highest-leverage next step.  See §8 for revised
work-item ordering.

---

## 4. Shared-antenna case — math, achievability

### 4.1 Budget summary

```
shared antenna, 95% excursion ≤ 1 ns:
  σ_Δ(τ) ≤ 500 ps      at all τ from 0.1s out to 1000s
  → σ_clock(τ) ≤ 354 ps per clock
```

### 4.2 Per-term allocation (OCXO + 16-bit DAC + 1 Hz servo)

**Theoretical (assumes loop closed across all arms):**

| Term | Allocation | Theoretical actual | Empirical actual (2026-06-01) |
|---|---|---|---|
| σ_meas (TD-CP) | 100 ps | 10 ps | ~10 ps ✓ |
| σ_servo_residual | 100 ps | 10-20 ps | **88-3500 ps at τ=100s** ✗ |
| σ_DO_above_BW | 200 ps | 10 ps (OCXO) | ~10 ps ✓ |
| σ_actuator_q (16-bit) | 50 ps | 18 ps | ~18 ps ✓ |
| **RSS** | **354 ps** | **~25 ps (theory)** | **88-3500 ps (actual)** |

**Verdict revision (2026-06-01).**  Three of four terms are at or
near their theoretical estimates; σ_servo_residual is **the
binding term in practice** because the chi² hard gate rejects
load-bearing phase observations during transients (see §3.2.1).
The "~50-100 ps actual sub-ns floor" prediction from the original
draft is **correct at τ=1 s** (we see 88 ps on PiFace, 148 ps on
clkPoC3, 103 ps on MadHat) but breaks badly at τ=100 s on hosts
where the gate rejects more than its lockout threshold.

### 4.3 Risks

- **Gate-lockout pathology** (§3.2.1).  The current binding
  constraint.  Mitigation: soft-gate + slow-integrator
  architecture, observability prerequisite landing as PR #118.
- **Plant-model error not caught**.  `innovControlMonitor-main`
  closes this — landed 2026-05-13, currently observer-only.
  Architecture A (slow integrator from monitor's `norm_bias`)
  would close the feedback loop.
- **Servo oscillation / limit cycle**.  The clkPoC3 day0512 TDEV
  hump at τ=128s (63.5 ns) — now identified as a milder version
  of the same gate-lockout dynamics observed on 2026-06-01.
- **Per-host calibration drift**.  PiFace has TICC + EXTINT bias
  calibrated against otcBob1 (2026-05-06); MadHat and TimeHat
  have none.  Uncalibrated bias on TICC means the servo drives
  the DO to *path-asymmetry-zero*, not GPS-zero — see Bob's note
  in this conversation thread.  EXTINT residual on MadHat is
  −40 ns largely from this.
- **Local TICC drift contaminating chA-alone TDEV measurement** —
  measurement-side artifact, not in σ_Δ.

### 4.4 Validation

**Original spec:** two PiFace-class hosts (OCXO + DAC + same
firmware) on the UFO1 splitter, both running the disciplining
loop.  Measure σ_Δ directly via TICC differential (chA-chB on a
shared TICC, eliminating that TICC's own reference noise).  Expect
σ_Δ ≤ 200 ps RMS, max excursion in 1 h ≤ 1 ns.

**Current empirical result (2026-06-01, virtual-pair CDFs across
the four-host moonshot fleet, ~9 h overlap window, hard chi² gate
at K=100):**

| Host pair | p50 | p95 | max | Status vs 1 ns @ 95% |
|---|---|---|---|---|
| PiFace ↔ MadHat | 2.5 ns | **6.9 ns** | 19.7 ns | 6.9× over |
| PiFace ↔ clkPoC3 | 4.9 ns | 13.1 ns | 28.7 ns | 13× over |
| clkPoC3 ↔ MadHat | 4.9 ns | 14.8 ns | 34.3 ns | 15× over |
| (TimeHat-paired) | 15.6 ns | 45-49 ns | 1.85 µs* | TCXO, best-effort only |

*TimeHat peak was a single 1.85 µs excursion attributable to the
chi² lockout pathology in §3.2.1 (345 ns TICC step at 02:54 UTC →
70 s of EKF coast → ~1.5 µs of open-loop DO drift → corrective
snap).  Other TimeHat windows are at the 45-50 ns p95 floor of the
TCXO.

**Best pair (PiFace ↔ MadHat) is 6.9× over the 1 ns target.**  The
2 × σ_Δ math from §4.2 says we need per-clock σ ≈ 354 ps; we're
measuring per-clock σ ≈ 2.5 ns (back-derived from p95).  ~7×
above budget, consistent with the §3.2.1 finding that σ_servo_
residual is currently swallowing the budget.

Acceptance test stays as defined; we are **not passing it** under
the current hard-gate architecture.  Re-run after soft-gate or
slow-integrator lands to verify the prediction that fixing
σ_servo_residual recovers the budget headroom.

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

**Current fleet inventory (2026-06-01):**

| Host | DO | Class | TDEV(1 s) measured | TDEV(100 s) measured | Notes |
|---|---|---|---|---|---|
| PiFace | CTI OSC5A2B02 | OCXO | 88 ps | 607 ps | Calibrated TICC + EXTINT bias against otcBob1 (2026-05-06).  Cleanest mid-tau of the fleet. |
| clkPoC3 | OCXO (per state/dos/ocxo-clkpoc3-v2.json) | OCXO | 148 ps | 3537 ps | TICC bias calibrated transitively against PiFace; EXTINT wired (96.3% admit in 2026-06-01 steady-state, +9.17 ns persistent innov bias, uncalibrated).  Worst mid-tau bulge — gate-lockout pathology most pronounced. |
| MadHat | IsoTemp OCXO-33 | OCXO | 103 ps | 1471 ps | No TICC bias calibrated.  EXTINT wired 2026-06-01, observed -40 ns innov bias (uncalibrated).  Mid-tau between PiFace and clkPoC3. |
| TimeHat | i226 internal TCXO | TCXO | 2.7 ns | 15.2 ns | Designated best-effort.  Not graded against budget. |

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

**DO-PPS observer — per-clock TICC vs free EXTINT (2026-06-10).**
A related measurement-chain *cost* question: to discipline the DO, is
a per-clock TICC (~60 ps, but a board + serial port + the DTR-reset
gotcha each) worth it over the **free** EXTINT phase the receiver
already provides (F9T TIM-TM2 / PHC EXTTS, ~5–10 ns)?
A controlled interleaved A/B on clkPoC3 (both/extint ×3, same
conditions, all locked, 0 re-acquire / 0 exit-5) says **no**:
EXTINT-only TDEV(1 s) = **52 ps** (runs 51/53/52) *beat* the fused
"both" = **92 ps** (90/117/90) at every τ.  The gentle EXTINT observer
disciplines cleaner; the tight TICC over-actuates.  So for the moonshot
budget a **per-clock TICC is not justified** — σ_meas is not the limiter
either way (§6.4 above) — and a TICC is better deployed as a *shared
validation* instrument than one-per-clock.  Full study:
[ticc-vs-extint-do-observer-experiment.md](ticc-vs-extint-do-observer-experiment.md).
Prerequisite: [[soloObserverChiGate]] (§3.2.1 / §8.1) removed the
chi²-gate lockout that had made a *sole* TICC observer unmeasurable, so
the comparison is now fair.

> **Skepticism — not yet proven.**  This verdict rests almost entirely
> on **one host (clkPoC3, OCXO)**.  Cross-host replication has been
> repeatedly foiled by host-reliability faults (PiFace drifts on every
> arm; MadHat ms-offset + intermittent chA; TimeHat chA-channel drops —
> tracked in `multiHostReliability`).  Supporting but weak: the first
> moonshot overnight pointed the same way on PiFace (extint ≤ both ≤
> ticc @1 s), and a one-off run that showed the *opposite* (xh3 extint
> 397 ps) proved non-reproducible.
>
> **Replication attempt #1 (PiFace, 2nd OCXO, 2026-06-10): inconclusive.**
> A matched interleaved A/B on PiFace was too noisy to call — EXTINT
> drifted on 2 of 3 runs (+62/+132 ppb, likely the F9P TIM-TM2 / EXTINT
> path, not the OCXO), "both" wandered at long τ on 2 of 3 (28–35 ns @10 s,
> chA following a noisier F9P/GPS reference), and the extint-vs-both
> ordering @1 s flipped with aggregation (medians agree, cleanest-run-each
> disagrees). It neither confirmed nor refuted. PiFace's OCXO *does* lock
> clean on its good runs (both-r1: −0.008 ppb, 295 ps @1 s) — the host's
> measurement/EXTINT chain is the limiter, not the DO.
>
> Until **EXTINT < both is replicated cleanly on ≥2 independent DO hosts**
> (ideally an OCXO *and* a different DO class), treat "per-clock TICC not
> justified" as **strongly indicated, not established** — a single OCXO's
> actuator/loop realization could be flattering EXTINT.  TimeHat (TCXO,
> different DO class) is the next replication candidate once its chA holds.

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

**Revised 2026-06-01.**  Original ordering pre-dated mid-tau data
showing that σ_servo_residual under hard chi² gating is the
binding constraint, not the actuator term.  Re-ordered: gate
architecture before DAC upgrade.

### 8.1 Gate-rejection architecture (current binding constraint)

1. **Land PR #118 `pullAttributionLog`** — per-arm K, would_pull,
   chi² columns in arm-state.csv.  Observability prerequisite for
   any architectural fix; the schema reveals exactly which arm's
   information is being lost on each host.  Status: draft PR open,
   deployed and validated on clkPoC3 + MadHat overnight.

2. **Land soft-gate (R-inflation) on TICC arm.**  Replace the
   binary `chi2 > _CHI2_GATE_THRESHOLD → skip update` path with
   `R_effective = R · max(1, (innov / (K·√S))²)`.  Admits all
   samples; the EKF's variance weighting handles outliers
   proportionally.  Single-host A/B run validates: TDEV(100s) on
   clkPoC3 should drop from 3.5 ns toward PiFace's 0.6 ns.

   **Observability-override required.**  R-inflation IS de-weighting,
   which collides with the `sole_observer_cannot_be_deweighted`
   finding (memory item, 2026-05-29): on a host where one arm is the
   sole observer of a state dimension, de-weighting that arm
   starves the state.  Soft-gate must inherit the observability
   floor that #95's OCXO physical gate already enforces: when an arm
   is the only one observing a state, R-inflation must be bounded
   so the arm continues to update the state (the gate-vs-noise
   trade-off becomes "admit with bounded weight" instead of "admit
   with arbitrarily inflated R").  The current fleet has multiple
   x[2] observers on all OCXO hosts (TICC + EXTINT both wired on
   PiFace, clkPoC3, and MadHat as of 2026-06-01) so the sole-observer
   case is not currently triggered, but the override must be in the
   implementation before any future host or arm-disable scenario can
   regress.

   **Step-2/step-3 ordering is host-dependent.**  On a sole-observer
   host, slow-integrator (#3, adds information) or bias calibration
   (#4) is the safer first move; soft-gate alone could under-correct
   while waiting for the slow path to engage.  On multi-observer
   hosts, either ordering works.  The clkPoC3 prediction in #2 is
   itself the test: if soft-gate doesn't drop clkPoC3's TDEV(100s),
   sole-observer de-weighting is the most likely cause and #3 needs
   to land alongside.

3. **Land slow innovation-mean integrator (Architecture A).**
   Feed `mean(innov/√S)` from the existing
   `InnovControlMonitor` rolling window back into a slow
   `x[1]`/`x[2]` correction term with gain β tuned empirically.
   Captures the long-term mean of rejected samples — the
   information that the gate currently throws away.  Complementary
   to #2; either alone may close the budget but both together is
   the design Bob and main converged on this morning.

   Per the host-dependent ordering note in #2: on sole-observer
   hosts, land this BEFORE soft-gate.  On multi-observer hosts,
   either order; the validation gate is the same.

4. **Per-host TICC + EXTINT bias calibration.**  PiFace was
   calibrated against otcBob1 PPS on 2026-05-06 (1131 samples,
   20 min, σ_about_bias=2.35 ns).  MadHat needs the same against
   the same reference.  TimeHat too once it's a sync candidate
   (today: best-effort only).  Tool exists at
   `scripts/calibrate_timestampers.py`; procedure in
   `docs/dofreq-est-measurement-ladder.md`.

### 8.2 Plant model (next-highest leverage)

5. **`innovControlMonitor-main` — already landed** (2026-05-13).
   Currently observer-only.  Closing the feedback loop is step 3
   above.

6. **`dacPpbSignClean-main`** — eliminate the most-likely source
   of plant-model error.  Pair with #5 as the validation gate.

### 8.3 Hardware (post-gate-fix, secondary)

7. **Upgrade to 18-bit DAC architecture.**  AD5781 + LTC6655.
   Re-prioritized after gate-fix lands: at 18 ps quantization
   noise, this term is ~5× below the soft-gate-fixed σ_servo_
   residual estimate (~100 ps), so the DAC upgrade buys margin
   but isn't the limiting term.  Lab-host hardware work, not
   software; file as `hw:`-labeled item once electronics ordered.

8. **(Stretch) Two-site separate-antenna run** — needs Bob's
   second antenna site to be production-ready.  PPP-AR + same
   SSR mount.

9. **(Optional experiment) SiT5358 eval-board pair** —
   characterize native-digital-OCXO architecture as an
   alternative path.

### 8.4 Validation

10. **Re-run §4.4 shared-antenna acceptance test** after items
    #1-3 land.  Same fleet, same SSR mount, same wall-clock window.
    Pass criterion: p95 ≤ 1 ns across all OCXO-host pairs.
    Current: p95 = 6.9 ns (PiFace ↔ MadHat best).

---

## 9. Open questions

- **~~Empirical TDEV slope~~** for our current OCXO+AD5693R hosts
  at τ=1-100s — is it actually WFM (slope −1/2) or closer to FFM
  (slope 0)?  ~~Re-analysis of clean day0512 PiFace data can
  answer.~~  **Answered 2026-06-01.**  The observed slope on
  clkPoC3 between τ=10 s and τ=100 s is approximately **τ¹·¹** —
  *positive* — between WFM (−1/2) and the budget-failure regime
  of RWFM (+1/2).  This is the gate-rejection signature (§3.2.1),
  not intrinsic DO or atmospheric noise.  PiFace shows a gentler
  positive slope (~τ⁰·²) in the same window, consistent with
  occasional gate-rejection events rather than sustained lockout.
  The 95%-excursion math in §2 turns out to be **optimistic under
  the current architecture** — it presumed bounded servo
  residuals that don't hold during gate lockouts.  Math becomes
  conservative again once §8.1 fixes land.

- **Loop dynamics** at servo BW — is the LQR gain critically damped
  in practice?  The clkPoC3 day0512 hump at τ=128s suggests
  maybe not.  Innov monitor (landed) confirms periodic
  control-vs-innov correlation excursions; PR #118 lets us
  attribute them per-arm.

- **Per-arm Q-tuning from DO ADEV characterization.**  Current
  Q[1,1] (clock-rate process noise) is hand-tuned per host.  The
  textbook Kalman result says Q should derive from
  `σ_y²(τ_loop) · τ_loop` for the specific DO.  Per-DO Q reads
  out of `state/dos/<do_label>.json` once the characterization
  process is complete (`committedDoCharacterization` dayplan
  thread, in flight).  Likely improvement on hosts where the
  current Q is mis-set.

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
*Revised: 2026-06-01 (main) — empirical mid-tau data, gate-lockout
finding, work-item re-ordering, fleet inventory.  See conversation
thread in this session for the data and analysis path.*
