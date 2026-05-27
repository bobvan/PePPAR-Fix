# GNSS TDCP: A Short-Tau Timing Perspective

Two-page literature survey focused on the short-tau time-stability angle.
Filed 2026-05-24 by charlie at Bob's request.

## 1. What TDCP is

Time-Differenced Carrier Phase (TDCP) is the per-epoch difference of a
receiver's raw carrier-phase observable to the same satellite, evaluated
over a short interval Δt (typically 0.1–1 s).  The integer cycle
ambiguity N cancels in the difference under continuous lock, leaving:

```
Δφ(sv, t→t+Δt) = Δρ_geom(sv) + Δ(c·dt_rx) − Δ(c·dt_sat) + Δ(iono, tropo) + ε
```

Subtracting the broadcast-ephemeris-predicted Δρ_geom and Δ(c·dt_sat)
leaves a residual dominated by the change in receiver clock plus
mm-level measurement noise.  Differencing across an ensemble of SVs
(median or LS) yields a robust per-epoch estimate of `c·Δdt_rx`, and
dividing by Δt gives the receiver's fractional-frequency offset `df/f`
at that timescale.  The technique is attractive precisely because
ambiguity resolution — the load-bearing step in absolute-phase PPP —
is sidestepped.

## 2. Where TDCP is mostly used today

The dominant published applications of TDCP are **not** clock work.
The literature concentrates on three areas:

- **Precise GNSS velocity estimation.**  The canonical reference (Freda
  et al., *GPS Solutions* 2014) demonstrated 3 mm/s RMS at 1 s using
  TDCP, vs cm/s for the Doppler observable.  The advantage is the
  inherent ~mm precision of carrier phase compared to ~m for code
  phase, combined with strong temporal correlation of atmospheric and
  orbit/clock errors that cancel in the difference.  The result has
  been reproduced down to commodity hardware — Pirazzi et al.
  (*Sensors*, 2022) reported mm/s-class TDCP velocity from a smartphone
  receiver.

- **Tight GNSS/INS coupling.**  Wendel & colleagues' *"Approach to aid
  INS using TDCP measurements"* (2006) established the EKF-state
  architecture: TDCP enters as a low-noise velocity observation, with
  receiver clock bias and drift estimated jointly as filter states.
  Modern variants (e.g., Liu et al., *Measurement* 2024 — Doppler/TDCP
  fused via factor-graph optimization) target urban robustness, where
  Doppler degrades under multipath but TDCP retains useful precision
  when carrier lock survives.

- **Cycle-slip and authentication.**  TDCP is itself the per-SV
  detection signal for cycle slips (Liu et al., *Sensors* 2016 —
  geometry-based TDCP slip detector).  More recently, Khanafseh et al.
  (*NAVIGATION* 72:2, 2025) used the spatial coherence of TDCP
  residuals across satellites in a software-defined receiver to
  identify spoofing and signal authenticity.

These applications share a common feature: TDCP is consumed as a
*measurement* in a filter that estimates kinematic state, with clock
terms as nuisance parameters.  The clock terms are well-behaved
by-products.

## 3. TDCP for short-tau time stability — what the literature does and doesn't say

There is a noticeable gap in the published TDCP literature: very few
papers treat the **receiver clock rate** as the *target* observable
rather than a nuisance.  Most carrier-phase timing work has gone in a
different direction.

**Carrier-phase time transfer (CPTT, PPP-based)** dominates the precise-
timing literature.  NIST receivers, characterized by Yao et al.
(*Metrologia* 2015), achieve frequency stability of 4.0×10⁻¹⁶ at 3
hours, 1.1×10⁻¹⁶ at 1 day, dropping into the low 10⁻¹⁷ range at 10+
days.  Su et al. (*Sensors* 2020), *"Fast Time Synchronization on Tens
of Picoseconds Level Using Uncombined GNSS Carrier Phase of Zero/Short
Baseline,"* reach 5×10⁻¹⁴ at 30,000 s using a zero-baseline common-clock
setup.  Lyu et al. (*Remote Sensing* 16:21, 2024) show that BDS
penta-frequency CV-SD reduces noise by ~8.5% on top of the
dual-frequency baseline.  Krawinkel & Schön (2023) demonstrated
chip-scale atomic clock disciplining via PPP with broadcast
ephemerides, holding sub-ns offset to UTC over hours-to-days.

All of these are **PPP-based**: they estimate the *absolute* receiver
clock bias using float ambiguity-resolved carrier phase.  The short-tau
behavior is reported on the disciplined-clock output, not on the
differential signal itself.

The TDCP-specific short-tau timing literature is comparatively sparse.
The closest match is the zero-baseline carrier-phase work: Yao et al.
note explicitly that **at averaging times < 0.5 hour the dominant noise
source is the antenna and antenna cable**, not the satellite or
atmospheric signal.  This implies that for short-tau (sub-second to
sub-minute) work, the achievable performance is bounded by
receiver-side hardware: the rx-TCXO's free-running phase noise, the
antenna RF chain, and the receiver's carrier-tracking loop bandwidth —
not by GNSS-side error sources.

This is the relevant framing for using TDCP as a **disciplining input**
to a local oscillator.  TDCP provides a direct, low-noise observation
of the receiver clock rate.  Over a 1-second interval with a 16-SV
ensemble at L1, the carrier-phase noise of ~3 mm per SV translates to
a per-epoch ensemble noise floor near 1 mm — equivalent to roughly 3
ps of phase or ~3×10⁻¹² of fractional frequency.  The dominant
contribution at this scale is not estimator noise but the *real* phase
walk of the receiver's own TCXO during Δt.

**This points to a research opportunity that the published literature
has not extensively explored**: rather than treating the receiver clock
as something to be solved out (PPP) or canceled out (single-difference
between-satellite), TDCP can be inverted to *characterize* the receiver
clock's short-tau motion with picosecond-class precision.  A
disciplining loop can then steer a downstream physical oscillator
against this measurement, with the steering quality limited by (a) the
rx-TCXO's own short-tau noise floor as the measurement-chain ceiling,
and (b) the actuator's resolution.

## 4. Limitations of TDCP for timing

Three constraints temper enthusiasm:

1. **Δt scaling of satellite-clock variation.**  TDCP performance
   degrades when Δt is large enough for the satellite clock variation
   to exceed broadcast-ephemeris fit quality (typically beyond 30–60 s
   with broadcast products).  Short-tau is the regime where TDCP
   shines; for long-tau timing, PPP with precise SP3+clk products
   dominates.

2. **Cycle slips become frequency steps.**  A missed slip on one SV
   produces a phase jump of ~0.19 m on L1, equivalent to a 0.6 ppb
   spike at 1 Hz.  Robust per-epoch MAD outlier rejection handles
   isolated slips; correlated slip events (e.g., sunrise, ionospheric
   storms across rising satellites) defeat naive median filtering.
   The slip-detection literature (Liu et al. 2016 and successors) is
   directly relevant.

3. **No absolute time anchor.**  TDCP measures rate, not bias.  A
   disciplined oscillator initialized off-truth will track GPS time
   only in *frequency*, not phase.  A separate phase reference —
   PPS+qErr, PPP dt_rx, or a hardware tick — must close the long-tau
   loop.

## 5. Summary

TDCP's published prominence is in velocity and INS aiding, with timing
applications historically routed through PPP-based architectures that
don't expose the TDCP signal directly.  The literature on short-tau
receiver-clock characterization via TDCP is thin.  The zero-baseline
carrier-phase time-transfer results imply that picosecond-class
short-tau performance is reachable when the dominant noise sources are
receiver-side hardware rather than the atmospheric/orbit signal.  This
is the regime where TDCP-as-servo-input is most likely to displace
traditional PPS-based GPSDO disciplining — particularly when paired
with an OCXO-class disciplined oscillator whose own free-running noise
floor sits below the GNSS measurement chain's ceiling.

---

## Sources

- [Time-differenced carrier phases technique for precise GNSS velocity estimation (Freda et al., GPS Solutions 2014)](https://link.springer.com/article/10.1007/s10291-014-0425-1)
- [Time-Differenced Carrier Phase Technique for Precise Velocity Estimation on an Android Smartphone (Sensors 2022)](https://www.mdpi.com/1424-8220/22/21/8514)
- [Reliable velocity determination through GNSS TDCP/Doppler combination using an improved FGO framework (Measurement 2024)](https://www.sciencedirect.com/science/article/abs/pii/S0263224124022371)
- [An approach to aid INS using time-differenced GPS carrier phase (TDCP) measurements (Wendel et al.)](https://www.researchgate.net/publication/227316052_An_approach_to_aid_INS_using_time-differenced_GPS_carrier_phase_TDCP_measurements)
- [A Geometry-Based Cycle Slip Detection and Repair Method with TDCP (Liu et al., Sensors 2016)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5191045/)
- [Identification of Authentic GNSS Signals in TDCP Measurements with an SDR Receiver (Khanafseh et al., NAVIGATION 72:2, 2025)](https://navi.ion.org/content/72/2/navi.698)
- [A Study of GPS Carrier-Phase Time Transfer Noise Based on NIST GPS Receivers (Yao et al., Metrologia)](https://pmc.ncbi.nlm.nih.gov/articles/PMC7339619/)
- [Fast Time Synchronization on Tens of Picoseconds Level Using Uncombined GNSS Carrier Phase of Zero/Short Baseline (Sensors 2020)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7506859/)
- [Carrier Phase Common-View Single-Differenced Time Transfer via BDS Penta-Frequency Signals (Remote Sensing 16:21, 2024)](https://doi.org/10.3390/rs16213955)
- [Precise disciplining of a chip-scale atomic clock using PPP with broadcast ephemerides (Krawinkel & Schön 2023)](https://www.researchgate.net/publication/372192662_Precise_disciplining_of_a_chip-scale_atomic_clock_using_PPP_with_broadcast_ephemerides)
- [Evaluation of carrier-phase precise time and frequency transfer using different analysis centre products for GNSSs](https://www.researchgate.net/publication/331723327_Evaluation_of_carrier-phase_precise_time_and_frequency_transfer_using_different_analysis_centre_products_for_GNSSs)
- [100 Picosecond / Sub-10⁻¹⁷ Level GPS Differential Precise Time and Frequency Transfer (Applied Sciences 13:19, 2023)](https://www.mdpi.com/2076-3417/13/19/10694)
