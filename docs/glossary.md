# PePPAR Fix Glossary

Terms used in code, docs, and lab notes.  When a term has multiple
meanings in different contexts, the PePPAR-Fix-specific meaning is
listed first.

## Oscillators and clocks

| Term | Definition |
|---|---|
| **DO** | Disciplined Oscillator.  The crystal whose frequency is steered by the servo.  On TimeHat/MadHat this is the i226 TCXO controlled via `adjfine()`.  On Timebeat OTC this is the on-board OCXO controlled via ClockMatrix FCW.  Use "DO" when the statement applies regardless of the actuator.  Every PHC has a DO at its core, but a DO can exist without any PHC. |
| **PHC** | PTP Hardware Clock.  The Linux kernel's interface to a NIC's hardware clock (`adjfine()` / `clock_settime()` / `EXTTS`).  Separable from the DO — a PHC always has a DO (some crystal is ticking under it), but not every DO has a PHC.  Use "PHC" only when referring to the kernel interface; use "DO" when talking about the underlying oscillator. |
| **RO** | Reference Oscillator.  The oscillator driving a TICC's 10 MHz reference input.  Parallels the DO: the RO sets the timescale the TICC uses to report edge timestamps.  We can't steer the RO, but we expect and assert long-term stability by comparing it against GPS time via the F9T PPS train on chB.  See `docs/future-work.md` "Reference Oscillator (RO) characterization". |
| **rx TCXO** | The TCXO inside the GNSS receiver (F9T).  Drives the receiver's 125 MHz clock and determines where PPS edges fire (quantized to the 8 ns tick grid).  Do NOT use bare "TCXO" — it's ambiguous with the DO's crystal on i226 hosts, which is also a TCXO. |
| **OCXO** | Oven-Controlled Crystal Oscillator.  Higher stability than TCXO.  The DO on Timebeat OTC boards. |
| **adjfine** | PHC frequency adjustment.  Sets the clock rate in parts-per-billion via the `PTP_CLOCK_SETFINE` ioctl. |
| **FCW** | Frequency Control Word.  The digital command that sets the Renesas ClockMatrix DPLL output frequency. |

## GNSS and corrections

| Term | Definition |
|---|---|
| **GNSS** | Global Navigation Satellite System.  Umbrella for GPS, Galileo, BeiDou, GLONASS. |
| **PPP** | Precise Point Positioning.  Single-receiver positioning using satellite orbit and clock corrections from an SSR stream.  Achieves sub-ns clock estimates after convergence. |
| **PPP-AR** | PPP with Ambiguity Resolution.  Fixes carrier-phase integer ambiguities using phase biases from a single analysis center.  3x lower dt_rx noise than float PPP. |
| **SSR** | State Space Representation.  Satellite-specific corrections (orbit, clock, code bias, phase bias) distributed via NTRIP.  Contrasts with OSR (Observation Space) like VRS/RTK. |
| **NTRIP** | Networked Transport of RTCM via Internet Protocol.  The standard for streaming GNSS corrections over the internet. |
| **RTCM** | Radio Technical Commission for Maritime Services.  The binary message format for GNSS corrections (1019, 1045, 1060, 1265, etc.). |
| **ISB** | Inter-System Bias.  The constant time offset between two GNSS constellations (e.g., GPS-Galileo). |
| **ephemeris** | Satellite orbit and clock parameters.  "Broadcast" = from the satellite signal.  "Precise" = from SSR corrections. |
| **carrier phase** | The phase of the GNSS L-band carrier signal.  Sub-centimeter precision but ambiguous by an unknown integer number of wavelengths. |
| **integer ambiguity** | The unknown whole-cycle count in a carrier-phase measurement.  Resolving it ("fixing") eliminates a major noise source. |
| **phase bias** | Per-satellite, per-signal correction that makes integer ambiguity resolution possible.  Only available from single analysis center SSR streams (CAS, CNES, WHU). |
| **code bias** | Correction for signal-dependent pseudorange offsets between different GNSS signals. |
| **dt_rx** | Receiver clock offset from GNSS time, estimated by the PPP filter.  In nanoseconds.  Tracks the rx TCXO phase. |
| **NAV2** | Secondary navigation engine on u-blox F9T (and similar).  Runs in parallel with the primary fix, reports via UBX-NAV2-* messages (NAV2-PVT, NAV2-SAT, etc.).  PePPAR Fix uses NAV2 as an independent position cross-check against its own PPP-AR solution: `displacement = |AntPosEst − NAV2|`.  Both come from the same antenna but NAV2 is a single-epoch code-only single-freq solution (~1-2 m accuracy), so agreement within a few metres is expected; a 10 m+ displacement with NL fixed is strong evidence of wrong integers and triggers a filter reset.  `nav2Δ` in logs is this displacement. |
| **MW** | Melbourne-Wubbena combination.  Geometry-free, ionosphere-free linear combination of dual-frequency code and carrier phase that isolates the wide-lane ambiguity N_WL = N1 − N2.  Converges by averaging out code noise (~60 s).  First step in two-step PPP-AR. |
| **WL** | Wide-lane.  The carrier-phase combination with wavelength λ_WL = c/(f1−f2) ≈ 75 cm (L1/L5).  The wide-lane ambiguity N_WL = N1−N2 is integer and easy to fix from the MW combination because λ_WL is large. |
| **NL** | Narrow-lane.  The carrier-phase combination with wavelength λ_NL = c/(f1+f2) ≈ 10.7 cm (L1/L5).  The narrow-lane ambiguity N1 is the precision target for PPP-AR.  Resolved from the float IF ambiguity once N_WL is known. |
| **GF** | Geometry-free combination.  Φ₁·λ₁ − Φ₂·λ₂ in metres.  Cancels receiver-satellite geometry, clocks, and troposphere, leaving only ionospheric delay (mm/s drift) and any carrier-phase cycle slip.  Used by CycleSlipMonitor as the primary single-cycle slip detector: a sub-centimetre-per-epoch baseline versus any λ/4 jump makes it sensitive to one-cycle slips on either L1 or L5.  Complements MW — GF catches single-signal slips, MW catches multi-cycle WL-only events that leave GF unchanged. |
| **cycle slip** | A GNSS receiver's usually temporary inability to follow every cycle of a signal's carrier.  Weak signal, antenna obstruction, ionospheric scintillation, or receiver-loop stress can cause the tracking loop to lose and re-acquire a cycle, injecting an unknown integer-cycle jump into the accumulated phase.  Because carrier-phase ambiguities are integers in units of whole cycles, a slip invalidates any integer fix that depended on pre-slip phase — the receiver is tracking the carrier again, but the integer ambiguity has shifted.  `CycleSlipMonitor` runs four independent detectors per epoch (UBX locktime drop, arc-gap, GF jump, MW jump) and flushes all per-SV phase-like state on any detector firing.  See `docs/sv-lifecycle-and-pfr-split.md`. |
| **slip confidence (LOW vs HIGH)** | Our design grades each detected slip by how many independent detectors fired.  **LOW confidence** = exactly one detector fired.  Flushing still happens because a spurious flush costs only one ambiguity re-convergence — far cheaper than letting a contaminated integer through — but the SV goes to FLOAT and the MW tracker's wide-lane history is preserved.  **HIGH confidence** = two or more detectors fired concurrently, or UBX reports `locktime = 0` explicitly.  The SV transitions to SQUELCHED instead of FLOAT so no new AR attempts are made for the cooldown period.  The distinction governs the severity of the response, not whether the flush happens.  See `scripts/peppar_fix/cycle_slip.py:flush_sv_phase`. |
| **IF** | Ionosphere-free combination.  A linear combination of L1 and L5 that cancels first-order ionospheric delay.  Used for the PPP position filter's carrier-phase observable (pr_if, phi_if_m).  Its ambiguity is not integer — PPP-AR decomposes IF into WL+NL to get integers. |
| **CNES** | Centre National d'Etudes Spatiales (French space agency).  Provides single-AC SSR corrections on `products.igs-ip.net` mount `SSRA00CNE0`.  CNES phase biases match F9T Galileo signals (L1C, L5Q, L7Q) — the only confirmed source for PPP-AR. |
| **CAS** | Chinese Academy of Sciences.  SSR provider on `SSRA00BKG0`.  Phase biases use I-component (data) codes that don't match F9T's Q-component (pilot) observations.  Cannot be used for PPP-AR. |

## PPS and timing

| Term | Definition |
|---|---|
| **PPS** | Pulse Per Second.  A 1 Hz signal edge used as a timing reference.  Two PPS streams in the system: `gnss_pps` (F9T output) and `do_pps` (PHC PEROUT). |
| **gnss_pps** | The F9T's PPS output.  Fires at the nearest 125 MHz tick to the GPS second.  Subject to 8 ns quantization (qerr). |
| **do_pps** | The DO's PPS output (PEROUT on i226, SMA on E810).  What the servo disciplines. |
| **qErr** | Quantization error.  The sub-8 ns offset between the true GPS second and the rx TCXO tick where gnss_pps actually fires.  Reported by TIM-TP.  This is a **physical quantization** — the PPS edge snaps to a discrete 125 MHz tick grid.  Do not use "qErr" for PPP-derived corrections (those correct for rx TCXO drift, not tick quantization). |
| **PPS correction** | Any correction applied to a PPS measurement to improve its accuracy.  Two types: **qErr** (TIM-TP, corrects tick quantization, discrete ±4 ns) and **PPP drift-model correction** (from smoothed dt_rx, corrects rx TCXO drift, continuous ~0.1 ns).  CLI: `--no-qerr` disables qErr; `--pps-corr ppp` selects PPP drift-model instead of TIM-TP qErr. |
| **TIM-TP** | u-blox UBX-TIM-TP message.  Predicts the qErr of the **next** PPS edge.  Arrives ~0.9 s before the PPS it describes. |
| **TAI** | International Atomic Time.  Continuous timescale (no leap seconds).  TAI - UTC = 37 s as of 2026. |
| **sawtooth** | The periodic phase modulation of gnss_pps caused by the rx TCXO beating against GPS time.  Alternates between "smooth ramp" and "jumpy" regimes. |
| **holdover** | Free-running the DO when the GNSS reference is lost.  The DO drifts at its last-known rate. |

## Measurement instruments

| Term | Definition |
|---|---|
| **TICC** | Time Interval Counter/Counter.  TAPR open-hardware instrument.  60 ps single-shot resolution.  Measures two input channels (chA, chB) independently against an internal timebase. |
| **TAPR** | Tucson Amateur Packet Radio.  The organization that designs and sells the TICC. |
| **EXTTS** | External Timestamp.  Linux PTP subsystem feature: timestamps an external GPIO edge against the PHC clock. |
| **PEROUT** | Periodic Output.  Linux PTP subsystem feature: generates a periodic pulse (e.g., 1 PPS) from the PHC clock. |
| **TDC** | Time-to-Digital Converter.  Hardware that converts a time interval to a digital measurement (used in ClockMatrix). |

## Servo and control

| Term | Definition |
|---|---|
| **qVIR** | qErr Variance Improvement Ratio.  `Δvar(uncorrected) / Δvar(corrected)`.  Measures how much qErr correction reduces the variance of a PPS timestamp stream.  >1.5 = good (qErr is helping).  ≈1.0 = qErr is uncorrelated with the measurement (wrong epoch match).  <1.0 = qErr is making things worse (wrong sign or epoch).  Computed separately for EXTTS (qVIR_extts) and TICC (qVIR_ticc).  High qVIR correlates with low TDEV; low qVIR correlates with high TDEV.  Primary use: detect qErr mis-correlation early so we don't waste a run collecting data with terrible TDEV.  qErr has so much variance (±4 ns) that applying it to the wrong PPS edge is immediately visible in the ratio — smoother correction streams would mask the error. |
| **EKF** | Extended Kalman Filter.  Nonlinear state estimator.  Used in the 4-state DOFreqEst for TICC+PPP fusion. |
| **LQR** | Linear-Quadratic Regulator.  Optimal control law that minimizes a cost function.  Used in the Kalman servo for frequency steering. |
| **PI servo** | Proportional-Integral servo.  Simple feedback controller with gain (Kp) and integral (Ki) terms. |
| **LAMBDA** | Least-squares AMBiguity Decorrelation Adjustment (Teunissen 1995).  The standard algorithm for GNSS integer ambiguity resolution.  Decorrelates the float ambiguity covariance via integer Z-transforms (LDL^T decomposition + Gauss reductions), then searches for integer candidates in the compact decorrelated space.  Validates via ratio test and bootstrap success rate.  Handles ZTD-height-ambiguity cross-correlations that per-satellite rounding ignores. |
| **LDL^T** | Matrix decomposition where L is unit lower triangular and D is diagonal, such that Q = L D L^T.  Used by LAMBDA to decompose the ambiguity covariance.  The D values determine the search bounds and the bootstrap success rate per dimension. |
| **bootstrap success rate** | P(correct integers) estimated from the decorrelated covariance: P = ∏(2Φ(1/(2√D[i])) − 1).  Each dimension contributes independently.  Used as a gate before accepting LAMBDA's integer solution — if P < 0.999, the float covariance is too uncertain for reliable fixing.  Prevents premature AR that the ratio test alone cannot catch. |
| **PAR** | Partial Ambiguity Resolution.  When LAMBDA fails on the full set of ambiguities, iteratively drop the worst-determined ambiguity and retry with the smaller subset.  Continues until the subset passes both bootstrap success rate and ratio test, or until fewer than 4 ambiguities remain. |
| **ratio test** | LAMBDA validation: R = Ω(2nd best) / Ω(best).  Accept if R > threshold (typically 2.0).  Measures how much better the best integer solution is than the runner-up.  Necessary but not sufficient — a high ratio with low bootstrap P can still produce wrong integers. |
| **TTFF** | Time To First Fix.  Time from filter start to first accepted NL integer fix.  With LAMBDA + bootstrap gate: ~10-15 min.  With per-satellite rounding: ~37 min. |
| **integer fix** | A per-SV integer assignment of a carrier-phase ambiguity (WL, NL, or both).  Distinct from a *position fix*; we never overload "fix" for both — position gets "solution".  See SvAmbState.  **Strictly, integer ambiguities are per-signal**, not per-SV: each tracked carrier accumulates its own whole-cycle count.  But in our dual-frequency PPP-AR design, WL (`N_L1 − N_L2`) and NL (related to `N_L1 + N_L2`) combinations *require* both of an SV's signals, and the PPPFilter carries one IF ambiguity per SV (not per signal), so the state machine tracks fix status per SV.  `NL_SHORT_FIXED` for G17 means "both of G17's integers are fixed."  The per-signal layer still matters in three places: (1) cycle-slip detection (GF catches a single-signal slip), (2) `dual=N` SVs — only one expected signal tracked, AR impossible for that SV, (3) cross-profile comparisons — L2-profile receivers track L1CA+L2CL, L5-profile track L1CA+L5Q, different signal pairs under identical per-SV state. |
| **self-consistent integer set** | A set of per-SV integer ambiguities that, taken as a group, are mutually consistent with the observed dual-frequency phase data.  The long-term promoter proves the set remains self-consistent as satellite geometry evolves (≥ 15° of Δaz).  Long-term members form the set demonstrated self-consistent temporally and geometrically; short-term members were consistent at their promotion moment but are on probation. |
| **short-term promoter** | The code path that transitions WL_FIXED → NL_SHORT_FIXED.  Lives in `NarrowLaneResolver.attempt` (LAMBDA + rounding).  Its job is to **identify** a candidate self-consistent integer set from the float ambiguity covariance — a joint-MAP pick, not per-SV independent picks.  The result is internally consistent at that epoch's geometry; whether it remains so is the long-term promoter's question. |
| **long-term promoter** | Class `LongTermPromoter`.  Transitions NL_SHORT_FIXED → NL_LONG_FIXED after ≥ 15° of Δaz with a clean false-fix window.  Its job is to **validate** that the self-consistency identified by the short-term promoter survives geometric diversity. |
| **false-fix monitor** | Class `FalseFixMonitor`.  Detects individual short-term members whose integer **cannot** be consistent with the established long-term set, via sustained elev-weighted PR residual against the filter state (which the long-term set has built).  Action: transition the offending SV back to FLOAT (false-fix rejection).  Operates per-SV, not on the set as a whole. |
| **fix-set integrity alarm** | Class `FixSetIntegrityAlarm`.  Detects breakdown of self-consistency of the set **itself** — many members failing at once, not one misbehaving SV.  Almost always means a correction-source problem (bad SSR batch, datum shift) rather than an AR mistake.  Action: full filter re-init.  Expected < 1/day. |
| **position solution** | The antenna position computed by the AntPosEst instance from the fix set.  Has its own state machine (`AntPosEstState`: UNSURVEYED → VERIFYING → VERIFIED → CONVERGING → RESOLVED → MOVED). |
| **fix set** | The SVs whose integer ambiguities contribute to the current position solution.  Persistent across epochs; membership changes only via admission, promotion, demotion, or retirement.  Partitioned into short-term and long-term members.  See `docs/sv-lifecycle-and-pfr-split.md`. |
| **candidate fix set** | Transient, per-epoch.  The group of WL_FIXED SVs whose float NL ambiguities are being evaluated together as possible new admissions to the fix set.  The short-term promoter (LAMBDA or rounding) decides which candidates pass; passing candidates join the fix set as short-term members.  Most candidates aren't admitted on any given epoch — the set is an evaluation unit, not a state object.  Contrast with **fix set** (persistent) and the transient-vs-persistent pairing is the main distinction. |
| **LAMBDA candidate set** | The candidate fix set at the specific stage where the LAMBDA short-term promoter operates on it — joint decorrelation of the float covariance, integer search, ratio + bootstrap validation, then per-SV join-test before admission.  Same set as the candidate fix set in most epochs; term used when the LAMBDA-specific evaluation step is under discussion. |
| **short-term member** | An SV in state NL_SHORT_FIXED.  Has an accepted NL integer but has not yet demonstrated consistency with long-term members across geometry change.  Contributes its integer to the solution but does not count toward the RESOLVED declaration. |
| **long-term member** | An SV in state NL_LONG_FIXED.  Integer fix has survived ≥ 15° of satellite azimuth motion without false-fix rejection.  Counts toward RESOLVED. |
| **join test** | Pre-commit anchor-defense check run by the short-term promoter before admitting a candidate to the fix set.  Projects the candidate's predicted state change through each long-term member's cached PR H-row to estimate the induced |Δresidual| on that anchor, and rejects candidates that would push any anchor past the elev-weighted FalseFixMonitor threshold.  See `NarrowLaneResolver._join_test` and `project_to_main_defensive_mechanisms_20260421.md`. |
| **false fix** | An integer fix that was later shown to be wrong.  Detected by the **false-fix monitor** via sustained elev-weighted PR residual growth on a short-term member.  Action: transition the SV back to FLOAT (false-fix rejection). |
| **setting-SV drop** | The intentional removal of an NL fix as its satellite descends into multipath-prone elevations.  Triggered by the **setting-SV drop monitor** on either an elev-weighted residual threshold or an absolute drop-mask elevation. |
| **SvAmbState** | Per-satellite integer-fix state machine.  Six states: TRACKING (receiver sees the SV but not yet processed), FLOAT (admitted, no integer fix), WL_FIXED (MW wide-lane integer only), NL_SHORT_FIXED (short-term member of the fix set), NL_LONG_FIXED (long-term member), SQUELCHED (cooldown-bound exclusion after high-confidence cycle slip).  Orthogonal to `AntPosEstState`. See `docs/sv-lifecycle-and-pfr-split.md`. |
| **squelch** | Temporary exclusion of an SV from integer-fix attempts after a high-confidence cycle slip.  Cooldown-bound, not permanent; "squelched" is radio-receiver terminology chosen over "blacklisted" precisely because the latter connotes permanence.  Cooldown default 60 epochs. |
| **~~PFR~~** | *Deprecated.* Post-Fix Residual monitor (the single class `PostFixResidualMonitor` with a L1/L2/L3 escalation ladder) has been replaced by three stateless monitors: `FalseFixMonitor`, `SettingSvDropMonitor`, and `FixSetIntegrityAlarm`.  See `docs/sv-lifecycle-and-pfr-split.md`. |
| **LS** | Least Squares.  Used for initial position estimation before the PPP Kalman filter takes over. |
| **IF** | Ionosphere-Free combination.  Dual-frequency linear combination that cancels first-order ionospheric delay. |
| **ZTD** | Zenith Tropospheric Delay.  The extra signal path length (in meters) that a GNSS signal accumulates traveling straight down through the atmosphere — ~2.3 m at sea level.  Two parts: a *hydrostatic* (dry) component near 2.3 m, predictable from surface pressure and stable to a few mm/day, and a *wet* component (0–40 cm) driven by water vapor and variable by cm/hr with weather.  For a satellite at elevation `e`, the delay is approximately `ZTD / sin(e)` (a mapping function; `1/sin(e)` used today, GMF/Niell are better below ~15°).  `solve_ppp.py` carries a `dZTD` residual state (`IDX_ZTD`) in both PPPFilter (`N_BASE=7`) and FixedPosFilter that absorbs wet-delay variation above a Saastamoinen a-priori dry model; without this state, wet-delay drift leaks into altitude because ZTD and height are ~95% correlated geometrically.  See `docs/ztd-state-for-ppp-ar.md`. |
| **TDEV** | Time Deviation.  Stability measure in time units (ns).  TDEV at tau=1s is the primary short-term metric. |
| **ADEV** | Allan Deviation.  Stability measure in fractional frequency units.  Related to TDEV by `TDEV(tau) = tau * ADEV(tau) / sqrt(3)`. |
| **pull-in** | Initial convergence phase where the servo acquires lock from a large phase offset. |
| **glide slope** | Smooth frequency ramp applied during bootstrap to converge the DO's phase offset without overshooting. |
| **loop bandwidth** | The frequency at which the servo's gain crosses unity.  Determines the crossover between DO noise (below) and reference noise (above). |
| **noise floor** | The minimum achievable TDEV of an oscillator or measurement system. |

## Hardware

| Term | Definition |
|---|---|
| **F9T** | u-blox ZED-F9T.  Multi-band GNSS timing receiver with L1/L5 support. |
| **i226** | Intel I226-LM/V.  2.5G Ethernet NIC with PTP hardware clock (PHC).  Used on TimeHAT v5. |
| **E810** | Intel E810-XXVDA4T.  100G Ethernet NIC with PTP hardware clock, DPLL, GNSS input.  Used on ocxo host. |
| **TimeHAT** | Time-Appliances-Project Raspberry Pi HAT with i226 NIC and F9T receiver. |
| **ClockMatrix** | Renesas 8A34002.  Programmable clock generator with DPLL and TDC.  Used on Timebeat OTC boards. |
| **DPLL** | Digital Phase-Locked Loop.  On ClockMatrix: locks an output frequency to an input reference. |
