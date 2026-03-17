# PePPAR Fix Data Flow

Two independent signal paths — the PPP filter and the PHC timestamp —
meet at the servo to produce a carrier-phase disciplined clock.

```
                    ┌─────────────────────────────────────────────────────┐
                    │                   F9T GNSS Receiver                 │
                    │                                                     │
  Roof antenna ────▶│  ┌─────────────┐                                   │
  (L1/L5, E1/E5a)  │  │ RF frontend │                                   │
                    │  └──────┬──────┘                                   │
                    │         │                                           │
                    │         ▼                                           │
                    │  ┌─────────────┐  RXM-RAWX (1 Hz)                 │
                    │  │ Correlator  │──────────────────────────┐        │
                    │  │             │  per-SV: pseudorange,    │        │
                    │  │ L1+L5 code  │  carrier phase, Doppler, │        │
                    │  │ & carrier   │  C/N0, lock time         │        │
                    │  └──────┬──────┘                          │        │
                    │         │                                 │        │
                    │         ▼                                 │        │
                    │  ┌─────────────┐  TIM-TP (1 Hz)          │        │
                    │  │  Timing     │───────────────────────┐  │        │
                    │  │  engine     │  qErr (ps),           │  │        │
                    │  │             │  qErrInvalid flag     │  │        │
                    │  └──────┬──────┘                       │  │        │
                    │         │                              │  │        │
                    │         ▼                              │  │        │
                    │  ┌─────────────┐                       │  │        │
                    │  │  PPS output │── coax ──┐            │  │        │
                    │  │  (1 pulse/s)│          │            │  │        │
                    │  └─────────────┘          │            │  │        │
                    │                           │            │  │        │
                    │  RXM-SFRBX (nav data) ────│────────────│──│──┐     │
                    └───────────────────────────│────────────│──│──│─────┘
                                                │   USB      │  │  │
                                                │   serial   │  │  │
                                                │            ▼  ▼  ▼
                                                │
                ┌───────────────────────────────│──── Host (Pi 5 / x86) ────┐
                │                               │                           │
                │            ┌──────────────────│───────────────────┐       │
                │            │          serial_reader()             │       │
                │            │          (realtime_ppp.py)           │       │
                │            │                  │                   │       │
                │            │    ┌─────────────┼──────────┐       │       │
                │            │    │             │          │       │       │
                │            │    ▼             ▼          ▼       │       │
                │            │  RXM-RAWX    TIM-TP    RXM-SFRBX   │       │
                │            │  obs_queue   QErrStore  BcastEph    │       │
                │            └────┬────────────┬──────────┬────────┘       │
                │                 │            │          │                 │
                │                 │            │          │                 │
                │                 │            │          │                 │
                │   NTRIP ────────│────────────│──────────│───────┐        │
                │   caster        │            │          │       │        │
                │                 │            │          │       ▼        │
                │                 │            │          │  ┌─────────┐   │
                │                 │            │          │  │  NTRIP  │   │
                │                 │            │          │  │ reader  │   │
                │                 │            │          │  └────┬────┘   │
                │                 │            │          │       │        │
                │                 │            │          │  RTCM3 msgs   │
                │                 │            │          │       │        │
                │                 │            │          ▼       ▼        │
                │                 │            │    ┌──────────────────┐   │
                │                 │            │    │  BroadcastEph    │   │
                │                 │            │    │  + SSRState      │   │
                │                 │            │    │  (corrections)   │   │
                │                 │            │    └────────┬─────────┘   │
                │                 │            │             │             │
                │  ═══════════════│════════════│═════════════│════ PPP ══  │
                │  ║              ▼            │             ▼          ║  │
                │  ║  ┌───────────────────────────────────────────┐    ║  │
                │  ║  │          FixedPosFilter (EKF)             │    ║  │
                │  ║  │          (solve_ppp.py)                   │    ║  │
                │  ║  │                                           │    ║  │
                │  ║  │  Inputs:                                  │    ║  │
                │  ║  │    • IF pseudorange per SV (from RAWX)    │    ║  │
                │  ║  │    • IF carrier phase per SV (from RAWX)  │    ║  │
                │  ║  │    • SV positions (from broadcast eph     │    ║  │
                │  ║  │      + SSR orbit corrections)             │    ║  │
                │  ║  │    • SV clock corrections (broadcast      │    ║  │
                │  ║  │      + SSR clock)                         │    ║  │
                │  ║  │    • Code biases (SSR, when available)    │    ║  │
                │  ║  │    • Known receiver position (fixed)      │    ║  │
                │  ║  │                                           │    ║  │
                │  ║  │  State vector:                            │    ║  │
                │  ║  │    [dt_rx, dt_dot, ZWD, N1..Nn, ISBs]    │    ║  │
                │  ║  │                                           │    ║  │
                │  ║  │  Outputs:                                 │    ║  │
                │  ║  │    • dt_rx (receiver clock offset, ns)    │    ║  │
                │  ║  │    • dt_rx_sigma (filter confidence, ns)  │    ║  │
                │  ║  │                                           │    ║  │
                │  ║  │  *** NO PHC data enters the filter ***    │    ║  │
                │  ║  │  The filter knows nothing about the PHC.  │    ║  │
                │  ║  │  It only estimates the F9T's TCXO clock.  │    ║  │
                │  ║  └───────────────────┬───────────────────────┘    ║  │
                │  ║                      │                            ║  │
                │  ║              dt_rx, dt_rx_sigma                   ║  │
                │  ║                      │                            ║  │
                │  ═══════════════════════│════════════════════════════  │
                │                         │                             │
                │                         │                             │
                │   ┌─────────────────┐   │                             │
                │   │  i226 TimeHAT   │   │                             │
                │   │  (/dev/ptp0)    │   │                             │
                │   │                 │   │                             │
  F9T PPS ─────│──▶│  SDP1 (extts)   │   │                             │
  (coax)       │   │   │             │   │                             │
               │   │   ▼             │   │                             │
               │   │  PHC timestamps │   │                             │
               │   │  PPS edge:      │   │                             │
               │   │  (phc_sec,      │   │                             │
               │   │   phc_nsec)     │   │                             │
               │   │                 │   │                             │
               │   │  SDP0 (perout)──│───│──▶ Disciplined PPS OUT      │
               │   │                 │   │    (to TICC chA)            │
               │   └────────┬────────┘   │                             │
               │            │            │                             │
               │        phc_sec,     dt_rx,        QErrStore           │
               │        phc_nsec     dt_rx_sigma   (qerr_ns)          │
               │            │            │             │               │
               │  ══════════│════════════│═════════════│═══ SERVO ═══  │
               │  ║         ▼            ▼             ▼            ║  │
               │  ║  ┌─────────────────────────────────────────┐   ║  │
               │  ║  │     compute_error_sources() (M6)        │   ║  │
               │  ║  │                                         │   ║  │
               │  ║  │  pps_error = pps_fractional(phc_nsec)   │   ║  │
               │  ║  │                                         │   ║  │
               │  ║  │  Three competing error estimates:       │   ║  │
               │  ║  │                                         │   ║  │
               │  ║  │  1. PPS-only:                           │   ║  │
               │  ║  │     error = pps_error     ±20 ns        │   ║  │
               │  ║  │                                         │   ║  │
               │  ║  │  2. PPS + qErr:                         │   ║  │
               │  ║  │     error = pps_error     ±3 ns         │   ║  │
               │  ║  │             + qerr_ns                   │   ║  │
               │  ║  │                                         │   ║  │
               │  ║  │  3. Carrier-phase:                      │   ║  │
               │  ║  │     error = pps_error     ±0.1 ns       │   ║  │
               │  ║  │             + dt_rx                      │   ║  │
               │  ║  │                                         │   ║  │
               │  ║  │  Winner: lowest confidence (best)       │   ║  │
               │  ║  └──────────────────┬──────────────────────┘   ║  │
               │  ║                     │                          ║  │
               │  ║            best.error_ns,                      ║  │
               │  ║            best.confidence_ns                  ║  │
               │  ║                     │                          ║  │
               │  ║                     ▼                          ║  │
               │  ║  ┌─────────────────────────────────────────┐   ║  │
               │  ║  │     DisciplineScheduler (M7)            │   ║  │
               │  ║  │                                         │   ║  │
               │  ║  │  Accumulates N error samples.           │   ║  │
               │  ║  │  Computes simple mean.                  │   ║  │
               │  ║  │  Adapts N from TCXO drift rate.         │   ║  │
               │  ║  │                                         │   ║  │
               │  ║  │  Every epoch: accumulate(best)          │   ║  │
               │  ║  │  Every N epochs: flush → avg_error      │   ║  │
               │  ║  │  Between: coast (no adjfine)            │   ║  │
               │  ║  └──────────────────┬──────────────────────┘   ║  │
               │  ║                     │                          ║  │
               │  ║            avg_error (every N epochs)          ║  │
               │  ║                     │                          ║  │
               │  ║                     ▼                          ║  │
               │  ║  ┌─────────────────────────────────────────┐   ║  │
               │  ║  │     PIServo (gain-scaled)               │   ║  │
               │  ║  │                                         │   ║  │
               │  ║  │  kp, ki scaled by confidence            │   ║  │
               │  ║  │  integral scaled by dt (= N seconds)    │   ║  │
               │  ║  │                                         │   ║  │
               │  ║  │  output: adjfine_ppb                    │   ║  │
               │  ║  └──────────────────┬──────────────────────┘   ║  │
               │  ║                     │                          ║  │
               │  ║                     ▼                          ║  │
               │  ║           ptp.adjfine(ppb)                     ║  │
               │  ║              │                                 ║  │
               │  ║              ▼                                 ║  │
               │  ║     i226 PHC frequency adjusted                ║  │
               │  ║     (steers TCXO timestamp rate)               ║  │
               │  ║                                                ║  │
               │  ══════════════════════════════════════════════════  │
               │                                                     │
               └─────────────────────────────────────────────────────┘


Signal quality at each stage:

  Satellite → F9T correlator:  carrier phase ±2mm (0.007 ns)
  PPP filter dt_rx estimate:   ±0.13 ns (σ from covariance)
  F9T PPS edge timing:         ±3 ns (qErr quantization)
  PPS through SDP1 timestamp:  ±15-30 ns (SDP path jitter)  ◄── BOTTLENECK
  Carrier error (pps + dt_rx): ±6 ns std (SDP jitter dominates)
  After M7 averaging (10 ep):  ±1.9 ns std
  adjfine resolution:          ~0.015 ppb (1/65536 ppm)


Key insight: the PPP filter and the PHC are independent.
The filter estimates WHERE GPS time is (via carrier phase).
The PHC timestamps WHEN the PPS arrives (via SDP hardware).
The servo combines both to compute HOW FAR the PHC is from GPS time.
```
