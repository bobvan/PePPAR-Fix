# Architecture Vision — AntPosEst + DOFreqEst

*2026-04-09 design discussion, updated 2026-04-15 with implementation
status and refined framing.  This document describes both where we are
and where we're going.*

## Naming principle

Name things for their **value**, not their mechanism.  There are EKFs
inside these components, but "filter" describes how they work, not what
they deliver.  The right names describe what value each component adds
to the system.

## Why position matters for time

GNSS receiver manufacturers solve general PVT problems.  PePPAR-Fix
is not a general PVT customer.  We take Velocity = 0 as a given.  We
need Position only as a means to an end.  **The end is Time.**

We need an approximate position fix to even begin tracking time
differences between GPS and the DO.  But we also need a precise
position fix to reduce time phase errors — every centimeter of
position error is ~33 ps of clock error.  This isn't a nice-to-have:

**Hard requirement: position agreement < 10 cm (< 1 ns time phase).**

Two clocks on the same ARP, or two sequential runs on the same ARP,
must converge to the same position within 10 cm.  At 3 ns/m, a 10 cm
position disagreement is 300 ps of GPS time phase uncertainty — already
a significant fraction of the error budget.  At 1 m disagreement
(typical float PPP after 10 minutes), the GPS time phase is uncertain
by 3 ns, which dominates every other error source in the system.

This is why AR is not optional for the production path.  Float PPP
converges to ~8 cm σ in 10 minutes but the *actual* position drifts
by meters over hours (tropospheric, ionospheric, solid earth tides).
Only integer-fixed ambiguities pin the position to the cm level and
hold it there across time.

PePPAR-Fix clocks must all agree on (1) the length of a second and
(2) when the second starts.  AR pushes the position to cm level,
which pushes the time-of-second-boundary error below 100 ps.

This motivates the two-estimator architecture: one that answers
"where is the antenna?" (position, long timescale, AR) and one that
answers "how fast is the DO drifting?" (frequency, short timescale,
no AR needed).

## The two estimators

### AntPosEst — Antenna Position Estimator

**Value**: "Where is GPS time?" — refines the antenna phase center
position, our phase reference, pushing GPS time phase bias toward zero.

**Timescale**: minutes to hours.  Works over long timescales using
carrier-phase observations and corrections (float PPP → PPP-AR) to
converge the antenna position from meters to centimeters.

**Independence**: runs completely independently of any DO, PHC, or
servo.  Just needs observations and corrections.  This is the
`--no-do` mode: peppar-fix reduces to pure AntPosEst.

**Implementation**: the same `PPPFilter` class that currently runs in
Phase 1 (position bootstrap), kept alive into Phase 2 and fed
decimated observations from a background thread.  No new filter code.
Extended with AR when phase biases are available (see
`docs/ppp-ar-design.md`).

**Output**: ECEF position that gradually migrates into DOFreqEst's
phase reference via exponential blending.

### DOFreqEst — Disciplined Oscillator Frequency Estimator

**Value**: "How fast is the DO drifting from GPS time?" — tracks the
DO's frequency offset from GPS, given a fixed or slowly-changing
phase reference (position) over short timescales.

**Timescale**: sub-second to minutes.  Works epoch-by-epoch at 1 Hz,
producing a frequency correction (`adjfine_ppb`) applied to the DO.

**Components** (internal, not separate peers):

- **EKF** (currently `FixedPosFilter`): estimates `dt_rx` at each
  epoch given the position from AntPosEst.  Uses time-differenced
  carrier phase — no ambiguity states, no AR needed.
- **FrequencyIntegrator** (currently `CarrierPhaseTracker`):
  integrates adjfine corrections into a running phase estimate,
  providing continuous phase knowledge between PPS edges.
- **Measurement fusion** (currently `compute_error_sources` source
  competition): combines PPS edges, qErr refinement, TICC
  measurements, and the frequency integrator's prediction into a
  single phase/frequency estimate each epoch.
- **PI servo**: converts the phase/frequency estimate into
  `adjfine_ppb` applied to the DO.
- **InBandNoiseEstimator**: continuously estimates the DO's noise
  floor from discipline gaps (see below).

**Output**: `adjfine_ppb` written to the DO's PHC via `clock_adjtime`.

## How the two estimators interact

```
AntPosEst                          DOFreqEst
(position, long timescale)         (frequency, short timescale)

  observations ──┐                 ┌─────────────────────────┐
  corrections  ──┤                 │  EKF (dt_rx)            │
  AR module    ──┘                 │  PPS + qErr measurement │
       │                           │  FrequencyIntegrator    │
       │  position ──(gradual)──▶  │  Measurement fusion     │
       │                           │  PI servo → adjfine     │
  NAV2-PVT ──(consensus)────────▶ │  Consensus watchdog     │
                                   │  InBandNoiseEstimator   │
                                   └──────────┬──────────────┘
                                              │
                                              ▼
                                       DO frequency output
```

Position flows one-way from AntPosEst to DOFreqEst.  The consensus
watchdog uses both estimators plus the F9T secondary engine (NAV2-PVT)
to detect antenna movement or filter corruption (see
`docs/future-work.md` "Three-source position consensus").

## Measurement fusion replaces source competition

The current `compute_error_sources()` builds a sorted list of error
sources (PPS, PPS+qErr, Carrier, PPS+PPP, TICC) and picks the winner
by lowest `confidence_ns`.  This is a stepping stone.

The sources aren't actually competing for the same thing:

- **PPS, PPS+qErr, TICC** are **epoch measurements** — snapshots of
  the DO's current phase offset from GPS.  Independent noise each
  epoch.
- **Carrier (FrequencyIntegrator)** is a **prediction** — "given
  where we started and every adjfine correction since, the DO should
  be at this phase."  Smooth, but accumulates systematic errors.

These are the **predict** and **update** steps of a single estimator,
not competitors.  The FrequencyIntegrator predicts; the PPS/qErr/TICC
measurements update.  The source competition's winner-take-all
evolves into measurement fusion.

### DOFreqEst: the fusion filter (target architecture)

**The fundamental insight** (2026-04-10): satellite clocks couple to
our DO *through* the receiver's TCXO.  PPP carrier phase gives ~0.1 ns
tracking of the TCXO-to-GPS relationship.  PPS edges (timestamped by
the TICC on the PHC clock) give the TCXO-to-PHC relationship.
Together, they triangulate the PHC-to-GPS relationship.

**The measurement model subtlety** (also 2026-04-10, learned the hard
way through three failed fusion attempts):

The raw TICC measurement *does* observe (δ_phc − δ_tcxo_quantized):
TICC chA (PEROUT, from PHC) minus chB (PPS, from TCXO).  But qErr
correction removes the TCXO quantization noise, leaving approximately
φ_phc — a direct observation of the PHC phase error.  After qErr
correction, the TCXO coupling in the measurement is gone.

This means a naive 4-state filter with:
```
z_ticc_corrected = φ_phc + noise       (H = [0, 0, 1, 0])
z_ppp            = φ_tcxo + noise       (H = [1, 0, 0, 0])
```
decomposes into two independent 2-state filters.  The PPP measurement
constrains φ_tcxo but *can't help* φ_phc.  The states are decoupled.

**The real coupling lives in the raw TICC measurement** (before qErr):
```
z_ticc_raw = φ_phc − φ_tcxo_quantized + noise_ticc
```
where φ_tcxo_quantized = φ_tcxo rounded to the nearest 8 ns tick.
The quantization is a nonlinear operation (modular arithmetic).
Combined with:
```
z_ppp = φ_tcxo + noise_ppp
```
these two measurements *together* constrain φ_phc better than either
alone — PPP resolves the quantization ambiguity, giving an effective
qErr correction at PPP precision (~0.1 ns) rather than TIM-TP
precision (~1 ps reported but limited by TCXO noise).

This requires an EKF (or particle filter) because the 125 MHz tick
quantization model is nonlinear.  It's analogous to PPP-AR: the
continuous carrier phase (dt_rx) provides the float solution, and
the PPS tick model provides the integer constraint.

**Implementation path:**

1. **Current (working)**: 2-state Kalman with TICC+qErr as sole
   measurement.  TDEV(1s) = 0.799 ns (best, 2026-04-11), routinely
   ~2.0 ns on 15-min runs (2026-04-13).  TICC-driven servo verified
   on three hosts (TimeHat, MadHat, pi4ptpmon) with PEROUT alignment
   reliable via detect-and-retry loop.  qVIR = 28–176× across hosts.
   The qErr correction is done externally (TIM-TP from the F9T),
   and the residual ~0.178 ns measurement noise is the TICC+qErr
   floor.

2. **Next step**: EKF that models the 125 MHz tick quantization
   explicitly.  Raw TICC + PPP dt_rx as joint measurements.  The
   filter performs qErr resolution internally using PPP precision
   instead of relying on TIM-TP.  This should push the effective
   measurement noise below 0.178 ns toward the PPP floor (~0.1 ns).

3. **Full DOFreqEst**: add the FrequencyIntegrator (adjfine history)
   as a process model enhancement, enabling the filter to predict
   between PPS edges using the known adjfine corrections.

### What we tried and why it failed (2026-04-10)

Three fusion attempts, three failures, each illuminating:

1. **Absolute carrier fusion** (commit 33c3b03): injected Carrier
   prediction (from CarrierPhaseTracker) as a second phase measurement.
   Failed because the Carrier had +24 ns systematic bias from
   unconverged drift rate D.  With carrier_sigma=0.1 ns, the Kalman
   trusted the biased Carrier 30× more than the truthful TICC.

2. **Time-differenced dt_rx** (commit e6e15e9): injected Δdt_rx as a
   frequency measurement to avoid the absolute bias.  Failed because
   the 2-state filter's frequency state = (f_phc − f_tcxo) — it
   conflates the two oscillators.  Δdt_rx observes f_tcxo alone, so
   injecting it as a measurement of the combined state pushes f_phc
   toward zero, stalling convergence.

3. **Naive 4-state filter** (commit d4725f0): separated the oscillator
   states but used the wrong measurement model (H_ticc = [-1,0,1,0]
   for TICC+qErr, which treats the corrected TICC as still coupled).
   Also set δ_tcxo = dt_rx (millions of ns) in the initialization,
   producing insane LQR corrections.  When corrected to H = [0,0,1,0],
   the filter decomposes into two independent 2-state filters — PPP
   can't help the PHC because there's no coupling in the measurements.

**The lesson**: fusion requires coupling in the measurement model.
The coupling exists in the raw TICC (before qErr), not in the
corrected TICC.  The raw-TICC EKF with explicit tick quantization
is the correct architecture.

## The GPS-to-DO chain (2026-04-12)

The system's job is to transfer GPS time phase to the DO.  Every
link in the chain is a measurement of one oscillator against another.
The chain runs master-to-slave:

```
GPS satellite clocks           (the master reference)
        │
        │  carrier-phase observations (RAWX)
        │  → PPP filter estimates dt_rx
        │  → PPP-AR fixes ambiguities for lower noise
        ▼
rx TCXO (F9T's 125 MHz crystal)
        │
        │  Two independent measurements of rx TCXO vs GPS:
        │
        │  1. dt_rx from PPP (carrier phase, ~0.1 ns, 30-60 min
        │     convergence, atmospheric corrections required)
        │
        │  2. qErr sequence from TIM-TP (beat note of rx TCXO
        │     against GPS, ~178 ps per sample, immediate,
        │     no atmospheric dependency)
        │
        │  Both measure the same thing: rx TCXO frequency and
        │  phase relative to GPS time.  They cross-validate.
        │
        │  PPS edge (gnss_pps): the rx TCXO's tick-quantized
        │  representation of the GPS second boundary
        ▼
DO (disciplined oscillator crystal)
        │
        │  Measured by TICC: gnss_pps vs do_pps
        │  → pps_err_ticc_ns = gnss_pps - do_pps
        │  → pps_err_ticc_qerr_ns (with qErr correction)
        │
        │  Also measured by EXTTS: gnss_pps on PHC timescale
        │  → pps_err_extts_ns (8 ns resolution, but on the
        │     timescale we discipline)
        ▼
DO frequency output (adjfine)
```

### The qErr beat note (2026-04-12 insight)

The qErr sequence is the beat note between the rx TCXO's 125 MHz
clock and GPS time, sampled at 1 Hz at each PPS edge.  Each sample
reports where the PPS edge landed within the 8 ns tick grid to
~178 ps precision.

When analyzed as a sequence rather than applied as independent
corrections, the unwrapped qErr reveals:

- **rx TCXO frequency relative to GPS**: the slope of the unwrapped
  phase (~2-3 ns/s, varying with temperature)
- **rx TCXO phase relative to GPS**: the accumulated unwrapped
  phase, precise to 178 ps per sample

This is the same information PPP's dt_rx provides, but from a
completely independent measurement path:

| Property        | dt_rx (PPP)     | qErr sequence        |
|-----------------|-----------------|----------------------|
| Noise/sample    | ~0.1 ns         | ~178 ps              |
| Convergence     | 30-60 min       | Immediate (1st PPS)  |
| Atmospheric?    | Yes             | No — hardware only   |
| Ambiguity?      | Float bias      | None (ticks observed) |
| Update rate     | 1 Hz            | 1 Hz                 |

The qErr-derived frequency tracks the rx TCXO to better than 0.7 ns
over 10-second windows (measured from 8-hour overnight data,
strictly causal linear fit, predicting 1 second ahead).

**Status (2026-04-13)**: qErr is fully operational as independent
per-epoch corrections (qVIR = 28–176×, TIM-TP window matching solved).
Beat note unwrapper implemented (`QErrBeatNote` in engine).

**Key finding**: the unwrapped qErr phase tracks `dt_rx mod 8 ns`, not
`dt_rx` directly.  At ~27 ns/s TCXO drift, the integer tick component
(~3.4 ticks/s) is invisible to qErr — only the sub-tick drift (~2.6 ns/s)
is captured.  Cross-validation compares *rates*: `dt_rx_rate mod 8`
should match `qErr frequency`.  This is a weaker check than absolute
phase tracking, but still catches cycle slips and filter faults.

### How qErr frequency tracking fits the architecture

**Cross-validation**: if the qErr-derived rx TCXO frequency diverges
from PPP's dt_rx rate, one of them has a problem (cycle slip,
atmospheric anomaly, or qErr correlation failure).  This is a
lightweight integrity check that runs at 1 Hz with no additional
computation beyond the unwrap.

**PPP filter seeding**: the qErr-derived frequency can seed the PPP
filter's initial clock rate estimate, accelerating convergence.
Currently the PPP filter starts at dt_rx=0 and takes minutes to
converge.  A qErr-derived seed would give it the right starting
frequency from the first PPS edge.

**Holdover quality**: during NTRIP outages, PPP dt_rx goes stale.
The qErr sequence continues (it depends only on the F9T's internal
clock and GNSS lock, not on external corrections).  The unwrapped
qErr keeps tracking the rx TCXO's frequency drift, providing
a backup frequency reference for the servo.

**PPP-AR synergy**: PPP-AR reduces dt_rx noise from ~0.1 ns to
~0.05 ns (see docs/ppp-ar-design.md).  This tighter dt_rx combined
with the qErr-derived phase gives two sub-100-ps estimates of the
rx TCXO state.  Their agreement or disagreement is a sensitive
indicator of carrier-phase integrity — exactly what PPP-AR needs to
validate its integer ambiguity fixes.

### Graceful degradation

In all fusion architectures, the sigma-inflation mechanism still
works.  When NTRIP corrections go stale:
- PPP dt_rx sigma inflates → its Kalman gain shrinks toward zero
- TICC+qErr (no NTRIP dependency) continues at full weight
- The filter naturally enters holdover: predict step dominates,
  last-known frequency sustains the DO

No winner-take-all.  No explicit mode switch.  Holdover falls out
of the math.

## State machines and seed verification

### AntPosEst states

```
UNSURVEYED  → no seed, no observations.
               DOFreqEst: blocked.  clockClass: 248.

VERIFYING   → seed exists (position.json / known_pos).
               Live LS fix being computed to check seed.
               DOFreqEst: blocked.  clockClass: 248.

VERIFIED    → live LS fix agrees with seed within threshold.
               DOFreqEst: may initialize.  clockClass: 52.

CONVERGING  → float PPP running, position improving.
               DOFreqEst: running.  clockClass: 6 (once locked).

RESOLVED    → AR fixed at cm level (≥ N SVs NL_LONG_FIXED).
               Phase bias < 100 ps.  clockClass: 6.
               Per-SV churn (new fixes, retirements) happens in
               the background without flipping host state.

MOVED       → consensus detected antenna displacement.
               DOFreqEst: holdover.  clockClass: 7.
               AntPosEst: re-enters CONVERGING.
```

Host-level states describe whether the *receiver's position* is
trustworthy.  A separate **per-SV state machine** (`SvAmbState`)
tracks each satellite's ambiguity independently — see
`docs/sv-lifecycle-and-pfr-split.md`.  The two machines are
orthogonal; the host stays RESOLVED while individual SVs cycle
through WL_FIXED → NL_SHORT_FIXED → NL_LONG_FIXED → FLOAT as
sky geometry evolves.

### DOFreqEst states

```
UNINITIALIZED   → waiting for AntPosEst.  clockClass: 248.

PHASE_SETTING   → stepping DO phase to GPS second.  clockClass: 52.

FREQ_VERIFYING  → checking drift file against PPS measurement.
                   clockClass: 52.

TRACKING        → servo running.  clockClass: 6.

HOLDOVER        → no usable measurements.  Coast on last adjfine.
                   clockClass: 7.
```

### Seed verification principle

No estimator trusts its seed.  Seeds are initial conditions that
accelerate convergence.  Verification is always the first step:

- **Position seed** (receiver state, keyed by F9T SEC-UNIQID):
  trusted when sigma < 10 m (from prior PPP convergence or known_pos).
  LS validation skipped for trusted seeds.  If no seed exists or
  sigma is high, fall back to PPP from scratch (UNSURVEYED).
- **Frequency seed** (drift.json): verified by PPS interval
  measurement within threshold (~50 ppb).  If rejected, use the
  measured value.
- **DO characterization seed** (do_characterization.json): verified
  by the InBandNoiseEstimator as discipline gaps accumulate.  If the
  live noise profile diverges significantly (temperature change, aging),
  the live estimate supersedes the file.

## Implementation status (2026-04-15)

### What exists in the code today

The engine (`peppar_fix_engine.py`) has two top-level functions:

- `run_bootstrap()` — AntPosEst in UNSURVEYED/VERIFYING states.
  Uses `PPPFilter` to converge position from scratch.  Runs MW+NL
  for AR when phase biases are available.  Exits when position
  sigma reaches threshold.  **Currently discarded after convergence.**

- `run_steady_state()` — DOFreqEst in TRACKING state.  Uses
  `FixedPosFilter` (time-differenced carrier phase, no ambiguity
  states) to extract dt_rx each epoch.  Feeds DOFreqEst 4-state
  EKF for servo control.

The engine transitions from `run_bootstrap` → `run_steady_state`
once position is established.  The code calls these "Phase 1" and
"Phase 2" — these names should be replaced:

| Old name | Maps to | AntPosEst state |
|----------|---------|-----------------|
| Phase 1 entry | Cold start | UNSURVEYED |
| Phase 1 w/ seed | Warm start | VERIFYING |
| Phase 1 → 2 boundary | Position gate | VERIFIED |
| Phase 2 | Servo running | CONVERGING (AntPosEst should still refine) |

### What's working (validated 2026-04-15)

- **DOFreqEst 4-state EKF**: process model sign fix validated.
  7.7-hour overnight on clkPoC3, TDEV(1s) = 0.294 ns.  The EKF
  fuses raw TICC + PPP dt_rx, resolving the 125 MHz tick quantization
  internally at PPP precision.

- **TICC as a competing error source**: TICC-drive path removed
  (-258 lines).  TICC competes in `compute_error_sources()` alongside
  Carrier, PPS+qErr, PPS.  Wins when available (lowest confidence_ns).

- **Adaptive discipline interval**: scheduler adapts based on drift
  rate vs measurement noise.  DOFreqEst receives actual wall-clock dt.
  Longer intervals reduce actuation noise, letting the OCXO's superior
  short-term stability shine through.

- **CNES SSR for AR**: `ssr_ntrip_conf` supports a separate SSR caster.
  CNES provides phase biases matching our F9T L5 profile: GAL L1C+L5Q
  (formal HIT), GPS L5I (shared carrier with L5Q — works).  WL fixing
  confirmed in Phase 1 (6/14 GAL SVs in 78 epochs).

### What's missing (the gap from vision to code)

1. ~~**AntPosEst as a persistent background thread.**~~  **Done.**
   `AntPosEstThread` keeps the PPPFilter alive after bootstrap (or
   creates a fresh one for warm starts).  Steady-state loop forwards
   every Nth observation.  MW+NL run in the thread.  Position callback
   fires on improvement (exponential blending into DOFreqEst is future
   work).  Cold start reuses the converged PPPFilter; warm start
   initializes from known position.

2. ~~**Named state machine in the engine.**~~  **Done** (commit dca8398).
   `AntPosEst` and `DOFreqEst` state machines in `states.py` with
   structured `[STATE]` transition logging and periodic `[STATUS]`
   summary.  AntPosEstThread drives CONVERGING → RESOLVED transitions
   based on NL fix count.

3. **DOFreqEst doesn't need AR.**  It uses time-differenced carrier
   phase (ambiguities cancel between epochs).  AR belongs exclusively
   in AntPosEst, which tracks absolute carrier-phase ambiguities over
   long timescales.  The benefit to DOFreqEst is indirect: AR-refined
   position reduces the systematic clock bias from position error.

## In-band DO noise estimation

The current model requires a one-time 30-minute `--freerun`
characterization run that produces `do_characterization.json`.  This
is a snapshot at one temperature, one time, one set of conditions.
It goes stale.

**The adaptive discipline interval creates natural measurement
windows.**  When the scheduler extends the discipline interval (because
the servo has converged and the oscillator is stable), epochs between
adjfine writes are genuinely free-running.  During a 10-second
discipline interval, epochs 2–9 see the DO drifting with no correction
applied.  After removing the known linear drift (constant adjfine × dt),
the residual is pure DO phase noise.

The `InBandNoiseEstimator` component of DOFreqEst runs two channels:

- **Gap channel**: phase noise from epochs with no adjfine change
  (pure DO noise — the oscillator's free-running floor).
- **Correction channel**: residual noise from epochs where adjfine
  changed.  The expected phase change is subtracted
  (`residual = measured - expected`, where `expected = Δfreq × Δt`).
  If adjfine() calls are truly free (as measured 2026-04-09:
  `docs/adjfine-noise-characterization-2026-04-09.md`), both channels
  should agree.  Divergence indicates DO stress or write-latency issues.

Both channels compute running ADEV/TDEV at τ = 1, 2, 4, ... seconds.
The gap channel gives the DO's noise floor; the correction channel
validates that the servo's corrections aren't adding hidden noise.

Dynamic uses of the two-channel comparison:
- **Adaptive correction clamping**: reduce max correction if the
  correction channel exceeds the gap channel (DO can't absorb it)
- **DO health monitoring**: aging crystals show increasing correction
  noise before catastrophic failure
- **Per-epoch confidence weighting**: epochs with large corrections
  get lower confidence for downstream consumers (PTP clients, TICC)
- **Correction scheduling**: skip corrections whose noise cost
  exceeds their benefit (Δfreq × Δt < DO noise floor → don't bother)

**Benefits over one-time freerun**:
- Continuously current (tracks temperature-dependent noise changes)
- No lost time (happens during normal operation)
- Better statistics (thousands of gaps per overnight)
- Temperature correlation possible (if board temp is logged)

**What it can't measure**: ADEV at τ longer than the maximum
discipline interval (the servo is steering within that window).
For initial deployment, the one-time freerun characterization still
seeds the noise model — the in-band estimator verifies and refines it.
See `docs/asd-psd-servo-tuning.md` and `docs/freerun-characterization.md`
for the current (pre-in-band) approach.

## `--no-do` mode

AntPosEst runs alone.  No PHC, no servo, no adjfine.  Doesn't need
root.  Produces `position.json` (converged position) and optionally
a convergence log.  Useful for:

- Pre-surveying a deployment site before DO hardware is installed
- Validating the GNSS chain independently of timing hardware
- The outdoor weatherproof unit scenario: run position survey first,
  then start frequency tracking once position has converged

## `--characterize-do` mode (replaces `--freerun`)

There IS a DO, but we deliberately don't steer it.  Measures the
free-running DO noise for initial servo tuning.  This is different
from `--no-do`: it opens the PHC, reads EXTTS, reads PPS, computes
phase error — everything except writing adjfine.  Produces
`do_characterization.json`.

As the in-band noise estimator matures, the initial characterization
window can shrink (from 30 minutes to 5 minutes) because the in-band
estimator will refine the profile from real operational data within
the first hour.

## Wrapper dissolution

The current `scripts/peppar-fix` shell wrapper provides seven pieces
of value.  In the integrated model:

| Wrapper value today | Future home |
|---|---|
| Sequencing (steps 1→2→2b→2c→3) | Internal state machine (AntPosEst / DOFreqEst states) |
| Retry dispatch (exit codes 0/2/3/5) | Internal state transitions (MOVED, HOLDOVER, self-heal) |
| PHC bootstrap (phc_bootstrap.py) | DOFreqEst initialization (PHASE_SETTING, FREQ_VERIFYING) |
| Receiver config | First thing in `main()` or `ExecStartPre` in systemd |
| DO characterization | `--characterize-do` mode or in-band estimation |
| clockClass management | Driven by estimator state transitions, not exit codes |
| Config resolution | Python argparse + TOML loader in `main()` |

The wrapper becomes a one-liner:

```sh
exec "$PYTHON" peppar_fix.py "$@"
```

Systemd's `Restart=on-failure` handles crash recovery.  The supervisor
cron job handles exponential backoff for repeated crashes.  All
intelligence lives in Python where it can be tested and debugged.

## Migration path

No big-bang rewrite.  Each step makes the wrapper simpler and the
engine more self-contained:

1. Merge `phc_bootstrap.py` into the engine as DOFreqEst initialization.
   Wrapper's Step 2b disappears.
2. Keep PPPFilter alive past Phase 1 as AntPosEst background thread.
   Wrapper's Step 2 disappears.
3. Move exit-code retry logic into internal state transitions.
   Wrapper's `while true` loop disappears.
4. Move clockClass management to estimator state transitions.
   Wrapper's `degrade_clock_class`/`promote_clock_class` disappear.
5. Add InBandNoiseEstimator.  DO characterization step shrinks or
   becomes optional.

Each step is independently testable and deployable.

## Naming concordance

| Current name | Future name | Notes |
|---|---|---|
| `PPPFilter` | `AntPosEst` | Antenna Position Estimator |
| `FixedPosFilter` | `DOFreqEst` (internal EKF) | DO Frequency Estimator |
| `CarrierPhaseTracker` | `FrequencyIntegrator` (inside DOFreqEst) | Integrates adjfine into running phase |
| `compute_error_sources()` | `fuse_measurements()` (inside DOFreqEst) | Predict/update fusion, not competition |
| `phc_bootstrap.py` | DOFreqEst `PHASE_SETTING` + `FREQ_VERIFYING` states | Absorbed into engine |
| `--freerun` (for DO characterization) | `--characterize-do` | Distinct from `--no-do` |
| `--freerun` (no PHC at all) | `--no-do` | AntPosEst only |
| `peppar-fix` wrapper | thin launcher or `exec peppar_fix.py` | Intelligence moves to Python |

## Design invariant: filter σ ≠ correctness

An EKF's reported σ measures *self-consistency with its own inputs*,
not *absolute truth*.  A filter can report σ=30 mm on a position
that's 5 m wrong, or σ=0.1 ns on a dt_rx that's 100 ns biased, if
its inputs are biased in a way the filter has already absorbed into
its state estimate.  The 2026-04-16/17 wrong-integer investigation
made this painful: PPPFilter reported σ<0.05 m while AR-fixed
positions on the same shared antenna disagreed by 1–7 m across
three hosts, and in one case Phase 1 converged 323 m from truth
with σ=0.03 m.

**Every EKF state therefore needs an independent cross-check.**
The filter's own confidence gate is necessary but not sufficient —
we need a measurement *external* to the filter's input stream that
a wrong state couldn't satisfy:

- **AntPosEst**: cross-checked against NAV2's secondary code-only
  single-freq fix (different physics, same antenna) plus post-fit
  PR residual trends (PFR monitor).
- **DOFreqEst**: cross-checked against TICC chA−chB differential
  (different measurement chain, independent of the EXTTS/qErr
  stream the filter consumes).  If chA−chB and the filter's state
  disagree over a window, the filter is servoing to a biased
  reference.
- **Cross-host consensus**: identical receivers on a shared antenna
  should agree to sub-cm on position and sub-ns on PPS phase.  Any
  larger disagreement is evidence that at least one host's filter
  has settled on a self-consistent wrong state.

If an EKF has no independent cross-check, treat it as unverified
and prefer exposing the raw measurement to a consumer that does
have one.  The temptation to trust a tight σ is the most common
failure mode — protect against it at the architecture level, not
just by "being careful".

## Cross-references

- **Three-source consensus + self-healing**: `docs/future-work.md`
- **PPP-AR + gradual position migration**: `docs/ppp-ar-design.md`
- **clockClass state mapping**: `docs/ptp4l-supervision.md`
- **Current PHC bootstrap (to be absorbed)**: `docs/phc-bootstrap.md`
- **Current freerun characterization (to be superseded by in-band)**: `docs/freerun-characterization.md`
- **Current source competition (to evolve to fusion)**: `docs/pps-ppp-error-source.md`, `docs/carrier-phase-servo.md`
- **Current data flow (to be updated)**: `docs/full-data-flow.md`
- **Servo noise tuning (impacted by in-band estimation)**: `docs/asd-psd-servo-tuning.md`
- **qErr correlation design (TIM-TP window matching)**: `docs/qerr-correlation.md`
- **PEROUT 500ms hardware bug and fix**: `docs/i226-perout-500ms-bug.md`
- **MadHat EKF overconfidence incident (motivating the consensus design)**: project memory `project_madhat_ekf_overconfidence`
