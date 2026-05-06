# DOFreqEst measurement ladder

> *Compiled 2026-05-06 after the no-gnss-pps experiment surfaced two
> structural truths: (1) the engine's source-competition layer was
> cosmetic — it never wired into actuation; and (2) the qErr-FIFO-vs-
> recv_mono correlation effort was solving the wrong problem.  Both
> findings point at the same redesign, sketched here.*

## TL;DR

Replace the source-competition layer with a four-arm Kalman measurement
ladder fused inside `DOFreqEst`.  Each arm observes a different piece
of the state vector (or a known function of it) and is gated on
availability.  The filter does proper Bayesian fusion across whichever
arms are arriving.  No source picks "wins"; every available
measurement contributes information weighted by its sigma.

```
EKF state vector x:
  x[0] = rx_tcxo phase offset from GPS time   (ns)
  x[1] = rx_tcxo frequency offset from GPS    (parts per billion)
  x[2] = DO PHC phase offset from GPS time    (ns)
  x[3] = DO PHC frequency offset from GPS     (parts per billion)

Measurement arms, each gated on availability:
  z_ppp     :  observes  x[0]                       σ ~ 0.1 ns      RAWX → PPP filter dt_rx
  z_qerr    :  observes  x[1] (slope-of-unwrapped)  σ ~ sub-ppb     TIM-TP qErr stream, time-averaged
  z_extint  :  observes  x[2]                       σ ~ 5–10 ns     TIM-TM2, DO PPS into F9T EXTINT
  z_ticc    :  observes  −x[0]·∂qerr/∂x − x[2]      σ ~ 60 ps       TICC chA−chB diff (transfer standard)

Predict step runs every epoch.  Each measurement arm runs only if
its measurement is available this epoch.  LQR reads x[2], x[3] to
compute adjfine.
```

This document is the design.  The lab wiring (DO PPS → F9T EXTINT)
and host configuration (`CFG-MSGOUT-UBX_TIM_TM2`) are owner-tracked
separately.

## Why this redesign

### What 2026-05-06 surfaced

The `no-gnss-pps` branch tried to test the hypothesis that F9T PPS
edge timing wasn't structurally needed for clock discipline.  We
implemented the hypothesis as a "source-competition lockout":
hardwire the confidence of every PPS-edge-derived source to a huge
value so the Carrier source (PPP `dt_rx`) would always win.  The
experiment failed in two distinct ways, both informative:

1.  **The lockout was cosmetic.**  The actual servo
    (`scripts/peppar_fix_engine.py:6396`) was hardcoded to call
    `servo.update(pps_err_ticc_ns, ...)`.  The `compute_error_sources`
    output (`best.error_ns`) was accumulated in a scheduler used only
    for logging.  Whatever source "won" the competition was a
    log-line label, not a steering signal.  Lockout → milestone-1
    "lock at +31 ns" was actually being driven by TICC chA−chB diff
    the entire time.

2.  **PPP `dt_rx` alone is structurally insufficient as a steering
    input.**  Once we modified DOFreqEst to actually run on `dt_rx`
    only (offset_ns=None at the call site, EKF skips the TICC update
    arm), the DO drifted at TCXO rate while the engine reported
    "locked, err=−2.5 ns".  The `x[2]` and `x[3]` states were
    structurally unobservable — PPP only touches `x[0]`, and the
    F9-class topology has the rx_tcxo and the DO TCXO as physically
    separate crystals.  Without a measurement coupling them, the LQR
    regulates a state it cannot see.

The diagnosis for both failures pointed to the same gap:
**measurement-architecture-driven design, not source-arbitration
design**.  The no-gnss-pps experiment is abandoned in its
lockout-only form.  Its findings shape this design.

### What we keep, what we drop

**Keep:**
- The DOFreqEst 4-state EKF + LQR control structure.  The state
  vector is correct; we just need the right measurement model.
- TIM-TM2 (DO PPS → F9T EXTINT) as the GPS-time anchor.
- RAWX/PPP for rx_tcxo phase observability.
- TICC chA−chB measurement, but as an EKF arm, not as a hardcoded
  servo input.
- TIM-TP qErr stream, but as a direct rx_tcxo state refinement,
  not as an edge-correction matched via FIFO.

**Drop:**
- `compute_error_sources` (`scripts/peppar_fix/error_sources.py`) and
  the scheduler-accumulator-source-competition layer.  Empty cost,
  cosmetic value.
- The qErr-edge correlation pipeline (`QErrStore.consume_next`,
  `match_pps_mono`, qVIR diagnostic).  The new qErr arm doesn't need
  edge matching.
- The hardcoded `servo.update(pps_err_ticc_ns, ...)` and its
  hold-previous-frequency else-branch.

## State vector

```
x[0] = rx_tcxo phase offset from GPS time, ns
x[1] = rx_tcxo frequency offset from GPS rate, ppb
x[2] = PHC phase offset from GPS time, ns
x[3] = PHC frequency offset from GPS rate, ppb
```

Two independent crystals, two phase + frequency states each.  The
F-matrix dynamics include:
- `dx[0]/dt = x[1]` (rx_tcxo phase walks at its frequency offset)
- `dx[2]/dt = x[3]` (PHC phase walks at its frequency offset)

There is no direct dynamical coupling between the two crystals — the
PHC's frequency state evolves independently of the rx_tcxo's.  The
*only* coupling is through measurements that observe both clocks.

Process noise (Q matrix) carries the random-walk uncertainty growth
of each state.  TCXO physics gives reasonable priors:
- Q[0,0]: rx_tcxo phase walk per second from frequency uncertainty
- Q[1,1]: rx_tcxo frequency drift (temperature, aging)
- Q[2,2]: PHC phase walk per second from frequency uncertainty
- Q[3,3]: PHC frequency drift (temperature, aging)

These are tunable from oscillator characterization.  The current
code uses literature-typical TCXO values; an OCXO-grade DO would
use much tighter Q[2,2], Q[3,3].  Tighter Q on x[3] is what enables
sub-ns disciplined output from a 10-ns-class phase measurement —
it lets the filter trust its own frequency-state prediction between
phase measurements.

## The four measurement arms

### Arm 1: z_ppp → x[0]

```
z_ppp = dt_rx_ns        from RAWX → PPP filter (FixedPosFilter)
H_ppp = [1, 0, 0, 0]
R_ppp = dt_rx_sigma_ns² ≈ (0.1 ns)² when PPP is converged
```

**Observes:** rx_tcxo phase offset from GPS time, directly and
linearly.  PPP is the gold-standard rx_tcxo phase observer once
ambiguities are fixed.

**Source:** the time filter's `dt_rx` and its 1-σ uncertainty,
carried through `realtime_ppp.py` to the engine and thence to
`DOFreqEst.update()`.  Already wired today.

**Gating:** if PPP isn't converged (no fixed ambiguities, or σ
exceeds `carrier_max_sigma_ns`), arm is skipped.  Filter relies on
remaining arms + dynamics.

### Arm 2: z_qerr → x[1]

```
z_qerr = slope of unwrapped qErr time series       from TIM-TP
H_qerr = [0, 1, 0, 0]
R_qerr = depends on averaging window and qErr noise
```

**Observes:** rx_tcxo frequency offset from GPS rate.  qErr in the
TIM-TP stream is the rx_tcxo's quantization residual on each PPS
edge, in picoseconds.  Unwrapped over time, it's a direct phase
record of the rx_tcxo against GPS time grid.  The slope of that
unwrapped phase is the rx_tcxo frequency offset.

**Source:** the TIM-TP stream we already ingest.  Drop the FIFO
matching against PPS edges (the qVIR mess) — instead, build a
running unwrapped-qErr buffer and feed its slope as the
measurement.  Each TIM-TP message has a GPS-time timestamp
(towMs+towSubMs) which is what we use for the time axis.  No
correlation with TICC timestamps or recv_mono required.

**Gating:** needs ≥ N samples in the buffer for a meaningful
slope.  Below threshold, arm skipped.

**Why this is robust where the old PPS+qErr was not:** the old
path tried to align a single qErr value with a single PPS edge to
correct the edge's apparent timestamp — that alignment was the
fragile FIFO-matching point.  The new path treats qErr as a
*time-averaged* property of rx_tcxo's frequency.  Drop a few qErr
samples, the slope estimate suffers slightly but doesn't catastrophe.
Get the FIFO matching wrong, no consequence — there's no FIFO.

### Arm 3: z_extint → x[2]

```
z_extint = (DO PPS edge time per F9T's GPS-time solution) − target
H_extint = [0, 0, 1, 0]
R_extint = (TIM-TM2 accEst)² ≈ (5–10 ns)²
```

**Observes:** PHC phase offset from GPS time, directly and linearly.

**Source:** TIM-TM2 message from the F9T after wiring DO PPS to
F9T EXTINT pin.  Each DO PPS edge generates one TIM-TM2 message
carrying `wnR/towMsR/towSubMsR` (rising-edge GPS-time of the edge)
and `accEst` (F9T's own estimate of the timestamp's sigma in ns).

The engine subtracts the intended target (e.g., the integer GPS
second the edge was supposed to land on) and feeds the residual
as `z_extint`.

**Gating:** needs a TIM-TM2 message this epoch.  If no edge
captured (DO PPS missed, or F9T busy), arm skipped.

**Why this changes everything:** without this arm, `x[2]` is
structurally unobservable from PPP alone.  This arm makes it
observable.  Sub-ns disciplined output then becomes achievable
through DO frequency stability + averaging — see "Phase pull-in,
frequency steady-state" below.

### Arm 4: z_ticc → coupling between x[0] and x[2]

```
z_ticc = TICC chA − chB diff, in ns
       = −x[2] − qerr(x[0], tick_ns) + measurement noise
H_ticc varies (nonlinear due to qerr modulo arithmetic)
R_ticc ≈ (60 ps + filter uncertainty in qerr prediction)²
```

**Observes:** the difference between PHC PPS edge and rx_tcxo PPS
edge.  In our filter's terms, this couples `x[0]` and `x[2]` via the
nonlinear qerr() modulo function.

**Source:** TICC chA = DO PPS, TICC chB = F9T PPS.  Already wired
on TimeHat / MadHat / PiFace today.  The measurement is the
chA−chB time difference per TICC reading.

**Crucial: no qErr edge matching required.**  The qerr() function
inside the measurement model uses the *filter's* state estimate of
rx_tcxo phase (`x[0]`) to predict the quantization on the chA−chB
diff.  External qErr stream is not used here.  This is what makes
TICC robust to FIFO-mismatch issues — the filter's internal
rx_tcxo model handles it.

**Gating:** needs valid chA + chB pair this epoch.  Without chB
(F9T PPS unplugged or F9T not generating PPS), arm skipped.

**Why we keep TICC even when TIM-TM2 is available:** TIM-TM2 has
~5–10 ns sigma; TICC chA−chB has ~60 ps sigma.  When both are
present, TICC dominates the information contribution to `x[2]`
estimation.  TIM-TM2 plays a supporting role: cross-check, recovery
during TICC dropouts, observability when F9T PPS isn't wired to
TICC chB.  The two are *complementary* in the EKF formulation;
running both costs nothing and information adds up.

### Sequential update order

Sequential Kalman updates aren't strictly commutative when state
correlations exist across measurements.  Recommended order, from
most-decorrelated to most-coupled:

1.  **z_ppp** — linear, observes x[0] alone.  Tightens x[0] state.
2.  **z_qerr** — linear, observes x[1] alone.  Tightens rx_tcxo
    frequency.  Together with z_ppp, the rx_tcxo side of the state
    space is fully constrained.
3.  **z_extint** — linear, observes x[2] alone.  Tightens DO phase.
    Independent of rx_tcxo arms.
4.  **z_ticc** — nonlinear, couples x[0] and x[2].  Refines both
    after PPP and EXTINT have nailed down their respective states.
    Running TICC last ensures the qerr() linearization happens at
    the most recent x[0] estimate.

Today's DOFreqEst code already orders PPP-then-TICC.  Adding qErr
between PPP and TICC, and EXTINT between qErr and TICC, follows the
same logic.

## Why this is not source competition

**Source competition (old):** every estimator produces an
estimate of the *same* thing (the PHC's offset from GPS time).
Engine picks one to drive actuation.  Mutually exclusive
contributors.  Cost: cosmetic; benefit: robust handover when one
estimator degrades.  Failure mode: hidden pseudo-redundancy because
all "sources" ultimately fed off the same qErr-corrected PPS chain.

**Bayesian measurement fusion (new):** every measurement observes
a *different* function of the state vector.  All measurements
contribute simultaneously, each weighted by its sigma.  The state
vector is the single source of truth; measurements are independent
information channels into it.  Cost: structural; benefit: optimal
information use under known-noise assumptions.  Failure mode: a
measurement with the wrong noise model contaminates the state
estimate proportionally.

The old code had four "sources" all observing the same PHC offset
because there was no explicit state-space; the picking-best-source
heuristic was a poor man's Bayesian fusion.  Putting the EKF state
explicitly in front of the measurement layer makes the fusion
proper, and the measurement design becomes about *what each
measurement physically observes* rather than *which source we
trust most right now*.

## Phase pull-in, frequency steady-state

The Kalman filter does the phase-then-frequency lock cadence
automatically via covariance evolution.  Initial `P[2,2]` and
`P[3,3]` are large (filter is uncertain about DO phase and
frequency) → measurement updates have high gain → aggressive
phase pull-in.  Steady-state `P[2,2]` and `P[3,3]` shrink →
measurement gain drops → process model and frequency-state
prediction take over → effectively narrow-bandwidth frequency lock.

This is structurally identical to the standard GPSDO design
pattern (PLL acquisition → frequency lock steady-state) but
emerges from the Kalman dynamics without explicit state-machine
gain scheduling.

If we want to make this more aggressive (faster pull-in or
tighter steady-state), the LQR `L` matrix can be made
sigma-conditional:

```
σ(x[2]) > 10 ns:  L weights phase regulation high (pull-in mode)
σ(x[2]) < 10 ns:  L weights frequency regulation high (track mode)
```

Today's `L` is constant in `do_freq_est.py`.  Conditional
scheduling is a one-day extension if we observe the
auto-emerging behavior is too slow.

## Ablation experiment plan

The four-arm structure makes selective ablation natural.  Each
arm has a kwarg in `DOFreqEst.update()`; passing `None`
disables that arm.  Engine takes per-arm enable flags from CLI
or config.  TICC chA on the TICC #4 chB-watching-otcBob1 setup
gives a per-host external truth metric independent of the loop
under test.

Ablation matrix per host (PiFace as the candidate; same
hardware works for any of the F9T hosts):

| Run | z_ppp | z_qerr | z_extint | z_ticc | Expected outcome |
|---|---|---|---|---|---|
| A | ✓ | ✗ | ✗ | ✗ | x[2] unobservable; engine drifts at TCXO rate (yesterday's failure) |
| B | ✓ | ✓ | ✗ | ✗ | rx_tcxo refined; x[2] still unobservable; same drift |
| C | ✓ | ✗ | ✓ | ✗ | x[2] observable to TIM-TM2 floor (~5-10 ns); steady-state via DO stability + averaging |
| D | ✓ | ✗ | ✗ | ✓ | x[2] observable through TICC coupling; sub-ns achievable |
| E | ✓ | ✓ | ✓ | ✗ | C plus tightened rx_tcxo; cleaner if F9T nav noise is rx_tcxo-dominated |
| F | ✓ | ✓ | ✗ | ✓ | D plus tightened rx_tcxo; isolates qErr's contribution to x[1] |
| G | ✓ | ✗ | ✓ | ✓ | TIM-TM2 + TICC redundancy; expected best phase steady-state |
| H | ✓ | ✓ | ✓ | ✓ | full ladder; reference performance |

Per-run characterization: TICC #4 chA−chB log against otcBob1 PPS
truth, TDEV at τ = 1, 10, 100, 1000 s; mean and std of the
differential over a steady-state window.

Outcome of interest: which arms are *load-bearing* and which are
*luxury*.  If H ≈ G ≈ E within measurement noise, qErr is luxury.
If H ≪ G, the qErr arm is contributing.  If A and C agree on the
pull-in shape but diverge in steady state, frequency lock is
working.  Etc.

## Implementation outline

### `scripts/peppar_fix/do_freq_est.py`

Refactor `DOFreqEst.update()` signature:

```python
def update(self, *,
           dt=1.0,
           dt_rx_ns=None, dt_rx_sigma_ns=None,         # arm 1: PPP
           qerr_freq_ppb=None, qerr_freq_sigma_ppb=None,  # arm 2: qErr-as-frequency
           extint_phase_ns=None, extint_sigma_ns=None,   # arm 3: TIM-TM2
           ticc_diff_ns=None, ticc_sigma_ns=None):       # arm 4: TICC chA−chB
```

Conditional sequential updates inside, as outlined above.  The
existing PPP and TICC update arms become two of the four; new
qErr and EXTINT arms are linear updates on x[1] and x[2]
respectively.

Keep the `_need_phc_seed` shortcut for the existing TICC-bootstrap
path so warm-boot doesn't lose its first-epoch alignment.  The
seed becomes optional rather than required.

### `scripts/peppar_fix/qerr_freq_estimator.py` (new)

Maintain a rolling buffer of (timestamp, unwrapped qErr) pairs.
Compute frequency offset as the slope of a linear regression over
the last N samples.  Report ppb and sigma_ppb.  Emit measurement
when buffer is full enough.  No FIFO matching against PPS edges.

### `scripts/peppar_fix/extint_reader.py` (new)

Subscribe to UBX-TIM-TM2 messages.  Parse rising-edge fields
(towMs + towSubMs + accEst).  Compute residual from intended
GPS-second target.  Emit measurement.

### `scripts/peppar_fix_engine.py`

At the servo call site (currently lines ~6396-6420), replace the
hardcoded `servo.update(pps_err_ticc_ns, ...)` with the four-arm
keyword call:

```python
adjfine_ppb = -servo.update(
    dt=dt_actual,
    dt_rx_ns=dt_rx_ns, dt_rx_sigma_ns=dt_rx_sigma,
    qerr_freq_ppb=qerr_freq, qerr_freq_sigma_ppb=qerr_freq_sigma,
    extint_phase_ns=extint_phase, extint_sigma_ns=extint_sigma,
    ticc_diff_ns=pps_err_ticc_ns, ticc_sigma_ns=ticc_sigma,
)
```

Drop the hold-previous-frequency else-branch entirely.  The EKF
runs every flush; missing arms are skipped, predict step always
runs.

Per-arm enable flags (`--no-ppp`, `--no-qerr`, `--no-extint`,
`--no-ticc`) for ablation.

### `scripts/peppar_fix/error_sources.py`

Delete.  No replacement needed.  The Kalman filter inside
`DOFreqEst` is the new "source competition", and it doesn't pick —
it fuses.  All callers (the source-change logging in the engine,
the scheduler accumulator for log-line `avg_error` reporting) get
removed alongside.

### Hardware

DO PPS → F9T EXTINT pin.  See `timelab/topology.md` (PiFace section
to be updated).  Current TICC #4 wiring (chA = PiFace DO, chB =
otcBob1 PPS) stays as the external-truth observer for ablation
runs — independent of the loop under test.

## Open questions

1.  **TIM-TM2 accEst empirics.**  We've assumed 5–10 ns sigma based
    on F9T documentation.  Real numbers from the lab will be the
    deciding factor for sub-ns achievability.  First-light
    measurement once PiFace is wired.

2.  **qErr-as-frequency window.**  How many samples to average?
    Too short → noisy slope estimate.  Too long → can't track
    real rx_tcxo frequency drift (temperature changes over
    minutes).  TCXO characterization data + lab measurement
    should land this number.

3.  **Q-matrix tuning for OCXO vs TCXO DOs.**  PiFace runs i226
    TCXO; clkPoC3 will eventually have an OCXO.  Q[2,2] and Q[3,3]
    should be much tighter on the OCXO host.  Per-host Q via host
    config seems clean.

4.  **TIM-TM2 EXTINT PPS conflict.**  PiFace's DO PPS currently
    drives TICC chA *and* would drive F9T EXTINT.  Need a buffer
    or fanout to avoid load issues.  Hardware detail.

5.  **Edge polarity.**  TIM-TM2 reports both rising and falling
    edge times.  Configure `CFG-TM-RISING_EDGE` (or whichever
    matches our DO PPS polarity) to capture only the relevant
    edge.

## Migration / rollout

Phased to limit lab disruption:

1.  **PiFace as proof-of-concept.**  Wire TIM-TM2.  Implement and
    deploy the four-arm DOFreqEst.  Run the ablation matrix.
    Branch lives off `no-gnss-pps` (renamed to something more
    accurate, e.g. `measurement-ladder`).

2.  **Cross-host validation.**  Once PiFace ablation produces
    expected ordering (H beats E beats C beats A), deploy on
    MadHat and TimeHat.  Multi-host bake confirms no regression
    against today's TICC-driven discipline.

3.  **Merge to main; remove old paths.**  Delete
    `compute_error_sources`, the qErr-FIFO matching code, the
    PPS+qErr / PPS+PPP source classes, the source-change logging,
    the scheduler `best.error_ns` accumulator.  Probably ~500
    lines deleted.

4.  **F10T (clkPoC3) port.**  Same architecture, different
    chipset.  TIM-TM2 should be cleaner on F10T.  Validation
    metric: sub-ns disciplined output achievable.

## Failure-mode handling

If `extint_phase_ns` arrives with `accEst > 100 ns` (F9T's nav
engine is degraded — antenna disconnect, ionospheric event), the
arm is gated off this epoch by feeding the EKF a sigma so large
the update has near-zero gain.  Same shape as the staleness-
inflation logic that `compute_error_sources` already does for
PPP/Carrier under stale corrections.

If TICC chA−chB pair is missing (chB cable pulled, F9T PPS
gone), arm 4 is gated off.  Arms 1–3 carry the load.  Filter
behavior is the no-TICC ablation case.

If PPP fails (no fixed ambiguities), arm 1 is gated off.
rx_tcxo state goes unobservable; arms 2 (qErr) help via the
frequency state but absolute rx_tcxo phase walks.  This degrades
arm 4's qerr(x[0]) prediction accuracy → arm 4 is effectively
de-rated.  Arm 3 (TIM-TM2) carries x[2] alone, so DO discipline
continues at TIM-TM2 noise floor.  Position fix may be lost
entirely; that's a separate problem.

## References

- `docs/no-gnss-pps-experiment.md` — the experiment that surfaced
  the cosmetic-source-competition finding.
- `docs/clock-state-modeling.md` — earlier framing of where time-
  domain knowledge enters our filters.
- `docs/td-cp-clock-design.md` — TD-CP clock estimation; the
  position-side analog of the "z_qerr → x[1]" arm pattern.
- `docs/qerr-correlation.md` — the qErr-FIFO-matching design
  this redesign supersedes.
- `scripts/peppar_fix/do_freq_est.py` — the EKF + LQR being
  refactored.
- u-blox UBX-TIM-TM2 message specification (F9 HPG 1.51 + F10 SPG
  6.00 interface descriptions).
