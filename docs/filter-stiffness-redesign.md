# Filter stiffness redesign — NAV2 seed, physics-tight priors, unified EKF

*Design doc, 2026-04-30.  Authors: bravo (architectural rethink),
main (empirical motivation + review).  Pending implementation by
charlie.  Refs: dayplan I-024532-charlie/bravo, I-133648-main.*

## Motivation

Today's PPPFilter routes uncertainty by data accumulation, not by
physical envelope.  This is the upstream cause of weeks of
downstream symptoms — NL admission cycling, ZTD doom loops,
integrity trips, SecondOpinionPosMonitor firing.  Last night's
overnight on cc23840 (TimeHat + MadHat, day0429eve* logs) is the
empirical case:

- **36 of 73 integrity trips** were `ztd_impossible` (16) or
  `ztd_cycling` (20) — the ZTD-loose-prior failure mode dominates
  fleet-wide, same shape on both hosts.
- **TimeHat dropped 22 m of altitude in 10 seconds** after a
  `SecondOpinionPosMonitor` reset to NAV2 LLA.  This is the textbook
  example of `scrub_for_retry`'s 100 m P_pos blowup colliding with
  systematic single-freq ionospheric bias and walking the filter
  back into the wrong basin within seconds.
- **MadHat held alt = 200.5 m + σ = 0.019 m all night on the same
  code**, but only because it tracks 22 dual-frequency SVs vs
  TimeHat's 15 (TIM 2.25 vs TIM 2.20 firmware).  IF combination
  cancels iono first-order on MadHat — masking the architectural
  problem.  Strip the dual-freq advantage and TimeHat's failure
  mode is what every weak-antenna deployment will hit.

The filter must constrain bias-sink states (position, ZTD) by
physics rather than waiting for observations to grow tight enough
to overcome a wrong prior.

## Diagnosis — two stiffness knobs, both miscalibrated

EKFs gate state changes through two distinct mechanisms:

| Knob | Role | Equation | Failure if too loose |
|---|---|---|---|
| **P** (state covariance) | one-shot resistance per measurement | `K = P·Hᵀ / (H·P·Hᵀ + R)` | single biased observation moves state by σ instead of mm |
| **Q** (process noise) | per-tick drift budget between predicts | `P_{k+1\|k} = F·P·Fᵀ + Q` | state can drift indefinitely with no observations contradicting |

Today's PPPFilter (`scripts/solve_ppp.py:335`) initializes:

```python
self.P = np.diag([
    100.0**2, 100.0**2, 100.0**2,   # position σ = 100 m
    1e8,                              # clock σ ≈ 10⁴ m  (~33 µs)
    1e6,                              # ISB GAL σ ≈ 10³ m
    1e6,                              # ISB BDS σ ≈ 10³ m
    0.5**2,                           # ZTD residual σ = 500 mm
])
```

Two states are fundamentally over-loose:

- **σ_position = 100 m** at cold-start, even when the F9T's NAV2
  module already has an SPP fix at hAcc ≈ 1 m.  We are throwing away
  hardware-grade information that is sitting in `UBX-NAV-PVT` every
  epoch.

- **σ_ZTD residual = 500 mm**.  Physical bound at sea level is
  ≤ 200 mm (cf. Saastamoinen + healthy weather variability).  The
  residual ZTD has nowhere physical to go beyond this.  Q_ZTD is
  similarly loose — sustained pseudorange-multipath can walk the
  state past ±700 mm in 5 minutes (the day0429 weak-antenna doom
  loop is exactly this).

When SSR phase-bias coverage has gaps (CNES has GPS L5Q + BDS
B2a-I gaps; per `docs/l5i-l5q-phase-bias-empirical.md` the
L5I/L5Q gap shows SD = 1.46 m + systematic mean = −0.73 m), the
**systematic-across-SVs component must land in states that
contribute jointly to every observation** — position, clock,
ZTD.  Today's loose P + Q on ZTD makes ZTD the sink in Phase 2;
loose P_pos at cold-start makes position the sink in Phase 1.
Both are wrong.  Float ambiguities are the natural sink for
per-SV systematic bias and have integer structure that catches
abuse.

PiFace data (2026-04-29) provides the cleanest A/B:

| Configuration | Cold-start outcome |
|---|---|
| `gps,gal,bds` (PB gaps in CNES) | 13–15 m vs NAV2, 3-retry abort |
| `gal` only (full PB coverage)   | converged 35 s, nav2_h = 2.5 m |

Same antenna, same epoch, same code — the only delta is whether
the SSR product covers the tracked signals.  The 13–15 m bias
is the systematic-PB component routing into position/ZTD instead
of staying in the (inherently bias-tolerant) ambiguity space.

## Proposed architecture

Five sub-changes.  Phased landing per the consensus call: A
(sub-changes 1+2+3) → B (sub-change 4) → C (sub-change 5,
deferred).

### Sub-change 1 — NAV2-seeded filter init

At startup, wait up to ~30 s for the F9T's NAV2 module to report
`fixType = 3` and `hAcc < 5 m`.  Use NAV2 LLA as `seed_ecef`.
Initialize:

```python
self.P[0:3, 0:3] = np.eye(3) * (nav2.hAcc_m ** 2)
Q_pos_per_axis = 1e-9   # m²/s, ~mm/day drift budget
```

Position is **stiff but not anchored** from epoch 0.  Q_pos = 10⁻⁹
m²/s gives sub-mm/day drift between predicts — physically
appropriate for a bolted-down lab antenna.  The initial P[0:3,0:3]
admits NAV2's hAcc as the actual uncertainty; observations tighten
from there (see "convergence walkthrough" below).

### Sub-change 2 — physics-tight ZTD prior

```python
ztd_residual_init = 0.0           # delta from Saastamoinen
sigma_ztd_init    = 0.20          # m (was 0.50)
Q_ztd_per_min     = (0.01) ** 2   # ~1 cm²/min random walk
```

σ_ZTD shrinks from 500 mm → 200 mm initial.  Q_ZTD ≈ 1 cm²/min
matches the documented worst-case healthy-weather variability
(strong frontal passage produces ~2 mm/min of ZTD swing; 1 cm/min
is the random-walk ceiling that lets that breathe while still
catching pathological multipath drift).  Float ambiguities absorb
systematic δ_PB instead of ZTD doing it.

**Inflation hook**: on `ztd_impossible` or `SO_POS` trip, multiply
Q_ZTD by 5× for 5 min, then exponentially decay back to baseline.
Keeps the prior tight day-to-day, lets it breathe under verified
disturbance.

### Sub-change 3 — collapse Phase 1 / Phase 2 dichotomy

Today's bootstrap is two filters:

- **Phase 1**: `PPPFilter` with σ_pos = 100 m, runs LS init →
  `W1` (residual-consistency) + `W2` (NAV2 horizontal cross-check)
  bootstrap gates → `CONVERGED` transition.
- **Phase 2**: `FixedPosFilter` (clock-only) takes the converged
  position as `known_ecef`, plus an `AntPosEstThread` running its
  own continuous `PPPFilter` for AR.

The Phase 1 / Phase 2 dichotomy made sense when the cold-start
problem was *finding* a position from scratch.  With NAV2 as the
seed, there is no cold-start position-finding step — the EKF starts
at hardware-grade position and tightens monotonically.  The
ceremony of "deciding when Phase 1 has graduated" disappears.

What changes:

| Today | After |
|---|---|
| Phase 1 LS → W1+W2 → CONVERGED | EKF from epoch 0 |
| Phase 2 loads saved → trust-skip | AntPosEst always-on |
| W1/W2 fire only at CONVERGED boundary | W1+W2 run continuously |
| `AntPosEstState` SURVEYING→…→ANCHORED | Same lifecycle, gated on σ + NL_LONG counts (not Phase boundary) |
| 100 m P-blowup on integrity trip | Sub-change 4 graduated NAV2-pull |

What stays:

- `AntPosEstState` lifecycle (SURVEYING → VERIFYING → CONVERGING →
  ANCHORING → ANCHORED) — useful semantics independent of Phase
  ceremony.
- `WlPhaseAdmissionGate` / `GfStepMonitor` / `IfStepMonitor` /
  `NlAdmissionTier` / `SecondOpinionPosMonitor` — all continuous
  monitors, all keep working.
- The receiver-state file at `state/receivers/<uid>.json` — kept
  for **identity** (UID, TCXO/qErr characterization, slot mapping).
  Position read is **retired**.  Position is always live from
  NAV2 + observations.

W1 becomes a continuous phase-residual-vs-σ_pos consistency
monitor with sustained-N-epoch trip logic similar to
`SecondOpinionPosMonitor`.  W2's NAV2-horizontal cross-check is
already what `SecondOpinionPosMonitor` does; the activation gate
moves from "≥ CONVERGED" to "NAV2 fixType = 3 + epoch ≥ 30 s".

### Sub-change 4 — replace `scrub_for_retry`'s 100 m P-blowup

`scripts/peppar_fix/bootstrap_gate.py:scrub_for_retry` overwrites
P_pos to 100 m² to escape locked-in wrong state.  This is what made
TimeHat walk 22 m in 10 s last night: blowing P open removes
all stiffness, observations re-bias the filter into the same wrong
basin within seconds.

Replacement: pull position to current NAV2 LLA with σ = NAV2.hAcc.
Same escape semantics, no 22 m re-walk, preserves the rest of the
filter's earned state (clock, ZTD, ISBs, ambiguities — touch
position only).

For `ztd_impossible` recovery: pull ZTD residual to the
Saastamoinen estimate (i.e. residual = 0) with σ_ZTD = 200 mm
(matches the new initial prior).

### Sub-change 5 — graduated discontinuity response (DEFERRED)

A single Q_pos can't serve both stationary (Q ≈ 0) and step (Q
large for one tick) regimes.  Real bumps, earthquakes, or
sled-mount shifts on non-penetrating roof installations need an
event-driven response.

**Primary detector**: direction-projected innovation correlation.
Per epoch, compute the EKF innovation `(z − Hx̂)` *pre-update* per
SV, project onto north / east / up unit vectors via each SV's
direction cosines, take a windowed mean per direction, trigger
when `|projected innovation| > k·σ_pos` sustained N epochs.

**Why pre-update**: a real 30 cm horizontal step produces
direction-correlated innovations.  Per-SV float ambiguities would
silently absorb the step *post*-update — RMS-based W1 would
miss it entirely.  Reading innovation pre-update bypasses the
ambiguity-absorption path.

**Elevation-weighted disambiguator**: ZTD residual ∝ 1/sin(elev)
(low-elev large); vertical step ∝ sin(elev) (high-elev large).
Opposite weighting separates ZTD drift from vertical antenna step.

**Cross-validation**: real step → NAV2 follows → nav2Δ stays
bounded.  Wrong-integer drift → nav2Δ widens → SO_POS handles
separately.

**Response — escalating only if the prior step doesn't recover**:

| Step | P_pos action | Use case |
|---|---|---|
| 1 | += (50 cm)²  | bump / minor earthquake |
| 2 | += (5 m)²    | mount partially failed |
| 3 | pull to NAV2, σ = NAV2.hAcc | cold-start-like |

Replaces `scrub_for_retry`'s 100 m blowup AND wholesale
`fix_set_integrity` re-bootstrap with one graduated path.  Recovers
in seconds for the realistic ≤ 10 cm step case while preserving
sub-cm precision for the no-step case.

Deferred until P1 + P2 prove out, since its design assumes Q_pos
is tight (which only becomes true after sub-change 1).

## Convergence walkthrough — why Q_pos ≈ 0 doesn't trap us at NAV2's 1 m floor

The reasonable skeptical question: if Q_pos = 10⁻⁹ m²/s and the
NAV2 seed is at hAcc ≈ 1 m, doesn't the filter get stuck at 1 m?

**Q sets the asymptote, observations set the convergence rate.
Two separate things.**

The Kalman update shrinks P by ~ −σ_obs²/N per epoch when P is
large, regardless of Q.  Q just adds +Q on each predict.  With
Q = 10⁻⁹ m²/s and 20 SVs of PR at σ ≈ 1 m, observation-driven
shrinkage (~5×10⁻² m²/epoch) beats Q growth (10⁻⁹ m²/epoch) by
seven orders of magnitude.  Q is irrelevant to the descent; it
sets where the descent stops.

Convergence stages from a 1 m NAV2 seed:

1. **0–30 s — PR-dominated.**  Float carrier ambiguities absorb
   per-SV residuals freely; phase contributes weakly.  PR info
   rate ≈ N_SV / σ_PR² ≈ 20 m⁻²/s.  P_pos drops from 1 m² to
   ~30 cm² in seconds.  The receiver-clock seed (initial σ ≈ µs
   or wider) gets dragged in the same step — clock σ drops from
   ms-scale to ns-scale.  This is where the "but won't a 1 ns
   timing error survive into Phase 2?" worry gets eaten.

2. **30 s – 5 min — float-PPP.**  PR done; carrier phase
   contributes at float-ambiguity precision.  Float ambiguities
   tighten (σ_N: ~10 cycles → ~0.3 cycles).  Position σ → ~5 cm.
   Clock σ → hundreds of ps.  ZTD pulls toward the Saastamoinen
   prior via the mapping function.

3. **5–30 min — integer fixing.**  LAMBDA / rounding fixes
   integers once σ_N < ~0.10 cycles + the ratio test passes.
   **Integer-fixed carrier phase is the magic.**  Information
   rate jumps ~10⁴×: pulling absolute range from sub-mm-precision
   phase, not meter-precision PR.  Position σ → mm-cm.  This is
   the mechanism that beats NAV2's 1 m floor.  NAV2 is
   structurally PR-only SPP, capped at PR-precision; PPP-AR
   breaks that cap.

4. **Steady state.**  P_pos asymptotes at Q × effective_τ ≈
   (70 µm)² — far below the measurement-noise floor and antenna
   phase-center uncertainty.  Q is never the limiter.

## Code sites

| File | Site | Change |
|---|---|---|
| `scripts/solve_ppp.py` | `:335` (`PPPFilter.initialize`, P diag) | tighten σ_ZTD to 200 mm; position σ becomes a parameter passed in by caller |
| `scripts/solve_ppp.py` | Q matrix in `_predict` | add Q_pos = 1e-9 per axis; Q_ZTD with inflation hook |
| `scripts/peppar_fix_engine.py` | `~:1140` (LS init) | replace LS bootstrap with NAV2-wait + NAV2 seed |
| `scripts/peppar_fix_engine.py` | Phase 2 entry | retire — PPPFilter starts as the only filter |
| `scripts/peppar_fix/bootstrap_gate.py` | `scrub_for_retry:150` | replace 100 m P-blowup with NAV2-pull (sub-change 4) |
| `scripts/peppar_fix/state_machines.py` | (no change) | `AntPosEstState` lifecycle stays |
| **NEW** `scripts/peppar_fix/w1_continuous_monitor.py` | new module | continuous phase-residual consistency monitor |
| `scripts/peppar_fix/second_opinion_pos_monitor.py` | activation gate | activate at NAV2 fixType=3 + epoch ≥ 30 s |

Estimated scope: ~250 LOC across 4-5 files for Phase A; ~30 LOC for
Phase B; ~100 LOC for Phase C (deferred).

## What this obsoletes

Many existing per-symptom monitors become belt-and-suspenders
once the upstream stiffness is right:

- The `--sigma` default-tuning conversation (the 0.02 → 1.0 → 3.0
  walk done 2026-04-29).  With NAV2-seed there is no
  convergence-σ gate.
- The Phase-1 σ-inflation save (`bad193f`).  No more Phase-1
  saves at all.
- W1/W2 as one-shot bootstrap gates.  They become continuous
  monitors.

What this does **not** obsolete:

- `WlPhaseAdmissionGate` (phase-residual-consistency check is
  independent of how the filter started)
- `GfStepMonitor` / `IfStepMonitor` (post-fix demoters; load-bearing)
- `NlAdmissionTier` (per-SV trust ladder for NL admission)
- `SecondOpinionPosMonitor` (continuous NAV2 witness with hAcc gate)

## Migration

**Land as main path, no flag-gate.**  Lab is the right place to
take the risk.  The status-quo failure mode (TimeHat 22 m drift) is
documented; the new code's potential failure mode is hypothetical;
`git revert` is the known-good fallback.  A flag-gated parallel
path is dead code we'd retire after one overnight anyway.

Receiver-state file becomes identity-only:

```jsonc
{
  "unique_id": ...,
  "module": ...,
  "tcxo": { ... },         // kept — TCXO characterization
  // "last_known_position": ...   ← retired
  "last_seen": ...
}
```

The position field can stay in the schema for read-back
compatibility but the engine ignores it.

## Validation plan

Tonight's overnight (2026-04-30 → 2026-05-01) deploys Phase A on
TimeHat + MadHat (PiFace as third host if available).  Compares
to last night's cc23840 baseline (day0429eve.log + day0429eve2.log).

Success criteria:

| Metric | cc23840 baseline (last night) | Phase A target |
|---|---|---|
| Integrity trips, 9 h | 39 (MadHat), 34 (TimeHat) | < 10 fleet-wide |
| `ztd_impossible` / `ztd_cycling` share of trips | 36 / 73 = 49 % | < 10 % |
| TimeHat min altitude post-SO_POS reset | 175 m (-26 m below seed) | NAV2 ± hAcc |
| MadHat sustained NL ANCHORED count | NL = 3, max NL = 6 | NL ≥ 4 sustained ≥ 1 h |
| Cold-start time to first NL_ADMIT | ~6 h on TimeHat afternoon | < 30 min |

If Phase A delivers on the trip-rate + TimeHat-altitude metrics,
Phase B (`scrub_for_retry` replacement) can land same-day.  If
either metric regresses, revert and iterate on Q values before
re-deploy.

## Risks and open questions

1. **TEC storms / strong frontal passages** could legitimately
   move ZTD past the new 200 mm initial bound or past the
   1 cm²/min Q ceiling.  Mitigation: Q_ZTD inflation hook on
   integrity trip + the inherently-loosened steady-state P_ZTD
   after a few minutes of running.

2. **Mobile or potentially-disturbed antennas**: Q_pos = 10⁻⁹
   m²/s assumes bolted-down installation.  If UFO1 / Patch3 gets
   physically bumped, the filter resists indefinitely.  Mitigation
   already in plan via sub-change 4 (NAV2-pull on integrity trip)
   and sub-change 5 (graduated discontinuity detector).  Doc-comment
   in `solve_ppp.py`'s Q matrix should warn that Q_pos values
   targeting field deployments may need to scale with expected
   antenna velocity allowance.

3. **NAV2 cold-acquisition time** (typically 30–60 s on clean sky)
   becomes the cold-start gating delay.  Phase 1's LS init also
   waited for `n_sv ≥ 4` with similar timeline, so net startup is
   comparable or faster.

4. **Saastamoinen ZTD prior** requires surveyed altitude.  We have
   it from `timelab/antennas.json` (or fresh from sub-change 1's
   NAV2 seed).  Implementation is small (< 30 LOC); the model is
   already implicit in `solve_ppp`'s tropospheric mapping function.

## References

- Dayplan: `I-024532-charlie/bravo` (proposal); `I-133648-main`
  (priority order + consensus call)
- `docs/l5i-l5q-phase-bias-empirical.md` — empirical SSR PB gap
- `docs/weak-antenna-doom-loop-2026-04-29.md` — Patch4 ZTD doom loop
- `docs/ppp-ar-filter-redesign.md` — earlier filter redesign (WL/NL
  decomposition)
- `docs/ztd-state-for-ppp-ar.md` — ZTD-state motivation
- `docs/ztd-impossibility-trigger-design.md` — bravo's earlier
  ZTD-as-corruption-signal design (already shipped; complementary)
- Memory: `project_gps_l5i_l5q_bias_fix`,
  `project_bds_cnes_capability_20260421`,
  `feedback_filter_slew_rate_limiter` (this work resolves it)
- Literature: BNC, RTKLIB, PRIDE PPP-AR — receiver SPP as seed +
  unified Kalman + physics priors; no Phase-1/2 distinction.
