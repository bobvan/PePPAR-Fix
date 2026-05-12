# Slip Detection & Surgical Handling — unified design for cycle and code slips

**Status**: design, 2026-05-12.
**Motivation**: clkPoC3 day0511night hit 11 catastrophic-reject cascades
in 14 hours. PiFace (same antenna, same SSR stream, F9T receiver) hit
zero. Investigation showed:

1. The existing slip detector **already catches** the per-SV bad-PR
   events — typically as repeating `mw_jump conf=LOW` log lines on
   low-elevation SVs, sometimes for an hour before the cascade fires.
2. The engine **doesn't act** on LOW-confidence slips, so the bad PRs
   accumulate in the EKF until cat-reject escalates to exit-5.
3. The two slip classes are dual problems — cycle slip (carrier phase)
   and code slip (pseudorange) — and deserve unified surgical handling.

PiFace's clean record doesn't mean PiFace will *always* be clean.  Any
receiver can occasionally emit massively-wrong observations (ionospheric
scintillation, multipath, firmware tracking-loop slip, thermal
transient).  The engine should fail-soft for all of them.

## Background: why code slip detection should be easier than cycle slip

|                    | **Cycle slip** (carrier phase ϕ)   | **Code slip** (pseudorange ρ)  |
|--------------------|------------------------------------|-------------------------------|
| Observable type    | Ambiguous — unknown integer N in cycles | Absolute — direct distance in meters |
| Detection method   | Need combinations (GF, MW) to spot integer jumps because ϕ alone has no reference | Predict ρ from prior + range-rate; compare |
| Why this matters   | The whole machinery of LAMBDA / WL / NL resolution exists *because* ϕ is ambiguous | PRs don't need any of that |

Cycle slips are unmarked and indistinguishable — hence the search for N
and the elaborate slip-detection machinery.  PR codes are supposed to be
absolute, so a 3500-meter jump in 1 second should be trivial to spot:
no SV can physically move 3500 m in 1 second.  The current engine catches
these (via the MW combination, which folds in code as well as carrier)
but doesn't act on the LOW-confidence signal.

## Empirical answer from day0511night

For each of the 11 cascades on clkPoC3, slip-log lines from the
suspected SV(s) appeared **before** the cat-reject cascade fired —
often for many minutes:

- **Cascade #11 (06:01)**: E19 emitted mw_jump LOW every second from
  05:00 CDT onward.  61 minutes of persistent LOW-conf slip flags
  before cat-reject escalated.
- **Cascade #6 (22:36)**: E26 mw_jump every second from 22:34; then
  E08 mw_jump every second from 22:35:57.  Within minutes of slip
  onset, cascade fired.
- **Cascade #2 (19:53)**: E15, C38, C26, E13 all chattering at low
  elevation; eventually E15 went HIGH-conf with gf=1275 cm (12.75 m
  geometry-free residual) but the engine had been ingesting bad data
  from those SVs for ~10 minutes.

The SVs are not snapping back to clean within seconds.  They stay
wrong for the duration of their slip event (often minutes), then either
recover when geometry changes (rising/setting) or remain wrong until
muted by an unrelated mechanism (lock loss, arc gap, etc.).

So the F10T isn't telling us SVs are teleporting briefly — it's telling
us a tracking loop has slipped and is reporting a constant N-chip
offset for the duration of the slipped state.  We already detect this;
we just don't act.

## Design: unified surgical slip handling

Replace the binary `30 consecutive rejects → exit 5` with a **graduated
response ladder** that uses the existing slip detector as the first
line of defense.

```
Level 0: per-SV continuity check       NEW    catches single-SV slips at the epoch boundary
Level 1: per-SV persistence escalator  NEW    promotes repeating LOW-conf slips to HIGH after N occurrences
Level 2: per-SV mute on escalation     NEW    flush ambiguity + suspend from EKF for cooldown period
Level 3: mode-classified cat-reject    NEW    distinguishes stationary chip-slip from drifting outlier
Level 4: state-flush                   NEW    reset GNSS state, preserve clock state
Level 5: re-bootstrap (exit-5)         keep   genuinely catastrophic faults
```

### Level 0: per-SV PR continuity check

For each SV at each epoch, predict the expected pseudorange from the
previous valid epoch plus the line-of-sight range rate (from broadcast
ephemeris):

```python
def check_pr_continuity(sv, pr_obs_m, prev_pr_obs_m, dt_s, ephemeris):
    # Range rate from SV velocity dotted into line-of-sight unit vector.
    r_dot = compute_range_rate_m_per_s(sv, ephemeris, our_known_pos)
    expected = prev_pr_obs_m + r_dot * dt_s
    residual = pr_obs_m - expected
    # σ_pr_continuity ≈ 5-10 m for clean tracking; tighten with
    # observed-rate-noise calibration.
    if abs(residual) > 50.0:  # 50m = ~0.17 chips: catches anything chip-slip-sized
        return SlipFlag(sv, kind="pr_continuity", residual_m=residual,
                        conf=infer_conf_from_residual(residual))
    return None
```

10-20 LOC, no MW or GF combinations needed.  Drops a `pr_continuity`
flag on any SV whose PR exceeds Keplerian-plausibility.  Note: a clock
bias common to all SVs cancels in `prev_pr + r_dot*dt` ↔ `pr_obs`
because it's the SAME bias both times.  So this check is robust to
clock-state slips: a receiver clock chip-slip would create N consistent
flags across all SVs, which is itself diagnostic.

### Level 1: per-SV persistence escalator

The existing slip log emits LOW-confidence flags for transient slips.
Currently those are noisy and ignored.  Make them load-bearing:

```python
class SvSlipTracker:
    def __init__(self, escalate_after_n=5, decay_window_s=60):
        self.low_conf_count = {}  # sv → count of consecutive LOW flags
        self.last_flag_t    = {}

    def feed(self, slip_flag):
        sv = slip_flag.sv
        # Reset count if more than decay_window since last flag
        if t_now - self.last_flag_t.get(sv, 0) > self.decay_window_s:
            self.low_conf_count[sv] = 0
        if slip_flag.conf == "LOW":
            self.low_conf_count[sv] = self.low_conf_count.get(sv, 0) + 1
            if self.low_conf_count[sv] >= self.escalate_after_n:
                # Promote to HIGH on persistence
                slip_flag.conf = "HIGH"
                slip_flag.reasons.append("persistence")
        self.last_flag_t[sv] = t_now
        return slip_flag
```

A SV that emits 5 LOW-conf slip flags within 60 seconds is no longer
noise — it's a sustained tracking failure.  Promote to HIGH so the
existing flush-ambiguity-and-mute machinery kicks in.

### Level 2: per-SV mute with cooldown

When a HIGH-conf slip fires (either inherently HIGH or promoted from
LOW), the existing code flushes the ambiguity.  Add: **suspend the SV
from EKF participation for a cooldown period** (e.g. 60 sec).

The SV's re-admission must satisfy the existing tracking re-acquire
criteria PLUS pass the Level 0 continuity check on its first epoch
back.  This prevents the same SV from getting re-admitted immediately
and re-contaminating the EKF.

### Level 3: mode-classified cat-reject

If Levels 0-2 don't catch the failure (e.g. the bad data is the
RECEIVER clock state slipping, not a single SV), cat-reject still
fires.  At that point, classify the mode based on the last 5-10
rejected epochs:

```python
class FailureMode(Enum):
    STATIONARY_CHIPSLIP = ...  # CV<5%, mean is integer chip multiple
    DRIFTING_OUTLIER    = ...  # CV>20%, drift over many epochs
    UNKNOWN             = ...
```

For STATIONARY_CHIPSLIP (5 of 11 cascades in day0511night):
- HOLD the EKF state.
- Do not increment `consecutive rejects`.
- Re-engage when median |PR| drops back below the threshold for ≥5
  clean epochs.
- Timeout: escalate to Level 4 after 120 sec.

For DRIFTING_OUTLIER (6 of 11):
- Should NOT normally reach this level — Levels 0-2 should have
  identified the offending SV(s) and muted them.
- If we DO reach here, the worst-PR-contributor SV (largest
  |residual / σ|) gets muted and the median re-checked.
- Escalate to Level 4 only if outlier exclusion doesn't recover.

### Level 4: state-flush (without restart)

Same as in earlier draft: flush all per-SV ambiguity state, preserve
clock state, preserve actuator, preserve subsystems.  ~20 LOC.  Skips
the 6-second restart overhead.

### Level 5: re-bootstrap (current exit-5 behavior)

Unchanged.  Fires only after Level 4 times out.

## Why this unification matters

The cycle/code dichotomy disappears at the engine level: both are "the
GNSS observable for this SV is anomalous against the model".  The slip
log already records both kinds (mw_jump combines them).  The fix is to
ACT on those records, not to add new detection machinery.

The per-SV continuity check (Level 0) is the only genuinely new
detector.  It exists to catch the case where the existing combinations
fail to flag — e.g., a PR-only slip where the carrier is clean.  In
day0511 data we didn't have a case of "PR slip with clean carrier",
but the check is cheap and defends against unknown unknowns.

## Implementation footprint

| Component | Location | LOC | Notes |
|-----------|----------|-----|-------|
| Level 0 continuity check | `scripts/peppar_fix/ppp_ar.py` or solve_ppp | 30 | Per-SV per-epoch, ephemeris-based |
| Level 1 persistence escalator | `scripts/peppar_fix/slip_detector.py` (or wherever slip flags are emitted) | 40 | Stateful SV tracker |
| Level 2 mute + cooldown | existing SV state machine | 20 | Lean on existing FLOATING/WAITING enum + a new cooldown timer |
| Level 3 mode classifier | `scripts/solve_ppp.py` cat-reject path | 50 | Replay-tested against day0511 data |
| Level 4 state-flush | `scripts/peppar_fix_engine.py` | 30 | Carefully avoid touching actuator/EXTTS |
| Tests (per-level synthetic injectors + replay harness) | scripts/peppar_fix/test_*.py | 200 | Critical for confidence |

Total: ~370 LOC + tests.  Estimate 6-8h of focused work to implement
all five levels.  Could ship in phases: Level 0+1+2 first (the
preventive layer that catches things early), then Level 3+4 as the
graceful-degradation layer.

## Open question for review

For Level 0, what's the right σ threshold for PR-continuity?  Options:
- Fixed threshold (50 m: catches anything > 0.17 chips, well above
  receiver noise floor for clean tracking).
- Adaptive: track per-SV PR-rate residual σ over a rolling window;
  threshold = 5σ.  Catches anomalies relative to that SV's recent
  noise.
- Elevation-weighted: tighter at high elevation, looser at low.

I lean toward fixed initially (simplicity, no tuning needed); promote
to adaptive if false-positive rate is problematic.

## Validation plan

**Lab data needed**: Once the F10T moves to MadHat (per Bob's plan to
swap F10T ↔ F9T-BOT, putting F9T-BOT on clkPoC3 and F10T on MadHat),
MadHat becomes the resilience test rig.

Targets per overnight:
- Existing slip detector firing rate (LOW-conf flags / hour).
- Level 1 promotions (LOW → HIGH via persistence): expect O(10) / night
  if the F10T tracking-loop pattern holds.
- Level 2 mutes: should be close to one mute per cascade in the
  current data.
- Level 3 mode-classified cat-rejects: expect 5/11 → STATIONARY
  cascades that hold-and-recover.
- Level 4 state-flushes: expect 1-2 cascades that need to flush.
- Level 5 re-bootstraps (exit-5): goal is 0 per night.

**Pre-deploy validation**: replay harness fed with day0511 clkPoC3 log
data should drive the engine through the 11 cascades and verify the
new ladder behaves as predicted.  Build that harness first; ship code
only when the replay matches the predicted classification.

**Fall-back**: if F9T-BOT+clkPoC3 produces 0 events overnight (matching
PiFace), we'll have to lean on the replay harness for validation.
That's fine — the failure modes are well-documented in the day0511
data.

## Bonus 10-line win (do this regardless)

In every cat-reject log line, also print the median magnitude in
chips and milliseconds:

```python
chips = median_pr_m / 293.05
ms    = median_pr_m / 299792.458
log.error("[CATASTROPHIC_REJECT] median |PR|=%.1fm (%.3f L1 chips, %.3f ms) ...",
          median_pr_m, chips, ms, ...)
```

Future debugging spots the chip-domain magnitude pattern at a glance.
