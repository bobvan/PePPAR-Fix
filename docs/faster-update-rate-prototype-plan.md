# Faster update-rate prototype plan (2/5/10 Hz)

**Item:** `I-fasterUpdateRate-main` (dayplan 2026-06-15).
**Goal:** run the measurement+servo loop faster than 1 Hz to suppress
free-running DO noise above the ~1 Hz loop bandwidth — the binding limiter
for the OCXO mid-τ discipline-noise bump, and the whole ballgame for
TCXO-class hosts. Only carrier-phase (TDCP) works above 1 Hz: `UBX-TIM-TP`
qErr is 1 Hz firmware-only (and `qErrInvalid` on the X20P), so the
prototype is a TDCP-arm servo at higher RAWX rate.

**1 Hz baseline to beat** (clkPoC3 CAS run, clean post-warmup window):
τ32 = 281 ps, τ64 = 659 ps. That mid-τ rise is the target.

## Key finding: the core math already self-adapts

The EKF/servo cadence is **not** hardcoded — the engine measures the real
inter-epoch interval from GPS-time deltas and passes it through:

```
peppar_fix_engine.py:1652/2874/4986/6144   dt = (gps_time - prev_t).total_seconds()
            → FixedPosFilter.predict(dt) / PPPFilter.predict(dt)
            → DOFreqEst.update(dt=...)   (recomputes F/B per call)
            → PIServo.update(dt=...)     (integral scales by dt)
peppar_fix_engine.py:8879                   servo.update(dt=dt_actual)
tdcp_estimator.py:214                       dt = t_gps_s - prev.t_gps_s
cycle_slip.py:250                           gap_s = t_mono_s - prev.t_mono_s
```

At 5 Hz those deltas become 0.2 s and the filter math, process-noise
scaling, TDCP `df_f`, and slip arc-gaps all self-adapt. **So the prototype
is not a filter rewrite** — it's (a) rate-scaling the epoch-*count*
thresholds whose intent is wall-time, (b) setting the receiver rate, and
(c) the live tuning. The `dt=1.0` values that remain are first-epoch
fallbacks, not the steady-state path.

Also verified **not** a blocker: the "suspicious dt" guards bound the
*upper* side only (`dt <= 0 or dt > 30` / `> 120`), so 0.2 s epochs pass
unflagged.

## Rate-hardcode map (from a full sweep)

**A — true 1 s/1 Hz that would be wrong at rate (all first-epoch fallbacks
or self-adapting via dt):** `solve_ppp.py:538,1433` `dt=1.0` fallback;
`do_freq_est.py:131,513` / `servo.py:20` `dt=1.0` defaults (callers pass
real dt); engine `time.sleep(1)` (bootstrap/holdover, not the epoch-driven
main loop).

**B — per-second COUNT thresholds (wall-time intent, must scale by rate):**
- `solve_ppp.py` `CATASTROPHIC_REJECT_LIMIT=30`, `HISTORY_MAX=10`,
  `HISTORY_MIN=5` — **DONE this PR** (see below).
- `discipline.py:62-77` scheduler `base/min/max_interval` (1 / 1 / 120 s) —
  the adaptive Goldilocks scheduler granularity. **TODO.**
- `peppar_fix_engine.py:2064` `nav2_alarm_count` consecutive-check. **TODO
  (review whether NAV2 itself stays 1 Hz).**

**C — already correct (dt from timestamps):** both `predict(dt)`,
DOFreqEst F/B recompute, `tdcp_estimator` dt, `cycle_slip` gap (mono
deltas), `servo` integral. Time-based windows `ARC_GAP_MAX_S=1.5`,
`ARC_RESTART_GAP_S=30`, TDCP `max_dt_s=1.5` are in **seconds** vs timestamp
deltas → self-adapt; no change.

**D — 1 Hz firmware constraint (cannot change):** `TIM-TP` qErr is 1 Hz
(and `qErrInvalid` on X20P) → TDCP-only above 1 Hz. The phase anchor
(do_pps / TICC chA) also stays 1 Hz; the loop fuses an N Hz TDCP *frequency*
with a 1 Hz *phase* anchor, and the chA TDEV metric stays 1 Hz-sampled.

## Done in this PR

`FixedPosFilter(..., meas_rate_hz=1.0)` — rate-scales the catastrophic
epoch-count thresholds by shadowing the class constants with instance
attrs (`round(intent × rate)`, floored at 1). At `meas_rate_hz=1.0` the
values are byte-for-byte unchanged (30/10/5), so 1 Hz behavior is
identical; at 5 Hz they become 150/50/25 — preserving the ~30 s / 10 s /
5 s wall-time intents. The engine's `getattr(filt,
'CATASTROPHIC_REJECT_LIMIT')` picks up the instance value automatically.
This also matters for [gracefulClkReset](#) — a clkReset wrap that slips
past the realign handler would otherwise hit the 30-epoch limit in 6 s at
5 Hz instead of 30 s. Tests: `test_faster_update_rate.py` (5 cases).

## Remaining for the live 5 Hz run (when clkPoC3 is back — `hw:clkPoC3`)

1. **Wire `meas_rate_hz` end-to-end** — **DONE.** The receiver rate was
   already wired (`--measurement-rate-ms` → `ensure_receiver_ready` →
   `configure_rate`). This PR derives `args.meas_rate_hz = 1000 /
   measurement_rate_ms` and passes it to all three `FixedPosFilter(...)`
   constructions (main servo, re-bootstrap, bootstrap). `--measurement-rate-ms
   200` ⇒ X20 at 5 Hz RAWX + filter thresholds scaled. 1 Hz identical.

2. **`discipline.py` scheduler — DONE (both A + B).** `DisciplineScheduler`
   now takes `meas_rate_hz` and `fire_every_epoch`:
   - **(A) rate-aware adaptive**: `compute_adaptive_interval()` clamps the
     Goldilocks coast in seconds exactly as before, then converts seconds →
     epochs via `× meas_rate_hz` at the final step (1 Hz byte-identical). So
     the adaptive scheduler coasts the same *wall-time* at any rate.
   - **(B) `--fire-every-epoch`**: `should_correct()` returns True every
     epoch, bypassing the coast → loop BW = measurement rate. The prototype
     mode for the max-rate test.

   Running A vs B at 5 Hz gives an *indirect* read on actuator σ_q (the
   gap between coast-optimal and fire-every-epoch chA TDEV is the σ_q cost),
   complementing a direct σ_q measurement. Even if fire-every-epoch isn't
   optimal for today's plants, it may be for a future one.

   _(original note kept for history)_ The blocker was:
   `should_correct()` fires at `len(self._errors) >= interval` (an epoch
   COUNT), but the **adaptive** path (`--adaptive-interval`, which defaults
   ON and has no CLI off-switch — `store_true`+`default=True`) sets
   `interval` from `compute_adaptive_interval()`, a Goldilocks τ in
   **seconds**. At 1 Hz seconds==epochs so it's right; at 5 Hz it fires at
   τ epochs = τ/5 s → **5× over-actuation**, injecting σ_q (the TimeHat
   "1 Hz worse than coasting" failure mode). So the 5 Hz run must NOT start
   until the scheduler converts τ-seconds → epochs via `× meas_rate_hz`
   (and scales `base/min/max_interval`). Pass `meas_rate_hz` into
   `DisciplineScheduler`; multiply the computed interval (and the clamps)
   by the rate; 1 Hz stays identical. **This is the next increment.**
3. **TDEV metric** — chA stays 1 Hz (`rate=1.0`); any *TDCP-rate* series
   plotted from `--tdcp-log` must use `rate=meas_rate_hz`.
4. **Run** (TDCP arm, `--servo-input tdcp`): 5 Hz RAWX on clkPoC3 (Pi 4,
   OCXO), watch (a) CPU — expected trivial now (~325 µs/s parse with the
   vectorized decoders, vs ~255 ms/s GIL if it were still pyubx2), and
   (b) whether the τ32/τ64 mid-τ bump shrinks vs the 281/659 ps baseline.
   Cross-check TimeHat (TCXO).

## First 5 Hz run (2026-06-15) — plumbing works, NEW blocker found

Launched mode B (`--measurement-rate-ms 200 --fire-every-epoch --servo-input
tdcp --no-extint`) on clkPoC3. Confirmed working end-to-end at the receiver:
**X20 reconfigured to 5.0 Hz (`CFG_RATE_MEAS=200 OK`), NAV-CLOCK streaming at
200 ms**, CAS SSR flowing, the scheduler/filter plumbing all engaged.

But the loop **starved — 0 OBS_ADMIT ever.** Root cause: the **obs↔PPS
correlation gate.** The DO-PPS / TICC phase anchor is **1 Hz** (one PPS edge
per second), and the gate matches each obs epoch to a PPS event by
CLOCK_MONOTONIC with an `expected_offset` tuned for **1 Hz** cadence. At 5 Hz
there are 5 obs epochs per PPS (5:1), and the 1 Hz-tuned matching window
admits **none** → the DO bootstrap's "10-epoch" clock filter took 2.5 min and
failed to converge, and the main loop produced no admitted obs. NAV-CLOCK
(serial-reader thread) kept streaming at 5 Hz, confirming the receiver side is
fine; it's the correlation layer.

**This is the next (deeper) blocker** — beyond the scheduler/filter plumbing,
which is correct and necessary. The fix is architectural: **decouple the N Hz
obs/TDCP path from the 1 Hz PPS-correlation requirement.** TDCP frequency
doesn't need a per-epoch PPS match (it's a carrier-phase delta); only the
phase *anchor* needs the 1 Hz PPS. So: let all 5 Hz obs flow to the TDCP arm,
and correlate only the 1 Hz subset (the epochs nearest a PPS edge) for the
TICC/EXTINT phase anchor — rather than gating *all* obs on a PPS match. See
docs/stream-timescale-correlation.md. Until then, the 5 Hz run can't admit
obs; the 1 Hz baseline (τ32=270 ps, τ64=527 ps) stands.

## Risks

- **Actuator σ_q injection** (the TimeHat "1 Hz worse than coasting"
  precedent: adjfine σ_q is ns-class, not the LSB floor). Faster cadence on
  a noisy actuator *adds* noise. clkPoC3's OCXO + AD5693R DAC is finer, so
  more likely to win — but it's empirical, and the win is only real if the
  per-correction actuator noise stays below the DO noise being suppressed.
- The CPU headroom that makes 5 Hz viable on the Pi 4 is *entirely* due to
  the vectorized RAWX+NAV-SIG decoders (#163/#167). On stock pyubx2 this
  prototype would be GIL-bound.
