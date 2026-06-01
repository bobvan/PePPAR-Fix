# Arch A — slow innovation-mean integrator (plan)

**Status**: implementation plan, ready for review.  Drafted
2026-06-01 in response to mid-tau TDEV blow-out empirically
attributed to chi² gate locking out load-bearing phase
observations (`docs/two-site-sync-budget.md §3.2.1`).

The architecture extracts the long-term mean of each measurement
arm's innovation — including epochs the fast EKF path rejects —
and applies a slow per-arm correction along the same direction
the fast Kalman gain would.  It is **complementary** to the
soft-gate work (R-inflation on the same arms); they can land in
either order but together they close `σ_servo_residual` back
toward the 10-20 ps theoretical estimate.

---

## 1. What it does, in one paragraph

Each arm in `DOFreqEst.update()` produces a per-epoch
innovation `innov_arm` and a per-epoch Kalman gain `K_arm`
(both already logged by PR #118 as `innov_arm` + `would_pull_arm_xN`).
The existing `InnovControlMonitor` (`scripts/peppar_fix/innov_control_monitor.py`)
already maintains a 300-sample rolling window of `(innov_arm,
S_arm)` pairs and reports `mean(innov_arm / √S_arm)` as a health
metric.  Today the monitor is observer-only: it logs alarms and
does not affect state.  Arch A closes the loop:

```
slow_bias_arm = mean(innov_arm)  over the monitor's window
slow_pull_arm = β · K_arm · slow_bias_arm
self.x       += slow_pull_arm        (component-wise on x[0..3])
```

Applied every `N_slow` epochs (`N_slow = window_size` by default →
once per 300 s).  β is a per-arm gain tuned to keep the slow
loop's bandwidth well below the fast loop's, so the two don't
fight.

---

## 2. Open decisions resolved (from clkPoC3 2026-06-01 data)

The empirical analysis was Bob's morning request before drafting
this plan.  Three decisions resolved cleanly from the 8.6-hour
clkPoC3 capture (steady-state window 14:30–17:50 UTC, 2547 rows):

### 2.1 Per-arm, not fused

Three independent findings forced this:

1. **Arm-specific biases differ by 100×.**  Steady-state mean
   innovations:

   | arm | mean(innov) | comment |
   |---|---|---|
   | ppp | −0.08 ns | calibrated rx chain — nothing to correct |
   | qerr | +1.38 (mixed units) | low |
   | tdcp | +0.01 ppb | clean |
   | **extint** | **+9.17 ns** (σ = 2.71, n = 2504; SNR = 57) | persistent |
   | ticc | +0.09 ns | near-zero |

   A fused single bias signal would either pick one arm
   (per-arm-with-extra-logic) or wash extint's 9-ns signal into
   the near-zero average across other arms.

2. **Each arm touches a different state.**  Mean `|would_pull_arm_xN|`:

   | arm | pull_x0 | pull_x1 | pull_x2 | pull_x3 |
   |---|---|---|---|---|
   | ppp | **10.0 ns** | 0.02 ppb | 0 | 0 |
   | qerr | ~0 | **0.04 ppb** | 0 | 0 |
   | tdcp | ~0 | **0.06 ppb** | 0 | 0 |
   | extint | 0 | 0 | **9.7 ps** | 9e-5 ppb |
   | ticc | 0 | 0 | **0.10 ns** | 9e-4 ppb |

   K vectors are essentially one-hot per arm.  Reusing each
   arm's K (already logged) routes the slow correction to the
   correct state automatically.

3. **Arm innovations are largely independent.**  Pearson r across
   co-fired epochs: most pairs < 0.1; only same-state pairs show
   weak common-mode (extint↔ticc r=−0.29, tdcp↔qerr r=−0.30,
   tdcp↔ppp r=−0.34).  Per-arm integrators don't double-correct
   meaningfully.

**Decision: one slow integrator per arm, reusing the arm's fast-path
K vector for state mapping.**

### 2.2 Which states to correct

Same direction as the fast Kalman gain:

```
slow_correction_arm[i] = β_arm · K_arm[i] · slow_bias_arm    for i in 0..3
```

K_arm is recomputed every epoch by the EKF and already logged.
This handles state mapping by construction.  No magic constants,
no hand-coded routing.

### 2.3 What window of innov to average

The existing `InnovControlMonitor.window` (default 300 samples
@ 1 Hz = 5 minutes) is the natural choice.  300 s is:

- Comfortably longer than the loop's response time (~1-10 s)
- Comfortably shorter than DO drift coherence (>1000 s for OCXO)
- Long enough to average out high-σ outliers (per-sample σ around
  the mean is 2-3 ns; window mean σ drops to ~150 ps at SNR 57)
- Short enough that a real systematic bias is caught within minutes

If empirical tuning suggests a different window per arm, the
monitor already accepts a `window` kwarg.  We can pass arm-specific
windows from `DOFreqEst.__init__` if needed; expect to leave at 300
in v1.

---

## 3. Code-site map

### 3.1 `scripts/peppar_fix/do_freq_est.py`

Three changes to `DOFreqEst`:

1. **One `InnovControlMonitor` per arm**, replacing the current
   single monitor.  Today the class instantiates a single
   `self.innov_monitor = InnovControlMonitor()` at line ~214.
   Replace with `self.innov_monitors = {arm: InnovControlMonitor()
   for arm in ['ppp', 'qerr', 'tdcp', 'extint', 'pseudo', 'ticc']}`.

2. **Feed each monitor from its arm's `_kalman_linear_update` call
   and the TICC inline path**.  The data needed (`u, innov, S`) is
   already computed.  Today the existing single monitor is fed
   only from the TICC arm; extend so each arm feeds its monitor.

3. **New `_apply_slow_corrections(self, dt)` method.**  Called
   once per `update()` AFTER the fast measurement updates.
   Iterates each arm's monitor:

   ```python
   def _apply_slow_corrections(self, dt):
       for arm_name, mon in self.innov_monitors.items():
           rpt = mon.evaluate()
           if rpt.status != "OK":
               continue   # warming up, or alarm raised — fast path only
           bias = rpt.mean_innov_arm   # NEW field — see §3.2
           K    = self.last_arm_K.get(arm_name)
           if K is None or bias is None:
               continue
           beta = self.beta_per_arm[arm_name]
           # rate-limit: slow correction never exceeds the
           # fast-path's typical per-epoch magnitude
           dx = np.array(K) * beta * bias
           dx = np.clip(dx, -self.slow_step_clamp, self.slow_step_clamp)
           self.x = self.x + dx
           self.last_slow_pull[arm_name] = dx.tolist()
   ```

   Called from `update()` after the per-arm measurement-update
   block, before the actuator computation (so the corrected state
   feeds the LQR's adjfine).

### 3.2 `scripts/peppar_fix/innov_control_monitor.py`

Extend `HealthReport` to expose `mean_innov` directly (today it
exposes `norm_bias = mean(innov/√S)` but the slow integrator
needs `mean(innov)` to scale with K, not with √S).

Add one field to `HealthReport`:

```python
@dataclass
class HealthReport:
    epochs:        int
    corr_u_innov:  float
    norm_bias:     float
    nis:           float
    mean_innov:    float   # NEW — for slow-integrator feedback
    status:        str
    consec_bad:    int
```

And one line in `evaluate()`:

```python
mean_innov = float(np.mean(innovs))
```

(plus thread through to `HealthReport(...)`)

This stays observer-friendly; existing callers ignoring `mean_innov`
remain compatible.

### 3.3 `scripts/peppar_fix_engine.py`

Schema v4 columns alongside the v3 `would_pull_X_xN` columns from
PR #118.  For each arm, four new state-dimension columns:

```
slow_pull_X_x0, slow_pull_X_x1, slow_pull_X_x2, slow_pull_X_x3
```

× 6 arms = 24 new columns.  Schema goes 62 → 86 cols, ~0.4 KB/row at
1 Hz → ~33 MB/day extra per host.  Manageable.

Plus one summary column per arm: `slow_bias_X` (the mean_innov fed
in this epoch) for diagnostic visibility.

Row-emit pattern mirrors PR #118's `_fmt_pull`.

---

## 4. Failure modes + defenses

### 4.1 Cascade state — slow correction tries to jump the EKF

During a chi² cascade (yesterday's 12:55 disturbance, today's
cold-start), `mean(innov_arm)` over the monitor's 300-sample
window can be huge — easily microseconds.  Applying `β · K · 5 µs`
unmodified would inject a massive slow correction.

**Defense: hard clamp.**  Add `slow_step_clamp_ns = 1.0` and
`slow_step_clamp_ppb = 0.5` as `__init__` kwargs.  Clamp `dx`
component-wise after the K · β · bias multiplication.  The slow
integrator is a smoothing term; the fast path handles big steps.

### 4.2 Plant-model bias amplification

If `dac_ppb_per_code` is mis-scaled, the existing fast path's
innovation carries a bias proportional to control input
(`InnovControlMonitor.corr_u_innov` measures this).  A slow
integrator on that biased innov would chase the plant-model error
into the state.

**Defense: gate on monitor status.**  Only apply slow correction
when `rpt.status == "OK"`.  When the monitor itself raises
`ALARM_corr` (plant-model error detected), the slow path is
silently disabled until the alarm clears.  This explicitly puts
plant-model error in the diagnostic stack instead of trying to
servo through it.

### 4.3 Warm-up

Each monitor needs `warm_min` samples (default `window // 2` = 150)
before any metric is computed.  During warm-up `status = "WARM"`
and the slow path skips that arm.  Per-arm warm-up is independent —
fast-firing arms warm up first.

### 4.4 Engine restart

The monitors are in-memory only.  On restart, all monitors start
in `WARM` state and accumulate fresh innovations.  The slow path
is silently inactive for the first 300 s post-restart.  Acceptable
because the cold-start regime is dominated by the fast path's
transient anyway.  Not worth persisting monitor state across
restarts (the data ages fast).

### 4.5 Per-arm enable flag for ablation

`__init__` kwarg `slow_arms = ['extint', 'ticc']` (default empty
= off).  Lets us start with just one or two arms and validate.
clkPoC3 data suggests starting with `extint` only — it's the only
arm with a statistically robust steady-state bias.

---

## 5. β tuning protocol

### 5.1 Why β matters

β sets the **slow loop's bandwidth**.  At β = 1, the integrator
would apply the full averaged bias each evaluation cycle — too
aggressive; oscillation likely.  At β = 0, no correction.  Right
value depends on:

- The fast loop's natural bandwidth (closes faster than 1 / N_slow)
- The per-arm bias's stability (extint's bias is statistically
  robust over 5-minute windows; ticc's is below detection)
- The cost of over-correcting vs under-correcting

### 5.2 Starting values from the data

For extint on clkPoC3, the 300-sample bias mean is +9.17 ns with
σ_about_mean = 2.71/√300 = 0.16 ns.  The mean is statistically
robust (z-score = 57).  Apply β small enough that one window's
correction is well below the bias signal: β = 0.1 means apply
~10% of the bias per window, i.e., 0.92 ns/300s = 3 ps/s slow
drift in the state.  The fast path's typical adj is ~20 ppb;
3 ps/s = 3 ppb steady-state contribution from the slow path.
Small relative to fast.

**Proposed starting β values:**

| arm | β | reasoning |
|---|---|---|
| ppp | 0 | no detectable bias; nothing to gain |
| qerr | 0 | wait until data shows steady bias |
| tdcp | 0 | wait until data shows steady bias |
| **extint** | **0.1** | strong bias signal; conservative start |
| pseudo | 0 | not used during normal ops |
| **ticc** | **0.05** | low bias but high impact arm; very conservative |

### 5.3 Validation gate

Re-run a 2-hour overnight on clkPoC3 with extint+ticc slow
integrator on; pull-attribution schema captures both `would_pull`
and `slow_pull` per arm.  Pass criteria:

- **TDEV(τ=100s) drops** from 3.5 ns (current with cascades) or
  ~500 ps (steady-state only) toward PiFace's 607 ps or below.
- **Cross-host CDF p95 (PiFace ↔ clkPoC3)** improves from today's
  13.1 ns.
- **No new failure modes**: no exit-5 cascades introduced by slow
  correction; `InnovControlMonitor` doesn't raise `ALARM_corr`.

If validation passes, raise β for ticc; consider extending to
qerr if steady bias has accumulated.

### 5.4 Per-host β

Different DOs have different intrinsic bias signatures.  PiFace
already runs near the budget at τ=100s; its β values may be
smaller (less work for the slow path to do).  MadHat just had the
F9P swap (new EXTINT chain, new bias) and needs its own
characterization run.

Store β_per_arm in `state/dos/<do_label>.json` alongside the
existing `timestampers` bias entries (which are static
calibrations against external truth — different role from β,
but adjacent to it).

Schema in dos.json:

```json
{
  "do_label": "isotemp-ocxo-33-madhat",
  ...
  "slow_integrator": {
    "version": 1,
    "calibrated_at": "2026-06-XX",
    "β_per_arm": { "extint": 0.1, "ticc": 0.05, ... }
  }
}
```

---

## 6. Test plan

### 6.1 Unit tests (`tests/test_slow_integrator.py`)

1. **Monitor exposes `mean_innov`** — `InnovControlMonitor.evaluate()`
   returns `HealthReport.mean_innov` correctly computed and `nan`
   during warm-up.

2. **Per-arm K reuse** — `_apply_slow_corrections` reads
   `self.last_arm_K[arm]` and applies the correction along that
   direction.  Mock K = [0, 0, 1, 0]; verify state delta is on x[2]
   only.

3. **Clamp engages** — feed a huge mock bias (e.g., 1 µs).  Verify
   the applied dx is bounded by `slow_step_clamp`.

4. **Warm-up suppresses correction** — feed 100 samples (below
   `warm_min`); verify no state change.

5. **Status != OK suppresses** — make the monitor report
   `ALARM_corr`; verify no state change.

6. **Disabled arm skipped** — slow_arms = ['extint'] only.  TICC's
   slow_pull should stay 0 even with non-zero bias.

7. **Slow + fast composition** — fast update applies `K·innov`,
   then slow applies `β·K·bias`.  Verify total state delta is
   the sum.

### 6.2 Integration tests

8. **300-sample bias signal recovers** — synthetic measurement
   stream with `innov = N(9.0, 2.7)` (clkPoC3 extint's profile).
   After 300 samples, monitor reports `mean_innov ≈ 9.0 ± 0.2`.
   Slow correction applied per spec.

9. **Cascade suppression** — synthetic innov with a 10-µs step at
   sample 100.  Verify the slow correction does NOT track it (the
   step is too brief to dominate the window mean before fast path
   catches up).

### 6.3 Lab validation

10. **clkPoC3 24-h capture** with slow integrator on extint+ticc.
    Pass criteria from §5.3.

11. **Three-host cross-host CDF** (PiFace + clkPoC3 + MadHat) on
    `pull-attribution-log` branch + slow-integrator branch.
    Pass: best-pair p95 ≤ 4 ns (down from 6.9 ns).

---

## 7. Roll-out

### Dependency order

1. **PR #118 lands** (pull-attribution schema) — provides `K_arm`
   and `would_pull_arm` columns for diagnostic comparison.
2. **PR #119 lands** (budget-doc revision) — documents the
   architecture this PR implements against.
3. **This PR**: `slowIntegratorArchA`.

### Sub-PRs (if too large)

This work could land as a single PR (~200 lines code + ~150 lines
tests) or split:

- **3a**: `InnovControlMonitor.mean_innov` field added (observer
  enhancement, ~5 lines).  Safe to land independently.
- **3b**: per-arm monitor instances + feeding + clamp.  Adds the
  feedback path, gated by `slow_arms=[]` default empty.  No
  behavior change until a host enables it.
- **3c**: schema v4 columns (`slow_pull_X_xN`, `slow_bias_X`).
  Always-on logging, no behavior change.
- **3d**: per-host enablement via `state/dos/<do_label>.json`'s
  `slow_integrator.β_per_arm`.  Operational; opt-in per host.

Recommend single PR.  The diff is small enough.

---

## 8. Open questions

These remain for review and don't block the plan.

- **Should TDCP-on-frequency-only hosts use slow integrator at
  all?**  TDCP measures `df/f`; the integrator on TDCP would
  correct `x[1]` (f_rx).  Probably fine but worth checking that
  the integrator doesn't fight the fast path's natural smoothing
  of TDCP measurements.

- **Per-arm β stored in dos.json vs config TOML.**  dos.json is
  the lab-inventory analogue of antennas.json; config TOML is
  per-host machine state.  Both could carry β.  Suggest dos.json
  for consistency with timestamper bias.

- **When the slow integrator is enabled, do we still need
  `InnovControlMonitor.norm_bias` and `corr_u_innov` alarm
  channels?**  Yes — they remain valuable observability signals
  (plant-model error detection), independent of whether the slow
  path is on.

- **Interaction with `--max-adjfine-step-ppb 2.0`?**  The slow
  correction modifies state directly, not adjfine.  The next LQR
  cycle reads the corrected state and computes a new adjfine,
  subject to the existing step clamp.  No additional interaction
  needed.

---

## 9. References

- `docs/two-site-sync-budget.md §3.2.1` — gate-lockout failure mode
- `scripts/peppar_fix/innov_control_monitor.py` — existing observer
- PR #118 `pullAttributionLog` — schema prerequisite
- PR #119 `budget-doc-mid-tau` — architectural target
- Conversation thread 2026-06-01 — per-arm vs fused decision data

---

*Drafted: 2026-06-01.  Status: ready for Bob's review.*
