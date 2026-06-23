# STEP C — mid-τ realization A/B (midTauTrackingResidual I-102153)

**Owner:** charlie (Bob reviews). **Status:** harness prepped; BLOCKED on lab
return (Main handed all hosts back at takedown; run when they're up).

## Why this shape (STEP A + B findings)

- **STEP A (free-running DO, chA-alone):** clkPoC3 DO floor is **199 ps@100 s /
  352 ps@200 s — UNDER the 354 ps budget**; MadHat rises steeply (409→4207 ps,
  100→1000 s).
- **STEP B (Q-vs-DO-freq-RW, charlie):** clkPoC3 configured `sigma_do_freq`
  =0.0006 ppb/√s ≈ the DO's measured mid-τ freq-RW (0.00062@100 s, 0.00027@300 s)
  → **Q is matched-to-slightly-loose, NOT too stiff.**
- **STEP B-PROPER (software solution chA−φ_DO, Main):** the EKF GPS-time estimate
  is **clean at mid-τ (~280 ps@100 s)**; the noise is in the **DO/SERVO
  REALIZATION**, not the estimate.

**Conclusion:** clkPoC3's disciplined 978 ps@100 s is neither the DO floor
(199 ps) nor the software estimate (~280 ps) — it's the **realization**: the loop
steers a DO that's *better than the reference at mid-τ* toward the reference,
injecting noise. The fix is to **reduce mid-τ loop aggressiveness** (coast
through 100–300 s where the DO wins; correct at long-τ where GNSS wins,
crossover ~300–500 s) — **NOT "looser Q"** (which would let *more* wander
through). MadHat is the opposite regime (DO worse than reference at mid-τ → loop
correctly helps → ~470 ps ≈ reference floor) and is the **control**.

## Arms (interleaved, time-of-day common-mode)

Primary host **clkPoC3** (servo-limited):

| arm | flags (added to the normal `--no-antposest … --qerr-latest-chi` invocation) | tests |
|---|---|---|
| **A baseline** | (none extra) | current production realization |
| **B coast** | `--coast-cap --coast-cap-k-sigma 0.5` | longer coast (k<1 caps coast *longer*) — the realization lever |
| **C stiffQ** | `--sigma-do-freq-override 0.0003` | stiffen Q toward the 300 s freerun rate (more coast via the filter) |
| **D both** | `--coast-cap --coast-cap-k-sigma 0.5 --sigma-do-freq-override 0.0003` | combined |

Control host **MadHat** (reference-floor-limited): arms **A baseline** vs
**B coast** only — expect **no improvement** (confirms it's reference-floored,
the lever there is a better receiver, not the loop).

> `--sigma-do-freq-override` (added 2026-06-23) is required because the do-char
> `.toml` value otherwise overrides `--kalman-sigma-freq`; the override wins over
> both and is logged loudly as a non-measured EXPERIMENT value.

## Schedule

Interleaved ~60 min segments per arm, ×2–3 cycles → ~6–13 h overnight per host
(mid-τ to 1000 s needs ≥~1 h segments; interleaving removes the time-of-day
confound, per the qErr-A/B method). Reset-free; archive per-arm logs.

## Metric & acceptance

- **Metric:** chA-alone detrended TDEV (CLAUDE.md metric; NOT chA-chB) at
  τ=100/200/500/1000 s, per arm, averaged across same-arm segments; χ² bands.
- **clkPoC3 success:** mid-τ chA-TDEV drops toward the **199–352 ps freerun
  floor** (target ≤354 ps), **short-τ (≤6 s) not regressed**, exit-5/state_sanity
  ≤ baseline. Identify which knob (coast vs Q vs both) moves it.
- **MadHat success:** confirm **no arm helps** (reference-floor-limited) — a
  negative result that localizes the remaining residual to the
  receiver/reference, not the loop.
- **Either way:** an honest per-host verdict (servo-realization-fixable vs
  reference-floor-bounded) + the concrete knob value, for Main's review.

## Harness

- `scripts/step_c_driver.py` — interleaved per-arm launcher (parametrized by
  host + arm; reset-free; per-arm CSV logs). **Do not run until lab is up.**
- `scripts/tdev_step_c.py` — per-arm long-τ chA-TDEV analysis (reuses the
  interleave seg-TDEV + averaging; χ² bands; version-stamped).
