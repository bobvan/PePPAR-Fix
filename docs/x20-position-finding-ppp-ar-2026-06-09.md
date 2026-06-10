# X20P position-finding + PPP-AR on PiPuss (no DO) — 2026-06-09

Does the engine do position finding on the ZED-X20P (no DO host), and
does it converge and lock, with internet SSR and with HAS over the air?

## Setup

PiPuss, ZED-X20P on UFO1 (lab roof, clean sky), no disciplined
oscillator.  Engine in default (AntPosEst) mode, **not** `--no-antposest`.
Cold position finding forced with `--ignore-arp-state` (the default config
otherwise pins to the trusted UFO1 survey seed and AntPosEst runs 0
epochs).  Cold start seeds from NAV2 (hAcc ~0.45 m, acquired in ~4 s).

## Result 1 — it starts and does float position-finding fine

The engine starts cleanly (no DO ⇒ no servo/DO-char path), acquires a
NAV2 seed, and **AntPosEst converges to a stable ~5 cm float** with every
correction source.  Clock + ZTD always converge (dt_rx σ ~0.12 ns).

## Result 2 — position *lock* (NL/cm AR) depends entirely on the stream

Whether AntPosEst actively estimates (vs just holds the seed) is gated by
whether NL ambiguity resolution can form, which needs phase biases on the
signal codes the X20 tracks (Q-channels: L5Q, E5aQ).

| Stream | Phase bias | NL AR | Position |
|---|---|---|---|
| BKG SSRA00BKG0 | 0 | none | pinned at seed (no estimation) |
| CAS SSRA01CAS1 | 164, on **I-channels** (L5I, E5aI/E5bI) | none — all MISS | pinned at seed |
| HAS (E6 OTA) | 0 (Phase 1) | none | pinned at seed |
| **CNES SSRA00CNE0** | 270, on **Q-channels** (L5Q, L7Q) | GAL biases apply | **AntPosEst estimates → ~5 cm float** |

Only **CNES** publishes GAL phase biases on the Q-channels the X20 tracks
(`E5aQ→C5Q` HIT), so only CNES unlocks active estimation.  BKG/CAS/HAS
leave the position pinned at the seed (clock still converges).

## Result 3 — but NL fixes never take hold (40-min runs)

Two 40-minute CNES runs, cold start:

| config | NL fixes | PFR/unfix | relaunches | position |
|---|---|---|---|---|
| `--systems gal` | **0** | 0 | 0 | stable ~5.2 cm float |
| `--systems gps,gal` | **0** | 0 | 0 | stable ~5.5 cm float |

Both are **stable** — converge to ~5 cm float and hold for 40 min, no
drift, no PFR/false-fix events, no catastrophic relaunches.  But neither
achieves an NL/cm lock.  `[AR_READINESS]` stays at **`n=1` (too few
screened)** the whole time.  Two reasons:

1. **GPS is excluded from NL by the L5 wall.**  CNES publishes GPS L5
   phase bias only as **L5I**; the X20 tracks **L5Q** → `f2 GPS-L5Q→C5Q =
   MISS` (`avail=['L1C','L2W','L5I']`).  GPS SVs have no valid f2 bias, so
   they never enter NL screening.  *(The L5I→L5Q alias is a known dead
   end — tried in `b71e2b1`, reverted in `15f9b01`; the RTCM bias bundles
   receiver I-channel group delays, not a constant λ/4.  See
   `docs/l5i-l5q-phase-bias-empirical.md`.)*
2. **GAL WL fixes are rare, transient, low-confidence.**  WL side warms up
   ~8 SVs but at `p_wl_ib ≈ 0.02` (needs >0.99); only ~12 WL fixes land in
   40 min and they're ejected quickly (`reason=gf_step`, i.e. cycle-slip /
   GF step).  So the NL-eligible set collapses to 1.

## Interpretation

This is the **same CNES GPS-L5 phase-bias wall** documented for the F9T
fleet (`project_gps_l5i_l5q_bias_fix.md`, `project_cnes_phase_bias_signals.md`),
not an X20 regression.  Notably the X20 stays **safely float** here rather
than wrong-fixing and drifting the way the 2026-04 F9T did (nav2Δ 2→5 m) —
the current AR-readiness confidence gate correctly *declines* to NL-fix
when WL confidence is low, so there's no false-fix contamination.

The X20's **float** position-finding (no DO) works and is stable at ~5 cm.
**Cm PPP-AR lock is blocked** by correction availability (GPS-L5 wall) plus
thin/low-confidence GAL WL→NL screening.

**Contrast with native RTK:** `f9p_single_rtk.py` got a 23 mm / 4 s RTK
FIXED on this same X20 — because the *receiver* resolves its own signals
against an RTCM base.  The engine's PPP-AR depends on the SSR analysis
center publishing phase biases on the X20's exact RINEX codes, which only
CNES does (GAL only).

## Why the X20 GAL WL confidence (p_wl_ib) is so low — root cause

`p_wl_ib` (`MelbourneWubbenaTracker.wl_bootstrap_success_rate`) is
Geng et al. 2010's integer-bootstrap success rate, `∏ (2·Φ(0.5/σ_i) − 1)`.
The σ it feeds in is **`resid_std_cyc`** — but that is a **fixed 60-epoch
rolling std of the RAW per-epoch MW residual** (`mw_raw − mw_avg`,
`_RESID_WIN=60`).  It is deliberately kept *raw* so the slip detector's
jump threshold scales to per-epoch PR noise (code comment, ppp_ar.py:211).
It does **not** shrink with averaging.

But the WL *fix* uses the **EMA-smoothed mean** `mw_avg` (τ≈100 epochs),
whose uncertainty is ≈ `raw_σ/√τ`.  So `p_wl_ib` is computed from the
wrong σ:

| σ source | value | p_wl_ib (7 GAL SVs) |
|---|---|---|
| raw per-epoch (`resid_std_cyc`, what the code uses) | 0.3–0.6 cyc | **0.02–0.36** |
| averaged-mean (`raw_σ/√τ`, what Geng intends) | ~0.03–0.06 cyc | **≈1.0** |

**This flaw is masked on a clean F9T** (raw σ 0.05–0.20 cyc → p_wl_ib ≈ 1
anyway) **and exposed on the X20**, whose raw GAL code noise is ~2–3×
higher (`resid_std_cyc` 0.3–0.6 cyc, `resid_pr_rms` ~4–5 m GAL-only).

**The X20's elevated code noise is real but secondary** — not weak signal
(C/N0 42–49) and not slips (6 GF-steps in 40 min).  It's **code multipath
/ X20 code-tracking** at UFO1 (the carrier is clean — RTK got 23 mm).
Some SVs show biased WL means (frac 0.3–0.5, e.g. E04/E12) consistent with
code multipath; those won't fix regardless of the metric.

**Net:** the low `p_wl_ib` is **mostly a metric artifact** (raw per-epoch σ
instead of averaged-mean σ), amplified by genuinely noisier X20 code.  The
AR-readiness gate is therefore overly pessimistic on the X20 and
contributes to NL screening collapsing to `n=1`.  The smoothed WL means
*do* converge and SVs *do* fix WL — the gate just doesn't believe it.

**Proposed fix (testable):** in `wl_bootstrap_success_rate`, use the
**averaged-mean ambiguity σ** (uncertainty of `mw_avg` ≈ `resid_std_cyc /
√N_eff`, or track `mw_avg`'s own variance) rather than the raw per-epoch
`resid_std_cyc`.  That matches Geng's definition (formal ambiguity σ, not
measurement noise).  Caveat: do it frac-aware — a naive `raw_σ/√τ` assumes
white noise; the X20's multipath SVs (frac 0.3–0.5) should still score
low.  Validate by re-running CNES GAL-only and checking whether NL
screening rises above `n=1` and NL fixes take hold.  This is a fleet-wide
AR-gate change, so test on the X20 branch before touching other hosts.

## Open threads (not filed — for discussion)

- **Why are X20 GAL WL fixes so low-confidence (p_wl_ib≈0.02) and
  cycle-slip-prone?**  Is it the X20's E5aQ carrier noise, ZTD/troposphere
  convergence, geometry, or AR-gate tuning?  This is the real lever for
  GAL-only NL on the X20.
- **Configure the X20 to track GPS L5I (sigId 6) instead of L5Q (sigId
  7)** so CNES's L5I phase bias matches — the one untested GPS avenue
  (needs a u-blox CFG that may or may not exist on Gen-10).
- A different AC publishing GPS **L5Q** biases (WHU-class) would unblock
  GPS directly.
