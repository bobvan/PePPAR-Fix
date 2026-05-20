# Position bias hunt — 2026-05-19 / 2026-05-20

## Why this was opened

Through 2026-05-18 and 2026-05-19 the engine's AntPosEst running-mean position
diverged 1–5 m from the surveyed UFO1 ARP on every lab host that had been
cold-started.  Daytime evidence (`project_session_handoff_20260519`) was that
the bias was independent of antenna calibration (proven by the CHOKE1 → UFO1
swap, same residual scale), independent of `--nav-sig-gate`, and independent
of receiver-firmware version.  The bias direction differed per host but the
multi-meter magnitude held across all four lab hosts (TimeHat TIM 2.20,
MadHat F10T, PiFace + clkPoC3 TIM 2.25).

The clock-side output (`FixedPosFilter` at the pinned ARP) was unaffected —
position bias didn't propagate into `dt_rx` — so the time mission was safe.
But the AntPosEst-vs-pin divergence meant the engine's own position
estimation was untrustworthy, blocking PPP-AR validation and any future
position-driven re-anchoring.

## How we approached it

Ten rounds of constellation/correction bisection across the four lab hosts
on the evening of 2026-05-19.  Notation: `→ N.NN m` = 3D Euclidean distance
between the AntPosEst running-mean ECEF and the antennas.json UFO1 ARP, at
the indicated elapsed time after engine restart.

### Round 1 — drop-one-constellation with full SSR
Per host: gate-on, SSR (orbit/clock/code-bias + phase-bias) active, drop a
different constellation each.

| Host | Recv | --systems | t=37 min 3D bias |
|------|------|-----------|------------------|
| TimeHat | TIM 2.20 | gps,gal | 1.04 m |
| MadHat | F10T | gps,bds | 3.85 m |
| PiFace | TIM 2.25 | gal,bds | 2.65 m |
| clkPoC3 | TIM 2.25 | gps,gal,bds | 2.66 m |

**PiFace and clkPoC3 reported identical biased positions to within 6 cm
across all three axes despite running totally different constellation
configurations.**  Their bias direction was identical (+0.6 m N, −2 m E,
−1.6 m alt).  Both are TIM 2.25 receivers; both run CNES-only SSR (no
secondary bias mount).  TimeHat and MadHat had a *different* SSR config
(CNES primary + WHU `ssr_bias_mount` for phase biases) — confounding the
"per-receiver-model" claim from the initial reading.  Round 1 actually
proved **the bias is per-(receiver-model × SSR-config), not constellation.**

### Round 2 — single-constellation + SSR
Three GAL-only hosts (TimeHat/MadHat/PiFace) and one GPS-only (clkPoC3).
TimeHat and PiFace seed-dominated to dh ≈ 3–5 cm at t < 5 min, then drifted
to ~1 m at t = 20 min as the rank-deficient `(rx_clk, ZTD, common-ambiguity)`
mode predicted by `peppar_fix_engine.py --systems` help text fired:

> single-constellation runs with smooth precise clocks (SSR-orbit+clock or
> SP3+CLK) are subject to a near-singular filter mode where (rx_clk, ZTD,
> common-ambiguity) drift together with zero residual.

ZTD ramped 25 mm per 10s epoch (+472 mm → +544 mm over 30s) — exact
absorption signature of the null mode.  MadHat (F10T GAL-only) couldn't
arm at all; clkPoC3 (GPS-only) never reached `[AntPosEst N]` output —
GPS-only + CNES misses on L5Q PB (CNES publishes L5I).

**Conclusion: single-constellation + SSR is not a usable experimental
mode.  The engine warning is load-bearing.**

### Rounds 3–5 — broadcast-only (`--no-ssr`, `--no-ssr-phase-bias`)
Attempted to remove SSR entirely to find a "zero-bias baseline" we could
add corrections back onto.  **All three rounds produced no AntPosEst
output at all.**

Architectural finding: `scripts/realtime_ppp.py:1420` set
`ar_phase_bias_ok = False` by default, only flipping `True` when SSR was
present *and* both bands of the IF combination had matching phase biases.
`peppar_fix/obs_routing.py:obs_for_position()` then dropped every SV with
`ar_phase_bias_ok=False`.  Result: with `--no-ssr` the PPPFilter saw zero
observations per epoch, the `if n_used < 4: continue` guard fired
silently, AntPosEst stayed in `CONVERGING` forever with no position
output.

The engine has no production path for broadcast-only AntPosEst.

### Patch — diag-allow-no-pb-position branch
Diagnostic 1-line change in `realtime_ppp.py:1420` to default
`ar_phase_bias_ok = True` and only flip to `False` when SSR was present
*and* PB lookup was partial (one band has, the other missing — biased IF
combination).  The "both bands missing" case is now treated as
broadcast-only intent and obs are admitted.

This is a **diagnostic** patch, not a production fix.  AR machinery
(LAMBDA, MW, NL resolver) still requires PB on both bands per
`docs/ac-datum-mixing.md` — that contract is unchanged.  The patch only
opens the position-side gate so the PPPFilter can run float-ambiguity
PPP without SSR.

### Round 7 — broadcast-only with patch
All 4 hosts, `--systems gps,gal,bds --no-ssr --nav-sig-gate`.  AntPosEst
produced output, sub-decimeter at t < 5 min (seed dominated), but drifted
to **3D ≈ 2.7 m at t=15 min** (PiFace 2.73, clkPoC3 2.61, TimeHat 1.65,
MadHat 1.41).  Different direction than Round 1 (broadcast UP, SSR DOWN).
Growing monotonically — classic broadcast-clock-error-into-float-amb
random walk.

### Round 8 — SSR-orbit-clock + no-PB
All 4 hosts `--no-ssr-phase-bias` only.  Kept SSR-corrected satellite
clocks (cm-class) but dropped phase bias application.

| Host | t=15 min 3D | t=28 min 3D |
|------|-------------|-------------|
| TimeHat | 0.31 m | 0.86 m |
| MadHat | 0.32 m | 1.84 m |
| PiFace | 0.47 m | **2.58 m** (matches Round 1's 2.65 m) |
| clkPoC3 | 0.37 m | 1.99 m |

**The "PB application is the bug" hypothesis was wrong.**  Round 8
delayed the bias emergence but didn't prevent it.  PiFace climbed to
exactly the Round-1 magnitude (2.58 vs 2.65 m) by t=28 min.  Removing PB
just changed the trajectory shape, not the destination.

### Round 9 — strip further per host
PiFace dropped BDS too: SSR-OC + gps,gal.  Still drifted, to 1.7 m at
t=18 min.

### Round 10 — broadcast + gps,gal (drop BDS, drop SSR)
TimeHat/MadHat/PiFace on `--no-ssr --no-ssr-phase-bias --no-ssr-code-bias`
+ `--systems gps,gal --nav-sig-gate`.  clkPoC3 on broadcast + GPS-only
(couldn't arm, σ_e > 0.05 threshold).

**3-host steady state at 0.5–0.8 m horizontal, < 15 cm vertical,
bounded.**  Same-time Round 7 (broadcast + gps,gal,bds) was 1.4–2.7 m.
**BDS contributed at least half of the broadcast-only bias.**

The three armed hosts (TIM 2.20 + F10T + TIM 2.25) all landed within
12 cm of each other.  Per-receiver-model signature from Round 1
disappeared.  Strongly suggests BDS handling is where the per-receiver
asymmetry enters — receivers track different BDS signals (TIM 2.20:
B1I+B2I; TIM 2.25: B1I+B2aI; F10T: B1C+B2aP) and the engine maps these
to ionosphere-free combinations that interact with PB lookup differently
per receiver.

## Overnight (~8 hr auto-restart, 2026-05-20 ≈ 21:30–06:00 CDT)

Persistent monitor polled per-host AntPosEst every 5 min and, when a
host stayed >1 m for 15 consecutive minutes, killed and restarted it on
the next config in a 5-element cycle (gate-on, no-gate, +BDS, gal-only,
SSR-OC).  Every host cycled through all 5 configs at least twice.

### Dominant pattern: ~30-min synchronized waves

All four hosts drifted up together to 1–3 m every ~30 minutes regardless
of config.  Waves affected gate-on, no-gate, BDS, gal-only, SSR-OC
equally.  Recovery 5–10 min after each wave as the 60-epoch running mean
cycled bad epochs out.

**The disturbance is external (atmospheric and/or satellite-geometry),
not config-specific.**

### Per-host restart counts (8 hrs)
TimeHat 11, MadHat 9, PiFace 11, clkPoC3 13.

### What no config achieved
- No sub-1 m steady-state lasting > 30 min on any host on any config
- The 0.5–0.8 m / < 15 cm-vertical band seen briefly in Round 10 was a
  single inter-wave window, not a steady state
- clkPoC3 (TIM 2.25, CNES-only SSR) had the worst spikes — multiple
  3–5 m vertical excursions
- MadHat (F10T) had long unarmed stretches on gal-only — σ_e at the
  0.05 arm gate but not below — F10T's BDS-B2aP signals don't pair as
  cleanly with GAL when BDS is dropped

## What this proved, what it didn't

### Proved
1. **The position bias isn't constellation-specific** (Round 1's
   PiFace=clkPoC3 with different configs both at 2.65 m; Round 8 paths
   all converged toward Round-1 values)
2. **The bias isn't SSR-PB-application alone** (Round 8 with PB
   disabled still reached Round-1 magnitude; the PB-application path is
   one contributor, not the sole one)
3. **BDS contributes ~half of the broadcast-only bias** (Round 10 vs
   Round 7 at same elapsed time)
4. **All our runs had `NL: 0 fixed`** — float narrow-lane ambiguities
   the entire night.  Without NL integer-fixing, position is coupled
   to (clock, ZTD, common-ambiguity) and absorbs disturbances rather
   than rejecting them
5. **The engine's broadcast-only path needs work** — the
   `obs_for_position` gate currently requires PB.  Filed as a
   diagnostic patch only

### Not proved
- The ~30-min wave cadence's origin (ionospheric? satellite geometry?
  broadcast-eph segment boundary?  Need a separate atmospheric +
  geometry investigation)
- Whether NL fixing on the lab hosts is achievable today with our SSR
  source(s); what blocks it; how the per-host signal-set differences
  interact with NL search
- The MadHat-F10T-gal-only arm difficulty (separate bead worth filing
  if BDS-less F10T runs are ever needed)

## Path forward — NL fixing

Every drift mode we observed has the same root cause: **float
narrow-lane ambiguities under disturbance**.  With NL integers locked,
the (position, clock, ZTD) coupling is broken — ambiguities can't
absorb disturbances and force them onto other states.  Cm-class PPP-AR
position is the standard outcome once NL is fixed.

The engine has the AR machinery (`peppar_fix/ppp_ar.py`, `ppp_ar_resolver`,
LAMBDA, the Anchoring state machine) but tonight no NL ever fixed on
any host on any config.  The next investigation is:

1. **What's blocking NL fixing on our lab setup?**  The
   `[NL_DIAG]` logs from the engine, the `bootstrap_*` parameters, the
   `ratio_test` thresholds — all in place.  But the float ambiguities
   never converged to integer-resolvable values.
2. **Does the AC datum issue** (`ac-datum-mixing.md`) prevent NL on
   the dual-mount (CNES+WHU) hosts even with the right phase biases?
3. **Why don't the TIM 2.25 hosts (CNES-only) ever reach NL?**  Their
   PB coverage is matched on GPS+GAL.  BDS B2aI has no AC providing PB
   (`bds-b2a-phase-bias-survey-2026-05-09.md`).

These are investigations to open in subsequent sessions — the bias
hunt for tonight is closed: PR #42 (RINEX writer) is the only landed
change.  My diagnostic patch is being deleted (Option A in the
session-end summary); the intellectual value is captured in this
document.

## Code change log

Tonight's PRs / branches:
- **PR #42 merged** (`charlie/rinexWriterPridePrep`): RINEX writer now
  preserves data on wrapper-respawn (r+ mode), writes correct
  APPROX POSITION XYZ from `arp_label → antennas.json`, and (per
  Main's canary findings) rewrites stale APPROX-on-disk when a
  prior session left (0,0,0).  Squash-merged as `0a5a8f2`.
- **diag-allow-no-pb-position deleted**: the two diagnostic commits
  that defaulted `ar_phase_bias_ok = True` and handled the "both PBs
  missing" case.  Useful for the bias hunt but not a production
  improvement (NL fixing requires SSR-PB).  Decision: not worth
  carrying as a feature; bias hunt extracted the insight, document
  the architecture finding here, delete the branch.
