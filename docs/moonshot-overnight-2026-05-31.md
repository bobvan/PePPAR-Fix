# Moonshot stack — first full overnight (2026-05-30/31)

**Status:** results capture (Bob + main, 2026-05-31 morning).  Successor /
companion to the daytime smoke-test runs that exposed two #107 latent
bugs ([`disciplineModeFsmCtxThreading`] hotfix `24fac8d`, [`#115`]
holdoverResetKwarg) and the `_compute_tdev` CPU-burst pathology
([`#117`] noiseEstimatorComputeTdevVectorize).

[`#115`]: https://github.com/bobvan/PePPAR-Fix/pull/115
[`#117`]: https://github.com/bobvan/PePPAR-Fix/pull/117
[`disciplineModeFsmCtxThreading`]: ../scripts/peppar_fix_engine.py

## Setup

- **Commit:** `e1f1197` on `main` (PR #116 design doc + PR #117 vectorize +
  PR #115 holdover-reset-kwarg + my disciplineModeFsm ctx-threading hotfix
  + the day's earlier moonshot stack).
- **Engine flags (all 4 hosts):**

      --no-antposest
      --graded-taper --gross-fault-reset
      --router-qvir
      --coast-cap
      --max-adjfine-step-ppb 2.0

- **Launch:** 2026-05-31 **02:19:21 UTC**.  All 4 engines plus the
  PiFace `ticc_capture.py` cross-host logger at 02:21:51 UTC.
- **Hosts:**
  - PiFace (Pi 5, OCXO + AD5693R DAC gain=0, ticc3)
  - MadHat (Pi 5, IsoTemp OCXO-33 + AD5693R gain=1, F10T receiver, ticc2)
  - TimeHat (Pi 5, i226 PHC internal TCXO, ticc1)
  - clkPoC3 (Pi 4, OCXO + AD5693R gain=1, ticc5) — the worst-case host
    per [`clkpoc3-is-pi4-others-pi5`] (~2.5× slower CPU + 906 MB RAM).
- **Cross-host TICC:** ticc4 on PiFace, chA = PiFace PPS OUT, chB = clkPoC3
  PPS OUT (shared reference).

[`clkpoc3-is-pi4-others-pi5`]: ../../.claude/projects/-home-bob-git-PePPAR-Fix/memory/reference_clkpoc3_is_pi4.md

- **Files (data/ on each host):**
  - `day0531-overnight-{host}.log` — engine log
  - `day0531-overnight-{host}-ticc.csv` — chA/chB time-interval samples
  - `day0531-overnight-{host}-filter-state.csv` — EKF state per epoch
  - `day0531-overnight-{host}-arm-state.csv` — per-arm gate stats
  - `day0531-overnight-xhost-2026-05-31.csv` (PiFace only) — ticc4
    cross-host capture, auto-rotated at UTC midnight

## Engine health at 8 h 44 min

All 4 engines still alive on a single launch each (except MadHat, which
had 4 cold-start exit-5 cascades in its first 13 min before settling).
Stall counters from the engine's per-tick `Skip stats` dict at 8h44:

| Host    | etime | launches | exit-5 | gross-fault | gt 1.5 s | gt 5 s | gt 30 s | max stall | latest err  |
|---------|-------|----------|--------|-------------|----------|--------|---------|-----------|-------------|
| PiFace  | 8:43  | 1        | 0      | 0           | **0**    | 0      | 0       | **0.0 s** | +0.1 ns     |
| MadHat  | 8:22* | 5        | 4      | 3           | 2697     | **0**  | 0       | 2.0 s     | +0.7 ns     |
| TimeHat | 8:43  | 1        | 0      | 0           | 2        | 0      | 0       | 3.0 s     | +14 ns      |
| clkPoC3 | 8:43  | 1        | 0      | 0           | 125      | **0**  | 0       | 2.0 s     | +1.0 ns     |

\* MadHat etime is the latest launch; cascades all in first 13 min.

**Compare to yesterday's run-2 at the same 2 h elapsed**:

| Host    | gt 5 s before / overnight | max stall before / overnight |
|---------|---------------------------|------------------------------|
| PiFace  | 38 / **0**                | 9.0 s / **0.0 s**            |
| MadHat  | 112 / 0                   | 14.0 s / 2.0 s               |
| TimeHat | 124 / **0**               | 15.0 s / **0.0 s**           |
| clkPoC3 | 193 / 0                   | 39.0 s / 2.0 s               |

## PR #117 validated in production (py-spy on clkPoC3 at 2 h)

180 s record on the Pi 4 worst-case host, sampling at 100 Hz, at the
exact post-buffer-fill window where the bug would have been peaking:

| Frame in noise_estimator.py        | Before PR #117 | After PR #117 |
|-------------------------------------|----------------|----------------|
| `_recompute_tdev` + `_compute_tdev` | **28.27 %**    | **0.94 %**     |

MainThread caught **idle in `queue.get`** instead of mid-`_compute_tdev`
inner loop.  New top frames are I/O (NTRIP, serial, UBX parsing) — the
shape an idle-mostly engine should have.  SVG: `/tmp/clkpoc3-overnight-pyspy.svg`
on gt.

## TDEV(τ) reproducibility — run-1 / run-2 / overnight

chA-vs-TICC-Rb, last 90 min, linear-detrended, ps.  Bracket field shows
`run-1 / run-2 / overnight` for each host.

| τ (s) | PiFace                 | TimeHat                | MadHat                 | clkPoC3                |
|-------|------------------------|------------------------|------------------------|------------------------|
| 1     | 83 / 80 / **80**       | 2.48k / 4.19k / 3.00k  | 106 / 84 / 104         | 91 / 83 / 95           |
| 2     | 126 / 116 / **106**    | 3.37k / 9.65k / 4.32k  | 76 / 73 / 78           | 71 / 72 / 74           |
| 5     | 324 / 284 / **185**    | 7.14k / 36.34k / 7.06k | 67 / 86 / **58**       | 106 / 118 / **94**     |
| 10    | 612 / 510 / **240**    | 17.06k / 107k / 9.62k  | 135 / 193 / **106**    | 244 / 287 / **230**    |
| 30    | 1.50k / 1.12k / **569**| 75.6k / 548k / 12.58k  | 499 / 733 / **419**    | 916 / 1.10k / **840**  |
| 100   | 3.95k / 2.56k / **1.44k** | 162k / -- / 12.20k  | 1.13k / 1.78k / **1.06k** | 2.74k / 3.15k / 2.93k |
| 300   | 3.25k / 2.04k / **1.33k** | 104k / -- / 6.45k   | 1.13k / 1.37k / **1.06k** | 1.65k / 2.91k / 2.74k |
| 1000  | 2.59k / 2.28k / **775**| 35.79k / -- / 4.03k    | 775 / 990 / **276**    | 1.97k / 1.39k / 1.52k  |

**Reads:**

- Overnight is the **cleanest column at almost every (host, τ)** —
  PR #117 settling + 8 h to converge.
- **MadHat τ = 1000: 775 → 990 → 276 ps**.  Below 1 ns at τ = 1000 s,
  very close to the moonshot per-clock budget of ~350 ps.
- **PiFace τ = 1000: 2.59k → 2.28k → 775 ps**, 3.3× tighter than run-1.
- **TimeHat finally has valid τ ≥ 100 numbers** that didn't compute in
  run-2 (the bug-induced phase steps had killed them).  Still TCXO-floor
  at 3-12 ns; expected per CLAUDE.md TCXO-class best-effort.

## Cross-host PiFace ↔ clkPoC3 phase-agreement CDF

The empirical CDF of |Δ| = |chA − chB| at matching `ref_sec`, with
constant offset + linear drift removed (cable / Rb wiring artifacts,
not clock-agreement noise).  See `tools/plot_xhost_agreement_cdf.py`
for the tool that produced this; the **95 % horizontal crossing IS the
moonshot's 95 % excursion bound** per [`two-site-sync-budget.md`].

[`two-site-sync-budget.md`]: two-site-sync-budget.md

| Capture                                        | n     | p50      | p95     | p99      | max      |
|------------------------------------------------|-------|----------|---------|----------|----------|
| 2026-05-29 baseline (pre-merge, 24 h)         | 86399 | 13.4 ns  | 63.1 ns | 260 ns   | 13.4 µs  |
| 2026-05-31 overnight, full 8.7 h (incl. boot) | 31429 | 298.6 ns | 741 ns  | 24.9 µs  | 58.9 µs  |
| 2026-05-31 overnight, post-bootstrap (≥ 03:30Z) | 27335 | **4.7 ns** | **14.3 ns** | **19.8 ns** | **33.4 ns** |

**Bootstrap window must be trimmed** before reading the CDF — the
first ~70 min of the capture is dominated by both engines' freq-search
phase, and the differential between two converging clocks is huge by
construction.  The "full 8.7 h" row is shown only to illustrate this;
the meaningful comparison is the post-bootstrap row.

### Moonshot stack vs pre-merge baseline (post-bootstrap)

- p50: **4.7 ns vs 13.4 ns** → 2.8× better
- p95: **14.3 ns vs 63.1 ns** → 4.4× better
- p99: **19.8 ns vs 260 ns** → 13× better
- max: **33.4 ns vs 13.4 µs** → 400× better

The merged stack delivered measurable cross-host improvement at every
percentile.  But we are still **not at the 1 ns @ 95 % shared-antenna
moonshot target**; we sit at **14 ns @ 95 %, ~10× away from the goal**
— roughly an order of magnitude closer than the pre-merge baseline.
Progress, not arrival.

PNG: `/tmp/xhost/agreement-overnight-trimmed.png` on gt.

## MadHat physical-disturbance episode — exclude from future analysis

At **2026-05-31 12:31:30 UTC** (07:31:30 CDT, Sunday morning), MadHat got
a **7.67 µs phase step between two consecutive epochs** while it had
been steady at sub-ns err for 8+ hours.  Most likely cause: an
operator-bumped cable or fixture (PPS path, ticc2 chB cable, or the
TADD-2 5 MHz → 1 PPS divider on GPIO 16).  Ruled out: F10T receiver
clock step (clkB drifted smoothly through the event at 376 ns/s, no
NAV-CLOCK discontinuity); CPU stall (n_stalls_gt_5s stayed at 0); OCXO
itself (post-disturbance frozen-DAC drift measured at −86.3 ppb, R² = 1.000,
matching the IsoTemp OCXO-33 cal center of +85.69 ppb within 1 ppb).

What followed: 2 exit-5 cascades + 1 binary-layer [GROSS_FAULT_RESET]
across ~20 min, with the DAC repeatedly resetting to center on each
engine teardown (which is also why the underlying drift kept being
visible).  A clean SIGTERM + 60 s frozen-DAC capture at 12:48:51 UTC
confirmed the OCXO was at its natural center frequency — i.e. the bump
caused no permanent physical change.  Relaunched at 12:52:06 UTC;
within 2 min the engine reconverged to **adj = −90.1 ppb** vs the
pre-bump steady state of −90.7 ppb (0.6 ppb match), and resumed
sub-ns tracking.

**Disturbance window — already excised from MadHat's TICC csv** on
2026-05-31 at 13:00:53Z.  The window `[12:30:00Z, 12:57:00Z)` was
filtered out of `data/day0531-overnight-madhat-ticc.csv` on the
MadHat host (2347 rows removed, chA + chB rebalanced to 36 515 each).
The original is preserved as
`data/day0531-overnight-madhat-ticc.csv.disturbance-bak-20260531T130053Z`
on MadHat if needed for reconstruction.

The other MadHat logs (`day0531-overnight-madhat.log`,
`-filter-state.csv`, `-arm-state.csv`) were **intentionally left
untouched** so the disturbance is still studiable from those
artifacts.  Other three hosts (PiFace, TimeHat, clkPoC3) saw no
anomalies and their files are unmodified.

The bootstrap-trim recommendation (skip before 03:30Z) remains in
place for all hosts as before.

When invoking `tools/plot_xhost_virtual.py` or
`tools/plot_xhost_agreement_cdf.py`, just pass `--skip-before
2026-05-31T03:30:00Z` — the MadHat disturbance window is already
gone from the TICC csv, so no additional trim is needed.  (A native
`--skip-window FROM..TO` flag would still be a worthwhile follow-up
if exclusion windows become common across multiple hosts/episodes
where editing the source files isn't desired.)

## Open follow-ups (queued, none blocking)

1. **`exitFiveToServoReset`** — PR #116 (design doc) merged; bravo's
   implementation PR pending.  Targets the MadHat cold-start cascade
   pattern (4 exit-5s in first 13 min of every run; gone after).
2. **`freerunPhcPeroutArm`** — TimeHat DO char blocked tonight because
   `do_freerun_char_phc.py` (PR #113) doesn't arm PEROUT itself, and on
   the current kernel/driver PEROUT does NOT survive engine exit
   (chA test post-engine-stop = 0 samples in 8 s, even after fresh
   rmmod/modprobe igc).  Filed as a dayplan item with implementation
   sketch.  Medium priority — TimeHat is TCXO best-effort, so default
   `sigma_do_freq` is fine until the freerun-char unblocks honest Q.
3. **Closing the cross-host moonshot gap** (14 ns @ 95 % → 1 ns @ 95 %)
   — the next ~10× wants identification.  Candidates:
   - Per-clock floor at the 10 ns level — both clocks contribute in
     quadrature to the diff
   - Pi 4 jitter on clkPoC3 specifically (already on shortlist)
   - Cable / fixture differentials
   - TICC #4 reference-Rb noise floor
   - Residual bootstrap convergence even at 03:30Z (longer settle?)

## Reproducing this

```bash
# Per-host TDEV
venv/bin/python -c "..."   # the snippet from the run log

# Cross-host CDF
venv/bin/python tools/plot_xhost_agreement_cdf.py \
    --input /tmp/xhost/ticc4-pifaceA-clkpoc3B-2026-05-29.csv \
    --input /tmp/xhost/day0531-overnight-xhost-2026-05-31.csv \
    --input /tmp/xhost/day0531-overnight-xhost-trimmed.csv \
    --label 'baseline (24h)' \
    --label 'overnight full' \
    --label 'overnight trimmed' \
    --output /tmp/xhost/agreement-overnight-trimmed.png
```

Raw data archived to the GT home server pending pull.
