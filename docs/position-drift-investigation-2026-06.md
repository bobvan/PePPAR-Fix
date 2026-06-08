# Free-position drift investigation (2026-06-04 → 06-07)

Dayplan umbrella: **seedNullDriftExperiment** (I-222315-main). Spawned
follow-up: **pickyEaterSSR** (I-210733-main). Closed: **solidTideAblationBias**
(I-211101-main). Related: timeHatFloatBias (I-053035), madhatF9pVerticalResidual
(I-134212).

## TL;DR

The lab fleet's PPP **position** solution wandered from the surveyed ARP —
originally read as a "~13 cm vertical bias," later as meter-scale drift.
A week of controlled, matched-pair experiments established:

1. **It is not** a seed bias, the solid-earth-tide model, PCV, ambiguity
   resolution (AR), Q_pos, or a single receiver. Each was a plausible
   suspect; each was ruled out by a same-time/same-sky test.
2. **It is** a *time-varying* excitation of the **(position, ZTD, clock)
   null** — the directions the geometry can't separate. Calm windows leave
   it ±0.3 m; active windows push it ±1–2 m.
3. **State constraints relocate the misfit, they do not eliminate it**
   (the "misfit-conservation" lesson). Pinning ZTD just pushes the error
   into height; pinning position pushes it into ZTD.
4. **The RTKLIB recipe works**: constrain *all* the null axes at once —
   white-noise clock + tight ZTD random-walk + static position-Q — and the
   free position **bounds around truth** (F9P: mean +45 mm, σ 260 mm) where
   one-axis attempts only relocated the misfit. Committed as first-class
   knobs (`--clock-model wno`, `--q-ztd-antpos`).
5. **The residual ~0.26 m is products-limited, not filter-limited.** A
   PRIDE yardstick on the *same* observations with *equal* (real-time)
   products lands at 0.13–0.41 m — same ballpark — and PRIDE-batch beats a
   plain forward float (±5 m) by smoothing, which the recipe largely
   recovers. **cm-class needs *final* products** (mm orbit/clock, days of
   latency), which a real-time engine can't use.
6. **Acceptance bar** (Bob): for the time mission, **stable ±10 cm with
   honest σ** is the goal, not cm. 10 cm ≈ 333 ps, well under the few-ns
   clock-agreement budget. Drift-away and false-confidence are the real
   failures.

**Operational conclusion (validated, unchanged):** establish the ARP with
the offline **PRIDE-final daily survey** (~5 mm) and **pin it**
(`--no-antposest`); don't free-estimate position in the real-time loop.
The real-time free-position floor is the corrections (~dm), not the filter.

## The question

PCV default-on (PR #135/#136) removed a masking factor and exposed an
altitude wander on the lab fleet (all on the shared UFO1 antenna via the
GUS splitter; see [[ufo1-choke1-two-arps]]). Bob's reframe drove the
method: *not* "converge to truth" but **"what leads it away from truth?"**

## Four stale premises, cleared

Every initially-attributed cause turned out to be true *before* recent
fixes but stale *now* — see [[position-bias-cleared-premises-20260605]]:

| Premise | Verdict |
|---|---|
| antennas.json NAD83-as-ITRF frame bug (1.71 m) | Real, but already **fixed** (#129–132). Beware: the UFO1↔CHOKE1 mount-point separation is *also* ~1.7 m — a look-alike trap ([[ufo1-choke1-two-arps]]). |
| Solid earth tide "~13 cm frame bug" | **Disproven** (PR #139). Model matches ERFA (0.1°) + all IERS DEHANTTIDEINEL cases (4–8 mm). The "frame bug" was a comparison artifact (IERS test vectors are ECI, fed into an ECEF compare). See [[solid-tide-verified-correct-20260605]]. |
| NAV2 seed bias (1.5–4 m) | **Stale.** Measured against the old (frame-buggy) survey; and a frozen biased seed gives a static offset, not the observed *ramp*. |
| "SFESPK6618H is L1/L2-only → L5 PCV leak" | **Wrong.** The ngs20.atx extract has G05/E05/E06/E07; the engine maps GPS-L5Q→G05 etc. L5-vs-L2 PCO differs ~1 mm — not a lever. |

## The seed-at-truth design

Bob's key insight: **a null is a property of (geometry, model), independent
of the seed.** So seed the position at the surveyed truth and leave it free:

- *Seed-bias hypothesis* → stays at truth.
- *Null-drift hypothesis* → drifts away anyway (a better seed never helps).

They make opposite predictions; one run falsifies the other. Tooling:
`scripts/analyze_seed_null_drift.py` (PRs #140/#141/#142) — parses the
`[AntPosEst]` log lines, fits the seed-independent drift slope, the
alt↔ZTD anti-correlation, the engine's own `worstσ` null-mode signature,
and cross-host common-mode vs divergent drift.

## Findings chronology

- **Seed at truth, free position → drifts.** Seed-bias falsified.
- **Matched AR-vs-float** (identical F9P pair, same sky): float ≈ AR to
  <120 mm while *both* wandered ~1.8 m. **AR is not the driver.** (This
  corrected an earlier, time-window-confounded "AR drives it" claim — a
  cautionary tale about non-matched comparisons.)
- **Cross-host correlation** of two SSR arms (different receivers): r = +0.99
  in one window (common-mode → shared source, not receiver), r = +0.31 in
  another (per-receiver dominates). **The common-mode/per-receiver split is
  itself time-varying** — the drift is a window-dependent *mix*.
- **Single-constellation is rank-deficient for free position.** Broadcast-
  only single-const never converges (AntPosEst stuck "converging", 0 position
  epochs; clock servo fine). Single-const + SSR would instead *drift* (the
  documented `solve_ppp` degeneracy). Minimum observable = 2 constellations.
- **ZTD constraint backfires.** Pinning ZTD to the apriori (residual→0)
  drove height to +0.5–0.8 m while the unpinned control stayed near truth.
  Even pinning to the *METAR-true* value (+199 mm) drove a sustained +329 mm
  height drift vs a default-loose-tie control at −8 mm. **Misfit is conserved
  across the null** — constraints choose *which state absorbs it*, not whether
  it exists.

## The (pos, ZTD, clk) null & misfit-conservation

At a single epoch the geometry can't separate a vertical shift from a ZTD
change from a receiver-clock change — they trade with little residual
penalty. Over time, changing satellite geometry *should* break the
degeneracy; when it doesn't fast enough (or a slowly-varying input forces
the weak direction), the solution slides along the null. Every state
constraint we tried (Q_pos tight, ZTD tie) merely **relocated** the misfit
to whichever axis was still free. You cannot constrain your way out; the
error source is upstream.

## pickyEaterSSR — the cross-AC test is blocked

To test whether the shared, common-mode source is the SSR stream vs the
atmosphere, we tried other analysis centers. **Only CNES (`SSRA00CNE0`)
free-solves.** Broadcast (too coarse), BKG (`SSRA00BKG0`, no phase biases),
and CAS (`SSRA01CAS1`, 3574 correction rejections despite having phase
biases) all leave AntPosEst stuck. The engine's correction handling
(GAP_FILL allow-list, signal mapping, CNES+WHU dual-mount) is CNES-specific.
Filed as **pickyEaterSSR** (I-210733-main): make a second AC digest, or
analyze the CNES correction stream directly.

## RTKLIB research → the recipe

RTKLIB's real-time PPP (`src/ppp.c`) has **no secret sauce** — it's a
conventional forward EKF (and is actually mediocre in real-time; its
strength is static post-processing). Its deliberate state-model choices:
**clock = white noise** (re-initialized every epoch), **position (static)
= zero process noise**, **ZTD = slow random walk** (~1e-4 m/√s), + robust
weighting. The "trick" is the **combination** — constrain *all* null axes
so the misfit lands in observable residuals instead of relocating. See
[[rtklib-ppp-state-model]].

We already had the knobs (`solve_ppp.py`): `clock_model ∈ {random_walk,
calibrated_white, wno}`; our ZTD RW Q was ~10× looser than RTKLIB's.

**Validated** (matched F9P): the all-axes recipe — `--clock-model wno`
+ `--q-ztd-antpos 1e-4` + `--q-pos-converged 1e-9` + float, **default ZTD
tie** — bounds the F9P free position *around truth* (mean +45 mm, σ 260 mm)
where a plain float wandered meters. **First config to bound the drift, not
relocate it.** Committed first-class in **PR #145** (`--clock-model wno`
choice + `--q-ztd-antpos` flag; default-preserving). Note: do **not** pair
it with a tight ZTD *tie* — that backfires (above).

## The PRIDE yardstick (2026-06-07)

Ran PRIDE (batch PPP-AR) on the *same* RINEX arcs the live engine drifted
on (`gt:/home/bob/gt/peppar-fix-data/pridecmp-20260606/`). **Important:**
DOY-157 final/rapid products weren't out, so PRIDE auto-used WUM
**real-time-stream** products — *the same grade our engine consumes*.

| Arc | PRIDE-batch (static) | Live engine (plain forward float) | RTKLIB recipe |
|---|---|---|---|
| F9P (L2C) | 0.41 m, bounded (σ 39 mm) | mean +587, **σ 886 mm, ±5 m** | ±0.26 m |
| F9T (L5) | 0.13 m, bounded (σ 17 mm) | mean +310, **σ 1357 mm, ±5 m** | ±0.26 m |

- **Batch > forward-float on equal products** (0.13–0.41 m bounded vs ±5 m).
- **The recipe closes that gap** — competitive with batch on equal products.
  So the residual is **products-limited, not filter-limited.**
- **cm needs *final* products** (the daily survey already hits ~5 mm); PRIDE
  on real-time products is dm-class, *and itself ~10× overconfident*.
- WUM L5 gap did **not** bite — F9T (0.13 m) beat F9P (0.41 m).

## Acceptance criterion

For the time mission (Bob): **stable ±10 cm with honest σ** beats cm-with-
drift. 10 cm ≈ 333 ps, well under the few-ns cross-clock budget (the DO
time random walk dominates). Grading: drift-away = fail; false-confidence
(σ ≪ actual error) = fail; bounded-±10cm-honest = PASS. See
[[position-accuracy-target-10cm]]. The recipe is the *right shape*
(centered, no drift) but its ±0.26 m σ is still outside 10 cm, and ±10 cm
in real time is a *corrections* problem, not a filter-smarts problem.

## Methodological lessons

- **Matched, same-time, same-sky pairs are essential.** The drift is
  time-varying, so non-matched comparisons repeatedly misled (the AR
  "verdict" flip; the apparent receiver-split; calm-window "convergence"
  that didn't hold). The clean answers all came from matched pairs.
- **Misfit-conservation:** state constraints relocate, don't eliminate.
- **Stale-premise pattern:** re-verify attributed causes after fixes land;
  several "known" biases were stale numbers entangled with already-fixed bugs.
- A throwaway analysis script had a parse bug that inflated one read; always
  verify against raw log lines.

## Artifacts

- **Code:** `scripts/analyze_seed_null_drift.py`; recipe knobs in
  `scripts/solve_ppp.py` (`PPPFilter` `ztd_q_coef`, `wno`) +
  `scripts/peppar_fix_engine.py` (`--clock-model wno`, `--q-ztd-antpos`).
  PRs #139, #140, #141, #142, #145.
- **Data:** `gt:/home/bob/gt/peppar-fix-data/` — `seednull-20260605/`,
  `pridecmp-20260606/`, `day0606-ztdprior-piface-tighttie.*`.
- **Memories:** [[position-bias-cleared-premises-20260605]],
  [[solid-tide-verified-correct-20260605]], [[converging-config-found-20260605]],
  [[rtklib-ppp-state-model]], [[position-accuracy-target-10cm]],
  [[ufo1-choke1-two-arps]].

## pickyEaterSSR — resolved (the source) + remaining (the engine gap)

Direct analysis of the live CNES stream (`scripts/diag_ssr_stream.py`,
2026-06-07, 15-min capture of SSRA00CNE0 clock corrections) settled the
"SSR-stream vs atmosphere" question:

- **The CNES feed is healthy** — steady ~5 s clock cadence, no real
  dropouts; the common-mode clock is stable (std 22 mm) and *benign*
  (the receiver-clock state absorbs it, so it doesn't reach position).
- **The position-relevant signal — the *de-meaned per-SV* clock (GPS+GAL)
  — sits at the ~cm level** (median std 12 mm, worst 45 mm; occasional dm
  jumps that are partly IOD / de-mean-set-change artifacts).
- **That ~cm differential, amplified through the ill-conditioned
  (pos, ZTD, clk) null, is the right magnitude to produce the dm-scale
  position excursions.** So the driver is the **inherent real-time-SSR
  product accuracy**, not a stream defect and not atmosphere — exactly
  consistent with the PRIDE yardstick (real-time products of *any* AC →
  dm floor). cm needs *final* products.

Caveat: this is correlation-by-*magnitude*, not time-aligned to a position
trajectory — the clean concurrent cross-AC correlation is still blocked by
the engine's CNES-only-free-solve limitation. **What remains under
pickyEaterSSR is therefore the *engine* gap**, not the science question:
the correction handling (GAP_FILL allow-list, signal mapping, CNES+WHU
dual-mount) is CNES-specific, so BKG/CAS/broadcast can't free-solve. That's
a resilience gap (no AC failover) + blocks any future cross-AC test.

## RTKLIB recipe — status & remaining work

The recipe **knobs are implemented and committed** (PR #145):
`--clock-model wno` + `--q-ztd-antpos` (default-preserving). The validated
config is runnable today: `--clock-model wno --q-ztd-antpos 1e-4
--q-pos-converged 1e-9 --no-ar` with the **default** ZTD tie.

It is **not operationalized, and arguably shouldn't be** for the time
mission: the fleet runs `--no-antposest` (position *pinned* at the
PRIDE-final surveyed ARP), so AntPosEst — and thus the recipe — is off the
operational path. The recipe's value is as the *best free-position config*
(competitive with batch on equal products), for diagnostics or a future
free-position use case. Remaining work, only if we pursue free-position to
the ±10 cm bar:
1. **Honest σ** — the recipe is ~10× overconfident (σ 8–58 mm while 0.26 m
   off); false confidence is its own failure ([[position-accuracy-target-10cm]]).
2. **Lower the residual** — products-limited, so it needs *better real-time
   corrections* (rapid products / a better SSR stream), **not** a smarter
   filter.
3. Config/wrapper wiring + a tested recipe mode, *if* it becomes a used path.
4. Housekeeping: reconcile the lab-local recipe edits still on MadHat
   (revert to the committed flags) and stop the lingering keeper run.

## Other open threads

- timeHatFloatBias (I-053035), madhatF9pVerticalResidual (I-134212) remain
  as per-host residuals, secondary to the common-mode story.
