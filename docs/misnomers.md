# Misnomer log

Naming-quality audit.  Sloppy names lead to sloppy thinking; this
file is the running record of identifiers that don't honestly
describe what they do.

**Not for batch renaming.**  Each entry is a candidate to fix
opportunistically when the surrounding code is being touched for
some other reason.  Renaming purely for naming pollutes git blame
and blast-radius for no test signal — only worth doing when you're
already in the diff.

## How to add an entry

```
### `<identifier>` — <severity>

**Where**: `path/to/file.py:LINE` (function `f`, class `C`)
**Claim**: what the name implies.
**Actual**: what the code actually does.
**Why it matters**: the bug or confusion this enables.
**Proposed**: suggested rename (or "see notes" if non-trivial).
**Notes**: anything about timing, scope, dependencies.
```

**Severity scale**:
- **Dangerous** — name implies wrong semantics; future reader
  could write a bug.  Fix sooner.
- **Misleading** — name oversells or omits a critical
  qualifier; reader has to dig.
- **Cosmetic** — imprecise but not misleading; fix only when
  in the diff anyway.

## Seed pass — 2026-04-24

Candidates found while auditing files I'd touched this week.
Initial pass; not exhaustive.

### `ecef_distance_m` — Dangerous

**Where**: `peppar_bus/cohort.py:131`
**Claim**: Distance between two ECEF positions.
**Actual**: Takes LLA tuples (lat_deg, lon_deg, alt_m), not ECEF
(X, Y, Z).  Uses flat-earth approximation, not Pythagoras on
ECEF deltas.
**Why it matters**: A caller who does
`ecef_distance_m(*ecef_a, *ecef_b)` would get a meaningless
number (interpreting X-meters as latitude-degrees, etc.).  The
flat-earth approximation is also caveat-worthy at intra-fleet
scales but already documented in the docstring.
**Proposed**: `lla_distance_flat_m` or
`topocentric_distance_m`.

### `PPPFilter.detect_cycle_slips` — Dangerous

**Where**: `scripts/solve_ppp.py:439`
**Claim**: Detects cycle slips.
**Actual**: Only catches the receiver's own lock-loss
indicator (`lock_duration_ms` decreasing).  Actual cycle-slip
detection (Melbourne-Wübbena jump, geometry-free residual,
arc-gap, etc.) lives in `MelbourneWubbenaTracker.detect_jump`
and the WL drift monitor.
**Why it matters**: A future reader looking for "where do we
detect cycle slips?" finds this method first and might assume
it's the slip detector.  The real detection stack is in
`scripts/peppar_fix/cycle_slip.py` and `ppp_ar.py`.  Using
this method in isolation will miss most slips.
**Proposed**: `detect_lock_loss_slips` or
`slips_from_lock_indicator`.

### `PPPFilter.add_ambiguity(sv, N_init_m)` — Misleading

**Where**: `scripts/solve_ppp.py:400`
**Claim**: Parameter `N_init_m` — capital `N` is the GNSS
convention for an integer cycle count, `_m` says metres.
**Actual**: The value is the float IF ambiguity in metres
(`phi_if_m - pr_if`), not an integer-derived value.  Stored
directly as a real-valued state variable until later
resolution.
**Why it matters**: A reader who knows the convention
(`N_WL`, `N1`, `N_NL` are integers; ambiguity-as-float is
typically `A` or `b`) sees the `N_` prefix and assumes
integer-derived.
**Proposed**: `amb_init_m` or `ambiguity_init_m`.  (Caller-
side variable `N_init` has the same issue but is more local.)

### `PPPFilter.tropo_delay(elevation_deg)` — Misleading

**Where**: `scripts/solve_ppp.py:502`
**Claim**: The tropospheric delay at this elevation.
**Actual**: The **a-priori dry/hydrostatic** tropospheric
slant delay only (`2.3 m * m_h(elev)`).  The wet residual is
estimated as a state variable and applied separately via
`x[IDX_ZTD] * wet_mapping(elev)` at the same callsite.
**Why it matters**: A reader sees `tropo = self.tropo_delay(elev)`
and thinks the full tropo is captured.  The full tropo at
this epoch is `tropo_delay + ztd_state * wet_mapping`.
**Proposed**: `apriori_hydrostatic_slant_m` or
`dry_tropo_slant_m`.

### `PPPFilter.wet_mapping(elevation_deg)` — Cosmetic

**Where**: `scripts/solve_ppp.py:514`
**Claim**: A *wet-specific* tropospheric mapping function.
**Actual**: Default impl is `1/sin(elev)` — identical to the
hydrostatic mapping factor.  With GMF active
(`_GMF_PROVIDER` set), it does return the wet-specific
Boehm 2006 mapping.
**Why it matters**: Until Phase 4 GMF (commit `c00a6dd` /
`b600519`) the "wet" qualifier was aspirational under the
trivial `1/sin(elev)` model.  Reader could think the wet
component is being mapped differently from dry — until
GMF is on, it isn't.  Now that GMF exists, the name is
honest in GMF mode and harmlessly redundant in default mode.
**Proposed**: leave for now; honest under GMF and
GMF is the future.

### `peppar_mon.LogState.antenna_position` — Cosmetic

**Where**: `peppar_mon/log_reader.py:131`
**Claim**: Tuple `(float, float, float)` is a "position".
**Actual**: Stores `(lat_deg, lon_deg, alt_m)` — mixed units
(degrees and metres) in a generic 3-tuple.  Units documented
in the docstring at line 124-130 but not in the type.
**Why it matters**: Mild — readers who need exact units have
to read the docstring.  A typed `NamedTuple` or `@dataclass
AntennaPositionLLA(lat_deg=..., lon_deg=..., alt_m=...)`
would make the unit mismatch visible at every callsite.
**Proposed**: `@dataclass class LLA: lat_deg: float;
lon_deg: float; alt_m: float`, used by `antenna_position`.

## 2026-04-28

### `WlDriftMonitor` / `wl_drift` / `[WL_DRIFT]` — Dangerous

**Where**: `scripts/peppar_fix/wl_drift_monitor.py:1` (class
docstring + log tag); engine call sites in
`scripts/peppar_fix_engine.py:2348-2386`.
**Claim**: Detects "wrong WL integer commits" — implies a phase-
side ambiguity-error detector, in line with the sunrise TEC slip
storm motivation.
**Actual**: Tracks the rolling mean of the **Melbourne-Wübbena
combination residual** post-fix.  MW = phase − pseudorange; the
residual responds to either-side disturbances.  Empirically (3-host
day0427night; Z = −0.17, p = 0.86 against BNC's independent IF-based
slip detector) the firing pattern is **statistically
indistinguishable from random** with respect to phase events.
Direct probe on three anti-correlated SVs (E29, E21, E19) showed
BNC's filter state smoothly drifting through engine wl_drift trips
— the disturbance was PR-side.
**Why it matters**: a future reader sees `wl_drift` events
demoting SVs and assumes phase-domain instability is the root
cause.  Investigation directions follow that framing (cycle-slip
diagnosis, ambiguity-resolution tuning) when the signal is
actually PR multipath / code-bias drift.  Burned hours of
investigation on day0427night until the BNC validator was built.
**Proposed**: not a simple rename — the underlying signal is
wrong for the documented use case.  Two paths in increasing
scope:

  1. Internal scope only: rename class to
     `MwResidualRollingMeanMonitor` and the log tag to `[MW_DRIFT]`
     to honestly describe the signal; keep adaptive thresholding
     by integer-history class (I-153334-main) on top.
  2. Replace the signal with a phase-only counterpart (GF or IF
     residual rolling mean, following BNC's lead).  Then the
     "wl_drift / wrong WL integer" framing becomes legitimate.
     Larger redesign — see proposal in dayplan.

**Notes**: see `project_wl_drift_smooth_float_signal_20260428` and
`project_wl_drift_vs_bnc_finding_20260428` for the full
investigation chain.

## Pattern — 2026-05-02

### Residual-domain mismatch — recurring shape

**The pattern**: a class/monitor whose name or stated job lives in
the integer-fix domain (NL, WL, ambiguity, fix-set, anchor,
setting-SV) but whose trigger reads PR (code) residuals while
ignoring IF (carrier-phase) residuals.

PR multipath inflates code residuals at low elevation while the
carrier-phase integer is unaffected.  When the trigger reads PR
only, every multipath spike gets misread as an integer-fix problem
and produces wasted filter actions (eviction, re-init, demotion).

**How to find more by inspection** — three conditions, mechanical:

1. Class/file name contains a token from the integer-fix
   vocabulary: `fix`, `ambiguity`, `integer`, `wl`, `nl`, `ar`,
   `anchor`, `lifecycle`, `sv_state`, `evict`, `drop`,
   `setting_sv`, `false_fix`, `integrity`.
2. `ingest()` or trigger reads PR residuals or filters out non-PR
   (`kind == 'pr'` / `kind != 'pr'` / iterates only `pr_resid`).
3. Docstring does **not** explicitly justify excluding carrier-phase
   residuals.

The clean exemplar that passes (1) and (2) but fails as a
candidate is `bootstrap_gate.py:79` — its docstring says *"Phi
residuals not checked — they pick up carrier-phase ambiguity bias
rather than measurement noise during cold-start float phase."*
That's the model: when PR-only is correct, the docstring tells
the inspector why.

When (3) fails — silent docstring or one that describes what the
code does without saying why IF is left out — you almost certainly
have a misnomer-and-bug.

### `SettingSvDropMonitor` Condition 2 — Misleading

**Where**: `scripts/peppar_fix/setting_sv_drop_monitor.py:124,191-235`
(class `SettingSvDropMonitor`, method `evaluate`).
**Claim**: Drops SVs that have become unreliable as they descend
through the setting band.  "Setting-SV drop: the intentional
removal of an SV from the fix set as it descends into
multipath-prone elevations."  Name and docstring frame this as an
integer-fix-domain action.
**Actual**: Trigger fires on elev-weighted **PR** mean only.  When
PR multipath inflates the code residual but the carrier-phase
integer remains correct (IF residual sub-cm), the SV is still
demoted ANCHORING → FLOATING.  The next epoch the same n_nl is
re-admitted because nothing was wrong.
**Why it matters**: 100% of same-second wasted evictions on
day0501 overnight (TimeHat 11/11, MadHat 10/10) trace to this
trigger.  Each wasted eviction costs trust-scaffold tier (mostly
PROVISIONAL preserved, some drop to NEW).  Across both hosts:
24/32 wasted evicts.
**Proposed**: Gate Condition 2 on IF residual breach in addition
to PR breach.  ~30 LOC change; see `I-161514-main`.
**Notes**: Condition 1 (absolute elev_mask) is geometric and
correct — leave alone.

### `FixSetIntegrityMonitor.window_rms` trip — Dangerous

**Where**: `scripts/peppar_fix/fix_set_integrity_monitor.py:195`
(method `ingest`, trip path `window_rms` in `evaluate`).
**Claim**: "Computes single-epoch RMS across SVs currently in
either ANCHORING or ANCHORED (the fix set)."  Fix-set is
integer-fix-domain language.  When the RMS exceeds threshold for a
sustained window, the most severe action in the system fires:
`[FIX_SET_INTEGRITY] TRIPPED reason=window_rms` → full filter
re-init.
**Actual**: RMS is computed over PR residuals only.  A multipath
storm on the fix set's PR residuals trips it identically whether
or not the integers are pathological.
**Why it matters**: 12 window_rms trips on TimeHat day0501.  Of
those, **8 (67%) are confirmed PR-multipath false positives** —
PR-RMS over the 30-epoch lookback was 5-12 m while IF-RMS was
4-80 mm (sub-cm to low-cm, integers clean).  3 (25%) were
justified (IF-RMS 122-226 mm, real integer pathology).  1 (8%)
ambiguous (no NL samples in window — re-init recovery edge case).
Each false positive is a full filter re-init (loses NL anchors,
restarts trust scaffold) — the most expensive recovery action in
the system.  Verification at `/tmp/evict-waste/window_rms_attribution.py`.
**Proposed**: Gate the trip on PR-RMS breach AND IF-RMS breach.
The other trip reasons (`ztd_impossible`, `ztd_cycling`,
`anchor_collapse`) read filter state, not residuals — leave alone.
~40 LOC; see `I-162353-main`.

### `SettingSvDropMonitor` — name + architectural seam — Misleading

**Where**: `scripts/peppar_fix/setting_sv_drop_monitor.py`, class
`SettingSvDropMonitor`.
**Claim**: "Setting-SV drop monitor — graceful drop as SVs descend
through the retirement elevation band."  Name implies the monitor's
job is to retire SVs that are descending into multipath-prone low
elevations.
**Actual** — two issues:

  1. **Direction-agnostic.**  The trigger is "elev in 18-30° band
     AND PR breach AND IF breach."  It fires equally on rising,
     descending, or stable SVs in that band.  Empirically (day0502
     night, 11 same-second wasted-evict events) ~45% of triggers
     hit *rising* SVs transiting the band on their way up — the
     opposite of "setting."  The mechanism docstring is honest
     ("removes a member whose observations are becoming too noisy
     to participate in the self-consistency check") but the name
     encourages a wrong mental model.

  2. **Per-SV-absolute vs per-SV-vs-cohort architectural seam.**
     The trigger compares this SV's own rolling residual mean to
     a fixed elev-weighted threshold (3 m PR base, 50 mm IF base).
     Sibling per-SV monitors `IF_STEP` and `GF_STEP` instead use
     cohort-relative thresholds (this SV's residual minus the
     cohort median, fixed-tier ±50 mm/100 mm).  The cohort-relative
     family auto-tunes for the antenna's specific multipath-vs-elev
     curve; the absolute family does not.  At noisy sites the
     absolute threshold over-fires on the whole fix set; at quiet
     sites it under-fires.  The fixed elev-weighted shape (1/sin)
     captures the *form* of the multipath curve but not the *level*
     for a given antenna mount.

**Why it matters**: today's empirical case (8 of 12 overnight
window_rms trips at 80mm-IF, 11/11 same-second wasted-evicts on
rising/stable SVs in band) shows the absolute-threshold family
generates wasted action — wasted re-inits and wasted evictions
where the cohort-relative family wouldn't have fired.  Bob's
mental model: "I only care if observations are noisy or clean.
I want clean ones in the filter regardless of elevation.  Elevation
is an *explanation* for why noise might be there, not the metric
I trip on."

**Proposed**: rename + cohort-relative redesign.  Candidates:
  - `LowElevNoiseDropMonitor` (acknowledges the band gate stays
    but drops "setting").
  - `BandNoiseDropMonitor` (Bob's framing — band-restricted noise
    filter for NL members).
  - `CohortNoiseDropMonitor` (architectural rename if we go full
    cohort-relative and drop the band gate).

The third is the deepest redesign: replace the elev-band gate AND
the absolute threshold with "drop if rolling-mean PR AND IF
residuals are > K × cohort median for sustained N epochs."
Elevation drops out of the trigger entirely — it remains only as
an explanation of why some SVs sit higher in the cohort.  Risks
worth designing through: small fix sets (cohort median jitter at
n_cohort < 4), correlated multipath (whole-set ride-through),
catastrophic-event detection (kept by the existing whole-set
window_rms alarm).

**Notes**: tonight's IF-gate (I-161514, commit 3bba60e) is the
narrow scope-fix on the existing absolute-threshold path; the
cohort-relative redesign is a separate, larger architectural
decision tracked here for now.  See `docs/sv-lifecycle-and-pfr-split.md`
for the design intent ("noisy-member removal, before degraded
input pulls the set off").  See empirical evidence in
`/tmp/evict-waste-night/elev_trajectory.py` (post-hoc tool that
classifies each event as EXPECTED / UNEXPECTED-rising /
UNEXPECTED-stable / AMBIGUOUS).

### `FalseFixMonitor` — Dangerous (already self-flagged)

**Where**: `scripts/peppar_fix/false_fix_monitor.py:166`
(method `ingest`).
**Claim**: "False fix" — the name implies detection of wrong NL
integer fixes.
**Actual**: Trigger reads PR residuals only.  Cannot reliably
distinguish wrong-integer fixes from PR multipath.  The codebase
already knows this — every `[FALSE_FIX]` log line carries the
literal qualifier `[observe-only — IF step is canonical demoter]`.
The monitor is neutralized but still emits.
**Why it matters**: 1675 `[FALSE_FIX]` log entries on TimeHat
day0501 vs zero real evictions.  That's the steady-state cost of
the wrong-domain trigger — log noise, no signal.
**Proposed**: Two paths, pick one.  (a) Gate trigger on IF
residual; promote from observe-only to canonical demoter.
(b) Demote to a diagnostic log at lower verbosity, keep IF_STEP as
the canonical demoter and rename the class `PrResidualOutlier`
which is what it actually computes.
**Notes**: Path (b) is cheaper but admits the misnomer rather
than fixing it.  Path (a) puts a second carrier-domain demoter
in the system alongside IF_STEP — could be redundant or could be
useful belt-and-suspenders.  Defer until after the
`SettingSvDropMonitor` and `FixSetIntegrityMonitor.window_rms`
fixes ship and we measure the residual log noise.

## 2026-05-09

### `freq_offset_ppb` — Dangerous (fixed)

**Where**: `tools/calibrate_do.py:96` (function
`measure_frequency_offset`).
**Claim**: a frequency offset in ppb.
**Actual**: returned the fractional **period** offset, i.e.
`(T_actual - T_nominal) / T_nominal`.  These differ by a sign
(δν/ν = −δT/T), so the value carried the opposite sign of the
name's implied convention.
**Why it matters**: every `dac_ppb_per_code` derived from a
calibration sweep using this script came out sign-flipped.
On clkPoC3's IsoTemp OCXO131-100 bring-up 2026-05-08 this
saved `−0.022869` for an OCXO with a datasheet positive Vctrl
slope; the engine then commanded the loop into the wrong
direction and rail-saturated at +749 ppb adj while chA-chB
diverged at +1100 ppb/s.  Reference test against PiFace's
already-converged loop with hand-set `+0.0361` (which never
went through this script) made the sign error decisive.
**Proposed**: minimal fix landed today — negate the expression
and add a citation comment pointing at this entry.  The
variable name stays as `freq_offset_ppb` because that's now
what the function actually returns.
**Notes**: filed alongside dayplan item queueing the deeper
follow-up — audit other callers of `measure_frequency_offset`'s
return value, decide whether the function itself should be
renamed for clarity vs the period-domain `mean_interval_ps`
it also returns, and consider whether `dac_ppb_per_code`
should be renamed in `DacActuator` to flag that it's the
SYSTEM response (can be ±) not the OCXO intrinsic slope
(see open dayplan item I-010246-main).

## Code-quality issues found alongside (not misnomers)

### `gmf._coeff_sum` — dead code

**Where**: `scripts/regression/gmf.py:218`
**Issue**: Function defined but never called.  The
spherical-harmonic sums it would compute are inlined in
`gmf_at` and `GMFProvider.__init__`.  Author's leftover —
remove next time the file is touched.

## 2026-05-29 — qFromCharPerActuator audit

### `"adjfine"` characterization source key — Misleading

**Where**: `scripts/peppar_fix/do_state.py` (`derive_do_process_noise`,
the `sources.get("adjfine")` read for `sigma_do_freq`); synthetic
test fixtures in `test_do_process_noise.py`.
**Claim**: `adjfine` is the Linux PTP `clock_adjtime` fine-frequency
adjustment — a **PHC-specific** mechanism.  As a characterization
source key it implies "the PHC adjfine command stream".
**Actual**: the concept it stands for is actuator-agnostic — the
frequency-steering command (ppb) applied to the DO, whether by PHC
adjfine, DAC code, or ClockMatrix FCW.  The abstraction already
exists: `FrequencyActuator.adjust_frequency_ppb(ppb)`
(`phc_actuator.py` / `dac_actuator.py` / `clockmatrix_actuator.py`).
Naming the source after one hardware mechanism leaks PHC details into
a level that should be hardware-neutral.
**Why it matters**: on DAC/ClockMatrix hosts (most DOs now), there is
no "adjfine"; a reader expecting that key on those hosts is confused,
and a future characterization writer would have to choose between an
honest key and the legacy one.  (Aggravating: nothing currently
*writes* an `"adjfine"` source — freerun chars write
`"DO PPS (chA vs TICC Rb)"` — so the read path is also vestigial.)
**Proposed**: `"freq_command"` (Bob, 2026-05-29) — the commanded
frequency offset, ppb, hardware-neutral.  Reserve "adjfine" strictly
for PHC contexts.
**Notes**: opportunistically adopted in `do_state.py` on the
qFromCharPerActuator branch — `derive_do_process_noise` now reads
`"freq_command"` as primary, `"adjfine"` as a back-compat alias.  When
a characterization tool starts writing the actuator-command noise, it
should use `"freq_command"`.

### characterization `"sources"` dict — Misleading

**Where**: top-level key in the DO characterization JSON.  Written by
`do_freerun_char.py:403`; read by `do_state.py:291`,
`peppar_fix_engine.py:5810`, `ocxo_trusted_gate.py:157`,
`validate_ocxo_gate_phase4.py:106`, `analyze_discipline_noise.py:234`.
**Claim**: "sources" — reads as source-vs-sink (a source of commands /
data flow).
**Actual**: it's a dict of *characterized signals* — each entry is a
recorded signal plus its noise statistics (ASD/TDEV/ADEV).  "Source"
here means only "source of characterization samples."  Worse, one
entry (`freq_command`/`adjfine`) is a recording of a signal that flows
to an actuator **sink**, so a sink-bound signal lives under "sources"
— Bob (2026-05-29): "I think of it as a sink... so I still don't
understand what it means to 'get' adjfine."
**Why it matters**: invites the reader to interpret control-loop
direction where none is meant; obscures that these are just measured
signals.
**Proposed**: `"signals"` (or `"characterized_signals"`).
**Notes**: DEFERRED — larger blast radius than an opportunistic fix
(1 writer + 5 readers + persisted state JSONs on lab hosts).  Do it
when next touching `do_freerun_char.py`'s writer, with a back-compat
read alias (`char.get("signals") or char.get("sources")`), not as a
standalone rename.
## Pass — 2026-05-30

Found during the clkPoC3 4-exit-5 overnight investigation
(`day0529-gapfix-clkpoc3`).

### ~~`skip_stats["obs_deferred_stalls"]` — Misleading~~

**FIXED** 2026-05-30 — renamed to `obs_uncorrelated_alarms` in PR
#101 (`534ee5f`), opportunistically alongside the CPU-stall
observability counters in the same `skip_stats` dict.  Entry kept
(struck through) per the audit-history rule in the file footer.


**Where**: `scripts/peppar_fix_engine.py:4584` (init in the
`skip_stats` dict) and `:4719` (increment).  Surfaced in the
periodic `Skip stats` INFO log.
**Claim**: A count of observation-pipeline stalls deferred for
re-processing.  Reader naturally parses it as "how many stalls
has this run seen."
**Actual**: A count of **alarm fires** for the specific
condition "obs queue has events but none can be correlated to
PPS this loop pass, AND `args.obs_idle_timeout_s` has elapsed
since the last usable obs, AND the alarm isn't already armed."
One-shot per episode — the `deferred_alarm` boolean blocks
further increments until a successful obs resets it at the
top of the next correlatable iteration.  So `N` over a run
means N **distinct correlation-gate alarm episodes**, not N
stalls and not N seconds of stall.
**Why it matters**: On the clkPoC3 overnight (2026-05-30), the
engine experienced multiple 30+ s main-loop freezes (16
`Gap dt=` events absorbed by the gap-recovery path plus 4
exit-5 cascades), and this counter showed `0` across every
Skip-stats sample — leading to "wait, are we even counting
stalls?" before realising the counter measures one specific
correlation-gate alarm, not CPU starvation or general stalls.
The log message at the same site is accurate
(`reason=obs_received_but_deferred`); only the dict key
oversells.
**Proposed**: `obs_uncorrelated_alarms` — concise, keeps the
`obs_` prefix for parity with sibling counters, and the
`_alarms` suffix correctly conveys one-shot-per-episode (not
per epoch).  Alternative: `obs_correlation_stall_alarms` if
explicitness is preferred.
**Notes**: Two occurrences, both in the same function in
`peppar_fix_engine.py`.  Rename deferred to the in-flight
CPU-stall observability work — that PR adds new counters to
this same `skip_stats` dict and is the natural opportunistic
touch.

## 2026-05-31

### `HoldoverState.holdover_entry_mono` — Dangerous

**Where**: `scripts/peppar_fix/holdover_wiring.py:34` (class
`HoldoverState`); read sites at `holdover_wiring.py:123` and
`peppar_fix_engine.py:8444` (after the binaryLayerHoldoverCheck
fix, commit `f384b9f`).
**Claim**: A timestamp — "the monotonic time we entered
holdover."  The type annotation is `float | None`.
**Actual**: **Dual-purpose** — both a timestamp AND the canonical
"are we in holdover right now?" sentinel.  `holdover_entry_mono is
None` means *not* in holdover; non-`None` means in holdover and
the value tells you for how long.  There is no separate
`in_holdover` property or method on the class.
**Why it matters**: PR #107 (disciplineModeFsm binary layer)
wrote `_ho.in_holdover()` at engine line 8444 — a method that
does not exist on `HoldoverState`.  It looked obviously correct
to the reviewer (and to me on the disciplineModeFsmCtxThreading
hotfix that first brought the line into scope), because *of
course* a class named `HoldoverState` would expose "are we in
holdover?" as a predicate.  It doesn't.  The bug was latent
across the entire moonshot stack and only surfaced when
`--gross-fault-reset` was combined with `--servo-input tdcp`
(the only path that creates a non-`None` `_ho` *and* runs
through the binary-layer code), at which point the engine
`AttributeError`-crashed on the first servo epoch (lab repro
2026-05-31).
**Proposed**: Add an explicit `@property def in_holdover(self)
-> bool: return self.holdover_entry_mono is not None` to the
class.  Leaves the timestamp field for the "how long?" callers
(`holdover_wiring.py:124`).  Optionally rename the field to
`_holdover_entry_mono` to signal "internal — prefer the
property" — but only worth doing if you're already in the file.
The property is additive; existing callers continue to work.
**Notes**: This is the third PR #107 latent bug landed in
2026-05 that turned on a flag-combination not exercised in the
original PR's tests.  Sequence:
`disciplineModeFsmCtxThreading` (commit `24fac8d` — #107 forgot
to thread `_convergence` + `_binary_layer` through `servo_ctx`),
`holdoverResetKwarg` (PR #115 — #107 changed `DOFreqEst.reset`
to keyword-only but didn't update the older call site), this
one.  An engine-level integration test that exercises all four
discipline flags together would catch the *next* one of these
— flagged on multiple PR reviews and the right longer-term
ask.

## Adding to this list

When you find another candidate while doing other work, add
an entry under a new dated section.  Don't sort or re-organise;
chronological accumulation is fine.  When the underlying
identifier gets renamed, strike through the entry rather than
deleting (so we keep the audit history visible).
