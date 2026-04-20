# Per-SV Lifecycle, the Fix Set, and the PFR Split

*Design doc 2026-04-19, rewritten 2026-04-20 with final terminology.
Introduces a per-SV state machine, a named three-way partition of the
"fix set", and replaces the old monolithic PFR L1/L2/L3 cascade with
three stateless per-eval monitors.*

## What this replaces

- **`PostFixResidualMonitor`** — single class that fired L1 surgical,
  L2 soft-unfix, or L3 full re-bootstrap from one 3 m threshold.
  Empirically (day0419c/d data) its `_level` state got stuck at 3,
  re-firing L3 on every subsequent misfit and burning ~10 min of
  convergence per re-fire.  See `project_pfr_event_analysis_20260419`.
- **Ambiguity bookkeeping scattered across MW tracker, NL resolver,
  filter, and slip monitor** — each had its own implicit notion of
  "is this SV fixed?" with no shared vocabulary or ordering.

## What this adds

- An explicit **per-SV state machine** with six named states.
- A named three-way **fix-set membership** partition derived from
  those states.
- Three stateless monitors, each with one job.
- Consistent vocabulary: `integer fix` (per-SV) vs `position solution`
  (the whole thing the AntPosEst computes) — never overloaded.

## Scope and related state machines

Per-SV ambiguity state (`SvAmbState`) is **orthogonal** to the
position-solution state (`AntPosEstState`) documented in
`docs/state-machine-refactor-plan.md`.  Don't conflate them:

| Scope | Name | What it answers |
|---|---|---|
| Per-SV | `SvAmbState` | Does this satellite's integer ambiguity contribute to the fix set? |
| Position solution | `AntPosEstState` | Is the antenna position the AntPosEst instance has computed trustworthy? |

They interact at exactly one point: the position solution transitions
CONVERGING → RESOLVED when the **long-term member count** in the
fix set reaches a threshold.  Routine sky motion (short-term fixes
arriving, long-term fixes being dropped as SVs set) does not flip
solution state.

The word **host** deliberately does not appear in this model.  Each
AntPosEst instance owns one fix set and one solution.  A deployment
with multiple antennas has multiple AntPosEst instances, each with
independent fix sets and solutions — no shared-state confusion.

## The six per-SV states

```
                (receiver loses tracking → record forgotten)
                              ▲
(receiver                     │
 acquires)   TRACKING ── admit ──► FLOAT ── WL fix ──► WL_FIXED ── NL fix ──► NL_SHORT_FIXED ── Δaz ≥ 15° ──► NL_LONG_FIXED
                                    ▲                    │                        │   ▲                          │
                                    │                    │                        │   │                          │
                                    │     false-fix rejection (monitor)           │   │                          │
                                    ◄────────────────────────────────────────────┘   │                          │
                                    │                                                 │                          │
                                    │     setting-SV drop (monitor)                   │                          │
                                    ◄─────────────────────────────────────────────────┴──────────────────────────┘
                                    │
                                    │     cycle slip (HIGH — ≥2 detectors or locktime=0)
                                    ▼
                              SQUELCHED ── cooldown expires ──► FLOAT
                                    ▲
                                    │     cycle slip (LOW — single detector, outlier-ish)
                         (any integer state) ──► FLOAT  (keeps WL/MW state for fast re-fix)
```

### State definitions

**TRACKING** — the receiver reports the SV via RXM-RAWX, but we have
not yet decided to process it.  Elevation, health, or constellation
gate has not passed.  Transient — typically lasts one epoch before
admit.

**FLOAT** — admitted into processing.  MW tracker is accumulating
phase history; no integer fix yet.  Pseudorange + float-phase
observations contribute to the filter, but nothing about this SV's
ambiguity is integer-constrained.  Not a member of the fix set.

**WL_FIXED** — Melbourne-Wübbena wide-lane integer fix has converged
(ambiguity `N_WL = N1 − N2` is integer).  NL is still float in the
filter.  Not yet a member of the fix set — waiting for LAMBDA to
accept the narrow-lane integer.

**NL_SHORT_FIXED** — LAMBDA (or rounding) has accepted the narrow-lane
integer.  Both N1 and N2 are implied integer.  Short-term member of
the fix set: contributes its integer to the position solution but
does not count toward the solution-state RESOLVED declaration — the
integer has not yet been validated across geometry change.

**NL_LONG_FIXED** — the integer fix has survived ≥ 15° of satellite
azimuth motion without triggering a false-fix rejection.  Long-term
member of the fix set: counts toward RESOLVED.

**SQUELCHED** — temporarily excluded from integer-fix attempts after
a high-confidence cycle slip.  Cooldown-bound (default 60 epochs),
not permanent.  "Squelched" is radio-receiver terminology and was
chosen over "blacklisted" because the latter connotes permanence;
an SV is squelched until signal quality recovers.

## The self-consistent integer set

The unifying concept behind all six states and three monitors is
**self-consistency of the integer set** — the group of per-SV
integer ambiguities that jointly fit the observed dual-frequency
phase data.  Each component of the design has a specific role
relative to that set:

### Two promoters

- **Short-term promoter** (WL_FIXED → NL_SHORT_FIXED): lives in
  `NarrowLaneResolver.attempt` (LAMBDA + rounding).  Its job is to
  **identify** a candidate self-consistent integer set from the
  float ambiguity covariance.  LAMBDA does a joint-MAP search: it
  picks integers that are mutually consistent with each other, not
  independently per-SV.  The output is internally consistent at
  that epoch's geometry.  Rounding is a per-SV fallback for
  single-SV additions that meet tighter frac/σ gates.
- **Long-term promoter** (NL_SHORT_FIXED → NL_LONG_FIXED): class
  `LongTermPromoter`.  Its job is to **validate** that the
  self-consistency identified by the short-term promoter survives
  ≥ 15° of satellite-azimuth motion without triggering a false-fix
  rejection.  Short-term members that survive this geometric-
  diversity test graduate to long-term.

### What each NL state tells you

- **`NL_SHORT_FIXED` member**: *was* consistent with the integer
  set at promotion time.  Whether it remains consistent as
  geometry evolves is unproven — on probation.
- **`NL_LONG_FIXED` member**: *has proven* consistent across
  temporal + geometric variation.  Drives the position solution's
  RESOLVED declaration.

### Three monitors, three scopes

- **False-fix monitor** — per-SV, operates on short-term members.
  Detects short-term integers that **cannot** be consistent with
  the already-established long-term set.  A short-term member's
  PR residuals growing against the filter state (which the
  long-term members built) is the signature.  Action: transition
  the offender back to FLOAT.  The long-term set continues
  unaffected.
- **Setting-SV drop monitor** — per-SV, operates on all NL members.
  Removes a member whose observations are becoming too noisy to
  *participate* in the self-consistency check, before that degraded
  input can pull the set off.
- **Fix-set integrity alarm** — whole-set.  Detects breakdown of
  self-consistency of the set **itself** — many members failing
  simultaneously.  That signature almost always means the input
  (SSR stream, datum, ephemeris) is broken, not that one SV's
  integer is wrong.  Action: full filter re-init.

Viewed this way, every transition in the state machine is either:
*adding* a candidate to the set (short-term promoter), *validating*
that the set remains self-consistent through geometry change
(long-term promoter), *removing* a single member whose integer
doesn't fit (false-fix monitor or setting-SV drop), or *scrapping
the set entirely* because its input assumptions are broken
(fix-set integrity alarm).

## The fix set

**Fix set** — the collection of SVs whose integer ambiguities contribute
to the current position solution.  A three-way partition:

| State | Fix-set role | Counts toward RESOLVED? |
|---|---|---|
| TRACKING | outside | no |
| FLOAT | outside (contributes pseudorange + float phase only) | no |
| WL_FIXED | outside (still float in the IF ambiguity) | no |
| **NL_SHORT_FIXED** | **short-term member** | **no** |
| **NL_LONG_FIXED** | **long-term member** | **yes** |
| SQUELCHED | outside | no |

**Short-term members** have an integer fix that has not yet
demonstrated consistency with long-term members' fixes as geometry
evolved.  They still contribute their integer to the solution — we
use them.  We just don't declare RESOLVED based on their count.

**Long-term members** are integer fixes that have survived the
geometry-change test.  They drive the RESOLVED transition.

## Events (transitions)

| Event | From | To | Trigger |
|---|---|---|---|
| observe | (nil) | TRACKING | receiver reports RXM-RAWX for this SV |
| admit | TRACKING | FLOAT | elevation + health + constellation gate passes |
| WL fix | FLOAT | WL_FIXED | MW tracker converges |
| NL fix | WL_FIXED | NL_SHORT_FIXED | LAMBDA accepts integer (or rounding-path success) |
| **promote** | NL_SHORT_FIXED | NL_LONG_FIXED | Δaz ≥ 15° accumulated with clean false-fix window |
| **false-fix rejection** | NL_SHORT_FIXED | FLOAT | false-fix monitor detects wrong-integer signature |
| **setting-SV drop** | NL_SHORT_FIXED / NL_LONG_FIXED | FLOAT | setting-SV drop monitor (elev-weighted residual or below drop mask) |
| slip (LOW) | any integer state | FLOAT | cycle-slip monitor, single-detector evidence |
| slip (HIGH) | any processing state | SQUELCHED | cycle-slip monitor, ≥2 detectors or locktime=0 |
| cooldown expire | SQUELCHED | FLOAT | elapsed cooldown time |
| forget | any | (record removed) | receiver stops tracking |

Transitions not listed are illegal and raise `InvalidTransition`.
The tracker logs every legal transition as one
`[SV_STATE] <sv>: <from> → <to> (epoch=N, elev=X°, reason=...)` line.

## The three monitors

All three are **stateless between evaluations**.  Each eval looks at
current tracker/residual data and decides independently.  None carry
an escalation ladder or persistent "last action" state — that's the
explicit fix for the old PostFixResidualMonitor cascade bug.

### False-fix monitor (`FalseFixMonitor`)

**What**: per-SV stateless monitor that watches short-term members
for wrong-integer signature (residuals grow as geometry shifts after
fix).

**Threshold**: elev-weighted `base * (sin(45°) / sin(elev))` with
45° clamp.  Base default 2.0 m at zenith — tighter than the old
monolithic 3 m gate because newly-fixed SVs should be nearly perfect
in residuals if the integer is correct.

**Action**: transition the SV to FLOAT (false-fix rejection).  The
filter unfixes the NL integer, inflates the ambiguity covariance,
and squelches the SV briefly so the next attempt has new evidence.

**Scope**: NL_SHORT_FIXED only.  Long-term fixes by definition have
survived geometry-change validation; the setting-SV drop is the
right gate for them.

### Setting-SV drop monitor (`SettingSvDropMonitor`)

**What**: per-SV stateless monitor that watches for elevation-driven
quality loss on any NL member of the fix set.

**Triggers** (either fires):
- **Elev-weighted PR residual exceeds threshold.**  Base 3.0 m at
  zenith (looser than the false-fix monitor — the SV's integer is
  still likely correct, just noisier).
- **Elev below absolute drop mask.**  Default 18°.  Fires regardless
  of residual quality — sub-mask integers aren't worth the risk.

**Action**: transition the SV to FLOAT (setting-SV drop).  The
filter unfixes the NL integer with gentle covariance growth; MW
state is preserved so the SV can be re-acquired cleanly if it
rises again in a different arc.

**Scope**: both NL_SHORT_FIXED and NL_LONG_FIXED.  Setting is setting.

### Fix-set integrity alarm (`FixSetIntegrityAlarm`)

**What**: fix-set-wide stateless alarm that watches for systemic
failures no single-SV monitor can attribute to one satellite —
genuine catastrophes like a bad SSR correction batch, clock-datum
shift, or reference-frame change.

**Threshold**: window-averaged PR RMS across all NL members
> 5.0 m sustained.

**Suppression**: if any SV transitioned to FLOAT within the
suppression window (default 60 epochs), the alarm stays silent —
the per-SV monitors are already handling it.  This prevents
double-counting routine churn as systemic failure.

**Action**: full PPPFilter re-init at `known_ecef`.  All NL fixes
released, MW state cleared, tracker flattened to FLOAT.  Solution
state returns to CONVERGING.

**Expected rate**: < 1/day in steady state.  If firing more often,
something is broken at the correction-source level — dig into SSR
feed or datum, not into AR tuning.

## Validation targets

A 24-hour steady-state run should show:

- Position solution stays RESOLVED for ≥ 95% of the run (excluding
  initial convergence).
- False-fix rejection rate < 1/hr in stable conditions.
- Setting-SV drops at approximately the natural sky-motion rate
  (~5/hr per host for GAL+BDS).
- Fix-set integrity alarm fires < 1/day.

Compare against pre-redesign day0419c/d: ~2 PFR L3 events/hr,
~35% time RESOLVED.  Day0419e (post-Beads-1+2+3) achieved 0–1
alarm events per host over 9 hours — design target met structurally;
remaining work is the wrong-integer *source* identification, not
the cascade bug.

## Known open problem: persistent-trouble SVs

Day0419h overnight data (clkPoC3 ~9 h) shows a fraction of SVs
cycling FLOAT ↔ NL_SHORT_FIXED ↔ SQUELCHED repeatedly without ever
contributing as long-term members:

- C48: 7 SQUELCHED entries + 10 false-fix rejections
- E32: 6 SQUELCHED + 16 false-fix rejections
- E08: 5 SQUELCHED + 9 false-fix rejections

These SVs appear to have a persistent per-SV bias (multipath,
systematic clock offset, or undocumented AC issue) that produces
internally-consistent wrong integers on every fix attempt.  Today's
60-epoch squelch cooldown is too short to break the loop — the SV
re-fixes the same wrong integer, false-fix monitor catches it,
squelch again, repeat.

**Proposed follow-up** (separate from this design): count false-fix
rejections per SV, extend squelch duration exponentially, or drop
the SV from AR attempts entirely after N rejections within a
rolling window.  Not in scope of this state machine.

## Implementation status

- **Beads 1 + 2 + 3** (per-SV state machine, `FalseFixMonitor`,
  `SettingSvDropMonitor`, `FixSetIntegrityAlarm`, elev-weighted
  thresholds): merged as PR #6 (commit `040c1b0`), renamed for
  clarity 2026-04-20.
- **Bead 4** (`LongTermPromoter`: Δaz-based NL_SHORT_FIXED →
  NL_LONG_FIXED promotion): merged as PR #8 (commit `7ca17c9`).
- **LAMBDA tuning** (P_bootstrap 0.999 → 0.97, PAR min_fixed 4 → 3):
  commits `d08e956` and `0395dff`.

All code lives in `scripts/peppar_fix/`:

| Module | Class | Role |
|---|---|---|
| `sv_state.py` | `SvAmbState`, `SvStateTracker` | state enum + per-SV records |
| `false_fix_monitor.py` | `FalseFixMonitor` | wrong-integer detection on short-term |
| `setting_sv_drop_monitor.py` | `SettingSvDropMonitor` | graceful drop on elev/resid |
| `fix_set_integrity_alarm.py` | `FixSetIntegrityAlarm` | fix-set-wide systemic alarm |
| `long_term_promoter.py` | `LongTermPromoter` | Δaz promotion short→long |

49 unit tests cover every legal edge, one illegal edge per
originating state, the elev-weighting formula, each monitor's
eligibility and threshold behavior, and the promoter's Δaz
accumulator + clean-window logic.  All pass.

## What's deferred

- **Host TOML parameters** for the three thresholds and Δaz
  promotion threshold.  Sensible defaults ship; TOML wiring later.
- **`FalseFixMonitor` extension to long-term members.**  Today only
  short-term; arguable but conservative.  Could add if long-term
  drift is observed.
- **Persistent-trouble SV handling** (see above).
