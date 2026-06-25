# Filter test rigs — simulators vs replay harnesses

*Design doc, 2026-06-25.  Names and defines the test rigs we use to
diagnose PePPAR Fix's estimators.  We have one today (for the time
filter); this doc proposes two new ones for the position filter and,
first, fixes the vocabulary so the three don't get conflated.*

The motivating problem is the position filter's **(pos, ZTD, clk)
null-mode drift** (see [position-drift-investigation-2026-06.md](position-drift-investigation-2026-06.md)):
uncertainty that should resolve into integer ambiguities instead gets
"shoved into ZTD," and we have never had a rig that lets us watch that
happen against a known answer.  The two new rigs attack it from
opposite directions.


## 1. Terminology (read this first)

There are **two paradigms**, and the distinction is load-bearing:

- **Simulator** — *synthetic, truth-down.*  A constructed ground-truth
  state emits measurements through a forward model; the **real filter**
  runs against them in closed loop.  Truth is **exact and dialable**
  (you set it, and you can deliberately excite a failure mode); realism
  is bounded by the forward model — it can only surface effects you
  modeled.

- **Replay harness** — *recorded, reality-up.*  **Real recorded inputs**
  are replayed deterministically through the real engine; the answer key
  comes from **independent post-processing + survey**.  Realism is total
  (real multipath, real correction errors, real receiver quirks); truth
  is **approximate and fixed** to the one realization the day gave you.

**Naming-honesty rule:** a replay harness does *not* simulate — it
re-runs reality.  Calling it a "sim" invites exactly the class of
misnomer [misnomers.md](misnomers.md) exists to prevent.  So, in this
codebase:

> **Simulators synthesize the world from a truth you set.
> Replay harnesses re-run the world you recorded against a truth you measured.**

### The three rigs

| Rig | Paradigm | Target filter | Status |
|---|---|---|---|
| **`servo_sim`** | synthetic simulator | time / servo (`DOFreqEst`) | exists |
| **`pos_sim`** | synthetic simulator | position (`FixedPosFilter` / `PPPFilter`) | **new (§3)** |
| **`pos_replay`** | replay harness | position (and the whole pipeline) | **new (§4)** |

`servo_sim` and `pos_sim` are **siblings** — same paradigm, different
filter.  `pos_replay` is an **evolution of the existing published-data
regression harness** ([regression-harness-plan.md](regression-harness-plan.md),
`scripts/regression/run_regression.py`), which already replays *published*
RINEX against *published* ITRF truth; `pos_replay` replays *our own*
multi-stream capture against *our own* cross-checked truth.

### Artifact vocabulary (for `pos_replay`)

- **reference capture** — one bundle: *{logged inputs}* (24 h of raw
  observations, all correction streams, METAR/surface pressure, every
  filter input, each stamped against `CLOCK_MONOTONIC`) **+** *{truth}*
  (position, ZTD(t) with CI, surveyed ARP).
- **case library** — a curated *set* of reference captures spanning
  regimes: a calm day, an active day, a known "GPS+GAL fails" day.  One
  golden day is not enough — the failures we care about are episodic.
- **product-matched replay** — replaying with the same (final) products
  the truth used, so a discrepancy is attributable to the *filter*, not
  to the real-time-vs-final *correction-quality* gap.


## 2. What we have now — `servo_sim` (time filter)

`scripts/peppar_fix/servo_sim.py` (design:
[closed-loop-servo-sim.md](closed-loop-servo-sim.md)) is a synthetic
closed-loop simulator for the **time/servo filter**.  A ground-truth
oscillator state (φ_do, f_do, φ_rx) emits a TICC measurement each epoch
that **responds to the servo's commanded `adjfine`** — that feedback is
what lets it reproduce acquisition, gate over-rejection, ringing, and
coast dynamics, which a replay cannot.  It drives the **real**
`DOFreqEst` (real EKF, six arms, χ² gate) and `OcxoTrustedGate`; the
only new code is the plant (truth evolution + measurement emission) and
the loop.

Its plant is *small* — three scalars and one measurement equation
(`ticc_diff = -φ_do - qerr(φ_rx) + noise`).  That smallness is why a
synthetic sim was tractable for the time filter and is the first thing
that changes for position.


## 3. New rig #1 — `pos_sim` (synthetic position simulator)

**Definition.**  The position-filter sibling of `servo_sim`: a
constructed truth (ARP, troposphere, receiver clock, integer
ambiguities, satellite geometry) emits synthetic per-SV code/carrier
observations through the forward model, and the **real**
`FixedPosFilter`/`PPPFilter` runs against them.  Truth is exact;
the failure mode is dialable.

**Faithfulness principle (same as `servo_sim`).**  `solve_ppp` already
computes predicted observables `h(x)` from the state (geometry,
tropo mapping, clock, SSR).  `pos_sim` runs that same model **forward
from a known truth** to *emit* `z = h(truth) + noise`, exactly as
`servo_sim` emits `ticc_diff` from the equation `DOFreqEst` inverts.
Reuse the filter's own model; only the plant + loop are new.

**The plant (what's bigger than `servo_sim`).**  A truth ARP (ECEF); a
satellite geometry with time-varying az/el (real broadcast ephemeris is
fine — only the *geometry* need be real); per-SV line-of-sight vectors;
a truth ZTD(t) + mapping; a truth receiver clock; per-SV integer
ambiguities; and the correction model (SSR/code-bias/phase-bias).  The
measurement is per-SV code + carrier on the IF/WL/NL combinations the
filter consumes.

**What it's for.**

- **Prove the null exists and characterize it.**  Set a known
  (pos, ZTD, clk, N); confirm the filter holds truth, then *ramp truth
  ZTD or clock* and watch whether the misfit relocates ("misfit
  conservation") rather than resolves — the exact mechanism behind
  "uncertainty shoved into ZTD," observed in a vacuum.
- **A/B filter knobs against an exact truth** (Q_pos, ZTD prior,
  `--clock-model wno`, AR on/off, single- vs multi-constellation
  geometry) with **no real-world confounds** — no multipath, no
  AC-datum offset, no orbit/clock error.  This is precisely what
  real data *cannot* isolate.
- **Reproduce "Galileo converges, GPS+GAL doesn't" structurally** — set
  up each geometry with a clean ISB and see whether it's an
  observability/rank issue independent of any one day's data.

**What it can't do.**  It is only as real as the forward model.  Its
"truth ZTD" is whatever you injected, so it can validate that the filter
*handles the tropo model correctly* but **cannot catch a wrong tropo
model** (mapping function, a-priori) — for that you need a real sky and
an external ZTD (→ `pos_replay`).  It will not surface real multipath,
real receiver firmware quirks, or real correction-stream errors.

**Sketch.**  `scripts/peppar_fix/pos_sim.py`, mirroring `servo_sim.py`:
a `Plant` (truth state + `emit_observations(epoch)`), a loop wiring
plant → real `FixedPosFilter` → (no actuator; position is open-loop in
the estimator sense, so "closed loop" here means the truth evolves and
the filter tracks), and an assertion/score layer (state error vs truth,
per-domain residual split, P-matrix sanity).  Unit-testable; fast;
sweepable across seeds and geometries.


## 4. New rig #2 — `pos_replay` (captured-replay harness with reference truth)

**Definition.**  Replays a **reference capture** (real logged inputs)
deterministically through the **real engine** and scores the output
against an **independently post-processed answer key**.  Reality-up:
total realism, approximate-but-external truth.

**The reference capture.**  A 24 h run logging *everything* the position
filter sees — raw observations, all correction streams, METAR/surface
pressure, and the assembled filter inputs — each stamped against
`CLOCK_MONOTONIC` (the only shared timescale; see
[stream-timescale-correlation.md](stream-timescale-correlation.md)).
The raw observations are then sent through independent post-processors
to build the truth:

- **PRIDE-PPP-AR** (local) — epoch-wise kinematic reference time series;
  same *class* of products you can also feed the filter for a
  product-matched run.
- **OPUS-Static** (GPS-only) — a static position anchor and a *GPS-only
  control* for the constellation question.
- **NRCan CSRS-PPP** (multi-GNSS) — position time series **and the
  crown jewel: ZTD(t) with a confidence interval**, the external
  observable `pos_sim` can't honestly provide.
- **Surveyed ARP** (`timelab/antennas.json`, OPUS multi-day, σ≈12 mm) —
  the one genuinely sub-cm position truth we already hold.

**What it's for.**

- **Localize the real-world gap.**  When our real-time output disagrees
  with truth, *product-matched replay* separates "our filter" from "our
  real-time corrections": feed final products → if it converges, it was
  products; if it still drifts into ZTD, it's the filter/observability.
- **Test ZTD allocation against the real atmosphere.**  Compare our
  ZTD(t) to NRCan's external ZTD(t) — does our estimate track the true
  sky, or absorb misfit that should have been an integer?  This is the
  test that motivated the whole exercise, and it needs a *real* ZTD
  truth.
- **Ablate the actual failure.**  It's our data, so we can drop a
  satellite, zero the GPS–GAL ISB, swap the correction AC, force AR, or
  lengthen the arc — to find *why* GPS+GAL fails on a day it fails.
- **Regression.**  Bit-reproducible replay = a guard that an
  architectural change fixes the drift instead of relocating it.

**What it can't do / design traps** (the hard-won caveats — encode them
in the harness, don't rediscover them):

1. **Truth is a correlated cousin, not ground truth.**  PRIDE/OPUS/NRCan
   are PPP engines eating the *same* RINEX with *similar* models and
   shared IGS products; their CIs are *formal*, not absolute accuracy.
   Treat as a strong reference with honest error bars, cross-checked,
   not as an oracle.  (We already saw PRIDE-float carry a ~1.6–2 m datum
   offset in one case, and the drift residual be "products-limited, not
   filter-limited.")
2. **Products mismatch masquerades as filter error** — final (truth) vs
   real-time SSR (us) differ at cm–dm.  Product-matched replay is
   mandatory for a *clean filter* verdict.
3. **ZTD comparison has bookkeeping traps.**  Match ZHD/ZWD split
   (METAR/pressure → ZHD), the mapping function, the **antenna
   reference height**, and accept that a batch-smoothed "hindsight" ZTD
   legitimately *leads* our real-time random-walk ZTD.  A convention
   mismatch will look like a physics finding.
4. **One realization.**  Failures are episodic → curate a *case
   library*, not a single golden day.
5. **Determinism + tap point.**  Capture raw byte streams + monotonic
   timestamps; *derive* a filter-input record from them.  Replay must be
   single-threaded and free of wall-clock/RNG/threaded-numpy
   nondeterminism or it isn't a regression.
6. **Position truth is largely already known** (surveyed ARP) — the new
   value is ZTD(t), the per-epoch reference, and the ablation surface,
   not "discovering the position."

**Relationship to the existing harness.**  `pos_replay` extends
`scripts/regression/run_regression.py` (published-data replay) with:
our full multi-stream capture, cross-checked + ZTD truth, product-swap
ablation, and a regime case library.  Reuse the runner; add the capture
manifest, the truth-ingest, and the comparison/ablation layers.


## 5. When to reach for which

| Question | Rig |
|---|---|
| Is the filter *algebraically* correct? Does Q/prior tuning behave? | `pos_sim` |
| Does the (pos,ZTD,clk) null exist, and how does the filter route misfit into ZTD when geometry is weak? | `pos_sim` (mechanism, in a vacuum) |
| Does our ZTD track the *real* atmosphere, or absorb misfit it shouldn't? | `pos_replay` (external NRCan ZTD) |
| Is a real-world gap our *filter* or our *corrections*? | `pos_replay`, product-matched |
| Why does GPS+GAL fail on the day it fails? | `pos_replay` + ablation |
| Acquisition/ringing/coast of the *time* servo | `servo_sim` |

The two new rigs are **complementary**: `pos_sim` proves the null
exists and shows the filter routing misfit into ZTD *in a vacuum*;
`pos_replay` confirms it *on the real sky* and localizes the cause.
`pos_sim`'s injected ZTD truth is circular for catching a wrong tropo
*model*; `pos_replay`'s external ZTD is not.


## 6. Sequencing & scope

1. **`pos_sim` first** — cheaper (~1–2 weeks; the plant is bigger than
   `servo_sim`'s but reuses `solve_ppp`'s forward model), exact, and
   sweepable.  It tells us *what to look for* before we invest in
   capture+truth tooling.
2. **`pos_replay` second** — the larger lift (~a month: deterministic
   multi-stream replay + truth ingest + ZTD-convention tooling +
   product-swap ablation).  Design the capture around its
   highest-value, hardest-to-fake input — the external ZTD truth.

**Out of scope / non-goals.**

- Per-commit CI for `pos_replay`: too slow (minutes–hours per case).
  It's a nightly / on-demand diagnostic and a long-regression tier, not
  a unit test.  `pos_sim` *can* run in CI.
- Renaming `servo_sim` — it's established and its name is honest under
  this taxonomy (a synthetic simulator of the servo).
- Replacing the published-data regression — `pos_replay` complements it.

## Open decisions

- Capture tap point: raw byte streams (full fidelity, replays the whole
  pipeline) vs assembled filter-input records (lighter, isolates
  AntPosEst).  Recommendation: capture raw, derive the filter-input
  record.
- ZTD comparison reference: NRCan total ZTD with our ZHD-from-METAR, or
  compare ZWD directly — pin the convention before building the
  comparison.
- `pos_sim` "closed-loop" semantics: the position estimator has no
  actuator, so the only feedback is truth-evolves / filter-tracks.
  Decide whether to also model a slowly-moving truth (kinematic) or hold
  a static ARP and move only ZTD/clk (the null axes).
