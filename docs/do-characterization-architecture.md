# DO Characterization Architecture

One per-DO file.  One physical quantity per section.  No silent
fallbacks.  Schema-validated on read; tools refuse to write the
wrong thing in the wrong place.

The PiFace overnight cascade (I-100945-main, 2026-06-15) was the
proximate trigger: PiFace's σ_DO file mixed receiver-side
measurements with DO-side measurements under a single `sources`
dict, the derivation function picked receiver-side over DO-side,
and the engine ran with `sigma_do_freq = 0.1975 ppb/√s` against
clkPoC3's correct `0.000582 ppb/√s` — a 517× looser Q[3,3] —
which made the actuator wobble until the ResetBudget exhausted.
The root cause was **not** wrong priorities; it was that physically
different quantities were stored under the same field name as if
they were comparable.  This document defines the schema that makes
that mistake unrepresentable.


## Goal

A Disciplined Oscillator (DO) is characterized by a known, fixed
set of physical quantities.  Each quantity must be:

- **Stored in exactly one place** — the per-DO file at
  `state/dos/<do_uid>.toml`.
- **Measured by exactly one tool** — one section, one writer.
- **Refused entry into the wrong section** — schema-validated.
- **Logged with provenance at engine startup** — never "value
  appeared, source unknown."

The engine's quality is bounded by the honesty of these numbers.
PiFace proved that a wrong σ_DO_freq, even by a factor we could
have caught with a sanity check, produces an unstable closed loop.


## Topology — why receiver-side ≠ DO-side

The single most important architectural point.  This is what
should drive every schema decision below.

```
PePPAR-Fix lab topology (PiFace, clkPoC3, MadHat, TimeHat):

  GPS satellites
       │
       ▼
  ┌─ F9P / F9T receiver ────────────┐
  │  rx-clock = internal rx TCXO    │ ← physically separate from
  │                                  │   the DO; the DO is NOT
  │  outputs:                        │   feeding this receiver's
  │   - GNSS PPS  ───────────────────┼──→ TICC chB
  │   - NAV-CLOCK (dt_rx)            │     external reference
  │   - UBX-TIM-TP (qErr)            │
  │   - Carrier-phase observations   │   These all describe the rx
  │   - RAWX → TDCP                  │   TCXO's drift vs GPS.
  └──────────────────────────────────┘   None of them sees the DO.

  ┌─ DO (OCXO / TCXO / Rb / PHC) ─┐
  │  steered by `adjfine` /        │   `adjfine` is the ENGINE's
  │  DAC code / FCW                │   control signal, not a
  │                                │   physical observation.
  │  outputs:                      │
  │   - DO PPS  ───────────────────┼──→ TICC chA
  └────────────────────────────────┘

  Lab Rb (independent of both above)
       │
       └──→ TICC reference clock

  TICC readings:
    chA vs Rb  = DO_motion + tiny_Rb_drift ≈ DO_motion (the
                 cleanest σ_DO source we have)
    chA − chB  = DO_motion − rx_TCXO_motion (acceptable fallback
                 when Rb isn't available; dominated by DO motion)
```

The OCXO and the F9P's internal TCXO are **physically independent
oscillators**.  Receiver-side measurements (`dt_rx`, `Carrier`,
`qErr`) tell us how the rx TCXO is drifting — they say nothing
about the DO.  Control-loop signals (`adjfine`) tell us how hard
the engine is steering — also not the DO.  Only chA vs Rb and
chA − chB point at the DO.

Different topologies are possible (the SparkFun GNSSDO+ feeds its
OCXO back into the receiver as the external reference, so
receiver-side measurements *do* see the DO there).  Those are
**out of scope** — they live as one-of-a-kind systems and are
treated as black boxes from the PePPAR-Fix engine's perspective.


## Code symmetry vs pull symmetry — the DO is an intersection (no magic center code)

A DAC-steered DO is the **intersection of two independently-imperfect
components**, and most of our DAC-control confusion has come from
treating them as one thing:

- the **DAC** — a code→voltage actuator, and
- the **OCXO** — a voltage→frequency oscillator.

Each has its own asymmetries and limits.  Two *separate* properties,
which we must stop conflating:

- **Code symmetry** is a property of the **DAC alone**: it produces
  ~half its full control voltage at the center (midscale) code.  A
  rail-to-rail DAC on a 5 V supply has it; PiFace's 3.3 V AD5693R in
  2× mode does **not** (it clips against VDD before full scale).
- **Pull symmetry** is a property of the **OCXO alone**: it matches
  GNSS at half control voltage, i.e. its frequency pull is symmetric
  around the GNSS-matching control point.  Any OCXO whose EFC curve
  has shifted under temperature does **not** — and that shift is
  normal, not a defect.

**The center code is meaningful *only* under code symmetry.**  The
moment you allow code asymmetry, *there is nothing special about the
center code*.  And even with a perfect DAC, thermal drift moves the
GNSS-matching point — a DO that matches GNSS at code C at 25 °C needs
a different code at 30 °C.

### The PHC analogy (why this is obvious for TimeHat and was hidden for the DAC hosts)

The cleanest way to see it: a DAC-steered DO is exactly like a PHC
steered by `adjfine`.

| PHC (`adjfine`) | DAC + OCXO | meaning |
|---|---|---|
| `adjfine = 0 ppb` | **center code** (`max_code // 2`) | the actuator's *neutral* command — "apply no correction." No relationship to GNSS. |
| the `adjfine` value that locks to GNSS (= −crystal free-run offset) | the **GNSS-matching code** | where the DO's frequency *equals* GNSS (pull = 0). |

Nobody ever treats `adjfine = 0` as "where GNSS lives": to discipline
a PHC you drive `adjfine` to whatever cancels the crystal's
free-running offset, that value is essentially never 0 ppb, and it
**drifts with temperature**.  The integral servo simply seeks it.  A
DAC-steered DO is identical — the **center code is the neutral command
(↔ `adjfine = 0`), and the GNSS-matching code is a measured,
temperature-drifting operating point (↔ the locking `adjfine`)**.  We
must stop anchoring DAC code math, clamps, and bootstrap seeds on the
center code, exactly as we never anchored PHC control on `adjfine = 0`.

If anything the center code is *even less* special than `adjfine = 0`:
`adjfine = 0` is a true zero-correction (the PHC applies no rate offset),
whereas the DAC center is merely an arbitrary midscale *voltage* — it has
no privileged relationship to either GNSS *or* "no correction."  It is
just the middle of the code axis.

This conflation never bit the PHC hosts (TimeHat/MadHat-i226) for two
reasons the DAC lacks: (1) nobody anchors PHC control on a "special"
zero, and (2) a PHC has effectively no hard rails near its operating
point.  A DAC has **both** — center-anchored math *and* hard
`[code_min, code_max]` linear-region rails — so on an asymmetric host
(PiFace) the assumptions break and a center-anchored seed lands on a
rail (the `dacBootstrapSeedRail` / `detectLinearRegionOverClip`
symptoms).

### Correct model — anchor to an edge, not a center

A DAC-DO's steering is fully described by its **linear region** plus
**one point on the line** — never a privileged center:

```
linear region   = { code_min, code_max, slope_ppb_per_code }
anchor point    = the GNSS-matching code  (pull = 0), which may sit
                  ANYWHERE in [code_min, code_max] (or be derived from
                  any measured (code, ppb) pair)
```

Compute code from desired pull anchored to an **edge**, with no center
in the equation:

```
code = code_min + (desired_pull_ppb − pull_at_code_min) / slope
```

Asymmetric pull is fine and expected.  What matters operationally is
**directional headroom** — the distance from the GNSS-matching code to
each rail:

```
# for positive slope (higher code → faster; true for all our DACs):
headroom_fast = (code_max − gnss_matching_code) · slope     # ppb of "speed up" available
headroom_slow = (gnss_matching_code − code_min) · slope     # ppb of "slow down" available
# for negative slope the two swap — what matters is |headroom| toward
# each rail; compute from the signed slope and take the side accordingly.
```

Combined with the OCXO's temperature coefficient (ppb/°C), the headroom
predicts the **ambient-temperature band over which the servo keeps
control** before the DO rails on one side.  This is the DAC-specific
budget a PHC has no analog for.

`center_code` is therefore neither the control anchor **nor** the
startup seed.  It is at most a cosmetic, derivable midscale value; the
control math anchors to `code_min + slope`, and where to *start* is a
separate, runtime question handled below.

### Where to start next time — warm-start seeding is universal, and it is not "parked"

The genuinely useful "where to start the actuator" concept is **runtime
state, not characterization**, and it lives one level above the DAC.

The justification is physical and actuator-agnostic: ambient temperature
at the next startup will be close to the temperature at the last
shutdown, so **the correction (from a nominal 0 ppb) that was needed at
shutdown is a good estimate of the correction needed at startup.**  That
statement is true for *every* DO regardless of actuator — a DAC code, a
PHC `adjfine` value, a ClockMatrix FCW word are all just realizations of
one last-known **frequency correction in ppb**.

The schema already expresses this at the right level, in runtime state
(`<uid>.runtime.toml [operational_state]`):

```
last_known_freq_offset_ppb = +144.65   # universal: the correction itself
last_known_dac_code        = 32768     # DAC-specific realization (convenience)
```

`last_known_freq_offset_ppb` is the portable truth (PHC seeds `adjfine`
from it, ClockMatrix seeds FCW from it, a DAC re-derives a code from it
via the anchor-to-edge formula).  `last_known_dac_code` is a convenience
cache of the same point.  Warm-start seeding should be driven by the
**ppb correction**, clamped to `[code_min, code_max]` for a DAC — never
by a characterization field.

**The danger in "parked".**  `[steering].parked_code` (a *characterization*
field) reads like "where the actuator parks on startup/shutdown," but its
actual role is the *reference code of the linear fit* — the code at which
`intercept_ppb_at_parked` was measured, i.e. just one point on the line.
The name invites exactly the conflation this whole section is purging:
treating a fit anchor as a place to sit, or as a stand-in for the
GNSS-matching point.  (Note the example file even has
`intercept_ppb_at_parked` = `last_known_freq_offset_ppb` = +144.65 — the
two got tangled because the fit was anchored at the then-current
operating code.)  PR2 of `noMagicCenterCode` removes `parked_code` from
the line description (which needs only `{code_min, code_max, slope}` + a
pull-reference such as the GNSS-matching code); the only "where to start"
state is the runtime last-known correction, better named `last_code` /
`shutdown_code` than "parked."

> **Why this matters now**: MadHat / clkPoC3 (5 V DAC + IsoTemp OCXO)
> are near-perfect, so the filter quietly absorbs implicit
> center/symmetry assumptions.  PiFace (3.3 V DAC + CTI OCXO) has no
> code symmetry, so the assumptions break — and the sloppy thinking
> masks PiFace's *real* DO-hardware troubles.  Purging the magic center
> code (tracked in dayplan `noMagicCenterCode`) is the prerequisite to
> cleanly separating thinking artifacts from genuine DO-HW behaviour.
> The `[steering]` schema below still records `parked_code` /
> `intercept_ppb_at_parked`; PR2 of `noMagicCenterCode` reframes those
> to express the linear region + GNSS-matching code directly.


## Schema

Two files per DO, with strict ownership.  The split is structural,
not stylistic — it eliminates a write-race and makes "the engine
never opens the characterization file" a property of the
filesystem, not a convention readers can violate.

| Path | Owner | Lifetime |
|---|---|---|
| `state/dos/<do_uid>.toml` | measurement tools | written once per characterization phase, persists for months |
| `state/dos/<do_uid>.runtime.toml` | engine only | rewritten every save_do_state checkpoint |

The characterization file is hand-commented and sectioned; the
runtime file is engine-rewritten with no comments to preserve.
Splitting them means a per-checkpoint engine write can't corrupt
tool-written comments (`tomli_w` doesn't preserve them) and can't
race with a characterization tool writing the main file.

### `state/dos/<do_uid>.toml` — characterization

```toml
schema_version = "1"

[identity]
do_uid = "ocxo-piface"
model = "Isotemp OCXO131-100"     # manufacturer + model
class = "OCXO"                    # OCXO | TCXO | Rb | PHC
actuator_type = "DAC"             # DAC | PHC_adjfine | ClockMatrix_FCW
dac_bits = 16                     # actuator resolution (omitted for PHC)
nominal_freq_hz = 10_000_000
registered = "2026-05-30T14:40:06Z"
notes = ""

[steering]                        # the actuator gain curve
source = "measured"               # measured | class-default
slope_ppb_per_code = +0.02569     # sign-correct, ppb per actuator unit
intercept_ppb_at_parked = +144.65 # freq offset at parked_code
parked_code = 32768
code_min = 1024                   # linear range floor
code_max = 64512                  # linear range ceiling
asymmetry_factor = 1.0            # 1.0 = symmetric ramp rates
measured_at = "2026-05-30T14:40:06Z"

[freerun_noise]                   # the DO's open-loop noise
source = "measured"               # measured | class-default
measurement_channel = "DO PPS (chA vs TICC Rb)"
                                  # MUST be a DO-pointing channel.
                                  # Allowed (preferred order):
                                  #   1. "DO PPS (chA vs TICC Rb)"  (preferred — Rb is the cleanest reference)
                                  #   2. "DO PPS (chA-chB)"          (acceptable fallback when no Rb available; picks up rx-TCXO motion)
                                  # Rejected at write time:
                                  #   anything receiver-side ("Carrier", "dt_rx", "qErr", ...)
                                  #   anything control-loop ("adjfine", "freq_command", ...)
sigma_do_phase_ns = 0.0425        # ns/√s — DOFreqEst Q[2,2]^0.5
sigma_do_freq_ppb = 0.000382      # ppb/√s — DOFreqEst Q[3,3]^0.5 (from ADEV RWFM tail — see §"Q[3,3] design change")
coast_tdev_ref_ns = 0.0425        # ns at tau=1s — Goldilocks coast-cap anchor
coast_tdev_slope = +0.530         # power-law slope (TDEV(τ) ∝ τ^slope) over the RWFM/flicker
                                  # RISING tail.  MUST be > 0 — a slope ≤ 0 would silently
                                  # disable the Goldilocks τ_opt computation
                                  # (compute_goldilocks_tau treats slope ≤ 0 as degenerate
                                  # and returns max_interval, scheduler quietly off).
                                  # Validator rejects slope ≤ 0 at write time.
                                  # The schema requires the rising tail specifically:
                                  # +0.5 white-FM, +1.0 flicker-FM, +1.5 RWFM.
                                  # do_freerun_char fits this tail explicitly — NOT a
                                  # global log-log LSQ over the whole U-shaped curve.
captured = "2026-05-30T14:40:06Z"
duration_s = 3604

[actuation_noise]                 # the actuator chain's per-write noise
source = "measured"               # measured | class-default
sigma_q_ns = 0.05                 # phase residual per actuation event, in ns.
                                  # Goldilocks formula consumes ns directly.
                                  # NOT stored in ppb — that would require a silent
                                  # σ_q_ns = σ_q_ppb × tau_ref conversion with tau_ref
                                  # pinned at 1 s elsewhere.  Bravo's actuatorNoiseChar
                                  # measurement produces phase residual in ns directly;
                                  # storing in the consumer's unit removes one units bridge.
write_settle_ms = 1
i2c_error_rate_per_million = 0
measured_at = "2026-06-01T00:00:00Z"
# Open question (Bravo's actuatorNoiseChar measurement, in flight on
# clkPoC3 now — HOLD/REWRITE/STEP2/STEP8 × 600s, chA-vs-Rb):
# if σ_q turns out to scale with step size rather than being fixed-
# per-actuation, this section grows to σ_q(δ) — either (intercept,
# slope) or a piecewise table.  Schema stays scalar-only until
# Bravo's measurement closes that question, then revises once.

[aging]                           # long-term drift, manual or refined
drift_ppb_per_year = 0.5
last_cal_date = "2026-05-30"
```

### `state/dos/<do_uid>.runtime.toml` — engine-only

```toml
schema_version = "1"

[operational_state]               # engine writes here every save_do_state
last_known_freq_offset_ppb = +144.65
last_known_dac_code = 32768
last_updated = "2026-06-15T16:00:00Z"
```

PSD curves, ADEV tables, and TDEV tables go in a sidecar
`<do_uid>.noise.toml` — they're 10+ KB and don't need to
round-trip through a human editor.  The engine reads only the
main-file scalars (`sigma_do_phase_ns`, `sigma_do_freq_ppb`,
`coast_tdev_ref_ns`, `coast_tdev_slope`, `sigma_q_ns`); the
sidecar exists for post-hoc analysis, replotting, and as the
durable home of the hour-long freerun capture artifacts.  Tools
that need to re-fit the RWFM tail or replot PSD read the sidecar
directly; the engine does not.

### Schema rules

1. **Every section's `source` field is one of `measured` or
   `class-default`.**  The engine logs the source for every
   numeric value it consumes.  Mixed files (some sections
   measured, others class-default) are legal and common during
   the lifecycle; the operator sees at a glance which is which.
2. **`[freerun_noise].measurement_channel` is enumerated.**  Only
   DO-pointing channels are accepted.  This is what makes the
   PiFace failure mode unrepresentable.
3. **`schema_version` is mandatory and gates the loader.**
   Unrecognized versions refuse to load (no silent forward-compat).
4. **No `sources` dict.**  The flat-bucket-of-measurements pattern
   from the old JSON schema is what conflated physical quantities.
   Each quantity gets its own typed field.
5. **The engine never opens the characterization file.**  This is
   structural, not a convention — the engine's runtime state lives
   in a separate file (`<uid>.runtime.toml`).  The engine reads
   `<uid>.toml` at startup, writes only to `<uid>.runtime.toml`
   afterwards.  Eliminates the write-race against tools and the
   comment-strip problem that `tomli_w` would otherwise create.


## Tools — one per section

Strict 1:1 mapping between sections and tools.

| Section | Tool | What it does |
|---|---|---|
| `[identity]` | `scripts/do_register.py <uid> <model>` | Creates a new DO file with class defaults filled in.  Mandatory before the engine will start with a new `do_label`. |
| `[steering]` | `scripts/do_steering_char.py` | DAC sweep, linear fit, range detection.  Outputs slope, intercept, code_min, code_max. |
| `[freerun_noise]` | `scripts/do_freerun_char.py` | TICC chA observation against Rb (preferred) or against chB (acceptable fallback), computes ADEV/TDEV/PSD, derives σ_do_phase + σ_do_freq + coast_tdev_ref_ns + coast_tdev_slope.  Refuses any source label not in the allowed enum.  When both Rb and chB are available, `chA vs Rb` wins — chB carries rx-TCXO motion that contaminates the DO signal.  `coast_tdev_slope` is fit explicitly to the RWFM/flicker **rising tail** (positive-slope region), NOT a single global log-log LSQ — a global fit over a U-shaped TDEV curve can return a negative slope that silently disables the Goldilocks scheduler at runtime.  The schema validator rejects slope ≤ 0 at write time. |
| `[actuation_noise]` | `scripts/do_actuator_char.py` | Holds adjfine constant, observes residual TDEV, derives σ_q. |
| `[operational_state]` | engine (only) | Updates last_known_* on each save_do_state checkpoint. |
| `[aging]` | manual entry initially; later `scripts/do_aging_refresh.py` extracts drift_ppb_per_year from `[operational_state]` history. |

The tools write ONLY their own section.  They never `setdefault`,
never merge, never preserve old keys outside their section.  This
is the inverse of the current `freerun_analysis.merge_characterization_into_json`
behavior and is what prevents PiFace-style stale-source
accumulation.

Out of scope (for this design):

- `[thermal]` section.  Bob is adding temperature sensors to the
  OCXOs; revisit when there's data to populate it.  The schema
  reserves the section name; the loader accepts its absence.


## Q[3,3] design change vs qFromCharPerActuator

The previous design (`qFromCharPerActuator`, codified in
`do_state.py:325-340`) deliberately made the actuator-command ASD
(`freq_command` / `adjfine`) the **primary** σ_do_freq source —
the "freq noise floor at the actuator input."  This document
overturns that decision.  Stating the change explicitly so it's
not silent:

**Old**: σ_do_freq derived from `freq_command` / `adjfine` ASD,
falling back to ADEV-RWFM only when those weren't present.

**New**: σ_do_freq derived **only** from the freerun-ADEV RWFM
tail.  The disciplined `adjfine` ASD is reframed as σ_q under
`[actuation_noise]` and **never** feeds Q[3,3].

The reframe matches the physics: a disciplined `adjfine` stream
reflects loop activity + reference noise + DO response, not the
DO's open-loop random-walk-FM.  Its honest home is the actuator
chain's per-write quantization + write-noise budget (σ_q), which
is what the Goldilocks τ* formula expects.  The freerun-ADEV RWFM
tail is the physically correct open-loop frequency random walk
and is what Q[3,3] models.

**Gating**: this change ships behind a closedLoopServoSim A/B
check before the schema PR lands.  Two-arm acceptance criterion:

1. **Over-loose arm** — replay PiFace's overnight trace under each
   Q[3,3] source.  Under the OLD source, the sim should reproduce
   the observed ~60 ppb adjfine std wobble.  Under the NEW source,
   it should drop to clkPoC3-class (single-digit ppb std).
2. **Over-tight arm** — inject a long GNSS-gap coast on clkPoC3
   under the NEW Q source.  Confirm no overconfident-coast
   divergence on GNSS-return (the longTauGnssCoupling √P[2,2]
   cap should still bound the coast residual).  This is the
   regression the over-loose arm can't see by itself.

PASS = wobble gone (PiFace, over-loose arm) AND coast stays
bounded (clkPoC3, over-tight arm).  Expected to pass because
`_sigma_do_freq_ppb_from_adev` picks largest-τ as the safe
direction, but verify, don't assume — that's the half of the
regression the original one-sided criterion couldn't catch.


## Lifecycle

A DO progresses through measurement phases.  At each phase the
engine can run, with progressively tighter Q values and honest
provenance logged.

| Phase | Trigger | Sections filled | Q source |
|---|---|---|---|
| 0. Register | new hardware install | `[identity]`, every other section with `source = "class-default"` and the class default values | `class-default[OCXO]` etc. |
| 1. Steering | run `do_steering_char.py` once | `[steering]` measured | unchanged (Q is about noise, not gain) |
| 2. Freerun | run `do_freerun_char.py` (~1 h capture) | `[freerun_noise]` measured | `measured-freerun` |
| 3. Actuator | run `do_actuator_char.py` (~30 min) | `[actuation_noise]` measured | unchanged (Q unchanged; Goldilocks τ* becomes exact) |
| 4. Runtime | engine running | `[operational_state]` per-checkpoint; ADEV-from-coast can refine `[freerun_noise]` over time | `measured-freerun-refined` |
| 5. Reverify | every 6 months or after hardware change | re-run phases 1, 2, 3 | refreshes provenance |

Phase 4 — runtime ADEV-from-coast refinement — is exactly the
intuition that "we can observe freerun noise whenever the
actuation scheduler lets the DO coast."  Yes, we can: the
engine watches its own TICC chA stream during scheduler coast
intervals, accumulates a running ADEV, and on each save_do_state
checkpoint updates `[freerun_noise]` with a tighter measurement.
This is sound **only because** the schema forces the coast
observation to flow through a DO-pointing channel.  In the old
JSON schema, runtime updates could silently land in
receiver-side `sources` and poison σ_DO.

The engine refuses to start without `[identity]` registered.
It accepts class-default values for the other sections, but logs
the source at startup for every Q value it consumes:

```
DOFreqEst Q: sigma_do_phase=0.0425 ns/√s (source=measured-freerun),
             sigma_do_freq=0.000382 ppb/√s (source=measured-freerun)
```

```
DOFreqEst Q: sigma_do_phase=0.1000 ns/√s (source=class-default[OCXO]),
             sigma_do_freq=0.001000 ppb/√s (source=class-default[OCXO])
```


## Class defaults

Conservative starting points so a freshly-registered DO can run
the engine immediately.  Each default must be looser than the
worst-real-unit measurement we have, so a default-using DO is
never *implicitly* asserted to be tighter than a measured DO of
the same class.

| Class | σ_DO_phase | σ_DO_freq | σ_q | Lineage (real unit this must beat) |
|---|---:|---:|---:|---|
| OCXO | 0.1 ns/√s | 0.001 ppb/√s | 0.05 ppb | ~2.6× looser than clkPoC3 measured (0.0382 / 0.000382); leaves headroom for noisier OCXOs |
| TCXO | **3.0 ns/√s** | 0.1 ppb/√s | 0.1 ppb | TimeHat measured freerun TDEV(1s) = 2.6 ns/√s (per Main's review); class default must be looser, hence 3.0 |
| Rb | 0.01 ns/√s | 0.0001 ppb/√s | 0.01 ppb | beats most OCXOs by design; a Rb that needs class-default is broken |
| PHC (i226 adjfine) | 0.5 ns/√s | 0.1 ppb/√s | 0.5 ppb | empirically from `[[adaptive-scheduler-motivation-timehat]]` — 1 Hz actuation worse than coasting implies actuator σ_q is ns-class, not the 15-fs LSB floor |

Class defaults live in code (`peppar_fix.do_schema.CLASS_DEFAULTS`),
not in a per-DO file.  When the engine reads a section with
`source = "class-default"`, it pulls the value from the in-code
table.  Changing a default is a code review, not a per-DO edit.

Honest framing for the engine's log line: a per-class default is
*not* a measurement, and we should be loud about that:

```
DOFreqEst Q: sigma_do_freq=0.001000 ppb/√s (source=class-default[OCXO]
             — RUN do_freerun_char.py FOR A REAL MEASUREMENT)
```


## Migration

One-shot, supervised, across all hosts.  Zero installed base, so
no need for reversibility or operator-confirmation logic — we
control every host, we run the migration, we delete the old files
when the new ones validate clean.

`scripts/migrate_do_state.py` per host:

1. Read whatever `state/dos/<uid>.json` / `<uid>_noise.json` /
   `<uid>-v2.json` files exist.
2. Best-effort extract DO-pointing measurements (`DO PPS (chA vs TICC Rb)`
   first, `DO PPS (chA-chB)` second).  Drop everything else.
3. Map known fields into the new schema sections.
4. Write three files: `state/dos/<uid>.toml` (characterization),
   `state/dos/<uid>.runtime.toml` (operational state), and
   `state/dos/<uid>.noise.toml` (sidecar — dense ADEV / TDEV /
   PSD curves extracted from the old JSON's sources).  The
   sidecar is the one genuinely irreplaceable artifact in the
   old JSON: everything else recomputes or class-defaults, but
   the hour-long freerun captures don't.  Writing the sidecar
   here makes the no-backup deletion in step 7 safe — the
   sidecar IS their new home, not a backup of it.
5. Validate the new files load cleanly through the new schema.
6. Print a per-section provenance summary so it's immediately
   obvious which sections fell to `class-default` and need
   urgent re-characterization:

   ```
   ocxo-piface migration summary:
     [identity]:          measured       (from ocxo-piface.json)
     [steering]:          measured       (from ocxo-piface.json:adjustment)
     [freerun_noise]:     measured       (DO PPS (chA vs TICC Rb), σ_phase=0.0425)
     [actuation_noise]:   class-default  ← run do_actuator_char.py
     [aging]:             class-default  ← manual entry pending
   ```

7. Delete the old `.json` / `_noise.json` / `-v2.json` / `.bak.*`
   files for this DO.  No `.preMigration` backups — the new files
   (main + runtime + sidecar) carry every artifact that was in the
   old JSON; everything else recomputes or class-defaults.

The migration is a single afternoon's work across the fleet
(PiFace, clkPoC3, MadHat, TimeHat).  Each host: `git pull && sudo
python3 scripts/migrate_do_state.py`, eyeball the provenance
summary, confirm.

After the migration lands, the hardcoded `_PHASE_FALLBACK_NS = 0.92`
in `do_freq_est.py` gets deleted.  Its replacement is the
class-default mechanism above — which has explicit provenance in
the log line instead of an unsourced 0.92.


## Execution

The work is sequenced for blast-radius.  Each step is independently
reviewable and shippable.

1. **This doc** — design sign-off.  No code changes.
2. **Schema + loader** — `peppar_fix/do_schema.py` defines the
   TOML schema, validator, class-default table, single
   `load_do_characterization()` function.  Old `load_do_state` becomes
   a thin shim that emits a deprecation log.  Tests cover schema
   round-trip + every refusal case (rx-side source, missing
   section, unrecognized version).
3. **Tools** — register / freerun_char / steering_char / actuator_char
   updated to write only their owned section, with schema
   validation on write.  `freerun_analysis.merge_characterization_into_json`
   is the file replaced.
4. **Migration** — `scripts/migrate_do_state.py` and per-host
   `git pull && python scripts/migrate_do_state.py` ritual.
5. **Re-characterize each lab DO** — PiFace, clkPoC3, MadHat,
   TimeHat.  Each: steering → freerun → actuator.  Each result
   documented in `docs/lab-dos/<uid>.md` with measurement
   plots and sign-off.
6. **Burn down** — delete `load_do_state` shim, delete
   `_PHASE_FALLBACK_NS`, engine refuses unregistered DOs.

Steps 2 and 3 are the load-bearing engineering work.  Step 4 is
mechanical.  Step 5 is lab time, sequenced per-host so the fleet
keeps a known-good baseline.  Step 6 closes the door on the old
mechanism.


## Non-goals

- GNSSDO+ topology where the DO feeds the receiver's external
  reference.  Treated as a one-of-a-kind black box from the
  engine's perspective.  Its internal characterization is the
  vendor's business.
- Temperature schema.  Deferred until temp-sensor data exists
  on the OCXOs.  The schema reserves the `[thermal]` section
  name; the loader accepts its absence.
- Posterity-style autopsy of the PiFace cascade or the historical
  drift in source-naming.  The reference at the top of this doc
  is enough; the rest of the document is forward-looking.
- Multi-DO host configurations (one host disciplining multiple
  DOs simultaneously).  Not a current need; the schema is
  per-DO and the host config selects one `do_label`.
