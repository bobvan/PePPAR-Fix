# Separating the clock, the survey, and the positioning engine

**Proposal, 2026-09-13, for Bob.**  Extract PePPAR-Fix's precise-positioning
machinery into a standalone project — working name **pgnssd** — consumed by
PePPAR-Fix, by BBGresearch, and by anyone else.

Companion to [`gpsd-for-precision-plan.md`](gpsd-for-precision-plan.md), which
inventories what is built vs designed.  This doc answers the different
question that one did not: **where should the code live, and what is the
boundary.**

---

## The proposal in one paragraph

`peppar-survey` is already decoupled from PePPAR-Fix's hardware — it opens no
serial port, consumes RINEX files plus NTRIP, and emits a state file.  That is
the seam.  Cut along it: **pgnssd** gets the cascade, the base discovery, the
engine adapters, the datum discipline, the provenance/σ model, and the
receiver-device layer that does not yet exist anywhere.  **PePPAR-Fix** keeps
the clock — servo, DO, actuators, observers, stream correlation — and becomes
a *consumer* of a pinned position.  **PePPAR-survey** shrinks to a lab-policy
adapter, and may dissolve entirely; that is an acceptable outcome, not a
failure of the design.

---

## 1. Why now

Two independent reasons, one of which is new.

**The trend is already underway.**  PePPAR-Fix has been getting out of the
positioning business for months.  `--no-antposest` is the *recommended*
operational mode ([`time-only-architecture.md`](time-only-architecture.md));
all four lab hosts switched 2026-05-20; the engine skips its own position
filter and runs `FixedPosFilter` at an ARP that `peppar-survey` delivered
offline.  The clock already treats position as an **input**, not a
responsibility.  This proposal finishes a migration rather than starting one.

**There are now two consumers.**  That is the threshold that makes
generalization honest.  One consumer produces a library shaped exactly like
its only caller; two force a real interface.  Bob names PePPAR-survey and
BBGresearch, plus standalone use.  I have asked BBGresearch directly what they
need — **that answer should land before the interface is frozen**, because a
pgnssd designed against ARP-pinning alone will need rework the first time a
different consumer touches it.

---

## 2. The seam already exists

The single most useful finding in the plan doc (§1.4) is that **peppar-survey
does not touch a receiver at all**.  `grep -E 'pyubx2|serial|/dev/tty'` across
`scripts/peppar_survey.py` and every `peppar_survey_*.py` backend returns one
hit, and it is an *antenna mount* serial number, not a port.

So its real dependency surface on PePPAR-Fix is only three things:

| coupling | what it is | disposition |
|---|---|---|
| **Inputs** | RINEX files on disk, NTRIP for a base | already generic — no change |
| **Output** | `state/positions/<uid>.survey.toml` | becomes pgnssd's published contract |
| **Shared code** | modules under `scripts/peppar_fix/` | the mechanical part of the move |

That is a much cleaner cut than a typical extraction, because the hard part —
decoupling from hardware — was done by accident of design.  What is *not*
there is the thing that would have made the cut hard: RINEX **production**,
which today happens inside the engine and is u-blox-only.  pgnssd builds that
fresh rather than inheriting it.

---

## 3. The three boxes

```
                    ┌──────────────────────────────────────────┐
                    │  pgnssd  —  precise positioning, general │
                    │                                          │
   receivers ──────▶│  device layer   (NEW — exists nowhere)   │
   NTRIP ──────────▶│  base discovery · engine adapters        │
                    │  tier cascade · datum · σ honesty        │
                    │  provenance tiers · ARP epoch            │
                    └───────┬──────────────────────┬───────────┘
                            │ facts                │ facts
                ┌───────────▼──────────┐  ┌────────▼───────────┐
                │  PePPAR-survey       │  │  BBGresearch       │
                │  (lab policy adapter)│  │  (station/antenna  │
                │  antennas.json,      │  │   positions)       │
                │  .survey.toml, epoch │  └────────────────────┘
                └───────────┬──────────┘
                            │ a pinned ARP
                ┌───────────▼──────────────────────────────────┐
                │  PePPAR-Fix  —  the clock                    │
                │  servo · DO · actuators · TICC/EXTINT/qErr   │
                │  stream correlation · FixedPosFilter · PPS   │
                └──────────────────────────────────────────────┘
```

**Say the middle box out loud:** once pgnssd exists, PePPAR-survey is a
configuration profile plus a state-file writer.  It may not deserve to be a
program.  Do not invent work to keep it alive — if it collapses into a pgnssd
profile and twenty lines in PePPAR-Fix, that is the design succeeding.

---

## 4. What moves, what stays, what is new

**Moves to pgnssd** — the orchestration with no OSS equivalent, per
[[peppar-survey-vs-oss]]:

- the `--auto` tier cascade (`peppar_survey_auto.py`)
- base discovery, both sources (`peppar_survey_discovery.py`, `peppar_survey_cors.py`)
- engine adapters (`peppar_survey_pride.py`, `peppar_survey_rtklib.py`)
- datum conversion (`geo_frames`) — the ETRS89/NAD83 → ITRF2020@epoch rule, in one place
- the lifecycle state machine (`survey_lifecycle.py`) and its σ/provenance model
- `rinex_writer.py`, as the *output* half of the new device layer

**Stays in PePPAR-Fix** — the time mission:

- servo, `DOFreqEst`, actuators (DAC, PHC `adjfine`, ClockMatrix)
- TICC / EXTINT / qErr observers and all `CLOCK_MONOTONIC` correlation
- `FixedPosFilter` — clock estimation at a *pinned* ARP is the clock's job
- the PPS/PEROUT path, ptp4l supervision, watchdogs

**Stays for now, deliberately** — the engine's *real-time* position filter
(`PPPFilter`, `AntPosEst`, `solve_ppp.py`).  It is position-shaped and in
principle belongs in pgnssd, but it is the most entangled code in the engine,
it shares filter states with the clock, and `--no-antposest` means the time
mission does not need it.  Moving it early buys nothing and risks the clock.
Revisit once pgnssd has earned trust.

**New in pgnssd, built there rather than extracted** — the **device layer**.
Per the plan doc §2.2 the rover path is u-blox-only for three reasons
(`rinex_writer` splatted into a single thread; a u-blox allowlist in
`_CAP_TABLE` whose unknown-module default is backwards; external obs sources
carrying no receiver UID).  **Do not patch that in PePPAR-Fix and then move
it.**  The fix *is* the device abstraction, which is pgnssd's reason to exist.
Build it once, in the right place.

---

## 5. The interface pgnssd must expose

Three surfaces, in descending order of how much they matter:

1. **A library.**  The primary artifact.  In-process consumers (BBG, tests,
   PePPAR-Fix itself) should never have to run a daemon to get a position.
2. **A daemon + line-oriented protocol.**  The gpsd analogy, and the thing
   that makes "continuously update it" real.  Publishes; never commands.
3. **A state file.**  `.survey.toml` generalized, for back-compat and for
   consumers that want a file rather than a socket.

All three publish the same record.  Minimum fields, each of which exists
because something went wrong without it:

| field | why |
|---|---|
| position + frame + **epoch** | ITRF2020@epoch; a coordinate without an epoch is not a position |
| **σ, and its kind** | honest across-day σ vs engine formal σ — never interchangeable |
| **provenance tier** | survey-class / PPP-solution / NAV2 — categorical, outranks σ |
| **antenna epoch** (`mount_sn`) | the move-detection gate; categorical, never a weight |
| method + engine + products | which tier answered, and with what corrections |
| receiver UID | without it, discovery, epoch resolution and seed read-back all fail |
| convergence state + **what is blocking** | "converged" and "gave up" are different answers |
| **inter-site covariance** | see §5.1 — a per-site API discards what a differential consumer needs |
| **phase-centre offset** | the position of interest is the antenna phase centre, not the mount; separable only if the record can hold the offset |
| **immutable history** | when a site is re-surveyed, every measurement taken under the old value is uninterpretable without it |

### 5.1 Two requirements from BBGresearch that a PePPAR-only design would miss

Asked and answered 2026-09-13 (their `docs/design/position-accuracy-requirements.md`,
commit `8951a0e`).  Both are interface-shaping, and neither would have occurred
to us from the ARP-pinning use case:

**(a) Expose the covariance *between* sites, not just per-site σ.**  What a
differential consumer uses is the **baseline**, not two independent absolute
positions.  Two sites surveyed against each other can have far better relative
accuracy than either has absolutely — the common-mode part cancels in the
difference.  A per-site API has already thrown that correlation away.  Note
this is not new computation: it is precisely what the RTK relative-baseline
stage computes internally and then **discards on the way to an absolute
answer**.  If pgnssd returns sites independently, BBG's main consumption path
is degraded by construction.

**(b) The record must hold a position that did not come from GNSS.**  A large
fraction of BBG clocks are **indoor by design** — no sky view, so no cascade
positions them, ever.  Those positions come from a tape off a surveyed point,
a total station, a floor plan, or from a network solving for them off
broadcast observations.  If the record cannot carry a non-GNSS position with
the *same* datum / epoch / σ / provenance discipline, the consumer ends up
maintaining a second, incompatible position system — and ours is the one that
loses.  Concretely this means the provenance tier map (invariant 2) must have
room for non-GNSS classes from the start, rather than treating GNSS method as
the only axis.

**Why the epoch field is load-bearing for them specifically:** their
measurements are long-lived and mid-continent plate motion is cm/yr, so a
position without its epoch becomes unusable within a few years — not merely
imprecise.

**An observation of theirs worth carrying**, because it explains why this
matters beyond one consumer: a differential architecture cancels the
transmitter's *clock*, not its *geometry*.  The residual is
`|T−B| − |T−R|`, so an error `d` in a transmitter position perturbs it by
roughly `d·(b/D)` for baseline `b` and range `D` — 0.5 m at a 20 km baseline
over 30 km range.  Since you cannot put a receiver on someone else's tower,
the resolution is to **invert it**: a network of well-surveyed receivers
solves for the transmitter, making transmitter positions a *product* rather
than an input.  Receiver survey error is therefore load-bearing twice.  That
is a second consumer whose needs genuinely differ from ARP-pinning — which is
the justification for generalizing at all.

---

## 6. Invariants that must survive the move

Each of these was learned the hard way.  They are the real payload of the
handoff, and re-deriving any of them costs more than the extraction.

1. **Publish facts, never commands.**  pgnssd must run identically where its
   consumers were never installed, and must never tell a consumer what to do
   with a position.  (`peppar-survey-modes.md`)
2. **σ is precision, not accuracy.**  Ranking heterogeneous sources by σ lets
   the most overconfident win — an AntPosEst `.ppp.toml` at σ 0.024 m and
   3.33 m off once shadowed an authoritative survey.  Fixed by provenance
   tiering in PR #275.  **Successor trap:** a candidate whose `kind` is
   missing from the tier map sorts *below* `.ppp.toml`.  Unknown provenance
   must fail loudly, never sort low.
3. **The antenna-epoch gate is categorical.**  A stale pin's σ is *small*, so
   σ-ranking structurally cannot detect a moved antenna.
4. **Keep the contingency-free floor.**  24 h dual-frequency → PRIDE works
   anywhere with no base, no AR, no phase biases.  Every heuristic above it
   buys *speed only*, which is what makes the cascade testable.
5. **Do not bake a receiver into a contract.**  The spool contract says "UBX
   RAWX + SFRBX bytes" and names the stream `"ubx"` — u-blox baked into the
   very contract meant to deliver receiver-agnosticism.  Raw bytes plus a
   `convbin -r` format tag instead; that also reaches the idle Trimble NetRS
   via `-r rt17` with no new decoder.
6. **Naming honesty.**  *Survey* (external, authoritative), *PPP solution*
   (our own filter's opinion) and *NAV2* (receiver SPP, 1.5–4 m biased) are
   three different things.  `peppar-survey --from-ppp` conflated two of them
   and was removed; do not reintroduce the pattern.
7. **Check signal overlap before declaring a base usable.**  An L1/L5 rover
   against an L1/L2 base shares only L1 — no baseline.  Cost us three times
   in one week.
8. **Adopt engines, do not write them.**  PRIDE, RTKLIB/convbin, gLAB, Ginan.
   The orchestration is the product.  (GIPSY-X benchmarks best and is *not*
   open source — not an option.)

---

## 7. Sequencing

| phase | what | where | gate |
|---|---|---|---|
| **0** | `converged` + honest `sigma_kind` (plan doc S1) | PePPAR-Fix | it defines the contract pgnssd publishes; also revives a built-but-dead state machine |
| **1** | cut the repo; move the modules; PePPAR-Fix consumes via state file | both | PePPAR-Fix's test suite still green; lab hosts unaffected |
| **2** | the device layer (plan doc S2) | pgnssd | a Mosaic-T rover survey end-to-end |
| **3** | daemon + protocol (S3–S4) | pgnssd | a second consumer uses it |
| **4** | *maybe* the engine's real-time position filter | both | only once 1–3 have earned it |

Phase 0 first because `converged` is the daemon's own termination predicate —
a daemon cannot be built on a stop condition that no code can set.

---

## 8. Risks

- **Premature abstraction.**  Substantially retired 2026-09-13: BBGresearch's
  requirements arrived before the schema froze, and two of them (§5.1) would
  not have come from the ARP-pinning case.  That is the two-consumer test
  doing its job.  The residual risk is now the opposite one — designing for
  *these* two and calling it general.
- **Breaking the clock.**  PePPAR-Fix runs on lab hosts continuously.  Every
  phase must leave it working; phase 1 should be a pure move with no
  behaviour change, verifiable by `./bin/test` alone.
- **The middle box lingering.**  If PePPAR-survey survives as a program
  without earning it, we will have three things to maintain instead of two.
- **pgnssd as a dumping ground.**  "Position-shaped" is not a membership
  criterion.  The test is: *would a consumer with no clock want this?*
- **Two repos, one test story.**  PePPAR-Fix's `./bin/test` discipline —
  literally all tests pass, no tolerated failures — has to be replicated, not
  diluted.

---

## 9. On the name

**`pgnssd` works; keep it.**  It follows the gpsd lineage, which is precisely
the pitch.  Two refinements:

- **Separate package from daemon,** as gpsd does with `libgps`/`gpsd`.  The
  library is the primary artifact and in-process consumers should not import
  something named "daemon".  Suggest package **`pgnss`**, daemon **`pgnssd`**,
  CLI **`pgnss`**.
- Minor and worth knowing rather than acting on: `pg` reads as PostgreSQL to a
  lot of people, and `pgnssd` is awkward aloud.

Alternatives considered and rejected: `pntd` (PNT means Position *Navigation
Timing* — overclaims; timing is explicitly PePPAR-Fix's job), `arpd` (collides
with Address Resolution Protocol), `surveyd` (too narrow — Bob wants it beyond
survey), `fixd` (collides with "repair", though the PePPAR-**Fix** lineage is
a point in its favour).

---

## Related

- [`gpsd-for-precision-plan.md`](gpsd-for-precision-plan.md) — built vs designed, and S0–S5
- [`arp-survey-strategy.md`](arp-survey-strategy.md) — the cascade and the floor
- [`peppar-survey-modes.md`](peppar-survey-modes.md) — on-demand vs daemon, facts-not-commands
- [`time-only-architecture.md`](time-only-architecture.md) — why the clock already treats position as an input
- [`pgnssd-handoff.md`](pgnssd-handoff.md) — the founding brief for the new repo
