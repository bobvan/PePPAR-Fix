# pgnssd — founding brief

**Handoff for the agent starting the pgnssd repo.  Written 2026-09-13 by
PePPAR-Fix's lead session, at Bob's direction.**

This file is written to be **moved into the new repo** as its first document.
It is self-contained on purpose: you should not need to read PePPAR-Fix to
start, though you will need to read it to finish.

---

## What you are building

**pgnssd — a precision GNSS daemon.**  Receiver-agnostic precise positioning
as a service.  Bob's framing:

> Something like a precision gpsd.  Something that works with a variety of
> GNSS receivers, starts with their position solution, but then also RTK or
> PPP to get a more precise fix and continuously update it.

The niche, stated as a pitch: **"the gpsd of survey-grade positioning."**
gpsd solved device abstraction at SPP precision.  Nothing open-source solves
*orchestration* at survey precision — choosing a method, finding a base,
handling datums, and reporting an honest uncertainty.  The engines are a
solved space; the orchestration is not.

**Adopt engines, never write them.**  PRIDE PPP-AR (best static PPP-AR),
RTKLIB/demo5 (`convbin` reads nearly every receiver; `rnx2rtkp` for
baselines), gLAB, Ginan.  GIPSY-X benchmarks best and is **not** open source
— not an option.  Your product is the layer above them.

---

## Where the code comes from

Most of it exists, working and tested, in **PePPAR-Fix**
(`github.com/bobvan/PePPAR-Fix`, checkout at `~/git/PePPAR-Fix`).  Read these
three first:

1. `docs/pgnssd-separation.md` — the boundary, what moves, what stays, the
   phasing, the risks.  **Start here.**
2. `docs/gpsd-for-precision-plan.md` — built vs designed, with file/line
   evidence, plus a sequenced breakdown S0–S5 and the traps.
3. `docs/arp-survey-strategy.md` — the tier cascade and the "contingency-free
   floor" mental model that makes the whole thing testable.

Then `docs/peppar-survey-modes.md` (on-demand vs daemon; the
facts-not-commands rule) and the memory `peppar-survey-vs-oss` for the
build-vs-adopt survey of the OSS landscape.

**The extraction is unusually clean**, because `peppar-survey` never touches a
receiver: it consumes RINEX files plus NTRIP and emits a state file.  Its
dependency on PePPAR-Fix is shared modules and a file format, not hardware.

---

## What is already built (move it)

- the `--auto` **tier cascade**: RTK relative-baseline (minutes, cm) →
  real-time PPP-AR → PRIDE-rapid → PRIDE-final floor → self-survey, each
  capability-gated
- **base discovery, two sources**: NTRIP sourcetable ranking, *and* the NGS
  CORS station catalogue (~1650 US sites that appear in no sourcetable,
  cached 30 days so field runs work offline)
- **engine adapters** for PRIDE and RTKLIB
- **datum discipline**: ETRS89/NAD83 → ITRF2020@epoch, in one place
- the **provenance/σ model** and the categorical antenna-epoch gate

## What does not exist anywhere (build it)

- **the device layer** — many receivers, many protocols, one internal model.
  This is pgnssd's reason to exist.  PePPAR-Fix's rover path is u-blox-only
  for three specific, localized reasons; see plan doc §2.2.  **Do not port
  the broken plumbing and patch it** — the fix *is* the abstraction.
- **the daemon** — and note its stop condition does not exist yet either
- **a client protocol** — machine-readable, streaming, versioned

---

## Eight invariants — do not re-derive these

Each cost real time to learn.  Violating one is the most likely way this
project goes wrong.

1. **Publish facts, never commands.**  Run identically where no consumer is
   installed.  Never tell a consumer what to do with a position.
2. **σ is precision, not accuracy.**  Ranking heterogeneous sources by σ lets
   the most overconfident win: a filter output at σ 0.024 m and **3.33 m off**
   once shadowed an authoritative survey.  Rank by **provenance tier** first,
   σ only *within* a tier.  **Unknown provenance must fail loudly, never sort
   low** — that is how the same bug comes back.
3. **The antenna-epoch gate is categorical, never a weighting.**  A stale
   pin's σ is *small*, so σ-ranking structurally cannot notice that the
   antenna moved.
4. **Keep the contingency-free floor.**  24 h dual-frequency → PRIDE works
   anywhere: no base, no AR, no phase biases.  Everything above it buys
   **speed only** — which is exactly why the cascade is testable, since the
   floor guarantees correctness and the heuristics are just a ranking
   function.
5. **Never bake a receiver into a contract.**  Raw bytes plus a `convbin -r`
   format tag, not "UBX RAWX + SFRBX".  That one change also reaches a
   Trimble NetRS via `-r rt17` with no new decoder.
6. **Naming honesty.**  *Survey* (external, authoritative) ≠ *PPP solution*
   (our filter's own opinion) ≠ *NAV2* (receiver SPP, carrying a persistent
   1.5–4 m receiver-specific bias).  Any threshold built on NAV2 must be loose
   enough to absorb that bias, or it produces continuous false positives.
7. **Check signal overlap before declaring a base usable.**  An L1/L5 rover
   against an L1/L2 base shares only L1 — no baseline.  This bit the project
   three times in one week.
8. **"Converged" and "gave up" are different answers.**  A position service
   that cannot say *what is blocking further refinement* is not finished.
9. **An honest σ is a safety feature, not a nicety.**  BBGresearch's reason
   for wanting it is worth internalizing: position error becomes a **timing
   bias**, and a confidently wrong bias is worse than a loosely known one.  An
   optimistic formal σ launders straight into a claimed accuracy downstream.
   Every engine reports a formal σ; reporting an *across-day* σ instead is the
   single clearest thing that distinguishes this project from a wrapper.

---

## Your first tasks

1. **Read the three docs above**, and push back in writing on anything in
   `pgnssd-separation.md` you think is wrong.  It is a proposal, not a
   specification, and it was written without BBGresearch's requirements in
   hand.
2. **Honour the second consumer's requirements — they have already landed.**
   BBGresearch answered 2026-09-13 (their
   `docs/design/position-accuracy-requirements.md`, commit `8951a0e`), and two
   of their needs are interface-shaping in ways the ARP-pinning case would
   never have surfaced.  Both are in `pgnssd-separation.md` §5.1; in short:

   - **Expose inter-site covariance, not just per-site σ.**  A differential
     consumer uses the *baseline*.  Two sites surveyed against each other can
     be far better relatively than either is absolutely, and a per-site API
     discards exactly that.  It is not new computation — the RTK
     relative-baseline stage already computes it and throws it away on the way
     to an absolute answer.
   - **The record must hold a non-GNSS position.**  Many of their clocks are
     *indoor by design*; no cascade will ever position them.  Tape off a
     surveyed point, total station, floor plan, network solve.  Same datum,
     epoch, σ and provenance discipline as a GNSS answer — so the provenance
     tier map needs room for non-GNSS classes **from the start**, not bolted
     on later.

   Also from them: carry an explicit **phase-centre offset** (the point of
   interest is the phase centre, not the mount), and keep **immutable
   history** — when a site is re-surveyed, every measurement taken under the
   old value becomes uninterpretable without it.
3. **Propose the repo skeleton** — package `pgnss`, daemon `pgnssd`, license
   BSD-3-Clause to match PePPAR-Fix — and the move plan for phase 1.  Phase 1
   must be a *pure move*: no behaviour change, PePPAR-Fix's suite still green.
4. **Do not start the device layer until the move lands.**  It is the most
   interesting work and the most tempting to start first.

---

## Working rules inherited from PePPAR-Fix

- **"All tests pass" means literally all tests pass** — no "N pre-existing
  failures we ignore."  Replicate that discipline; do not dilute it.
- **Never `scp`/`rsync` code that is tracked in a repo.**  Drift between
  hosts that git cannot see has caused real incidents.
- **Naming honesty as a code-review standard** — an identifier whose name
  implies one thing while the implementation does another is a defect.  The
  most dangerous are names that are wrong about *what kind of signal* a thing
  measures.
- **Research tasks produce a markdown document**, not just a conclusion.

## Open questions you inherit

- Does PePPAR-survey survive as a program, or collapse into a pgnssd profile
  plus a state-file writer?  **The second outcome is fine.**  Do not invent
  work to keep it alive.
- Does the engine's real-time position filter (`PPPFilter`/`AntPosEst`)
  eventually move?  Deferred deliberately — most entangled code in the
  engine, and `--no-antposest` means the clock does not need it.
- Library-first or daemon-first?  This brief assumes **library-first**, with
  the daemon as a thin wrapper, because in-process consumers should not have
  to run a service to get a position.  Argue if you disagree.
