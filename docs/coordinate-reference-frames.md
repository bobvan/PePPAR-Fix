# Coordinate reference frames — design

**Status:** design (Bravo, 2026-06-03).  Owner of dayplan item
`coordFrameAudit`.  Companion: this is the "attach a reference system
to all coordinates" design Bob asked for after the Onocoy signup forced
an ITRS-vs-NAD83 choice and surfaced a latent frame ambiguity.
**Revised 2026-06-03** after Main + Charlie APPROVEd the diagnosis and
Bob set the constraint: *zero installed base, no legacy, four people —
rip the bandaid off, do it once and right, no gradual transitions.*
The migration below (§5) and the decisions (§7) reflect that: a single
clean cut, required frames, hard failures, no back-compat shims.

## 1. Why — the bug this exists to kill

Coordinates move through PePPAR-Fix as **bare `(X, Y, Z)` ECEF tuples
with no attached reference frame.**  The frame is an *assumption*
carried in docstrings, not *data* carried with the value.  That
assumption is already wrong in at least one load-bearing place:

- The engine's central coordinate carrier, `PositionState.ecef_m`, is
  documented as **"WGS84/ITRF"** (`position_state.py:65`).  All
  satellite orbits it processes against (broadcast, IGS, SSR) are
  **ITRF**.
- But `timelab/antennas.json` stores the operational ARPs in
  **NAD83(2011) EPOCH 2010.0** (it even has a `"frame"` field saying
  so), and `load_arp_from_antennas` (`position_state.py:328-334`)
  reads `entry["ecef_m"]` **verbatim** into the engine's ITRF
  `ecef_m` — it never reads the `frame` field and never converts.

NAD83(2011)@2010.0 and ITRF2020@2026.32 differ by **1.71 m** at the lab
(OPUS UFO1, verified by Main: ΔX = +1.016 m, ΔY = −1.376 m,
ΔZ = +0.095 m, ‖Δ‖ = 1.71 m — mostly horizontal).  So every host that
seeds its ARP from `antennas.json` pins a position **1.71 m off the
orbit frame** — a constant geometric error that biases the clock
solution and any cross-frame comparison.

This is the *"1.6–2 m datum offset"* the 2026-05-20 filter-side
position-bias investigation flagged
([[project_position_bias_filter_side_proven_20260520]]) — **and that
investigation had the attribution backwards.**  The 1.6 m gap was
read as a PRIDE error; it was actually PRIDE (real ITRF2020) vs
`antennas.json` (NAD83 read as ITRF).  PRIDE was right; the engine's
frame was wrong.  Why the lab's *cross-host* agreement still looked
sub-cm for months (Main's point): every host seeds from the **same**
`antennas.json` entry, so every host carries the **same** 1.71 m
offset in the **same** direction — it cancels in host-vs-host
residuals.  It only surfaces against frames that *don't* share the
bias: PRIDE, broadcast/SSR orbits, and (ironically) F9P RTK against US
CORS, which are real NAD83 and could carry their *own* ~1.7 m offset
the other way.

The fix is not a one-off coordinate edit — it's to make **every
coordinate self-describe its frame**, validate frames at every
boundary, and convert at the one place conversion belongs.

## 2. Audit — where coordinates live and what frame they're (implicitly) in

| Source / sink | Code site | Implicit frame today | Tagged? | Risk |
|---|---|---|---|---|
| `PositionState.ecef_m` (central carrier) | `position_state.py:65,77` | "WGS84/ITRF" (docstring) | **No field** | every consumer assumes ITRF |
| `antennas.json` ARP | `position_state.py:328` (`load_arp_from_antennas`) | **NAD83(2011)@2010.0** (`frame` field, *ignored*) | tag present, **not read** | **the ~1.6 m bug** |
| `.ppp.toml` (engine AntPosEst out) | `position_state.py` writer (keys: mount_sn, ecef_m, sigma_m, updated, source) | ITRF (engine frame) | **No** | untagged; safe only by luck |
| `.survey.toml` (peppar-survey out) | `peppar_survey_pride.py` | **ITRF2020** (PRIDE output) | **No** | untagged; happens to match canonical |
| `--known-pos` (operator) | `peppar_fix_engine.py:9532` `lla_to_ecef` | **ambiguous** (whatever the operator typed) | **No** | operator may paste NAD83 or ITRF |
| NAV2 SPP | engine NAV2 path | **WGS84** (receiver) | **No** | ~WGS84≈ITRF, but undeclared |
| Satellite orbits | broadcast / SSR / IGS | **ITRF** (IGS20=ITRF2020) | n/a | the reference everything must match |
| F9P RTK (CORS) | `f9p_*` tools | **base-station datum** (NAD83 for US CORS!) | **No** | RTK against a NAD83 CORS yields NAD83 |
| OPUS / survey docs | `timelab/surveys/*.md` | OPUS reports **both** ITRF2020 & NAD83(2011) | text only | which one got into `antennas.json`? NAD83 |

Two structural facts from the sweep:

- **No frame-transform / Helmert / datum-shift code exists** anywhere
  in the tree.  The lla↔ecef helpers (`6378137`, WGS84 ellipsoid) are
  *coordinate-shape* transforms (ellipsoid↔Cartesian), **not** datum
  transforms — they don't move between NAD83 and ITRF.
- The one coordinate field that *does* carry a frame tag
  (`antennas.json.frame`) is read by nothing.

## 3. Decision — the canonical frame

**Canonical processing frame: ITRF (IGS20 ≡ ITRF2020 realization) at
the observation epoch.**  Rationale: the geometry the engine computes
is `range = |sat_ITRF − antenna|`; the satellite orbits are
current-epoch ITRF; therefore the antenna position fed to the geometry
**must** be current-epoch ITRF.  Any other frame injects a constant
(datum) + slowly-varying (plate-motion) position error.

NAD83 is **never** the processing frame.  It is a valid *storage* /
*deliverable* frame (US official datum, matches OPUS/USGS/state-plane),
but it is plate-fixed at epoch 2010.0 and offset ~1.6 m from ITRF — so
it must be **converted to canonical at load**, never used raw.

Epoch matters: ITRF is epoch-dependent (~2 cm/yr plate motion in
North America).  So "ITRF" alone is under-specified.  We store
coordinates with an explicit **(realization, epoch)** and convert to
the *observation epoch* for processing.

## 4. Design — every coordinate carries its frame

### 4.1 The frame tag

No legacy → we go all the way: a **frame travels with every
coordinate, as a type, not a tuple.**

- A frozen **`GeoPoint(ecef: (x,y,z), frame: Frame)`** value type is
  the *only* way coordinates cross a module/file boundary.  Bare
  `(X, Y, Z)` tuples are banned at boundaries (still fine as locals
  inside a single numeric routine).
- A **`Frame(realization, epoch)`** value type with parse/format
  (`"ITRF2020@2026.42"`, `"NAD83(2011)@2010.0"`).  `GeoPoint` cannot
  be constructed without a `Frame` — there is no default, no implicit
  ITRF.
- `CANONICAL` = `ITRF2020 @ <obs-epoch>`, resolved per run.
- `PositionState` carries a `GeoPoint` (its `ecef_m` + a real `frame`
  field) instead of a bare tuple + a docstring promise.

Because there's no installed base, we rip-and-replace the bare-tuple
signatures wholesale rather than adding a `GeoPoint` *alongside* the
old ones — one type, enforced everywhere, from the first PR.

### 4.2 The conversion layer

One module, `peppar_fix/geo_frames.py`, owns *all* datum/epoch
conversion:

```
to_canonical(ecef, src_frame, obs_epoch) -> ecef_itrf2020_at_obs_epoch
convert(ecef, src_frame, dst_frame)      -> ecef
```

Implementation: a **time-dependent 14-parameter Helmert** transform
(7 params + 7 rates) between NAD83(2011) and ITRF2020, with plate-motion
epoch propagation.  Recommended backend: **`pyproj`** (PROJ ships the
official NGS/IGS transform pipelines incl. epoch handling) — add to
`pyproject.toml`.  Hand-rolled HTDP-parameter Helmert is the fallback
if we don't want the dependency.

**Validation anchor we already have:** the OPUS UFO1 report gives the
*same physical point* in both NAD83(2011)@2010.0 and ITRF2020@2026.32
(`timelab/surveys/2026-05-03-ufo1-opus-static.md`).  Any conversion
must round-trip that pair to < 1 cm.  That's a ready-made regression
test.

### 4.3 Enforcement at every boundary

The rule: **no bare `(X,Y,Z)` crosses a module or file boundary
without a frame.**  Enforced by:

- **State-file schemas** gain a *required* `frame` field:
  `.ppp.toml`, `.survey.toml`, `receivers/*.json`.  Writers stamp it;
  readers validate it and convert to canonical on load.  A missing
  `frame` on read is a **hard error**, not a warning-plus-default —
  because we rewrite every on-disk file with a frame up front (§5),
  an untagged coordinate can only mean a bug, and we want it loud.
- **`antennas.json` boundary fix (the bug):** `load_arp_from_antennas`
  reads the `frame` field and calls `to_canonical(...)` before
  returning the `PositionState`.  NAD83(2011) ARPs become ITRF at
  load; the ~1.6 m offset disappears.
- **`--known-pos`** gains an explicit `--known-pos-frame` (default
  `ITRF2020`, with a warning that NAD83 paste is a common foot-gun);
  the value is converted to canonical.
- **Function signatures / docstrings** that pass ECEF/LLA state the
  frame, and ideally pass a framed type rather than a bare tuple.
- **CORS/RTK tools:** US CORS bases are NAD83 — an RTK fix against them
  is NAD83(2011) and must be tagged + converted (relevant to the
  PATCH3 RTK cross-check and the Onocoy roadtrip pin; Onocoy itself is
  set to ITRS, which is canonical-compatible).

## 5. Migration — one clean cut (no phases, no shims)

Zero installed base means there is nothing to be gradual *for*.  We do
it in one tight change (one PR, or a short same-day stack — not a
multi-week phased rollout):

1. **`geo_frames.py` + tests** — `Frame`, `GeoPoint`, `to_canonical`,
   `convert` (pyproj, §7).  Validated against the OPUS UFO1 ITRF/NAD83
   pair (< 1 cm round-trip).
2. **Rip-and-replace the coordinate type** — `PositionState` carries a
   `GeoPoint`; every loader/writer/signature in §2's table takes/returns
   `GeoPoint`, not a bare tuple.  No alongside-the-old shim — the old
   bare-tuple signatures are *deleted*.
3. **Rewrite the on-disk data, once** — convert every stored
   coordinate to a frame-tagged form in place:
   - `antennas.json`: keep the surveyed values but make the **canonical
     `ecef_m` the ITRF2020 one** (we already have it from OPUS) with an
     explicit `frame`; retain the NAD83 values as clearly-secondary
     provenance.  **This is the 1.71 m correction.**
   - `state/positions/*.{ppp,survey}.toml`, `state/receivers/*.json`:
     stamp the real frame (engine output → ITRF2020; PRIDE → ITRF2020)
     and delete-and-regenerate where simplest (lab hosts will just
     re-bootstrap).
4. **Fix the loader + operator path together** — `load_arp_from_antennas`
   reads `frame` and `to_canonical()`s it; `--known-pos` gains a
   required/`ITRF2020`-default `--known-pos-frame`.
5. **Hard enforcement from day one** — readers reject an untagged or
   wrong-frame coordinate (no defaults); a test/lint fails on any bare
   `(X,Y,Z)` crossing a boundary.

The only thing that *isn't* code-gated is **confirmation**: the loader
fix shifts every antennas.json-seeded ARP by 1.71 m, and Main does the
lab read — does the 2026-05-20 filter-side residual collapse to the
receiver-noise floor?  That's a *validation*, not a rollout phase: we
ship the correct code and check the empirical payoff, rather than
staging it behind a flag.

## 6. Validation

- **Unit:** OPUS ITRF↔NAD83 round-trip < 1 cm; epoch-propagation over
  a known interval matches HTDP; `Frame` parse/format; every loader
  rejects an untagged or wrong-frame coordinate.
- **Integration:** seed a host from `antennas.json` (NAD83) and assert
  the resulting `known_ecef` is ITRF (~1.6 m moved), and matches the
  OPUS ITRF2020 value for that antenna.
- **Empirical (the real proof):** re-run the cross-host / filter-side
  position residual that surfaced the "1.6–2 m datum offset" on
  2026-05-20.  If the offset collapses to the receiver-noise floor,
  the bug is confirmed fixed.  Main's lab call.

## 7. Decisions (resolved in review)

- **Conversion backend: `pyproj`** (Main + Charlie both).  PROJ ships
  the official EPSG/NGS-HTDP transforms with epoch handling baked in;
  hand-rolling a plate-motion 14-param Helmert is ~50 lines where a
  sign/epoch-direction slip looks plausible at cm but is wrong at mm.
  **Pin PROJ ≥ 9.0** (Charlie — 8.x lacks ITRF2020).  pyproj is
  already present on the lab hosts.
- **Framed type: pervasive `GeoPoint` from the first PR** (Bob's
  no-legacy mandate) — not "minimal field first, pervasive later."
  We delete the bare-tuple signatures rather than adding alongside.
- **Storage epoch convention: store the surveyed coordinate in
  ITRF2020 with its true `(realization, epoch)`** (e.g. the OPUS
  ITRF2020@2026.32 value), and **propagate to the observation epoch at
  use**.  The antenna is stationary, so the ~2 cm/yr plate-motion term
  is a clean linear propagation in `geo_frames`.  No fixed-reference-
  epoch indirection.
- **Epoch term folded in now, not deferred.**  The 1.71 m datum fix is
  the urgent payoff; the ~2 cm/yr epoch propagation is sub-cm over
  current lab timescales but is the same machinery and matters for the
  separate-antenna 2-site budget, so it ships in `geo_frames` from the
  start.

## 8. References

- `position_state.py:65,285-344` — the central carrier + the
  antennas.json loader that drops the frame.
- `timelab/antennas.json` — `frame: NAD_83(2011) EPOCH:2010.0000`.
- `timelab/surveys/2026-05-03-ufo1-opus-static.md` — the OPUS
  ITRF2020 / NAD83(2011) pair (validation anchor).
- [[project_position_bias_filter_side_proven_20260520]] — the
  "1.6–2 m datum offset" this design explains and fixes.
- Onocoy frame choice (2026-06-03) — pick ITRS; the trigger for this
  work.
