# Position acquisition — `peppar-survey --auto`

`peppar-survey` owns **position**; the engine
([time-only-architecture.md](time-only-architecture.md)) owns **time**. This
doc is the position half: how a deployment gets from a coarse NAV2 fix to a
surveyed ARP, and how `--auto` picks the fastest safe path to get there.

The time mission needs the ARP only to **~10 cm** (10 cm ≈ 333 ps, well under
the few-ns cross-clock budget); cm is a bonus for the moonshot's sub-ns
cross-host target, not a requirement. The engine is never blocked waiting for
it — it runs from NAV2 with honest confidence and tightens as the survey lands
(see the position→time-confidence coupling in the engine doc).

## The mental model: a contingency-free floor + graceful degradation

The whole problem collapses once you see it as **one optimization axis:
time-to-fix**, with a guaranteed floor.

- **The floor** is contingency-free: feed a long session of **dual-frequency**
  raw observations from any receiver anywhere into **PRIDE** → cm, globally.
  24 h *float* PPP already converges to ~cm (the long arc averages the float
  ambiguities down), so the floor needs **no base, no AR, no phase biases** —
  it sails past the whole SSR-bias saga. Its only requirements are dual-freq +
  internet + patience (product latency: rapid ~1 day, final ~2 weeks).
- **Everything above the floor just buys speed** by attacking one of two cost
  components — *session length* (24 h → minutes) and *product latency*
  (~1 day → immediate). A nearby base (RTK-in-post) attacks both; PPP-AR
  attacks latency but needs matching phase biases.

This is what makes the heuristics tractable and testable: a wrong caster
guess, a missing credential, or a too-long baseline just fails its gate and
**falls through to the next tier — ultimately the PRIDE floor.** There is
never a "no path" dead-end. Heuristics are an optimization, not a correctness
requirement.

## The `--auto` cascade

Ordered by expected time-to-fix; each tier is capability-gated and degrades to
the floor. `--max-time` shifts the ranking (give it 24 h and it may *prefer*
PRIDE-final for accuracy; give it "minutes" and it reaches for a baseline).

| Tier | Backend | Gate | Time-to-fix | Accuracy |
|---|---|---|---|---|
| **A** | RTKLIB relative baseline | open base ≤ ~30–50 km with overlapping signals; base data via **archive** (no creds) or stream-log | **minutes** | cm |
| **B** | Real-time PPP-AR (engine live) | SSR stream *with phase biases for the rx's signals* | minutes–hours | cm (fragile) |
| **C** | PRIDE PPP-AR, **rapid** products | dual-freq + internet | ~1 day | cm |
| **D (floor)** | PRIDE **float/AR, final** products | dual-freq + internet | ~2 weeks | best cm |
| **D′** | degraded: single-freq / offline long static-average | always | long | dm–m (a number, at least) |

The optimizer's three inputs:

1. **Receiver capabilities** — bands (L1/L2 vs L1/L5), dual-freq?, RTK
   firmware?, raw-obs? (UBX FWVER / VALGET; see the matrix below).
2. **NAV2 coarse position** — reverse-geocoded to a region (offline boundary
   polygons) to pick the region's caster/archive set, and to seed the base
   search.
3. **Open-access casters** — a sourcetable search (Haversine-rank the STR
   records by baseline, filter by signal overlap + open-access; `find_base.py`
   prototype). NTRIP sourcetables are geolocated directories, so "nearest base"
   falls right out.

`--auto` **emits the chosen cascade before capturing** (`--plan-only`), e.g.
*"Tier A: SHOE 62 km archive, ~cm in ~1 h; fallback → PRIDE-rapid ~1 day"* —
inspectable, overridable, side-effect-free to test.

```
peppar-survey --auto [--max-time 30m|--overnight] [--offline] [--plan-only] <uid>
```

## Why this is testable

The scary matrix (L1/L2 vs L1/L5 × US/EU/Asia × find-caster × creds ×
RTK-or-not) reduces to three pieces, none needing live GNSS:

1. **Capability predicates** — pure functions (`is_dual_freq`, `bands`,
   `rtk_firmware`, `base_signal_overlap`), unit-tested with fixtures.
2. **A ranking function** — `{caps, region, ranked bases, internet?, budget}`
   → ordered backends, unit-tested with mocked inputs.
3. **A region→source data *table*** — `US→NGS CORS`, `EU→EUREF`,
   `global→IGS`, plus the caster list. Wrong/missing entries fall through.
   Tested as data; trivial to extend.

Each backend is tested in isolation; the floor guarantees correctness
regardless of heuristic quality. You test the floor hard, each backend once,
and the selector as decision logic.

## Progressive refinement (the engine's feed)

`--auto` consumes the engine's raw-obs logs (locally, or pulled to gt/ptpmon
and crunched there — the offload pattern) and writes **progressively-refined**
estimates to `state/positions/<uid>.survey.toml` as each tier completes
(schema + atomic-write contract in the engine doc). Over the first ~month at a
new APC it averages **rapid then final** products into the authoritative sub-cm
mean (how `ufo1` was nailed). When the multi-day final mean stops moving within
its σ it writes `converged = true` — the engine's cue to stop logging.

**Datum care is a hard requirement, in one place.** Archive base coordinates
carry regional datums — **EUREF = ETRS89, NGS CORS = NAD83** — that differ from
the canonical **ITRF2020** by up to ~1 m of accumulated plate motion (an
ETRS89 base cost 0.87 m in the 2026-07-03 London run until transformed). Every
backend converts its base/result to **ITRF2020 at the observation epoch**
(`pyproj`, e.g. ETRS89 `EPSG:4936` → ITRF2020 `EPSG:9988`) before it hits disk,
so what the engine reads is always ITRF2020@epoch. See
[coordinate-reference-frames.md](coordinate-reference-frames.md).

## Receiver-capability matrix (drives tier selection)

| Receiver | Bands | RTK-capable (Tier A/B real-time)? | Notes |
|---|---|---|---|
| ZED-F9P | L1/L2 | **Yes** | full RTK; portable/field workhorse. *Post-processed* baseline (Tier A) needs no onboard RTK — raw obs suffice |
| ZED-F9T (timing) | L1/L2 or L1/L5 | firmware varies | timing-focused; raw obs → Tier A (post-baseline) or C/D regardless of RTK firmware |
| ZED-X20P | L1/L5 | per-unit | confirm before offering real-time RTK |
| NEO-F10T / others | per-unit | per-unit | confirm before offering Tier A/B real-time |

Note the distinction: **Tier A (post-processed baseline) works on any
raw-obs receiver** — onboard RTK is only needed for a *real-time* streamed-base
fix, which timing acquisition doesn't require.

## Build status

- **Tier C/D (`--pride`)** — built; the lab default. ~5 mm from 24 h + finals.
- **Tier A (RTKLIB relative baseline)** — proven end-to-end 2026-07-03
  (London: F9T raw + open EUREF archive base → 20 cm at 62 km float; cm at a
  short baseline). Not yet a clean `peppar-survey` backend — currently a manual
  `f9p_rawx_log → convbin → hatanaka → rnx2rtkp` pipeline. Promoting it (with
  the caster/archive discovery + datum transform) is the near-term work.
- **`--auto` selector + caster discovery + `--plan-only`** — designed here,
  not built.
- **Tier D′ self-survey** (static-averaged own solution when no products *and*
  no base) — designed; needs the honest-σ + static-mean quality-gate first.

## Related

- [time-only-architecture.md](time-only-architecture.md) — the time half: how
  the engine bootstraps from NAV2, consumes these estimates, and slews.
- [peppar-survey-install.md](peppar-survey-install.md) — install / backends.
- pickyEaterSSR (I-210733) — non-CNES AC digestion would widen Tier B/C
  correction options.
