# HAS vs broadcast vs SSR — correction-magnitude comparison (Galileo focus)

**Scope / plan — not yet run.**  Companion to
[`has-time-transfer-experiment.md`](has-time-transfer-experiment.md): that
experiment measures the *end effect* (TDEV of the estimated receiver clock,
with vs without HAS).  **This one visualizes the *corrections themselves*** —
per-SV clock + radial-orbit correction magnitude in ns — to answer a different
question:

> **How far off is the broadcast, and how much of that does free HAS (E6 /
> IDD) recover versus a premium SSR stream (CNES)?**

Hypothesis from the broadcast-error figure (`visual-stories.md` Plot 6):
**Galileo broadcast is already so good (~0.9 ns peak clock+orbit error) that
HAS can't improve the _time_ much** — an inherently interesting "broadcast is
already excellent" result, even as a null.  GPS broadcast (~1.4 ns) has a bit
more room; GLONASS isn't in HAS at all (HAS serves GPS + Galileo), which is
itself a contrast with the SSR plots where GLONASS dominates.

## Tiers to overlay

| Tier | Source | Decode path |
|---|---|---|
| **broadcast** | nav message (I/NAV, LNAV/CNAV) | baseline = 0 — the error corrections remove |
| **HAS — E6 SIS** | Galileo E6-B signal | `tools/has_ssr_bridge.py` (`has_page_monitor` → `has_decode_cssr` → `has_ssr_adapter.cssr_to_records`) |
| **HAS — IDD** | HAS internet feed | the IDD decoder → SSR records (no E6 tracking needed) |
| **CNES SSR** | `SSRA00CNE0` NTRIP | the existing capture / `log_ssr_corrections.py` |

All of these land as **SSR-format records in `SSRState`** (the HAS bridge
already emits `cssr_to_records`; `SSRState.update_from_records` ingests them),
so the existing `log_ssr_corrections.py` → `plot_ssr_corrections.py` pipeline
produces the figure — **no new decoder needed** (the E6 and IDD HAS decoders
already exist).

## Receiver / host

- **E6-SIS HAS** needs a receiver that tracks **E6-B** — only the **ZED-X20P**
  in the lab does (per `f9t-firmware-capabilities.md`); run on **PiPuss** (X20,
  no DO/TICC), as the time-transfer experiment does.
- **IDD HAS** needs only internet, so any host (gt / ptpmon) can pull + decode
  it concurrently.  **E6-SIS vs IDD** is a bonus axis — latency / availability /
  agreement between the two HAS delivery paths.
- Capture broadcast eph + CNES SSR concurrently so every tier covers the same
  SVs and epochs.

## Method

1. Log each tier's per-SV **clock + radial-orbit** correction to the long CSV
   (`log_ssr_corrections.py`, one run per source; broadcast = the 0 baseline).
2. The correction a tier supplies = the broadcast error it removes
   (`−(c0 + orbit_radial)`); overlay all tiers per SV on the broadcast-error
   axes (detrended, common ns scale, draw-order/line-weight emphasis), **plus a
   ±1 ns Galileo zoom** — the Galileo story is sub-ns.
3. For an absolute "residual vs truth": difference each tier's implied precise
   clock against the **IGS/CODE final** clock (truth), so broadcast / HAS / CNES
   residuals are directly comparable.  (Finals → ~12–20 d latency, per the
   ptBoat dry run.)

## Expected story

- **Galileo:** broadcast ~0.9 ns → HAS recovers little additional → CNES ~0.
  HAS's own noise (≈ **20–30× CNES/BKG**, measured on the X20P bring-up) may be
  *comparable to the broadcast error itself* — the null-but-interesting result.
- **GPS:** broadcast ~1.4 ns → a more visible HAS benefit.
- **GLONASS:** absent from HAS — a clean contrast with the SSR plots.
- **E6-SIS vs IDD:** agreement, and the latency / coverage differences between
  signal-in-space and internet delivery.

## Deliverable

A `visual-stories.md` figure (broadcast · HAS-E6 · HAS-IDD · CNES, per-SV
clock+orbit, GPS + Galileo, ns scale + a Galileo zoom) and a one-paragraph
verdict on HAS's time-domain value given Galileo's already-excellent broadcast.
The free-service angle (HAS needs no terrestrial NTRIP) is the framing — not
"beat the premium SSR clock," which the X20P data says it won't.

## Implementation status

- **DONE — comparison plotter** (`scripts/plot_ssr_comparison.py`, 2026-06-12):
  overlays N per-source CSVs (`--source LABEL=csv`) as broadcast clock+orbit
  time error per SV — **constellation = colour, source = linestyle** — with a
  two-key legend, the ±5 ns full view, and a **±1 ns Galileo zoom**.  Validated
  against the CNES capture (CNES + a stand-in second source render + overlay
  correctly); real HAS-E6 / HAS-IDD CSVs slot in as added linestyles.
- **TODO — log the HAS tier.** `log_ssr_corrections.py` only reads an NTRIP
  stream; needs a `--records-file` mode that tails the HAS bridge's records
  JSON and ingests via `SSRState.update_from_records` (the engine already does
  this for `--ssr-records-file`), so the HAS-E6 / HAS-IDD tiers become CSVs the
  plotter can consume.
- **BLOCKED — the capture.** E6-SIS HAS needs the X20P on PiPuss (only lab
  receiver tracking E6); IDD needs an internet HAS feed.  Concurrent with
  broadcast eph + CNES SSR.  Hardware/agent-gated, not codeable here.
- **LATER — truth residuals.** Differencing each tier against IGS/CODE finals
  (~12–20 d latency) for the absolute "residual vs truth" view.
