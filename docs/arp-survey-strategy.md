# ARP survey strategy — tiered acquisition to the ±10 cm bar

Forward plan coming out of the 2026-06 free-position investigation
(see [position-drift-investigation-2026-06.md](position-drift-investigation-2026-06.md)).
Status: **plan / design** — not yet implemented.

## Why this is the real work

The investigation's load-bearing conclusion: **real-time free position is
products-limited (~dm) — the filter can't converge it to cm.** So the
operational design is right: **establish the ARP offline, then pin it**
(`--no-antposest`); the engine never free-estimates position in the timing
loop. That makes **`peppar-survey` (ARP acquisition) the load-bearing
component** for every deployment, fixed or portable.

And the bar is forgiving: the time mission needs the ARP only to **~10 cm**
(10 cm ≈ 333 ps, well under the few-ns cross-clock budget — see
[[position-accuracy-target-10cm]]). cm is a bonus (helps the moonshot's
sub-ns cross-host target), not a requirement. **So every deployment needs
*some* path to a ≤10 cm ARP — and right now several deployments (portable /
field, no nearby CORS, no patience for PRIDE-final products) have none.**

## Current state of peppar-survey

Two real backends; the NTRIP-RTK path is half-there; no self-survey:

| Backend | Accuracy | Needs | Gap |
|---|---|---|---|
| `--pride` (PRIDE PPP-AR) | ~5 mm | final/rapid products (hours–days latency), pdp3/Fortran, internet | latency; heavy install |
| `--rtklib` (rnx2rtkp PPP-static; or RTK-static vs a CORS base) | cm (RTK) / dm (PPP) | rnx2rtkp; CORS base for RTK | CORS-RTK is buried under `--rtklib --cors-ntrip-*`, not a clean backend |
| `--cors` (NTRIP CORS-RTK) | — | — | **docstring-aspirational only; not implemented** |
| self-survey | — | — | **does not exist** |

## The plan: a tiered fallback chain (best available wins)

peppar-survey should pick the **best ARP source available for this
receiver + site**, falling through to coarser-but-self-contained options:

### Tier 1 — PRIDE PPP-AR (`--pride`) — fixed sites, cm, authoritative
Already the lab default. ~5 mm from a 24 h capture + final products. Best
where the host is fixed and can wait hours–days for products. Keep as-is.

### Tier 2 — NTRIP CORS-RTK (`--cors`) — RTK-capable receivers, cm, FAST  ← *the tidy-up*
**This is the "NTRIP option for capable receivers" to add/clean up.** For an
**RTK-capable receiver** (F9P; not the timing-only F9T) within RTK baseline
of a **CORS reference reachable over NTRIP**, an RTK-static fix gives a
**cm ARP in minutes** — no product-latency wait, no Fortran. Ideal for
**portable / field** deployments with cell-NTRIP (e.g. the PiFace roadtrip:
F9P + a state-DOT/NOAA-RTN CORS mount → instant cm ARP).

Work:
- **Promote `--cors` to a first-class backend.** Today RTK-vs-CORS only
  exists as `--rtklib --cors-station / --cors-ntrip-host/port/mount`. Lift
  it into a clean `--cors` backend that streams the CORS base over NTRIP +
  runs RTK-static, and writes the standard `.survey.toml`.
- **Capability-gate it.** Only offer/auto-select it for RTK-capable
  receivers (see matrix). For F9T (timing firmware), fall through.
- **CORS discovery.** Nearest NOAA CORS / state RTN mount by approximate
  position (NAV2 / coarse fix), with baseline + age sanity gates. Config or
  auto.
- **Datum care.** CORS bases are NAD83(2011); convert to the canonical
  ITRF2020 the engine expects ([[ufo1-choke1-two-arps]] — datum + ARP
  look-alikes are a known trap).

### Tier 3 — our own PPP self-survey to ~10 cm (NEW) — last resort, self-contained
**"Our own 10 cm PPP survey if no other options exist."** When there's no
PRIDE products *and* no usable CORS (remote field, no RTN coverage), fall
back to **our own engine**: run the **RTKLIB recipe in static mode and
average over a long window**.

This is *exactly* what the recipe enables. The recipe
([[converging-config-found-20260605]]) bounds the free position **around
truth with an unbiased mean** (F9P: mean +45 mm over 6.6 h) — it just has
±0.26 m *scatter*. A **static time-average over a multi-hour window**
collapses that scatter toward the (unbiased) mean → a **≤10 cm ARP from the
receiver alone**, using only the real-time SSR (or even broadcast) it
already has. Config: `--clock-model wno --q-ztd-antpos 1e-4
--q-pos-converged 1e-9 --no-ar` (default tie), static, averaged.

Work (this is the (b) free-position work, repurposed for the self-survey —
*not* for real-time free position):
1. **Honest σ first.** The recipe is ~10× overconfident; the self-survey's
   convergence/quality gate is only trustworthy once σ tracks actual error.
   This is the prerequisite.
2. **Static-mean accumulator + quality gate.** Accumulate the static
   position over the window; declare done when the running mean is stable to
   ≤10 cm with honest σ (n_obs, σ_3d, stability/innovation checks) — spanning
   ≥1 active window so the average isn't a calm-window fluke.
3. **Write `.survey.toml`** like the other backends (same contract; only a
   peppar-survey-class backend writes survey-class state — the engine reads,
   never writes it).

## Receiver-capability matrix (drives tier selection)

| Receiver | RTK-capable (Tier 2)? | Notes |
|---|---|---|
| ZED-F9P | **Yes** | full RTK; the portable/field workhorse |
| ZED-F9T (timing) | RTK firmware varies | timing-focused; prefer Tier 1/3 unless RTK confirmed |
| NEO-F10T / others | per-unit | confirm before offering Tier 2 |

(Confirm actual RTK support per unit before wiring auto-select.)

## Band selection — capture L1/L2 when the receiver can (London-survey lesson)

**Learned from the 2026-06-12 London LEA-F9T survey:** the band set you
capture decides which post-processing products the survey can use, and a
receiver's `MOD=` string does **not** fix its band set.

- **Prefer L1/L2** (GPS L1C/A+L2C, GAL E1+E5b → classic ionosphere-free).
  L1/L2 is accepted by **CSRS-PPP, OPUS-Static, and IGS/CODE final products**,
  so it gives a genuine multi-source third opinion alongside Tier 1 PRIDE.
- **L1/L5 is the fallback**, not the default: **CSRS-PPP and OPUS won't use
  L5**, so an L1/L5 capture is limited to MGEX products + PRIDE/RTKLIB-demo5 —
  no independent OPUS/CSRS cross-check.
- **Don't trust `MOD=` for the band set.** Confirmed: ptBoat's **LEA-F9T
  (TIM 2.22) is L5-only** — it NAKs L2C/E5b/B2I even with the L5 slot freed —
  while London's **LEA-F9T-11B does L1/L2 *or* L1/L5**, switchable.  Same base
  part string, different bands.  **Probe with a CFG-VALSET (one key per VALSET)
  before relying on a band.**
- **Switching to L1/L2 on the -11B takes TWO ordered VALSETs**: (1) disable
  *all* L5-band signals (GPS L5, GAL E5a, BDS B2a, NavIC L5), *then* (2) enable
  L2C/E5b — they NAK while any L5-band signal is still on.  One combined VALSET
  fails.

**For peppar-survey:** add a receiver-prep step that probes band capability and
configures **L1/L2 if supported** (maximising product compatibility for the
survey backends), falling back to L1/L5 only when L2C/E5b NAK.  Full
ACK/NAK matrix in [`f9t-firmware-capabilities.md`](f9t-firmware-capabilities.md).

## Techniques proven on the London run (implemented in `--consensus`)

peppar-survey historically ran **exactly one backend** (`--pride` / `--rtklib`
/ `--opus` / `--cors`).  The 2026-06-12 London Mini PT 24 h survey (writeup +
`london.survey.toml` under `~/gt/peppar-survey-data/raw/london-24h-l1l2-20260612/`)
ran several and cross-checked them, and that surfaced four reusable techniques:

1. **Multi-backend consensus, not single-backend.**  Run ≥2 independent solvers
   over the *same* RINEX and adopt the agreement-gated **mean** as the survey.
   London: PRIDE PPP-AR (WUM rapid-RTS) + CSRS-PPP (EMR ultra-rapid) agreed to
   **2.3 cm 3D / 1.3 cm 2D** → mean written as the APC (σ ≈ 23 mm).
   **Implemented** as `peppar-survey --consensus pride,rtklib[,csrs]`
   (`--consensus-tol-cm`, default 3): each backend exposes a compute-only
   `solve_*_capture()` returning `(RunningArp, grade, meta)`; the consensus
   layer (`peppar_fix/peppar_survey_consensus.py`) fans out (isolated per-backend
   work/history dirs), gates on `|Δ| ≤ tol`, and writes one `.survey.toml`.

2. **Product-grade-aware: keep broadcast-eph as a *sanity check*, never in the
   mean.**  RTKLIB with broadcast eph landed **~48 cm off (almost all vertical)**
   — that's the broadcast-products *signature*, not a pipeline bug.  Form the
   consensus from **precise-product** solvers (PRIDE-RTS, CSRS) only; use a
   broadcast-eph run as a coarse gross-error cross-check and record why it was
   excluded (`rtklib_excluded_reason`).  Averaging it in would have wrecked a
   2 cm answer into 50 cm.

3. **APC-not-ARP honesty for uncalibrated antennas.**  When the antenna PCV is
   unknown (the F9T-11B antenna), survey with **ANT=NONE / zero-PCV** and record
   the result as the **antenna phase centre (APC)**, with a `kind_note`, rather
   than pretending it's the physical ARP.  Don't chase the missing antex as a
   bias.  (PCV → ARP can be applied later if/when the antenna is calibrated.)

4. **Provisional-now → finals-later, two-pass lifecycle.**  Adopt a *provisional*
   survey from NRT/rapid products immediately (so the host isn't blocked), then
   auto-schedule a **re-run when IGS/WUM finals land (~13–20 d)** for the
   authoritative value.  London adopted a provisional APC and filed
   `londonArpFinals` to re-run ~2026-06-25.  A `provisional = true` flag in
   `.survey.toml` + a finals-rerun reminder closes the loop.

**Provenance schema** (also from London): `.survey.toml` carries the quality
metadata that makes a survey auditable — `source` (which products/ACs),
`consensus_2d_cm`/`consensus_3d_cm`, per-solver coordinates, fixing rates
(`csrs_iar_pct`, NL/WL %), `frame`, `*_excluded_reason`, and `kind_note`.

**Status:** all four implemented — `--consensus` (technique 1) with broadcast-
as-sanity-only (2), `--apc` kind_note (3), and `--provisional` +
`finals_rerun_after_iso` (4); provenance written to `.survey.toml`.  Backends
gained `solve_*_capture()` (compute-only) without changing single-backend
behavior.  Remaining: CSRS auto-submit (today `--csrs-result` ingests an
operator-transcribed TOML stub, since the CSRS REST endpoint is unreliable);
`capture_{start,end}_iso` from the RINEX header.

## Selection logic

```
ARP needed (≤10 cm)
 ├─ fixed site + can wait + products available → Tier 1 (PRIDE, cm)
 ├─ RTK-capable rx + CORS reachable over NTRIP  → Tier 2 (CORS-RTK, cm, fast)
 └─ else (no products, no CORS)                 → Tier 3 (self-PPP static-avg, ≤10 cm)
```

All three write the same `.survey.toml`; the engine pins it via
`--no-antposest`. The chain guarantees **every** deployment — including
portable ones with neither products nor CORS — has a path to a ≤10 cm ARP.

## Open questions / next steps

- Tier 2: confirm per-receiver RTK capability; pick the CORS/RTN source(s)
  + credentials; baseline limit; NAD83→ITRF2020 conversion path.
- Tier 3: the honest-σ fix (prerequisite); the static-mean quality-gate
  thresholds; how long a window reliably hits ≤10 cm (the recipe residual
  is products-limited, so the answer depends on capture length × correction
  grade — characterize it).
- peppar-survey ergonomics: auto-select the tier vs explicit flag; keep the
  install lean (Tier 2/3 shouldn't require the PRIDE Fortran stack).
- **Receiver band-prep (London lesson):** probe band capability and configure
  **L1/L2 when supported** (CSRS/OPUS/IGS-final compatible) before capture,
  falling back to L1/L5 only on L2C/E5b NAK; implement the two-ordered-VALSET
  switch (disable all L5-band first).  Don't infer bands from `MOD=`.
- Cross-ref: pickyEaterSSR (I-210733) — making non-CNES ACs digest would
  also widen Tier 3's correction options.
