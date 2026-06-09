# HAS vs broadcast time-transfer comparison (DO-less X20)

The question this experiment answers: **does feeding our float-PPP time
filter Galileo HAS corrections (orbit + clock + code-bias, decoded off
the X20's E6 signal) tighten the long-τ stability of the receiver-clock
solution versus running on broadcast ephemeris alone — and by how
much?**  This is the payoff of the whole HAS thread (see
`galileo-has-research.md`): HAS is a long-τ-accuracy lever (it can't help
the short-τ rx-TCXO/DO floor, which is already below HAS clock noise).

Run on **PiPuss** — the X20 host with **no DO and no TICC** — so there's
no servo and no DO noise in the loop; the metric is purely the engine's
estimated receiver clock vs GNSS time.

## Arms

Both arms run the same engine, `--no-antposest` (FixedPosFilter pinned at
the surveyed ufo1 ARP, so the clock is the only free large state), on the
same X20 RAWX.  They differ only in the satellite corrections:

| Arm | Corrections | How |
|---|---|---|
| **broadcast** | broadcast ephemeris only | engine with `--eph-mount`, **no** SSR |
| **HAS** | broadcast eph + HAS orbit/clock/code-bias | engine + `--ssr-records-file` fed by the HAS bridge |
| *(optional)* **BKG** | broadcast eph + BKG SSRA00BKG0 SSR | engine + `--ssr-mount SSRA00BKG0` — a precise-SSR reference point |

The BKG arm is an optional sanity anchor: it shows what a mature
combined-IGS SSR product buys, bracketing the HAS result.

## Architecture (live, no serial conflict)

The engine can't import CSSRlib (heavy galois/numba), and the X20 has one
USB port, so HAS is decoded out-of-process and handed back via a file:

```
            ┌─────────────── PiPuss ───────────────┐
 X20 USB ──→│ engine (peppar-fix --no-antposest      │
            │   --ubx-out x20.ubx                    │──→ servo-log (dt_rx)
            │   --ssr-records-file has_records.json) │
            │        ▲ reads records      │ writes UBX
            │        │                    ▼
            │  has_records.json     x20.ubx
            │        ▲                    │
            │        │ writes      reads ▼
            │   tools/has_ssr_bridge.py (CSSRlib)    │
            │   E6 pages → RS decode → SSR records   │
            └────────────────────────────────────────┘
```

- Engine writes all raw UBX to `x20.ubx` (existing `--ubx-out`), reads
  SSR corrections from `has_records.json` (new `--ssr-records-file`,
  ingested via `SSRState.update_from_records`).
- `tools/has_ssr_bridge.py` tails `x20.ubx`, extracts E6 pages, RS-decodes
  HAS (CSSRlib), and writes the latest corrections to `has_records.json`.
  Reuses `has_page_monitor` (E6 extraction) + `has_decode_cssr` (RS) +
  `has_ssr_adapter.cssr_to_records`.  Only this process needs CSSRlib.

No second reader touches the serial port — the bridge consumes the
engine's UBX file.  The `--ssr-records-file` hook is source-agnostic
(any external SSR producer can write it).

### Records file schema

```json
{"epoch_s": 1217.0, "generated_mono": 12345.6,
 "records": [{"prn": "E03", "iode": 53, "orbit": [0.235,-0.352,-0.176],
              "clock": -0.168, "code_bias": {"C1C": -0.92, "C5Q": -1.64}}]}
```

## Metric

The FixedPosFilter receiver-clock estimate `dt_rx(t)`.  Compute
**TDEV(dt_rx)** on the linearly-detrended series, per arm, with
`allantools`, and compare across τ.

**Where dt_rx comes from on a DO-less host (lesson learned 2026-06-09):**
`--servo-log` and `--dt-rx-log` are both wired into the servo subsystem
(`servo_ctx`) and produce **no file when there's no `--servo`** — which
is always the case on PiPuss.  Use instead the engine's
`[FIXEDPOS_ZTD] ... dt_rx=±N ns` log line, emitted **every 30 epochs
(~30 s)** unconditionally — parse `dt_rx` from the engine log.  30 s
cadence is fine for the long-τ region this experiment targets (τ ≥ 100 s);
it gives TDEV from τ ≈ 60 s up.  (A clean 1 Hz `--clock-log` that works
without a servo is a small worthwhile engine add for a future refined run,
but isn't needed for the headline long-τ result.)

Compare across τ:

- **Short τ (≤ few s):** expect ~no difference — rx-TCXO-limited, below
  HAS clock noise.
- **Long τ (≥ ~100 s):** expect HAS < broadcast.  Broadcast satellite
  clocks have ~2 h polynomial fit intervals with discontinuities at
  uploads; those structured errors project into `dt_rx` at long τ.  HAS
  corrects the satellite clock continuously (~0.17 ns), removing that
  contaminant.

Sign convention of the HAS corrections is already validated against BKG
(`galileo-has-research.md`): Galileo slope +0.96/corr 0.98.

## Run plan

1. **Capture-free live run** is fine — both arms run live; no offline
   replay needed.  Run each arm ≥ 1 h (longer is better for τ ≥ 1000 s);
   ideally back-to-back or same overnight window for comparable sky.
   - **Disabling BKG SSR for the broadcast arm:** `ntrip.conf` carries
     `mount = SSRA00BKG0`, which the engine auto-loads as `ssr_mount` —
     so a plain run is actually the *BKG-SSR* arm, not broadcast.  Run
     with an `ntrip.conf` that has the `mount` line stripped (eph still
     comes from `eph_mount = BCEP00BKG0` in the host config):
     `grep -vi '^mount' ntrip.conf > /tmp/ntrip_nossr.conf`.  Confirm
     `SSR: 0 orbit, 0 clock` and clock tagged `[broadcast]` in the log.
   - broadcast: `peppar-fix --no-antposest --ntrip-conf /tmp/ntrip_nossr.conf`
   - HAS: start `has_ssr_bridge.py` (PiPuss has the minimal CSSRlib
     stack), then `peppar-fix --no-antposest --ntrip-conf
     /tmp/ntrip_nossr.conf --ubx-out /tmp/x20.ubx --ssr-records-file
     /tmp/has_records.json`
   - metric for both: parse `[FIXEDPOS_ZTD] ... dt_rx=` from the engine log
   - **serial conflict:** one X20 USB port → the arms run **sequentially**,
     not concurrently; a first-look comparison has a sky/time confound.
     The clean identical-obs version needs a UBX-replay harness (feed the
     captured `--ubx-out` back through the engine via a pty) — deferred.
2. Pull the servo-logs to gt; extract `dt_rx`; `allantools.tdev` per arm.
3. Compare TDEV(τ) curves; the headline number is the long-τ ratio
   (HAS/broadcast) and the τ where they diverge.

### Where the bridge runs (CSSRlib is heavy)

CSSRlib pulls galois/numba — not something to install on the Pi lightly.
Two execution options:

- **Live on PiPuss:** install CSSRlib on PiPuss and run the bridge there
  alongside the engine (HAS decode is light — a few messages/min — so the
  numba JIT cost is one-time).  True real-time.
- **Replay on the dev box (recommended):** capture the X20 UBX once on
  PiPuss (`--ubx-out`, which carries RAWX + E6 + SFRBX-for-broadcast-eph),
  pull it to gt, then run the engine in replay against it twice — once
  broadcast-only, once with the bridge (run on gt, `--ubx-file` the
  capture) feeding `--ssr-records-file`.  All heavy compute (CSSRlib +
  engine) stays on the dev box; the Pi only captures.  This also makes
  both arms run on **identical** observations, removing sky/epoch
  variation between arms.

## First result (2026-06-09) — indicative, confounded

Both arms ran on PiPuss (X20, no DO).  Broadcast arm 09:04–09:34 (60
`dt_rx` samples, SSR off, `[broadcast]`).  HAS arm ~10:02–10:17 (~12–27
usable samples once HAS engaged, `[SSR]`, rms ~0.8 m — comparable to BKG's
0.94 m, confirming the corrections apply cleanly after the bridge fix).

TDEV of linearly-detrended `dt_rx`:

| τ (s) | broadcast | HAS | HAS/bcast |
|---|---:|---:|---:|
| 30 | 4.0 ns | 3.0 ns | 0.75 |
| 60 | 11.9 ns | 4.2 ns | 0.35 |
| 120 | 23 ns | (too short) | — |
| 480 | 67 ns | — | — |

**Reading:** HAS `dt_rx` TDEV is lower, and the advantage grows with τ
(0.75 → 0.35) — directionally consistent with the hypothesis (HAS corrects
the satellite-clock errors that dominate broadcast at longer τ).

**But this is not a clean result — two dominant confounds:**

1. **The arms ran in different wall-clock windows (~1 h apart) on one USB
   port.**  On an *undisciplined* receiver clock, the detrended `dt_rx`
   TDEV is **dominated by the free-running rx-TCXO frequency wander**
   (hence the ns-level, steeply-rising values — the ~0.1–0.2 ns
   satellite-correction contribution that HAS improves is *swamped* by it).
   Different windows ⇒ different rx-TCXO drift, so the ratio could reflect
   a quieter 10:00 window as much as HAS itself.
2. **The HAS segment is short** (~6 min usable ⇒ TDEV only to τ ≈ 60 s,
   few points ⇒ noisy).

**Conclusion:** the harness works end-to-end and the first look favors
HAS, but the sequential-different-window design **cannot cleanly attribute
the improvement to HAS** — the rx-TCXO drift confound is dominant, not
minor.  The clean experiment is the **identical-observations replay**:
capture the X20 `--ubx-out` once, then reprocess the *same* bytes twice
(broadcast vs bridge-fed HAS).  With identical obs the rx-TCXO cancels and
the `dt_rx` *difference* is purely the correction contribution.  That
needs a UBX→pty replay harness (the engine's `run_replay` only does
SP3/CLK files, not our SSRState path) — the recommended next build.

## Multi-arm harness (built + validated 2026-06-09) — supersedes replay

Rather than the replay route (which needs eph+SSR capture/replay synced to
obs — eph comes from NTRIP, not the receiver — plus an engine file-replay
mode), the clean comparison is **one engine, N parallel clock filters on
identical observations** (`tools/has_multiarm_compare.py`): the X20 is read
once; each arm gets its own `SSRState` fed by a different source; pairwise
`dt_rx` differences cancel the common rx-TCXO drift.  Analyzer:
`tools/multiarm_tdev.py` (TDEV of `dt_rx_arm − dt_rx_broadcast`, detrended).

**Five-way bracket, all in one run:** broadcast · HAS (E6 SIS) · BKG
`SSRA00BKG0` · CAS `SSRA01CAS1` · CNES `SSRA00CNE0`.  Eph+BKG+CAS share the
Australian caster (`ntrip.conf`); CNES via `ntrip-cnes.conf`
(products.igs-ip.net, chunked — verified 97 SVs).  HAS arm: the external
bridge tails the comparator's `--ubx-out` → records → `--ssr-records-file`.

**Validation (5-min run): the difference method cancels rx-TCXO.** All 5
arms produced `dt_rx` on identical epochs; the std of (arm − broadcast)
was **3–47 ps**, vs the **~4000 ps** absolute TDEV of the confounded
sequential first look — a ~1000× drop confirming rx-TCXO cancellation.
Means are the inter-source clock-datum offsets (HAS +0.6, BKG −3.9, CAS
−1.6, CNES −5.2 ns).  First pairwise-difference TDEV over the HAS-engaged
portion (~3 min, indicative only):

| arm | TDEV(1s) | TDEV(16s) | TDEV(32s) |
|---|---:|---:|---:|
| HAS | 19 ps | 57 ps | 64 ps |
| BKG | 4.5 ps | 7.5 ps | 5.6 ps |
| CAS | 7 ps | 15 ps | 11 ps |
| CNES | 5 ps | 9.5 ps | 24 ps |

**Read with care:** this is a 3-min window with the HAS arm only just
engaged (still settling), so HAS's higher short-τ number is not yet
meaningful.  And TDEV(arm − broadcast) measures inter-source *agreement /
correction noise*, not absolute accuracy vs GPS time (no truth reference
here — the precise streams are assumed to cluster near truth).  The
**definitive bracket needs a long run** (HAS engaged from the start,
30–60 min) for TDEV out to τ ≈ 1000 s, plus HAS-vs-each-precise-stream
differences.  The harness and analyzer are ready for it.

## Coverage intersection (why the run is GAL-only)

The arms cover different SV/signal sets, and the comparison must account
for it.  **Two axes:**

**1. Constellation/SV coverage** (which SVs get orbit+clock):

| stream | constellations | ~SVs |
|---|---|---|
| broadcast | all the X20 tracks | all in view |
| **HAS (Phase 1)** | **GPS + GAL only** | 25 GPS + 24 GAL |
| BKG SSRA00BKG0 | GPS+GLO+GAL+BDS | ~74 |
| CNES SSRA00CNE0 | GPS+GAL(+BDS) | ~97 |
| CAS SSRA01CAS1 | GPS+GAL+BDS | broad |

The X20 tracks GPS+GAL+BDS (no GLONASS).  **HAS can't touch the X20's
BDS** (a coverage disadvantage vs the precise streams); GLONASS is moot.

**2. Per-SV signal coverage** (which signals get a code bias — needed for
the IF combination the engine forms).  The X20PDriver IF pairs are GPS
**L1CA+L5Q** and GAL **E1C+E5aQ**.  Decoded HAS code-bias signals:

| | HAS code biases | covers X20 IF? |
|---|---|---|
| GPS | C1C, C2L, C2P (**L1/L2, no L5**) | **NO** — X20 uses L1+**L5**; HAS has no GPS L5 bias |
| GAL | C1C, C5Q, C6C, C7Q (E1/E5a/E6/E5b) | **YES** — matches E1C+E5aQ |

HAS's GPS corrections are L1/L2-centric (geodetic L2W-style); an L5-band
receiver gets no GPS L5 code bias — the same class as the documented CNES
L2W-vs-L2CL / L5I-vs-L5Q mismatches (docs/f9t-firmware-capabilities.md).
The precise streams (CNES L5I→L5Q etc.) do cover GPS L5, so a GPS
HAS-vs-CNES comparison is confounded by L5 coverage (HAS-vs-broadcast is
not — L5 is uncorrected in both, so it cancels in the difference).

**How the engine handles a gap:** missing orbit/clock → broadcast fallback
for that SV; missing code bias → the signal's hardware delay stays in the
obs and lands in the IF/clock (contamination).

**Conclusion → GAL-only is the fair quality comparison.**  Galileo is the
one constellation where *all four* correction streams fully cover the
X20's tracked IF signals (E1+E5a).  The definitive run uses `--systems gal`
so every arm corrects the same SVs with the same signals — isolating
correction *quality* from coverage.  HAS's GPS-L5 gap and BDS gap are
documented coverage findings (a real-world cost of HAS on an L5 receiver),
not things to demonstrate via a confounded run.

## DEFINITIVE RESULT — GAL-only LCD, 60 min (2026-06-09)

3601 epochs on identical GAL obs (16 SVs), rx-TCXO cancelled by the
pairwise difference.  TDEV of `dt_rx_arm − dt_rx_broadcast` (ps):

| arm | 1 s | 16 s | 32 s | 256 s |
|---|---:|---:|---:|---:|
| **BKG** SSRA00BKG0 | 14 | 4.4 | 3.5 | 3.2 |
| **CNES** SSRA00CNE0 | 16 | 4.9 | 4.3 | 3.3 |
| **CAS** SSRA01CAS1 | 18 | 23 | 26 | 30 |
| **HAS** (E6 SIS) | 24 | 67 | 106 | 94 |

BKG and CNES **agree to ~1–3 ps** at long τ (the mature precise cluster).
**HAS is ~25–30× noisier (~100 ps) and CAS ~3× (~30 ps)** at the
timing-relevant τ (16–256 s).  Crucially, **HAS's ~100 ps stays well
within the moonshot per-clock budget (≤ 350 ps)** — so HAS is *usable*
for the timing mission, just measurably less precise than internet-
delivered precise SSR.  Given HAS is free and needs no internet (E6 SIS),
that's the tradeoff: ~100 ps clock-correction noise for zero
connectivity/cost.  (Datum offsets: HAS +1.0, CAS +0.5, BKG/CNES ~0 ns.)

## X20 GPS L5 config state (queried 2026-06-09)

- `CFG_SIGNAL_GPS_L5_ENA`: RAM=1, **Default=1**, Flash/BBR unset → GPS L5
  tracking is on by **factory default** (no flash override, none from us).
- `CFG_SIGNAL_HEALTH_L5` (0x10320001): RAM=0, Default=0 → L5 **health
  override OFF** → the X20 respects broadcast L5 health (tracks healthy
  L5 SVs, skips unhealthy).  We have not forced unhealthy L5.

So the X20 ships L5-enabled (contra the old F9-era "factory disables L5"
assumption), and our integration sends no signal/health config at all.

## Best-effort (Philosophy 1): per-band runs, not per-arm IF

A single run with each service on its *own* GPS band (HAS→L1+L2CL,
CNES→L1+L5) is architecturally blocked: `serial_reader` forms the IF
combination **once** (one driver's `if_pairs`) before the obs reach the
filters, so all arms share one IF band.  Per-arm IF would need an
obs-pipeline refactor (emit raw per-signal obs, form per-arm IF with
per-arm code biases).  The realizable form is **per-band runs**: hold the
global IF band, run the comparison on each band (`--gps-band`):
`IF_PAIR_PARAMS` has both GPS-L1CA+L2CL and GPS-L1CA+L5Q.  Each single-band
run is fair to the services covering that band — GAL E1+E5a (all),
GPS L1+L2CL (HAS covers; CNES L2W won't match L2CL), GPS L1+L5 (CNES
covers via L5I→L5Q; HAS has no L5).  Together they map the coverage space.

## Complete coverage-space map (GAL + GPS per-band, 2026-06-09)

TDEV of `dt_rx_arm − dt_rx_broadcast` (ps), one row per band-run:

| arm | GAL E1+E5a 16s / 256s | GPS L1+L2CL 16s / 256s | GPS L1+L5 16s / 256s |
|---|---|---|---|
| **BKG** | 4 / 3 | 5 / 27 | 3 / 32 |
| **CNES** | 5 / 3 | 9 / 97 | 6 / 50 |
| **CAS** | 23 / 30 | 6 / 19 | 7 / 56 |
| **HAS** | 67 / 94 | 35 / 136 | **diverged (nan, 53% of epochs)** |

**Findings:**
- **The mature precise streams (BKG, CNES) are excellent and self-consistent
  everywhere they cover** — sub-ps to few-ps at short τ on every band;
  they agree with each other to ~1–3 ps.  CAS is the intermediate single-AC.
- **HAS is the noisiest option where it works** (GAL ~100 ps, GPS-L2CL
  12–136 ps) but stays within the moonshot per-clock budget (≤ 350 ps).
- **HAS *cannot* do GPS L1+L5** — with no GPS L5 code bias, the IF
  combination (α≈2.26) amplifies the uncorrected L5 delay and the filter
  diverges (`nan` for 53% of epochs; the precise arms stayed finite 100%).
  **HAS's only viable GPS band on the X20 is L1+L2CL.**
- GPS long-τ TDEV rises for all arms (vs flat GAL) — GPS Rb/Cs clocks are
  harder than Galileo PHM, and the L2W-vs-L2CL mode mismatch leaks in as
  the GPS SV set changes.

**Actionable: to run HAS on the X20, configure GAL E1+E5a + GPS L1+L2CL
(NOT the X20PDriver default L1+L5, which diverges the HAS arm).**

## What these numbers are — and are NOT (read before quoting them)

The per-arm TDEV values above are the **pairwise DIFFERENCE** `dt_rx_arm −
dt_rx_broadcast`, which deliberately cancels the rx-TCXO.  They are **not**
the receiver-clock stability.  The **absolute** clock TDEV (rx-TCXO floor,
DO-less host) is:

| τ | 1 s | 16 s | 64 s | 256 s |
|---|---:|---:|---:|---:|
| absolute TDEV(dt_rx) | **65 ps** | 2.9 ns | 22 ns | 139 ns |

65 ps at 1 s matches the X20 TDCP floor (~68 ps); it rises as the
free-running rx-TCXO wanders.  The few-ps difference numbers are **below
this floor** — only the differencing (identical obs → common rx-TCXO)
makes the inter-source correction differential visible at all.

Consequences:
- **The difference RANKS the sources but is not clock stability.**  Quote
  "65 ps @1 s rising to ns" for the clock; quote the differential only as
  "how much each source perturbs the smoothed clock vs broadcast."
- **Refined reading of GAL:** broadcast ≈ BKG ≈ CNES to ~3 ps — Galileo
  *broadcast* clocks are already excellent at these τ and the precise
  corrections barely move the smoothed clock.  HAS deviates ~100 ps from
  that consensus, so on GAL **HAS is the least accurate of the four** (its
  ~100 ps is correction noise, arguably worse than broadcast), though
  still within budget.
- **On a DO-less host the source choice is moot for the delivered clock**
  (rx-TCXO dominates).  It matters only with a DO good enough to discipline
  below the satellite-error floor — the timing mission's case — so this
  differential ranking is the relevant *preview* for DO hosts, not
  something PiPuss itself would feel.

## Bottom line

Galileo HAS on the X20 is **free, no-internet, and good enough** for the
timing mission (within budget on GAL and GPS-L2), but **measurably
noisier (~20–30×) than internet-delivered precise SSR** (BKG/CNES) and
**unusable for GPS L1+L5**.  The precise streams win decisively on
quality; HAS wins decisively on operational cost.  Choose by which
constraint binds: connectivity/cost → HAS; last factor of ~30 in clock
stability → BKG/CNES.

## Success criteria

- **Primary:** HAS reduces TDEV(dt_rx) at τ ≥ 100 s by a clear margin
  (target ≥ 2×), with no regression at short τ.
- **Secondary:** HAS tracks or beats the BKG arm (it should be
  comparable — both precise SSR; HAS is the free, no-internet source).
- **Null result is informative too:** if HAS doesn't help, it means our
  broadcast-PPP long-τ clock is already correction-limited elsewhere
  (ZTD, multipath, rx-TCXO) rather than by satellite clock error.

## Caveats

- Phase 1 HAS has **no phase biases** → float PPP only (no AR).  The
  ambiguities stay float; that's fine for a static time-transfer clock.
- HAS clock datum (GST-referenced) differs from broadcast; this is a
  constant offset removed by the TDEV detrend, not a stability term.
- GPS HAS-vs-BKG agreement is looser than Galileo's (different ACs; see
  the sign-check) — a Galileo-only variant of each arm is worth running
  to isolate the cleaner constellation.
