# Known-good observations — subtracting the clock out of the error budget

*A validation technique (courtesy of Time Nut **Ole Petter Rønningen**) plus
the NTRIP stream-name decoder ring, so correction/observation mount names stop
being opaque.  Complements [`simulators-and-replay.md`](simulators-and-replay.md)
and [`pos-replay-capture-manifest.md`](pos-replay-capture-manifest.md): those
build a **synthetic/replay** reference; this uses a **free, real-world** one.*

## The idea

Run our PPP on **observations from a receiver whose clock is a national
timescale**.  Since that clock is truth at the ps level, any junk in the clock
bias our filter computes **cannot have come from the clock** — it came from our
corrections or our processing.  It's a subtraction: delete the clock as a
suspect, and the residual fingerprints the *correction + filter* chain.

For us it deletes even more than the clock in one move: a geodetic reference
station also removes the **DO**, the **rx TCXO**, the **antenna**, and
**multipath** (choke-ring / mm-known ARP) from the error budget.  What's left is
exactly the pair we keep struggling to separate in the position-drift and
SSR-bias work — *is it our SSR, or is it our filter?*

**Framing caveat — this is a diagnostic, not a production input, and not a
differential.**  Ole optimizes stability *as an end* (post-processed vs a
national lab).  We chase stability *as a means* to two-clock PPS agreement, in
real time.  These streams validate the single-ended PPP clock solution feeding
one clock; they can **never** be one of our two clocks (we don't own their
PPS/DO), so they can't stand in for the shared-antenna TICC differential.  And
they don't touch the moonshot's actual limiter (DO free-running noise +
actuator resolution) — the measurement/correction chain they exercise is
already deep inside budget (TD-CP ~5–10 ps ≪ the ns-scale DO terms).

## The two known-good stations

| Station | Clock | Operator | Receiver | Where |
|---|---|---|---|---|
| `PTBB00DEU0` | UTC(PTB) | PTB, Braunschweig 🇩🇪 | SEPT POLARX5TR | igs-ip.net, euref-ip.net |
| `BRUX00BEL0` | UTC(ORB) | Royal Obs. Belgium 🇧🇪 | SEPT POLARX5TR | igs-ip.net, euref-ip.net |

Both feed BIPM's TAI/UTC, so their receiver/clock behaviour is independently
documented after the fact (Circular T) — the belt-and-suspenders check that the
station itself wasn't misbehaving during your window.  Both broadcast **RTCM 3.3
MSM7** (1077/1087/1097/1127 = GPS/GLO/GAL/BDS), **1006** (station ARP, so the
position is in-band), and broadcast ephemeris (1019/1020/1042/1044/1045/1046).
Basic auth required.

## What we already have (nothing to build)

- **Credentials:** `bob` on `igs-ip.net` **and** `euref-ip.net` (both carry
  PTBB + BRUX).  Verified streaming 2026-07-22 from PiFace — both return
  `d3 00 15 3e e0…` (RTCM msg 1006).
- **Decode:** `pyrtcm` (core dep) decodes MSM7; we already decode MSM4/5
  elsewhere.
- **Engine wiring:** the RTCM-MSM obs-source seam (I-030423, PRs #278/#281/#283/#286)
  is already in the engine — `--obs-ntrip-mount` fills the obs queue instead of
  a serial receiver.  See [`rtcm-msm-obs-ingest.md`](rtcm-msm-obs-ingest.md).

## How to run it

Obs + eph must share `--ntrip-caster` (both on igs-ip.net); SSR comes from a
separate caster via `--ssr-ntrip-conf`.  Use BRUX as the eph mount (its in-band
1019/1045/1046) so we don't open two connections to PTBB.  Pin the position to
the station's known ARP and go time-only — that's Ole's exact setup: fixed
truth position, watch the clock.

```sh
# obs.conf → [ntrip] caster=igs-ip.net port=2101 user=bob   password=… tls=false
# ssr.conf → [ntrip] caster=ntrip.data.gnss.ga.gov.au port=443 user=bobvan \
#            password=… mount=SSRA00BKG0 tls=true

python scripts/peppar_fix_engine.py \
  --ntrip-conf obs.conf --obs-ntrip-mount PTBB00DEU0 --eph-mount BRUX00BEL0 \
  --ssr-ntrip-conf ssr.conf \
  --known-pos "<lat,lon,alt of the station ARP>" --known-pos-frame ITRF2020 \
  --no-antposest --no-do --no-ticc --ignore-arp-state --systems gps,gal \
  --filter-state-log state.csv --dt-rx-log dtrx.csv
```

Get the station ARP from the in-band 1006 (ECEF → geodetic via pyproj
EPSG:4978→4979), or from the IGS site log.  PTBB ARP (2026): ECEF
`(3844059.79, 709661.54, 5023129.74)` → `52.296191, 10.459750, 130.31`.

**Reading the result:** with the position pinned and the clock = truth, the
`dt_rx` / filter-state clock bias should be a clean line.  Wander that survives
is *our SSR + our filter*.  Swap the SSR mount (BKG ↔ CNES ↔ WHU gap-fill) and
re-run to attribute wander to a specific correction stream — the direct feed to
`pickyEaterSSR` and the filter-stiffness work.

## NTRIP stream-name decoder ring

BKG's product/observation mounts are a 9-char long-name + a trailing stream
digit: **4-char** content/site · **2-digit** monument+receiver (usually `00`) ·
**3-char** country *or* agency · **stream digit**.

| Mount | 4-char | 3-char | Kind |
|---|---|---|---|
| `ptbb00deu0` | PTBB (station) | **DEU** (country) | **observations**, PTB 🇩🇪 |
| `brux00bel0` | BRUX (station) | **BEL** (country) | **observations**, ROB 🇧🇪 |
| `algo00can0` | ALGO (station) | **CAN** (country) | **observations**, Algonquin 🇨🇦 |
| `SSRA00BKG0` | SSRA (SSR stream A) | **BKG** (agency) | **corrections** (orbit/clock/bias) |
| `BCEP00BKG0` | BCEP (broadcast eph) | **BKG** (agency) | nav data |
| `OSBC00WHU1` | OSBC (obs-specific bias) | **WHU** (agency) | bias corrections (L5 gap-fill) |

**Rule of thumb:** 3-char field is a **country** (DEU/BEL/CAN) → a physical
station streaming *observations* (raw code+phase, RTCM MSM).  3-char field is an
**agency** (BKG/WHU/CNE) → a computed *product* (SSR corrections, ephemeris,
biases).  The two are orthogonal NEC content types — you apply *corrections* to
*observations*.  Our production diet is corrections (`SSRA00BKG0` + `BCEP` +
`OSBC` gap-fill); this technique adds *observations* as a diagnostic input.

## See also

- [`gpsdo-noise-and-external-clock.md`](gpsdo-noise-and-external-clock.md) — the
  external-clock architecture MSM ingest unblocks; why measurement noise isn't
  our limiter.
- [`rtcm-msm-obs-ingest.md`](rtcm-msm-obs-ingest.md) — the MSM→obs-model adapter
  this rides on.
- [`ac-datum-mixing.md`](ac-datum-mixing.md) — why swapping SSR mounts between
  runs is legitimate for float-PPP attribution but not for shared-signal AR.
