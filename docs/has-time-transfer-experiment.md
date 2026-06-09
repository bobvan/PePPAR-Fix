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
