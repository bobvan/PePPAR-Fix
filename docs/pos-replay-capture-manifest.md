# pos_replay — reference-capture manifest

*Design doc, 2026-06-25.  The concrete capture spec for `pos_replay`
(the captured-replay harness defined in
[simulators-and-replay.md](simulators-and-replay.md)).  It says exactly
what a **reference capture** logs, how it's timestamped, how truth is
derived, and the conventions that make the comparison honest — so a 24 h
run produces a reusable, replayable artifact with an external answer key.*

A reference capture has two halves:

> **{ logged inputs }** (everything the position filter sees, raw, with a
> shared timescale) **+** **{ truth }** (an independently post-processed
> position and ZTD with confidence intervals).

`pos_replay` then replays the inputs through the **real engine** and
scores the trajectory against truth with the divergence monitor from
`pos_sim` (far AND still growing → abort).


## 0. What filter we're capturing for

`pos_replay` studies the **position-estimating** path (AntPosEst /
`PPPFilter`) — the filter with the (pos,ZTD,clk) null.  The recommended
*operational* mode is `--no-antposest` (pinned ARP, `FixedPosFilter`),
which does **not** estimate position.  So a reference-capture run is a
**diagnostic** run that deliberately enables the position filter, and the
engine-output log must carry **`PPPFilter`** position + ZTD state and σ
(not just the pinned-clock `FixedPosFilter` internals).


## 1. Capture principles

1. **Tap raw + timestamp at the wire.**  Capture the raw byte streams and
   stamp each against `CLOCK_MONOTONIC` (the only shared timescale —
   [stream-timescale-correlation.md](stream-timescale-correlation.md)).
   Replaying raw re-runs the *whole* pipeline (decoders, the obs↔PPS
   correlation gate, the filter) and lets us vary upstream.  Derive a
   decoded "filter-input record" from the raw if a lighter artifact is
   wanted — but the raw is the source of truth.
2. **Log our outputs too.**  The filter's per-epoch estimate **and its
   reported σ** (P-matrix diagonal) are needed for the `error/own-σ`
   (false-confidence) half of the score.
3. **Derive truth offline.**  Post-processing is not part of the live
   run; it consumes the captured observations afterward.
4. **One self-describing bundle.**  Every capture carries a `manifest.toml`
   pinning provenance, conventions, software versions, and the antenna
   reference height — so a replay years later is unambiguous.


## 2. Group A — input streams (for deterministic replay)

These are what the engine ingests.  Readers already stamp `recv_mono`
and queue state (`full-data-flow.md` S1–S5); the capture writes the raw
payloads plus that monotonic stamp.

| Stream | Source | Capture form | Rate | Timestamp |
|---|---|---|---|---|
| GNSS UBX (RAWX, SFRBX, NAV-CLOCK, NAV-PVT, NAV-SAT, NAV-SIG, TIM-TP) | receiver serial/USB | **raw UBX bytes** | 1 Hz (RAWX) | `recv_mono` per message |
| RTCM SSR (orbit/clock/code-bias/phase-bias) | NTRIP caster | **raw RTCM3 bytes** | ~varies | `recv_mono` per frame |
| Broadcast ephemeris (RTCM 1019/1042/1046 or UBX SFRBX) | NTRIP `BCEP` mount / receiver | **raw bytes** | sparse | `recv_mono` |
| TICC chA/chB lines (DO-PPS, GNSS-PPS) | TICC serial | raw lines | 1 Hz | `recv_mono` per line |
| EXTTS / qErr (if present) | PHC / TIM-TP | **raw TIM-TP / EXTTS + `recv_mono`** (not the already-correlated form) | 1 Hz | `recv_mono` per message |

Notes:
- The **obs↔PPS correlation** is timing-sensitive; capturing each
  stream's `recv_mono` is what lets the replay reproduce the gate's
  matching decisions deterministically.  Do **not** collapse streams to a
  single re-ordered log.
- RAWX is the seed of the **truth pipeline** (§4) as well as the filter
  input — one capture serves both.
- **Everything is captured raw, including qErr/EXTTS.**  Capturing the
  *already-correlated* qErr↔edge form would bake in the live match
  decision and prevent `pos_replay` from re-deriving or ablating it —
  contradicting the capture-raw principle.  Capture raw TIM-TP +
  `recv_mono`; the replay re-runs `match_pps_mono` (the gnss-pps-qErr
  work already made this match replayable from raw + `recv_mono`).


## 3. Group B — engine outputs (for scoring)

| What | Existing hook | Gap to close |
|---|---|---|
| Per-epoch position estimate (ECEF/ENU) **+ position σ** | `--filter-state-log` | today it logs `FixedPosFilter` (pinned-clock) internals; **extend to dump `PPPFilter` pos E/N/U + √P_pos and ZTD + √P_ztd** |
| Per-epoch residual ZTD estimate **+ ZTD σ** | `--dt-rx-log` writes `ztd_mm` on some runs | make it **always-on** in capture mode, with σ |
| Per-SV residuals, admit/reject, slip flags | `--per-sv-resid-log`, `--slip-log` | reuse as-is (diagnostic context) |
| Correction provenance (which AC, age, gaps) | `log_ssr_corrections.py` | run alongside; tags the product-quality axis |

The position σ and ZTD σ are **load-bearing** — without them the
`error/own-σ` false-confidence metric (the signature of misfit hidden in
the null) is not computable.


## 4. Group C — environment + the truth pipeline

### Environment
| What | Source | Why |
|---|---|---|
| Surface pressure / temperature / dewpoint (METAR) | nearest METAR station | **ZHD** (hydrostatic delay) — the bookkeeping anchor for the ZTD comparison (§5) |
| Antenna reference height, ARP, antenna model | `timelab/antennas.json` | ZTD is height-dependent; ARP is the position truth |

The engine already derives Saastamoinen ZHD/ZWD from METAR at startup
(`_seed_ztd_from_metar` → `[INIT_ZTD]` line, `saastamoinen.py`).  **Gap:**
log station pressure **periodically** through the run (METAR updates
~hourly, so hourly is sufficient), not only at init.

### Truth (derived offline from the captured RAWX → RINEX)
The captured RAWX becomes a RINEX obs file via the existing
`peppar_fix/rinex_writer.py` (the peppar-survey `--pride` path), then
feeds three independent post-processors with distinct roles:

| Backend | Constellations | Role |
|---|---|---|
| **PRIDE-PPP-AR** (local) | multi-GNSS | epoch-wise kinematic reference time series; same product *class* we can also feed the filter (product-matched replay) |
| **OPUS-Static** | **GPS-only** | static position anchor **and the GPS-only control** for the constellation question |
| **NRCan CSRS-PPP** | multi-GNSS | position time series **and the crown jewel: ZTD(t) with a CI** |
| Surveyed ARP | — | `timelab/antennas.json`, OPUS multi-day, σ≈12 mm — the one sub-cm position truth we already hold |

OPUS-S (GPS) and NRCan (multi-GNSS) **bracket** the GPS-vs-Galileo
question that `pos_sim` localized to the observation side.


## 5. The ZTD-truth convention (the crown jewel — and its traps)

The external NRCan ZTD(t) is the observable `pos_sim` cannot honestly
provide.  The comparison only means something if the conventions match;
pin these in `manifest.toml` and the compare tool:

1. **ZHD/ZWD split.**  Compare total ZTD, but compute **ZHD from the
   captured station pressure** (Saastamoinen, the same path the engine's
   `_seed_ztd_from_metar` uses) so the hydrostatic part is consistent;
   the estimated (wet) part is what's really under test.
2. **Mapping function.**  Record which mapping the engine used (GMF/VMF vs
   `1/sin(e)`) and which NRCan reports; mismatched mappings shift the
   apparent ZTD.
3. **Antenna reference height.**  ZTD is height-dependent — both sides
   must reference the same ARP height (from `antennas.json`).
4. **Temporal grid + real-time lag.**  NRCan ZTD is batch-smoothed at its
   own cadence; our ZTD is a real-time random walk that legitimately
   *lags* the hindsight truth.  Score the lag-aware departure, not the
   instantaneous difference — a convention mismatch otherwise reads as a
   physics finding.

**Implemented** (`scripts/peppar_fix/`): `nrcan_tro_reader.py` parses the
CSRS-PPP SINEX_TRO `.tro` total-ZTD series (TROTOT mm→m, `YY:DOY:SSSSS`
epochs, `SOLUTION_FIELDS`-driven columns).  `pos_replay_compare`:
`ztd_series_from_tro` adapts it; `interpolate_ztd` puts the ~5-min truth onto
our 1 Hz `[PPP_STATE]` timestamps so the **whole** series compares 1:1 (not
just our points near each 5-min mark); `compare_ztd` removes the **constant
median (our − truth) offset** — which absorbs *both* trap 4's real-time lag
*and* the residual-vs-total apriori difference (traps 1–3's constant parts) —
and scores the *detrended* departure with the shared `DivergenceMonitor`.
The `[PPP_STATE]` line carries a `gps=` (GPS-time) key so our series joins to
the external time axis.

**Total-ZTD assembly** (`ztd_total_series_from_ppp`): the filter's *total*
zenith ZTD is `ENGINE_ZTD_APRIORI_M` (2.3 m, the fixed hydrostatic apriori the
`IDX_ZTD` residual rides on) `+ residual`.  At zenith the wet mapping is 1, so
a *total-vs-total* comparison needs no mapping function (trap 2 is a
slant-domain concern only).  Assembling the total makes the constant 2.3 m
apriori **explicit** instead of letting `compare_ztd`'s offset-removal absorb
it together with the lag — which is what would otherwise hide a genuine
*constant* total bias (a wrong antenna height → wrong ZHD, trap 3).  With
totals, the removed offset is the physical `our_total − truth_total` median
(lag + any real bias), and `abs_bias_warn_m` flags `|offset|` beyond a
threshold — a constant bias the *detrended* divergence verdict structurally
cannot see.  The `[METAR]` Saastamoinen ZHD/ZWD/ZTD (parsed, joined to
`[PPP_STATE]` by epoch) provides an **independent weather-model total**
cross-check on the truth's absolute level (trap 1), report-only — it is not
the filter's ZTD, which is what the main comparison scores.


## 6. Determinism, product-matching, and the bundle

- **Deterministic replay = a virtual `recv_mono` clock.**  Replay the raw
  streams single-threaded, in `recv_mono` order, with no
  wall-clock/RNG/threaded-numpy nondeterminism, so a fixed capture + fixed
  code → bit-identical output (the property that makes it a regression).
  The deepest risk: the live engine is multi-threaded and some
  gate/age/timeout decision may evaluate against **live
  `time.monotonic()`** (`age = now − recv_mono > max_age`) rather than the
  captured stamp.  Replay is bit-identical only if **every** such decision
  is a pure function of the captured `recv_mono` — a *virtual clock*
  driven by the replayed stream, with no hidden wall-clock read at
  decision time.  This is **build milestone 0** (§8): audit the obs↔PPS
  correlation gate, `match_pps_mono`, and all freshness/`max_age` checks
  for wall-clock dependence *before* building replay; any that read
  wall-clock must be driven from the virtual clock (or that decision point
  captured).  Otherwise it isn't a regression.
- **Product-matched replay.**  Support swapping the correction source:
  replay with the real-time SSR as captured **or** with final products
  (the truth used).  Final-products → if it converges, the gap was
  products; if it still drifts into ZTD, it's the filter/observability.
  This is the knob that separates *our filter* from *our corrections*.
- **Bundle layout** (one reference capture):
  ```
  captures/<host>-<YYYYMMDD>/
    manifest.toml          # provenance, conventions, ARP height, versions
    raw/  ubx.bin  ssr.rtcm  eph.rtcm  ticc.log   (+ .recvmono sidecars)
    obs.rnx                 # derived from raw/ubx.bin (rinex_writer)
    env/  metar.csv  antenna.json
    engine/ filter-state.csv  ssr-corrections.csv  run.log
    truth/  pride.pos  opus.txt  nrcan.pos  nrcan.ztd  arp.json
  ```


## 7. Case library

One golden day is not enough — the failures we chase are episodic.
Curate captures spanning regimes, each with truth:

- a **calm** day (baseline);
- an **active** day (large real-sky excitation);
- a **GPS+GAL-fails** day (the observation-side asymmetry `pos_sim`
  predicts must exist).

Each is one bundle; `pos_replay` iterates the library.


## 8. What to build (scope, smallest → largest)

0. **Determinism / gate-purity audit** *(milestone 0 — before replay).*
   Audit the obs↔PPS correlation gate, `match_pps_mono`, and every
   freshness/`max_age` check for **live `time.monotonic()` reads at
   decision time** (§6).  Deliver a finding: which decisions are pure
   functions of `recv_mono` vs which read wall-clock, and the plan to
   drive the latter from a virtual clock.  Read-only; gates whether
   deterministic replay (step 5) is even achievable.  *Small but
   load-bearing.*
1. **Periodic pressure log** — extend `_seed_ztd_from_metar` to log
   station pressure hourly through the run.  *Small.*
2. **Engine-output completeness** — make `--filter-state-log` (in
   position-estimating mode) dump `PPPFilter` pos E/N/U + √P_pos and
   ZTD + √P_ztd; make `--dt-rx-log` ztd+σ always-on in capture mode.
   *Small.*
3. **Unified raw capture wrapper** — a `peppar-capture` mode that writes
   the Group-A raw streams + `recv_mono` sidecars + `manifest.toml` in
   the bundle layout.  Reuses the readers that already stamp `recv_mono`.
   *Medium — the main new artifact.*
4. **Truth-ingest + ZTD-compare tooling** — RAWX→RINEX→{PRIDE,OPUS,NRCan},
   parse outputs, build the truth series, and the §5 convention-matched
   ZTD comparison.  *Medium (the ZTD convention is the subtle part).*
5. **Deterministic replay + product-swap** — extend
   `scripts/regression/run_regression.py` to consume a bundle, replay the
   raw multi-stream capture deterministically, and swap products.  Score
   with the `pos_sim` divergence monitor.  *Largest.*

Sequence 1→2 first (cheap, unblock a first capture), then 3 (capture a
real bundle), then 4+5 (truth + replay).  Per the parent doc, this is the
~month effort; `pos_sim` already covers the vacuum half.


## Open decisions

- **Raw vs decoded tap** — recommendation: capture raw + `recv_mono`,
  derive the decoded filter-input record.
- **Volume + storage** — UBX is tens of MB/h, but multi-GNSS RTCM SSR
  (orbit/clock/code+phase bias) alone can be a few KB/s → hundreds of
  MB/day, so a 24 h all-streams bundle may be **~GB-class**.  Measure
  against a real 1 h capture before committing to 24 h.  Write the bundle
  to **gt (RAIDZ)**, *not* the lab host's eMMC/SD — per the lab-storage
  rule, eMMC/SD fail without warning, and a 24 h capture is exactly the
  kind of artifact that must not live only on a lab card.
- **Which host** — a lab host with clean sky and a surveyed ARP
  (sub-cm truth already in `antennas.json`), running the position filter
  (not `--no-antposest`).
- **METAR proximity** — how near a METAR station must be before its
  pressure is a usable ZHD anchor (the engine already warns on
  pressure/temperature dissimilarity; reuse that gate).
