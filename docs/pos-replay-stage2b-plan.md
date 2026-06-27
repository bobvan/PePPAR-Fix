# pos_replay stage 2b — regenerating `[PPP_STATE]` from a replayed bundle

Stage 2a applied the corrections (SSR/eph via the shared `route_rtcm_message`).
Stage 2b is the payoff: drive the **position filter** from a replayed bundle to
regenerate `[PPP_STATE]`/`[METAR]`, score with the `pos_sim` `DivergenceMonitor`,
and swap real-time SSR ↔ final products.

This is **the largest piece**, and it touches the engine's hot path.  This doc
sequences it into safe, separately-reviewable PRs rather than one sweeping cut —
the cost of a silent error here is a corrupted production EKF, so each extraction
lands behind the existing engine tests as a behavior-preserving refactor first,
and only then is reused by the replay.

## The two seams (both large, deeply coupled)

1. **RAWX → observations** — `realtime_ppp.serial_reader`, ~L1400–1926.
   Decodes `RXM-RAWX` into `raw_obs[sv][role]`, forms IF observations (dual-freq
   combos, BDS-L1-ref-cycle conversion, half-cycle/lock/cno fields), counts
   off-constellation/single-freq, and packs an `ObservationEvent` onto
   `obs_queue`.
   - **Risk: moderate.**  No filter-state mutation; the `continue`s are inside a
     `for sv, roles in raw_obs.items()` loop and **stay** `continue` when the
     loop moves into a function (they don't become loop-exit `return`s).
   - **Extract:** `rawx_to_observations(rawx_decoded, systems, windup_tracker,
     …) -> (observations, counts)`.  `serial_reader` calls it; the replay calls
     it on each replayed RAWX frame to rebuild the exact `(gps_time,
     observations)` the live engine queued.
   - **Do this one first** — it's the more contained extraction and unblocks the
     runner with real obs.

2. **The per-epoch EKF step** — `AntPosEstThread._run_inner`, ~L2938–3400.
   EKF predict; slip detection + per-SV phase flush; ambiguity add/remove; solid
   tide; GMF; `filt.update`; ZTD tie; NAV2 anchor; `[PPP_STATE]` emit; AR
   (MW/NL).
   - **Risk: high.**  ~500 lines, heavy `self.*` coupling, **9 `continue`s that
     become `return`s** when the body moves into a method (each is "skip the rest
     of this epoch" — semantically a `return` from a per-epoch callable, but
     every one must be converted correctly or the filter silently mis-steps).
   - **Extract:** `AntPosEstThread._process_epoch(self, gps_time, observations,
     obs_counts)`.  `_run_inner` keeps the dequeue + heartbeat + freshness gate;
     the body moves wholesale with `continue → return`.  Behavior-preserving;
     the 7 existing AntPosEst test files (`test_position_state`,
     `test_null_mode_monitor`, `test_reached_resolved_regimes`,
     `test_surgical_ztd_reset`, `test_obs_for_position`, …) are the regression,
     plus a new direct-call test.
   - **Do this one second**, as its own PR, only after RAWX→obs is in.

## Determinism (milestone-0 extension for the filter path)

Stage 1 made the UBX/TICC stores `now_mono`-pure.  The filter path adds:
- **`CorrectionFreshnessGate.accept`** — reads correction ages; confirm it's
  `recv_mono`-based (corrections carry captured `recv_mono` from stage 2a) and
  drive any wall-clock read from the virtual clock.
- **Heartbeat / `last_epoch_mono`** — diagnostic only (logging, timeout counts);
  must not influence `filt` state or the `[PPP_STATE]` line.  Audit, don't
  necessarily virtualize.
- The EKF itself is pure in `(gps_time, observations, corrections)`; `gps_time`
  comes from the obs, so the filter trajectory is a function of the bundle.

## The runner

`pos_replay_filter` (new): given the stage-1/2a `ReplayDriver` stores (corrections
populated) + the RAWX→obs reconstruction, for each epoch call the extracted
`_process_epoch` (or a thin filter wrapper), emit `[PPP_STATE]` via the shared
`format_ppp_state_line` (done — `peppar_fix/ppp_state_line.py`), and feed the
lines straight into `pos_replay_compare` (position + ZTD scoring with the
`DivergenceMonitor`).  Bob's "no point continuing" rule then applies to a
*replayed* run, not just a captured one.

## Product-swap **(done)**

`run_pos_replay(..., corrections_loader=fn)`: with a `corrections_loader`
(`callable(stores)` that populates `stores['ssr']`/`stores['beph']` from an
alternate source), `ReplayDriver(apply_captured_corrections=False)` traces the
captured SSR/eph as `<stream>:swapped-out` but does **not** apply them — so the
filter *and* the RAWX obs bias correction run against the swapped corrections
(final products instead of the captured real-time SSR).  If a ZTD drift
disappears under final products, the gap was *products*; if it persists, it's
the *filter/observability* (manifest §6).  The specific products ingest
(SP3/CLK / Bias-SINEX → `SSRState`/`BroadcastEphemeris`) is the loader the
caller supplies — the case-library step wires concrete ones.

## Product-swap orbit-representation feasibility (Charlie #245-1)

The question: with `apply_captured_corrections=False` the captured *eph* is also
swapped out, but final orbits are SP3 *tabulated* positions while `beph` is
`BroadcastEphemeris` (Keplerian).  Can the filter ingest a precise orbit?

**Answer: yes, via the interface — not via `beph`.**  The filter's orbit/clock
contract is `eph_source.sat_position(sv, t) → (pos, clk)` and
`clk_file.sat_clock(sv, t)` (`filt.update(obs, eph_source, t, clk_file=…)`).
Two implementations already satisfy it: `RealtimeCorrections(beph, ssr)`
(broadcast Keplerian + SSR delta, the live path) and `SP3` (solve_pseudorange) +
`CLKFile` (precise tabulated, the regression-harness path —
`run_regression.py` passes `SP3(args.sp3)` / `CLKFile(args.clk)` as exactly this
pair).  So a precise-products swap **swaps the corrections *object*** (SP3 +
CLKFile), it does not shoehorn SP3 into `beph`.

Implications for the concrete loaders:
- **Two swap classes, two mechanisms.**  (a) *SSR-source swap* (a different
  real-time SSR / Bias-SINEX biases over the SAME broadcast orbits) fits the
  current `corrections_loader(stores)` seam — but needs the seam refined to swap
  `ssr` while KEEPING the captured `eph` (today `apply_captured_corrections=
  False` drops both, leaving no orbits → `n_used<4`).  (b) *precise-orbit swap*
  (SP3+CLK) swaps the corrections object at `build_filter_thread`, distinct from
  the `stores` fill.
- **Biases always need an `ssr`-like provider.**  SP3/CLK carry no code/phase
  biases; the obs bias correction + AR read `ssr.get_code_bias`/`get_phase_bias`,
  so a precise swap still needs a Bias-SINEX-backed bias provider alongside the
  SP3 orbits.

So the seam is sound; the loaders split into "ssr-source" (stores fill, keep
eph) and "precise-object" (swap corrections) — a per-stream granularity the
case-library wires concretely.

## Carries (open notes to honor here)

- **#230** — obs↔PPS RAWX canonical-stamp once-over, when RAWX→obs lands.
- **#236-F2** — the replay must use the captured run's `late_edge_filter` config
  for the TIM-TM2 arm (record it in the manifest, like the #237 bias-skip flags).
- **#236-F4** — reconstruct the TICC estimator-derived fields
  (`correlation_confidence`, `estimator_residual_s`, `queue_remains`) by running
  the recv-estimator over the replayed `(source_time_s, recv_mono)` pairs before
  the filter consumes the TICC events.

## Order of work

1. `format_ppp_state_line` shared output seam **(done — #238).**
2. `rawx_to_observations` extraction (behavior-preserving) + replay wiring
   **(done — #239; sig config to replay #242; RAWX decode #243).**
3. `_process_epoch` extraction (behavior-preserving) + direct-call test
   **(done — #240).**
4. `pos_replay_filter` runner: epoch loop → `[PPP_STATE]` → compare
   **(done — drives the real `AntPosEstThread._process_epoch` inline via the
   driver's `epoch_sink`, captures `[PPP_STATE]` with `PppStateCapture`, scores
   via `pos_replay_compare`).  End-to-end `[PPP_STATE]` output needs a real
   bundle (broadcast eph for sat positions) — the wiring is unit-tested with a
   synthetic RAWX + the real filter construction.**
5. Product-swap **(done — `corrections_loader` seam)** + the case-library batch
   runner **(done — `pos_replay_case_library.run_case_library`, per-bundle guard
   for #244 note 2)**.  Remaining: concrete products loaders (the two classes
   from the feasibility note — ssr-source via stores-fill keeping eph, and
   precise-orbit via corrections-object swap with a Bias-SINEX bias provider) +
   **real captured bundles** for the end-to-end `[PPP_STATE]` / real-RAWX /
   real-eph / `.tro` confirmations (field artifacts, not in-repo).
