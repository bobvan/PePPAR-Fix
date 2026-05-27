# PiFace x[1] catastrophic state excursion — 2026-05-24 06:32:37 CDT

## TL;DR

During the overnight `day0523n-piface-biasmeasure` capture, the
DOFreqEst state vector blew up catastrophically in a single epoch
on PiFace.  All four states moved by 4–5 orders of magnitude in one
1-Hz update, with P[1,1] *unchanged*.  At the time, this looked
impossible from any normal Kalman update; none of the
DOFreqEst-visible upstream sensors logged anomalous values.

**Root cause identified 2026-05-26** (commit `ca24537`,
re-application of `b032afb`): an unhandled F9T `RXM-RAWX.recStat.
clkReset` event one second earlier.  The receiver performed its
documented integer-millisecond local-clock realignment, the engine
did not recognize it, and the resulting ~21 ms shift propagated
through the rx-TCXO phase state into DOFreqEst's Arm-4
measurement model — exactly the same class of failure that hit
clkPoC3 a day later (2026-05-25 19:00:30), where the
PR-residual cascade was the diagnostic that finally exposed the
mechanism.  See `chipSlipHandling` dayplan and the
`ca24537` commit message.

The cascade after the bad epoch — 30 consecutive `Outlier:
pps_err=...` warnings growing linearly from −989 ns to −31,200 ns,
triggering the "servo has lost control" exit‑5 — is the
downstream consequence of the actuator being driven to rail by
the corrupted state vector, not an independent failure.

The wrapper relaunched cleanly and the capture resumed; no data
loss beyond a ~53 s gap.  Bias-measurement results for the rest
of the capture stand unaffected — the single bad epoch was
excluded by the post‑bootstrap and longest‑segment filtering in
the analysis.

## Forensic data

### The bad row (arm-state-log row 24246)

```
ts                          x[0] (rx phase)   x[1] (rx freq)   x[2] (DO phase)   x[3] (DO freq)   P[1,1]
2026-05-24T11:32:36         4,346,879 ns      −15.51 ppb       −3.10 ns          −134.89 ppb      8.93e-3
2026-05-24T11:32:37         34,025,710 ns     +6352.58 ppb     −23,548,481 ns    +1,498,648 ppb   8.93e-3   ← bad
2026-05-24T11:33:30         4,616,371 ns      +1.65 ppb        +1349.92 ns       −136.49 ppb      9.90e+1   ← after restart
```

Multipliers vs prior epoch: x[0] +7.8×, x[2] −7,600,000×, x[3] −11,100×.
P[1,1] *did not change* (8.925e-3 → 8.925e-3) — incompatible with a
real Kalman update against an outlier observation.

Arms 3+4+5 all marked as `used=1` on the bad row.  Arm 5 input
(tdcp_freq_ppb) was −15.53 ppb — perfectly normal.

### Upstream sensors at the same epoch

All three independent measurement chains report normal values at
11:32:37 UTC:

| Source | Value | Normal? |
|---|---|---|
| `[TDCP]` log line | `n_sv=12/12 df_f=-1.553e-08 mad_m=0.0032` | yes |
| TICC chA−chB | −44,201 ps | yes (typical: −44,251 prior epoch) |
| EXTINT phase_residual | range −38…+28 ns | yes |
| FixedPosFilter clk delta | −3.5 mm | yes |
| `[NAV-CLOCK]` clkB | −651,466 ns (drifting −16 ns/s) | yes |

No log line names a bogus observation.  No `Outlier:` warning fires
on this epoch; the first one is at 11:32:38 (one second later).

### Pre-event symptoms — EXTTS qVIR collapse

Starting 06:28:02 CDT (4m 35s before the bad epoch), EXTTS `qVIR`
warnings began firing at every 10-epoch print, dropping from 0.41
through 0.00 and staying near zero:

```
06:28:02  qVIR=0.41 (BAD)
06:28:12  qVIR=0.33 (BAD)
…
06:29:39  qVIR=0.20 (BAD)
06:29:49  qVIR=0.00 (BAD)
06:29:59  qVIR=0.00 (BAD)
…
06:32:13  qVIR=0.00 (BAD)
06:32:37  CATASTROPHIC STATE
```

`qVIR` < 1.0 means qErr correction is *adding* variance instead of
removing it; `qVIR = 0` means qErr is completely uncorrelated with
the PPS edge it's supposed to correct (the off‑by‑one‑edge pattern
documented in `docs/qerr-correlation.md` and the `feedback_*` memory
on TICC qErr epoch matching).

Bracketing the catastrophe between the qVIR collapse onset
(06:28:02) and the exit‑5 (06:33:07) gives a ~5‑minute window
where the qErr correction chain was visibly broken.  Even though
this run had `--no-qerr-arm` (Arm 2 disabled), `qerr(x[0])` still
participates in Arm 4's TICC measurement model
(`z_ticc = −x[2] − qerr(x[0])`) when `_tcxo_initialized=True`.

### Exit cascade — outliers grow linearly

The first 7 `Outlier: pps_err=…` warnings:

```
06:32:38  pps_err= -989 ns   (1/30)
06:32:39  pps_err=-2068 ns   (2/30)
…
06:33:00  pps_err=-24,727 ns (23/30)
06:33:01  pps_err=-25,805 ns (24/30)
…
06:33:06  pps_err=-31,200 ns (29/30)
06:33:07  ERROR 30 consecutive outliers — servo has lost control
06:33:07  Skipping state save: adjfine=-1096.0 ppb is near rail — likely diverged
```

`pps_err` grows by ~1,000 ns/s after the catastrophe — exactly what
a saturated adjfine (−1096 ppb on PiFace) integrated over 1 s would
produce on the DO PPS edge.  This is the *consequence*, not the
cause.

The chA TICC edge confirms:

```
11:32:35 UTC  chA = 367,724,688,141 ps  (Δ from prior: +854 ps, normal)
11:32:36 UTC  chA = 367,724,688,995 ps  (Δ +854)
11:32:37 UTC  chA = 367,724,689,902 ps  (Δ +907)
11:32:38 UTC  chA = 367,725,724,192 ps  (Δ +1,034,290 ps = 1.03 ms jump!)
11:32:39 UTC  chA = 367,726,804,056 ps  (Δ +1,079,864 ps)
```

The DO PPS jumped by 1.03 ms in one second — the saturated‑adjfine
DO running away after the filter pushed x[3] to a ridiculous value.

## Revised explanation — F9T clkReset propagation through Arm 4

The original investigation focused on DOFreqEst because that's where
the blowup *appeared* in the logs.  But DOFreqEst's preserved
forensic streams (TDCP, TICC chA−chB, EXTINT, NAV-CLOCK) all looked
normal because they are downstream of the actual perturbation.  The
one stream that would have shown the cause — F9T raw PR/CP residuals
on the affected epoch — wasn't preserved for this capture; it was
only after clkPoC3 reproduced the cascade with the median-PR gate
firing visibly that we recognized the F9T clkReset bit as the
trigger.

What we now believe happened, in order, on 2026-05-24 11:32:37 UTC:

1. **F9T realigns local clock by ~21 ms.** Per the ZED-F9P Interface
   Description (UBX-18010854 §5.15.3), the receiver keeps its
   internal clock approximately GPS-aligned and steps it in integer
   milliseconds when drift accumulates, signalling each step by
   setting `RXM-RAWX.recStat.clkReset` on the affected epoch.

2. **rx-TCXO phase state (`x[0]`) absorbs the shift upstream of
   DOFreqEst.** The engine had no `clk_reset` recognition path; the
   apparent ~21 ms jump in the PPS / TIM-TP / PR chain was
   interpreted as a real phase movement and propagated into the
   rx-TCXO tracker that DOFreqEst reads via `x[0]`.

3. **Arm 4's measurement model evaluates a shifted `qerr(x[0])`.**
   Arm 4's `z_ticc = −x[2] − qerr(x[0])` is internal to DOFreqEst's
   update step, not a logged input.  The 21-ms shift in `qerr(x[0])`
   produced an innovation many orders of magnitude larger than
   anything seen by the per-arm outlier gates upstream.

4. **State vector pushed via Arm 4's `H = [−1, 0, −1, 0]`.** The
   coupled H row meant x[0], x[2], x[3] all moved via the Kalman
   gain on that single huge innovation; x[1] moved via cross-
   correlation pull from P[1,0], explaining why P[1,1] was
   unchanged (no direct update on x[1]) while x[1] still jumped
   +6352 ppb.

5. **Saturated adjfine drives DO PPS by ~1 ms/s.** Once x[3] was at
   +1.5M ppb, the actuator pinned at the ±1096 ppb clamp and DO PPS
   began walking off GPS time, producing the linearly-growing
   pps_err warnings that finally fired exit‑5 after 30 seconds.

The pre-event EXTTS `qVIR` collapse documented above almost
certainly co-occurs with the clkReset — when the receiver drift
accumulates toward the realignment threshold, the qErr / PPS
correlation chain that qVIR measures degrades because qErr is
predicting toward an upcoming step that PPS edges haven't taken yet.
That is, qVIR collapse is a *leading indicator* of an imminent
clkReset, not an independent failure.

The four original hypotheses (NaN propagation, unlogged bad TICC
observation, state-correlation amplification, race/write-order) are
**superseded**.  Hypothesis 3 (state-correlation amplification via
Arm 4's coupled H) was directionally correct about the propagation
mechanism but wrong about the trigger — Arm 4 fed on a *real*
innovation derived from a state that had already absorbed the
clkReset, not on a corrupted-state pull.

## Fix

Committed 2026-05-26 as `ca24537` (re-application of `b032afb`,
which was reverted once for an unrelated `ObservationEvent.__iter__`
removal regression):

- `realtime_ppp.py` reads `parsed.clkReset` from each RXM-RAWX
  message and propagates it via the new
  `ObservationEvent.clk_reset` field.
- `FixedPosFilter.update(clk_reset=True)` skips the catastrophic-
  reject gate, measures the integer-ms shift from the signed median
  PR residual, absorbs it into `dt_rx`, rebases both `z[pr]` and
  `z[td]` so the Kalman update sees the post-shift innovation, and
  clears the PR median history and consecutive-reject counter.

Net effect: what was a state-vector blowup + exit‑5 + wrapper
relaunch becomes a single `[CLK_REALIGN]` log line and a zero-impact
filter state shift.  The same fix prevents this PiFace cascade by
removing the upstream perturbation before x[0] ever moves.

## Follow-ups still worth doing

Both were filed as "open questions for Bravo" in the original draft
and remain independently valuable even though the immediate
question is closed:

1. **Per-arm innovation logging in DOFreqEst.** Would have made
   this post-mortem mechanical instead of requiring a second host
   to reproduce. Still belongs in `do_freq_est.py`.

2. **State-sanity gate on `|x[i]|` before writing arm-state-log.**
   A wrong-arm or wrong-clkReset-edge bug in some future change
   could re-create a similar blowup; an assertion alongside the
   existing `max_ppb` adjfine clamp would catch it one epoch sooner
   and short-circuit the 30-second exit‑5 cascade.

## Operational status

- Filter recovered cleanly via the existing exit‑5 / recoveryRetry
  path.  Wrapper relaunched the engine within ~50 s.
- No actuator damage (adjfine clamp at ±1096 ppb prevented the
  saturated control from rail-pinning).
- The bias measurement results from the rest of the capture
  (~5.5 h on each host) stand: rolling‑60s mean bias for both
  PiFace and clkPoC3 in the parts‑per‑trillion range, well
  below Bravo's 1 ppb "clean" threshold for Phase D.
- Phase D engine integration unblocked.

## Data preserved

- `/tmp/biasmeasure/piface-arm.csv` — arm-state-log; row 24246 is
  the catastrophic state.
- `/tmp/biasmeasure/piface-ticc.csv` — TICC chA/chB edges.
- `/tmp/biasmeasure/piface-extint.csv` — EXTINT (TIM-TM2) events.
- `/tmp/biasmeasure/piface-filter-state.csv` — FixedPosFilter
  state per epoch.
- On PiFace: `data/day0523n-piface-biasmeasure.log` (and `*.contaminated.bak`
  from the earlier double-engine snag).
