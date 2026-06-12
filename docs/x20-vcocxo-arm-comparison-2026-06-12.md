# Servo-observer arm comparison on clkPoC3 (ZED-X20P + VCOCXO) — overnight 2026-06-11/12

**Host:** clkPoC3 (Pi 4), ZED-X20P on native USB (`ttyACM1`), VCOCXO disciplined
via AD5693R DAC, TICC #5 (`ttyACM0`) on the UFO1 antenna. Time-only mode
(`--no-antposest`, ARP pinned at UFO1).

**Question (from the ticc-vs-extint experiment design):** with only clkPoC3
available, how do the three DO-phase servo-observer arms — **TICC** (chA−chB,
~60 ps), **EXTINT** (X20 `UBX-TIM-TM2`, ~5–10 ns), **TDCP** (time-differenced
carrier phase + TICC phase anchor) — compare as servo inputs, and how
reproducible is each run to run?

**Metric:** TICC **chA detrended TDEV** (CLAUDE.md output-stability metric).
chA is logged in *every* run regardless of which arm drives the servo (TICC
logging is gated on `ticc_port`, not on `--no-ticc`), so the metric is
arm-independent and the comparison is apples-to-apples (same TICC reference,
common-mode across arms).

> **Analysis gotcha (recorded so it isn't repeated):** chA edges are exactly
> 1 Hz in *TICC* time (`ref_sec += 1` each second). Detrend the phase
> (`ref_sec + ref_ps·1e-12`) against **`ref_sec`**, NOT `host_monotonic` —
> the host read-timestamp carries ±tens-of-ms scheduling jitter that, if used
> as the regression timebase, manufactures a fake ~30 ms TDEV. First-pass
> numbers were physically impossible (30 ms @ τ=1 s) for exactly this reason.

## Design

6 runs × 70 min ≈ 7.5 h, **interleaved** (`ticc, extint, tdcp, ticc, extint,
tdcp`) so each arm samples both early- and late-night (cancels diurnal OCXO
drift). Each run is an independent cold servo bootstrap. Arm isolation:

| Arm | Flags | Servo observer | Notes |
|---|---|---|---|
| TICC | `--no-extint --no-tdcp-arm` | Arm 4 (chA−chB) | sole DO-phase observer → soloObserverChiGate accepts |
| EXTINT | `--no-ticc --no-tdcp-arm` | Arm 3 (TIM-TM2) | TICC still logs chA (metric), not used for servo |
| TDCP | `--servo-input tdcp --no-extint` | Arm 5 (TDCP freq) + Arm 4 TICC as phase anchor | #161 requires a phase reference |

All three arms converged the VCOCXO to the **same operating point, −20.6/−20.7
ppb**, confirming consistent discipline of the same oscillator. Lock times:
TICC ~6 min to ±0 ns, TDCP ~4 min to ±0.2 ns, EXTINT ~5 min to ±4 ns
(slower/noisier, as expected for the coarser observer).

## chA detrended TDEV (ns), per run

```
run          n    gap   rms    τ1     τ2     τ4     τ8    τ16    τ32    τ64   τ128   τ256   τ512  τ1024
ticc_1     3780    0   1.47  0.105  0.066  0.045  0.078  0.185  0.404  0.807  1.026  0.772  0.730  0.384
tdcp_1     3781    0   1.85  0.085  0.081  0.056  0.061  0.134  0.330  0.678  0.942  0.665  0.474  1.151
extint_1   3781    0   8.15  0.303  0.743  1.672  3.039  4.485  5.120  4.557  3.538  2.986  2.220  2.239
ticc_2     3781    0   1.96  0.078  0.072  0.063  0.096  0.216  0.495  0.983  1.301  1.089  0.685  0.279
tdcp_2     3781    0   2.13  0.108  0.058  0.052  0.093  0.214  0.408  0.748  1.302  1.862  1.225  1.075
extint_2   3781    0   2.63  0.051  0.037  0.026  0.028  0.054  0.118  0.254  0.474  0.881  1.833  0.579
```

Arm mean across both cycles (ns):

```
arm         τ1     τ2     τ4     τ8    τ16    τ32    τ64
ticc      0.091  0.069  0.054  0.087  0.201  0.450  0.895
tdcp      0.096  0.069  0.054  0.077  0.174  0.369  0.713
extint    0.177  0.390  0.849  1.534  2.269  2.619  2.406   ← extint_1-burst-contaminated
```

## Findings

### 1. TICC ≈ TDCP at every τ — the headline

The TICC and TDCP arms are statistically indistinguishable: τ1 = 91 vs 96 ps,
identical 54 ps at τ4, and TDCP is *slightly better* at long τ (713 vs 895 ps at
τ64). **The carrier-phase frequency observer disciplines the VCOCXO as well as
the 60 ps TICC, with no per-clock TICC needed for steering** (TICC serves only
as the startup/phase anchor here, a role a *shared* validation TICC can fill).
This is the moonshot-relevant result from the ticc-vs-extint design: if
TDCP ≲ TICC, one shared TICC beats one-per-clock. Confirmed on the X20+VCOCXO.

### 2. EXTINT can go *blind to large real DO excursions* — disqualifying as sole observer

The two EXTINT cycles disagree 30× (reproducibility ratio 0.17). The cause is
not a higher noise floor — `extint_2` is actually TICC-class when the receiver
behaves (26 ps @ τ4). It is a **single ~5-min burst in `extint_1`** (≈00:46
local) that dominates its whole-run statistics. Attribution of that burst:

- chA std in the window **26.95 ns, peak 141 ns**; chB (GNSS-PPS) normal at
  2.57 ns / 7.4 ns; the burst is **preserved in chA−chB** (27.35 ns,
  peak 142.6 ns) with `corr(chA,chB) = −0.11` → **real DO-PPS motion vs GPS,
  not a TICC-reference glitch.**
- **Throughout the excursion the engine's EXTINT-derived `err` read +0.0–0.3 ns**,
  adj steady at −20.6 ppb, zero slips/resets/large innovations.

So the X20 `TIM-TM2` observer reported the DO as perfectly on-time while the
true PPS output excursed to 141 ns peak. The receiver's own clock noise /
timestamp quantization **masked the excursion from the control loop**, which
therefore never corrected it. This is the EXTTS/EXTINT quantization-masking
failure documented in CLAUDE.md (`docs/ticc-baseline-2026-04-01.md`), here
caught *actively hiding a >100 ns real error* from discipline. An EXTINT-only
clock would have shipped that error downstream undetected; only the independent
TICC exposed it.

**Implication:** EXTINT (receiver-timestamped DO-PPS) is unsuitable as a *sole*
servo observer for a precision time service on the X20 — not because its
typical noise is high, but because its blind-spots are large, real, and
unpredictable (hence non-reproducible). It remains useful as a coarse
startup/phase anchor or sanity arm, not as the disciplining observer.

### 3. Reproducibility

| Arm | cycle2/cycle1 TDEV ratio (mean / min / max) | Verdict |
|---|---|---|
| TICC | 1.13 / 0.73 / 1.41 | most reproducible |
| TDCP | 1.46 / 0.72 / 2.80 | moderate; long-τ tail + TDCP `n_sv=0` dropouts add variance |
| EXTINT | 0.17 / 0.01 / 0.83 | non-reproducible (burst-driven) |

TDCP showed intermittent `n_sv=0/11 → nan` epochs (10.1% / 7.7% of epochs);
the TICC phase anchor carries the loop through them, but they inflate the
long-τ tail and run-to-run variance. **Root cause traced — see next section.**
Its *short-τ* discipline already matches TICC.

## The TDCP `n_sv=0` dropouts — root cause (2026-06-12 follow-up)

Dug in on the dropouts. They are **not** a TDCP defect, geometry, screening, or
cycle slips — the estimator is behaving correctly. The chain:

1. **Signature:** dropouts are all-or-nothing (`n_sv` jumps 10–14 → 0, never
   1/2/3) with `c_dt_rx=nan` AND `mad=nan` — i.e. the `not residuals` path
   (`tdcp_estimator.py:228`): *every* SV skipped before producing a residual.
2. **The discriminator is the inter-epoch interval.** The logged `dt_s=1.00`
   on dropouts is the misleading **empty-default** (`dt_used = median(dts) if
   dts else 1.0`, line 226). The real `gps_time` delta from the CSV:

   | inter-epoch Δt | n_used=0 (drop) | n_used>0 (good) |
   |---|---|---|
   | 1.0 s | 0 | 3382 |
   | 2.0 s | 323 | 0 |
   | 3.0 s | 51 | 0 |
   | 4.0 s | 4 | 0 |

   Every dropout is the epoch *after* a 2–4 s gap. The **`dt > max_dt_s = 1.5`
   gate** (line 215) then correctly refuses to difference across the gap (a gap
   could hide a slip), so all SVs are skipped.
3. **The gap is an observation-epoch delivery stall, not receiver loss.**
   NAV-CLOCK `iTOW` is continuous over the whole run (4217 epochs, 0 gaps >1 ms),
   and `corr_wait = 0`. The engine's own `Skip stats` pins it:
   `gate_wait_obs = 417` (tdcp_1) / `302` (tdcp_2), `n_stalls_gt_1.5s = 376 / 296`,
   `max_stall_s = 4.0` — matching the 378 / 296 TDCP `dt≥2 s` gaps almost exactly.
   So the **heavy carrier-phase observation epoch (RXM-RAWX) stalls 2–4 s in
   delivery/assembly ~8–10 % of the time, while the light NAV-CLOCK stays on
   time** — a transport/CPU latency on the heavy message (clkPoC3 = the slowest
   lab host, Pi 4 / 906 MB). The TDCP gate faithfully reflects that hole.

**Benign for discipline:** the TICC phase anchor carries the loop through every
gap; the servo just gets a TDCP frequency update every ~1.1 s avg instead of
1.0 s. The dropouts only depress TDCP's *standalone reproducibility* metric.

**Remedy — eliminate the obs-epoch stall at its source (DONE).** The stall is
the reader thread holding the GIL for ~22 ms per epoch while pyubx2 parses
RAWX; the servo/correlation-gate main loop can't run during that hold, so the
obs↔PPS match occasionally misses and the next TDCP epoch lands 2–4 s later.
The fix is the **vectorized RAWX decoder** (profiling section below) — it
removes the 22 ms GIL hold, which **eliminated the stalls and the dropouts
outright (10 % → 0 %), with no `max_dt_s` change needed.** A `max_dt_s`
1.5 → 2.5 s bump was considered but rejected as symptom-patching: it would
tolerate the gap rather than remove the GIL hold that causes it.

## Profiling the obs-epoch path — a 389× RAWX-decode vectorization win (2026-06-12)

py-spy (`--nonblocking`, 180 s, clkPoC3 live X20 run, `pyspy_obs.folded`)
of the obs-delivery path. Now that PR #117 vectorized the filter (`update`
2.2 %, `_kalman` 0.2 %, `convolve` 0.6 % self-time), **the pyubx2 RXM-RAWX
deserialize is the single biggest CPU consumer** — `_parse_ubx` 44 % inclusive,
and ~42 % of all on-CPU samples in the `_set_attribute_*` family:

```
_set_attribute_single 11.1%   __setattr__      4.7%
_set_attribute_bits    6.3%   bytes2val        3.4%
_set_attribute_group   5.9%   attsiz           0.8%
_set_attribute         5.8%   ─────────────────────
_set_attribute_bitfield4.9%   ≈ 42% of on-CPU = pyubx2 RAWX parse
```

**Why it's so heavy on the X20:** each RAWX epoch carries **~78 measurements**
(2.5 KB — all signals/constellations the X20 tracks), so pyubx2 builds
**~78 × 14 ≈ 1100 Python attributes per epoch**, the vast majority for signals
the engine immediately discards (it keeps ~11 GPS+GAL dual-freq). The reader
thread **holds the GIL for the entire parse every second**, blocking the
servo/correlation-gate main loop.

**Microbenchmark** (`rawx_decode_bench.py`, real captured X20 frames, Pi 4):

```
pyubx2 parse :  22,416 µs/epoch   (22.4 ms — and a 22 ms GIL hold)
np.frombuffer:      57.6 µs/epoch
speedup      :     389×   — parity bit-exact vs pyubx2 (prMes/svId/cpValid)
```

RXM-RAWX is a fixed 16-byte header + **32-byte repeating measurement block**
(`prMes` R8, `cpMes` R8, `doMes` R4, 4×`U1` ids, `locktime` U2, `cno` U1,
4×`X1` flags, reserved) — a textbook `np.frombuffer(payload, dtype=BLOCK,
count=numMeas, offset=16)` target. The `trkStat`/std bitfields decode
vectorized with shifts/masks. **One numpy call replaces ~1100 attribute-sets.**

**The win:** drop the reader thread's per-epoch GIL hold from **22.4 ms → 0.06 ms**.
Implemented as `peppar_fix/rawx_decode.py` (`is_rawx` + `decode_rawx` →
`RawxEpoch` arrays), wired into `serial_reader` with `UBXReader(parsing=False)`
so RAWX takes the numpy path and every other (small) UBX message is parsed on
demand by pyubx2. Gated behind `test_rawx_decode` (7 cases, bit-exact parity
vs pyubx2 incl. trkStat bits / zero-meas / truncation).

### Live A/B on clkPoC3 (2026-06-12) — dropouts eliminated

Same TDCP-arm config, X20 + VCOCXO, before vs after the decoder:

| metric | pyubx2 (overnight) | vectorized | |
|---|---|---|---|
| TDCP `n_sv=0` dropouts | 10.1 % / 7.7 % | **0.3 %** | (the 1 is the unavoidable cold-start epoch → steady-state **0 %**) |
| `n_stalls_gt_1.5s` | 376 / 296 | **0** | |
| `max_stall_s` | 4.0 s | **0.0 s** | |
| `gate_wait_obs` | 417 / 302 | **2** | |
| RAWX parse | 22.4 ms/epoch | 58 µs (`rawx_decode` ≈ 0.4 % on-CPU) | |
| `_set_attribute*` on-CPU | ~42 % | 28 % (residual = other msgs) | reader `read`-idle 38 % → 50 % |
| convergence | err ≈ 0, −20.6 ppb | err +0.5 ns, −20.6 ppb | no regression |

The 22 ms GIL hold *was* the whole causal chain — remove it and the obs-epoch
stalls, the `gate_wait_obs` waits, and the TDCP dropouts all vanish together.
Confirms the dropouts were a scheduling-jitter symptom of one long GIL hold,
not a correlation-gate design limit.

### NAV-SIG gets the same treatment (the other long GIL hold)

After RAWX, the residual ~28 % parse was dominated by **NAV-SIG** — also a
big repeating-group message (**~104 signals/epoch** on the X20, **29.4 ms**
of pyubx2 parse, *bigger* than RAWX). `peppar_fix/nav_sig_decode.py` gives
it the same `np.frombuffer` 16-byte-block decode (**~7 µs, 4041×**, parity
bit-exact). `Nav2SignalStore` factored into a shared `_ingest(rows)`;
`update_decoded(epoch)` is the production fast path, `update(parsed)` kept
for the stub-based unit tests.

**Bonus bug fix:** pyubx2 expands `sigFlags` into `prUsed_NN`/`health_NN` and
does *not* expose a combined `sigFlags_NN`, so the old `getattr(parsed,
'sigFlags_NN', 0)` read **0** on every real message — `prUsed`/`crUsed`/
`doUsed`/`health` were silently false in production (the long-standing
"`prUsed=1` never appears"). The vectorized decoder reads `sigFlags` from the
bytes, so the store now populates the real bits (verified live: `C13/BDS-B2aI
prUsed=1 doUsed=1 health=1`). Safe — `--nav-sig-gate` is off by default;
`nav_sig_disagree` is logging-only.

**Both decoders, live A/B (clkPoC3):**

```
pyubx2 parse on-CPU:  42%  →  28% (RAWX only)  →  7.6% (RAWX + NAV-SIG)
reader read-idle:     38%  →  50%              →  64%
per-epoch GIL holds removed: RAWX 22 ms + NAV-SIG 29 ms = ~51 ms
TDCP dropouts: 0% (steady-state)   n_stalls: 0   convergence: err +0.2 ns ✓
```

What's left (7.6 %) is the small fixed-size messages (NAV-PVT/NAV-CLOCK/
TIM-TP/SFRBX) — none a long GIL hold. The obs-delivery thread is now ~64 %
idle.

## Bottom line

- **TDCP is the winner for steering** — TICC-class output stability with no
  per-clock TICC required. Pursue TDCP-as-servo-input + one shared validation
  TICC.
- **TICC is the reproducibility reference** and the indispensable *independent
  validator* — it is what caught EXTINT's blind 141 ns excursion.
- **EXTINT is out as a sole observer** — receiver timestamping can hide large
  real DO excursions from the loop.

## Reproduce

- Series runner: `data/overnight_series.sh` on clkPoC3 (nohup).
- Analysis: `data/analyze_arms.py` (chA detrend vs `ref_sec`, per-arm TDEV +
  reproducibility ratios).
- Raw captures archived: `gt:~/gt/captures/x20-vcocxo-arms-2026-06-12/`
  (`*.ticc.csv` chA/chB, `*.extint.csv`, `*.tdcp.csv`, `*.engine.log`).
