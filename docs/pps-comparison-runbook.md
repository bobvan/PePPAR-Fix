# PPS-comparison runbook

How to run a multi-clock PPS head-to-head (our clocks vs each other, and
vs external clocks otcBob1 / M600 / GNSSDO+), and how to keep results
comparable across capture and analysis runs.

Grading metric and budget come from
[`two-site-sync-budget.md`](two-site-sync-budget.md):

| Configuration | Bound | Probability | Window |
|---|---|---|---|
| Shared antenna (same RF via splitter) | \|Δ\| ≤ 1 ns | 95% | τ from 0.1 s to ~1000 s |
| Separate antennas, baseline ≤ 5 km | \|Δ\| ≤ 2 ns | 95% | any 1-hour window |

## 1. Wiring a head-to-head

The ref-immune differential method puts **both** clocks under test into
**one** TICC so the TICC's own reference noise cancels in the chA−chB
difference:

```
clock A  PPS OUT ──► TICC chA
clock B  PPS OUT ──► TICC chB     (same shared-reference TICC)
```

The TICC measures each PPS edge against its own internal reference; the
difference chA−chB is independent of that reference. This is the cleanest
test bed and what `compare_clocks.py` analyzes.

### PPS voltage / level caution

TICC inputs expect a clean logic-level edge. **Check the PPS OUT level of
each clock before connecting** — they are NOT all the same:

- Many u-blox EVK / GNSSDO PPS OUTs are **3.3 V** logic.
- Some appliance / OCXO clocks (and 50 Ω-terminated 1 PPS distribution)
  drive **5 V** or higher.
- Mixing a 5 V source into a 3.3 V-expecting input — or vice versa —
  risks a missed edge (too low) or input damage (too high).

When in doubt, scope the edge and add a level shifter / attenuator rather
than guessing. A mismatched level shows up as missing chA or chB rows
(the reader drops to "only N paired samples").

### Separated clocks — cross-TICC + virtual pairing

When the two clocks can't reach one TICC (different sites), use one TICC
per clock and pair virtually:

- **Shared lab Rb reference** on both TICCs →
  [`tools/plot_xhost_virtual.py`](../tools/plot_xhost_virtual.py)
  (chA − chA across hosts, linear detrend absorbs cable/boot offset).
- **Independent references** on the two TICCs →
  [`tools/plot_xhost_indep_refs.py`](../tools/plot_xhost_indep_refs.py)
  (each host's chB carries the common-view GNSS PPS; differencing the
  per-host chA−chB residuals cancels each TICC's reference). Costs
  √2 × the F9T PPS noise, so it needs longer-τ averaging.

The single-TICC method is preferred whenever both clocks can reach it.

## 2. Running `compare_clocks.py`

One command captures and analyzes a head-to-head:

```sh
# Live capture (needs the TICC hardware):
scripts/compare_clocks.py \
    --ticc /dev/ticc4 \
    --label-a ours --label-b M600 \
    --duration 3600 \
    --out-dir data/m600-headtohead \
    --budget-ns 1.0          # 1.0 = shared antenna; 2.0 = separate antennas

# Analyze an existing two-channel capture (no hardware):
scripts/compare_clocks.py \
    --from-csv data/m600-headtohead/capture.csv \
    --label-a ours --label-b M600 \
    --out-dir data/m600-headtohead \
    --budget-ns 1.0
```

Outputs in `--out-dir`:

- `agreement_cdf.png` — the excursion CDF (pass/fail metric).
- `stability_stack.png` — TDEV/ADEV overlay (the "why / at which τ").
- `tch.png` — three-cornered-hat individual-node stability (only with
  `--tch`; see §2.1).
- `verdict.txt` — one-line VERDICT plus stats, carrying the provenance
  stamp.

`compare_clocks.py` does **not** reimplement any math: it imports
`render_cdf` from
[`plot_xhost_agreement_cdf.py`](../tools/plot_xhost_agreement_cdf.py),
`render_pair_stability` from
[`plot_clock_stability_stack.py`](../tools/plot_clock_stability_stack.py),
and `render_tch` from [`plot_tch.py`](../tools/plot_tch.py).
Consistency is the whole point — the head-to-head and the standalone
plots compute identical numbers.

### 2.1 Three-cornered hat (`--tch`) — recovering individual stability past the Rb floor

The very same two-channel capture contains a **complete three-cornered
hat** over the three nodes `{chA, chB, Rb}`. The TICC measures two legs
directly — `chA − Rb` (the chA rows) and `chB − Rb` (the chB rows) — and
the third leg `chA − chB` is their per-second difference (the Rb cancels).
With all three pairwise differences the standard 3CH closed form solves
for each node's **individual** ADEV/TDEV at every τ, without needing a
reference better than the devices under test.

```bash
scripts/compare_clocks.py --from-csv data/run.csv \
    --label-a 'DO-A' --label-b 'GNSS PPS' \
    --out-dir data/run --tch            # add --groslambert for the
                                        # covariance estimator
# or standalone:
python3 tools/plot_tch.py --from-csv data/run.csv \
    --label-a 'DO-A' --label-b 'GNSS PPS' --output data/run/tch.png
```

Why it matters: per §3.1.1 of
[`two-site-sync-budget.md`](two-site-sync-budget.md), the single-clock
`chA − Rb` curve **cannot** attribute a long-τ TDEV bulge to the clock vs
the Rb's own random walk (the FE-5680A's RWFM climbs past the per-clock
budget beyond ~1000 s). TCH recovers each clock's own stability past that
Rb floor, and identifies the **free-runner** as the node whose individual
TDEV *ramps* at long τ while a locked node stays bounded.

**Independence caveat (load-bearing).** The closed form is unbiased only
if the three nodes are **mutually uncorrelated**. Two GNSS-locked clocks
sharing one antenna are correlated (common SSR/atmosphere/multipath), and
the shared noise leaks into the Rb estimate — the split is biased. Use two
**independent** disciplined oscillators (or one disciplined clock vs an
independent GNSS PPS) for a clean decomposition. When a solved component
variance goes negative (one node dominates, correlation, or too few
samples), that point is set **NaN and counted** rather than clamped to
zero — the count in `verdict.txt` flags assumption violations.

## 3. The metric

- **Pass/fail is graded on the CDF p95 excursion** vs the budget
  (`--budget-ns`). p95 |Δ| ≤ 1 ns (shared antenna) or ≤ 2 ns (separate
  antennas) is a PASS. p99 and max are reported for context.
- **TDEV/ADEV explain *why* and *at which τ*** one clock beats the other.
  TDEV is a variance metric and does **not** bound the phase-difference
  trajectory — never substitute it for the excursion grade. Use it only
  to localize where a clock's noise dominates (short-τ DO floor vs
  long-τ GPS-tracking).

## 4. Provenance policy (read before comparing any two runs)

Capture and analysis tooling get different treatment because of an
asymmetry (see the feedback memo
`feedback_capture_recapture_analysis_version_stamp`):

- **Capture tools define the DATA.** A capture-tool change invalidates
  cross-run comparability — you cannot re-derive new-capture data from
  old. `scripts/ticc_capture.py` carries `CAPTURE_VERSION` and stamps a
  `# capture_tool=ticc_capture.py capture_version=N git=<sha[-dirty]>
  utc=<iso>` comment line into each CSV header. **A `CAPTURE_VERSION`
  bump ⇒ re-capture every dataset you still intend to compare against.**

- **Analysis tools transform data → results.** Analysis can be re-run on
  the SAME captures, so the mitigation is provenance, not recapture.
  `tools/analysis_provenance.py` holds `ANALYSIS_VERSION` (a deliberate
  integer — Bob's "second thoughts" marker, bumped **by hand** whenever
  the analysis math changes). Every analysis figure is stamped
  `<tool> · analysis v<N> (<sha[-dirty]>) · <UTC>` in a small gray footer,
  and the same string is echoed to stderr / `verdict.txt`.

Rules:

- **Never compare two figures carrying different `ANALYSIS_VERSION`.**
  Re-run the older captures through the CURRENT analysis version instead
  — analysis is cheap, the captures are intact.
- The `-dirty` marker is load-bearing: a figure produced from
  uncommitted analysis edits says `-dirty`, so it can never masquerade
  as a clean version. If a campaign figure is `-dirty`, commit and
  regenerate before trusting it.
- Bump `CAPTURE_VERSION` only deliberately and rarely (it's expensive —
  forces re-capture). Examples that REQUIRE a bump: changed timestamping,
  edge handling, channel mapping, decimation, HUPCL/reset behavior.

## 5. Analysis gotchas (must stay stable across versions, or force a re-run)

These are baked into the shared analysis core; if you change any of them,
bump `ANALYSIS_VERSION`:

- **Detrend on `ref_sec`, not sample index.** A gap in the capture
  desyncs an index-based 1 Hz rate subtraction. The readers take the
  longest gap-free segment and detrend against the true second count.
- **Subtract-separately ps precision.** Per-event `total_ps =
  sec*1e12 + ps` is built in Python int (arbitrary precision); the
  pairing subtraction stays in int and is cast to int64 only AFTER the
  subtraction. Building a float64 `total_ps` quantizes at a level that
  scales with TICC uptime (~100 ps at ref_sec≈10⁶) and silently kills
  sub-ns work.
- **Longest gap-free clean window.** Bootstrap transients and engine
  restarts poison TDEV/CDF; use `--skip-before` to trim, and the readers
  pick the longest contiguous run.
- **UTC vs local.** TICC `ts_iso` is UTC; lab `run.log` timestamps are
  CDT/CST. Don't cross them when picking `--skip-before`.
- **Append-contamination.** `ticc_capture.py` appends per UTC day; a CSV
  may contain multiple runs. Confirm you're analyzing the intended
  window (the `#` capture header marks each fresh file's provenance).

## See also

- [`two-site-sync-budget.md`](two-site-sync-budget.md) — the math behind
  the 1 ns / 2 ns excursion bounds.
- [`tools/analysis_provenance.py`](../tools/analysis_provenance.py) —
  `ANALYSIS_VERSION`, `stamp`, `provenance_line`, `skip_comment_lines`.
- [`scripts/ticc_capture.py`](../scripts/ticc_capture.py) —
  `CAPTURE_VERSION` and the CSV header.
- [`tools/plot_tch.py`](../tools/plot_tch.py) — the three-cornered-hat
  decomposition over `{chA, chB, Rb}` (the `--tch` mode, §2.1).
