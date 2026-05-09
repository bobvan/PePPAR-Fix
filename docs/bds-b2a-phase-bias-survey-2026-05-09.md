# BDS B2a-I phase bias — survey of IGS RTS mounts (2026-05-09)

## Question

When MadHat ran a `--ar-mode wl` cold-start on dual-mount (CNES O/C +
WHU bias) on 2026-05-09 morning, the AntPosEst position filter
converged into a **6–8 m basin trap** with the **ZTD residual stuck
at +1146 mm** (physically impossible — real ZTD residual is mm-scale).
A run-log spot-check showed **102 BDS `[PB_APPLIED]` emits with
`src=?` on the L5/B2a band** during the basin window — every BDS SV
contributing biased phase observations to the WL / MW / float math
without correction.

This survey was filed under dayplan `I-165118-charlie` (parent
`I-155354-main`).  The opening hypothesis was an allow-list config
bug: WHU publishes BDS B2a-I phase biases under one RINEX code
(say `L5X` or `L5I`), our engine asks under another, and the
`GAP_FILL_SIGNALS` allow-list at `scripts/ssr_corrections.py:265`
was suppressing what would otherwise be valid writes.

## What the probe found

Probed every IGS RTS mount on `products.igs-ip.net:2101` per the
[IGS RTS Products](https://igs.org/rts/products/) page, using
`scripts/probe_cas_ssr.py` (existing tool, originally built for an
analogous CAS investigation), with sample windows ranging 60 s →
15 min depending on whether earlier samples gave clean or surprising
results.

Result, **phase biases only**:

| Mount | AC | GPS | GAL | BDS | GLO | Notes |
|---|---|---|---|---|---|---|
| `SSRA00CNE0` | CNES | L1C, L2W, L5I | L1C, L5Q, L6C, L7Q | **L2I, L6I, L7I** | — | Legacy BDS only (B1I + B3I + B2I-legacy). No B2a. |
| `OSBC00WHU1` | WHU | L1C, L2W, L5Q | — | **(none)** | — | Sends RTCM 1059 / 1065 / 1242 / 1260 / 1265 only — multi-GNSS code biases plus GPS phase. No 1267 / 1270 / 4076_*. Confirmed at 60 s, 5 min, and 15 min sample windows. |
| `SSRA01CAS1` | CAS | C1C, C2W, C5I (sic — C-prefix; see parser-bug note) | C5I, C7I | **C2I (B1I), B1C** | — | IGS-SSR 4076_106; sig_ids seen are 0 (B1I) and 3 (B1C-pilot). No B2a. |
| `SSRA00CHC1` | CHC | C1C, C2W, C5I | C5I, C7I | **C2I, C7I** | — | B1I + B2I-legacy only. |
| `SSRA01SHA1` | SHAO | C1P, C2W | C5I | **(none)** | — | No BDS phase biases at all. |
| `SSRA00GFZ0` | GFZ | (none) | (none) | (none) | (none) | Stream is orbit/clock + code biases only. |
| `SSRA01GFZ0` | GFZ | (none) | (none) | (none) | (none) | Same. |
| `SSRA00DLR0` | DLR | — | — | — | — | Connection failed (no stream content). |
| `SSRA00BKG0` | BKG | (none) | (none) | (none) | (none) | Same as GFZ — no phase biases. |

**No mount publishes BDS B2a-I (= RINEX `L5I` for BDS) phase
biases.**  The IGS RTS Products page nominally describes WHU
`OSBC00WHU1` as covering "BDS-3: B1I/B1C/B2a/B2b/B3I", and the
[Geng et al. 2023 WHU phase-bias paper][geng2023] describes B2a OSBs
as a research deliverable, but the live `OSBC00WHU1` stream content
as of 2026-05-09 doesn't match that scope — it's GPS-phase + multi-
GNSS-code only.

[geng2023]: https://link.springer.com/article/10.1007/s10291-023-01610-6

## Why this stings under `--ar-mode wl`

`scripts/realtime_ppp.py:1030-1035` leaves the carrier-phase
observation **uncorrected** (effectively bias = 0) for any band whose
phase bias is MISS:

```python
if pb_f1 is not None: cp_f1 -= pb_f1 / wl_f1
if pb_f2 is not None: cp_f2 -= pb_f2 / wl_f2
ar_phase_bias_ok = (pb_f1 is not None and pb_f2 is not None)
```

The `ar_phase_bias_ok = False` flag excludes the SV from **NL fix
candidacy** (`scripts/ppp_ar.py:641`) but does **not** exclude it
from float-position math or from the MW / WL ambiguity tracker.
Under `--ar-mode wl`, NL is gone — so the gate doesn't matter — and
every BDS B2a-I-tracking SV with MISS L5 phase bias contributes a
half-bias-corrected MW combination (L1 fully corrected from CNES,
L5 uncorrected) to the WL math.  Wrong WL integer → m-scale
position bias per SV → multi-meter basin when 4–7 BDS SVs all carry
the contamination.

The `+1146 mm` ZTD residual on the basin-trap host is the smoking
gun: the filter has nowhere else to absorb the bias, so it dumps it
into the troposphere state.

## Why no AC publishes BDS B2a-I phase biases

BDS B2a is a modernized BDS-3 signal first commercially relevant
around 2020–2022.  IGS RTS analysis centers add new signals to their
phase-bias products case-by-case as they validate in-house
processing pipelines.  Publishing a phase bias requires the AC to
have processed enough reference-network observations on that signal
to compute a stable correction; that reference-network coverage
takes years to mature for any new signal.

As of 2026-05-09, every AC on IGS-IP is still publishing BDS phase
biases for the legacy signals (B1I, B3I, B2I-legacy) only.

## What about BDS PPP-B2b?

China's BeiDou-3 system broadcasts its **own** real-time SSR over
the **B2b-data signal** from BDS-3 GEO satellites.  The PPP-B2b
service includes orbit / clock / code-bias / **phase-bias** for
BDS-3 (and partial GPS), with B2a in scope.  Two practical blockers:

1. **The receiver must decode B2b** to consume it.  The F9T cannot;
   the F10T cannot; only specific multi-frequency receivers
   (e.g. some Septentrio Mosaic-X5 variants, some Trimble) decode
   the B2b-data sub-frame format.
2. **PPP-B2b uses different message types** than IGS-SSR.  The
   broadcast is a custom MT 1–7 layout defined in the BDS ICD,
   not RTCM-SSR / IGS-SSR.  Our `scripts/ssr_corrections.py`
   ingest path doesn't speak that format.

Out of scope as a near-term option for this lab.

## Secondary finding — BDS B2a *code* bias is also limited

Even at the code-bias level, F9T's BDS B2a-I (RINEX `C5I`) is
unsupported by our AC set:

| AC | BDS B2a code biases |
|---|---|
| CNES `SSRA00CNE0` | C5Q (B2a-Q) only |
| WHU `OSBC00WHU1` | C5Q + C5X (B2a-Q + B2a I+Q) |

The F9T-20B + F10T track **B2a-I** and report under `C5I`.  They
**do not** track B2a-Q, and the C5X (combined I+Q) bias doesn't
substitute cleanly because of the same I-vs-Q calibration-datum
issue documented for GPS L5I/L5Q in
[`docs/l5i-l5q-phase-bias-empirical.md`](l5i-l5q-phase-bias-empirical.md).

So even if BDS phase biases existed for our AC set, the F9T's
C5I-coded observations wouldn't match what's published on the code
side either.  BDS B2a is structurally unsupported on our hardware
at the AC layer — both code and phase fronts.

## Operational mitigation (immediate, landed)

Drop BDS from `systems=` on hosts that track BDS B2a-I:

| Host | Pre-2026-05-09 | Post |
|---|---|---|
| MadHat | `gps,gal,bds` | `gps,gal` |
| clkPoC3 | `gps,gal,bds` | `gps,gal` |

TimeHat (TIM 2.20, no L5/B2a-I tracking) and PiFace (no BDS in
config) already have `gps,gal`; unaffected.  ptpmon, otcBob1,
ocxo all already on `gps,gal`.

The branch `charlie-i-165118-drop-bds-systems` has the config
edits.  Per-host engines need a restart to pick them up; do that
after the next merge to main.

## Engine-side fix (queued as I-165118 Fix #2)

The MISS-bias path in `scripts/realtime_ppp.py:1030-1035` should
also exclude the SV from float position + MW / WL math — not just
NL fix candidacy via `ar_phase_bias_ok`.  ~2–3 h work + tests +
A/B validation.  Lands as a separate dayplan item; see the parent
discussion at `I-165118-charlie`.

After Fix #2 lands, the per-host `systems=` BDS drop becomes
optional rather than load-bearing — the engine will just refuse to
use BDS B2a-I observations rather than silently injecting bias.
With Fix #2 plus an eventual third-party BDS B2a-I phase-bias
source, BDS could be re-enabled cleanly.

## Next probe

Re-run the AC-coverage probe quarterly (or whenever an AC announces
a new signal in their product description) until BDS B2a-I phase
biases appear somewhere on IGS-IP.  Tool:
`scripts/probe_cas_ssr.py --mount <name> --duration 60`.
The summary table at the top of this doc is the form to update.

## Files

  - Branch with config edits: `charlie-i-165118-drop-bds-systems`
  - Probe tool: `scripts/probe_cas_ssr.py` (existing)
  - Diagnostic `[PB_GAP_DROP]` log instrumentation:
    `charlie-i-165118-bds-b2a-bias-allowlist @ 327d0d1` — kept for
    evidence trail; not actually needed (the gap-fill allow-list
    isn't the bug, missing AC coverage is).
  - Run-log smoking gun: MadHat `data/day0509-madhat-coldstart.log`
    + the basin-trap ZTD residual line
    `[FIXEDPOS_ZTD] residual = +1146 mm`.

## References

- [IGS RTS Products page](https://igs.org/rts/products/)
- [Geng et al. 2023 — WHU phase bias stream paper][geng2023]
- [BNC Help — BKG NTRIP Client](https://software.rtcm-ntrip.org/export/HEAD/ntrip/trunk/BNC/src/bnchelp.html)
- [`docs/l5i-l5q-phase-bias-empirical.md`](l5i-l5q-phase-bias-empirical.md) — sister analysis for GPS L5I/L5Q.
- [`docs/ac-datum-mixing.md`](ac-datum-mixing.md) — why phase biases from different ACs aren't interchangeable.
- [`docs/ssr-mount-survey.md`](ssr-mount-survey.md) — broader F9T-focused mount survey from earlier this year.
