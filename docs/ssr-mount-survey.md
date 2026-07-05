# SSR mount survey for F9T-based PPP-AR

*2026-04-17 research summary — what mounts other F9T users run with, and
where our current CNES setup stands.*

## The premise we had — partly wrong

Our early-2026-04-17 investigation of the wrong-integer epidemic
blamed **CNES publishing GPS L5 phase biases only under L5I (RTCM
sig_id=14)** while our F9T tracks L5Q (sig_id=7).  Subsequent
literature review contradicts the strong form of that claim:

- Banville et al., *Data and pilot biases in modern GNSS signals*,
  GPS Solutions 2023 — [link](https://link.springer.com/article/10.1007/s10291-023-01448-y)
- Wang et al., *GNSS OSB for all-frequency PPP-AR*, J Geodesy 2022 —
  [link](https://link.springer.com/article/10.1007/s00190-022-01602-3)

Both papers state that GPS L5I and L5Q phase biases are treated as
equal by every major analysis centre.  The physical difference is the
deterministic 90° quadrature offset between the data (I) and pilot (Q)
components of the same carrier, which is absorbed by the receiver's
tracking loop and by the integer ambiguity.  Applying an L5I phase
bias to an L5Q observation is canonical practice.

**So why did applying the L5I bias blow up MadHat in our failed
experiment on 2026-04-16?**  Best current hypothesis: we remapped
`SIG_TO_RINEX` for *both* phase and code, and code biases really do
differ between C5I and C5Q.  Alternatively, CNES's L5I phase bias
carries an integer offset chosen relative to an L5I-tracking reference
receiver — a constant per satellite — and that integer shift lands
LAMBDA on a different (wrong) integer.  Neither is the "sub-cycle
incompatibility" we assumed.

## u-blox ZED-F9T cannot be reconfigured to track L5I

- F9T Interface Description UBX-20033631 and Integration Manual
  UBX-21040375 expose no CFG key to select data (I) vs pilot (Q).
- `CFG-SIGNAL-GPS_L5_ENA` enables the band; pilot tracking is
  hard-wired in firmware for modernized signals.
- Scratch "configure F9T to track L5I" from the candidate-fix list.

## Mounts ranked for F9T + PPP-AR

Ordered by likelihood of working with our F9T's L5Q (pilot) tracking:

| Mount | Caster | Provider | Biases | Access | Verdict |
|---|---|---|---|---|---|
| **OSBC00WHU1** | `products.igs-ip.net` | Wuhan University | Per-code OSB (C1C, C2W, C5I, C5Q, etc.) | IGS login, free | **Strongest candidate, validated 2026-04-25** — OSB is per-code by design; in the 2x2 isolation WHU biases on CNES O/C land sub-meter from Leica with 18 cm cohort spread (best result of any tested combination).  See `docs/ssr-cross-ac-diagnostic-2026-04-25.md`. |
| MADOCA-PPP | JAXA NTRIP | JAXA | GPS/GLONASS/QZSS L1/L2/L5 code + phase | Free R&D registration | Entirely different AC; good second try. |
| Galileo HAS IDD | GSC-issued | European GNSS Service Centre | GPS + GAL | Free, GSC registration | Already on roadmap (`docs/galileo-has-research.md`). |
| **SSRA01CAS1** | `ntrip.data.gnss.ga.gov.au` | Chinese Academy of Sciences (phase 2) | Code + phase | Existing credentials work | **Validated 2026-04-25 post-bug-fix** — uses IGS-SSR `4076_*` proprietary message IDs.  An engine bug (commit 485612d) treating pyrtcm's IDF013-021 mm-output as metres caused 280 m position divergence on first try.  Post-fix, CAS is in the same regime as CNES (~m offset, sub-meter cohort).  Multi-constellation `gps,gal,bds` works on CAS post-fix (~+1.7 m east of Leica truth, σ 0.28 m at 13 min).  Open: GAL `sig_id=2` phase bias still unmapped (one signal dropped per SV, ~0.34 m impact). |
| **SSRA00CNE0** | `products.igs-ip.net` | CNES | L1C/L2W/L5I phase | IGS login, free | Our orbit/clock source.  CNES biases land 1.59 m east of WHU biases on the same orbit/clock (2026-04-25 2x2).  Whether this 1.59 m is a real CNES datum/frame difference or a subtle bias-magnitude-sensitive bug in our application remains an open question — see `docs/ssr-cross-ac-diagnostic-2026-04-25.md` and below. |
| IGS combined (SSRA02IGS0, …) | `products.igs-ip.net` | IGS RTS Kalman combination | **Orbit + clock only, no phase bias** | IGS login, free | Skip for AR. |

## CAS bug history (2026-04-25)

CAS produced 280 m position divergence on first try, attributed
incorrectly to a missing `sig_id=2` phase bias map entry.  The real
cause was an engine bug in `_parse_orbit` / `_parse_clock`: pyrtcm
returns IDF013-021 (IGS-SSR orbit/clock) in mm and mm/s — same as
DF365-378 (standard RTCM SSR) — but the engine's IGS-SSR branch used
the values raw, treating them as already-meters.  Every CAS orbit and
clock correction was 1000× too large.  Fixed in commit 485612d.
After the fix, CAS works.  See `docs/ssr-cross-ac-diagnostic-2026-04-25.md`.

## Open question: CNES vs WHU 1.2 m gap

In the 2026-04-25 30-min long-convergence run, CNES + CNES biases
settled at -0.77 m east of Leica truth and CNES O/C + WHU biases
settled at +0.45 m east — gap of 1.22 m.  Two possible explanations:

1. **Real AC datum / reference-frame difference** between CNES's bias
   datum and WHU's.  CNES biases are large per-SV (±1-2 m); WHU biases
   are small (±0.3 m).  Different ACs anchor their phase biases to
   different reference networks.
2. **Subtle application bug** in how we apply the larger CNES biases
   vs the smaller WHU biases.  No bias-magnitude-sensitive code path
   is visible in inspection, but we can't fully rule it out without
   external verification.

Resolution requires comparing our PPP solution to a reference engine
(RTKLIB, PRIDE PPP-AR) running on the same recorded RINEX OBS + SSR
RTCM.  If RTKLIB+CNES lands at -0.77 m too → real AC datum.  If RTKLIB+
CNES lands near 0 → bug is ours.  Memory `feedback_research_established_ar_impls`
flagged this as the right verification approach but we don't have it
set up yet (would need a RINEX writer and RTCM capture in the engine).

## Candidate failure modes beyond L5I/L5Q

The NL-residual monitor lets us distinguish these; instrumentation
landed 2026-04-17:

- **GPS L5 ISC / TGD handling** — `docs/f9t-firmware-capabilities.md`
  notes a 50 m bias still open as of 2026-04-16.
- **C5I vs C5Q code-bias difference** — code biases are genuinely
  per-code, unlike phase.  CNES publishes C5Q for GPS in at least some
  frames; need to check coverage per SV.
- **ZTD-ambiguity correlation on single-constellation runs** —
  partially addressed by the ISB-pin fix + ZTD state, but may still
  leak in GAL-only under poor geometry.
- **Sign/cycle-slip handling** — standard suspect list.

## Action plan for the morning of 2026-04-18

1. **Try `OSBC00WHU1`** — one-line mount change.  Existing RTCM decoder
   handles OSB messages.  Target the F9T L5Q question by changing
   providers rather than reconfiguring the receiver.
2. **Register for MADOCA-PPP and Galileo HAS IDD** so we have
   alternative paths tomorrow.
3. **Read the NL-residual instrumentation** from overnight runs — if
   WL residuals are clean but NL shows a per-satellite *fractional*
   cycle bias, the L5I/L5Q intuition is right after all.  If NL shows
   a per-satellite *integer* (multi-cycle) bias, it's code-bias or
   ISC/TGD.

## Status as of 2026-04-18 evening

- **Dual-mount CNES (orbit/clock/code-bias/GAL phase) + WHU
  (observable-specific code+phase biases for GPS/BDS) landed** as
  commit `7ae1392`.  CLI: `--ssr-bias-mount OSBC00WHU1
  --ssr-bias-ntrip-conf ntrip-whu.conf`.
- First-deploy diagnosis: the initial *phase-bias-only* filter
  blocked WHU's code biases — CNES had no C5Q code bias and F9T's
  L5Q pseudoranges ran uncorrected, producing ~30 m vertical drift.
  Fix was to pass code+phase (not just phase) through the bias
  mount.
- The L5I/L5Q phase-bias premise in "*The premise we had — partly
  wrong*" is itself only partly right.  Literature claims *within-AC*
  L5I = L5Q; that's true but doesn't help us when CNES and WHU use
  mutually inconsistent datum conventions (see
  `docs/l5i-l5q-phase-bias-empirical.md` — 19 GPS SVs, mean Δ = −0.73 m,
  SD 1.46 m).  Cross-AC substitution never works for RTCM-published
  biases; dual-mount from one AC or matched-datum fusion is the
  right path.

## Diagnostic method: cross-AC bias comparison

Whenever a new AC-vs-AC pairing is considered (e.g., investigating
ptpmon's L2-profile drift, or evaluating MADOCA vs CNES for a future
GPS AR path), the fastest empirical answer is **run both streams
simultaneously and compare the published bias values per satellite**.
Both values land in `SSRState._phase_bias` (or `_code_bias`) keyed
by RINEX observable code; if the two ACs use different code labels
for the "same" observable (as CNES does for GPS L5I while WHU uses
L5Q), the dict keeps them separate automatically.

Recipe:

1. Run the engine with both SSR mounts active (primary + bias).
2. Grep the engine log for the periodic `Phase bias: <sv> <code> =
   <value>` dump emitted at startup for each new AC's first batch
   (source file `ssr_corrections.py`; the dump happens inside
   `_store_phase_bias` when the `_phase_bias[sv][code]` entry is
   first created).
3. Build a side-by-side table — one row per satellite, one column
   per (AC, code) pair.  Compute differences where the same SV is
   published by both ACs for either the same observable or two
   observables the literature claims are interchangeable.

Interpret:

- **Δ < 5 cm per SV across the constellation** — ACs are datum-
  consistent for this observable; substitution or either-source
  use is safe.
- **Δ has mean ≈ 0 with SD 10–50 cm** — small AC-calibration
  scatter; substitution mostly works; drift rarely visible at the
  host level but may limit ultimate AR performance.
- **Δ at meter scale with multi-meter per-SV variance** — ACs are
  datum-incompatible for this observable; cannot mix freely.  All
  biases applied to one observable must come from one AC; if only
  one AC covers the F9T-tracked code, prefer a dual-mount split
  that respects that constraint.

The L5I-vs-L5Q analysis in
`docs/l5i-l5q-phase-bias-empirical.md` is the worked example for
this recipe (GPS case).  The awk one-liner at the bottom of that
doc is portable to any AC pair — swap the SV prefix (`G`/`E`/`C`)
and RINEX code accordingly.

Result of running this method should be logged either as a new
section in `docs/l5i-l5q-phase-bias-empirical.md` (if the topic
is closely related) or as a sibling `docs/<signal>-phase-bias-
empirical.md` doc (if the story is its own thing).

## Signal-matching: the obs and the AC phase biases must agree per frequency (2026-07-05)

The load-bearing rule for PPP-AR, made concrete during the RTCM MSM ingest
work (I-030423): **an SV enters ambiguity resolution only if the SSR AC
publishes a phase bias for the *exact* RINEX signal each frequency of the
obs uses.**  In the engine this is the `ar_phase_bias_ok` gate — a per-SV
`[PB_APPLIED] f1=…→L1C val=MISS` means the AC has no phase bias for that
signal, so the SV is float-only.  Both `f1` and `f2` of the IF pair must
match, or no AR for that SV.

**What the obs give us (universal): GPS L1 is C/A, not P(Y).**  Every RTCM
obs source we can reach streams GPS **`L1CA` + `L2W` + `L2S`(L2C) + `L5Q`** —
rich on L2/L5, but L1 is **C/A (`L1C`) only, never `L1W`**.  Confirmed
2026-07-05 by decoding the ALIC IGS MSM (via the GA caster) and every mount
on the two local casters:
- **ISTHA** `50.149.86.86:12054` (10 mounts: ALSIP/ROCKFORD/NAPERVILLE/…),
  RTCM 3.1 MSM5, GPS+GLO+GAL(+BDS).
- **DuPage** `50.149.86.86:12055` (6 mounts: WHEATON/ELMHURST/…), RTCM 3.
  (creds `VANVALZAH:8888`; DuPage mounts were quiet/text-only in a short probe.)

So the obs side is fixed at `L1CA`; the AC side must therefore publish an
**`L1C`** phase bias (an `L1W`-only AC cannot AR our obs).

**What the ACs publish (probed 2026-07-05 on `products.igs-ip.net:2101`,
IGS-IP creds; reachable from a London host / PiPuss — IPv6-only-blocked from
gt).  GPS phase-bias signals per mount:**

| Mount | GPS phase-bias signals | Matches `L1CA+L2W` obs? |
|---|---|---|
| **`SSRA01CAS0`** (CAS) | `L1C + L2W + L5Q` | **YES** — the working AR stream |
| **`OSBC00WHU1`** (WHU-OSB) | `L1C + L2W + L5Q` | **YES** (OSB is bias-only; pair with orbit/clock) |
| `SSRA00NRC0` (NRCan) | `L1W + L2W` | NO — `L1W` misses our `L1CA` |
| `SSRA00GFZ0`/`SSRA00SHA0`/`SSRA00GMV0` | *(none)* | no phase biases at all |
| `SSRA00CNE0` (CNES) | *(historically `L1C`-class)* | **DOWN** — 404, absent from the sourcetable since the 2026-06-28 BKG outage |
| `SSRA00BKG0` (BKG combined) | *(0 phase biases)* | float-PPP only |

**Working recipe with CNES down: single-AC CAS.**  Obs `L1CA+L2W` (ALIC or
any local caster) + **all** SSR (orbit+clock+code-bias+phase-bias) from
`SSRA01CAS0` (single-AC per the `ac-datum-mixing.md` rule) → `[PB_APPLIED]`
matches both frequencies → **wide-lane AR resolves** (observed WL 7/9 fixed,
strength 37 on PiPuss/ALIC).  **Narrow-lane still needs a second
constellation:** GPS-only + SSR hits the near-singular `(rx_clk, ZTD,
common-ambiguity)` mode (formal σ 0.19 m but 1.8 m biased + drifting, NL
0 fixed) — so cm PPP-AR is blocked on **multi-GNSS (`gps,gal`)**, i.e. the
GAL MSM decode (ingest step 4), NOT on correction availability.  See also
`docs/bds-b2a-phase-bias-survey-2026-05-09.md` (the BDS analog of this
per-signal availability probe) and `docs/ac-datum-mixing.md` (why single-AC
for AR).

## Sources
- Banville et al. 2023 (data + pilot bias handling): <https://link.springer.com/article/10.1007/s10291-023-01448-y>
- Wang et al. 2022 (GNSS OSB all-frequency PPP-AR): <https://link.springer.com/article/10.1007/s00190-022-01602-3>
- Geng et al. 2024 (WHU phase-bias stream): <https://link.springer.com/article/10.1007/s10291-023-01610-6>
- IGS RTS Products page: <https://igs.org/rts/products/>
- MADOCA-PPP internet distribution: <https://qzss.go.jp/en/technical/dod/madoca/madoca_internet_distribution.html>
- Galileo HAS IDD: <https://www.gsc-europa.eu/galileo/services/galileo-high-accuracy-service-has/internet-data-distribution>
