# Antenna signal-quality metrics — and which of them predict timing

Written 2026-08-26, prompted by onocoy grading the UFO1/SXT-D station and by
the question of whether an RTK-oriented quality score is a useful judge of a
*timing* antenna.  Related: [two-site-sync-budget.md](two-site-sync-budget.md),
[antenna-calibration-plan.md](antenna-calibration-plan.md),
[local-caster-onocoy-plan.md](local-caster-onocoy-plan.md).

## Three metrics that look alike and are not

All three start from code-minus-carrier.  They differ in what they remove,
which is why their numbers differ by 3x on the same antenna.

| Metric | Formula | Ionosphere | Absolute? | Elevation |
|---|---|---|---|---|
| **testAnt `cmc_std`** (`bobvan/testAnt`) | `P − λφ`, per-arc **mean** removed | **retained** — the arc's iono trend is still in it | relative | separate plots vs elev / skyplot |
| **TEQC-style MP** (used here 2026-08-26) | dual-frequency MP1/MP2 | **cancelled exactly** | absolute | binned by C/N0 |
| **onocoy "RMS Code"** | undisclosed | unknown | absolute | empirically masked or weighted |

Measured on the same 2.4 h of UFO1 observations:

    testAnt-style (mean-removed CMC)  ~0.88 m   ← inflated by residual iono
    TEQC MP, all arcs unweighted       0.79 m
    TEQC MP, C/N0 44-50 dBHz           0.31 m
    TEQC MP, C/N0 >=50 dBHz            0.19 m
    onocoy RMS Code                    0.23 m   ← sits between the top two buckets

That reconciles all three: onocoy's number is the same physical quantity as
ours, restricted to high-elevation/high-C/N0 satellites.  **testAnt's metric is
not wrong** — it is designed for A/B antenna comparison on one host, where the
ionosphere is common-mode between the two antennas and cancels in the
difference.  It is simply not an absolute figure, and should not be compared
against an absolute threshold.

## Which metrics actually predict timing

onocoy's quality score is
`(RMSCode² + RMSPhase² + CycleSlipRatio² + SkyVisibility²)/4`, built for **RTK**.
For a carrier-phase timing system the four are *not* equally relevant:

| onocoy metric | Relevance to PePPAR-Fix timing |
|---|---|
| **RMS Phase** | **Directly predictive.**  1.2 mm = **4.0 ps**, which *is* the noise on the TD-CP observable the clock rides on (budget: 5-10 ps per epoch). Same quantity, same units. |
| **Cycle-slip ratio** | **Directly predictive.**  A slip forces ambiguity re-fixing = a phase discontinuity = a timing glitch. |
| **Sky visibility** | Moderately — sets observation count and geometry, hence the variance of the clock estimate. |
| **RMS Code** | **Weakly.**  0.23 m = 0.77 ns sounds alarming, but code enters carrier-phase timing only through coarse sync and WL/MW ambiguity fixing — not through the observable the clock actually rides on. |

**So the metric we currently fail (code) is the one that matters least for our
goal, and the metric we pass (phase) is the one that matters most.**  Do not
chase the code number for timing reasons.  Chase it if you care about the reward
multiplier, or about RTK users consuming the stream.

**The correlation is not zero, though**, and this is the honest qualifier: code
multipath is a cheap, very sensitive *proxy* for "this antenna sees
reflections."  Phase multipath is bounded near λ/4 (~5 cm) and usually
millimetres, but it is not zero and it is elevation/azimuth dependent — which
makes it a *time-varying* error as the satellite mix rotates, exactly the kind
that hurts a clock.  A site with 0.23 m code MP is weak evidence of elevated
phase multipath too.  Treat code RMS as a screening indicator, not as the
quantity to optimise.

### The useful corollary for antenna calibration

For a **pinned-ARP timing** receiver, a constant vertical PCO error is absorbed
into the clock offset and calibrates out.  What actually damages timing is the
**elevation-dependent PCV** error, because the satellite mix changes with time
and therefore so does the error.  When judging whether an uncalibrated antenna
is usable for timing, the question is not "do we know its absolute PCO?" but
"is its PCV *shape* right?"

## Can we self-calibrate an uncalibrated antenna?

Situation: CHOKE1 is a 3D choke ring, a Chinese clone, uncalibrated.  UFO1 is an
`SFESPK6618H`, which **is** NGS-calibrated.  They are **0.98 m apart** on the
same roof.

That is close to the classic NGS *relative* field-calibration geometry: a
calibrated reference and an unknown on a very short baseline.  What makes it
work here is the **swap**: run A-at-site-1 / B-at-site-2, then swap.  Four
configurations let you separate the **antenna** term from the **site** term,
which a single pairing cannot do — at 1 m the multipath environments are *not*
common-mode, and that is precisely the confound the swap removes.

Realistic expectations:

- **Achievable:** a decent vertical PCO and a coarse elevation-dependent PCV
  relative to the SFESPK6618H, from days-to-weeks of dual-frequency data.
- **Not achievable:** anything publication-grade or azimuth-resolved.  Precision
  is limited by the two sites' own multipath, and the result inherits every
  error in the reference antenna's calibration.
- **Sufficient for our purpose?**  Probably yes, given the corollary above —
  we need the PCV *shape*, not an absolute PCO.

**Substituting a similar NGS/IGS model** is tempting and risky.  Many Chinese
choke rings are dimensional clones of the Dorne-Margolin / JPL design, so a
catalogue entry may be close in PCV shape.  But PCV depends on the element,
radome and LNA, not only the choke geometry.  Recommended use: take a candidate
entry as a *prior*, then test it against our own relative calibration; if the
shapes agree within a few mm across elevation, adopting it is defensible.  Do
not adopt one untested.

## Experiment design — and the confound to avoid

The proposal to put an idle **ZED-X20P on CHOKE1 with its own Pi** would change
receiver, antenna and mount simultaneously.  That confound is real and is
exactly what `bobvan/testAnt` was built to avoid: **two matched ZED-F9T
receivers, one host, one TICC**, reference antenna on TOP and antenna-under-test
on BOT, so only the antenna differs.  The rig already exists and its
`report_card.py` takes `--antenna`, `--mount` and `--receiver` as *separate*
fields — the crispness this question worries about losing is already in the
tooling.  Minimum 24 h for a non-provisional result.

`report_card.py` also already scores `cmc_std` **and** `adev_1s` on one page,
which is the multipath-vs-timing pairing this document is about.

**Do not stand up a second onocoy reference station 0.98 m from the first.**
Beyond the confound, co-located stations carry no independent information for a
network that rewards spatial coverage (see `local-caster-onocoy-plan.md`).  Use
testAnt for the controlled A/B; use onocoy as the *absolute* external judge of
whatever configuration ends up on the production mount.

## Mount vs antenna vs receiver — keep the three separate

This question surfaced a genuine naming hazard: "CHOKE1" and "UFO1" name
**ARPs / mount points** in `antennas.json`, but are habitually used to mean the
**antennas** sitting on them.  A swap makes every past statement ambiguous.
Before swapping anything, decide and write down which the names follow — the
mount or the hardware — and if the answer is "the mount", give the antennas
their own identifiers.  `report_card.py`'s separate `--antenna` / `--mount`
fields are the model to copy.

## A top-of-scale anchor: PTBB, measured with the same code

We already relay **PTBB00DEU0** — PTB Braunschweig, a national metrology
institute — for the known-good-obs diagnostic, and `@obs` has been logging it.
That log is genuine RTCM3 MSM7, so the *identical* metric can be run on it, on
the *identical* signal pairs (GPS L1CA/L2W, GAL E1C/E5aQ). That makes it a
calibration point for the top of the scale, obtained with no new hardware.

Measured 2026-08-26, ~1 h of PTBB against 2.4 h of UFO1:

| | UFO1 / SXT-D | PTBB / PTB | ratio |
|---|---|---|---|
| MP RMS, all arcs | 0.789 m | **0.363 m** | 2.17× |
| MP RMS, C/N0 38–44 | 0.416 m | 0.296 m | 1.41× |
| MP RMS, C/N0 44–50 | 0.305 m | 0.233 m | 1.31× |
| MP RMS, C/N0 ≥50 | 0.191 m | **0.099 m** | 1.93× |
| phase noise | 1.00 mm | **0.62 mm** | 1.61× |

Three things fall out.

**1. onocoy's thresholds are real, not arbitrary.**  PTBB's high-C/N0 code MP is
**0.099 m — inside the 0.140 m bar** (0.71×), while ours is 0.191 m (1.36×).
The top-tier threshold is calibrated to what a genuine geodetic-grade
installation actually achieves. That is worth knowing before dismissing a score
we fail.

**2. The phase gap is the one that matters here.**  0.62 mm = **2.07 ps**
against our 1.00 mm = 3.34 ps. Both sit inside the 5–10 ps TD-CP per-epoch
budget, so neither is disqualifying — but a national metrology institute's chain
is **1.6× quieter on the observable a carrier-phase clock rides on**, and that is
a more meaningful gap for us than the 2× on code.

**3. This is a scenario-1 measurement and cannot attribute anything.**  PTBB
differs from us in receiver, antenna, mount, site, professional installation
*and* latitude (52.3 °N vs 41.8 °N, which genuinely changes sky coverage). It
says the *net chain* is better; it does not say which part. Attribution needs the
simultaneous differential rig — see `bobvan/testAnt`
`docs/future-work-signal-quality.md`.

### Other national-lab anchors already reachable

On `igs-ip.net` with the credentials we already hold (385 mounts; codes checked
against national timing/metrology institutes):

| Mount | Lab | Systems |
|---|---|---|
| `PTBB00DEU0` | PTB Braunschweig (DE) | GPS+GLO+GAL+BDS |
| `BRUX00BEL0` | ORB Brussels (BE) | GPS+GLO+GAL+BDS+QZS+SBAS |
| `IENG00ITA0` | INRIM Torino (IT) | GPS+GLO+GAL+BDS+QZS+SBAS |
| `SPT000SWE0` | RISE Borås (SE) | GPS+GLO+GAL+BDS+QZS |
| `MIZU00JPN0` | NAOJ Mizusawa (JP) | GPS+GLO+GAL+BDS+QZS+IRNSS |

So a *panel* of four European/Japanese metrology institutes is available in real
time, not just one — enough to characterise the spread at the top of the scale
rather than trusting a single station.

**NIST and USNO are not on this caster** (nor NPL, OPMT, TWTF, KRISS, METAS).
They are IGS stations, so their observations exist as post-processed **RINEX**
from CDDIS — which is exactly the "RINEX ingest is the missing fourth" item in
the testAnt groundwork. Adding RINEX would unlock the whole public archive as a
comparison corpus, including the US labs.

**Caveat worth carrying:** these are the *published* streams of those stations,
which may be decimated, filtered, or from a different antenna than the one
feeding their timing systems. A national lab's IGS station is not necessarily the
chain their UTC(k) rides on.

## The panel, measured — and a correction to the single-anchor result above

2026-08-26. `IENG00ITA0` (INRIM Torino) and `SPT000SWE0` (RISE Borås) were
captured live for ~15 min and run through `scripts/obs_quality.py` alongside
PTBB and our own station, all on a **matched ~15 min window** with the **same
mask** (`--signals l1`: GPS-L1CA + GAL-E1C only).

| Station | Site | MP all | MP 44–50 | MP ≥50 | phase |
|---|---|---|---|---|---|
| **UFO1 / SXT-D** (ours) | Wheaton IL, 41.8 °N | 0.443 m | 0.248 m | 0.171 m | 0.55 mm = 1.83 ps |
| **PTBB** / PTB | Braunschweig DE, 52.3 °N | 0.344 m | 0.242 m | n/a | **0.40 mm = 1.33 ps** |
| **SPT0** / RISE | Borås SE, 57.7 °N | 0.256 m | 0.140 m | 0.081 m | 0.51 mm |
| **IENG** / INRIM | Torino IT, 45.0 °N | **0.164 m** | **0.090 m** | 0.075 m | 0.54 mm |

onocoy's 0.99 bars for reference: code ≤ 0.140 m, phase ≤ 1.40 mm (4.67 ps).

### The panel was necessary, and the single anchor was misleading

There is a **2.7× spread among the national-lab stations themselves**
(0.164 → 0.443 m). PTBB, which the section above used as *the* top-of-scale
anchor, turns out to be the **worst of the three labs** on code multipath and
only 1.3× better than our rooftop. INRIM is 2.7× better than us and 2.1× better
than PTBB.

Had we anchored the scale on PTBB alone we would have concluded our station was
close to national-lab quality. Against the panel, it is not: on code we are last
by a clear margin. **One anchor sets a point; a panel sets a scale.** This is the
argument from `testAnt` `docs/future-work-signal-quality.md` §4b, now with the
evidence that motivated it.

### Correction: the phase gap was overstated

The section above reported PTBB's phase noise as 0.62 mm against our 1.00 mm —
a 1.6× gap. **Those were unmatched windows** (1 h vs 2.4 h). On matched 15 min
windows the same two stations read **0.40 mm vs 0.55 mm — 1.38×**.

The metric is **duration-sensitive**, materially so for phase: PTBB alone reads
0.40 mm over 15 min and 0.62 mm over an hour. So *window length is itself a mask
that has to be published*, alongside the elevation/C/N0 cut and the signal set.
Three masks, not one, and each of them moves the answer.

### What the panel actually says about us

- **Code: we are last, by 2.7× against the best.** Not marginal, and not
  explained by latitude — the ordering (IENG 45.0 °N best, SPT0 57.7 °N, PTBB
  52.3 °N, UFO1 41.8 °N worst) has no geographic pattern. It is siting and
  installation quality.
- **Phase: we are last but only by 1.38×**, and **all four stations sit
  comfortably inside onocoy's phase bar**. On the metric that predicts
  carrier-phase timing, a rooftop station with a non-choke-ring antenna is
  within ~40 % of national metrology institutes.

That asymmetry is the useful result. It says the rooftop environment costs us a
lot of *code* quality and comparatively little *phase* quality — which is
precisely the split that makes onocoy's failing grade tolerable for a timing
mission, and precisely what a choke ring would be bought to fix if the goal were
RTK.

**Caveat carried forward:** ~15 min is a short window and these are single
captures. Treat the ordering as indicative, not settled — the numbers move by
~10 % on code and ~50 % on phase between a 15 min and a 1 h window.
