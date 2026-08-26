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
