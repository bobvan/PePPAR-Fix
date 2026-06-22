# PCV error bounds relative to the PCO

**Use case: uncalibrated antennas where the PCO can be borrowed from a similar
calibrated model, but no PCV calibration exists.** How much error does omitting
(or borrowing) the PCV cost, given a good PCO? This note bounds it empirically
from the IGS ANTEX database, with a focus on choke rings and our SparkFun lab
antenna.

## PCO vs PCV — what's actually missing

ANTEX models a receiver antenna's carrier-phase pattern as two pieces:

- **PCO** — a constant 3-D offset from the antenna reference point (ARP) to the
  *mean* electrical phase centre. Large (cm–dm), and **largely shared by
  antennas of the same element family/radome** — so it can be borrowed from a
  calibrated analog with mm-class residual.
- **PCV(zenith[, azimuth])** — the *residual* phase-centre variation **relative
  to the PCO**, in mm. This is the per-design fingerprint that an uncalibrated
  antenna lacks.

Apply PCO **+** PCV → fully corrected. Apply **PCO only** (the uncalibrated
case) → **your residual error *is* the PCV pattern.** Two consequences:

1. **The mean PCV is ≈ 0 by construction** — the PCO is *defined* to absorb the
   mean, so "mean PCV" (~−2 mm here) is not the error that matters.
2. **The peak-to-peak PCV is the uncorrected error**, and it is
   **elevation-correlated** — so it does **not** average to zero; it leaks into
   the height / receiver-clock / ZTD estimates through their elevation
   dependence (a bias, not white noise). 1 mm ≈ **3.336 ps** of range.

## Findings (IGS `igs20.atx`, GPS L1/L2, NOAZI elevation pattern)

| Antenna class | L1 mean | **L1 p-p** | L2 mean | **L2 p-p** |
|---|---:|---:|---:|---:|
| **Choke rings** (124 type+radome, 54 models) | ~−2 mm | **~20 mm** (5–29) | ~−1 mm | **~15 mm** (8–20) |
|  · older AOA/Ashtech Dorne-Margolin, TRM29659, Topcon | | **22–29 mm** | | 14–18 mm |
|  · modern Leica AR25, NovAtel-750 | | **5–8 mm** | | 13–17 mm |
|  · Septentrio PolaNt (SEPCHOKE) | | ~13 mm | | ~19 mm |
| **SparkFun SFESPK6618H** (our lab antenna) | +0.3 mm | **3.3 mm** | −0.5 mm | **6.1 mm** |

The headline is the **peak-to-peak**, and it spans a wide range:

- **Choke ring is not one number.** Old Dorne-Margolin rings swing ~25 mm L1
  (~83 ps); modern designs (Leica AR25, NovAtel-750) are ~5–8 mm — a 5× range.
- **The SparkFun SFESPK6618H is flatter than *every* choke ring** in the
  database: **3.3 mm L1 / 6.1 mm L2** (≈ 11 / 20 ps). That's ~6× tighter on L1
  than the choke-ring fleet average.

## Error bound for using an uncalibrated antenna

> **Borrowed-PCO error** ≈ mm (PCO is design-shared and dominant).
> **Missing-PCV error** ≤ the **peak-to-peak PCV of the nearest calibrated
> analog** — i.e. the numbers above.

Guidance:

- **PCO: borrow from the closest calibrated model** (same element family +
  radome). Low risk — the inter-model PCO difference within a family is mm-class.
- **PCV: borrow the nearest model's PCV pattern, don't assume zero** — *unless*
  the antenna is a known small-PCV design. Borrowing leaves only the
  *intra-family* p-p spread as residual (small for modern designs, larger for
  old DM rings).
- **Zero-PCV (no correction) is only defensible for small-PCV antennas**
  (SparkFun-class, a few mm = ~10–20 ps). For a choke ring it's a **~cm-class,
  elevation-correlated bias** — never assume zero there.

### Connection to the APC-not-ARP survey choice

This directly justifies the London-survey **APC framing**
([`arp-survey-strategy.md`](arp-survey-strategy.md)): the SFESPK6618H was
surveyed with **zero-PCV** (ANT=NONE), and its true L1 PCV p-p is only **3.3 mm
(~11 ps)** — negligible against the ±10 cm position bar and small against the
few-ns time budget. So adopting the zero-PCV APC was sound *for this antenna*;
it would **not** have been sound for an old choke ring (~25 mm / ~83 ps).

## Tooling & reproduction

`tools/analyze_chokering_pcv.py` — parses an ANTEX file (`scripts/antex.py`
`ANTEXParser`), token-matches antennas, computes per-band NOAZI **mean** and
**peak-to-peak** PCV, and writes a per-antenna CSV + a PCV-vs-zenith plot.

```sh
# choke-ring fleet:
analyze_chokering_pcv.py --out-dir ~/gt/antenna-analysis
# any antenna by name (e.g. the SparkFun):
analyze_chokering_pcv.py --tokens SFESPK6618H --out-name SparkFun_pcv \
    --title "SparkFun SFESPK6618H antenna PCV (relative to PCO)" \
    --out-dir ~/gt/antenna-analysis
```

Source: `~/PRIDE-PPPAR/table/igs20.atx` (current IGS20 absolute calibration,
with our lab antennas injected by `scripts/inject_lab_antennas.sh`). Artifacts:
`~/gt/antenna-analysis/{chokering_pcv,SparkFun_pcv}.{csv,pdf,png}`.

## Caveats

- **NOAZI (elevation-only)** PCV — the dominant component; the azimuthal term
  (full grid) adds a smaller contribution and can be reported separately.
- **Choke-ring classification is name-token-based** (no clean ANTEX field); the
  match list is printed/auditable, and 3 Ashtech families (`ASH700228/701933/
  701941`) are flagged *uncertain* (possibly plain geodetic, not choke).
- **PCO and PCV are coupled and frequency-dependent**: a borrowed PCV is only
  valid relative to the *same* PCO convention, and the bound assumes the
  uncalibrated unit electrically resembles its analog.
- These are **type-mean** calibrations; individual units vary (sub-mm–mm), which
  is itself part of the residual when borrowing.

Related: [`arp-survey-strategy.md`](arp-survey-strategy.md) (APC framing),
[`l5i-l5q-phase-bias-empirical.md`](l5i-l5q-phase-bias-empirical.md).
