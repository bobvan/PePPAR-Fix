# PulsePuppy OCXO Buying Guide

Notes on TAPR PulsePuppy footprint nomenclature and OCXO options
researched for the lab in 2026-04. Written when planning a 4-unit
buy at $100/each cap for cross-host PPS-OUT agreement work.

## PulsePuppy footprints — two overlaid, one that matters

Per the [PulsePuppy Manual rev_c](https://web.tapr.org/~n8ur/PulsePuppy_Manual.pdf)
the board has two overlaid OCXO/TCXO footprints labeled `X1` and `X2`:

- **X1 — "Eurocase" (a.k.a. "Euro pack")** — 5-pin telecom-industry
  standard.  Pin centers **17.78 × 25.4 mm** (0.7″ × 1.0″), case
  typically **27 × 36 mm**, 0.79 mm pin diameter.  Pin function:
  1 = VCO/EFC, 2 = Vref or oven monitor (or NC), 3 = Vcc, 4 = RF
  OUT, 5 = GND/case.  PulsePuppy was designed around the IsoTemp
  OCXO-131 in this footprint.  This is the larger of the two
  footprints.
- **X2 — Crystek CXOH20 footprint** — 4-pin (with one extra hole)
  TCXO format.  Body 20 × 20 × 10.5 mm, lead row spacing 0.6″
  (15.24 mm).  TAPR's note for X2 only points buyers at the
  Crystek CH-OX20 TCXO.  **No real OCXOs are made in this size**
  — practically a dead end if you want oven-controlled stability.

**Practical rule: real OCXOs always go in X1.**

A measurement gotcha to remember next time: the *smaller* footprint
on the PulsePuppy (15.25 mm pin square / 19–20 mm case) is the
TCXO-format X2, not the OCXO-friendly one.  "Eurostyle" /
"Eurocase" in this product context refers to the *larger* (X1)
footprint, despite intuition.

## Comparison table — OCXOs evaluated for $100/each lab buy

τ=1s ADEV figures from datasheets where available, family
estimates otherwise.  Phase noise at 10 Hz offset is the most
relevant close-in number for a GPSDO discipline loop.

| Model | Freq | Vcc | Output | ADEV @ 1s | PN @ 10 Hz | Footprint | Used eBay | Fits PulsePuppy? |
|---|---|---|---|---|---|---|---|---|
| **IsoTemp OCXO-131-100** | 10 MHz | 5 V | HCMOS sq | ≤ 2 × 10⁻¹¹ | ~−115 typ | Eurocase X1, 27 × 36 mm, 5-pin | $20–25 | **Yes, native X1** |
| **CTI OSC5A2B02** | 10 MHz | 5 V (+ Vref) | sine ~3 Vpp/50 Ω | ~5 × 10⁻¹² typ | ~−125 dBc/Hz | 25.4 × 25.4 mm, 5-pin, **metric pin grid** | $3–7 (lots cheaper) | Yes, but pads need drilling — not on 0.1″ |
| **NDK ENE3311B** (or A/D/E/F) | 10 MHz | 5 V | HCMOS / sine variants | ~1 × 10⁻¹¹ | ~−130 dBc/Hz | 26 × 26 mm, 4-pin SC-cut | $3–6 | Yes with pad drilling |
| **IsoTemp OCXO33-80** (parts-bin, 069683-002H, 51×51 mm) | **5 MHz** | likely 12 V (or 6.71 V Lucent OEM) | likely sine into 50 Ω | **~1–5 × 10⁻¹²** | ~−130 to −140 | **51 × 51 mm, MV89A-class double-oven SC-cut** | n/a | **No** — too big for X1; needs carrier + 5 MHz divider |
| **Morion MV89A** | 10 MHz | **12 V** | sine, +7 ± 2 dBm/50 Ω | **2 × 10⁻¹²** | **−130 dBc/Hz** | 51 × 51 × 38 mm | $30–60 | **No** — physically too big |
| **Rakon STP3593LF / ROX5242T1N family** (used in SparkPNT GNSSDO) | 10 MHz | 5 V | sine, +5 to +9 dBm/50 Ω | **1.5 × 10⁻¹² typ, 5 × 10⁻¹² max** | **−130 dBc/Hz typ** | 52 × 42 × 14 mm, 5-pin | sample-only (no public price) | **No** — too big for X1 + integrated 5-pin pinout includes I²C; needs custom carrier |

ADEV ranking @ τ=1s, best first:
**Rakon STP3593LF ≥ MV89A ≈ OCXO33-80 > OSC5A2B02 > NDK ENE3311 > OCXO-131-100.**
The MV89A and parts-bin OCXO33-80 are in the same architectural
tier (double-oven SC-cut, telecom-grade), about an order of
magnitude better than OCXO-131-100 at short tau.  The Rakon
STP3593LF family sits at or just above the MV89A tier (typical
1.5 ppt vs MV89A's 2 ppt), with the major architectural
difference that the DAC + thermal compensation + power management
are integrated into the module rather than supplied by the
carrier board.

**On the OCXO33-80 designation**: this *is* the Isotemp **OCXO 33
family**, but in the larger 51 × 51 mm OEM package — not the
better-known small-can OCXO 33-46 (which is ~1-inch 14-pin DIP).
The NSN catalog lists a sister part **OCX033-25** (NSN
5955-01-286-2237) explicitly at **"2-inch nominal" (50.8 mm)**,
confirming the family has a high-grade large-can variant.  The
small-can and large-can share the OCXO 33 family name but are
different physical lines.

P/N **069683-002H** (oscillator), **069747-001 Rev C/E** (carrier
PCB), and **069749-002** (stamped on the can side) are sequential
Isotemp OEM customer-config project numbers — likely a Lucent /
AT&T plant order given the era and topology.  Date code **0231 =
2002 week 31** (YYWW).  The label `FSC 31785` is the CAGE code
confirming Isotemp Research Inc. (Charlottesville, VA — acquired
by Pletronics 2021).

Isotemp's `<family>-<suffix>` numbers are sequence/OEM-config IDs,
not frequency codes — e.g. OCXO91-30 is 5 MHz at 6.71 V,
OCXO107-10 is also 5 MHz despite "-10".  So "OCXO33-80" doesn't
decode to a frequency on its own; the can label is what tells us
5.000 MHz.

## Recommended buy at $400 for four units

**4× IsoTemp OCXO-131-100, eBay, ~$22 each = $88.**  TAPR's
reference part for PulsePuppy.  Drop-in to X1, no rework, sits
right at the moonshot floor for short-tau ADEV.  Avoid the
`131-191` variant — that's 12 V (works on PulsePuppy with the
R6/JP7 mod, but is more soldering each time).

Spend leftover budget on a 5-pack each of CTI OSC5A2B02 (~$30)
and NDK ENE3311 (~$30) for side-by-side characterization on
TICCs.  That's another ~$60 and matches the lab philosophy of
characterizing each component without contaminating the result.

**If PulsePuppy is not a hard constraint, build an MV89A
carrier.**  4× MV89A at ~$50 each = $200, plus ~$30 per unit
for a separate carrier PCB or TADD-2.  Total ~$120/each — still
in budget — and 10× better at long τ than anything PulsePuppy
can host.

## On vintage MV89A from Chinese eBay sellers

The two-digit number ("03", "04", "05", "07", "11" …) is
**year of manufacture** — standard Morion convention.  Often
paired with a batch / serial-number code (e.g. ZA5310).

Tradeoffs by vintage:
- **Older (2003–2007)**: settled into the 1e-12/day aging floor.
  Better short-term repeatability without warm-up.  But: less
  remaining EFC pull range — at least one EEVblog user reported
  a well-aged sample that could no longer be tuned back to
  10.000 MHz at all.  For a slow GPSDO servo this matters.
- **Newer (2010–2015 sweet spot)**: still has EFC headroom for
  pulling, mostly aged in.  Probably the safest choice unless
  you specifically want a settled holdover reference.
- **2020+**: needs months of warm-up before meeting datasheet
  aging — fine if you have time.

No specific "good vintage" called out by time-nuts community —
quality variation tracks individual seller QA, not year.

Spotting fakes / re-marks on receipt:
- **Heavy** — ~250–300 g for the double-Dewar oven.  Light
  cans are suspect.
- **Year code laser-engraved**, not painted.  Paint = re-marked.
- **Power profile**: >1 A peak warm-up at 12 V, settling to
  ~350 mA steady.
- **Output**: clean sine ≥ +5 dBm into 50 Ω, harmonics ≥ 30 dBc
  down.  A unit with low output power (sub-zero dBm) probably
  has a blown coupling cap — known failure mode on China pulls.
- Serial numbers two letters + four digits, e.g. ZA5310.

## Parts-bin Isotemp OCXO33-80 — usage path

These are **51 × 51 mm vintage Lucent/telecom-pull double-oven
SC-cut 5 MHz OCXOs** in the MV89A performance class.  They will
not fit any PulsePuppy footprint and need a separate carrier.

Likely electrical (confirm with multimeter continuity tracing on
the carrier PCB before powering up):

- **Supply: 12 V most likely** (matches OCXO 82, 107, 134 same-era
  siblings).  Single rail; the carrier's plain input-filter passives
  imply no switching regulator.
- **Output: sine into 50 Ω**, +5 to +7 dBm typical.
- **EFC: 0 to Vref**, where Vref is internally regulated to ~5 V
  or ~7 V from the 12 V supply.
- **Pin layout: 5 pins on a ~41 mm pattern** (4 corner + 1 center
  in the 51 × 51 mm can).  Center pin is almost certainly RF OUT.

**Bring-up procedure on the existing Isotemp carrier PCB**:

1. Continuity from each pin to the SMA center → **RF OUT**.
2. Continuity from each pin to SMA shell / case / large pour →
   **GND**.
3. Continuity from each pin to power-input connector + (through
   the input-filter passives) → **Vcc**.
4. Trace the 10-turn pot.  One end → GND (already known).  Other
   end → **Vref**.  Wiper through series R → **EFC**.

Powerup: start at **+5 V current-limited to 200 mA**, watch for
RF.  If dead, step to **+12 V current-limited to 1 A** — oven
warmup pulls heavy for the first minute, settles to ~150–300 mA.
Pot at mid-travel.  Don't exceed +15 V without a datasheet.

For PulsePuppy use you would still need to divide 5 MHz → 1 PPS
via the **PD14 firmware** on the PIC12F675 in the divider socket
(replacing the stock PD13 10 MHz firmware) — but this only matters
if you're routing the 5 MHz output through a PulsePuppy as the
divider stage.  More likely you'll build a one-off carrier PCB
that produces 1 PPS directly via 74HC4040 / 74HC4060 dividers or
a separate PIC.  Sources for PD14:

- [leapsecond.com PICDIV page](http://www.leapsecond.com/pic/picdiv.htm)
- [aewallin/PICDIV on GitHub](https://github.com/aewallin/PICDIV)

**Best lab use of these parts-bin oscillators**:

1. **TICC chB reference** — cleaner than F9T PPS for chA-chB
   diff measurements after a one-off carrier board exists.
2. **Free-run characterization yardstick** — an MV89A-class
   reference for ADEV / phase-noise contrast against the i226
   TCXO and OTC OCXO.
3. **Eventual disciplined-DO candidate** with a DAC board and
   temperature-controlled enclosure.

These don't fit a PulsePuppy, but they're free.  Don't redirect
the $400 PulsePuppy budget away from OCXO-131-100s — instead,
spend a separate ~$30–50 on a protoboard carrier and a
programmable EFC DAC (see options below) to put one of these
parts-bin units onto the bench.

## Rakon STP3593LF / ROX5242T1N — premium integrated digital OCXO module

Found while reviewing the SparkPNT GNSSDO BOM in 2026-05.  The
datasheet is for the **STP3593LF / ROX5242T1N family**; SparkPNT
ships a specific variant from this family (the page footer of the
preliminary datasheet identifies the documented variant as
`ROX5242T1N-CO @ 10MHz Sine 5V`).  Family numbers below are
typical/max envelopes — a specific binned variant may land
anywhere within them, so treat these as the upper bound on what
that family can deliver rather than the guaranteed shipping
spec of any one part.

The whole reason this part is interesting is that it folds the
"upgrade the EFC DAC" path (see [EFC DAC options](#efc-dac-options)
below) **into the OCXO can itself**: a 20-bit, 0.8 ppt/step
internal DAC driven over I²C, with thermal compensation and
power-supply management already integrated.  An external
microcontroller writes a 20-bit register; the module's own
thermal-comp loop + double-oven OCXO do everything else.

**Headline specs** (preliminary datasheet, 2020-11-03 draft —
disclaimer §17 explicitly states samples are pre-production and
no performance warranty applies):

| Parameter | Typ | Max |
|---|---|---|
| ADEV(τ = 1 s to 100 s) | **1.5 × 10⁻¹² (1.5 ppt)** | **5 × 10⁻¹² (5 ppt)** |
| Freq stability over temp (−32 to +70 °C, p-p) | 0.03 ppb (30 ppt) | 0.05 ppb (50 ppt) |
| Aging | 0.2 ppb/day | 30 ppb/year, 300 ppb/10 yr |
| Phase noise @ 10 Hz | −130 dBc/Hz | −125 dBc/Hz |
| Phase noise @ 1 kHz | −155 dBc/Hz | −150 dBc/Hz |
| EFC range | > ±350 ppb (>700 ppb total) | |
| EFC resolution | **20-bit, 0.8 ppt/step** (≈ 0.0008 ppb LSB) | |
| Warm-up to ±10 ppb of final | 3 min @ +25 °C, 5 min @ −40 °C | |
| Retrace (24h on / 24h off / 1h on) | ±1 ppb | ±5 ppb |
| Power, steady state | 3.0 W @ +25 °C still air | |
| Power, warm-up | 7.5 W peak | |
| Control bus | I²C slave, addr 0xE0, 400 kHz max | |
| Size | **52 × 42 × 14 mm** (17.6 mm max with pins) | |

**Why it's interesting for the lab**

- ADEV(1 s) typ **1.5 ppt** sits 2× tighter than MV89A
  (2 × 10⁻¹²) and 13–20× tighter than what we actually run today
  (PiFace's Isotemp-class freerun TDEV(1 s) ≈ 85 ps ≈ ADEV ≈
  3 × 10⁻¹¹ — see [`freerun-characterization.md`](freerun-characterization.md)).
  At τ = 1 s integrated, 1.5 ppt → ~1.5 ps RMS phase noise — two
  orders of magnitude below the ~150–200 ps `σ_DO_above_BW`
  budget the [two-site sync budget](two-site-sync-budget.md) §7
  needs to hit the 1 ns shared-antenna excursion bound.
- The integrated 20-bit DAC is **exactly the AD5781 + LTC6655 Vref
  upgrade path** the budget recommends (§7), folded into the can.
  No separate Vref board, no DAC noise + drift to characterize
  independently, no software DAC-temp model — just an I²C write.
- I²C control matches the **existing AD5693R plumbing on PiFace
  + MadHat** and the **Renesas 8A34002 ClockMatrix bus on
  ptBoat + otcBob1**.  No new infrastructure.
- 5 V single supply (vs MV89A's 12 V) simplifies the carrier and
  lets it share a buck regulator with the Pi.

**Why it's not a drop-in**

- **Form factor** — 52 × 42 × 14 mm.  Won't fit PulsePuppy X1
  (27 × 36 mm Eurocase), won't fit X2 (CXOH20 footprint),
  won't fit any of our existing carriers.  Same custom-carrier
  problem as the parts-bin OCXO33-80.
- **Power & thermal** — 3.0 W steady-state at +25 °C with max
  2 m/s airflow.  In a small Pi 5 enclosure with TICC + GNSS
  receiver also running, the host's own dissipation drives
  ambient walk that the OCXO has to ride out.  The 50 ppt
  over-temp headline assumes maintained airflow; without it,
  performance degrades toward the operable-but-not-spec range.
  This is the largest practical risk to characterize before
  committing to the part.
- **Sample-only commercial status** — the datasheet is a 2020
  preliminary draft labeled "Draft" in the revision history,
  with §17 disclaiming all warranties.  No public price, no
  Rakon distributor stock that we've found.  SparkPNT may be
  buying these direct from Rakon at a volume Rakon doesn't
  catalog publicly.
- **Family vs. specific variant** — the spec table is the
  envelope across the family.  A specific binned variant
  (especially a sample from SparkPNT's pipeline) could land
  at typical (1.5 ppt) or at the loose end (5 ppt).  Without a
  guaranteed lower bound from the vendor, any moonshot-class
  budget allocation should plan against the **max-spec** numbers,
  not the typical.

**Honest framing vs. our actual measurement chain**

Our rx-side observation floor is **not** the F9T PPS edge (TDEV(1 s)
~2.1 ns, [`ticc-baseline-2026-04-01.md`](ticc-baseline-2026-04-01.md))
— that's the F9T-PPS-via-EXTTS chain quantized at 125 MHz.  The
**TDCP arm** (Time-Differenced Carrier Phase, Phase D / Arm 5/6 in
the engine) reads the rx-TCXO motion directly from per-epoch
carrier-phase increments and hits **TDEV(1 s) well below 100 ps**:
roughly **23 ps on PiFace's F9T-20B, 54 ps on clkPoC3's F9T-20B**
(per-unit crystal-noise floor, white-FM PSD differs 5× between
units; see `rxTcxoInherentVsThermal` 2026-05-24).  An ensemble TDCP
prototype across multiple F9T units has reached ~15 ps short-tau
floor.

So the relevant comparison is:

| Source | TDEV(1 s) | Note |
|---|---|---|
| F9T PPS via TICC (chA-chB diff) | ~2.1 ns | 125 MHz quantization; gates EXTTS-derived input only |
| **TDCP arm** | **23–54 ps per host, ~15 ps ensemble** | Sub-ns observation arm currently in use |
| PiFace Isotemp DO (freerun, Rb-ref) | ~85 ps | Current host's DO floor sits **above** TDCP |
| clkPoC3 DO (freerun, Rb-ref) | ~45–69 ps | Same — DO floor above the host's TDCP |
| **Rakon STP3593LF (typ)** | **~1.5 ps** | Below TDCP by ~10–35× per-host |

That ordering matters: today, **TDCP is below the DO**, so any DO
upgrade *reduces* the on-host TDEV(1 s) ceiling we measure (the
TDCP arm sees the better DO).  At the Rakon's claimed floor we'd
**cross over** — TDCP would become the dominant short-τ floor, not
the DO.  The implication for the moonshot decision:

- The Rakon's short-τ improvement **is observable** in our existing
  measurement chain (via TDCP), not "above the servo BW only" as
  this section originally claimed.  The loop can actually use it.
- The biggest practical wins still land in **holdover** (the better
  short-τ floor extends the time before phase wanders past the
  budget) and at **τ > 100 s** (where DO RWFM dominates over both
  TDCP and the loop's correction bandwidth).
- A 1.5 ps DO behind a 15–54 ps TDCP arm means the **measurement
  chain becomes the floor**, not the DO — so the next budgeted
  investment after a Rakon-class DO is **TDCP-arm improvements**
  (ensemble across units, longer integration, better cycle-slip
  exclusion), not another DO upgrade.

**Where it would fit in our lab**

Same architectural slot as the parts-bin OCXO33-80: needs a
custom carrier, replaces an existing PiFace / MadHat / clkPoC3
DO outright rather than going into a PulsePuppy.  Different
question from "what to put in PulsePuppy X1" — that question still
answers IsoTemp OCXO-131-100 — and more like "what's the moonshot
upgrade for the highest-grade lab host."  Worth probing SparkPNT
for stand-alone module pricing before committing to a discrete
AD5781-on-MV89A path.

## EFC DAC options

For driving the EFC pin under software discipline.  Assuming a 5 V
EFC range on an MV89A-class part with ±2.5 ppm pull, the LSB step
sets the DAC's contribution to the freq-quantization floor:

| DAC | Bits | LSB step (freq) | Output noise | Vref | Cost |
|---|---|---|---|---|---|
| MCP4725 | 12 | ~240 ppt | not spec'd | internal, mediocre | ~$5 breakout |
| **AD5693R** | **16** | **~15 ppt** | **14 nV/√Hz** | **internal 2.5 V, good** | **~$10 Adafruit breakout** |
| AD5781 | 18 | ~4 ppt | 7.5 nV/√Hz | external precision Vref required | ~$30–50 chip + Vref board |

**AD5693R is the sweet spot.**  16-bit/15 ppt LSB sits below the
~1 × 10⁻¹¹ short-tau floor of OCXO-131-class parts and at the edge
of MV89A territory (~3 × 10⁻¹²/s).  The on-chip 2.5 V reference
means no external precision Vref board — a meaningful hardware
simplification.  I²C interface is easy on a Pi.  **clkPoC3 already
has one wired up** as of 2026-04.

**MCP4725** is fine for first bring-up and proof-of-life — cheap
breakouts, easy I²C, useful to learn what EFC range the OCXO
actually wants.  240 ppt LSB shows up as quantization in TDEV at
short tau on the better oscillators, so don't ship production
work on it.

**AD5781 is the upgrade path** only if you measure that DAC
quantization is limiting τ=10–100 s ADEV with an MV89A-class part.
Requires an external low-noise Vref (LTC6655 / ADR4525 / similar)
to hit datasheet 18-bit performance, so the hardware footprint and
parts cost step up.

## Skip list

- **Crystek CH-OX20 / TCXO at X2** — TCXO, ADEV ~10⁻⁹/s, three
  orders worse than what we need.
- **Trimble 34310-T, HP/Symmetricom 10811** — too big for either
  PulsePuppy footprint, sine output, often 12/24 V.  Buy these
  only if committing to a different carrier.
- **Connor-Winfield OH300 series** — modern surface-mount only.
  Distributor price ~$50–80 new, but no through-hole option.
- **Vectron MX-503** — MCXO, not the right architecture for a
  fast-loop GPSDO, and SMD anyway.
- **Generic Chinese "ultra-low-phase-noise OCXO" assembled
  modules with SMA out** — pre-built reference boards, not bare
  oscillators.  Won't go on a PulsePuppy and you can't
  characterize the underlying crystal independently.
- **Rakon STP3593LF / ROX5242T1N family** — not a PulsePuppy
  candidate (52 × 42 × 14 mm + I²C-only frequency control), but
  worth tracking as a candidate for premium hosts on a custom
  carrier.  See the [premium integrated module section](#rakon-stp3593lf--rox5242t1n--premium-integrated-digital-ocxo-module)
  above for full specs.

## Sources

- [PulsePuppy Installation and Operation Manual (PDF)](https://web.tapr.org/~n8ur/PulsePuppy_Manual.pdf) — TAPR / N8UR
- [TAPR Pulse Puppy product page](https://tapr.org/product/pulse-puppy/)
- [IsoTemp OCXO-131 datasheet](https://www.isotemp.com/wp-content/uploads/2011/03/OCXO-131.pdf)
- [IsoTemp OCXO-131-1003 outline drawing 125-587 (PDF)](http://www.analysir.com/downloads/isotemp%20ocxo%20131%2010MHz.pdf)
- [Morion MV89 datasheet](https://www.morion-us.com/catalog_pdf/mv89.pdf)
- [CTI OSC5A2B02 datasheet (dl6gl mirror)](https://dl6gl.de/media/files/ocxo-cti-osc5a2b02-datasheet.pdf)
- [NDK ENE3311 series catalog (PDF)](https://www.ndk.com/images/products/catalog/ocxo_e.pdf)
- [time-nuts OCXO comparison (febo.com archive, 2012)](https://www.febo.com/pipermail/time-nuts/attachments/20121010/642cf253/attachment.pdf)
- [NTMS / AA5C "Low Cost Surplus 10 MHz Reference"](https://www.ntms.org/files/Mar2022/AA5C_LowCost_10MHz_Reference-1.pdf)
- [VA3TO Bliley/CTI OCXO Board Rev A](https://va3to.com/VA3TO%20-OCXO%20Board%20RevA.pdf)
- [Sync Channel — NDK ENE3311A teardown](http://syncchannel.blogspot.com/2016/04/10mhz-ocxo-teardown-ndk-ene3311a.html)
- [EEVblog forum — MV89A ref-pin thread](https://www.eevblog.com/forum/projects/mv89a-ocxo-ref-pin/)
- [EEVblog forum — CTI OSC5A2B02](https://www.eevblog.com/forum/testgear/cti-osc5a2b02/)
- [EEVblog forum — Recommend me an OCXO](https://www.eevblog.com/forum/projects/recommend-me-an-ocxo/)
- [Tom Van Baak's PICDIV firmware page (leapsecond.com)](http://www.leapsecond.com/pic/picdiv.htm)
- [aewallin/PICDIV on GitHub](https://github.com/aewallin/PICDIV)
- [Rakon STP3593LF / ROX5242T1N preliminary datasheet (SparkPNT mirror, 2020-11-03 draft)](https://docs.sparkpnt.com/GNSSDO/files/component_documentation/STP3593LF_ROX5242T1N_Preliminary_Datasheet.pdf)
