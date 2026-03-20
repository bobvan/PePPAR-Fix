# Timebeat OTC Signal Routing and ClockMatrix Architecture

**Status: Partially reverse-engineered. Some details unconfirmed.**

## Key insight: the OCXO is free-running

The OCXO on the Timebeat OTC is a free-running oscillator — not a
VCOCXO (voltage-steered) or DCOCXO (digitally-steered). It provides a
stable timebase to the Renesas 8A34002 ClockMatrix chip. The OCXO
frequency is never adjusted.

Instead, the ClockMatrix uses fractional dividers (DPLLs) to synthesize
disciplined output clocks from the OCXO's stable input. The discipline
happens inside the divider, not at the oscillator. This is a digital
frequency synthesis architecture, not an analog VCO feedback loop.

## Signal flow (confirmed and unconfirmed)

```
                    ┌──────────────────────────────────┐
                    │     Renesas 8A34002 ClockMatrix   │
                    │                                    │
  F9T PPS ─────────┤──→ CLK14 (reference input)         │
  (from u-blox      │                                    │
   via GPIO)        │   ┌─────────────┐                  │
                    │   │  DPLL A     │                  │
                    │   │  (closed    │ ← currently a    │
  OCXO ────────────┤──→│   loop)     │   hardware PLL   │
  (free-running,    │   │             │   locking to     │
   stable timebase) │   │  Fractional │   F9T PPS        │
                    │   │  divider    │                  │
                    │   └──────┬──────┘                  │
                    │          │                          │
                    │          ├──→ 25 MHz ──→ i226 PHY  │
                    │          │              clock       │
                    │          └──→ PPS OUT ──→ i226 SDP │
                    │                                    │
                    │   ┌─────────────┐                  │
                    │   │  DPLL B     │                  │
                    │   │  (open loop,│ ← phase          │
                    │   │   phase     │   comparison     │
                    │   │   compare   │   only, for      │
                    │   │   only)     │   shelf clocks   │
                    │   └─────────────┘                  │
                    │                                    │
                    │   (DPLLs C, D may also be in use)  │
                    │                                    │
                    │   I2C ←──────────────────────── Pi │
                    └──────────────────────────────────┘

  Pi (CM5) ←── I2C ──→ ClockMatrix (read phase, write steering)
      │
      └── PTP via i226 (PHC timestamps relative to disciplined 25 MHz)
```

## What Timebeat software does

Timebeat reads PTP timestamps from the i226 PHC. Since the PHC is
clocked from the ClockMatrix's disciplined 25 MHz output, PTP offset
measurements reflect how well the ClockMatrix tracks GPS time. Timebeat
uses these PTP offsets for monitoring and reporting.

The actual GPS discipline of the OCXO-derived clock happens **in
hardware** — DPLL A is a closed-loop PLL that locks the fractional
divider output to the F9T PPS input. Timebeat does not steer this loop
in software (at least not in normal operation).

Timebeat may also run a second DPLL (B) in open-loop mode for comparing
the phase of a PPS from another clock on a "shelf" of multiple time
cards. This open-loop DPLL does phase measurement only, no feedback.

## What peppar-fix needs to do

To add value, we must **open DPLL A's loop** — switch it from hardware
PLL mode (autonomous) to software-steered mode (we provide the
corrections). This lets us replace the chip's internal loop filter with
our EKF-based estimator, which can use carrier-phase PPP (sub-ns) rather
than raw PPS (1.7 ns sawtooth) as the error signal.

### Steps

1. **Map DPLL → output routing**: Confirm which DPLL feeds the 25 MHz
   and PPS OUT that go to the i226. (Register dump captured in
   `data/clocktree_ptboat.json`.)

2. **Open the loop**: Switch that DPLL from PLL mode (pll_mode=0) to
   write_freq mode (pll_mode=1) or write_phase_set mode (pll_mode=3).
   This may require EEPROM reprogramming via Renesas Timing Commander.

3. **Read phase error**: After opening the loop, the TDC phase
   comparison between F9T PPS (CLK14) and OCXO feedback needs to be
   triggered after each PPS edge. The phase error register
   (DPLL_PHASE_STATUS) gives us the offset in ITDC_UIs.

4. **Write corrections**: Steer the fractional divider via the DCO
   frequency control word (WRITE_FREQUENCY, 42-bit, ±244 ppm,
   0.11 fppb resolution) or phase offset (WRITE_PH, 32-bit, in
   ITDC_UIs).

### Why this matters

The i226 TCXO-based PHC on TimeHat gives TDEV(1s) = 100-130 ps
(free-running). The Timebeat OCXO should give TDEV(1s) ~10-30 ps,
and the ClockMatrix's TDC resolution is 2.7 ps (fine mode). This is
a 5-50x improvement in the oscillator noise floor, which directly
improves the discipline result at all taus.

## Unconfirmed details

- [ ] Which DPLL number (0-3) drives the 25 MHz / PPS OUT to i226?
- [ ] What is DPLL A's current pll_mode in the EEPROM config?
- [ ] Can pll_mode be changed at runtime, or only via EEPROM reflash?
- [ ] How does Timebeat's software trigger phase measurements?
- [ ] Does the TDC auto-trigger on PPS edges, or need explicit polling?
- [ ] What clock input numbers correspond to the physical pins?
  (CLK14 = F9T PPS confirmed from register dump)

## Register references

- DPLL mode: page 0xC6, offset 0x02 (trigger register)
- Phase measurement config: page 0xC6, offset 0x30 (ref/fb clock select)
- Phase status (coarse): page 0xC0, offset 0xDC (36-bit, ITDC_UI resolution)
- Phase status (fine): page 0xC0, offset 0x24 (48-bit, ITDC_UI/128 resolution)
- DCO frequency write: page 0xC8, offset 0x00 (42-bit)
- Phase write: page 0xC8, offset 0x38 (32-bit)
- Input TDC config: page 0xCD, offset 0x20-0x26

## I2C access

| Host | Bus | Address | Notes |
|---|---|---|---|
| otcBob1 | /dev/i2c-15 | 0x58 | OTC SBC, has OCXO |
| ptBoat | /dev/i2c-16 | 0x58 | OTC Mini PT, preferred for dev |

Behind PCA9548 mux at 0x70 on /dev/i2c-1 (kernel-owned, UU).
Page register at offset 0xFC (1B addressing mode).

**HAZARD**: Writing to page register while Timebeat runs can corrupt
its state and crash it. Always restore page to 0x00 after reads. If
Timebeat crash-loops, power cycle the host (soft reboot insufficient).

## DPLL mapping (confirmed from ptBoat clean register dump)

| DPLL | Mode | Ref Input | Feedback | Role (inferred) |
|---|---|---|---|---|
| 0 | PLL (closed) | CLK8 | CLK2 | Secondary reference? |
| 1 | write_phase_set (open) | CLK14 (F9T PPS) | CLK12 | Timebeat phase comparison |
| 2 | **PLL (closed)** | **CLK14 (F9T PPS)** | **CLK3** | **Disciplines 25MHz → i226** |
| 3 | write_phase_set (open) | CLK14 (F9T PPS) | CLK12 | Timebeat phase comparison? |

**DPLL_2 is the target.** It's the closed-loop PLL that locks to F9T PPS
(CLK14) with feedback from CLK3 (likely the OCXO-derived divided clock).
This is the DPLL we need to open for peppar-fix steering.

Only OUTPUT_1 is enabled (src=24). Need to confirm OUTPUT_1 sources
from DPLL_2's output.

### To open DPLL_2

Change DPLL_2.MODE from pll_mode=0 (PLL) to pll_mode=1 (write_freq)
or pll_mode=3 (write_phase_set). This may require EEPROM reprogramming
via Renesas Timing Commander since runtime mode writes don't stick.

Once open, we:
1. Read phase error from DPLL_2 phase status registers
2. Run our EKF-based servo
3. Write frequency corrections to DPLL_2's DCO via WRITE_FREQUENCY register
