# Timebeat OTC Signal Routing and ClockMatrix Architecture

**CORRECTION (2026-04-04)**: The chip is an 8A34012, not 8A34002.
See `timebeat-otc-register-map.md` for the correct register map.
Register addresses in this document are wrong. The DPLL mapping and
signal routing analysis below is also outdated — see the register map
doc for confirmed DPLL configurations on both hosts.

**Status: Partially reverse-engineered. Some details unconfirmed.**

> **Mini PT (ptBoat) wiring** — the authoritative CLK/DPLL/output map and the
> OTC-vs-Mini DPLL topology are Timebeat board-specific detail and are kept in
> the **private lab repo** (`timebeat-mini-wiring.md`), not in this public repo.

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

### Two-tier architecture (corrected 2026-04-06)

peppar-fix operates alongside the ClockMatrix hardware PLL, not
instead of it.  There are two independent control loops:

**Tier 1 — PHC frequency (Channel 0 / DPLL_2, hardware PLL):**
Timebeat wires F9T PPS to CLK14 and configures DPLL_2 as a closed PLL
that synthesises GPS-disciplined 25 MHz for the i226 PHY and PHC
clocks.  This handles PHC frequency automatically.  peppar-fix sets
`adjfine(0)` and only needs to step the PHC phase at bootstrap.

- With Timebeat stopped, DPLL_2 goes to holdover/freerun and the
  25 MHz free-runs on the OCXO (~79 ppm on otcBob1).  In this state
  peppar-fix must use `adjfine()` to steer PHC frequency (like
  TimeHat).  A future improvement: re-lock DPLL_2 to PPS ourselves
  so Timebeat doesn't need to run.
- The DPLL_2 loop bandwidth controls how much F9T PPS phase noise
  (sawtooth, 1.7 ns RMS) leaks into the 25 MHz and therefore into
  PHC timestamps.  Lower bandwidth = cleaner PHC, slower tracking.

**Tier 2 — PPS OUT (Channel 3 / DPLL_3, software-steered via FCW):**
A separate output channel produces a high-quality PPS that peppar-fix
steers using carrier-phase PPP + qErr knowledge.  This PPS output is
far more precise than what DPLL_2's hardware PLL can achieve from
raw PPS alone.

- FCW on DPLL_3 controls PPS output timing, NOT PHC frequency.
  Changing FCW is invisible to EXTTS (which timestamps using the PHC
  clocked by DPLL_2's 25 MHz).
- FCW steering is visible on: the Input TDC for DPLL_3, and an
  external TICC measuring PPS OUT from that channel.
- This is where the ClockMatrix's 0.111 fppb FCW resolution and
  50 ps TDC resolution provide real value over the PHC-only approach.

### Why this matters

The i226 TCXO-based PHC on TimeHat gives TDEV(1s) = 100-130 ps
(free-running). The Timebeat OCXO should give TDEV(1s) ~10-30 ps,
and the ClockMatrix's TDC resolution is 2.7 ps (fine mode). This is
a 5-50x improvement in the oscillator noise floor, which directly
improves the discipline result at all taus.

Tier 1 (DPLL_2 PLL) gets us a GPS-locked PHC with OCXO stability.
Tier 2 (DPLL_3 FCW) adds sub-ns PPS output steering on top.

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

## E810-XXVDA4T GNSS Integration Notes

The E810's onboard ZED-F9T connects via **I2C** internally (not UART).
The kernel ice driver exposes it as `/dev/gnss0` (type: UBX).

Key differences from EVK F9T on serial port:

| Aspect | EVK F9T (serial) | E810 F9T (kernel GNSS) |
|---|---|---|
| Device | /dev/ttyACMx or /dev/gnss-top | /dev/gnss0 |
| Open method | pyserial Serial() | open("r+b") |
| Port config | CFG_MSGOUT_*_UART1 | CFG_MSGOUT_*_I2C |
| Baud rate | 115200-460800 | N/A (kernel handles) |
| DTR reset | Yes (Arduino on same bus) | No |
| UBX + NMEA | Both default | NMEA default, UBX on request |

To enable RAWX observations on E810:
```python
messages = {
    "CFG_MSGOUT_UBX_RXM_RAWX_I2C": 1,
    "CFG_MSGOUT_UBX_RXM_SFRBX_I2C": 1,
    "CFG_MSGOUT_UBX_NAV_PVT_I2C": 1,
    "CFG_MSGOUT_UBX_TIM_TP_I2C": 1,
}
```

SEC-UNIQID confirmed: `675836739647` (ZED-F9T, TIM 2.20, PROTVER 29.20)
