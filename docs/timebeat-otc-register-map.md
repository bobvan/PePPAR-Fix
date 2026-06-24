# Renesas 8A34002 ClockMatrix Register Map

**Date**: 2026-04-04 (updated 2026-06-23)
**Status**: Confirmed via live I2C reads on ptBoat and otcBob1

> ⚠️ **OTC vs Mini — DPLL and CLK-input assignments DIFFER between the two
> hosts. Do NOT cross-apply.** `otcBob1` is an **Open Time Card (OTC)**;
> `ptBoat` is an **OTC Mini PT**. They are different boards with different
> wiring. The clearest known difference (verified 2026-06-23 + Timebeat
> confirmation): **F9T PPS is on CLK2 on the OTC (otcBob1)** but **CLK5 on
> the Mini (ptBoat)**. Earlier revisions of this doc listed a single
> "confirmed" mapping for both — that conflated the two. Every host-specific
> fact below is now labelled OTC or Mini; treat any unlabelled legacy claim
> as suspect until re-verified on the specific board.

## Chip and register addressing

The chip is a Renesas 8A34002 (confirmed by Timebeat). The 8A34xxx
family shares a common register set (Linux kernel driver treats all
variants identically). Key differences from earlier 8A34002 research
docs that assumed page-register addressing:

- **16-bit register addressing** via `i2c_rdwr` (2-byte address prefix)
- **All 4 DPLLs share page 0xC3/0xC4** at different base offsets
- **Status module** at 0xC03C

## I2C access method

> **OTC (otcBob1) is behind a `pca954x` I2C mux** (verified 2026-06-23).
> The mux sits at `1-0070` on the PCIe i2c-1; its 8 downstream channels are
> the kernel virtual buses **i2c-15..i2c-22 = mux channels 0..7**. The
> ClockMatrix (0x58) is on **channel 0 = i2c-15**. The kernel mux framework
> auto-selects the channel when you open i2c-15, so no manual mux write is
> needed — but the chip is NOT directly on i2c-1. (`smbus2.SMBus(15)` works.)
> Access works with **timebeat stopped** — DPLL0/DPLL3 `MODE` read `0x00`
> (PLL mode) which can look like a dead bus; validate against a register
> known to be non-zero (e.g. DPLL1/DPLL2 `MODE = 0x20` synthesizer, or the
> input monitors at 0xC044), not against MODE. Mini (ptBoat) historically
> used bus 16; re-verify (mux topology may differ).

```python
import smbus2

bus = smbus2.SMBus(bus_num)  # OTC otcBob1: 15 (= pca954x mux ch0); Mini ptBoat: 16
addr = 0x58

# Read: write 2-byte register address, then read
msg_w = smbus2.i2c_msg.write(addr, [reg >> 8, reg & 0xFF])
msg_r = smbus2.i2c_msg.read(addr, nbytes)
bus.i2c_rdwr(msg_w, msg_r)
data = list(msg_r)

# Write: 2-byte register address followed by data bytes
msg = smbus2.i2c_msg.write(addr, [reg >> 8, reg & 0xFF] + data)
bus.i2c_rdwr(msg)
```

Do NOT use `write_byte_data(addr, 0xFC, page)` — that's the 8A34002
1B mode and produces garbage on the 8A34002.

## DPLL configuration registers

| DPLL | Base | MODE (base+0x37) | Notes |
|------|------|-------------------|-------|
| 0 | 0xC3B0 | 0xC3E7 | otcBob1: PLL/manual, holdover on CLK2 |
| 1 | 0xC400 | 0xC437 | Both hosts: PLL, freerun |
| 2 | 0xC438 | 0xC46F | Both hosts: PLL, freerun |
| 3 | 0xC480 | 0xC4B7 | otcBob1: PLL/manual, holdover on CLK2 |

### DPLL register offsets (from module base)

| Offset | Name | Size | Description |
|--------|------|------|-------------|
| 0x02 | CTRL_0 | 1 | force_lock_input[7:3], global_sync[2], revertive[1], hitless[0] |
| 0x03 | CTRL_1 | 1 | |
| 0x04 | CTRL_2 | 1 | |
| 0x0F | REF_PRIORITY_0 | 1 | Primary reference input (CLK0-15, 0x10=write_phase, 0x11=write_freq) |
| 0x10 | REF_PRIORITY_1 | 1 | Secondary reference |
| 0x11 | REF_PRIORITY_2 | 1 | Tertiary reference |
| 0x12 | REF_PRIORITY_3 | 1 | |
| 0x13 | REF_PRIORITY_4 | 1 | |
| 0x23 | FASTLOCK_CFG_0 | 1 | |
| 0x24 | FASTLOCK_CFG_1 | 1 | |
| 0x2E | WRITE_PHASE_TIMER | 1 | |
| 0x35 | REF_MODE | 1 | 0=automatic, 1=manual, 2=gpio, 3=slave |
| 0x36 | PHASE_MEASUREMENT_CFG | 1 | |
| 0x37 | MODE | 1 | pll_mode[5:3], state_mode[2:0] |

### DPLL_MODE encoding

> **Corrected 2026-06-17 (PR #185 review).** This section previously
> listed `pll_mode[2:0]` / `state_mode[4:3]`, which is **wrong** — it
> would have callers write `phase_meas=0x05` / `write_freq=0x02`, the
> latter actually selecting a *state* bit (force-lock), not write_freq.
> The fields are `pll_mode[5:3]` and `state_mode[2:0]`.  Ground truth:
> the running, TICC-confirmed actuator `scripts/peppar_fix/clockmatrix_actuator.py`
> (`_PLL_MODE_SHIFT=3`, `_PLL_MODE_MASK=0x07<<3`, write_freq=`0x10`).
> So **write_freq = 2 << 3 = `0x10`** and **phase_meas = 5 << 3 = `0x28`**.

Bits [5:3] — PLL mode (write the value `<< 3`; e.g. write_freq=`0x10`,
phase_meas=`0x28`):

| Value | Name | Description |
|-------|------|-------------|
| 0 | PLL | Closed-loop hardware PLL |
| 1 | write_phase | Software writes phase corrections |
| 2 | write_freq | Software writes frequency corrections |
| 3 | gpio_inc_dec | GPIO frequency adjustment |
| 4 | synthesizer | Fixed frequency output |
| 5 | phase_meas | Phase measurement only |
| 6 | disabled | DPLL off |

Bits [2:0] — State mode:

| Value | Name | Description |
|-------|------|-------------|
| 0 | automatic | Normal operation |
| 1 | force_lock | Force lock to configured ref |
| 2 | force_freerun | Force freerun (ignore refs) |
| 3 | force_holdover | Force holdover (keep last freq) |

**Runtime writes to MODE stick on the 8A34002.** Confirmed 2026-04-04:
wrote pll_mode=2 (write_freq) — i.e. byte `0x10` (`2 << 3`) — to DPLL_2,
read back confirmed. No Timing Commander or EEPROM reprogramming needed.

## DPLL control registers (frequency/phase write targets)

| DPLL_CTRL | Base | FOD_FREQ (base+0x1C) | PHASE_OFFSET_CFG (base+0x14) |
|-----------|------|----------------------|------------------------------|
| 0 | 0xC600 | 0xC61C | 0xC614 |
| 1 | 0xC63C | 0xC658 | 0xC650 |
| 2 | 0xC680 | 0xC69C | 0xC694 |
| 3 | 0xC6BC | 0xC6D8 | 0xC6D0 |

### DPLL_CTRL offsets

| Offset | Name | Size | Description |
|--------|------|------|-------------|
| 0x00 | HS_TIE_RESET | 1 | |
| 0x01 | MANU_REF_CFG | 1 | Manual reference config |
| 0x04 | BW | 4 | Loop bandwidth |
| 0x14 | PHASE_OFFSET_CFG | 6 | Phase offset (Timebeat writes here) |
| 0x1A | FINE_PHASE_ADV_CFG | 2 | Fine phase advance |
| 0x1C | FOD_FREQ | 6 | Fractional output divider frequency |
| 0x28 | COMBO_SW_VALUE_CNFG | 4 | |

## DPLL phase write registers

| DPLL_PHASE | Base | WRITE_PH (base+0x00) |
|------------|------|----------------------|
| 0 | 0xC818 | 0xC818 |
| 1 | 0xC81C | 0xC81C |
| 2 | 0xC820 | 0xC820 |
| 3 | 0xC824 | 0xC824 |

## Status registers

Status module base: **0xC03C**

All offsets below are added to 0xC03C.

### Input monitor (1 byte each)

| Offset | Register | Absolute |
|--------|----------|----------|
| +0x08..+0x17 | IN0_MON..IN15_MON | 0xC044..0xC053 |

Non-zero = signal present.

### DPLL status (1 byte each)

| Offset | Register | Absolute |
|--------|----------|----------|
| +0x18 | DPLL0_STATUS | 0xC054 |
| +0x19 | DPLL1_STATUS | 0xC055 |
| +0x1A | DPLL2_STATUS | 0xC056 |
| +0x1B | DPLL3_STATUS | 0xC057 |

Lock state in bits [2:0]: 0=freerun, 1=locked, 2=locking, 3=holdover,
4=write_phase, 5=write_freq.

### DPLL reference status (1 byte each)

| Offset | Register | Absolute |
|--------|----------|----------|
| +0x22 | DPLL0_REF_STAT | 0xC05E |
| +0x23 | DPLL1_REF_STAT | 0xC05F |
| +0x24 | DPLL2_REF_STAT | 0xC060 |
| +0x25 | DPLL3_REF_STAT | 0xC061 |

Current reference input in bits [4:0].

### DPLL filter status (fine phase, 8 bytes each)

| Offset | Register | Absolute |
|--------|----------|----------|
| +0x44 | DPLL0_FILTER_STATUS | 0xC080 |
| +0x4C | DPLL1_FILTER_STATUS | 0xC088 |
| +0x54 | DPLL2_FILTER_STATUS | 0xC090 |
| +0x5C | DPLL3_FILTER_STATUS | 0xC098 |

### DPLL phase status (coarse phase, 8 bytes each)

| Offset | Register | Absolute |
|--------|----------|----------|
| +0xDC | DPLL0_PHASE_STATUS | 0xC118 |
| +0xE4 | DPLL1_PHASE_STATUS | 0xC120 |
| +0xEC | DPLL2_PHASE_STATUS | 0xC128 |
| +0xF4 | DPLL3_PHASE_STATUS | 0xC130 |

### TDC measurement registers

| Offset | Register | Absolute | Size |
|--------|----------|----------|------|
| +0xAC | TDC_CFG_STATUS | 0xC0E8 | 1 |
| +0xAD..+0xB0 | TDC0..TDC3_STATUS | 0xC0E9..0xC0EC | 1 each |
| +0xB4 | TDC0_MEASUREMENT | 0xC0F0 | 16 |
| +0xC4 | TDC1_MEASUREMENT | 0xC100 | 8 |
| +0xCC | TDC2_MEASUREMENT | 0xC108 | 8 |
| +0xD4 | TDC3_MEASUREMENT | 0xC110 | 8 |

### Input frequency status (2 bytes each)

| Offset | Register | Absolute |
|--------|----------|----------|
| +0x8C + 2*i | INi_FREQ_STATUS | 0xC0C8 + 2*i |

Bits [13:0] = signed frequency offset.
Bits [15:14] = unit: 0=1ppb, 1=10ppb, 2=100ppb, 3=1000ppb.
Value of -8192 at 1000ppb = no signal / saturated.

## Output registers

| Output | Address | Size |
|--------|---------|------|
| 0 | 0xCA14 | 8 |
| 1 | 0xCA24 | 8 |
| 2 | 0xCA34 | 8 |
| 3 | 0xCA44 | 8 |
| 4 | 0xCA54 | 8 |
| 5 | 0xCA64 | 8 |
| 6 | 0xCA80 | 8 |
| 7 | 0xCA90 | 8 |

## TDC subsystems — Output TDC & Input TDC

Module base addresses from the mainline Linux `idt8a340_reg.h` (v5.15;
removed in later kernels) — a primary source — and **verified live on
otcBob1 2026-06-23** (these addresses respond; OUTPUT_TDC_0 is configured).

| Module | Address | Size | Notes |
|--------|---------|------|-------|
| OUTPUT_TDC_CFG | 0xCCD0 | 8 | global Output-TDC config; live byte[4]=0x03 |
| OUTPUT_TDC_0 | 0xCD00 | 8 | **configured live** (`0A 00 00 00 06 03 07 00`) |
| OUTPUT_TDC_1 | 0xCD08 | 8 | unused (all 0) |
| OUTPUT_TDC_2 | 0xCD10 | 8 | unused (all 0) |
| OUTPUT_TDC_3 | 0xCD18 | 8 | unused (all 0) |
| INPUT_TDC | 0xCD20 | 8 | input-TDC config (all 0 live) |

**Full register definitions** — from the *8A3xxxx Family Programming Guide
v49* (©2023-04-06; pp.285-291, §OUTPUT_TDC), publicly downloadable from
renesas.com (no login). Archived: `gt:~/gt/datasheets/renesas-8a3xxxx-family-
programming-guide-v49.pdf`. Companion: ClockMatrix TDC Manual R31UZ0005EU.

OUTPUT_TDC_n module register index (offset from module base, e.g. 0xCD00):

| Off | Register | Fields |
|-----|----------|--------|
| 000h | OUTPUT_TDC_CTRL_0 | `SAMPLES[15:0]` — # samples to average (0 = 4096); one sample/100µs |
| 002h | OUTPUT_TDC_CTRL_1 | `TARGET_PHASE_OFFSET[15:0]` signed ps — **alignment mode only** |
| 004h | OUTPUT_TDC_CTRL_2 | `ALIGN_TARGET_MASK[7:0]` — **alignment mode only** (bit i = DPLLi) |
| **005h** | OUTPUT_TDC_CTRL_3 | `TARGET_INDEX[7:4]` / `SOURCE_INDEX[3:0]`: 0–7=DPLL0–7, 8=GPIO6, 9=GPIO1, A=GPIO2, B=GPIO7 |
| **006h** | OUTPUT_TDC_CTRL_4 | **TRIGGER byte**: `GO[0]`, `MODE[1]` (0=measure/1=align), `TYPE[2]` (0=single/1=continuous), `ALIGN_RESET[3]`, `DISABLE_MEASUREMENT_FILTER[7]` |

A DPLL operand is measured via its **Master Sync** (fixed 9-FOD-cycle delay to
the output). **Raw CLK inputs are NOT selectable** — to compare against the F9T
PPS (a CLK input), present it as a DPLL Master Sync (a DPLL referencing it) or
wire it to GPIO1/2.

Global config `OUTPUT_TDC_CFG` (0xCCD0): GBL_0=000h `FAST_LOCK_ENABLE_DELAY`,
GBL_1=002h `FAST_LOCK_DISABLE_DELAY`, **GBL_2=004h (0xCCD4)** = `ENABLE[0]`,
`REF_SEL[1]` (0=XTAL,1=XO_DPLL); writing GBL_2 activates the module.

Measurement readout `STATUS.OUTPUT_TDCn_MEASUREMENT` (n=0:0xC0F0, 1:0xC100,
2:0xC108, 3:0xC110): **`PHASE[47:0]` = signed 48-bit integer in PICOSECONDS**
(= sum/of/samples, one per 100µs). **Positive = target edge leads source.**
Status: `OUTPUT_TDC_CFG_STATUS`@0xC0E8, `OUTPUT_TDCn_STATUS`@0xC0E9..EC.

**RESOLUTION — Output TDC ≠ 50 ps measurement; Input TDC IS** (measured
2026-06-23, otcBob1). The "50 ps" associated with the **Output** TDC is its
*fixed alignment threshold* (TDC Manual R31UZ0005EU §2.9: "TDC mechanism only
adjusts … when it drifts by more than 50ps. The 50ps threshold is fixed"), NOT
a measurement step. The Output TDC is an **alignment** engine; its measurement
mode is **~2 ns single-sample** (one ref-clock period — live DPLL3-vs-DPLL0 read
only 0/2000/4000 ps), improvable only by **`SAMPLES` averaging** (slow: ~N s at a
1 Hz Master Sync). The high-resolution **measurement** instrument is the
**Input TDC** (a DPLL's PFD): `DPLLn_PHASE_STATUS`@0xC118+ reads on a **50 ps
grid** (signed 36-bit × 50 ps — verified: every PFD sample a multiple of 50 ps,
tracing the ~8 ns F9T sawtooth), and `DPLLn_FILTER_STATUS`@0xC080+ gives the
high-precision path (× 50/128 = **0.39 ps**; needs the `tdc_clk` divider set so
it's not an integer multiple of the input). 625 MHz default TDC clock.
**For 50 ps / sub-ps F9T-vs-output: use the Input TDC with PPS OUT looped to a
spare CLK input — NOT two GPIOs into the Output TDC.**

**Measurement procedure (spare Output TDC):** subsystem already enabled by
Timebeat (CFG GBL_2 = 0x03) → don't touch CFG; pick a spare module; write
CTRL_0 (samples), CTRL_3 (target/source), then CTRL_4 with `MODE=0, GO=1`
(writing CTRL_4 triggers); poll `OUTPUT_TDCn_STATUS`/read MEASUREMENT.

**⚠️ Live: OUTPUT_TDC_0 is IN USE by Timebeat** — `0A 00 00 00 06 03 07 00`
decodes to SAMPLES=10, ALIGN_TARGET_MASK=0x06 (DPLL1+DPLL2), CTRL_3 SOURCE=
DPLL3, CTRL_4=0x07 (`GO+MODE=align+TYPE=continuous`) → continuously aligning
DPLL1/DPLL2 outputs to DPLL3. **Use OUTPUT_TDC_1/2/3 (read all-zero = free)
for measurement; do not disturb OUTPUT_TDC_0.**

### GPIO config bases (for "wire F9T PPS → GPIO" option)

`GPIO_USER_CONTROL`=0xC160; per-GPIO config blocks: GPIO_0=0xC8C2,
**GPIO_1=0xC8D4, GPIO_2=0xC8E6**, GPIO_3=0xC900 … GPIO_7=0xC948 (18 bytes
each). GPIO1/GPIO2 are externally exposed on the OTC and are valid Output-TDC
operands (0x9 / 0xA) — the path for a raw PPS edge into the Output TDC.

## Other registers

| Register | Address | Size |
|----------|---------|------|
| HARDWARE_REVISION | 0x8180 | 4 |
| RESET_CTRL | 0xC000 | ? |
| GENERAL_STATUS | 0xC014 | 8 |
| OTP | 0xCF70 | ? |
| BYTE | 0xCF80 | ? |

## Clock input mapping — **OTC and Mini DIFFER**

⚠️ Prior revisions listed one mapping "confirmed" for both hosts; that was a
conflation. The two boards are wired differently.

### OTC (otcBob1) — verified 2026-06-23 (live recon + Timebeat data point)

| Input | OTC Signal | Evidence |
|-------|-----------|----------|
| **CLK2** | **F9T PPS** | Timebeat confirmation + live: CLK2 active (freq≈0), DPLL0/DPLL3 ref=CLK2, config `pps_clk=2` |
| CLK5 | **dead** (no signal) | live: `-8192` = no signal/saturated on OTC (this is the *Mini's* F9T PPS input, not the OTC's) |
| CLK0, CLK8, CLK10, CLK11 | active (roles TBD) | present in input monitors, freq≈0 |
| CLK1, CLK3, CLK13 | present but unqualified | input-monitor nonzero, freq `-8192` |
| OCXO | **input TBD on OTC** | NOT confirmed to be CLK2 (that was the old wrong claim); needs identification |

### Mini (ptBoat) — prior recon

| Input | Mini Signal | Evidence |
|-------|------------|----------|
| CLK5 | F9T PPS | DPLL3 locked to CLK5, PFD ~25.3 ns (ptBoat recon) |
| CLK2 | OCXO | only input with real freq data (−106 ppb) |
| others | inactive | only CLK2,5 active on the Mini |

## Host comparison

| Aspect | otcBob1 (OTC SBC) | ptBoat (OTC Mini PT) |
|--------|-------------------|----------------------|
| I2C bus | 15 (= pca954x mux **ch0**) | 16 |
| F9T PPS input | **CLK2** (CLK5 dead) | **CLK5** (CLK2 = OCXO) |
| Active inputs | 9 (CLK0,1,2,3,5,8,10,11,13) | 2 (CLK2,5) |
| DPLL_0 | PLL, holdover, ref=CLK2 (2026-06-23) | PLL, freerun |
| DPLL_1 | **synthesizer, freerun** (2026-06-23) | PLL, freerun |
| DPLL_2 | **synthesizer, freerun** (2026-06-23) | PLL, freerun |
| DPLL_3 | PLL, holdover, ref=CLK2; FCW actuator (steering disabled — feeds i226) | PLL, freerun |
| Timebeat DCO | Active (freq_rho on DPLL_3) | Inactive (clkgen commented out) |
| DPLL_3 PHASE_OFFSET | Live value (Timebeat steering) | Zero |
| DPLL_0 PHASE_STATUS | Non-zero (measuring) | Zero |
