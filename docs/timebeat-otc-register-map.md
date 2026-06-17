# Renesas 8A34002 ClockMatrix Register Map

**Date**: 2026-04-04
**Status**: Confirmed via live I2C reads on ptBoat and otcBob1

## Chip and register addressing

The chip is a Renesas 8A34002 (confirmed by Timebeat). The 8A34xxx
family shares a common register set (Linux kernel driver treats all
variants identically). Key differences from earlier 8A34002 research
docs that assumed page-register addressing:

- **16-bit register addressing** via `i2c_rdwr` (2-byte address prefix)
- **All 4 DPLLs share page 0xC3/0xC4** at different base offsets
- **Status module** at 0xC03C

## I2C access method

```python
import smbus2

bus = smbus2.SMBus(bus_num)  # 15 for otcBob1, 16 for ptBoat
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

## Other registers

| Register | Address | Size |
|----------|---------|------|
| HARDWARE_REVISION | 0x8180 | 4 |
| RESET_CTRL | 0xC000 | ? |
| GENERAL_STATUS | 0xC014 | 8 |
| OTP | 0xCF70 | ? |
| BYTE | 0xCF80 | ? |

## Clock input mapping (confirmed)

| Input | otcBob1 Signal | ptBoat Signal | Evidence |
|-------|---------------|---------------|----------|
| CLK2 | OCXO | OCXO | Only input with real freq data (-106 ppb on ptBoat, 0 ppb on otcBob1) |
| CLK5 | F9T PPS | F9T PPS | DPLL_0/3 priority, active on both hosts |
| CLK0 | ? (active on otcBob1) | inactive | |
| CLK1 | ? (active on otcBob1) | inactive | |
| CLK3 | OCXO-derived? (active on otcBob1) | inactive | DPLL_1/2 priority 0 |

## Host comparison

| Aspect | otcBob1 (OTC SBC) | ptBoat (OTC Mini PT) |
|--------|-------------------|----------------------|
| I2C bus | 15 | 16 |
| Active inputs | 9 (CLK0,1,2,3,5,8,10,11,13) | 2 (CLK2,5) |
| DPLL_0 | PLL/manual, holdover, ref=CLK2 | PLL, freerun |
| DPLL_1 | PLL, freerun | PLL, freerun |
| DPLL_2 | PLL, freerun | PLL, freerun |
| DPLL_3 | PLL/manual, holdover, ref=CLK2 | PLL, freerun |
| Timebeat DCO | Active (freq_rho on DPLL_3) | Inactive (clkgen commented out) |
| DPLL_3 PHASE_OFFSET | Live value (Timebeat steering) | Zero |
| DPLL_0 PHASE_STATUS | Non-zero (measuring) | Zero |
