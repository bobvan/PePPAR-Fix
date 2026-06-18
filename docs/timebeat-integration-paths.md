# peppar-fix on Timebeat OTC: Integration Paths

**Date**: 2026-04-04
**Status**: Ready to implement. No Timing Commander needed.

## Key finding: runtime register writes work

The 8A34002 accepts runtime writes to the DPLL MODE register via I2C.
Confirmed 2026-04-04: wrote pll_mode=2 (write_freq) to DPLL_2 on
ptBoat, read back confirmed, restored original. This means we can
reconfigure DPLLs at runtime without EEPROM reprogramming or Renesas
Timing Commander.

## Two hosts, two paths

### Path A: ptBoat (OTC Mini PT) — easier

**Why easier**: Timebeat is not steering the ClockMatrix. All DPLLs are
in freerun. The OCXO feeds the 25 MHz output to the i226 via the
ClockMatrix dividers, but no feedback loop is active. There is nothing
to conflict with.

**Approach**: Switch one DPLL (e.g., DPLL_2) to write_freq mode, lock
it to CLK2 (OCXO), and steer it from peppar-fix. Or: just use PHC
adjfine on the i226 (same as TimeHat) and benefit from the OCXO's
lower noise floor without touching the ClockMatrix at all.

**Steps for PHC-only (simplest possible)**:
1. Stop Timebeat: `sudo systemctl stop timebeat`
   (Optional — Timebeat isn't adjusting anything, but avoids I2C bus
   contention if we later add ClockMatrix reads)
2. Run peppar-fix with `--servo /dev/ptp0 --pps-pin 1` (or whatever
   the i226 PTP device is on ptBoat)
3. The OCXO's stability gives better TDEV than TimeHat's TCXO at all
   taus, even with the same adjfine interface

**Steps for ClockMatrix write_freq (advanced)**:
1. Stop Timebeat
2. Switch DPLL_2 to write_freq mode:
   ```python
   # Write pll_mode=2 (write_freq).  pll_mode is MODE bits[5:3], so the
   # byte value is 0x10, NOT 0x02 (0x02 = pll_mode 0 = PLL).  See the
   # "CRITICAL BIT FIX" below; ground truth: clockmatrix_actuator.py.
   write16(0xC46F, [0x10])
   ```
3. Read phase error from DPLL_0 or DPLL_2 phase status registers
4. Write frequency corrections to DPLL_CTRL_2.FOD_FREQ (0xC69C) or
   DPLL_PHASE_2.WRITE_PH (0xC820)
5. The i226 25 MHz tracks automatically since it comes from the
   ClockMatrix output

**HAZARD (confirmed 2026-04-04)**: Switching a DPLL to write_freq mode
at runtime may glitch the output frequency. If that DPLL feeds the
i226's 25 MHz clock, the NIC driver or kernel can crash, taking the
host offline. **otcBob1 was lost this way** — required power cycle.
Before switching any DPLL, determine the output routing to identify
which DPLL feeds the i226. Use a DPLL that does NOT feed the i226
for initial experiments, or ensure the mode transition is glitch-free.

**FOD_FREQ encoding (confirmed)**:
- 8 bytes: M (6 bytes LE, 48-bit) + N (2 bytes LE, 16-bit)
- Output frequency = M / N (in Hz at VCO level)
- Nominal: M=32,767,500,000,000, N=65,535 → M/N = 500 MHz (VCO)
- To steer by Δf ppb: M_new = M_NOM + M_NOM × Δf × 10⁻⁹
- Resolution: 1 count / 32,767,500,000,000 ≈ 0.03 ppt (30 fppb)
- Constants from Go source: PPS_M=32767500000000, PPS_N=65535,
  PPS_DIV=500000000, PPS_DC=1000000000

**Resolved (2026-04-04)**:
- **DPLL_3 feeds the i226** — OUTPUT_1 (25 MHz, divider=20) has
  CTRL_0=0x03 (DPLL_3). OUTPUT_0 (10 MHz, divider=50) also DPLL_3.
- **FOD_FREQ writes work without mode change** — write to
  DPLL_CTRL_3.FOD_FREQ (0xC6D8) while DPLL_3 stays in PLL mode.
  No crash. DPLL_3 transitions from holdover to a new state (0x10).
- **DPLL_0 phase status is live** — accumulates at ~25 ps per count.
  At +10 ppb, 400,540 counts/s ÷ (10 ns/s) = 25.0 ps/count.
- **No mode change needed** — steer via FOD_FREQ in PLL mode, read
  phase from DPLL_0 phase status. This is the safe, confirmed path.

**write_freq mode confirmed working (2026-04-04)** — ⚠️ SUPERSEDED / value
mislabeled (reconciled per #185 review): the `0x02` written here is `pll_mode=0`
(PLL) at the correct MODE bits `[5:3]`, **not** write_freq. So what this section
actually exercised was **FOD_FREQ in PLL mode** (the ±8 ppb wrong-register path,
retracted under "FCW steering confirmed" below), not true write_freq. The real
write_freq/FCW path is `DPLL_FREQ_3` (0xC850) with MODE byte **0x10** — see the
"CRITICAL BIT FIX". Kept for history; **do not copy the 0x02 values.**
- Switched DPLL_3 from PLL (0x00) to "write_freq" (wrote `0x02` = PLL at `[5:3]`;
  true write_freq is `0x10`) at nominal FOD_FREQ
- Host survived, TICC continued seeing PPS (phase step but no loss)
- +100 ppb FOD_FREQ → +8,736 ps/s TICC drift (same gain as PLL mode)
- Gain factor: **0.0874** (commanded FOD ppb × 0.0874 = output ppb)
- The M/N = 500 MHz model is wrong — actual sensitivity is ~2.67 ppt/count
- Gain is linear and stable (±3% epoch jitter over 30s test)
- Baseline drift -250 ps/s (OCXO vs Geppetto 10 MHz) cleanly restored

**To steer by X ppb on the output**: write M_NOM + X/0.0874 * 327675 counts.
Or equivalently: delta_M = X * 3,750,000 (empirical, needs refinement).
Resolution: 0.0874 / 327675 ≈ 0.27 ppt per M count — extraordinary.

**Safe mode switch procedure**:
1. Write nominal M to FOD_FREQ (ensure frequency matches current output)
2. Switch MODE from PLL (0x00) to write_freq (`0x10` — NOT `0x02`; pll_mode is
   bits[5:3])
3. Phase step occurs but no frequency discontinuity — host survives
4. To restore: write nominal FOD, switch back to PLL mode (0x00)

**FCW steering confirmed (2026-04-04)**:
- The FOD_FREQ approach had only ±8 ppb range (non-linear). WRONG REGISTER.
- The correct register is **DPLL_FREQ_3** at 0xC850 (6 bytes, 42-bit FCW)
- CRITICAL BIT FIX: pll_mode is in MODE bits[5:3], not [2:0].
  Write 0x10 (not 0x02) to set write_freq mode.
- FCW = (1 - 1/(1+ppb/1e9)) × 2^53 ≈ ppb × 9,007,199
- **Gain = 1.000** confirmed by TICC at ±100, ±500, ±1000 ppb
- Range: ±244 ppm. Resolution: 0.111 fppb (femto-ppb)
- Linear, immediate, no trigger needed
- Host survives mode switch when done at nominal FOD_FREQ
- DPLL_0 phase status measures OCXO drift (not useful as servo feedback)
- Phase measurement: use EXTTS on i226 (PHC clocked from steered output)

**Output TDC configuration (2026-04-05)**:
- Per-TDC config registers at 0xCD00 (TDC0), 0xCD08 (TDC1), etc.
- CTRL_3 (offset +5): SOURCE_INDEX[3:0] + TARGET_INDEX[7:4]
  - Source: 0-7=DPLL0-DPLL7
  - Target: 0-7=DPLL0-DPLL7, 8=GPIO6, 9=GPIO1, 10=GPIO2, 11=GPIO7
- CTRL_4 (offset +6): GO[0], MODE[1] (0=meas,1=align), TYPE[2] (0=single,1=continuous)
- Measurement: STATUS + 0xB4 (TDC0), 6 bytes signed LE **in picoseconds**
- Default config: source=DPLL_3, target=DPLL_0 → measures ~0 ps (same clock tree)
- Problem: F9T PPS is on CLK5, but the TDC targets are GPIOx indices,
  not CLK indices. We don't know which GPIO CLK5 maps to physically.

**Blocking question (asked Timebeat 2026-04-05):**
Which GPIO pin on the 8A34002 is the F9T PPS (CLK5) connected to on the
OTC SBC? Need this to set OUTPUT_TDC CTRL_3 TARGET_INDEX correctly.
Once we know, write target=GPIOx to CTRL_3 and the TDC measures
PPS vs DPLL_3 output in picoseconds at ~50 ps resolution — exactly
what the servo needs.

**Remaining unknowns**:
- CLK5-to-GPIO mapping on OTC SBC board (waiting on Timebeat)
- Output routing for outputs 2-7

### Path B: otcBob1 (OTC SBC) — more complex but more data

**Why more complex**: Timebeat IS actively steering DPLL_3 via
PHASE_OFFSET_CFG. DPLL_0 is producing live phase measurements.
Stopping Timebeat puts DPLLs in holdover (not freerun), and the
phase measurement may continue or freeze.

**Why more data**: otcBob1 has 9 active clock inputs vs ptBoat's 2.
DPLL_0's phase status is already non-zero — the TDC is measuring
something. If we can read that phase error, we get ClockMatrix TDC
resolution (~2.7 ps fine) without any DPLL reconfiguration.

**Approach**: Stop Timebeat. Check if DPLL_0 phase status still
updates (it's in holdover, not write_freq). If it does, we have a
live TDC measurement without touching any mode registers.

**Steps**:
1. Stop Timebeat: `sudo systemctl stop timebeat`
2. Read DPLL_0 phase status twice, 1 second apart:
   ```python
   # 0xC118 = MOD_STATUS + STATUS_DPLL0_PHASE_STATUS
   data1 = read16(0xC118, 8)
   time.sleep(1)
   data2 = read16(0xC118, 8)
   # If data1 != data2, TDC is live
   ```
3. If live: use DPLL_0 phase status as the error signal for our servo
4. Steer via PHC adjfine on the i226, or switch DPLL_3 to write_freq
   and steer the ClockMatrix directly
5. If frozen: switch DPLL_0 to phase_meas mode (pll_mode=5):
   ```python
   write16(0xC3E7, [0x28])  # phase_meas (pll_mode=5 at bits[5:3]); NOT 0x05
   ```

**Unknowns**:
- Does DPLL_0 phase status update in holdover after Timebeat stops?
- What's the phase status encoding? (8 bytes — need datasheet or
  reverse engineering from Timebeat's Go parsing code)
- Is CLK5 (F9T PPS) the reference for the phase measurement, or CLK2?

## Recommendation: start with ptBoat PHC-only

The PHC-only path on ptBoat requires zero ClockMatrix work. It gives
us an immediate comparison point:

| Platform | Oscillator | Steering | Expected TDEV(1s) |
|----------|-----------|----------|-------------------|
| TimeHat | TCXO | PHC adjfine | 1.17 ns (measured) |
| ptBoat | OCXO | PHC adjfine | ~0.1-0.3 ns (est.) |
| ptBoat | OCXO | ClockMatrix write_freq | ~0.01-0.03 ns (est.) |

The PHC-only run proves the OCXO value. The ClockMatrix path adds
finer control resolution (0.11 fppb vs ~1 ppb adjfine) which matters
at long tau.

## What we need from Timebeat (nice to have, not blocking)

1. **Phase status register encoding**: How to convert the 8-byte
   DPLL_PHASE_STATUS to picoseconds. The Go code must have a parser.

2. **FOD_FREQ encoding**: How to convert a frequency offset in ppb to
   the 6-byte FOD_FREQ format. All 4 DPLLs show `00 9B 32 47 CD 1D`
   which must represent the nominal output frequency.

3. **Output routing**: Which output (0-7) sources from which DPLL?
   OUTPUT_0 and OUTPUT_1 have different first bytes (0x32 vs 0x14)
   which likely encode the source DPLL.

4. **Input monitor status encoding**: What do the non-zero status
   bytes mean? (0x10, 0x11, 0x50, 0x55 on various inputs)

## Phase status register encoding (needs confirmation)

Based on the 8A34002 datasheet (may differ for 8A34002):

**Coarse (DPLL_PHASE_STATUS, 8 bytes)**:
- Signed value in ITDC_UIs
- ITDC_UI depends on input TDC FBD configuration
- On the 8A34002: ITDC_UI = 1/(32 * fbd_integer * ref_freq)
- Need to read FBD config registers to determine resolution

**Fine (DPLL_FILTER_STATUS, 8 bytes)**:
- Signed value in ITDC_UI/128
- 128x finer than coarse

The otcBob1 DPLL_0 phase status `E8 FF FF FF 0F 00 00 00` as a
little-endian signed 64-bit: 0x0000000FFFFFFFE8 = 68,719,476,712.
At 340 ps/ITDC_UI that would be ~23 seconds — clearly wrong. The
encoding must be different for the 8A34002, or only some bytes are
the phase value. Needs Timebeat Go parser or datasheet.
