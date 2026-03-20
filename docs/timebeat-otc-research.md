# peppar-fix on Timebeat OTC: Renesas ClockMatrix Integration Research

## Summary

**Yes, this is feasible and potentially very valuable.** The Renesas
8A34002 ClockMatrix on the OTC SBC exposes its DPLL, DCO, and TDC via
I2C. We can open the DPLL loop, read phase error, and steer the
oscillator independently. This gives us a much better measurement and
control path than the i226 PHC alone.

## Hardware on otcBob1

- **Renesas 8A34002** at I2C address `0x58` on bus `/dev/i2c-1`
  (address `0x70` is likely a mux, shown as `UU` = in use by kernel)
- **i226-LM NIC** with PHC at `/dev/ptp0`
- **OCXO** driven by the ClockMatrix output
- **F9T** on `/dev/ttyAMA0` at 460800 baud
- **PPS** from F9T to SDP0

## Can we open the DPLL loop?

**Yes.** The ClockMatrix supports "write frequency" mode where the
DPLL loop filter is bypassed and an external processor (our Pi)
controls the DCO directly via I2C:

- `WRITE_FREQUENCY_EN_DPLLn` — enables external DCO control
- `WRITE_FREQUENCY_DPLLn[41:0]` — 42-bit 2's complement frequency
  control word. Range: ±244 ppm. Resolution: **1.1 × 10⁻⁷ ppb**
  (0.11 femto-ppb), which is extraordinarily fine

When this mode is active, the internal phase detector and loop filter
are bypassed. Filtering is done by our software. The frequency offset
written to the register is passed directly to the output clock.

Source: [8A34002 datasheet](https://www.mouser.com/datasheet/2/698/REN_8A34002_DST_20230813-1996740.pdf)

## Can we read phase error?

**Yes.** The ClockMatrix TDC (Time-to-Digital Converter) provides
continuous phase measurements between any two reference clocks:

- Measures phase offset between PPS input and OCXO-derived clock
- Resolution: better than **50 ps** with averaging (TDC clock must be
  asynchronous to input for sub-50ps resolution)
- Range: ±0.86 seconds before saturation
- Readable via I2C registers

This is far better than the i226 PHC's `extts` resolution (1 ns) or
the TICC (60 ps single-shot). The TDC gives us a built-in,
high-resolution phase comparator right on the chip.

Source: [ClockMatrix TDC Application Note AN-1010](https://www.renesas.com/en/document/apn/clockmatrix-time-digital-converter)

## Can we steer phase independently from frequency?

**Yes.** The ClockMatrix provides separate controls:

- **Frequency steering**: via DCO write mode (42-bit FCW)
- **Phase offset**: via output coarse phase alignment registers
- **Phase step**: can apply discrete phase jumps

This is exactly what peppar-fix needs: frequency steering for the
servo loop, and phase steps for initial alignment.

Source: [ClockMatrix Output Coarse Phase Alignment](https://community.renesas.com/analog-products/timing/f/forum/33778/clockmatrix-output-coarse-phase-alignment)

## What Timebeat is doing with it

From the Timebeat config on otcBob1:

```yaml
clkgen:dco:strategy:freq_rho    # Discipline oscillator using freq-rho strategy
clkgen:input:offset:2:20000     # Static offset in picoseconds
clkgen:ocxo:aging_compensation  # OCXO aging compensation
```

Timebeat runs its own DPLL in software. It reads the phase error from
the ClockMatrix TDC, applies its own loop filter (the "freq_rho"
strategy), and writes frequency corrections back to the DCO. The
hardware DPLL loop is open — Timebeat IS the loop.

This means **we can do the same thing**. The ClockMatrix is already
configured as a measurement + actuation device, not a closed-loop
controller. We just need to:

1. Stop Timebeat (`systemctl stop timebeat`)
2. Read TDC phase error via I2C
3. Run peppar-fix's PI servo
4. Write frequency corrections to the DCO via I2C

## Comparison: i226 PHC vs ClockMatrix

| Aspect | i226 PHC (current) | ClockMatrix (proposed) |
|---|---|---|
| Phase measurement | extts, 1 ns resolution | TDC, <50 ps resolution |
| Frequency control | adjfine (±62.5 ppm, ~ppb resolution) | DCO FCW (±244 ppm, 0.11 fppb resolution) |
| Oscillator | TCXO (100-130 ps TDEV(1s)) | OCXO (expected ~10-30 ps TDEV(1s)) |
| Phase step | phc_ctl adj (ns) | Coarse phase alignment register |
| Interface | ioctl/sysfs | I2C register read/write |

The ClockMatrix gives us 20x better phase measurement, 1000x better
frequency control resolution, and a much quieter oscillator. The
tradeoff is more complex I2C programming vs the standard Linux PTP API.

## Implementation plan

### Phase 1: Measurement only (low risk)
- Stop Timebeat on otcBob1
- Write a Python script that reads TDC phase error via I2C
- Compare TDC readings with TICC measurements for validation
- No steering, just measurement

### Phase 2: Open-loop DCO steering
- Add I2C DCO write to peppar-fix as a new "actuator" alongside PHC adjfine
- Run peppar-fix servo with ClockMatrix as both sensor (TDC) and actuator (DCO)
- Compare stability with i226 PHC servo

### Phase 3: Unified platform support
- Abstract the actuator interface (PHC adjfine vs I2C DCO)
- Support both TimeHAT (i226 only) and OTC (ClockMatrix + i226)
- The i226 PHC can still be disciplined via the ClockMatrix 25MHz output

## Compatibility with existing PHC work

The i226's 25 MHz PHY clock on the OTC is driven by the ClockMatrix.
When we steer the ClockMatrix DCO, the i226 PHC automatically tracks
because its timebase comes from the ClockMatrix output. So:

- **PHC PPS OUT** from the i226 reflects the ClockMatrix-disciplined clock
- **PTP timestamps** on the i226 are automatically disciplined
- **No need to run adjfine** on the i226 — the ClockMatrix does it upstream

This means peppar-fix on OTC would discipline the ClockMatrix, and
the i226 PHC follows for free. Existing PTP functionality is preserved.

## References

- [8A34002 Datasheet](https://www.mouser.com/datasheet/2/698/REN_8A34002_DST_20230813-1996740.pdf)
- [8A3xxxx Programming Guide](https://www.renesas.com/en/document/gde/8a3xxxx-family-programming-guide-v48)
- [ClockMatrix TDC Application Note AN-1010](https://www.renesas.com/en/document/apn/clockmatrix-time-digital-converter)
- [ClockMatrix DPLL Lock Time](https://www.renesas.com/en/document/apn/clockmatrix-dpll-lock-time)
- [ClockMatrix Oscillator Compensation](https://www.renesas.com/en/document/apn/clockmatrix-oscillator-compensation)
- [ClockMatrix Phase Noise Contributors](https://www.renesas.com/en/document/apn/clockmatrix-phase-noise-contributors)
- [ClockMatrix PHC Driver Compatibility](https://www.renesas.com/en/document/apn/clockmatrix-firmware-compatibility-linux-phc-driver)

## I2C Access Findings (2026-03-20)

### Bus discovery

The 8A34002 is NOT directly on `/dev/i2c-1`. Address `0x70` on bus 1
is a PCA9548 I2C mux owned by the kernel (shows as `UU`). The mux
creates virtual buses `/dev/i2c-13` through `/dev/i2c-22`.

**The 8A34002 is on `/dev/i2c-15` at address `0x58`.**

### Register access

The 8A34002 uses 1B addressing mode:
- Page register at offset `0xFC`: write a single byte to set the upper
  address byte
- Then read/write at the lower address byte offset

```python
import smbus2
bus = smbus2.SMBus(15)
addr = 0x58

# Set page to 0xC0 (status registers)
bus.write_byte_data(addr, 0xFC, 0xC0)
# Read DPLL0_FILTER_STATUS at offset 0x24
data = bus.read_i2c_block_data(addr, 0x24, 12)
```

### Confirmed register reads

| Register | Address | Sample Data | Notes |
|---|---|---|---|
| GENERAL_STATUS | 0xC014 | `06 00 55 00 00 13...` | Chip alive, DPLLs configured |
| DPLL0_FILTER_STATUS | 0xC024 | `4B 4F EA E0 7B 01...` | Filter active with real values |
| DPLL0_PHASE_STATUS | 0xC058 | `F0 C9 FA 9D F8 07...` | Phase measurement present |

### Permissions

Bob must be in the `i2c` group (added 2026-03-20). Use `sg i2c 'command'`
for the first session after group add, or re-login.

### Next steps

1. Map the DPLL0_PHASE_STATUS register fields to extract phase offset in ps
2. Find the TDC measurement trigger and result registers
3. Correlate TDC readings with TICC measurements
4. Timebeat must be stopped before I2C access (`sudo systemctl stop timebeat`)

### I2C hazard: page register corruption

**WARNING**: Writing to the 8A34002 page register (0xFC) while Timebeat
is stopped can leave the chip in a state that causes Timebeat to crash
on restart. The `DPLLStatusMonitor.Update` function panics with an
out-of-bounds index error, presumably because it reads a status register
from the wrong page.

Workaround: If Timebeat crash-loops after I2C probing:
1. Try resetting page register: `bus.write_byte_data(0x58, 0xFC, 0x00)`
2. If that doesn't help, the chip may need a power cycle (not just reboot)
3. As a last resort, contact Timebeat support

**The safe approach for Phase 1**: Stop Timebeat, configure the DPLL for
phase measurement mode ourselves, read TDC, then restore the original
DPLL configuration before restarting Timebeat. Or: coordinate with
Timebeat to read phase data from their API rather than directly via I2C.

### ptBoat findings (2026-03-20)

- 8A34002 is on **bus 16** (not 15 like otcBob1 — different mux channel)
- Timebeat configures DPLL0 in phase measurement mode (pll_mode=5)
- Phase detector: ref=CLK14, fb=CLK10 (PPS vs OCXO-derived)
- FBD integer=34, fbd_int_mode_en=1 → ITDC_UI ≈ 919 ps (not 50ps default)
- DPLL halts when Timebeat stops — cannot read live phase without Timebeat running
- Concurrent I2C reads with Timebeat return zeros/garbage ~80% of the time
- Timebeat survived our probing (page register restore worked)

### Revised approach for Phase 1

The DPLL depends on Timebeat's software loop to stay active. Options:

1. **Parse Timebeat's logs/API** for phase data (if it exposes TDC readings)
2. **Implement our own clock tree initialization** — configure inputs, DPLL,
   TDC from scratch without Timebeat. This is essentially Phase 2/3.
3. **Use Timebeat's HTTP API** (if it has one) to query phase data

The simplest path: ask Timebeat (the company) if they expose the
ClockMatrix phase error via API. Their binary has Go functions like
`ShowOpentimecardClockgenDpllStatus` that may be accessible via CLI
or HTTP endpoint.

### Clock tree dump breakthrough (2026-03-20)

**Timebeat uses pll_mode=3 (write_phase_set), NOT pll_mode=0 (PLL).**

The hardware DPLL loop is completely open. Timebeat is the loop:
1. Reads TDC phase error (CLK14=PPS vs CLK12=OCXO)
2. Runs software loop filter (freq_rho strategy)
3. Writes phase corrections to DPLL_CTRL_0.PHASE_OFFSET

For peppar-fix to replace Timebeat, we replay the captured clock tree
config (data/clocktree_ptboat.json), then:
- Read TDC phase from STATUS.DPLL0_PHASE_STATUS (coarse, 340ps)
  or STATUS.DPLL0_FILTER_STATUS (fine, 340/128 = 2.7ps)
- Run our PI servo
- Write corrections to DPLL_CTRL_0.PHASE_OFFSET

Key register values from ptBoat (Timebeat running):
- DPLL_0.MODE: 0x93 0xB0, pll_mode=3 (write_phase_set)
- DPLL_0.PHASE_MEAS_CFG: ref=CLK14 fb=CLK12
- INPUT_TDC.FBD_CTRL: fbd_int_mode_en=1, fbd_integer=92
- ITDC_UI = 1/(32 × 92MHz) ≈ 340 ps
- Fine resolution: 340/128 ≈ 2.7 ps

Full config captured in data/clocktree_ptboat.json.

### Bus numbers

| Host | I2C Bus | Address | Notes |
|---|---|---|---|
| otcBob1 | 15 | 0x58 | OTC SBC, OCXO |
| ptBoat | 16 | 0x58 | OTC Mini PT, weatherproof |
