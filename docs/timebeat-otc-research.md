# Timebeat OTC: Renesas 8A34002 TDC Phase Measurement

## Hardware

Timebeat OTC boards (otcBob1, ptBoat) include a **Renesas 8A34002 ClockMatrix**
connected to the Raspberry Pi via I2C bus 1, address **0x58**.

Address 0x70 on the same bus is an I2C mux (kernel-owned, shows `UU` in
`i2cdetect`).  Do not access it directly.

## TDC Overview

The Time-to-Digital Converter measures the phase offset between two clock
edges with sub-50 ps single-shot resolution (per AN-1010).  On OTC hardware
the relevant measurement is:

- **Reference input**: F9T PPS (GNSS-derived 1PPS)
- **Feedback clock**: OCXO output via the 8A34002 DPLL0 feedback divider

The TDC captures the time difference each PPS edge and stores it in a
5-byte signed register (DPLL0_PHASE_STATUS, address 0xD294–0xD298).

## Register Details

The 8A34002 uses paged 16-bit register addressing over I2C:

1. Write the high byte of the target address to the PAGE register (0xFD)
2. Read/write at the low-byte offset

### Key registers (DPLL0)

| Address   | Name                 | Size  | Description                      |
|-----------|----------------------|-------|----------------------------------|
| 0x0002–03 | DEVICE_ID            | 2     | 0x3400 for 8A34002 (little-endian) |
| 0xD280    | DPLL0_LOCK_STATUS    | 1     | Bit 0: DPLL locked               |
| 0xD294–98 | DPLL0_PHASE_STATUS   | 5     | TDC phase error, fixed-point ns, LE |

### Phase status encoding

The 5-byte (40-bit) value is signed fixed-point nanoseconds, little-endian:

    [31:0]  fractional nanoseconds (unsigned)
    [39:32] integer nanoseconds (signed byte, ±127 ns range)

    LSB = 1/2^32 ns ≈ 0.233 ps ≈ 0.233 fs

The effective single-shot resolution is limited by TDC analog noise to
approximately 20–50 ps, not the digital LSB.

## I2C Bus Contention

The Timebeat service (`timebeat.service`) manages the 8A34002 in normal
operation.  **Stop it before running tdc_reader.py** to avoid bus contention:

    sudo systemctl stop timebeat

## Validation Plan

Compare TDC phase readings against TICC measurements on the same PPS signals:

1. TICC chA: OCXO PPS (from 8A34002 output divider)
2. TICC chB: F9T PPS (raw GNSS)
3. TDC: internal phase measurement of the same pair

Agreement within the TICC noise floor (~60 ps single-shot) validates the
TDC readout.  The TDC should show lower noise than the TICC.

## References

- Renesas 8A3xxxx Programming Guide v4.8: register map, I2C protocol
- ClockMatrix TDC Application Note AN-1010: measurement setup, precision specs
- `scripts/tdc_reader.py`: readout implementation
