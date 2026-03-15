# NIC and Timing Hardware Survey for PHC Discipline

All devices must have **both PPS input and PPS output** from a disciplined PHC.
Devices without both are disqualified.

## Qualified Hardware

### Consumer / Hobbyist Tier

| Device | PHC Res | adjfine (ppb/LSB) | max_adj | PPS I/O | Oscillator | Interface | Price | Notes |
|---|---|---|---|---|---|---|---|---|
| Intel i210 | 1 ns | ~0.12 | ±62.5 ppm | 4 SDP (pin header) | 25 MHz XO | PCIe 2.1 x1, 1G | ~$50 | Best community docs, proven |
| Intel i225 | 1 ns | ~0.12 | ±62.5 ppm | 4 SDP | 25 MHz XO | PCIe 3.1 x1, 2.5G | ~$50 | Adds PTM; early rev bugs |
| Intel i226 | 1 ns | ~0.12 | ±62.5 ppm | 4 SDP | 25 MHz XO | PCIe 3.1 x1, 2.5G | ~$55 | Current best consumer; SDP3 is strapping pin |
| TimeNIC | 1 ns | ~0.12 | ±62.5 ppm | 2 SMA | TCXO ±280 ppb | PCIe 3.1 x1, 2.5G | $200 | i226 + TCXO + SMA, turnkey |
| TimeHAT | 1 ns | ~0.12 | ±62.5 ppm | 2 SMA + 2 U.FL | TCXO ±280 ppb | Pi 5 HAT, 2.5G | $200 | i226 + TCXO, Pi 5 only |

### Enterprise Tier

| Device | PHC Res | adjfine | max_adj | PPS I/O | Oscillator | Interface | Price | Notes |
|---|---|---|---|---|---|---|---|---|
| **Intel E810-XXVDA4T** | **Sub-ns (7-bit)** | Very fine | ±1000 ppm | **2 SMA** + U.FL | **OCXO** (4h holdover) | PCIe 4.0 x16, 4×25G | ~$1,100 | **Best-in-class: 6 ns RMS measured** |
| OCP Time Card | FPGA-dep | FPGA | FPGA | 4 SMA | Rb/OCXO (modular) | PCIe full | $3,200-10k | Reference grandmaster, not a NIC |
| Timecard Mini 2.0 | FPGA-dep | FPGA | FPGA | SMA | TCXO-OCXO | CM4/CM5 | $290-1,500 | Compact grandmaster |
| Oregano syn1588 | Sub-ns (2⁻⁴⁵) | FPGA | FPGA | 2 SMA + 2 int | TCXO/OCXO opt | PCIe 2.0 x1, 1G | $2k-5k+ | Enterprise FPGA NIC (Meinberg) |
| Meinberg GNS183PEX | 5 ns | Proprietary | Proprietary | D-Sub | TCXO/OCXO + GNSS | PCIe LP | $2k-5k+ | Professional, GNSS built-in |
| Safran TSync | 5 ns | Proprietary | Proprietary | Multiple | TCXO/OCXO + GNSS | PCIe | $3k-8k+ | Military/telecom grade |

## Key Findings

### The E810-XXVDA4T stands out

The Intel E810-XXVDA4T ("T" = timing variant) is the only NIC with:
- **Sub-nanosecond timestamping** (7-bit sub-ns field in hardware)
- **Onboard OCXO** with 4-hour holdover (<±1.5 µs)
- **SMA connectors** on the bracket (no soldering, no breakout boards)
- **Measured 6 ns RMS** PTP sync accuracy (Scott Laird oscilloscope tests)
- Optional GNSS mezzanine module (u-blox)

At $1,100 it's 5× the cost of a TimeHAT but offers fundamentally better
timestamping resolution. For a PPP-AR project aiming at sub-nanosecond clock
estimation, the servo output precision shouldn't be limited by 1 ns
timestamping granularity.

**Caveat**: The mainline Linux `ice` driver does NOT support PPS I/O. Must use
Intel's out-of-tree driver compiled via DKMS.

### Intel i210/i225/i226 share the same timing core

All three use the same 31-bit INCVALUE register architecture:
- 1 ns SYSTIM resolution
- ~0.12 ppb per LSB adjfine granularity
- ±62.5 ppm max_adj range
- **Dual-edge quirk**: timestamps both rising AND falling PPS edges

The i210 actually measures better than the i226 for timing (76 ns vs 439 ns
mean offset) because the i226's 2.5G DSP adds latency. For pure PPS
timestamping (no packet timestamps), they should be equivalent.

### TCXO matters for holdover

The bare i226's commodity 25 MHz crystal drifts 20-50 ppm over temperature.
The TimeHAT/TimeNIC's TCXO (±280 ppb) is ~100× better. For a GPSDO that
needs to maintain accuracy during brief GNSS outages, the TCXO is essential.

## Disqualified

| Device | Reason |
|---|---|
| Broadcom BCM54210PE (CM4/CM5) | Single pin for PPS — can't do simultaneous IN + OUT |
| Marvell/Aquantia AQR | No PPS I/O pins |
| Microchip LAN743x | No PPS input (n_ext_ts=0) |
| Microchip LAN937x | No PPS I/O |
| Trimble Thunderbolt | Standalone GPSDO, not a NIC/PCIe card |
| SiTime eval boards | Oscillator test platforms, no PHC |
| Calnex Sentinel | Test equipment ($15k+), not a timing card |
| EndRun Technologies | Complete appliances, not PCIe cards |

## Recommendation for PePPAR Fix

**Development platform**: TimeHAT (i226 + TCXO, $200) on Pi 5. Same hardware
as SatPulse evaluation. Good enough for initial filter development — the 1 ns
timestamping exceeds what PPS+qErr can deliver anyway.

**Upgrade path**: Intel E810-XXVDA4T ($1,100). Once the PPP-AR filter is
producing sub-nanosecond clock estimates, the i226's 1 ns timestamping becomes
the bottleneck. The E810's sub-ns timestamping and onboard OCXO would let us
measure the filter's true performance.

**Order now**:
- [ ] TimeHAT v6 ($200) — for SatPulse eval + PePPAR Fix development
- [ ] Intel E810-XXVDA4T (~$1,100) — for precision measurements later

## Sources

- [Scott Laird — NIC timing features](https://scottstuff.net/posts/2025/05/20/time-nics/)
- [Scott Laird — Measuring NTP/PTP accuracy (Part 3: NICs)](https://scottstuff.net/posts/2025/06/07/measuring-ntp-accuracy-with-an-oscilloscope-3/)
- [jclark — PPS NIC guide](https://github.com/jclark/pc-ptp-ntp-guide/blob/main/pps-nic.md)
- [SatPulse — Intel build](https://satpulse.net/hardware/intel-build.html)
- [Linux igb_ptp.c](https://github.com/torvalds/linux/blob/master/drivers/net/ethernet/intel/igb/igb_ptp.c)
- [Linux igc_ptp.c](https://github.com/torvalds/linux/blob/master/drivers/net/ethernet/intel/igc/igc_ptp.c)
- [Intel E810-XXVDA4T User Guide](https://cdrdv2-public.intel.com/646265/646265_E810-XXVDA4T%20User%20Guide_Rev1.2.pdf)
- [TimeNIC (Tindie)](https://www.tindie.com/products/timeappliances/timenic-i226-pcie-nic-with-pps-inout-and-tcxo/)
- [TimeHAT (Tindie)](https://www.tindie.com/products/timeappliances/timehat-i226-nic-with-pps-inout-for-rpi5/)
- [OCP Time Card](https://github.com/Time-Appliances-Project/Time-Card)
- [Oregano syn1588](https://www.oreganosystems.at/products/syn1588/hardware/syn1588r-pcie-nic)
- [Timebeat DKMS ice driver guide](https://support.timebeat.app/hc/en-gb/articles/13199965947026)
