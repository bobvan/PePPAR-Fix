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

### Solarflare (AMD/Xilinx) — Bob has ×2 SFN6322F, ×4 SFN8522

| Device | PHC Res | adjfine | max_adj | PPS I/O | Oscillator | Interface | Price | Notes |
|---|---|---|---|---|---|---|---|---|
| **SFN6322F** ×2 | 1 ns | Unknown | Unknown | **u.FL on PCB** (PPS bracket) | Stratum 3 (<0.37 PPM/day) | PCIe 2.0/3.0, 2×10G SFP+ | ~$30 used | PTP standard (no license). HW tstamp: PTP pkts only, single port. **Bob has 2.** |
| **SFN8522** ×4 | 1 ns | Unknown | Unknown | **u.FL on PCB** | Stratum 3 (<0.37 PPM/day) | PCIe 3.1 x8, 2×10G SFP+ | ~$25 used | **Needs AppFlex PTP license** (PLUS variant has it pre-installed). HW tstamp: all packets. **Bob has 4 (license status unknown).** |
| SFN8522-PLUS | 1 ns | Unknown | Unknown | u.FL on PCB | Stratum 3 (<0.37 PPM/day) | PCIe 3.1 x8, 2×10G SFP+ | ~$50 used | PTP license pre-installed. Same HW as SFN8522. |

**Solarflare notes:**
- All use `sfc` driver + `sfptpd` daemon (AMD maintains both, open source)
- Stratum 3 oscillator: < 0.37 PPM/day, < 4.6 PPM over 20 years — better than
  bare i226 XO (~20-50 PPM/temp) but worse than TimeHAT TCXO (±280 ppb)
- PPS connectors are **u.FL on the PCB**, not SMA on the bracket. Need u.FL→SMA
  pigtails to connect to external PPS sources/sinks
- adjfine/adjfreq granularity not documented in public sources
- SFN6322F: HW timestamping limited to PTP packets, single port (closest to PCIe)
- SFN8522/PLUS: HW timestamps on ALL packets, both ports — significantly better

#### Testing for PTP license on SFN8522

The SFN8522 base model requires an AppFlex license key for PTP. To check:

```bash
# 1. Install Solarflare utilities (sfkey, sfupdate, sfboot)
#    Available from AMD: https://www.amd.com/en/support/downloads/drivers.html/solarflare.html

# 2. List installed license keys
sfkey list

# 3. Check hardware timestamping capability (quick test)
ethtool -T <interface>
# If PTP is licensed, you'll see:
#   SOF_TIMESTAMPING_TX_HARDWARE
#   SOF_TIMESTAMPING_RX_HARDWARE
#   SOF_TIMESTAMPING_RAW_HARDWARE
# If NOT licensed, only software timestamping will appear.

# 4. Check if PHC device appeared
ls /dev/ptp*
# A /dev/ptpN should appear for a licensed adapter

# 5. Verify driver version and firmware
ethtool -i <interface>
```

If no PTP license is present, the cards are still 10G NICs but cannot do
hardware timestamping or PPS — functionally equivalent to the base SFN8122F
for our purposes (disqualified). AppFlex license keys were sold by Solarflare;
with the company absorbed into AMD, obtaining new keys may be difficult.
Check if the keys are tied to the NIC's MAC or serial number.

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
| Solarflare SFN6122F / SFN7122F | Base 10G NICs — no PPS connectors. PTP variants (SFN6322F / SFN7322F) have PPS. SFN7000 bracket kit (SOLR-PPS-DP10G) unobtainable. |
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

### Dual-setup: PePPAR Fix + SatPulse simultaneously

Each setup needs: F9T receiver + NIC with PHC + PPS IN + PPS OUT.

**Current setup (TimeHat)**: TimeHAT v5 (i226) + F9T-TOP → runs SatPulse.
This is already deployed and working.

**Second setup needed for PePPAR Fix**: Needs another NIC with PHC + PPS I/O,
plus a host with PCIe or a Pi 5. Options from inventory first:

| Option | Cost | Pros | Cons |
|--------|------|------|------|
| **SFN6322F** (have ×2) | $0 + u.FL pigtails | Already owned, Stratum 3 osc, 10G | Needs PCIe host, sfptpd toolchain, HW tstamp PTP-only |
| **SFN8522** (have ×4) | $0 + u.FL pigtails | Already owned, Stratum 3, all-pkt tstamp | **PTP license may be missing** — must test with `ethtool -T` |
| TimeHAT v6 + Pi 5 | ~$200 + Pi 5 | Identical to SatPulse setup, proven | Cost |
| TimeNIC + any x86 | ~$200 | Same i226+TCXO, works with any PCIe host | Cost |
| Intel i210 + breakout | ~$50 + wiring | Cheapest, best timing docs | Pin header needs SMA breakout, no TCXO |

**Recommendation**: Test the SFN8522 cards first — if any have the PTP license,
that's a $0 solution with a better oscillator than a bare i226. The SFN6322F
pair is a guaranteed fallback (PTP is standard, no license needed). Either way
you'll need u.FL→SMA pigtails (~$5 each) and a PCIe host. If no PCIe host is
available, a second TimeHAT on a Pi 5 remains the cleanest path.

**Action items**:
- [ ] Plug an SFN8522 into any Linux box, run `ethtool -T <iface>` to check PTP license
- [ ] If licensed → u.FL pigtails + PCIe host = done
- [ ] If not licensed → use SFN6322F (PTP standard, guaranteed)
- [ ] Order u.FL→SMA pigtails (×2 per card: PPS IN + PPS OUT)
- [ ] Intel E810-XXVDA4T (~$1,100) — precision upgrade (later)

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
- [Solarflare Enhanced PTP User Guide (Issue 8)](https://www.amd.com/content/dam/amd/en/support/downloads/solarflare/drivers-software/linux/ptp/SF-109110-CD-8_Solarflare_Enhanced_PTP_User_Guide.pdf)
- [sfptpd — AMD Solarflare Enhanced PTP Daemon](https://github.com/Xilinx-CNS/sfptpd)
- [Solarflare SFN6322F product brief](https://www.bhphotovideo.com/c/product/1017856-REG/solarflare_sfn6322f_solarflare_srvr_adptr_crd.html)
- [Solarflare PPS I/O specification (Manualzz)](https://manualzz.com/doc/o/k605n/solarflare-enhanced-ptp-user-guide-solarflare-sfptpd-1pps-i-o-specification)
- [Solarflare Server Adapter User Guide (sfkey/sfboot/sfupdate)](https://www.amd.com/content/dam/amd/en/support/downloads/solarflare/drivers-software/SF-103837-CD-28_Solarflare_Server_Adapter_User_Guide.pdf)
- [SFN8522-PLUS product brief](https://www.xilinx.com/content/dam/amd/en/documents/products/ethernet-adapters/SFN8522-plus-Onload-product-brief.pdf)
