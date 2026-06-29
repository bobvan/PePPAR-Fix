# Gear Inventory

## Hosts

### MadHat (was PiPuss) — Data collection and GNSS observation

| Field | Value |
|-------|-------|
| Hostname | MadHat (renamed from PiPuss 2026-04-13) |
| Hardware | Raspberry Pi 5 (BCM2712, aarch64) |
| OS | Debian Bookworm, kernel 6.12.62+rpt-rpi-2712 |
| IP | 10.168.60.242 (DHCP) |
| Access | `ssh MadHat.local` |
| Serial ports | /dev/ttyACM0 |
| Symlinks | TBD (old gnss-top/gnss-bot symlinks may be stale after rewire) |
| Services | chronyd |
| Python | `~/peppar-fix/venv` (pyubx2, numpy, scipy) |
| Role | PePPAR Fix AR observation, testAnt data collection |
| Docs | — |

### Onocoy — **MOTHBALLED 2026-04-08**

Powered down and disconnected from peripherals.  TICC #2 was moved to
host `ocxo` for the i226 bring-up.  F10T and PX1125T are physically
disconnected and stored.  Confirmed never had a peppar-fix checkout.
Don't try to ssh — host may not be reachable, and the entries below
are a record of what *was* connected, not what *is*.

| Field | Value |
|-------|-------|
| Hostname | ~~Onocoy~~ (mothballed) |
| Hardware | Raspberry Pi 4 (BCM2711, aarch64) |
| OS | Debian, kernel 6.12.47+rpt-rpi-v8 |
| IP | 10.168.60.143 (DHCP) — no longer assigned |
| Access | (mothballed) |
| Serial ports | (none — host powered down) |
| Services | (none) |
| HW timestamping | **No** (BCM54210PE — single PPS pin, disqualified) |
| Role | (was: Onocoy GNSS mining, serial console to FS switch) |
| Docs | — |

### otcBob1 — Timebeat Open Time Card (SBC)

| Field | Value |
|-------|-------|
| Hostname | otcBob1 |
| Hardware | Timebeat OTC SBC (Raspberry Pi CM5 carrier, OCXO) |
| OS | Debian Bookworm, kernel 6.12.47+rpt-rpi-2712 |
| IP | 10.168.13.16/24 (PTP LAN) |
| Access | `ssh otcBob1` |
| NIC | Intel i226-LM (igc driver), HW timestamping |
| PHC | /dev/ptp0 (i226) |
| GNSS | u-blox ZED-F9T on /dev/ttyAMA0 at 460800 baud |
| Antenna | UFO (SparkFun SPK6618H), West roof slope |
| PPS | /dev/pps0 (from F9T via SDP0) |
| Software | Timebeat 2.3.5-enterprise (license valid through 2030) |
| Services | timebeat (enabled, running) |
| PTP | GM on domain 40 (serve_multicast, server_only); PPS primary |
| PTP secondaries | Domains 0 (M600), 10 (ptBoat, monitor), 20 (bbb, monitor), 30 (TimeHat, monitor) |
| NTP secondaries | ntp[1-5].VanValzah.Com |
| Role | PTP GM on domain 40, OCXO stability reference |
| Notes | Renesas 8A34002 ClockMatrix drives i226 25 MHz PHY clock via I2C; DPLL state: locked |
| Docs | — |

### ptBoat — Timebeat Open Time Card Mini PT in Weatherproof Enclosure

| Field | Value |
|-------|-------|
| Hostname | ptBoat |
| Hardware | Raspberry Pi CM5 on Timebeat OTC Mini PT carrier |
| OS | Debian Bookworm, kernel 6.12.47+rpt-rpi-2712 |
| IP | 10.168.13.15/24 (PTP LAN) |
| Access | `ssh ptBoat`, sudo available |
| NIC | Intel i225/i226 (igc driver), HW timestamping, all-filter RX |
| PHC | /dev/ptp0, SDP0 = PPS IN from F9T |
| GNSS | u-blox ZED-F9T on /dev/ttyAMA0 at 115200 baud |
| Antenna | SparkFun SPK6618H (dedicated, 1 ft feedline) |
| Mount | West roof slope, vertical, weatherproof enclosure, PoE powered |
| Software | Timebeat 2.3.5-enterprise (license valid through 2030) |
| PPS | /dev/pps0 (from F9T via SDP0) |
| Config | PPS primary, PTP GM domain 10 (serve_multicast, server_only) |
| PTP secondaries | Domains 0 (M600), 20 (bbb, monitor), 30 (TimeHat, monitor), 40 (otcBob1, monitor) |
| NTP secondaries | ntp[1-5].VanValzah.Com |
| Role | GPS-disciplined PTP GM — reference for commercial Timebeat product |
| Notes | Short feedline (1 ft) minimizes cable delay uncertainty; weatherproof tradeoff is thermal stress on electronics |
| Docs | — |

### TimeHat — SatPulse GPSDO + PePPAR Fix development

| Field | Value |
|-------|-------|
| Hostname | TimeHat (trusted LAN), TimeHat.PTP (PTP LAN) |
| Hardware | Raspberry Pi 5 |
| OS | Debian Trixie |
| Access | `ssh TimeHat` (passwordless), sudo without password |
| NIC | TimeHAT v5 (Intel i226-LM, TCXO), eth1 on PTP LAN |
| PHC | /dev/ptp0 (i226, 1 ns resolution, HW timestamping) |
| SDP pins | SDP0 (PHC PPS OUT, SMA1 J4 → TICC #3 chA), SDP1 (PPS IN from F9T-3RD) |
| GNSS | u-blox ZED-F9T (EVK-F9T-10-00, F9T-TOP) on /dev/gnss-top at 460800 baud |
| Antenna | UFO (SparkFun SPK6618H) via GUS #1 |
| Software | SatPulse (Go daemon), satpulsetool |
| Services | satpulse (systemd) |
| TICC | TAPR TICC #3, /dev/ticc (udev symlink), 115200 baud |
| TICC channels | chA = PHC PPS OUT (SDP0, disciplined), chB = F9T-3RD PPS (raw GPS) |
| TICC reference | 10 MHz from SV1AFN dist amp |
| Role | SatPulse GPSDO evaluation, PePPAR Fix real-time development |
| Docs | — |

### bbb — BeagleBone Black PTP GM

| Field | Value |
|-------|-------|
| Hostname | bbb |
| Hardware | BeagleBone Black (AM335x Cortex-A8, 480 MB RAM) |
| OS | Debian Bullseye, kernel 6.1.83-ti-r40 |
| IP | 10.168.13.14/24 (PTP LAN) |
| Access | `ssh bbb` (passwordless), sudo available |
| NIC | CPSW (AM335x MAC), 100 Mbps, HW timestamping (ptpv2-event) |
| PHC | /dev/ptp0 (CPSW CPTS) |
| GNSS | Adafruit Ultimate GPS (MTK-3301), GPS L1 only, on /dev/gps0 at 9600 baud |
| Antenna | Patch1 via SV1AFN L1 splitter |
| PPS | /dev/pps0 (from GPS), /dev/pps1 (unused) |
| Services | gpsd, chronyd (stratum 1, PPS-disciplined) |
| PTP | linuxptp installed, ptp4l configured but not running |
| Role | GPS-disciplined PTP GM (chrony + ptp4l) |
| Docs | — |

### pi4ptpmon — PTP GM monitoring host

| Field | Value |
|-------|-------|
| Hostname | pi4ptpmon |
| Hardware | Raspberry Pi CM4 Lite (1 GB RAM) on IO board |
| OS | Debian Trixie (upgrading) |
| IP | 10.168.13.13/24 (PTP LAN, static) |
| Access | `ssh bob@pi4ptpmon` (passwordless), sudo without password |
| NIC (onboard) | BCM54210PE (eth0) — DHCP, not used for PTP (~µs jitter) |
| NIC (PCIe) | Intel i210 (eth1) — igb driver, PHC /dev/ptp1, HW TX+RX timestamping, HWTSTAMP_FILTER_ALL |
| PTP LAN | eth1: 10.168.13.13/24 (static) |
| Role | ptpgm listener — monitor and compare PTP GMs |
| Notes | All 5 PTP GMs verified visible via i210 (2026-03-13) |
| Docs | — |

### M600 — Meinberg PTP Grandmaster

| Field | Value |
|-------|-------|
| Hostname | M600 |
| Hardware | Meinberg microSync M600 |
| IP | 10.168.13.5 |
| PTP identity | ec4670.fffe.0024cb |
| PTP domain | 0 |
| Clock class | 6 (GPS-locked) |
| Clock accuracy | 0x21 (100 ns) |
| Priority1/2 | 0/0 |
| Time source | 0x20 (GPS) |
| Sync interval | 1 Hz (logSyncInterval=0) |
| Delay mechanism | E2E |
| Role | Reference PTP GM on lab network |

### NTP servers

| Hostname | IP | Hardware | Role |
|----------|-----|----------|------|
| ntp1.VanValzah.Com | 10.168.60.5 | Meinberg M600 | Stratum 1 NTP (GPS, L1 antenna Patch1 via SV1AFN splitter) |
| ntp2.VanValzah.Com | 10.168.60.20 | Proxmox PVE bare metal, TCXO | Stratum 2+, syncs from ntp1 + Internet NTS |
| ntp3.VanValzah.Com | 10.168.60.28 | Proxmox PVE bare metal, TCXO | Stratum 2+, syncs from ntp1 + Internet NTS |
| ntp4.VanValzah.Com | 10.168.60.30 | Proxmox PBS bare metal, TCXO | Stratum 2+, syncs from ntp1 + Internet NTS |
| ntp5.VanValzah.Com | 10.168.60.25 | Proxmox PVE bare metal, TCXO | Stratum 2+, syncs from ntp1 + Internet NTS |

ntp[2-5] peer with each other for local resilience. Internet NTS sources have
applied offsets to compensate for asymmetric Internet path latency, calibrated
against ntp1. NTS is used for MITM protection on Internet sources.

### XXX Notes to Bob: understand offline report on otcBob1

## GNSS Receivers

### F9T-3RD

| Field | Value |
|-------|-------|
| EVK model | EVK-F9T-20-00 |
| Module | u-blox ZED-F9T-20B |
| Firmware | TIM 2.25 (PROTVER 29.25) |
| SEC-UNIQID | 394029318459 |
| Host | clkPoC3 (moved from TimeHat 2026-04-13) |
| Port | /dev/ttyACM1 |
| Baud | 460800 |
| Signals | GPS L1+L5, Galileo E1+E5a, BeiDou B1+B2a |
| Limitation | **No GLONASS** (-20B hardware variant) |
| Antenna | Shared with F9T-TOP and F9T-BOT via splitter (same ARP) |
| Notes | Moved to clkPoC3 2026-04-13. Was on TimeHat 2026-03-16 to 2026-04-13. |

### F9T-TOP

| Field | Value |
|-------|-------|
| EVK model | EVK-F9T-10-00 |
| Module | u-blox ZED-F9T (no -20B suffix in MON-VER) |
| Firmware | TIM 2.20 (PROTVER 29.20) |
| SEC-UNIQID | 136395244089 |
| Host | TimeHat (moved from MadHat/PiPuss 2026-04-13) |
| Port | /dev/gnss-top |
| Baud | 460800 |
| Signals | Currently L1+L5 (auto-configured by ensure_receiver_ready); hardware natively L1/L2 |
| Antenna | Shared with F9T-BOT and F9T-3RD via splitter (same ARP) |
| Notes | EVK-F9T-10-00 is nominally L1/L2 hardware but ZED-F9T silicon supports L5. ensure_receiver_ready() auto-switches to L5 (tries L5 first). CFG-VALGET confirms L2C_ENA=0, L5_ENA=1. To test L2 AR, force `--receiver f9t`. |

### F9T-BOT

| Field | Value |
|-------|-------|
| EVK model | EVK-F9T-20-00 |
| Module | u-blox ZED-F9T-20B |
| Firmware | TIM 2.25 (PROTVER 29.25) |
| SEC-UNIQID | 262843023907 |
| Host | MadHat (was PiPuss, renamed 2026-04-13) |
| Port | /dev/ttyACM0 |
| Baud | 460800 |
| Signals | GPS L1+L5, Galileo E1+E5a, BeiDou B1+B2a |
| Limitation | **No GLONASS** (-20B hardware variant) |
| Antenna | Shared with F9T-TOP and F9T-3RD via splitter (same ARP) |
| Notes | Physically on bottom of the stacked EVK pair |

**USB serial note:** Both F9T EVKs present the same USB VID:PID and serial
descriptor under `/dev/serial/by-id/`, so udev cannot reliably distinguish them.
When both are on the same host, symlink assignment depends on USB enumeration
order. The udev rules on PiPuss (`/dev/gnss-top`, `/dev/gnss-bot`) assume a
specific enumeration order that may not hold after re-plugging.

### Adafruit Ultimate GPS (bbb)

| Field | Value |
|-------|-------|
| Model | Adafruit Ultimate GPS Breakout (MTK-3301, AXN 2.31) |
| Host | bbb |
| Port | /dev/gps0 at 9600 baud |
| Signals | GPS L1 only |
| Antenna | Patch1 via SV1AFN L1 splitter |
| PPS | /dev/pps0 |
| Role | Time-of-day for chrony NTP + PTP GM |
| Notes | Low-cost consumer GPS, adequate for PPS-disciplined NTP/PTP but not for carrier-phase work |

### PX1125T (mothballed 2026-04-08, was Onocoy)

| Field | Value |
|-------|-------|
| Model | SkyTraq PX1125T (timing firmware) |
| Host | Onocoy |
| Port | /dev/ttyUSB1 (CP2102) |
| Antenna | UFO (SPK6618H) via GUS #1 |
| PPS | Connected to TICC #2 (on Onocoy) |
| Role | PPS timing evaluation (px1125t_eval project) |
| Notes | qErr from $PSTI,00 correlates with velocity, not phase — firmware bug. Raw PPS is good: TDEV(1s)=3.42 ns |

### ZED-F10T (mothballed 2026-04-08, was Onocoy)

| Field | Value |
|-------|-------|
| Model | u-blox ZED-F10T |
| Host | Onocoy |
| Port | /dev/ttyACM0 |
| Antenna | Patch3 via TAPR GUS active splitter |
| Role | Onocoy GNSS mining stream |

## Timing Instruments

### TAPR TICC #1 (PiPuss)

| Field | Value |
|-------|-------|
| Model | TAPR TICC |
| Resolution | 60 ps |
| Host | PiPuss |
| Port | /dev/ticc (udev symlink) |
| Baud | 115200 |
| Channels | chA = TimeHAT PHC PPS OUT (SDP0), chB = F9T-BOT PPS |
| Reference | 10 MHz from SV1AFN distribution amp (fed by FE-5680A Rb standard) |
| Role | SatPulse / PePPAR Fix TDEV measurement |
| Docs | [TAPR TICC](http://www.tapr.org/ticc.html) |

### TAPR TICC #2 (host: ocxo as of 2026-04-08; was Onocoy)

| Field | Value |
|-------|-------|
| Model | TAPR TICC |
| Resolution | 60 ps |
| Host | ocxo (moved 2026-04-08; was Onocoy) |
| Port | /dev/ticc2 (USB CDC ACM, Arduino serial 44236313835351B02001) |
| Baud | 115200 |
| Channels | chA = i226 PEROUT (SDP0), chB = F9T-TOP PPS — matches TimeHat / MadHat layout |
| Reference | 10 MHz (when fed) |
| Role | i226 add-in card PPS measurement on host ocxo |
| Docs | [TAPR TICC](http://www.tapr.org/ticc.html) |

### TAPR TICC #3 (TimeHat)

| Field | Value |
|-------|-------|
| Model | TAPR TICC |
| Resolution | 60 ps |
| Host | TimeHat |
| Port | /dev/ticc (udev symlink, matched by Arduino serial number) |
| Baud | 115200 |
| Channels | chA = TimeHAT PHC PPS OUT (SDP0, SMA1 J4, disciplined), chB = F9T-3RD PPS (EVK SMA, raw GPS) |
| Reference | 10 MHz from SV1AFN distribution amp (fed by FE-5680A Rb standard) |
| Role | PePPAR Fix M5 TDEV measurement (disciplined PHC vs raw GPS PPS) |
| Docs | [TAPR TICC](http://www.tapr.org/ticc.html) |

**Reference clock note:** When using TICC for cross-channel time transfer
(comparing chA vs chB), the shared reference cancels — both channels share the
same timebase, so the measurement reflects only the PPS difference. For
single-channel absolute measurements, the reference's noise floor contributes
directly to the result and must be accounted for. The reference is now the
FE-5680A Rb standard (was a Geppetto OCXO GPSDO until 2026-05-07), so that
single-channel floor is now Rb-class (sub-ns), much lower than the old OCXO floor.

## Reference Oscillators & Distribution

### Frequency Electronics FE-5680A Rb standard — TICC reference (CURRENT)

| Field | Value |
|-------|-------|
| Model | Frequency Electronics FE-5680A rubidium frequency standard |
| Oscillator | Rb |
| Output | 10 MHz — **not GPS-disciplined**, so not exactly 10.0000000 MHz (small fixed frequency offset) |
| Role | **Lab TICC reference clock** — feeds the SV1AFN distribution amplifier, which buffers/fans it out to every TICC reference input |
| Notes | Replaced the Geppetto GPSDO (below) as the lab reference. Free-running/undisciplined, so its absolute frequency is slightly off 10 MHz, but its **short-term TDEV is far better than an OCXO** — which is what sets the TICC measurement floor. A constant frequency offset detrends out of any chA stability metric. Commissioned 2026-05-10 (one of the two shelved Rb units); FE-5680A model confirmed 2026-06-29. |

### Geppetto Electronics GPSDO — OUT OF SERVICE (storm 2026-05-07)

| Field | Value |
|-------|-------|
| Model | Geppetto Electronics GPSDO |
| Oscillator | OCXO |
| Output | 10 MHz |
| Role | FORMER lab reference clock — fed the SV1AFN distribution amp until knocked out by thunderstorms 2026-05-07; replaced by the FE-5680A Rb (above) on 2026-05-10 |
| Notes | Decent OCXO for a home timelab. Not currently in the reference chain |

### SV1AFN 10 MHz Distribution Amplifier

| Field | Value |
|-------|-------|
| Model | SV1AFN.Com 10 MHz distribution amplifier |
| Input | 10 MHz from FE-5680A Rb standard (was Geppetto GPSDO until 2026-05-07) |
| Outputs | Multiple 10 MHz (buffers/fans out to TICC reference inputs) |
| Role | Fan-out lab reference clock to multiple instruments |
| Docs | [SV1AFN.Com](https://www.sv1afn.com/) |

### Rubidium Oscillators (×2)

| Field | Value |
|-------|-------|
| Quantity | 2 (one is the FE-5680A now serving as the TICC reference — see above) |
| Status | One **commissioned 2026-05-10** as the lab TICC reference (FE-5680A → SV1AFN); the other shelved/spare |
| Expected ADEV | ~1e-11 at τ=1s (100× better than F9T TCXO) |
| Potential roles | TICC reference clock (DONE — replaced the Geppetto OCXO), independent stability reference, holdover backbone, second Rb for cross-check |
| Notes | The active Rb is undisciplined (slightly off 10 MHz) but its short-term stability sets the new, much lower lab TDEV measurement floor |

## SDR Equipment

### Ettus Research USRPs

| Field | Value |
|-------|-------|
| Type | Software Defined Radio (multiple units) |
| Status | Available, not currently used for GNSS |
| Potential role | SDR GNSS receiver (e.g. via [GNSS-SDR](https://gnss-sdr.org/)) |
| Notes | Could accept external Rb reference. However, F9T carrier phase (~2mm) already matches SDR capability — the bottleneck is orbit/clock correction quality (~1.3m from SP3+CLK), not receiver noise. SDR adds raw correlation access and custom PLL bandwidth, but marginal benefit for timing. |

## RF Distribution

### TAPR GUS Active Splitters (×2)

**GUS #1 and #2** — wiring changed 2026-04-13. All three F9Ts
(F9T-TOP on TimeHat, F9T-BOT on MadHat, F9T-3RD on clkPoC3) now
share one antenna via a splitter (same ARP). Antenna and splitter
identity needs verification — update this section once confirmed.

| Field | Value |
|-------|-------|
| Model | TAPR GUS Active GPS Antenna Splitter (×2) |
| Bandwidth | Covers all GNSS bands (L1/L5/E1/E5a/B1/B2a verified) |
| Docs | [TAPR GUS Manual](https://web.tapr.org/~n8ur/GUS_Manual.pdf) |

### SV1AFN L1 GPS Antenna Splitter

| Field | Value |
|-------|-------|
| Model | SV1AFN GPS Antenna Splitter (GPS/GLONASS L1, BeiDou B1, Galileo E1) |
| Input | Patch1 antenna (coax from roof) |
| Outputs | Adafruit Ultimate GPS (bbb), (other port available) |
| Bandwidth | L1 band only (1575.42 MHz ± ~20 MHz) |
| Docs | [SV1AFN Shop](https://sv1afn.com/shop/rf-splitter/gps-antenna-splitter-gps-glonass-l1-beidou-b1-galileo-e1/) |

## Network

### FS IES3110-8TFP-R

| Field | Value |
|-------|-------|
| Model | FS.COM IES3110-8TFP-R |
| MAC | 64:9D:99:AA:5D:A0 |
| IP | 10.168.13.90/24 (static) |
| Firmware | FSOS V1.5, Software V1.1 |
| PoE | 250 W budget, class-consumption mode |
| PTP | p2ptransparent, onestep, ip4mixed, twoway |
| Console | Serial via /dev/ttyUSB2 on Onocoy, 115200 8N1, admin/admin |
| Web | https://10.168.13.90 |
| Role | PTP broadcast domain switch (10.168.13.0/24) |
| Docs | [FS IES3110-8TFP-R](https://www.fs.com/products/145113.html) |

## Antennas

### Mount sites

**West roof slope** — vertical axis eyeball-vertical, non-penetrating mounts.
UFO and ptBoat's SPK6618H are both here.

**East roof slope** — 5:12 pitch. Patch1, Patch2, and Patch3 are mounted with
vertical axes tilted by the roof slope (~22.6° from true vertical). Expected
slight disadvantage on SVs near the west horizon due to tilt.

**Indoors (desk)** — Choke ring antenna only. Not representative of outdoor
performance. Requires a stiffer non-penetrating mount for outdoor deployment
due to larger windload from the radome; mount construction starting 2026-03-13.

### Antenna inventory

| Name | Type / Model | Mount site | Axis | Mean C/N0 | Avg SVs | Notes |
|------|-------------|------------|------|-----------|---------|-------|
| Patch3 | Patch (unknown mfr) | East roof slope | Tilted (5:12) | **39.42 dBHz** | 60.7 | Best performer, full BDS-B2I, wide band |
| Patch2 | Patch (Quectel) | East roof slope | Tilted (5:12) | 38.01 dBHz | 44.0 | Consistent across runs (~38 dBHz) |
| UFO | SparkFun SPK6618H | West roof slope | Vertical | ~37 dBHz | — | 24h run with choke ring; same model as ptBoat antenna |
| Choke | Choke ring with radome | Indoors (desk) | Vertical | ~37 dBHz | — | Indoor test only, not representative; outdoor mount under construction |
| SPK6618H (ptBoat) | SparkFun SPK6618H | West roof slope | Vertical | — | — | Dedicated to ptBoat, 1 ft feedline |
| Patch1 | Patch (unknown mfr) | East roof slope | Tilted (5:12) | 31.55 dBHz | 47.1 | **Disqualified** — dead on L5/E5a/E1B (~10 dBHz) |

## Hardware on order / planned

| Item | Purpose | Price | Status |
|------|---------|-------|--------|
| TimeHAT v5 + RPi 5 | Second PPS-capable setup for PePPAR Fix (matches TimeHat's v5) | ~$140 + Pi 5 | Ordered 2026-03-15 |
| Intel E810-XXVDA4T | Precision PTP timestamping (sub-ns, OCXO) | ~$1,100 | Not ordered |

### Solarflare inventory (on hand, not deployed)

| Card | Qty | PPS I/O | PTP License | Notes |
|------|-----|---------|-------------|-------|
| SFN6322F | 2 | u.FL on PCB (PPS bracket) | Standard (included) | HW tstamp PTP pkts only, single port. Stratum 3 osc. |
| SFN8522 | 4 | u.FL on PCB | **Unknown — needs AppFlex key** | All-pkt tstamp if licensed. Test with `ethtool -T`. |

## Subnets

| Network | Purpose |
|---------|---------|
| 10.168.60.0/24 | Trusted LAN (hosts, SSH, data transfer) |
| 10.168.13.0/24 | PTP broadcast domain (GMs, switch, listener) |
