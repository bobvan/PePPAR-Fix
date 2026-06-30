# Lab Topology

> **Diagram is a 2026-03-16 snapshot.**  See the change log at the
> bottom for everything that has happened since.  Notably, **Onocoy
> was mothballed 2026-04-08** and **receivers were shuffled 2026-04-13**
> (F9T-TOP → TimeHat, F9T-3RD → clkPoC3, PiPuss renamed to MadHat,
> all three F9Ts on same antenna).  Wherever the diagram shows
> Onocoy, F10T, or PX1125T, treat as historical.

## Current connections

*Last updated: 2026-03-16 (host blocks); see change log for deltas*

```
  ┌────────────────────────────────┐  ┌──────────────────────────────┐
  │       West Roof Slope          │  │       East Roof Slope        │
  │       (vertical axis)          │  │       (tilted, 5:12 pitch)   │
  │                                │  │                              │
  │  [UFO]        [SPK6618H]      │  │  [Patch3]  [Patch2]  [Patch1]│
  │    │          (ptBoat, 1ft)    │  │     │         │         │    │
  └────┼──────────────┼───────────┘  └─────┼─────────┼─────────┼────┘
       │ coax         │ coax (1 ft)        │ coax    │ coax    │ coax
  ┌────┴─────────┐    │              ┌─────┴─────┐   │    ┌────┴─────────┐
  │   GUS #1     │    │              │  GUS #2   │   │    │ SV1AFN L1    │
  │  Splitter    │    │              │ Splitter  │   │    │ Splitter     │
  ├───┬────┬─────┤    │              ├─────┬─────┤   │    ├────┬─────────┤
  │o1 │ o2 │ o3  │    │              │out1 │out2 │   │    │out1│out2 (-)│
  └┬──┘─┬──┘──┬──┘    │              └──┬──┘──┬──┘   │    └──┬─┘────────┘
   │    │     │       │                 │     │      │       │
   │    │     │       │                 │     │      │       │
   ▼    ▼     ▼       ▼                 ▼     ▼      ▼       ▼
  ┌────────────────────────┐  ┌──────────────────┐ ┌─────────────────────┐
  │    otcBob1 (OTC SBC)   │  │  Onocoy (Pi 4)   │ │   TimeHat (Pi 5)    │
  │                        │  │                  │ │                     │
  │  F9T ← UFO (via GUS1) │  │  F10T ← Patch3   │ │ F9T-3RD ← UFO      │
  │  /dev/ttyAMA0, 460800  │  │    (via GUS2)    │ │   (via GUS1)        │
  │  PPS → SDP0            │  │  /dev/ttyACM0    │ │ /dev/gnss-top,115200│
  │  i226, OCXO, Renesas   │  │  onocoy-stream   │ │ PPS → SDP1+TICC3chB │
  │  Timebeat 2.3.5        │  │                  │ │ SDP0→PPS OUT→TICC3A │
  │  PTP GM domain 40      │  │  PX1125T ← UFO   │ │ TICC3 /dev/ticc     │
  │  eth0: 10.168.13.16    │  │    (via GUS1)    │ │ TimeHAT v5 (i226)   │
  └────────────────────────┘  │  /dev/ttyUSB1    │ │ SatPulse daemon      │
                              │                  │ │ eth1: PTP LAN       │
                              │                  │ └─────────────────────┘
                              │  /dev/ttyUSB2 →  │
                              │    FS switch     │
  ┌────────────────────────┐  │  eth0:           │
  │    ptBoat (OTC Mini PT)│  │  10.168.60.143   │ ┌─────────────────────┐
  │    weatherproof, PoE   │  └──────────────────┘ │  bbb (BeagleBone)   │
  │                        │                       │                     │
  │  F9T ← SPK6618H       │                       │  Adafruit GPS       │
  │  /dev/ttyAMA0, 115200  │                       │    ← Patch1 (L1     │
  │  PPS → SDP0            │                       │      via SV1AFN)    │
  │  i226                  │                       │  /dev/gps0, 9600    │
  │  Timebeat 2.3.5        │                       │  PPS → /dev/pps0    │
  │  PTP GM domain 10      │                       │  chrony stratum 1   │
  │  eth0: 10.168.13.15    │                       │  ptp4l domain 20    │
  └────────────────────────┘                       │  eth0: 10.168.13.14 │
                                                   └─────────────────────┘
  ┌──────────────────────────────────────────┐
  │           PiPuss (Pi 5)                  │
  │                                          │
  │  F9T-BOT ← Patch3 (via GUS2)            │
  │  /dev/gnss-bot, PPS → TICC1 chB         │
  │                                          │
  │  TICC1 chA: TimeHAT PHC PPS OUT (SDP0)  │
  │  TICC1 chB: F9T-BOT PPS                 │
  │  TICC1 ref: 10 MHz from FE-5680A Rb     │
  │                                          │
  │  TICC2 chA: otcBob1 PPS OUT             │
  │  TICC2 chB: (not connected)             │
  │  TICC2 ref: 10 MHz from FE-5680A Rb     │
  │                                          │
  │  eth0: 10.168.60.242                     │
  └──────────────────────────────────────────┘

         10.168.13.0/24 (PTP LAN)
        ┌────────────────────────────────────────────────┐
        │  FS IES3110-8TFP-R (10.168.13.90)              │
        │  PTP transparent clock, PoE                    │
        │                                                │
        │  Port: M600 (10.168.13.5) — PTP GM domain 0   │
        │  Port: ptBoat eth0 (10.168.13.15) — GM dom 10 │
        │  Port: bbb eth0 (10.168.13.14) — GM dom 20    │
        │  Port: TimeHat eth1 (10.168.13.26) — GM dom 30│
        │  Port: otcBob1 eth0 (10.168.13.16) — GM dom 40│
        │  Port: pi4ptpmon eth0 (10.168.13.13) — listener   │
        └────────────────────────────────────────────────┘
```

### PTP domain assignments

| Domain | GM host | PTP identity | Clock source | Discipline | NIC / PHC | Software |
|--------|---------|-------------|--------------|-----------|-----------|----------|
| 0 | M600 | ec4670.fffe.0024cb | GPS | Internal GPSDO | Meinberg internal | Meinberg firmware |
| 10 | ptBoat | 8c1f00.fffe.10417f | F9T PPS → SDP0 | Timebeat PPS+qErr | i226 (igc) | Timebeat 2.3.5 |
| 20 | bbb | 3403de.fffe.7f5514 | Adafruit GPS PPS | chrony → ptp4l | AM335x CPSW (100 Mbps) | linuxptp ptp4l |
| 30 | TimeHat | 54494d.fffe.45006b | F9T-3RD PPS → SDP1 | SatPulse | i226 (igc) | ptp4l via SatPulse |
| 40 | otcBob1 | 8c1f00.fffe.104149 | F9T PPS → SDP0 | Timebeat PPS+qErr | i226 (igc), OCXO | Timebeat 2.3.5 |
| 50 | clkPoC3 | dca632.fffe.e6dd68 | F9T PPS → SYNC_IN | ts2phc | BCM54210PE (bcm_phy_ptp), one-step | linuxptp ts2phc + ptp4l |

All PTP GMs share one L2 broadcast domain via the FS IES3110-8TFP-R switch
configured as a PTP transparent clock (p2ptransparent, onestep, ip4mixed).

### NTP infrastructure

| Hostname | IP | Role | Clock source |
|----------|-----|------|-------------|
| ntp1.VanValzah.Com | 10.168.60.5 | Stratum 1 NTP server | Meinberg M600 (GPS, Patch1 via SV1AFN L1 splitter) |
| ntp[2-5].VanValzah.Com | 10.168.60.20/28/30/25 | Stratum 2+ NTP servers | ntp1 + Internet NTS servers (offset-corrected) |

ntp1 is the M600 acting as a local stratum 1 NTP server.
ntp[2-5] are Proxmox PVE/PBS bare-metal servers with TCXO-only clocks,
syncing from ntp1 and a handful of Internet NTS servers. Offsets applied to
Internet NTS sources to compensate for asymmetric Internet path latency.
ntp[2-5] also peer with each other for local resilience.
NTS is used on Internet sources for MITM protection.

### Antenna assignments

| Receiver | Host | Antenna | Splitter | Mount site |
|----------|------|---------|----------|------------|
| F9T-TOP (ZED-F9T, TIM 2.20) | TimeHat | Patch3 | GUS #2 | East roof slope |
| F9T-BOT (ZED-F9T-20B, TIM 2.25) | MadHat | Patch3 | GUS #2 | East roof slope |
| F9T-3RD (ZED-F9T-20B, TIM 2.25) | clkPoC3 | Patch3 | GUS #2 | East roof slope |
| F9T (OTC SBC) | otcBob1 | UFO (SPK6618H) | GUS #1 | West roof slope |
| PX1125T (SkyTraq) | Onocoy | UFO (SPK6618H) | GUS #1 | West roof slope |
| Adafruit Ultimate GPS (MTK-3301) | bbb | Patch1 | SV1AFN L1 | East roof slope |
| F9T (OTC Mini PT) | ptBoat | SPK6618H (dedicated, 1 ft) | — | West roof slope |

### TICC wiring

**TICC #1** (TimeHat, /dev/ticc1)

| Channel | Source | PPS from |
|---------|--------|----------|
| chA | TimeHat i226 PEROUT (SDP0, PPS OUT) | PHC-disciplined |
| chB | F9T-TOP PPS (EVK SMA) | Patch3 via GUS #2 |

**TICC #2** (MadHat, /dev/ticc2)

| Channel | Source | PPS from |
|---------|--------|----------|
| chA | MadHat i226 PEROUT (SDP0, PPS OUT) | PHC-disciplined |
| chB | F9T-BOT PPS (EVK SMA) | Patch3 via GUS #2 |

**TICC #3** (clkPoC3, /dev/ticc3)

| Channel | Source | PPS from |
|---------|--------|----------|
| chA | DO PPS (IsoTemp OCXO via AD5693R DAC, 2× mode) | DAC-disciplined |
| chB | F9T-3RD PPS (EVK SMA) | Patch3 via GUS #2 |

> ⚠️ **TICC↔host and chA-source rows above are STALE after the 2026-06-30
> rewiring** (TICC #2 is now on PiFace; MadHat's i226 was removed 2026-05-24).
> See the 2026-06-30 change-log entry for the authoritative current DO assignments.

### 10 MHz reference chain

```
[FE-5680A Rb] ──10 MHz──→ [SV1AFN Dist Amp] ──10 MHz──→ [TICC #1 ref]
 (Rb, undisc.)             (fan-out/buffer) ──10 MHz──→ [TICC #2 ref]
                                            ──10 MHz──→ [TICC #N ref ...]
```

**Reference is the Frequency Electronics FE-5680A Rb standard** (10 MHz),
buffered/fanned out by the SV1AFN distribution amp to every TICC reference
input.  It replaced the Geppetto Electronics GPSDO (OCXO) on 2026-05-10 after
storms knocked the Geppetto out on 2026-05-07.  The Rb is undisciplined, so its
frequency is slightly off 10.0000000 MHz (constant offset, detrends out of any
chA stability metric), but its short-term TDEV is far better than the OCXO —
that's what sets the lab TICC measurement floor (now Rb-class, sub-ns).

### Serial console

| Host port | Target |
|-----------|--------|
| Onocoy /dev/ttyUSB2 | FS IES3110 console (115200 8N1) |

---

## Change log

Record topology changes here so the history of what was connected when
is preserved. Include date, what changed, and why.

### 2026-06-30 (round 2) — PiPuss recommissioned as a clock host

Pre-London "round 2" peripheral swap.  **PiPuss** returns as a CLOCK host (it had
been receiver-only ZED-X20P, then mothballed) — F9T-20 + IsoTemp-class OCXO +
per-clock TICC, mirroring PiFace.  Peripherals consolidated onto it (all verified
present by inventory 2026-06-30):

| Peripheral | From | On PiPuss | Notes |
|---|---|---|---|
| F9T-20B receiver **SEC-UNIQID `2342aa323d`** (uid 262843023907) | PiFace | `/dev/gnss-bot` (USB 1546:01a9) | this unit had been on PiFace (state file 06-29; ultimately ex-MadHat).  PiFace is NOT left receiver-less — it keeps its OTHER F9T-20B **`3b41fabd5b`** (ex-MadHat). |
| OCXO + AD5693R DAC + PulsePuppy + divider (DO `ocxo-clkpoc3`) | clkPoC3 | i2c-1 `0x4C` | char files moved to PiPuss `state/dos/`; archived `gt/do-char-archive/ocxo-clkpoc3-moved-to-pipuss-20260630/` |
| TICC #5 | clkPoC3 | `/dev/ticc5` | Arduino serial …406232; udev rule added to `99-timelab.rules` 2026-06-30 |
| OCXO temperature sensor | (new) | i2c-1 `0x48` | NEW; auto-read by engine `TempSensor(bus_num=1)` → `[TEMP_C]` |

Resulting DO assignments (supersedes the round-1 clkPoC3 row below):
- **PiPuss** — DO `ocxo-clkpoc3` (OCXO + AD5693R DAC, 1× gain) + F9T-20B **`2342aa323d`** + TICC #5 + temp sensor.  CLOCK host.
- **clkPoC3** — DO REMOVED (→ PiPuss); no receiver (X20 pulled earlier).  Now a TICC-only logging host (keeps **TICC #4** = GNSSDO+ / PiFace logger).
- **PiFace** — STILL a live clock host: F9T-20B **`3b41fabd5b`** (ex-MadHat) + IsoTemp `isotemp-ocxo131-100-madhat` + TICC #2, running `--no-antposest`.  (Earlier note that PiFace went "receiver-less" was WRONG — corrected here after reading serials 2026-06-30.)

**F9T-20B unit inventory (SEC-UNIQID — the only way to tell EVKs apart; no USB serial).**
Confirmed by reading `Receiver identity ... id=` log lines + receiver state files 2026-06-30:

| SEC-UNIQID | uid (decimal) | now on | provenance |
|---|---|---|---|
| `2342aa323d` | 262843023907 | **PiPuss** | MadHat → PiFace → PiPuss (most recently from PiFace, round 2) |
| `3b41fabd5b` | 394029318459 | **PiFace** | MadHat → PiFace (came with the IsoTemp board); MadHat still has its `394029318459.json` state file |

(PiFace's prior F9P `5d58b2dad4` was replaced by the ex-MadHat F9T-20B on 2026-06-30.)
⇒ **PiPuss + PiFace are now the matched F9T-20B + IsoTemp clock pair** the recommissioning aimed for.

`config/pipuss.toml` rewritten from the X20P receiver-only config to the clock-host
config (X20P config preserved in git history).  **BROUGHT UP 2026-06-30** and disciplining
(`--no-antposest`, servo err ~0.46 ns RMS, TICC resid ~50 ps).  Root cause of the initial
"divider won't lock" was a **missing `gpiod` module** in the venv (silent no-op ARM), not
wiring — fixed by `pip install gpiod==2.4.2` and now a declared dep (see
`docs/misnomers.md` / memory `reference_gpiod_required_on_tadd_hosts`).  The `ocxo-clkpoc3`
do_label is now a host-named misnomer (DO no longer on clkPoC3) — logged in
`docs/misnomers.md`, rename deferred.

### 2026-06-30 — DO (disciplined-oscillator) reassignments + board moves

Corrects long-stale DO attributions in the TICC-wiring tables above (they had
listed i226 PEROUT for TimeHat/MadHat and "CTI OSC5A2B02" for clkPoC3 — wrong).
**Authoritative current DO-per-host assignments:**

| Host | Disciplined Oscillator | Actuator | Notes |
|---|---|---|---|
| TimeHat | i226 internal TCXO | PHC `adjfine` | i226 PHC still here |
| clkPoC3 | **IsoTemp OCXO** | AD5693R DAC (2× mode) | NOT the CTI — that was a stale doc error |
| **PiFace** | **IsoTemp OCXO131-100 (ex-MadHat)** | AD5693R DAC + TADD-2 divider | swapped in 2026-06-30; do_label=`isotemp-ocxo131-100-madhat`; also took TICC #2 (`/dev/ticc2`) + an F9T |
| MadHat | (DO moved to PiFace; currently DO-less / being reworked) | — | its IsoTemp OCXO131-100 went to PiFace; i226 removed 2026-05-24 |
| otcBob1 | ClockMatrix OCXO | combo servo (on-chip) | unchanged |
| ptBoat | ClockMatrix OCXO | — | combo blocked |

- **CTI OSC5A2B02 OCXO + TADD-2 divider** (10 MHz, AD5693R DAC) was **PiFace's**
  former DO — moved OFF PiFace 2026-06-30, destination TBD. Its characterization is
  archived at `gt/do-char-archive/ocxo-piface-CTI-OSC5A2B02-TADD2-saved-20260630/`
  (restore on its next host: place the state/dos files, set that host's
  `do_label = "ocxo-piface"`).
- **TICC #2 moved MadHat → PiFace.** The "TICC #2 (MadHat)" / "TICC #3 (clkPoC3)"
  table headers above and their chA rows need re-verification against the current
  rewiring before being trusted.

### 2026-05-10 — TICC 10 MHz reference switched to FE-5680A Rb standard

- The Geppetto Electronics GPSDO (OCXO) that fed the SV1AFN distribution amp
  was knocked out by thunderstorms 2026-05-07.  The dist amp briefly ran on its
  own internal TCXO (which set a ~1 ns @τ=1s lab-wide floor — see
  `project_geppetto_out_2026_05_07` memory).
- **2026-05-10**: Bob wired a shelved **Frequency Electronics FE-5680A Rb
  standard** as the SV1AFN input, replacing the Geppetto.  Rb (FE-5680A model
  confirmed 2026-06-29) → SV1AFN dist amp (buffers/fans out) → every TICC
  reference input.  Undisciplined (slightly off 10 MHz, constant offset that
  detrends out), but far better short-term TDEV than the OCXO → the lab TICC
  measurement floor is now Rb-class (sub-ns at all reasonable τ).
- Earlier change-log/gear entries naming "Geppetto GPSDO" as the TICC reference
  are superseded by this entry.

### 2026-04-16 — TICC wiring updated, PiPuss in drawer, antenna clarified

- **PiPuss is powered down and stored in a drawer.**  Both F9Ts have
  moved to other hosts (see 2026-04-13 entry).  PiPuss no longer has
  any receivers or TICCs.
- **All three F9Ts share Patch3 antenna via GUS #2 splitter.**  This is
  the same antenna/splitter that was used for PiPuss zero-baseline testing
  (see 2026-03-20 entry).  Same ARP for all three receivers.
- **TICC wiring (current)**:
  - TICC #1 on TimeHat (/dev/ticc1): chA = TimeHat i226 PEROUT (PPS OUT),
    chB = F9T-TOP PPS (GNSS PPS)
  - TICC #2 on MadHat (/dev/ticc2): chA = MadHat i226 PEROUT (PPS OUT),
    chB = F9T-BOT PPS (GNSS PPS)
  - TICC #3 on clkPoC3 (/dev/ticc3): chA = DO PPS (OCXO-derived),
    chB = F9T-3RD PPS (GNSS PPS)
  - All three TICCs referenced to Geppetto GPSDO 10 MHz via SV1AFN dist amp
- **PTP device paths**: TimeHat and MadHat have i226 PHC at `/dev/ptp1`
  (not `/dev/ptp0` as in the host configs — needs config file update)

### 2026-04-13 — Receiver shuffle, PiPuss renamed to MadHat, all three F9Ts on one antenna

- **PiPuss renamed to MadHat** (hostname changed, reachable via `ssh MadHat.local`)
- **F9T-TOP** (EVK-F9T-10-00, ZED-F9T, TIM 2.20, uniqueId=136395244089)
  moved from PiPuss/MadHat back to **TimeHat** on `/dev/gnss-top`
- **F9T-3RD** (EVK-F9T-20-00, ZED-F9T-20B, TIM 2.25, uniqueId=394029318459)
  moved from TimeHat to **clkPoC3** on `/dev/ttyACM1`
- **F9T-BOT** (EVK-F9T-20-00, ZED-F9T-20B, TIM 2.25, uniqueId=262843023907)
  stays on MadHat on `/dev/ttyACM0`
- **All three F9Ts now share one antenna** via a splitter (same ARP).
  Antenna and splitter identity TBD — verify which antenna/GUS.
- Note: F9T-TOP is EVK-F9T-10-00 (L1/L2 hardware) but
  `ensure_receiver_ready()` auto-configures it for L5 (ZED-F9T silicon
  supports both L2 and L5). CFG-VALGET confirms L2C_ENA=0, L5_ENA=1.
- Verified 2026-04-14 via MON-VER + CFG-VALGET + SEC-UNIQID on all
  three hosts.

### 2026-04-08 — Onocoy mothballed; TICC #2 moved to ocxo for i226 bring-up

- **Onocoy host powered down and mothballed.**  No further work planned
  on this host.  F10T (ArduSimple) and PX1125T physically disconnected
  and stored.  Confirmed Onocoy never had a peppar-fix checkout, so
  nothing to clean up software-side.
- **TICC #2 relocated** from PiPuss to ocxo (was on PiPuss as of the
  2026-04-04 entry, briefly back to Onocoy in between, now on ocxo).
  Now `/dev/ticc2` on ocxo via the udev symlink rule (Arduino serial
  44236313835351B02001 — stable across host moves).
- **TICC #2 channel mapping on ocxo**: chA = i226 PEROUT (SDP0),
  chB = F9T-TOP PPS via SMA tee.  Matches the TimeHat/MadHat layout
  for direct comparison.
- **F9T-TOP moved to ocxo** as the timing receiver for the new i226
  add-in card (was on PiPuss).  USB CDC ACM, accessed via
  `/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00`
  (the only stable handle — no SEC-UNIQID exposed via udev).
- New i226 add-in card on ocxo: bare Intel-branded i226 retail card,
  hand-wired pin headers + Timebeat u.FL adapter + SMA.  Stable name
  `i226` via MAC-keyed udev rule landed today.  See
  `docs/ocxo-i226-bringup-2026-04-08.md` for the bring-up history.

### 2026-04-04 — TICC #2 moved to PiPuss for otcBob1 ClockMatrix work

- TICC #2 moved from Onocoy to PiPuss (/dev/ticc2)
- TICC #2 chA wired to otcBob1 PPS OUT (ClockMatrix-derived via i226 PEROUT)
- TICC #2 chB not connected
- TICC #2 10 MHz ref from Geppetto GPSDO via SV1AFN dist amp (unchanged)
- Purpose: independent verification of ClockMatrix frequency steering via
  peppar-fix. DPLL_3 drives the 25 MHz → i226, we steer via FOD_FREQ I2C
  writes, TICC measures resulting PPS against F9T PPS reference.
- Onocoy no longer has a TICC

### 2026-03-20 — PiPuss zero-baseline: both F9Ts on Patch3 via GUS #2

- F9T-TOP moved from Patch2 to Patch3 via GUS #2 (was TBD/Patch2)
- Both F9T-TOP and F9T-BOT now share Patch3 antenna via GUS #2
- Purpose: zero-baseline caster/client testing for NTRIP caster development
- GUS #2 now feeds 3 receivers: F9T-TOP (PiPuss), F9T-BOT (PiPuss), F10T (Onocoy)
- Verified both F9Ts responding: F9T-TOP uniqueId=136395244089, F9T-BOT uniqueId=262843023907

### 2026-03-19 — TICC skeptical calibration rewire + USB audit

- TICCs moved for skeptical calibration: #1→TimeHat, #2→Onocoy (unchanged), #3→PiPuss
- SMA TEE tree distributes PPS OUT and PPS IN to all 3 TICCs for cross-validation
- Universal udev rules deployed (99-timelab.rules): TICC serial numbers portable across hosts
- Onocoy USB fully audited: TICC #2 (Arduino ACM), PX1125T (CP2102), FS console (Prolific), F10T (ArduSimple FTDI D30GD1PE)
- F10T confirmed as FTDI FT230X, NOT u-blox CDC ACM
- USB isolators tested on TimeHat: no measurable effect (dominant noise is F9T sawtooth character)
- TICC calibration: all 3 agree within 30ps noise floor
- Stale analyze_servo.py process (running since Mar 16) killed on PiPuss

### 2026-03-16 — F9T-3RD on TimeHat, TICC #3, F9T-TOP back to PiPuss

- F9T-3RD (EVK-F9T-20-00, ZED-F9T-20B, TIM 2.25) installed on TimeHat
  - Replaces F9T-TOP which moves back to PiPuss for testAnt use
  - PPS wired to TimeHAT SDP1 + TICC #3 chB
  - Antenna: UFO (SPK6618H) via GUS #1, port: /dev/gnss-top (udev symlink)
- F9T-TOP (EVK-F9T-10-00, ZED-F9T, TIM 2.20) returned to PiPuss
  - Back on /dev/gnss-top (PiPuss udev rules already in place)
- New TAPR TICC (#3) installed on TimeHat
  - chA: TimeHAT PHC PPS OUT (SDP0, SMA1 J4, disciplined)
  - chB: F9T-3RD PPS (EVK SMA, raw GPS reference)
  - ref: 10 MHz from SV1AFN distribution amp (spare output)
- udev rules deployed: /etc/udev/rules.d/99-timehat-devices.rules
  - /dev/ticc: TICC #3 (matched by Arduino serial number 44236313835351B0A091)
  - /dev/gnss-top: F9T-3RD (matched by USB ID_PATH)
- Purpose: local TDEV measurement for PePPAR Fix M5 PHC discipline loop;
  previously TICC #1 (PiPuss) measured TimeHAT PPS via cross-host cable

### 2026-03-13 — PTP domain assignments, Timebeat 2.3.5, TICC rewire

- PTP domains assigned: M600=0, ptBoat=10, bbb=20, TimeHat=30, otcBob1=40
- ptBoat: Timebeat upgraded 2.2.20→2.3.5, PTP GM enabled on domain 10
- otcBob1: Timebeat upgraded 2.3.2→2.3.5, PTP GM enabled on domain 40
- Both Timebeat hosts now monitor all other PTP domains as secondaries
- NTP secondaries changed from dns[1-5] to ntp[1-5].VanValzah.Com
- pi4ptpmon moved to static IP 10.168.13.13/24, swapped into PTP LAN switch
- TimeHAT SDP0 configured as PHC PPS OUT (systemd phc-pps-out.service)
- TICC rewired: chA = TimeHAT PHC PPS OUT, chB = F9T-BOT PPS

### 2026-03-12 — SatPulse setup: F9T-TOP moved to TimeHat

- F9T-TOP (EVK-F9T-10-00, ZED-F9T, TIM 2.20) moved from PiPuss to TimeHat
- F9T-TOP wired directly to Patch2, PPS to TimeHAT SDP1
- F9T-BOT (EVK-F9T-20-00, ZED-F9T-20B, TIM 2.25) stays on PiPuss as /dev/gnss-bot
- Patch3 split 2-way via GUS: F9T-BOT (PiPuss) + F10T (Onocoy)
- TICC chA now from F9T-BOT; chB disconnected
- SatPulse config: pin=1 (SDP1), /dev/ttyACM0, 115200 baud

### 2026-03-12 (earlier) — Patch3/Patch2 on PiPuss

- Patch3 on BOT, Patch2 on TOP, both East roof slope
- This is the config from the 25h run completed 2026-03-11
- Previous config was Patch1-BOT / Patch2-TOP (2026-03-09 to 03-10)

### 2026-03-09 — Patch1/Patch2 swap

- Swapped from UFO/Choke to Patch1-BOT / Patch2-TOP
- Both on East roof slope
- Stopped the 84h UFO-top / Choke-desk run to free serial ports

### 2026-03-06 — UFO/Choke ring long run

- UFO on TOP (East roof slope), Choke ring on BOT (desk, indoors)
- 84h+ multi-day capture (started 2026-03-06 05:06 UTC)
- Choke ring was indoors on desk — not representative of outdoor performance
