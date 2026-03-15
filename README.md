# PePPAR Fix — Fixed-Position PPP-AR Disciplined Oscillator

A carrier-phase GNSS disciplined oscillator for precision UTC phase alignment.
Uses fixed-position PPP-AR (Precise Point Positioning with Ambiguity Resolution)
to continuously estimate receiver clock offset and steer a NIC's PTP Hardware
Clock — replacing the traditional PPS + qErr approach with sub-nanosecond
carrier-phase corrections.

*"Arr" = Ambiguity Resolution (and pirates).*

## Why this exists

The best PPS-based PHC discipline (SatPulse with qErr correction) achieves
~6 ns mean offset.  The PPS+qErr error signal is:
- **Discrete** — one correction per second
- **Coarse** — ±4-5 ns quantization on modern u-blox receivers
- **Code-phase limited** — fundamentally bounded by pseudorange noise

Carrier-phase observations are ~100x more precise than code.  A fixed-position
PPP-AR filter can estimate the receiver clock continuously at 1-10 Hz with
sub-nanosecond precision, giving a much richer error signal to the PHC servo.

## Architecture

```
┌──────────────┐  UBX-RXM-RAWX        ┌───────────────┐
│  u-blox F9T  │──────────────────────▸│               │
│  (serial)    │  L1+L5 code+carrier   │  Fixed-Pos    │
└──────────────┘                       │  PPP-AR       │
                                       │  Kalman       │──▸ adjfine(PHC)
┌──────────────┐  RTCM3-SSR (NTRIP)   │  Filter       │
│  IGS/CNES    │──────────────────────▸│               │
│  (CLK93 etc) │  orbits, clocks,     │               │
└──────────────┘  phase biases         └───────────────┘
```

### Fixed position simplifies PPP-AR dramatically

| Full PPP-AR (moving) | PePPAR Fix (fixed) |
|---|---|
| Estimate X, Y, Z every epoch | Position known (from bootstrap or survey) |
| 15-30 min convergence | Seconds (clock-only after bootstrap) |
| Velocity + dynamics states | None |
| ~30-50 state Kalman filter | ~25-45 states, simpler process noise |

### State vector

**Bootstrap phase** (first run, ~30 min):
```
x = [X, Y, Z, dt_rx, dt_dot, ZWD, N1, N2, ..., Nn, ISB_gal, ISB_bds]
```
Position converges to cm-level, then is saved and frozen.

**Operational phase** (all subsequent runs):
```
x = [dt_rx, dt_dot, ZWD, N1, N2, ..., Nn, ISB_gal, ISB_bds]
```
Clock converges in seconds.  The filter outputs `dt_rx` at observation rate —
this is the PHC servo's error signal.

### Dual-frequency ionosphere cancellation

With L1 + L5 (GPS) or E1 + E5a (Galileo), the ionosphere-free combination
eliminates first-order ionospheric delay.  No external ionosphere model needed.

### Real-time IGS corrections

SSR (State Space Representation) corrections from IGS real-time service:
- Satellite orbit corrections (cm-level)
- Satellite clock corrections (sub-ns)
- Phase biases (for integer ambiguity resolution)
- Delivered via NTRIP (e.g., CNES CLK93 stream, ~5-30s latency)

### Starting position accuracy

A u-blox survey-in position (±2 m XY, ±4 m Z) is fine.  The filter absorbs
the initial position error into clock bias and converges the position during
bootstrap.  4 m vertical error ≈ 13 ns initial clock bias, which resolves
in minutes once carrier-phase ambiguities fix.

## Hardware requirements and limitations

### Required: PPS → SDP IN (closes the control loop)

The GNSS receiver's PPS output must be wired to an SDP input pin on the PHC.
This is the only way to observe the PHC's actual phase relative to GNSS time
and close the discipline loop.  Without it, PePPAR Fix can compute a clock
model from carrier-phase observations, but has no way to measure the PHC's
TCXO frequency offset — the loop is open.

The Kalman filter estimates the *GNSS receiver's* clock (dt_rx, dt_dot), not
the PHC's clock.  The PHC has its own TCXO with an unknown, temperature-
dependent frequency offset that must be observed and corrected.  Each PPS
edge timestamped by the PHC via SDP provides that observation:

```
PHC error = T_phc(PPS) − T_true(PPS)
          = PHC timestamp of PPS edge − (GPS second + dt_rx from filter)
```

PePPAR Fix gains over SatPulse because carrier phase provides the frequency
component (dt_dot) at sub-ppb precision.  The noisy SDP path (~15-30 ns
jitter on i226) only needs to carry the slowly-varying phase correction,
not the frequency — so it can be averaged much more aggressively.

### Optional: SDP OUT (disciplined PPS or other frequencies)

If a second SDP pin is available, the PHC can generate a disciplined PPS
output (or other frequencies) for external instrumentation or verification.
Without SDP OUT, the disciplined PHC is only accessible to software on the
host — there is no physical output path without significant precision loss.

### Limitation: PHC and PHY TX clock are independent

On standard Intel NICs (i210, i225, i226, E810), the PHC is a digital
timestamp counter adjusted by `adjfine()`.  The Ethernet PHY's transmit
clock runs from a separate fixed crystal.  Disciplining the PHC does **not**
change the Ethernet wire frequency.  There is no connection between the two
clock domains.

This means:
- **No SyncE from a disciplined PHC** on standard Intel NICs.  Even on the
  E810 (which has SyncE via a dedicated DPLL), the PHC and the SyncE DPLL
  are separate subsystems.
- **PTP is the network path** for distributing the disciplined time.  PTP
  Sync/Follow_Up messages embed PHC timestamps — a better-disciplined PHC
  produces more accurate timestamps — but the transfer is limited by
  network packet jitter and HW timestamp resolution, not by the PHC
  discipline quality.

### Expected performance vs. GPSDO-driven PTP GM

A PePPAR Fix PTP GM should significantly outperform an ordinary GPSDO-driven
PTP GM (where GPSDO → PPS → SDP → PHC discipline) because:
- Same PTP packet path to the slave, same HW timestamp precision
- But the GM's PHC timestamps are more accurate: carrier-phase frequency
  tracking (sub-ppb) vs. PPS-only (~20 ns jitter per edge)
- The bottleneck shifts from "how well is the GM's PHC disciplined" to
  "how well does PTP transfer that discipline to the slave"

### Development hardware

| Component | Model | Notes |
|-----------|-------|-------|
| SBC | Raspberry Pi 5 | |
| NIC | TimeHAT (Intel i226) | TCXO ±280 ppb, 4 SDP pins |
| GNSS | u-blox ZED-F9T | L1/L5, UBX-RXM-RAWX at 1 Hz |
| Antenna | TBD | Active, L1/L5, roof-mounted |
| NTRIP | IGS real-time service | CLK93 or similar SSR stream |

## Quick start — Milestone 1: Raw observations

```bash
pip install pyubx2 pyserial

# Configure F9T (factory reset → signals → survey-in → enable messages)
python scripts/configure_f9t.py /dev/ttyF9T

# With options:
python scripts/configure_f9t.py /dev/ttyF9T \
    --rate 1 --survey-dur 300 --survey-acc 5.0

# Log observations (Ctrl-C or --duration to stop)
python scripts/log_observations.py /dev/ttyF9T --baud 460800 --duration 3600
```

### configure_f9t.py

Configures a u-blox ZED-F9T for PPP-AR:
1. Factory reset (controlled SW reset)
2. Enable dual-frequency: GPS L1C/A+L5, Galileo E1+E5a, BeiDou B1+B2a
3. Disable GLONASS (FDMA complicates ambiguity resolution)
4. Set measurement rate (1-10 Hz, default 1 Hz)
5. Enable UBX messages: RXM-RAWX, RXM-SFRBX, NAV-PVT, NAV-SAT, TIM-TP
6. Disable NMEA output (bandwidth)
7. Start survey-in (300s, 5m accuracy — enough to bootstrap PPP-AR)
8. Set UART baud to 460800 (headroom for 10 Hz RAWX)
9. Save to flash

### log_observations.py

Captures observations to CSV + raw UBX binary:
- `<prefix>_rawx.csv` — per-SV per-epoch: pseudorange, carrier phase, Doppler, C/N0
- `<prefix>_pvt.csv` — position/velocity/time (for bootstrap monitoring)
- `<prefix>_timtp.csv` — PPS quantization error
- `<prefix>_raw.ubx` — raw binary (for replay with different parsers)

## Implementation plan

### Phase 1: Observation pipeline

- ✓ Configure F9T for dual-frequency raw observations
- ✓ Log RXM-RAWX, RXM-SFRBX, NAV-PVT, TIM-TP to CSV + binary
- Parse UBX-NAV-SAT (satellite positions, or compute from broadcast ephemeris)
- Connect to NTRIP caster, decode RTCM3-SSR corrections
- Log everything for offline development

### Phase 2: Offline Kalman filter (Python/numpy)

- Implement the fixed-position filter in Python for rapid iteration
- Process logged RXM-RAWX + SSR data
- Validate clock estimates against TICC measurements from the PE rig
- Bootstrap: estimate position, verify convergence
- Operational: freeze position, verify clock-only mode

### Phase 3: Real-time filter + PHC discipline

- Port filter to real-time operation (Python or Rust)
- Output clock corrections to a PHC servo (adjfine)
- Compare ADEV against SatPulse (PPS+qErr) baseline
- Measure improvement at various tau

### Phase 4: Holdover (future)

- Characterize the TimeHAT TCXO (drift, temperature sensitivity)
- Model oscillator behavior in the Kalman filter
- Predict and compensate drift during GNSS outages

## References

- [SatPulse](https://satpulse.net/) — PPS+qErr baseline to beat
- [PRIDE-PPPAR](https://github.com/PrideLab/PRIDE-PPPAR) — PPP-AR reference
- [ginan](https://github.com/GeoscienceAustralia/ginan) — Full GNSS toolkit
- [RTKLIB](https://github.com/tomojitakasu/RTKLIB) — Classic GNSS processing
- [IGS Real-Time Service](https://igs.org/rts/) — SSR correction streams
- [CNES CLK93](http://www.ppp-wizard.net/products/REAL_TIME/) — Free SSR stream
- [u-blox F9T Integration Manual](https://content.u-blox.com/sites/default/files/ZED-F9T_IntegrationManual_UBX-21040375.pdf)

## Quick start — Milestone 4: Real-time clock estimation

```bash
pip install pyubx2 pyserial pyrtcm numpy

# 1. Register for IGS Real-Time Service: https://igs.org/rts/access/
#    You'll receive a username and password for the NTRIP caster.

# 2. Real-time mode (live F9T + NTRIP corrections):
python scripts/realtime_ppp.py \
    --serial /dev/gnss-bot --baud 115200 \
    --known-pos "41.8430626,-88.1037190,201.671" \
    --caster products.igs-ip.net --port 2101 \
    --eph-mount BCEP00BKG0 --ssr-mount SSRA00CNE0 \
    --user YOUR_USER --password YOUR_PASS \
    --duration 3600 --out data/realtime_1h.csv

# 3. Replay mode (validate with existing data):
python scripts/realtime_ppp.py \
    --replay data/rawx_1h_top_20260303.csv \
    --sp3 data/gfz_mgx_062.sp3 \
    --clk data/GFZ0MGXRAP_062_30S.CLK \
    --osb data/GFZ0MGXRAP_062_OSB.BIA \
    --known-pos "41.8430626,-88.1037190,201.671" \
    --systems gal --out data/replay_test.csv
```

### NTRIP caster registration

The IGS Real-Time Service requires free registration at https://igs.org/rts/access/.
Key mountpoints on `products.igs-ip.net:2101`:
- **BCEP00BKG0** — broadcast ephemeris (GPS+GLO+GAL+BDS+QZS, RTCM 1019/1042/1045/1046)
- **SSRA00CNE0** — CNES SSR corrections (standard RTCM SSR: 1059/1060/1242/1243/1260/1261)
- **SSRA00CNE1** — CNES SSR corrections (IGS SSR format: 4076_023/025/026/063/065/066/103/105/106)

## Repository layout

```
.
├── scripts/
│   ├── configure_f9t.py      # F9T configuration (reset → signals → survey-in)
│   ├── log_observations.py   # Raw observation logger (RAWX, PVT, TIM-TP)
│   ├── solve_pseudorange.py  # M2: GPS L1 pseudorange solver + SP3 parser
│   ├── solve_dualfreq.py     # M2: Dual-freq IF combination solver
│   ├── solve_ppp.py          # M3: Multi-GNSS PPP EKF + FixedPosFilter
│   ├── solve_gim.py          # Single-freq + GIM ionosphere solver
│   ├── ppp_corrections.py    # M3: File-based SP3/CLK/OSB correction parsers
│   ├── broadcast_eph.py      # M4: Broadcast ephemeris → ECEF (Keplerian model)
│   ├── ntrip_client.py       # M4: NTRIP v2 client for RTCM3 streams
│   ├── ssr_corrections.py    # M4: Real-time SSR correction state manager
│   └── realtime_ppp.py       # M4: Real-time PPP orchestrator
├── docs/
│   └── nic-survey.md         # PHC hardware survey (PPS IN+OUT qualified)
├── data/                     # Observation runs + correction products
├── requirements.txt          # Python dependencies
└── README.md
```

## Deployment goal

PePPAR Fix should be a self-contained software package that can be downloaded
and run on any Linux host with:
- A PTP Hardware Clock (PHC) with SDP pins or SyncE output (e.g., Intel i210/i225/i226)
- A dual-frequency GNSS receiver outputting UBX-RXM-RAWX (e.g., u-blox F9T)
- Internet access for NTRIP SSR corrections

**TODO (post-M4):** The tool needs an install/setup phase that handles:
- Dependency checks (linuxptp, kernel PTP support, serial access)
- PHC discovery and pin mapping (`testptp -c`, SDP enumeration)
- PPS OUT configuration (periodic output on an available SDP pin for
  external verification — see `phc-pps-out.service` on TimeHat)
- GNSS receiver detection and protocol version
- NTRIP credential configuration
- systemd service installation

This can't be just `pip install && run` — hardware configuration is required.

### Stretch goal: SyncE frequency distribution via Renesas DPLL

The "PHC and PHY are independent" limitation (above) has one exception:
on **Timebeat OTC hardware** (OTC, OTC Mini PT), the onboard **Renesas
8A34002 ClockMatrix** generates the 25 MHz reference clock that feeds the
i226 PHY.  PePPAR Fix could steer the *actual Ethernet wire frequency* —
not just the PHC timestamp counter — by writing frequency adjustments to
the 8A34002 via I2C.

Two independent control paths:
1. `adjfine()` on the i226 PHC — adjusts timestamps (PTP accuracy)
2. Renesas 8A34002 DPLL steering — adjusts wire frequency (SyncE)

A downstream host with SyncE receive capability (e.g., Intel E810) could
recover the GNSS-disciplined frequency from the Ethernet signal, enabling
physical-layer frequency syntonization without SDP pins or PPS cables.

The kernel has an in-tree driver (`ptp_clockmatrix.c`, `rsmu_i2c.c`) and
Renesas ships `pcm4l` for userspace DPLL control. The i226 doesn't need
SyncE receive support — it acts as a SyncE source, with the Renesas DPLL
providing the disciplined reference.

This does **not** enable White Rabbit.  WR requires FPGA-based DMTD (Dual
Mixer Time Difference) for sub-ns phase measurement — the i226 has no DMTD
capability.  PTP+SyncE from OTC hardware achieves ~50-100 ns, not WR's
sub-ns.

**Note:** This only applies to hardware with the Renesas clock generator
(Timebeat OTC boards). Plain i225/i226 NICs (including the TimeHAT) use a
fixed crystal and cannot do SyncE. SDP pins remain required on those
platforms.

## Status

**Milestone 4 in progress**: Real-time PPP clock estimation.

| Milestone | Status | Description |
|-----------|--------|-------------|
| M1 | Done | F9T configuration + observation logging |
| M2 | Done | Pseudorange position solver (GPS L1, dual-freq IF) |
| M3 | Done | Fixed-position clock estimation (file-based SP3+CLK+OSB) |
| M4 | **Active** | Real-time operation (NTRIP + broadcast eph + SSR corrections) |

M3 results (file-based, GAL-only): ADEV τ=1s: 1.5e-9, τ=30s: 2.3e-10.
Geometry error floor ~1.3m from SP3+CLK products — carrier phase precision
(~0.01m) swamped by correction product limitations. M4's real-time SSR
corrections should eliminate this bottleneck.
