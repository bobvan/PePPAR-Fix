# PePPAR Fix — Polecat Operating Manual

You are working on PePPAR Fix, a GNSS-disciplined precision clock system.
This file contains hard-won operational knowledge. Read it before writing
code or touching lab hardware.

## Project goal

PePPAR Fix aims to faithfully transfer the **long-term stability of GPS
time** to the **Disciplined Oscillator** (DO) — the crystal at the
servo's actuator (e.g., the i226 TCXO, the Timebeat OTC's OCXO via
ClockMatrix) — while preserving the DO's **superior short-term
stability**.

Two oscillators bound the achievable result:

1. **The Disciplined Oscillator (DO)** — the servo can't make it more
   stable than its own free-running noise floor.  It can only steer
   the DO's frequency, not eliminate its phase noise.

2. **The GNSS receiver's oscillator (RX TCXO)** — every carrier-phase
   observation is tainted by the receiver's clock noise.  Servo inputs
   derived from the receiver (PPP dt_rx, PPS edges with qErr) inherit
   this floor.  We can't pull the DO below the RX TCXO's stability
   using GNSS-based inputs.

The **moonshot target**: at every tau, the DO output is as stable as
the *best* of (DO free-running noise floor, RX TCXO noise floor).  At
short tau the better oscillator's noise floor should shine through
unmolested by the discipline loop.  At long tau the DO should track
GPS time as faithfully as the GNSS receiver allows.  The discipline
loop should guide the transition ever so gently — no servo-induced
noise, no overshoot, no loop-bandwidth artifacts.

Beating PPS or PPS+qErr alone is not the goal — those are limited by
the F9T's measurement resolution, not by what either oscillator can
actually deliver.

### Cross-host PPS OUT agreement

An equal-weight goal: **any pair of PePPAR Fix clocks must produce
PPS OUT edges that agree in frequency and phase, ideally sub-ns.**
Measured by connecting the two PPS OUT signals to two channels of a
shared-reference TICC (chA and chB on the same unit), so the
differential TDEV is unaffected by that TICC's own reference noise.

Two stages:

1. **Shared antenna first** — two clocks driven by the same RF via a
   splitter.  Eliminates atmospheric, multipath, and orbit/clock
   correction variability as sources of disagreement.  What remains
   is the discipline loop's own contribution to phase noise plus any
   per-receiver biases or per-filter integer-resolution errors.  This
   is the cleanest test bed for servo design and for catching
   ambiguity-resolution bugs.

2. **Separate antennas next** — the real-world goal.  Two clocks at
   independent sites driving independent antennas must agree in
   phase/frequency after both converge, limited only by per-site
   atmospheric and multipath differentials.

Cross-host PPS agreement is downstream of cross-host *position*
agreement: until two PePPAR Fix receivers converge to the same ARP,
their clock solutions will absorb the position disagreement and
their PPS edges can't agree either.  So before chasing sub-ns PPS
alignment, the position solutions must agree to sub-cm on a shared
antenna (and to within multipath/atmospheric limits on separate
antennas).

We are searching for:

- The best **servo input** — PPS, PPS+qErr, PPP carrier phase, PPP-AR
- The best **servo tuning** — loop bandwidth, gain scheduling, anti-windup
- The right **bootstrap initialization** — drift file, frequency seed
- The right **measurement chain** — TICC, EXTTS, on-chip TDC — to
  characterize each component without contaminating the result

Along the way we document and illustrate the obstacles: measurement
noise floors, quantization errors at every stage, oscillator drift
sources, two-oscillator differentials, loop dynamics.  Each gets a
story in `docs/visual-stories.md`.

## Before running on a lab host — read this first

**Read `docs/lab-operations.md`** for the deployment procedure,
pre-flight checklist, and known stumbling points.

### The repo is the source of truth

Every lab host has its `peppar-fix` checkout at `~/peppar-fix`, and
that directory **must** be a git working tree.  The first thing to do
when starting work on any lab host is:

```sh
ssh <host>
cd ~/peppar-fix
git status        # ← see what's locally modified before doing anything
git pull          # ← when you actually want fresh code from upstream
```

`git pull` is on-demand, not automatic.  Lab hosts can lag the upstream
indefinitely — that's a feature when one host is in a known-good state
you want to keep as a comparison baseline.  Pull only when you have a
reason to.  Always `git status` before pulling so you know whether
local edits are about to land in a merge.

**Local edits on a lab host are encouraged**, not avoided.  When
you're debugging something that only reproduces on a particular host,
edit files directly in `~/peppar-fix` on that host, test there, and
let `git status`/`git diff` track what you tried.  Don't pre-commit
every speculative fix — `git checkout -- <file>` discards what didn't
pan out, and `git add && git commit` keeps what did.  When a fix is
worth keeping, push *from the lab host*:

```sh
git push origin main           # ← lands on gt's bare local upstream
                               #    which auto-mirrors to GitHub
```

You can then pull the same fix to other lab hosts to confirm it didn't
break them, all before publishing anything beyond the bare upstream.

### **Hard rule: never `scp` or `rsync` code that's tracked in the repo.**

If you find yourself wanting to `scp scripts/foo.py to:somewhere` or
`rsync -a scripts/peppar_fix/ to:somewhere`, **stop**.  That's a sign
the workflow has broken — fix it via commit + push + pull on the
affected hosts.  scp around version control just creates drift between
hosts that git can't see.  Catastrophes from violating this rule
include the 2026-04-08 ocxo incident where multiple non-git copies
piled up at `~/peppar-fix`, `~/git/PePPAR-Fix`, and `~/PePPAR-Fix` and
nobody knew which was authoritative.

The only legitimate cross-host file copies are for things that **are
not in the repo**: `ntrip.conf` (credentials), `data/*.csv` (capture
artifacts pulled back to gt for archival), `data/position.json` and
`data/drift.json` (host-local runtime state).

### gt is the local upstream

The primary git upstream for lab hosts is the bare repo on the gt
home server at `bob@gt:git/PePPAR-Fix.git`, **not** GitHub directly.
Pushes to gt's bare are auto-mirrored to GitHub by a `post-receive`
hook (additive only — never deletes refs on GitHub even if they're
gone from the bare; see `hooks/post-receive` inside the bare for the
incident that produced this rule).

The reason for using gt as the local upstream rather than GitHub
directly:
- **Faster** — local network instead of github.com round trip.
- **Safer iteration** — push a fix from one lab host to gt, then pull
  on a *second* lab host and confirm it didn't break things there,
  *before* the change ever reaches GitHub.  Catches "fixed it on
  host A, broke host B" early.
- **Works without internet** — the lab is on a local network; gt is
  always reachable even when GitHub is not.

GitHub is still the public origin and remains the long-term home of
record.  It's just that lab hosts and gt's dev tree both push *through*
the gt bare upstream, not directly to it.

### Common lab-host failures (in order of frequency)

1. **Missing Python deps**: set up the venv first:
   `cd ~/peppar-fix && python3 -m venv venv && venv/bin/pip install pyubx2 pyserial`
   (add `smbus2` on I2C hosts). Never use `--break-system-packages`.
2. **Missing directories**: `mkdir -p ~/peppar-fix/data`
3. **Missing ntrip.conf**: `scp TimeHat:~/peppar-fix/ntrip.conf ~/peppar-fix/`
   (this is the *one* legitimate scp — credentials are not in the repo).
4. **Stale processes**: `sudo pkill -f peppar` before starting.
5. **TICC args need splitting**: `--engine-arg --ticc-port --engine-arg /dev/ticc1`
   (NOT `--engine-arg "--ticc-port /dev/ticc1"`).
6. **Timebeat must be stopped on OTC hosts**: `sudo systemctl stop timebeat`.

Always use the `peppar-fix` orchestration wrapper, not individual scripts.

## Lab Test Protocol

PePPAR Fix is implemented as component scripts that can be run directly from the CLI, but
users would be unlikely to run them individually. They are normally invoked by the orchestration
wrapper in scripts/peppar-fix. That's over 500 lines of code that should be tested whenever
possible. Always prefer testing using the wrapper as a user would, but it's ok to
run components individually for diagnosis or troubleshooting.

## Lab Hosts and Access

All lab hosts are Raspberry Pis or similar SBCs. SSH access is
passwordless for user `bob`.

| Host | Access | Role | GNSS | Notes |
|---|---|---|---|---|
| TimeHat | `ssh TimeHat` | Primary peppar-fix dev + PHC discipline | F9T-3RD on `/dev/gnss-top` | Has i226 PHC, TICC #1, heatsink on TCXO |
| PiPuss | `ssh PiPuss.local` | Dual-F9T, caster/client testing | F9T-TOP `/dev/gnss-top`, F9T-BOT `/dev/gnss-bot` | Zero-baseline (both on Patch3 via GUS #2) |
| ~~Onocoy~~ | mothballed 2026-04-08 | F10T + PX1125T disconnected; TICC #2 moved to ocxo | – | Powered down. Never had a peppar-fix checkout. |
| otcBob1 | `ssh otcBob1` | Timebeat OTC SBC, OCXO, Renesas ClockMatrix | F9T on `/dev/ttyAMA0` at 460800 | Stop `timebeat` before accessing I2C or GNSS |
| ptBoat | `ssh ptBoat` | Timebeat OTC Mini PT, weatherproof, PoE | F9T on `/dev/ttyAMA0` at 115200 | Same Renesas ClockMatrix as otcBob1 |
| ocxo | `ssh ocxo` | E810-XXVDA4T x86 host, OCXO, DPLL | F9T on `/dev/gnss0` (kernel, I2C) | PHC at `/dev/ptp1`, trusted net + PTP net |
| bbb | `ssh bbb` | BeagleBone, GPS L1 only | `/dev/gps0` at 9600 | Legacy NTP/PTP GM |

**Hostname resolution**: Try `<host>` first (DNS search domain VanValzah.Com).
If that fails, try `<host>.local` (mDNS). PiPuss only resolves via
`.local`.  Never use the PTP LAN (10.168.13.x) for SSH — keep that
clean for timing traffic.

## Serial Port Gotchas

### TICC resets on serial open

TAPR TICCs use Arduino Mega 2560. **Opening the serial port toggles DTR,
which reboots the Arduino.** The TICC goes silent for ~10 seconds during
boot, then outputs its config header before starting measurements.

The Arduino resets on the **rising edge** of DTR (via a capacitor to
RESET). When a process closes the serial port, the `cdc_acm` driver
drops DTR. When the next process opens it, DTR rises — triggering a
reboot. `dsrdtr=False` in pyserial is **not sufficient** to prevent
this; it only controls pyserial's flow control, not the kernel driver.

The fix is to clear the `HUPCL` termios flag, which tells the kernel
to leave DTR asserted when the fd closes:

```python
import termios

# RIGHT — prevents DTR drop on close, so next open won't reboot:
ser = serial.Serial("/dev/ticc1", 115200, dsrdtr=False, rtscts=False)
attrs = termios.tcgetattr(ser.fd)
attrs[2] &= ~termios.HUPCL  # cflag
termios.tcsetattr(ser.fd, termios.TCSANOW, attrs)
```

All TICC access should go through `scripts/ticc.py` which handles
this automatically via the `_SharedTiccPort` helper.

If you WANT to reset a TICC intentionally:
```python
ser = serial.Serial("/dev/ticc1", 115200, dsrdtr=True)
ser.close()
# Wait 15 seconds for boot
```

### F9T EVKs have no USB serial number

All F9T EVKs report the same VID:PID (`1546:01a9`) with no serial number.
You cannot distinguish them by USB descriptor. On PiPuss (two F9Ts),
udev uses USB path matching which breaks if cables move. On single-F9T
hosts (TimeHat), VID:PID matching is fine.

Each F9T does have a unique `SEC-UNIQID` queryable via UBX protocol,
but this is not visible to udev.

### Stable device names

Devices with unique serial numbers get stable names everywhere via
`99-timelab.rules`:

| Device | Name | Basis |
|---|---|---|
| `/dev/ticc1` | TICC #1 | Arduino serial `95037323535351803130` |
| `/dev/ticc2` | TICC #2 | Arduino serial `44236313835351B02001` |
| `/dev/ticc3` | TICC #3 | Arduino serial `44236313835351B0A091` |
| `/dev/f10t` | NEO-F10T (ArduSimple) | FTDI serial `D30GD1PE` |

Devices without unique serials (PX1125T, F9T EVKs) do NOT get udev
symlinks. Your code must identify them at runtime.

## NTRIP Credentials and Configuration

NTRIP config file: `/home/bob/peppar-fix/ntrip.conf` on TimeHat.

Credentials are in `ntrip.conf` on each lab host (not committed to the repo).
See `ntrip.conf.example` for the format.  To deploy credentials to a new host:

```bash
scp TimeHat:~/peppar-fix/ntrip.conf .
```

Caster: `ntrip.data.gnss.ga.gov.au:443` (TLS).
SSR mount: `SSRA00BKG0`.
Broadcast ephemeris mount: `BCEP00BKG0` (same caster, pass via `--eph-mount`).

**SSR status**: Orbit + clock + code bias available. **Phase bias = 0**
(not provided by this stream). This means PPP-AR is not possible with
this SSR source alone.

## Known Broken Things

### ~~BDS ISB 1500 ns~~ — closed 2026-04-19 (PR #3 + PR #5)

Default is `--systems gps,gal`.  BDS previously produced ISBs of
1500+ ns (should be <200 ns).  Two root causes identified and
fixed 2026-04-19:

1. **BDS signal-code map for RTCM 1260 was wrong** (sig_ids 2-11
   mis-labeled or missing).  F9T L5-fleet tracks BDS-3 MEO on
   B1I + B2a-I; sig_id=9 (B2a-I) was stored under the wrong RINEX
   key, so every BDS L5-band code-bias lookup MISSed.  **Fixed in
   PR #3, commit `150c495`.**
2. **BDS-3 modernized cpMes is L1-reference cycles.**  F9T firmware
   TIM 2.25 reports B2a-I carrier phase in L1-band cycles, not
   native.  GF/MW/IF downstream math assumed native cycles,
   producing ±60-190 m per-epoch GF blow-ups and 1800+ false-positive
   slips per minute.  **Fixed in PR #5, commit `12a76a3`**
   (multiply cpMes by λ_L1 / λ_native at ingest).

Validation: day0419c on MadHat + clkPoC3 reached BDS AR via LAMBDA+
rounding on C24/C35/C44 with sub-cm NL residuals matching GAL
quality.  Cross-host horizontal agreement < 1 cm.  See
`project_bds_ar_first_success_20260419.md`.

Still open (lower priority — residuals don't demand it):

- Dual-TGD handling for IF(B1I, B2a-I) on B3I-referenced broadcast
  clock.  Commit `63a4af1` decoded TGD2 (DF514) into `_BDS_MAP` but
  `_sat_clock` still applies TGD1 only.  SSR phase biases appear to
  absorb the per-SV TGD variance in practice.

Prior attribution to the 14 s BDT/GPST offset was wrong — that's
already handled correctly in `broadcast_eph.py:_bds_seconds_of_week`.
Full research + references: `docs/bds-ppp-integration.md`.

### ~~F10T on Onocoy doesn't respond to UBX~~ — Onocoy mothballed 2026-04-08

The NEO-F10T issue is parked along with the host.  When we revive F10T
work it'll be on a different host.

## peppar-fix Python Environment

| Host | Venv | Activation |
|---|---|---|
| TimeHat | `/home/bob/peppar-fix/venv` | `source ../venv/bin/activate` |
| PiPuss | `/home/bob/pygpsclient` | `source ~/pygpsclient/bin/activate` |
| ~~Onocoy~~ | mothballed 2026-04-08 | – |
| otcBob1 | (system python3) | May need `pip install smbus2` for I2C |

Scripts live in `/home/bob/peppar-fix/scripts/` on TimeHat. Other hosts
may not have the full peppar-fix repo — deploy scripts via `scp` as
needed.

## Lab Storage Warning

**Lab hosts use eMMC or SD cards.** These can fail without warning.
After any significant capture or test, pull results back to the GT
server (`/home/bob/gt/`) which has RAIDZ-3 + offsite backup. Large
datasets can stay on lab hosts; everything else should be pulled.

## Resource Allocation

Lab hardware is shared. Before using a host or device, check that no
other work is running on it:

```bash
# Check for running processes using a serial port
ssh <host> "fuser /dev/gnss-top 2>/dev/null"

# Check for running peppar-fix or analysis processes
ssh <host> "ps aux | grep -E 'peppar|servo|ticc|analyze' | grep -v grep"
```

Beads that need hardware carry `hw:` labels (e.g. `hw:TimeHat`,
`hw:F9T-3RD`). The Mayor checks these before assigning work. If your
bead has a hardware label, you are the exclusive user of that hardware
for the duration of your work.

## Key Technical Context

### Terminology — DO vs PHC, rx TCXO vs bare TCXO

**Read `docs/glossary.md`** for definitions of all domain terms.

Critical naming rules:

- **DO** (Disciplined Oscillator): the crystal being steered.  Use
  "DO" in any context that isn't PHC-specific.
- **PHC**: use only when referring to the Linux PTP Hardware Clock
  API (`adjfine`, `clock_settime`, `EXTTS`).  Not all DOs are PHCs.
- **rx TCXO**: the TCXO inside the GNSS receiver (F9T).  Never use
  bare "TCXO" — it's ambiguous with the DO's crystal on i226 hosts.
- **gnss_pps** / **do_pps**: the two PPS streams.  PPS error =
  gnss_pps − do_pps (positive = DO is late).

### Stream correlation via CLOCK_MONOTONIC — read this first

**Read `docs/stream-timescale-correlation.md` before modifying any
code that matches data from different streams** (qErr, TICC, EXTTS,
PPP, NTRIP).

Every data stream operates on its own timescale.  The TICC makes
this obvious (seconds since boot), but even PHC timestamps have no
guaranteed relationship to GPS or UTC unless we establish it by
measurement.  A PPS edge is just a voltage transition — it's only
correlated with a GNSS epoch because **we** correlate it.

The **only** shared timescale is `CLOCK_MONOTONIC`.  Whether it's
an EXTTS read through the PTP driver, a TICC timestamp read through
serial port X, or a qErr message read through serial port Y, we
have a timestamp on the read against `CLOCK_MONOTONIC`.  From that,
we must correlate everything.  There is no other reliable way given
queueing, CPU scheduling, and network delays.

**Critical rules for qErr correction of TICC measurements**:

- **Sign**: `corrected = ticc_diff_ns + qerr_ns` (plus, not minus)
- **Matching**: qerr must correspond to the **same PPS edge** the
  TICC measured.  "Same PPS edge" = their `CLOCK_MONOTONIC` read
  timestamps match the expected timing relationship.  Match using
  `ticc_measurement.recv_mono` (expected_offset ≈ 0.95s), NOT the
  EXTTS `pps_event.recv_mono`.  Off-by-one edge makes TDEV **worse**
  than raw PPS (3.3 ns vs 2.1 ns — confirmed 2026-04-11).
- **qVIR (qErr Variance Improvement Ratio) `Δvar(raw)/Δvar(raw+qerr)`
  must be > 1.5.  If ≤ 1.0, the correlation is broken — stop using
  qerr immediately.  Do not discover this after an overnight run.

This applies to **all** time-based correlation between streams with
independent timescales, not just qErr.

### Position finding

The PPP position finder (`peppar_find_position.py` or Phase 1 of
`peppar_fix_cmd.py`) converges in ~90 seconds at sigma 0.5m with
GPS+GAL and NTRIP broadcast ephemeris. The convergence requires:
- Per-system ephemeris warmup (all configured systems must have ≥8 SVs)
- Satellite health filtering (excludes unhealthy Galileo E14/E18)
- Satellite clock sanity check (|sat_clk| < 2ms)
- LS outlier rejection (>50m residuals excluded)

### Unified CLI

`peppar_fix_cmd.py` is the unified entry point:
- Phase 1: PPPFilter bootstrap (estimates position from scratch)
- Phase 2: FixedPosFilter (clock estimation with optional servo)
- `--servo /dev/ptp0 --pps-pin 1` enables PHC discipline
- `--position-file` skips Phase 1 if the file exists and is valid
- Position file is validated against live LS fix before Phase 2

### Free-running TCXO baseline

TimeHAT v5 TCXO with heatsink: TDEV(1s) = 1.17 ns as measured by
TICC (60 ps resolution, 2h capture, 0.2% reproducibility).
F9T PPS TDEV(1s) = 2.3 ns (2h baseline, varies 1.0-1.4 ns on 30 min
windows depending on sawtooth phase).

### EXTTS TDEV measurements are unreliable — use TICC

**Both i226 and E810 EXTTS have ~8 ns effective resolution.**  This
matches the F9T's 125 MHz clock period.  EXTTS TDEV measurements
underreport true timing noise because the quantization masks real
PPS jitter:

- **E810 EXTTS**: 77% identical adjacent timestamps.  Reports falsely
  low TDEV (0.34 ns for a signal that's actually 2.3 ns).  The sub-ns
  timestamp format does not reflect sub-ns measurement resolution for
  GPIO/SMA events.
- **i226 EXTTS**: adds ~2.9 ns RSS noise but at least tracks PPS
  movement (0% identical adjacent).  Still underreports at short tau.

**Never report TDEV from EXTTS alone.**  EXTTS-only TDEV makes results
look better than they are.  Always use TICC (60 ps resolution) for
TDEV characterization.  EXTTS data may be shown alongside TICC for
contrast — shade the area between TICC and EXTTS to reveal the
measurement error:
- Area between TICC and E810 EXTTS = "actual TDEV unreported by E810"
- Area between TICC and i226 EXTTS = "i226 measurement noise"

See `docs/ticc-baseline-2026-04-01.md` for the full analysis and
`docs/visual-stories.md` for plot specifications.

### TICC stability metric: use chA alone, not chA-chB

When characterizing servo output stability (TDEV/ADEV), use **TICC chA
alone** (the DO's PPS, detrended).  This measures the absolute phase
stability of the disciplined oscillator — what a downstream consumer
actually sees.

**Do not use chA-chB** (DO PPS minus GNSS PPS) for this purpose.
chA-chB measures how well the DO *tracks GPS* — i.e., the work the
servo did — not the quality of the output.  A perfect servo tracking a
noisy GPS reference would show low chA-chB but high chA: the output
inherited the reference noise.  Conversely, a servo that drifts slowly
from GPS might show large chA-chB while chA stays smooth.

Use chA-chB only when the question is "how faithfully does the DO
follow GPS?" — e.g., diagnosing servo gain or loop bandwidth.

## Design Documentation

The `docs/` directory contains design documents and research notes. Start
here before changing anything in the areas they cover.

| File | Summary |
|---|---|
| [stream-timescale-correlation.md](docs/stream-timescale-correlation.md) | **Read this first.** How to correctly correlate events from independent timescales (GNSS, PPS, TICC, NTRIP). Covers why queue-order matching fails, the strict correlation gate design, confidence scoring, and fault injection testing. |
| [full-data-flow.md](docs/full-data-flow.md) | Complete inventory of live data sources, their timescales, sink policies (freshest-only vs loss-free vs correlated-window), freshness requirements, and decimation effects. |
| [platform-support.md](docs/platform-support.md) | Per-platform status for TimeHat (i226) and ocxo (E810). Documents device paths, PHC behavior, GNSS transport differences, and bring-up checklists. |
| [time-and-platform-todo.md](docs/time-and-platform-todo.md) | Concrete work breakdown: remaining tasks for E810 GNSS, TimeHat PPS, correlation model, legacy cleanup, diagnostics. |
| [timebeat-otc-research.md](docs/timebeat-otc-research.md) | Early ClockMatrix research (some addresses wrong). See register-map doc for confirmed addresses. |
| [timebeat-otc-signal-routing.md](docs/timebeat-otc-signal-routing.md) | Signal flow sketch (DPLL mapping outdated — see register-map doc for confirmed configs). |
| [timebeat-otc-register-map.md](docs/timebeat-otc-register-map.md) | **Authoritative** 8A34002 register map: correct I2C addressing, DPLL/status/TDC registers, confirmed on both hosts. |
| [timebeat-integration-paths.md](docs/timebeat-integration-paths.md) | Integration plan: ptBoat (easy, PHC-only or write_freq) vs otcBob1 (complex, live TDC). Runtime MODE writes confirmed working. |
| [data-flow.md](docs/data-flow.md) | Original data flow sketch (superseded by full-data-flow.md for sink policy details). |
| [position-convergence.md](docs/position-convergence.md) | PPP position bootstrap convergence analysis and tuning. |
| [nic-survey.md](docs/nic-survey.md) | Survey of NICs with PTP hardware timestamping support. |
| [e810-cm5-research.md](docs/e810-cm5-research.md) | E810 on Raspberry Pi CM5: showstopper (ice driver x86-only). |
| [phc-bootstrap.md](docs/phc-bootstrap.md) | PHC bootstrap design: cold/warm start, optimal stopping, glide slope, characterization method, drift file, how the servo starts with bounded error. |
| [pps-ppp-error-source.md](docs/pps-ppp-error-source.md) | PPS+PPP servo error source: using carrier-phase dt_rx to replace TIM-TP qErr via 125 MHz tick model. Experiment results, calibration procedure, formula. |
| [correction-sources.md](docs/correction-sources.md) | How to get SSR corrections: registration, caster options, which streams for float PPP vs PPP-AR, why AR requires a single analysis center. |
| [ssr-mount-survey.md](docs/ssr-mount-survey.md) | F9T-focused survey of SSR mounts for PPP-AR.  Ranks WHU OSBC00WHU1, MADOCA-PPP, Galileo HAS, CAS phase-2, CNES by F9T-L5Q suitability.  Corrects our earlier "L5I bias can't be used on L5Q" premise. |
| [galileo-has-research.md](docs/galileo-has-research.md) | Galileo HAS: free PPP-AR corrections via E6-B signal. |
| [peer-bootstrap-sketch.md](docs/peer-bootstrap-sketch.md) | NTRIP caster mode for peer-to-peer bootstrap. |
| [caster-ephemeris.md](docs/caster-ephemeris.md) | Spec + implementation plan: encode F9T RXM-SFRBX as RTCM 1019/1042/1046 so the local caster serves broadcast ephemeris alongside observations. Removes external NTRIP dependency for peer bootstrap. |
| [ntrip-mdns-discovery.md](docs/ntrip-mdns-discovery.md) | Spec: mDNS service advertisement for NTRIP peer discovery. Caster announces `_ntrip._tcp`, client discovers and selects by accuracy/proximity. |
| [ticc-calibration-2026-03-19.md](docs/ticc-calibration-2026-03-19.md) | TICC calibration procedure and results. |
| [hw-labels.md](docs/hw-labels.md) | Hardware labeling conventions for beads. |
| [draft-dupage-inquiry.md](docs/draft-dupage-inquiry.md) | Draft inquiry to DuPage County about GNSS antenna siting. |
| [packaging-plan.md](docs/packaging-plan.md) | Plan for making peppar-fix pip-installable from GitHub Releases. Phased: pyproject.toml stub (done), flatten imports, versioned releases. |
| [ptp4l-supervision.md](docs/ptp4l-supervision.md) | Layered ptp4l clockClass supervision via systemd. Three layers: engine (Python UDS), wrapper (pmc command), systemd ExecStopPost. Covers clock-class mapping, ptp4l config, privilege model, and example unit file in `deploy/`. |
| [extts-lifecycle.md](docs/extts-lifecycle.md) | EXTTS (PPS IN/OUT) initialization lifecycle. Bootstrap owns pin programming; engine inherits and verifies. Covers PTP profile extension for IN+OUT pins, PEROUT for TICC, fd persistence, platform matrix (i226/E810), and phased migration path. |
| [i226-perout-500ms-bug.md](docs/i226-perout-500ms-bug.md) | **Read before debugging PEROUT.** i226 PEROUT can fire at 500ms phase — hardware issue, confirmed with SatPulse. Some boards always land wrong regardless of software. Detection, workaround, affected hosts. |
| [qerr-correlation.md](docs/qerr-correlation.md) | **Read before modifying qErr matching.** How qErr is matched to TICC timestamps, why TICC qVIR is the definitive check (no DO noise, no servo), why chA-chB diff qVIR is wrong, the TIM-TP-initiated window matching design, queue_remains principle. |
| [glossary.md](docs/glossary.md) | **Terminology reference.** DO vs PHC, rx TCXO vs bare TCXO, gnss_pps vs do_pps, all acronyms (EKF, LQR, TDEV, ADEV, TICC, PPP, SSR, etc.). |
| [wr-gm-research.md](docs/wr-gm-research.md) | White Rabbit GM architecture review: softpll internals (helper/main/external PLLs), how GM uses PPS vs 10 MHz, qErr injection points, PEROUT at 10 MHz, two integration paths (PHC PEROUT vs OCXO+ClockMatrix). |
| [ticc-baseline-2026-04-01.md](docs/ticc-baseline-2026-04-01.md) | F9T PPS baseline TDEV(1s)=2.3 ns (2h runs); i226 TCXO PEROUT TDEV(1s)=1.170 ns (0.2% spread); servo bandwidth implications; EXTTS quantization analysis. |
| [ppp-ar-design.md](docs/ppp-ar-design.md) | Design for PPP-AR: phase bias sources, filter changes, ambiguity resolution algorithm, 4-phase implementation plan, 5 validation tests. |
| [ppp-ar-filter-redesign.md](docs/ppp-ar-filter-redesign.md) | **Read after ppp-ar-design.md.** Why IF ambiguities are not integer, WL/NL decomposition, Melbourne-Wubbena tracker + narrow-lane resolver design, ~210 lines total. Supersedes Phase B/C integrality approach. |
| [ztd-state-for-ppp-ar.md](docs/ztd-state-for-ppp-ar.md) | **Next step for AR.** PPPFilter needs a ZTD state to separate atmospheric drift from position. Without it, NL fixes lock in tropospheric bias — cross-host agreement stuck at ~5m horizontal despite correct integers. |
| [clockmatrix-bootstrap-plan.md](docs/clockmatrix-bootstrap-plan.md) | ClockMatrix supplements PHC: bootstrap sequence, FCW handoff, hybrid architecture for Timebeat OTC. |
| [igc-kernel-patches.md](docs/igc-kernel-patches.md) | **Read before driver work on i226 hosts.** Inventory of igc patches (ppsfix + adjfine), per-host deployment status, verification checklist, incident history. v3 adjfine patch reduces but does not eliminate TX timeout cascade — EXTTS wedges after ~44 min MTBF. |
| [lambda-ar-plan.md](docs/lambda-ar-plan.md) | LAMBDA integer least-squares AR: decorrelation, search, ratio test, partial AR. Replaces per-satellite rounding — handles ZTD-ambiguity correlation, faster TTFF, stronger validation. ~150 lines. |
| [sv-lifecycle-and-pfr-split.md](docs/sv-lifecycle-and-pfr-split.md) | **Per-SV state machine** (TRACKING → FLOAT → WL_FIXED → NL_SHORT_FIXED → NL_LONG_FIXED, plus SQUELCHED) orthogonal to `AntPosEstState`.  Defines the **fix set** (short-term + long-term members) and splits the old PFR monitor into three stateless per-eval monitors: **FalseFixMonitor** (wrong-integer detection on short-term members), **SettingSvDropMonitor** (elev-weighted graceful drop), and **FixSetIntegrityAlarm** (systemic failure catcher).  Replaces the PFR L1/L2/L3 cascade; motivated by day0419d ~2 PFR L3/hr observations caused by a level-persistence bug. |
| [lab-operations.md](docs/lab-operations.md) | **Read before running on any lab host.** Deployment procedure, pre-flight checklist, stumble analysis, standard host layout, future automation work. |
| [position-bootstrap-reliability-plan.md](docs/position-bootstrap-reliability-plan.md) | Implementation plan to make cold-start Phase-1 trustworthy so lab runs no longer need `--known-pos`. Residual-consistency gate, NAV2 cross-check, harder abort, σ tightening, state persistence. 6 work items, W1–W5 core, W6 fallback. |
| [l5i-l5q-phase-bias-empirical.md](docs/l5i-l5q-phase-bias-empirical.md) | Empirical proof (19 GPS SVs, lab day0418h) that CNES L5I and WHU L5Q phase biases differ by mean −0.73 m with 1.46 m SD — far beyond the λ/4 carrier-phase offset.  RTCM "phase bias" bundles satellite-side delays + AC-datum offsets + reference-receiver I/Q group delays; can't be substituted with a single-cycle correction.  Supports the dual-mount fusion path. |
| [bds-ppp-integration.md](docs/bds-ppp-integration.md) | Research notes on BDS PPP integration: the 1500 ns ISB isn't the 14 s BDT/GPST offset (already correct).  Real causes are (1) wrong RTCM 1260 signal-code map — fixed in PR #3, and (2) BDS-3 broadcast clock is B3I-referenced so IF(B1I,B2a-I) needs TGD1+TGD2 not TGD1 alone — still open.  Includes the complete RTCM 3.3 BDS sig-id table and PRIDE/RTKLIB references. |

## Lab Documentation Pointers

The `timelab/` directory at the town root has authoritative lab state:

| File | Contents |
|---|---|
| `timelab/topology.md` | Current wiring: antenna→splitter→receiver→host→TICC, PTP domains, NTP |
| `timelab/gear.md` | Hardware inventory: every host, receiver, TICC, antenna, with specs |
| `timelab/usb-identification.md` | USB serial numbers, udev policy, device identification |
| `timelab/status.md` | Active experiments and recent results (may be stale) |
| `timelab/calibration.md` | TICC calibration procedures |
| `timelab/99-timelab.rules` | Universal udev rules (deployed to all Pis) |
| `timelab/scripts/` | Lab utility scripts (ticc_read.py, calibration_capture.py, etc.) |
| `timelab/antPos.json` | Surveyed antenna positions (UFO1 and any others).  Gitignored at both ends — coords stay out of the public PePPAR-Fix repo.  Read this when you need the lab ARP (e.g., to seed `--known-pos` for a dev/test run, or to validate a cold-start convergence against a reference). |

If you need to know what's physically connected where, start with
`topology.md`. If you need device specs, start with `gear.md`.
Operational positions are **not** hardcoded in PePPAR-Fix — they're
loaded from `state/receivers/<uid>.json` at runtime (written by
Phase 1 bootstrap) and verified against `timelab/antPos.json`.

## Acceptance Criteria for Beads

Your bead is NOT done until:
- Code changes are committed to your feature branch
- A test plan is documented in a bead comment
- If the bead has `hw:` labels: code stays on branch (`--no-merge`),
  lab validation happens separately
- If pure software: include unit tests or a verification script
- Research tasks MUST produce a markdown document in `docs/`

Do not close a bead with "Completed with no code changes" unless you
can explain in a comment exactly what you verified and why no changes
were needed.
