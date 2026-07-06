# Running PePPAR-Fix on SparkPNT SXT-D (GNSSDO+) hardware

Investigation + integration plan for using the SparkPNT **SXT-D
(GNSSDO+)** as a PePPAR-Fix clock — i.e. making its 10 MHz OCXO the
PePPAR-Fix **DO**, disciplined by *our* servo instead of SparkFun's
built-in loop.

Firmware source: <https://github.com/sparkfun/SparkFun_GNSSDO>
(analyzed at repo HEAD, firmware v3.x, ESP32 core 3.0.7).

Companion deliverable: [`support/gnssdo-plus/0001-external-oscillator-control.patch`](../support/gnssdo-plus/0001-external-oscillator-control.patch)
— a minimal, opt-in firmware change (one new file + small edits) that
adds a host-driven discipline mode with a fail-safe watchdog. Intended to
be upstreamed to SparkFun as a PR.

> **Status: CI-built and hardware-validated (2026-07-06).** Built by
> SparkFun's own CI on the fork `bobvan/SparkFun_GNSSDO@external-oscillator-control`,
> flashed to the MadHat SXT-D over front-panel USB, and every command
> (`$R`/`$T`/`$E`/`$W`) plus the watchdog auto-revert confirmed working —
> see §7. The current validated binary and the pre-flash stock backup live
> at `~/gt/firmware/gnssdo-plus/` on gt.

---

## 1. Hardware architecture — where the DO lives

The SXT / SXT-D is a **two-chip** board:

- **mosaic-T** (Septentrio) — the GNSS receiver. USB-C, Ethernet
  (incl. PoE), multiple UARTs. It uses the **10 MHz oscillator as its
  external frequency reference** and generates PPS OUT.
- **ESP32-WROVER** — "rides shotgun." Runs the SparkFun firmware: reads
  the mosaic-T's receiver clock bias over SBF and **steers the
  oscillator over I²C**, and drives the OLED.

| Product | Oscillator (the DO) | Driver | I²C addr |
|---|---|---|---|
| SXT (GNSSDO) | SiT5358 **DCTCXO** | `GNSSDO_SIT5358` | — |
| **SXT-D (GNSSDO+)** | **STP3593LF double-oven OCXO** | `GNSSDO_STP3593LF` | `0x70` |
| (alt build) | SiT5811 OCXO | `GNSSDO_SIT5811` | — |

**The oscillator is the disciplined oscillator; the ESP32 is the
servo.** For PePPAR-Fix we want to *be* the servo: consume the
mosaic-T's phase/frequency error and write the oscillator control word
ourselves.

### The stock discipline loop (`States.ino`)

1. ESP32 configures mosaic-T COM1 to emit SBF `PVTGeodetic+ReceiverTime`
   and `FugroTimeOffset` (`GNSS.ino`).
2. `PVTGeodetic` (block 4007) → `RxClkBias` → `gnssClockBias_ms`
   (`Tasks.ino`). **This is the servo error.**
3. State machine walks `FINETIME → FREQUENCY_LOCK → PHASE_LOCK`, calling
   `updateTCXO(bias,P,I)` → `myTCXO->setFrequencyByBiasMillis()` →
   `setFrequencyControlWord()`.

The **STP3593LF control word** (SXT-D) is 20-bit unsigned,
**LSB = 8e-13** (0.8 ppt fractional frequency), so full range ≈ ±0.42 ppm
of pull. `setFrequencyControlWord()` is an absolute write — the clean
primitive for an external servo. (SiT5811: 39-bit signed. SiT5358:
26-bit signed.)

---

## 2. The "TCP backdoor" — real, but read-only in stock firmware

The README's "the ESP32 console can be accessed via TCP" is exactly a
back door through the mosaic-T:

- The ESP32 console UART is daisy-chained to **mosaic-T COM3**.
- With `enableTCPServer` on, the ESP32 issues
  `IPServerSettings, IPS1, <port>, TCP2Way` to the mosaic-T
  (default **port 28785**). A TCP socket to the mosaic-T's IP:28785 lands
  on the ESP32 console, fully bidirectional, over Ethernet/PoE — no extra
  wiring. (Same box you already reach for SBF; cmd port 28784 —
  see the Mosaic-T SBF lab-test note in memory.)

**But the stock console cannot do either thing we need** (every menu
handler was read):

- **No command disables discipline.** No hold/manual/coast toggle. Every
  live state actuates; the only way actuation stops is a mosaic-reported
  error — which also drops PPS.
- **No command writes the control word.** `setFrequencyControlWord()`
  exists in the driver but *nothing in the console calls it*; only
  `getFrequencyControlWord()` is exposed (as CSV telemetry). Menu item 60
  only tells the oscillator to persist its *current* word.

What the backdoor gives us **for free** (zero code change): a CSV
telemetry stream (`printCurrentConditions`) with epoch, `RxClkBias`,
drift, **current control word**, state, temperatures. Useful for
monitoring; read-only.

**Conclusion: host-driven discipline is impossible without ESP32
cooperation** — the oscillator I²C bus is ESP32-only; the mosaic-T has
no path to it. Both needed features require a (small) firmware change.

---

## 3. The firmware change (the patch)

Design goals: opt-in, a pure superset of stock behavior (easy to rebase
on SparkFun releases and to upstream), and **fail-safe by default**.

### Command channel

A new line-based command channel on the *existing* console, so it works
over USB **and** the mosaic-T TCP backdoor. `updateSerial()` routes any
line beginning with `$` to the new parser (`ExternalControl.ino`);
**every other keystroke still opens the interactive menu, unchanged.**

| Command | Meaning |
|---|---|
| `$E,1` / `$E,0` | Enable / disable external control |
| `$W,<word>` | Write the oscillator control word (absolute). Implicitly enables external control and re-arms the watchdog. **This is the discipline command.** |
| `$T,<seconds>` | Set the watchdog timeout (1–600 s; persisted) |
| `$R` | Report status |

Replies are one line, echoing the command letter:

```
$W,OK,<word>                              control word written; read back
$E,OK,<0|1>                               external-control state
$T,OK,<seconds>
$R,<ext>,<word>,<bias_s>,<state>,<watchdog_s>,<since_ms>
$<letter>,ERR[,<reason>]                  rejected
```

`$R` returns the current control word and clock bias so PePPAR-Fix can
**seed its own servo from the oscillator's current state before taking
over** (no step at hand-off).

### What "external control" does

- Enters a new **`STATE_GNSS_EXTERNAL_CONTROL`**. In that state the
  internal loop **never calls `updateTCXO()`** — plus a hard interlock
  `if (externalControl) return;` at the top of `updateTCXO()` backs it up
  (belt-and-suspenders).
- **PPS keeps running** and clock-bias telemetry stays fresh (so `$R` and
  the CSV remain live while PePPAR-Fix disciplines).
- Raw control-word writes bypass the stock PI integrator and its 3 ppb/step
  clamp — **full loop authority** for PePPAR-Fix, which does its own rate
  limiting.

### Fail-safe watchdog (the requested feature)

Once the loop is open, a watchdog (default **15 s**, `$T`/menu-settable,
persisted) demands a discipline command at least that often. **Every
`$W` (and `$E,1`) re-arms it.** If the host goes quiet:

1. The firmware logs `External control watchdog expired`.
2. It **resets the servo integrator** and resumes internal discipline
   **from the oscillator's current control word** — the external host may
   have steered far, so we must not slam back to a stale integral.
3. PPS is left running throughout.

Two independent safety nets, so a dead PePPAR-Fix host can never leave
the oscillator un-serviced:

- **Watchdog** — quiet host → internal discipline resumes in ≤ timeout.
- **Non-persistence** — `externalControl` is runtime-only; a power cycle
  always boots into internal discipline.

### The one non-obvious code change

The STP3593LF (the SXT-D oscillator) kept its PI(D) integrator in
**function-static** locals, so a driver rebuild would *not* reset it —
and a stale integrator would cause a frequency step on hand-back. The
patch moves that state into the driver **object**
(`_integrator`/`_integratorInitialized`/`_previousChangeInLSBs`), so
`resetTcxoIntegrator()` (rebuild the driver) cleanly reseeds from the
current control word for **every** oscillator type. This is also a
legitimate correctness fix for their code (reentrant / multi-instance
safe).

### Files touched

| File | Change |
|---|---|
| `ExternalControl.ino` | **new** — command parser + line reader |
| `States.ino` | watchdog check, `STATE_GNSS_EXTERNAL_CONTROL` case, `begin/endExternalControl()` |
| `Begin.ino` | interlock in `updateTCXO()`, `resetTcxoIntegrator()` |
| `settings.h` | new state + name, runtime globals, `externalControlWatchdog_s` |
| `NVM.ino` | persist/load the watchdog setting |
| `menuMain.ino` | route `$` lines to the parser |
| `menuSystem.ino` | Operation menu item 52 (watchdog timeout) |
| `STP3593LF_OCXO.{h,ino}` | integrator: function-statics → object members |

---

## 4. Build toolchain

The build is fully Dockerized and reproducible (`Firmware/Dockerfile`,
driven by `.github/workflows/build-for-release.yml`).

**Recommended — reproduce CI exactly** (`apt install docker.io`):

```sh
git clone https://github.com/sparkfun/SparkFun_GNSSDO
cd SparkFun_GNSSDO
git am < .../support/gnssdo-plus/0001-external-oscillator-control.patch
cd Firmware
docker build -t gnssdo_fw --no-cache \
  --build-arg CORE_VERSION=3.0.7 \
  --build-arg FIRMWARE_VERSION_MAJOR=9 --build-arg FIRMWARE_VERSION_MINOR=9 \
  --build-arg DEBUG_LEVEL=none .
docker create --name gi gnssdo_fw:latest
docker cp gi:/GNSSDO_Firmware.ino.bin .
docker container rm gi
```

Everything else is pinned inside the Dockerfile. The container just runs
`arduino-cli`; if building natively instead you need:

- `arduino-cli` (nightly; via curl, not apt)
- ESP32 Arduino core **3.0.7** (`esp32:esp32:esp32`)
- `python3` + `pyserial` (`apt install python3 python3-pip`; esptool needs it)
- libraries (all `arduino-cli lib install`): ESP32Time 2.0.0,
  JC_Button 2.1.2, SparkFun Qwiic OLED 1.0.13, SparkFun SiT5358 DCTCXO
  1.0.1, **SiT5811 OCXO 1.0.1**, **STP3593LF OCXO 1.0.2**, PHT MS8607
  1.0.5, SparkFun Toolkit 0.9.2
- custom partition table `Firmware/app3M_fat9M_16MB.csv` (copied over the
  core default)

> **Not yet compiled.** These changes were written against a static read
> of the source; there is no ESP32 toolchain in the authoring
> environment. First step in the lab: run the docker build above and fix
> any compile nits before flashing.

### Flashing

ESP32 programs over UART0 via esptool (`Firmware/Utils/`, or the
[SparkFun RTK Firmware Uploader](https://github.com/sparkfun/SparkFun_RTK_Firmware_Uploader)).
**The ESP32 USB port is on the front of the case** (confirmed) — no need
to open the enclosure to reflash. There is no OTA path today; each
iteration is a front-USB flash.

---

## 5. How PePPAR-Fix uses it

The GNSSDO+ becomes a PePPAR-Fix host where:

- **Error signal** = the mosaic-T's own SBF, consumed **directly** by
  PePPAR-Fix (not through the ESP32). We already ingest mosaic-T SBF
  (`--obs-sbf-tcp`; see the Mosaic-T SBF lab-test procedure in memory).
  PePPAR-Fix runs its own servo / TD-CP on that stream.
- **Actuator** = `$W,<word>` over the backdoor at the servo cadence
  (≈ 1 Hz). PePPAR-Fix converts its desired fractional-frequency
  correction to a control-word delta (STP3593LF: 1 LSB = 8e-13) and
  writes the absolute word.
- **Output** = the mosaic-T's 10 MHz + PPS OUT, which are locked to the
  OCXO PePPAR-Fix is steering — so the disciplined output *is*
  PePPAR-Fix's.

Recommended bring-up sequence:

1. Enable the TCP server (Operation menu item 50) and set the port.
2. Let the unit reach `FINETIME`/lock on its own (poll `$R`; watch the
   state field and that bias is small).
3. Read `$R` → seed the PePPAR-Fix servo from the reported control word.
4. Begin streaming `$W,<word>` (first write opens the loop). Keep the
   cadence under `externalControlWatchdog_s`.
5. To hand back: `$E,0`, or just stop — the watchdog resumes internal
   discipline within the timeout.

**DO classification.** The STP3593LF is a double-oven OCXO, so this host
is an **OCXO-class sync target** (per CLAUDE.md), not best-effort — it's
in scope for the 1 ns / 2 ns cross-host excursion bounds once
characterized. (Same OCXO family as the GNSSDO+ / AtomiChron combo
already in the lab.)

---

## 6. Open items / risks

- ~~**Compile the patch**~~ — done: CI-built and hardware-validated (§7).
- **Not yet driven by PePPAR-Fix.** The command channel is validated
  standalone; wiring PePPAR-Fix's servo output to `$W` and grading against
  a TICC is the next step.
- **Servo input choice.** `RxClkBias` (block 4007) is the mosaic-T's
  composite receiver clock bias — usable, but PePPAR-Fix will likely do
  better running TD-CP on the mosaic-T MeasEpoch directly (same argument
  as every other host). The firmware change is agnostic to which error
  signal we use; it only provides the actuator + interlock + fail-safe.
- **Control-word range.** PePPAR-Fix must clamp to the oscillator's range
  (STP3593LF: 0…1048575). The firmware casts int64→uint32; out-of-range
  writes are the host's responsibility.
- **PPS timing.** PPS OUT is generated by the mosaic-T from the disciplined
  10 MHz; confirm the PPS parameters (Operation menu 40–45) match what the
  measurement chain (TICC) expects.
- **Upstreaming.** The change is a clean superset; worth offering to
  SparkFun as a PR (external-control mode + the STP3593LF integrator fix +
  the CR/LF console-hardening fix).

---

## 7. Flash + smoke-test checklist (validated on MadHat 2026-07-06)

The SXT-D's **ESP32 front-panel USB** enumerates as a CH340 serial port.
On MadHat it is `/dev/ttyUSB0` (`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
— note the `ttyACM0` on MadHat is a TICC, not this). Everything below runs
from MadHat.

### One-time tooling

The Debian `esptool` package (4.7.0) ships **without its stub-flasher data
files** — `read_flash`/`write_flash` fail. Use a venv instead:

```sh
python3 -m venv ~/esptool-venv
~/esptool-venv/bin/pip install esptool          # v5.x, stub included
E=~/esptool-venv/bin/esptool
```

### Flash (app-only image + otadata clear)

The CI artifact `GNSSDO_Firmware.ino.bin` is the **application** image
(not a merged full-flash). Write it to `app0` (offset **0x10000**) and
erase `otadata` so the bootloader comes up on it regardless of the
current OTA slot. **Back up first** — the whole thing is reversible:

```sh
# 0) Full backup (16 MB, ~7 min @ 460800) -> keep on gt
$E --chip esp32 -p /dev/ttyUSB0 -b 460800 read_flash 0x0 0x1000000 stock-backup.bin

# 1) Flash the app
$E --chip esp32 -p /dev/ttyUSB0 -b 460800 --after no_reset \
   write_flash 0x10000 GNSSDO_Firmware.ino.bin      # "Hash of data verified."

# 2) Boot the freshly written slot
$E --chip esp32 -p /dev/ttyUSB0 -b 460800 --after hard_reset erase-region 0xe000 0x2000
```

Partition map (`Firmware/app3M_fat9M_16MB.csv`): `nvs@0x9000`,
`otadata@0xe000`, `app0@0x10000`, `app1@0x650000`, `spiffs@0xc90000`. The
flash above leaves NVS and the LittleFS settings (`spiffs`) intact, and
the previous firmware survives in `app1` as a rollback slot.

**Revert to stock:** `$E --chip esp32 -p /dev/ttyUSB0 write_flash 0x0 stock-backup.bin`
(the pre-flash backup is archived at
`~/gt/firmware/gnssdo-plus/gnssdo-stock-fullflash-backup-madhat-20260706.bin`).

### Smoke test (USB console, 115200 8N1)

With `enableTCPServer` off (factory default) the console is on the USB
port. Machine commands are line-oriented; **CR, LF, or CRLF all work**
(the hardened firmware ignores stray terminators). pyserial ships in the
esptool venv. A verified sequence:

| Send | Expect | Checks |
|---|---|---|
| `$R` | `$R,0,<word>,<bias_s>,<state>,15,0` | channel alive; telemetry |
| `$T,5` | `$T,OK,5` | watchdog settable |
| `$E,1` | `$E,OK,1`; then `$R` shows `…,EXTERNAL_CONTROL,…` and `ext=1` | internal discipline suspended |
| `$W,<word±small>` | `$W,OK,<word>` (reads back the written value) | **real OCXO actuation** |
| *(stay quiet > watchdog)* → `$R` | `ext=0`, state back to `FINETIME`/`PHASE_LOCK` | **fail-safe auto-revert** |
| `$T,15` | `$T,OK,15` | restore default watchdog |

`$R` fields: `ext, control_word, bias_s, state, watchdog_s, ms_since_last_cmd`.
`$W`/`$E,1` require the oscillator to be detected (`online.tcxo`); on a
half-powered board they return `…,ERR,NO_OSC`.

> Observed on MadHat: `$E,1` → `EXTERNAL_CONTROL`; `$W,489600` read back
> `489600`; after 8 s of silence with a 5 s watchdog, `$R` returned
> `ext=0 … PHASE_LOCK` — internal discipline resumed cleanly (integrator
> reset, no blow-up). The unit is currently running this firmware; the
> stock image is backed up on gt.

---

## 8. Driving `$W` from the PePPAR-Fix servo + grading vs TICC

### Software (done)

`$W` is now a first-class engine actuator:
`scripts/peppar_fix/gnssdo_actuator.py` implements `FrequencyActuator`
(ppb ↔ control-word, `$E`/`$W`/`$T`/`$R`, serial or TCP transport,
watchdog re-arm), unit-tested in `test_gnssdo_actuator.py` (8 tests).
Wired into engine actuator selection
(`_setup_servo`, `--gnssdo-transport …`), the TOML `_MAP`, the wrapper
`KEYS`, and `config/madhat-sxtd.toml`. The servo calls
`adjust_frequency_ppb()` each `DisciplineScheduler` tick (1–120 s), which
also re-arms the firmware watchdog. Nothing about the servo/EKF changed —
the new actuator drops into the existing chain (DAC > ClockMatrix >
**gnssdo** > PHC).

### Prerequisites for a *graded* run (need decisions / lab work)

1. **Error source.** The engine's phase error comes from a `PhaseSource`
   (TICC / EXTINT / ClockMatrix TDC) or, preferably here, the mosaic-T's
   own carrier phase via the **SBF obs seam** (PRs #287–#290:
   `sbf_obs.py` / `sbf_obs_source.py`). That code is on **`main`, not yet
   on `delta/main`** — a graded SBF run must be based on a branch that has
   it. Without SBF, a first-light demo can use a TICC as the phase
   reference (SXT-D PPS OUT vs a GNSS reference PPS), but a pure-TICC
   servo with no GNSS obs is not a validated engine mode — confirm before
   relying on it.
2. **Calibrate `gnssdo_ppb_per_controlword`.** The magnitude is the
   STP3593LF digital constant (8e-4 ppb/LSB) but the **sign is
   unconfirmed**. `tools/calibrate_do.py` already drives any
   `FrequencyActuator` against a TICC — sweep control-word setpoints,
   fit `measured_ppb = gain·commanded`, and copy the slope into the TOML.
3. **Physical wiring / resource conflict.** Grading uses **chA = SXT-D
   PPS OUT, chB = reference PPS** on one TICC, then
   `tools/plot_chA_tdev_goldilocks.py` (chA detrended TDEV). On MadHat
   today there is a single TICC (`/dev/ticc4`), and it appears to be the
   otcBob1-vs-dot166 comparison Bob asked to keep running. **This SXT-D is
   also the lab's GNSSDO+ AtomiChron reference** — disciplining its OCXO
   with PePPAR-Fix changes it from a reference into a DUT, which can
   disturb captures that rely on it. Pick the host/TICC/wiring
   deliberately.

### Run recipe (once the above are settled)

```sh
# 1) calibrate the slope (chA = SXT-D PPS OUT on the chosen TICC)
tools/calibrate_do.py --ticc-port /dev/ticcN --channel A \
    --gnssdo-transport serial --gnssdo-serial /dev/ttyUSB0 \
    --gnssdo-ppb-per-controlword 8e-4 --do-label gnssdo-sxtd-madhat \
    --sweep-ppb -5,5 --steps 11 --dwell 60          # → fitted slope

# 2) discipline (SBF error source; branch must have the SBF seam)
peppar-fix --host-config config/madhat-sxtd.toml --servo ... \
    # obs = mosaic-T SBF; actuator = $W; TICC logging on chA/chB

# 3) grade
tools/plot_chA_tdev_goldilocks.py --host sxtd data/ticcN-*.csv teal \
    --out plots/sxtd_chA_tdev.png       # chA detrended TDEV(τ)
```

Acceptance: STP3593LF is a double-oven OCXO → **OCXO-class sync target**;
grade against the per-clock budget (TDEV(1 s) ≤ ~150 ps, no positive-slope
region below τ ≈ 1000 s) from `docs/two-site-sync-budget.md`.

### Smoke run 2026-07-06 — pipeline assembles; one frontier remains

First closed-loop bring-up on MadHat (branch `delta/gnssdoActuator`,
rebased onto `main`). The **whole pipeline assembled and ran**:

- Mosaic-T `10.101.101.153` SBF `MeasEpoch@1Hz` on IPS2:28800 → engine
  `--obs-sbf-tcp` ingest → PPP. Position **converged** (σ 1.25 m, 42 s)
  at the London site; broadcast eph + SSR via NTRIP.
- Phase 2 → **`GNSSDO+ actuator: external control ON … anchor word=489538`**
  — the actuator took control over `/dev/ttyUSB0`, DOFreqEst initialized
  with OCXO class-default Q, calibrated slope +7.885e-4.
- Clean shutdown ran `teardown()` → `$E,0`; the GNSSDO+ returned to
  SparkFun internal discipline. (Earlier, an engine crash left it in
  external control and the **firmware watchdog recovered it** in 30 s —
  the failsafe works.)

**Bugs found + fixed** (the SBF path had never driven a servo before —
it was obs-only until now): the `--obs-sbf-tcp` path assumed
`args.serial` in three places (`os.path.basename`, the `--serial`
requirement, USB-serial UID synth); `want_servo` didn't trigger on a
non-PHC/non-TICC actuator; and `GNSSDO_controlword` needed adding to the
DO-schema actuator types. All committed.

**The remaining frontier:** the servo held control but did **not write
`$W`** — it sat at `[0] Awaiting correlatable observation (queued=128)`.
DOFreqEst's actuation loop expects a **correlatable hardware phase
event** (TICC chA / PHC EXTTS matched via `CLOCK_MONOTONIC`, per
`docs/stream-timescale-correlation.md`). In the no-TICC/no-PHC gnssdo
case the phase *is* the PPP `dt_rx`, which is a filter output, not a
timestamped hardware event — so nothing satisfies the correlation gate.
**Next task: wire PPP `dt_rx` as the servo's actuation phase arm for the
receiver-obs-drives-actuation case** (FixedPosFilter `dt_rx` →
DOFreqEst), so the loop closes. This is genuinely new servo territory —
the SBF obs seam had only ever fed position/obs, never an actuator.

Grading (chA TDEV) is unchanged and ready once the loop closes; the TICC
rig (chA = GNSSDO+ PPS OUT, chB = dot166 OTC, Rb reference) stays a pure
out-of-band monitor.

### Loop closed 2026-07-06 — dt_rx actuation arm

The frontier is closed. A new **`_dt_rx_servo_epoch`** (mirrors
`_cm_servo_epoch`) handles the receiver-clock-is-DO topology, routed by a
`dt_rx_phase` ctx flag (set when `actuator_type == "gnssdo"`):

- main loop **bypasses the PPS correlation gate** (takes the `popleft`
  path) so obs flow and `dt_rx` is computed with an empty `pps_history`;
- `dt_rx` feeds a new **DOFreqEst Arm 8 `do_phase`** (observes x[2] = DO
  phase, same H as EXTINT), with the PPP x[0] arm OFF (one oscillator, no
  double-count) — reusing the EKF's LQR + state-sanity + rate-limit
  safety.

Two things the hardware taught us (both fixed):

1. **Sign** — measured on the SXT-D: `+freq_ppb` speeds the OCXO up, which
   drives `dt_rx` *more positive*, so positive `dt_rx` = DO ahead. The
   `do_phase` arm uses "positive = late", so we feed **`-dt_rx`**. (First
   run with `+dt_rx` diverged: `+6.5 → +263 ns`; a bounded, watchdog-
   recovered runaway.)
2. **Outlier gate** — `dt_rx` is the *absolute* receiver-clock bias
   (legitimately large during acquisition), not a small aligned error, so
   the absolute `TRACK_OUTLIER_NS` gate rejected the whole acquisition.
   Replaced with a per-epoch **innovation (glitch)** gate; runaway safety
   is the EKF state-sanity gate + actuator clamp + firmware watchdog.

**Result (supervised, MadHat, obs from `10.101.101.153` SBF):**

```
phase +7.20 → +4.58 → +2.92 → +1.82 → +1.16 → +0.81 → +0.47 → +0.33 → +0.23 ns
freq  -0.21 → -0.14 → -0.08 → -0.03 → +0.01 → +0.02 → +0.04 → +0.05 ppb
```

`dt_rx` converges from 7.2 ns to **~0.23 ns in ~2 min**, frequency settles
near +0.05 ppb, **zero glitches, zero resets** — a locked loop. Clean
teardown → `$E,0`; Mosaic-T restored; DO back on internal discipline.

### Our steering vs the Mosaic-T's own solution (2026-07-06)

Done: the SBF reader now also captures `PVTGeodetic` (`RxClkBias` /
`RxClkDrift`, the Mosaic-T's own AtomiChron-corrected solution) into a
`PvtClockStore`, and `_dt_rx_servo_epoch` logs it next to our steering
(`--gnssdo-compare-log <csv>`; DT_RX log line too). A locked run showed:

```
ours:   dt_rx ~0.0 ns,  freq ~0.001–0.05 ppb   (we hold our carrier clock at 0)
mosaic: RxClkBias +6.2 → +7.8 ns (slow drift),  RxClkDrift ~0.005 ppb
```

So our ps-class carrier-phase clock and the Mosaic-T's internal solution
differ by **~7.8 ns with a slow relative drift** — the offset between the
two clock *definitions* (carrier-phase PPP vs the receiver's own SPP/
AtomiChron). Compare CSV archived at `~/gt/firmware/gnssdo-plus/
gnssdo-compare-20260706.csv`. (`Mode` field TODO: pysbf2 doesn't expose it
under that name; bias/drift are the signal.)

### Free-run characterization (collecting 2026-07-06)

`scripts/gnssdo_freerun_hold.py` holds the OCXO **free-running** for
`do_freerun_char`: it takes external control (`$E,1`, SparkFun's loop
stops) and re-writes the SAME control word every `--interval` s — a no-op
frequency-wise that just kicks the firmware watchdog so it doesn't reclaim
the oscillator. Running overnight on MadHat (`--duration 36000`, word held
≈489537) alongside a TICC capture (`gnssdo-freerun-ticc-*.csv`, chA = free
GNSSDO+ PPS, chB = dot166, Rb ref). **Morning: analyse chA detrended
(`tools/plot_chA_tdev_goldilocks.py` / `build_do_characterization.py`) →
STP3593LF free-run ADEV/TDEV → replace the OCXO class-default Q** in
`state/dos/gnssdo-sxtd-madhat.toml`, then re-grade the disciplined loop.

### Comparing our steering vs theirs — what's valid (2026-07-06)

`tools/analyze_gnssdo_compare.py` computes the *metrologically valid*
comparisons (not a naive TDEV-vs-TDEV):

- **TDEV of the difference** `(our dt_rx − their RxClkBias)` — the two
  *estimators'* relative stability (common OCXO cancels). This is the valid
  one. Individual TDEVs are context only: our `dt_rx` is a **closed-loop
  residual** (the signal we minimise — suppressed, not a clock; the
  chA-vs-chA-chB trap), theirs is an open-loop estimate.
- **TDEV is NOT run on the frequency corrections** — it's a phase metric.
  Servo smoothness is a **PSD / roughness** question.

On the 200 s snippet (settled window 141 s, so trustworthy only to τ≈17 s):

```
TDEV, detrended:  difference ★  τ1=5ps  τ4=11ps  τ8=24ps  τ16=41ps  (τ32=73ps, noisy)
                  ours (resid)  τ1=4ps  ...              theirs  τ1=3ps  ...  (all ps-class, similar)
```

The **"we're smooth, they're bumpy" is real and shows in the PSD** of the
correction signals (the roughness stat, 1.7×, understated it): at
f > 0.05 Hz the Mosaic-T's `RxClkDrift` carries **~4–6× more power** than our
`freq_ppb` — their per-epoch code/SPP estimate vs our loop-integrated
command that rolls off. Low-f they match (both track the same slow OCXO
motion). So: the smoothness gap = servo/measurement (PSD); the ~7.8 ns
offset = the carrier-PPP-vs-AtomiChron **datum**, whose long-τ behaviour
needs the longer capture below.

### The two overnight captures (sequenced — they're mutually exclusive)

The hardware can't free-run AND discipline at once, so:

1. **Free-run** (running now): `gnssdo_freerun_hold.py` + TICC chA → STP3593LF
   free-run ADEV/TDEV → measured Q into `state/dos/gnssdo-sxtd-madhat.toml`.
2. **Disciplined grade + long compare** (after #1, with the measured Q): the
   engine disciplining with `--gnssdo-compare-log` (stream
   `MeasEpoch+PVTGeodetic`) **plus** a TICC capture of the disciplined chA:

   ```sh
   # mosaic: sso, Stream3, IPS2, MeasEpoch+PVTGeodetic, sec1 ; siss, IPS2, 28800
   peppar-fix --host-config config/madhat-sxtd.toml \
       --obs-sbf-tcp 10.101.101.153:28800 --eph-mount BCEP00BKG0 \
       --ntrip-conf ntrip.conf --systems gps,gal --skip-bootstrap \
       --duration 36000 --gnssdo-compare-log data/gnssdo-compare-long.csv
   # separately: ticc_capture.py --device /dev/ticc4 --prefix gnssdo-disc-ticc
   ```

   → difference-TDEV out to long τ (`analyze_gnssdo_compare.py`) **and** the
   honest disciplined-output grade (chA detrended TDEV,
   `plot_chA_tdev_goldilocks.py`) — the real "grade vs TICC".
