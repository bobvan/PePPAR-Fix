# Lab Operations Guide

## Problem analysis: deployment stumbles (2026-04-04 to 2026-04-06)

Every stumble from the ClockMatrix integration sessions, categorized:

### 1. Missing dependencies on lab hosts

| Host | Issue | Recovery time |
|---|---|---|
| otcBob1 | `pyubx2` not installed (user bob) | 1 min |
| otcBob1 | `pyubx2` not installed (root/sudo) | 1 min |
| ocxo | `pyubx2` not installed (root/sudo) | 1 min |

**Root cause**: No venv on otcBob1/ocxo. TimeHat has a venv.
Packages installed ad-hoc with `--break-system-packages` — this
is wrong and creates fragile, non-reproducible environments.

**Fix**: Every host gets a venv. The `peppar-fix` wrapper already
discovers venvs at `$REPO_ROOT/venv/`. PTP ioctls need root, so
`sudo` must use the venv's python — the wrapper handles this via
`$PYTHON_BIN`. Never install into system python.

### 2. Missing directories

| Host | Issue | Recovery time |
|---|---|---|
| ocxo | `data/` directory missing, position save failed | 2 min |
| otcBob1 | No `~/peppar-fix` directory at all | 1 min |

**Root cause**: rsync of scripts/ doesn't create data/. First-time
deployment to a new host has no setup procedure.

**Fix**: `peppar-fix` wrapper should `mkdir -p data/` at startup.
Or: an install/setup script that creates the standard layout.

### 3. Missing or wrong credentials

| Host | Issue | Recovery time |
|---|---|---|
| otcBob1 | No `ntrip.conf` | 2 min (scp from TimeHat) |
| ocxo | No `ntrip.conf` | 2 min (scp from TimeHat) |

**Root cause**: Credentials not in repo (correctly). No provisioning
step for new hosts.

**Fix**: The `peppar-fix` wrapper already validates `--ntrip-conf`.
Need a deploy checklist or setup script that copies credentials.

### 4. Host config mismatches

| Host | Issue | Recovery time |
|---|---|---|
| otcBob1 | No host config file existed | 5 min (created otcbob1.toml) |
| otcBob1 | `ubx_port = "UART1"` invalid (should be "UART") | 3 min |
| ocxo | `ptp_dev = /dev/ptp2` vs profile `device = /dev/ptp1` | 5 min (known issue) |

**Root cause**: Host configs created ad-hoc. No validation that
config values match what the hardware actually has.

**Fix**: A `peppar-preflight` script that:
- Verifies serial port exists and responds
- Verifies PTP device exists and has EXTTS capability
- Verifies TICC device exists (if configured)
- Checks pyubx2/smbus2 imports
- Checks ntrip.conf exists
- Creates data/ directory

### 5. Engine argparse restrictions

| Issue | Recovery time |
|---|---|
| `--ptp-profile` had `choices=["i226", "e810"]` hardcoded | 3 min |
| `--engine-arg "--ticc-port /dev/ticc1"` passed as single string | 10 min |

**Root cause**: Hardcoded choices don't allow new profiles. The
`--engine-arg` interface is fragile — arguments with values need
TWO `--engine-arg` invocations.

**Fix**: Remove `choices=` from argparse (done). Document the
`--engine-arg` splitting requirement. Better: add `--ticc-port`
and `--ticc-log` as first-class wrapper options.

### 6. Stale processes from previous runs

| Host | Issue | Recovery time |
|---|---|---|
| otcBob1 | Multiple crashed python3 processes from DPLL experiments | 2 min |
| otcBob1 | Engine retry loop running from previous failed attempt | 3 min |
| TimeHat | Wrapper piping to `tail -1` hid all output | 5 min |

**Root cause**: SSH-launched background processes don't clean up on
disconnect. The wrapper's retry loop keeps restarting the engine
even when the error is permanent (wrong profile name).

**Fix**: PID file management in the wrapper. `peppar-fix` should
check for and kill stale processes before starting. Permanent
errors (argparse failures) should not trigger retry.

### 7. Device permission and driver issues

| Host | Issue | Recovery time |
|---|---|---|
| otcBob1 | `/dev/ptp0` is root-only (0600) | 1 min (sudo) |
| otcBob1 | TICC USB didn't enumerate (no power on USB port) | 5 min |
| ocxo | E810 EXTTS `ioctl: Input/output error` (wrong pin/channel for out-of-tree driver) | 10 min (pre-existing, not resolved) |

**Root cause**: No udev rules for PTP devices on otcBob1. TICC needs
USB power that the OTC board doesn't provide. E810 out-of-tree driver
uses different SDP pin mapping.

**Fix**: Deploy 99-timelab.rules to all hosts. Document USB power
requirement for TICC on OTC. Fix E810 pin mapping in e810 profile.

### 8. Code deployment inconsistency

| Issue | Recovery time |
|---|---|
| rsync of scripts/ only, missing config/ | 2 min |
| rsync missed new files in peppar_fix/ subdirectory | 3 min |
| Full rsync sends 100 MB due to data/ and .git/ | 1 min |
| No git on lab hosts — can't `git pull` | ongoing |

**Root cause**: No standard deployment method. Ad-hoc rsync with
varying exclude lists. Some hosts have partial repos.

**Fix**: Standardize on `git clone` + `git pull` on each host.
Or: a `deploy.sh` script that does the right rsync with proper
excludes.

### 9. ClockMatrix-specific issues

| Issue | Recovery time |
|---|---|
| Assumed 8A34002 register map (actually 8A34002) | 2 hours |
| pll_mode bits[2:0] vs bits[5:3] confusion | 30 min |
| Phase status interpreted as 64-bit (actually 32-bit + flags) | 20 min |
| DPLL mode switch crashed host (cycled all 4 DPLLs) | 10 min (power cycle) |
| FOD_FREQ saturation at ±8 ppb (wrong register for steering) | 30 min |
| FCW didn't work (wrong pll_mode bits) | 20 min |
| TDC Output targets don't include CLK inputs | 30 min |
| DPLL phase status frozen when locked | 20 min |

**Root cause**: Reverse engineering without full documentation.
Many of these are one-time learning costs, now documented in
`docs/timebeat-otc-register-map.md` and memory files.

**Fix**: Already documented. Future sessions read the register
map doc and memory before touching hardware.

## Standard lab host layout

Every peppar-fix lab host should have:

```
~/peppar-fix/
├── scripts/          # peppar-fix code (from git)
│   ├── peppar-fix    # orchestration wrapper
│   ├── peppar_fix/   # Python package
│   └── *.py          # component scripts
├── config/           # host configs + profiles (from git)
│   ├── receivers.toml
│   └── <hostname>.toml
├── data/             # runtime data (NOT in git)
│   └── position.json
├── ntrip.conf        # NTRIP credentials (NOT in git)
├── venv/             # Python virtualenv (if not system python)
└── drift.json        # frequency drift file (NOT in git)
```

## Deployment procedure

### First-time setup on a new host

```bash
# 1. Clone the repo
ssh <host>
git clone <repo-url> ~/peppar-fix
cd ~/peppar-fix

# 2. Create venv and install deps
python3 -m venv venv
source venv/bin/activate
pip install pyubx2 pyserial
pip install "pyproj>=3.6"  # NAD83(2011)↔ITRF2020 datum conversion (needs bundled PROJ ≥ 9.0)
pip install smbus2  # only on hosts with I2C (otcBob1, ptBoat)

# 3. Create runtime directories
mkdir -p data

# 4. Deploy credentials
scp TimeHat:~/peppar-fix/ntrip.conf ~/peppar-fix/

# 5. Create host config (if not already in repo)
# Edit config/<hostname>.toml with correct serial port, PTP device, etc.

# 6. Verify
sudo scripts/peppar-fix --duration 60
```

**Why venv, not system python?** The `peppar-fix` wrapper already
auto-discovers `$REPO_ROOT/venv/bin/python` and uses it for all
subprocesses including sudo. This keeps dependencies isolated and
makes the host reproducible. `--break-system-packages` is a last
resort that should never be needed if the venv is set up correctly.

**Sudo and the venv**: `sudo scripts/peppar-fix` works because the
wrapper sets `PYTHON_BIN` to the venv's python before calling sudo.
The sudo'd subprocesses inherit this. PTP ioctls need root, but
Python packages come from the venv. For future production use, a
systemd unit with `AmbientCapabilities=CAP_SYS_TIME CAP_NET_RAW`
would eliminate sudo entirely (see `docs/ptp4l-supervision.md`).

### Updating code on a lab host

```bash
ssh <host>
cd ~/peppar-fix
git pull
```

Do NOT use rsync or scp for code updates — it creates inconsistency.

### Pre-flight checklist (manual, until automated)

Before starting a long run:
- [ ] `ps aux | grep peppar` — no stale processes
- [ ] Serial port exists and is accessible
- [ ] PTP device exists: `ls /dev/ptp*`
- [ ] TICC device exists (if using): `ls /dev/ticc*`
- [ ] ntrip.conf present
- [ ] data/ directory exists
- [ ] `python3 -c "import pyubx2"` succeeds (in the right python)
- [ ] `python3 -c "import pyproj"` succeeds (geo_frames datum conversion)
- [ ] Timebeat stopped (on OTC hosts): `systemctl is-active timebeat`
- [ ] System time daemon running and locked (see below)
- [ ] PHC phase check (see below)

### System time and PHC phase verification

**System time daemon**: A time daemon must be running and locked to a
trustworthy master before starting peppar-fix.  The daemon can be
`systemd-timesyncd`, `chrony`, or `ntpd` — it doesn't matter which,
but we must know what it is and confirm it has a confident lock:

```bash
# systemd-timesyncd:
timedatectl show --property=NTPSynchronized   # should say "yes"

# chrony:
chronyc tracking | grep -E "Leap|Stratum"     # stratum > 0

# ntpd:
ntpq -p | head -5                             # should show * peer
```

**PHC phase check**: Compare PHC to system clock.  The PHC runs on
TAI, so the expected offset is ~37 seconds (current TAI-UTC = 37s):

```bash
sudo phc_ctl /dev/ptp0 cmp
```

Expected: **~37,000,000,000 ns** (37 seconds).

- **36,500,000,000 or 37,500,000,000** → PHC is 500 ms out of phase.
  The bootstrap will land PEROUT on the wrong half-second.
  Fix: `sudo phc_ctl /dev/ptp0 set` to reset PHC to system time,
  then verify with `cmp` again.
- **Wildly different** (seconds or more off) → PHC was never set,
  or the i226/igc driver was reloaded and PHC reset to epoch 0.
  Fix: `sudo phc_ctl /dev/ptp0 set` and verify.

This check catches the i226 PEROUT 500 ms phase ambiguity before it
wastes an overnight run.  The bootstrap's ADJ_SETOFFSET step only
makes fine adjustments — it cannot fix a 500 ms offset because it
assumes the PHC is already within a few milliseconds of GPS time.

## pi4ptpmon bringup stumbles (2026-04-11)

Fresh CM4 Lite (906 MB RAM, kernel 6.12.62+rpt-rpi-v8) with i226,
TICC #2, and F9T EVK.  Brought up from zero to running peppar-fix
freerun + carrier-phase PPP.

### 10. gt host key unknown on fresh host

New host had never talked to gt.  `git clone bob@gt:git/PePPAR-Fix.git`
failed with "Host key verification failed."

**Fix**: `ssh-keyscan gt >> ~/.ssh/known_hosts` before cloning.

**Add to first-time setup procedure**: step 0 is accept gt's host key.

### 11. No SSH keypair for gt access

Fresh Pi OS image has no `~/.ssh/id_*` keypair.  After accepting the
host key, git clone still failed with "Permission denied (publickey)."

**Fix**: `ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519` on the new
host, then authorize the public key on gt.  Used TimeHat as a hop to
reach gt since the dev workstation also lacked gt access.

**Add to first-time setup procedure**: generate key, authorize on gt.

### 12. F9T USB-attached path: use `/dev/serial/by-path/` (lab convention)

F9T EVKs share VID:PID `1546:01a9` with no USB serial number.  udev
can't manufacture a stable symlink from the device descriptor (every
F9T looks identical), so the lab convention is:

- **Devices with a unique ID readable by udev** (TICC Arduino serials,
  i226/E810 PHC MAC) get a stable symlink via `99-timelab.rules`.
- **Devices that look identical to udev** (F9T EVKs) get **no** udev
  rule.  Configs reference the kernel-generated path directly.

For F9Ts, the only kernel-generated path that survives the addition
or removal of other USB devices is **`/dev/serial/by-path/`**, which
binds to the physical USB port (bus + port chain).  `/dev/ttyACMx`
shifts whenever a sibling device enumerates differently;
`/dev/serial/by-id/` collapses to one name across all F9Ts.

**Convention**: single-F9T hosts set `serial = "/dev/serial/by-path/<port>"`
in `config/<host>.toml`.  Find the value with:

```sh
ls -la /dev/serial/by-path/   # F9T entry without -port0 suffix
```

The by-path string only breaks if you move the F9T's USB cable to a
different port on the host.  Lab-edit-and-test workflow handles that
case: `git status` after a cable move flags it.

**Multi-F9T hosts** (e.g., PiPuss) can't be solved by by-path alone if
both ports are equivalent — use a host-specific udev rule that
matches `ID_PATH` and produces a friendly symlink (`gnss-top`,
`gnss-bot`).  This is the documented exception, not the default.

### 13. Wrapper freerun + retry loop surprises

`peppar-fix --duration 60` first ran a 30-minute freerun
characterization, then restarted the engine for the 60-second
discipline run, then kept restarting on exit.  Not a bug — the
wrapper's design — but surprising when you expect 60 seconds total
and SSH sessions accumulate zombie process trees.

**Observation only** — matches stumble #9 in madhat-bringup doc
(wrapper override of `--duration`).

### 14. pi4ptpmon PHC is /dev/ptp1, not /dev/ptp0

CM4 Lite has a Broadcom PHC at `/dev/ptp0` (bcm_phy_ptp).  The i226
is at `/dev/ptp1`.  The pi4ptpmon.toml config correctly has `ptp_dev =
"/dev/ptp1"`, but `phc_ctl /dev/ptp0 cmp` in the pre-flight
checklist checks the wrong device.  The pre-flight check must use
the host config's `ptp_dev`, not hardcoded `/dev/ptp0`.

### 15. No igc DKMS on pi4ptpmon — PEROUT 500ms after step

Stock igc on kernel 6.12.62+rpt-rpi-v8.  Our code's period-nudge
workaround (`999_999_999 ns`) avoids frequency mode, but the
PEROUT still lands at 500ms after the bootstrap's PHC step.  The
PEROUT phase verification in `_enable_pps_out()` didn't catch it
(EXTTS read may have timed out or returned the wrong pin's event).

**Fix**: Install igc-6.12.0-ppsfix.1 DKMS (from TimeHAT repo) and
rebuild for the v8 kernel.  Also investigate why the PEROUT phase
verification silently failed.

### 16. DO frequency 3790 ppb on pi4ptpmon

First-boot i226 crystal shows 3790 ppb offset — 30x higher than
TimeHat/MadHat (~125 ppb).  Possibly the crystal hasn't thermally
stabilized, or the PHC was never properly set before the PPS
frequency measurement.  The bootstrap kept re-stepping and failed
to converge.

**Fix**: Set PHC from system time (`phc_ctl /dev/ptp1 set`) before
running peppar-fix.  Let the crystal stabilize for 30+ minutes.
If the frequency remains >1000 ppb, the crystal may be defective.

## Future work

### Near-term (reduces stumbling)

1. **Pre-flight script** (`scripts/peppar-preflight`): automated version
   of the checklist above. Run before `peppar-fix`, or integrated into
   the wrapper.

2. **Add --ticc-port and --ticc-log as wrapper options**: currently
   passed via fragile `--engine-arg` splitting. Should be first-class.

3. **mkdir -p data/ in wrapper**: trivial fix, prevents first-run failure.

4. **Remove argparse choices on --ptp-profile**: allow any profile name.
   Already done for the engine; check bootstrap too.

5. **Stale process detection**: wrapper checks for and warns about
   existing peppar-fix processes before starting.

### Medium-term (production readiness)

6. **systemd unit file** (`deploy/peppar-fix.service`): proper service
   management with restart policy, logging to journald, and clean
   shutdown. See `docs/ptp4l-supervision.md` for the supervision model.

7. **Install script** (`scripts/peppar-install`): creates venv, installs
   deps, creates directories, deploys udev rules, validates config.

8. **Persistent TICC port config in host TOML**: add `ticc_port`,
   `ticc_log`, `ticc_ref_channel`, `ticc_phc_channel` to the host
   config. The wrapper reads them like other host config keys.

9. **Drift file in standard location**: currently ad-hoc. Should be
   `data/drift.json` on all hosts.

### Long-term (multi-host fleet)

10. **Ansible playbook for fleet deployment**: deploy code, configs,
    credentials, udev rules, systemd units to all lab hosts.

11. **Health monitoring dashboard**: watch all hosts via PTP or
    peppar-fix status endpoint. Detect stuck servos, lost PPS, etc.
