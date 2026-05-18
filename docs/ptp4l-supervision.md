# ptp4l Supervision — Layered Clock-Class Degradation

> **Future direction**: see `docs/architecture-vision.md` — clockClass to be driven by estimator state transitions.

When peppar-fix disciplines a PHC that ptp4l distributes as a PTP
grandmaster, the two processes must coordinate quality signaling.  If
peppar-fix loses GNSS lock, enters holdover, or crashes, ptp4l must
stop announcing the clock as a locked primary reference.  Otherwise PTP
clients receive confidently-stamped bad time — the worst possible failure
mode.

## Architecture

ptp4l exposes a runtime management interface via a Unix datagram socket
(default `/var/run/ptp4l`).  The `SET GRANDMASTER_SETTINGS_NP` command
changes clockClass and related attributes without restarting ptp4l.
PTP clients see the new clockClass in the next Announce message and
BMCA selects a better GM if one exists.

peppar-fix uses three layers to ensure clockClass degrades whenever
the clock is untrusted:

```
┌──────────────────────────────────────────────────────────┐
│  Layer 1: Engine (fast path)                             │
│  Sets clockClass during normal operation.                │
│  Handles: servo lock → 6, holdover → 7, diverged → 248  │
├──────────────────────────────────────────────────────────┤
│  Layer 2: Wrapper script (peppar-fix)                    │
│  Sets clockClass 248 on any engine exit, 6 on restart.   │
│  Handles: engine crash, re-bootstrap gap, exit code 5    │
├──────────────────────────────────────────────────────────┤
│  Layer 3: systemd ExecStopPost (last resort)             │
│  Sets clockClass 248 unconditionally when the service    │
│  stops for any reason — including SIGKILL and OOM.       │
│  Handles: wrapper crash, unhandled signal, kernel OOM    │
└──────────────────────────────────────────────────────────┘
```

Each layer catches failures that the layer above it cannot.

## Clock-Class Promotion and Degradation

ptp4l starts at clockClass 248 (freerun).  peppar-fix promotes the
clock through three stages as confidence increases, and demotes it
when confidence is lost:

```
248 (freerun) ──bootstrap──▶ 52 (bootstrapped) ──settled──▶ 6 (locked)
                                  ▲                            │
                                  └──unsettled──◀──────────────┘
                                        │
     248 ◀──diverged/crash──────────────┘
     248 ◀──watchdog alarm──────────────┘
       7 ◀──holdover (from 6)───────────┘
```

| peppar-fix state | clockClass | clockAccuracy | timeSource | Meaning |
|---|---|---|---|---|
| Boot / freerun | 248 | 0xFE (unknown) | 0xA0 (internal osc.) | No idea what time it is |
| PHC bootstrapped | 52 | 0x23 (1 µs) | 0x20 (GPS) | Phase/freq set, servo converging |
| Servo settled | 6 | 0x20 (25 ns) | 0x20 (GPS) | Primary GNSS reference |
| Holdover | 7 | 0x23 (1 µs) | 0x20 (GPS) | Previously locked, coasting |

### Promotion gates (confidence-driven, with hysteresis)

The engine computes σ_total = RSS(σ_phase, σ_frequency) every
[CONFIDENCE_TOTAL] cycle and compares it to thresholds with a
deliberate hysteresis gap around each band boundary.  See
`docs/position-state-and-monitoring.md` for σ_phase / σ_frequency
definitions and the `[CONFIDENCE_*]` log line shape.

- **248 → 52**: σ_total falls below 800 ns.  The wrapper still
  asserts class 52 right after `phc_bootstrap.py` completes as a
  conservative default before the first [CONFIDENCE_TOTAL] cycle
  arrives; that path is unchanged.
- **52 → 6**: σ_total falls below 20 ns.  (5 ns margin under the
  25 ns clockAccuracy band edge.)

### Degradation triggers (confidence-driven where applicable)

- **6 → 52**: σ_total rises above 30 ns.  (5 ns margin above the
  band edge; together with the 20 ns rising threshold this gives
  a 10 ns dead band that prevents flapping when σ wobbles around
  25 ns.)
- **52 → 248**: σ_total rises above 1200 ns.  (200 ns margin above
  the 1 µs band edge, symmetric with the 800 ns rising threshold.)
- **6 → 7**: Observation idle timeout (holdover entry).
- **any → 248**: PHC diverged (exit code 5), watchdog alarm step
  action, engine crash, wrapper exit.

The σ-driven gates supersede the previous
`DisciplineScheduler._converging` boolean, which was a pure
frequency-tracking signal that ignored absolute-time uncertainty
from position.  The scheduler still runs and its `_converging`
state is still meaningful for the servo logic; it is no longer
the sole gate for clockClass promotion.

## Layer 1: Engine

The engine talks directly to ptp4l's UDS from Python
(`peppar_fix.pmc.PmcClient`), avoiding subprocess overhead for
latency-sensitive transitions.  The engine **auto-probes common
ptp4l UDS paths at startup** (`/var/run/ptp4l`, `/var/run/ptp4l.0`,
`/var/run/linuxptp/uds`, `/tmp/ptp4l`) and enables PMC management
when one is found; pass `--pmc <path>` to override or point at a
non-standard location.  The engine logs which path was discovered
(or that none were found) as `[PMC] Auto-discovered ...` /
`[PMC] No ptp4l UDS found ...` at startup.

Transitions:

- **σ_total falls below 20 ns**: promote clockClass 52 → 6.
- **σ_total rises above 30 ns**: demote clockClass 6 → 52.
- **σ_total falls below 800 ns**: promote clockClass 248 → 52.
- **σ_total rises above 1200 ns**: demote clockClass 52 → 248.
- **Holdover entry**: set clockClass 7.
- **PHC diverged (exit code 5)**: set clockClass 248 before exiting.
- **Watchdog alarm (step action)**: set clockClass 248 before exiting.

If the UDS send fails (ptp4l not running, wrong path), the engine logs
a warning but continues — the servo's job is clock discipline, not PTP
distribution.

## Layer 2: Wrapper

The `peppar-fix` wrapper script uses the `pmc` command (not the Python
module) for reliability — if the Python runtime is broken, `pmc` still
works.  Pass `--pmc /var/run/ptp4l` to the wrapper.

The wrapper sets clockClass at these points:

- **PHC bootstrap succeeds**: promote clockClass 248 → 52.
- **Any engine exit** (before re-bootstrap or retry): degrade to 248.
- **PHC re-bootstrap succeeds** (after exit code 5): promote back to 52.

```bash
# Degrade clockClass — used by wrapper and systemd ExecStopPost
pmc -u -b 0 "SET GRANDMASTER_SETTINGS_NP \
    clockClass 248 \
    clockAccuracy 0xfe \
    offsetScaledLogVariance 0xffff \
    currentUtcOffset 37 \
    leap61 0 \
    leap59 0 \
    currentUtcOffsetValid 0 \
    ptpTimescale 1 \
    timeTraceable 0 \
    frequencyTraceable 0 \
    timeSource 0xa0"
```

## Layer 3: systemd ExecStopPost

The systemd unit file includes `ExecStopPost=` running the same `pmc`
command.  systemd executes this even after SIGKILL or OOM-kill of the
main process — the only failure mode is systemd itself dying.

See `deploy/peppar-fix.service` for a complete example.

## ptp4l Configuration

ptp4l must run with `free_running 1` so it does not attempt to
discipline the PHC itself.  The initial clockClass should be 248
(freerun).  peppar-fix will promote to 52 after PHC bootstrap and
to 6 once the servo settles.

Example `/etc/ptp4l.conf`:

```ini
[global]
free_running            1
clockClass              248
clockAccuracy           0xFE
timeSource              0xA0
domainNumber            0
```

The `uds_address` defaults to `/var/run/ptp4l`.  If ptp4l runs
unprivileged, set `uds_address` to a writable path and pass the same
path to `--pmc`.

## Domain Number

The `--pmc-domain` flag (or `PEPPAR_PMC_DOMAIN` env var) must match
`domainNumber` in ptp4l's config.  If they don't match, ptp4l silently
ignores the management message.  The default is 0.

## Privilege Considerations

**Production deployments** will typically run both peppar-fix and ptp4l
as root.  This is the simplest and most common configuration for a GM
appliance.

**Development / unprivileged operation** is possible because PHC
discipline uses `clock_adjtime()` via the `/dev/ptpN` file descriptor,
which only requires write permission on the device file (no
`CAP_SYS_TIME` needed).  On Debian/Ubuntu, `/dev/ptp0` is typically
`root:dialout` mode 0664, so any user in the `dialout` group can
discipline the PHC.

For unprivileged pmc access, set `uds_file_mode 0666` in ptp4l's
config.  The `pmc` command also needs a writable path for its client
socket — use `-i /tmp/pmc.$$` to avoid `/var/run/` permission issues.

## Alternative: BindsTo=

For simpler deployments where graceful BMCA failover is unnecessary
(single GM, no backup), add `BindsTo=peppar-fix.service` to the ptp4l
unit.  systemd will stop ptp4l whenever peppar-fix stops.  This is
coarser than clockClass degradation — PTP clients lose their GM entirely
rather than seeing it degrade — but is trivial to set up.

## Testing

Verify the integration by querying ptp4l's current GM settings:

```bash
pmc -u -b 0 "GET GRANDMASTER_SETTINGS_NP"
```

Simulate a peppar-fix crash and confirm clockClass degrades:

```bash
sudo systemctl kill --signal=KILL peppar-fix.service
pmc -u -b 0 "GET GRANDMASTER_SETTINGS_NP"
# Should show clockClass 248
```
