# Receiver Signal Requirements

## Required signals

PePPAR-Fix forms ionosphere-free (IF) linear combinations from dual-frequency
pseudorange and carrier-phase observations. This requires two signals per
constellation on every satellite:

| Constellation | Primary (f1) | Secondary (f2) | Notes |
|---------------|-------------|----------------|-------|
| GPS | L1 C/A (1575.42 MHz) | L5 Q (1176.45 MHz) | L5 health override required |
| Galileo | E1 C (1575.42 MHz) | E5a Q (1176.45 MHz) | Same frequency as L5 |
| BeiDou | B1I (1561.098 MHz) | B2a I (1176.45 MHz) | MEO/IGSO only (PRN >= 19) |

GLONASS is excluded (FDMA complicates IF processing). SBAS and QZSS are
disabled.

### Why L1+L5 (not L1+L2)

The F9T supports either L1+L2 or L1+L5 but not both simultaneously (two
frequency bands maximum). We standardize on L1+L5 because:

- L5/E5a is a modernized signal with better code structure and lower noise
- GPS L5 availability is now sufficient (30+ SVs as of 2025)
- L5 and E5a share the same center frequency (1176.45 MHz), simplifying
  the IF math for cross-constellation consistency
- BeiDou B2a also shares this frequency

### GPS L5 health override

GPS satellites broadcast a health flag for each signal. As of 2026, many
GPS L5 signals are still marked "unhealthy" even though they are fully
usable. Without an explicit override, the F9T will not track these signals.

The override is set via UBX CFG-VALSET with key `0x10320001` (value 1).
This key is documented in u-blox Application Note UBX-21038688 ("GPS L5
configuration") but is not yet exposed in pyubx2's key database.

After setting the override, a warm restart is required for the receiver
to begin tracking the newly-enabled L5 signals. The warm restart preserves
ephemeris data, so there is no cold-start penalty.

If the receiver NAKs this key, it means the firmware does not support L5
health override. L5 signals will still be tracked for SVs that broadcast
healthy L5 status, but some satellites will be unavailable.

## Required UBX messages

The following messages must be enabled on whichever port the host reads:

| Message | Purpose | Expected rate |
|---------|---------|---------------|
| RXM-RAWX | Pseudorange, carrier phase, Doppler, C/N0 | Every epoch (1 Hz) |
| RXM-SFRBX | Broadcast navigation data (ephemeris) | Per subframe (~2-6s) |
| NAV-PVT | Position/velocity/time solution | Every epoch (1 Hz) |
| TIM-TP | PPS quantization error (qErr) | Every epoch (1 Hz) |

NAV-SAT (satellite status) is optional, enabled at 1/5 rate when available.

## Startup signal validation

At startup, the code listens for RAWX observations and checks that
dual-frequency GPS+GAL observations are arriving. If they are not:

1. Configure signals via CFG-VALSET (L1+L5 config)
2. Apply GPS L5 health override
3. Warm restart
4. Re-check for dual-frequency observations

If dual-frequency observations still aren't arriving after reconfiguration,
startup fails with a clear error.

## UBX command/response sequencing

UBX CFG-VALSET messages produce one ACK-ACK (success) or ACK-NAK (failure)
response **per VALSET message** — not per key inside the message.  A
single VALSET that carries multiple keys returns a single ACK/NAK for
the bundle.

### Policy: no bundled CFG-VALSET at the wire level

**Every CFG-VALSET sent to a receiver MUST carry exactly one key, and
the caller MUST synchronously wait for the ACK/NAK before sending the
next.**  Multi-key VALSETs are prohibited.

Why: a bundled VALSET that NAKs tells us "at least one key failed" but
not which one.  When something goes wrong (firmware version mismatch,
key renamed across chipsets, unsupported feature on this hardware), the
identity of the failing key is lost — undiagnosable spray-and-pray.

### Acceptable: dict at the caller, single-key on the wire

Higher-level code is welcome to compose multi-key dicts for
organizational purposes — `configure_messages()` groups MSGOUT keys,
`configure_signals()` groups signal-enable keys, etc.  These are passed
through `send_cfg()` or `send_cfg_per_key()`, both of which **serialize
the dict to one-key-at-a-time at the wire level** and log each ACK/NAK
individually.

Wire-level helper: `_send_cfg_one(ser, ubr, key, value)` — sends one
key, waits one ACK/NAK, returns bool.  The only place that writes a
VALSET to the serial port.

Caller-facing helpers:
- `send_cfg(ser, ubr, dict)` → bool (True iff every key ACK'd)
- `send_cfg_per_key(ser, ubr, dict)` → `(ok_set, nak_set)` partition

Both call `_send_cfg_one` per key; choose between them by return shape.
Use `send_cfg_per_key()` when the caller needs to act on the specific
failing key (e.g., retry, fall back, log a structured diagnostic).
Use `send_cfg()` when an aggregate pass/fail is enough.

### Why this matters

- Signal configuration, message routing, and rate changes are separate
  groups of VALSET keys; a NAK in one group has a different meaning
  than a NAK in another
- Receiver chipsets diverge on key names (F9 vs F10 vs Timebeat) and
  on which features the chip supports — without per-key ACK/NAK we
  can't tell whether the burst failed because of a typo, a chipset
  difference, or a firmware-version gap
- A 3-second timeout per key is acceptable startup latency
  (typical config burst: 5-15 keys, 15-45s worst-case); a silent
  startup failure with no log evidence of which key broke is not

### History

- 2026-04-17 ptpmon: L5 signal enable bundled as 15 keys → NAK on
  the whole bundle → no way to identify the rejected key without a
  per-key rebuild.  Led to `send_cfg_per_key()` being added as a
  diagnostic helper.
- 2026-05-12 MadHat F10T: post-config burst bundled 5 keys
  (NAV2_OUT_ENABLED + NAV_SIG + NAV_CLOCK + NAV_TIMEGPS + TIM_TM2)
  → suspected NAK → same undiagnosable failure.  Led to making
  per-key behavior the default for all callers.

### DO NOT add a new bundled-VALSET helper

If a future caller has a documented need for raw multi-key VALSET
semantics (e.g., the receiver explicitly documents atomic key-group
behavior for a specific feature), document the departure inline at
the call site and justify why the diagnostic loss is acceptable
for that case.  No new general-purpose bundled helpers.

## Port types

The F9T exposes multiple communication ports. The port ID determines
which `CFG_MSGOUT_*` suffix to use:

| Port | ID | Suffix | Typical use |
|------|----|--------|-------------|
| UART1 | 1 | `_UART1` | External serial (ArduSimple, EVK) |
| UART2 | 2 | `_UART2` | Secondary serial |
| USB | 3 | `_USB` | USB connection (most common for external F9T) |
| SPI | 4 | `_SPI` | SPI bus |
| I2C/DDC | 0 | `_I2C` | I2C bus (E810 onboard F9T uses this) |

The E810's onboard F9T connects via I2C (port 0). External F9T boards
(ArduSimple, EVK) typically use USB (port 3). Message routing must target
the correct port or observations won't arrive on the host.

## Hardware variants

Three F9T variants exist in the lab.  Firmware string alone doesn't
tell them apart — use **MON-HW3 vpManager_07** to detect L5-capable
hardware (=1) vs classic L2-only (=0).  See
`docs/f9t-firmware-capabilities.md` for the full capability matrix.

| | ptpmon (E810) | TimeHat | MadHat / -20B units |
|---|---|---|---|
| **MOD** | ZED-F9T | ZED-F9T | ZED-F9T-20B |
| **FWVER** | TIM 2.20 | TIM 2.20 | TIM 2.25 |
| **PROTVER** | 29.20 | 29.20 | 29.25 |
| **ROM** | 0x118B2060 | 0x118B2060 | 0x3BFC8935 |
| **vpManager_07** | 0 | 1 | 1 |
| **Second freq** | L2C, E5b, B2I only | L2C or L5 (not simultaneous) | L5, E5a, B2a (locked; NAKs L2C) |
| **L5 signal config** | NAK (no RF front-end) | OK | OK |
| **Driver** | F9TL2E5bDriver | F9TL5Driver (or F9TDriver for diag) | F9TL5Driver |
| **Transport** | I2C (/dev/gnss0, kernel) | USB serial | USB serial |
| **Host** | x86 E810-XXVDA4T, OCXO | Raspberry Pi 4, i226 | Raspberry Pi 4 |

### Auto-detection in peppar-fix

`ensure_receiver_ready()` handles L5-capable hardware automatically:
1. Tries L5 signal config (F9TL5Driver)
2. If NAK'd, falls back to L2C (F9TDriver with E5a)

This auto-detection does **not** cover the L2-only hardware variant
(ptpmon) — the fallback F9TDriver expects E5a, which NAKs on L2-only
hardware.  For those units, set `receiver = "f9t-l2-e5b"` explicitly
in the host config.
