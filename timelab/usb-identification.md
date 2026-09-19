<!-- STALE COPY — DO NOT TRUST -->
> # ⚠ STALE COPY — the authoritative file is in the `timelab` repo
>
> This is a **tracked duplicate** inside PePPAR-Fix, not a symlink and not a
> submodule.  It is **63 lines behind** the real file and nobody
> maintains it.  Anyone who greps the PePPAR-Fix tree finds this plausible,
> well-formed, badly out-of-date answer with no signal that it is stale — that
> has already caused a collaborating agent to write a correct commit into a
> file nobody reads (2026-09-19).
>
> **Authoritative:** `git@github.com:bobvan/timelab.git`, sibling checkout at
> `~/git/timelab/usb-identification.md`.  Edit, commit and push *there*.
>
> Left in place rather than deleted because this directory is not a clean set
> of duplicates: `resources.json` exists **only here**, and
> `99-timelab.rules` is *larger* here than in the timelab repo — so some of
> this content may be unique.  Untangling that needs Bob.

# USB Device Identification Notes

## TICCs: Unique by serial number

TAPR TICCs use an Arduino Mega 2560 (VID `2341`, PID `0042`) with a
**unique serial number** per unit. udev rules should always match by
`ID_SERIAL_SHORT` so the TICC keeps its `/dev/ticc` symlink regardless
of which USB port it's plugged into.

| TICC | Arduino Serial Number | Label suggestion |
|---|---|---|
| #1 | `95037323535351803130` | ticc-1 |
| #2 | `44236313835351B02001` | ticc-2 |
| #3 | `44236313835351B0A091` | ticc-3 |

Note: TICCs also have an FTDI FT230X chip for the data serial port.
The FTDI chip shows up as `/dev/ttyUSBx` separately from the Arduino
(`/dev/ttyACMx`). The FTDI chip's serial number should also be checked
and documented. The Arduino serial identifies the TICC unit; the FTDI
serial identifies the data channel.

## F9T EVKs: NOT unique via USB — but have UBX SEC-UNIQID

All u-blox F9T EVKs (both ZED-F9T and ZED-F9T-20B) report:
- VID: `1546`, PID: `01a9`
- Model: `u-blox_GNSS_receiver`
- USB Serial: **(none)**

There is no way to distinguish two F9T EVKs by USB descriptor alone.

However, each chip has a unique hardware ID queryable via `UBX-SEC-UNIQID`:

| Receiver | SEC-UNIQID | Model | Label |
|---|---|---|---|
| F9T-TOP | `136395244089` | ZED-F9T | TimeHat /dev/gnss-top (as of 2026-04-13) |
| F9T-BOT | `262843023907` | ZED-F9T-20B | MadHat /dev/ttyACM0 (as of 2026-04-13) |
| F9T-3RD | `394029318459` | ZED-F9T-20B | clkPoC3 /dev/ttyACM1 (as of 2026-04-13) |

The Deacon should periodically verify SEC-UNIQID matches expected
receiver-to-host assignment (runs when host is idle, via a small
UBX query script). This catches unrecorded cable swaps.
Options for multi-F9T hosts:

1. **USB path matching** (`ID_PATH`): Stable across reboots if cables
   stay in the same physical USB ports. Breaks on replug to different
   port. PiPuss uses this.

2. **One F9T per host**: Simplest. Match by VID:PID only. TimeHat uses
   this.

3. **USB hub with known topology**: Dedicated hub per receiver,
   identified by hub serial + port number.

When cabling changes happen, USB path-based udev rules MUST be
reverified. Run `udevadm info /dev/ttyACMx | grep ID_PATH` after
any cable change.

## Other devices

| Device | Host | VID:PID | Serial | Identification |
|---|---|---|---|---|
| PX1125T (SkyTraq) | ~~Onocoy~~ (mothballed 2026-04-08) | Silicon Labs CP2102, `10c4:ea60` | `0001` (generic) | Stored. |
| FS switch console | ~~Onocoy~~ (mothballed 2026-04-08) | Prolific, `2478:2008` | (none) | Stored. |
| F10T (ArduSimple board) | ~~Onocoy~~ (mothballed 2026-04-08) | FTDI FT230X, `0403:6015` | **`D30GD1PE`** | Stored. `/dev/f10t` udev rule still in `99-timelab.rules` for whenever F10T is revived on a different host. |

Note: TICC #2 has NO separate FTDI data port. Data comes over the
Arduino ACM connection. The FTDI `D30GD1PE` belongs to the F10T's
ArduSimple carrier board, not to a TICC.

## Lab udev policy

- **Unique serial** → universal udev rule creates stable `/dev/` symlink
  on any host. Rules in `timelab/99-timelab.rules`, deployed to all Pis.
- **No unique serial** → no udev symlink. Application must identify
  the device at runtime (probe protocol, check VID:PID, etc.).
- **F9T EVKs** are a special case: no USB serial, but only one or two
  per host. Host-specific path-based rules are acceptable with a
  WARNING comment that re-cabling requires rule reverification.
