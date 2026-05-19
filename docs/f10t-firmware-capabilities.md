# u-blox NEO-F10T Firmware Capability Matrix

Experimentally determined via per-key CFG-VALSET probing on the lab's
single F10T receiver after the 2026-05-12 swap from F9T to F10T on
MadHat.  Companion to `docs/f9t-firmware-capabilities.md`; same
methodology, different chipset family.

## Test receiver

| Name | Module | Firmware | PROTVER | Host | Interface |
|------|--------|----------|---------|------|-----------|
| F10T (ArduSimple) | NEO-F10T-00B-01 | TIM 3.02 | 42 | MadHat | UART1 @ 38400 baud |

The NEO-F10T module datasheet lists UART, I2C, and SPI interfaces.
**No USB.**  What appears as `/dev/f10t` on the host is the
ArduSimple breakout board's USB-UART bridge (FTDI-class chip),
NOT a USB interface on the F10T itself.  Every CFG-MSGOUT key
must use the `_UART1` suffix; `_USB`-suffixed keys NAK
categorically because the F10T has no USB port to bind them to.

## Port-suffix matrix

| Suffix | F10T result | Notes |
|---|---|---|
| `_UART1` | accepted | The only working suffix on this hardware |
| `_USB`   | NAK universal | No USB port on the F10T module |
| `_I2C`   | not tested | Hardware-wise supported per datasheet |
| `_SPI`   | not tested | Hardware-wise supported per datasheet |
| `_UART2` | not tested | F10T datasheet does not document a UART2 |

The F10T has effectively **one serial port** (UART1).  This differs
from F9T's wider port matrix (UART1, UART2, USB, I2C, SPI all
supported by the chip).

## CFG-MSGOUT capability matrix

Tested by sending CFG-VALSET (RAM+BBR+Flash layers) for each key
individually after the 2026-05-12 `aac39ba` no-bundle refactor.
ACK = accepted; NAK = rejected by firmware.

| Message | CFG key on `_UART1` | F10T (TIM 3.02 / PROTVER 42) | Notes |
|---|---|---|---|
| NAV2 secondary engine | `CFG_NAV2_OUT_ENABLED` | **NAK (idempotent)** | NAV2-PVT flows; the NAK appears to indicate "already enabled" rather than "unsupported."  Skip the enable; let MSGOUT do the work. |
| NAV2-PVT | `CFG_MSGOUT_UBX_NAV2_PVT_UART1=5` | ACK | Position consensus stream |
| NAV-SIG | `CFG_MSGOUT_UBX_NAV_SIG_UART1=1` | ACK | Per-(SV, signal) usage verdict; Phase A.5 logger consumes |
| NAV-CLOCK | `CFG_MSGOUT_UBX_NAV_CLOCK_UART1=1` | ACK | Receiver clock-bias / drift / accuracy telemetry |
| NAV-TIMEGPS | `CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1=5` | **NAK (rate cap)** | rate=5 rejected; NAV-TIMEGPS messages flow anyway at default rate.  Retry with rate=1 in the post-config burst when the F10T-aware fix lands. |
| TIM-TM2 | `CFG_MSGOUT_UBX_TIM_TM2_UART1=1` | ACK | DO PPS edge timestamps (fires only on EXTINT edges) |

Pre-existing (confirmed flowing without explicit re-enable in this
session, presumably enabled in flash from prior config or factory
default):

| Message | Confirmed flowing? | Evidence |
|---|---|---|
| RXM-RAWX  | yes | Position filter has 18-19 SVs/epoch |
| RXM-SFRBX | yes | Broadcast ephemeris populating |
| TIM-TP    | yes | qErr ASD@0.1Hz = 2.9218 ns/√Hz logged |
| NAV-PVT   | (not directly observed) | Engine doesn't consume; not tracked |
| NAV-SAT   | (not directly observed) | Engine doesn't consume; not tracked |

## NAK semantics — three categories

1. **Idempotent NAK**: receiver rejects the key but the underlying
   feature is already in the desired state.  Observed for
   `CFG_NAV2_OUT_ENABLED=1` on F10T (NAV2-PVT messages flow despite
   the NAK).  Likely persisted as enabled in BBR/Flash from earlier
   config or factory default.  **Workaround**: check before set, or
   accept the NAK and move on.

2. **Rate-cap NAK**: receiver rejects a specific rate value
   (decimation factor) but accepts the key at other rates.
   Observed for `CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1=5`; the message
   continues to flow at the receiver's default rate.  **Workaround**:
   retry at rate=1 for the next post-config burst revision.

3. **Hardware NAK**: receiver rejects the key because the underlying
   hardware doesn't exist.  Observed for all `_USB`-suffixed keys on
   F10T.  **No workaround**: use the correct port suffix.

The no-bundle CFG-VALSET policy (`docs/receiver-signals.md`) lets us
distinguish these categories at the wire.  Bundled VALSETs would NAK
the entire burst on any one of them, losing the diagnostic.

## Signal capability matrix

Not directly probed during the 2026-05-12 session (the receiver was
already signal-configured from earlier).  Per `F10TDriver` in
`scripts/peppar_fix/receiver.py`:

| Constellation | Primary (f1) | Secondary (f2) | Notes |
|---------------|-------------|----------------|-------|
| GPS | L1 C/A | L5 Q | Same as F9T-L5 profile |
| Galileo | E1 C | E5a Q | Same as F9T-L5 profile |
| BeiDou | **B1C P** | **B2a P** | Modernized signals — F10T tracks BDS-3 only; **NOT** B1I/B2I that F9T tracks |

The F10T's BDS profile is the key difference from F9T family.  See
`F10_BDS_SIG_NAMES` in `scripts/peppar_fix/receiver.py` for the
sigId table and the (3,5) collision with F9T's BDS-B2aI mapping.

## Other F10T constraints (from F10TDriver)

| Attribute | Value | Notes |
|---|---|---|
| `protver` | 42 | vs F9T's 27 / 29 |
| `default_baud` | 38400 | vs F9T's 115200/460800 |
| `supports_timing_mode` | False | No CFG-TMODE on F10T — no Survey-In, no fixed-position time-mode |
| `supports_l5_health_override` | False | The 0x10320001 override that F9T accepts is rejected here |

## References

- `docs/receiver-signals.md` — no-bundle CFG-VALSET policy (the
  prerequisite that made per-key F10T diagnosis possible)
- `docs/f9t-firmware-capabilities.md` — sibling doc for the F9T
  family; same methodology
- `scripts/peppar_fix/receiver.py` — `F10TDriver` class
- Dayplan `f10tMsgEnable-bravo` (2026-05-12) — diagnostic session
  + per-key burst data
- u-blox NEO-F10T Integration Manual (PROTVER 42) — vendor
  reference for keys + features
