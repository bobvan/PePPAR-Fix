# CFG-VALSET NAK Semantics — Three Categories

When a u-blox receiver NAKs a `CFG-VALSET` key, the NAK is not a
single binary signal.  Lab experience across F9T (TIM 2.20 / 2.25),
F9P (HPG 1.51), and F10T (TIM 3.01 / PROTVER 42) firmware shows
**three distinct meanings**, each with its own diagnostic value and
its own workaround.  Knowing which category a given NAK falls into
is the difference between "ignore it" and "the receiver is broken."

## Prerequisite: the no-bundle CFG-VALSET policy

This taxonomy only works if VALSETs are sent **one key per
message**.  When multiple keys are bundled into a single
CFG-VALSET, the receiver NAKs the entire burst on any one
rejected key — and the per-key identity that distinguishes the
categories below is lost in the wire-level "VALSET NAK."

Per-key VALSET is mandated by `docs/receiver-signals.md` and
enforced by `send_cfg()` / `_send_cfg_one()` in
`scripts/peppar_fix/receiver.py`.  Without it the rest of this
doc is unactionable.

## The three categories

### 1. Idempotent NAK — "already in the desired state"

The receiver rejects the key but the underlying feature is
already configured as the request would set it.  Often a
side-effect of persistence in BBR/Flash from prior configuration
or factory defaults, surfaced as a NAK because the firmware's
per-key handler treats "no change required" as an error path.

**Observed**:
- `CFG_NAV2_OUT_ENABLED=1` on F10T (TIM 3.01) — NAKs, but
  NAV2-PVT messages flow anyway.  Likely persisted as enabled
  in flash.

**Diagnostic value**: confirms the feature is live without
needing a separate read-back.  Combined with the downstream
data flowing, it's a non-event.

**Workaround**: check before set, or accept the NAK and
verify the downstream message stream.  Do not retry — the
NAK will repeat.

### 2. Rate-cap NAK — "this rate value is too high"

The receiver rejects a specific rate (decimation factor) but
accepts the same key at a different rate.  Messages typically
continue to flow at the receiver's default rate, so the
operational consequence is small; the NAK is a clue that the
firmware enforces an upper bound on that particular message.

**Observed**:
- `CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1=5` on F10T — NAKs at rate=5
  but messages flow at the default cadence.  Accepts rate=1.

**Diagnostic value**: tells you the receiver supports the
message but caps the decimation factor.  Distinguishes a
firmware quirk from a hardware-absent NAK (category 3).

**Workaround**: retry at the lower rate the receiver accepts.
On F10T this is the `nav_timegps_rate=1` class attribute on
`F10TDriver`.  Generalize to a per-driver `<message>_rate`
attribute when more such messages are discovered.

### 3. Hardware NAK — "the hardware that key talks to doesn't exist"

The receiver rejects the key categorically because the
underlying RF front-end, peripheral, or port doesn't physically
exist on this module.  Unlike categories 1 and 2 there is no
"the message flows anyway" — the feature is unreachable.

**Observed**:
- Every `_USB`-suffixed `CFG_MSGOUT_UBX_*` key on F10T — the
  NEO-F10T module has no USB peripheral, only UART1 (the
  `/dev/f10t` device on the host is the ArduSimple breakout
  board's FTDI USB-UART bridge, not a USB port on the receiver).
- Every `CFG_SIGNAL_GPS_L5_ENA=1` on F9P-15 (HPG 1.51) — the
  module's RF front-end is L1+L2 band only; no 1176.45 MHz
  hardware to enable.
- `CFG_SIGNAL_GAL_E5B_ENA=1` on L5-capable F9T variants
  (vpManager_07=1) and vice-versa for E5a on L2-only variants.
- `CFG_SIGNAL_GPS_L2C_ENA=1` on ZED-F9T-20B (TIM 2.25) — the
  -20B firmware drops L2C entirely.

**Diagnostic value**: distinguishes hardware variants that
report identical firmware/module strings.  The F9T family in
particular ships two physically distinct L5/L2 RF builds under
the same `MOD=ZED-F9T, FWVER=TIM 2.20, PROTVER=29.20` —
hardware NAKs are the only software-visible way to tell them
apart.

**Workaround**: there is none at the key level.  Use the
correct key (e.g. `_UART1` instead of `_USB`), pick a different
driver profile (`F9TDriver` vs `F9TL5Driver` vs `F9TL2E5bDriver`),
or accept that the feature is unavailable on this hardware.

## When the category is unclear — disambiguation steps

Sometimes a NAK could be category 1 or category 3 (the feature
might be off, or it might not exist).  Two cheap checks:

1. **Look for the downstream data**: if a `CFG_MSGOUT_*`
   NAKs but the corresponding UBX message is observed in the
   stream, the key is category 1 (idempotent) — the message
   flow is already enabled.  If nothing flows, suspect
   category 2 or 3.
2. **Try a different rate**: if the same `CFG_MSGOUT_*` key
   ACKs at rate=1 but NAKs at rate=5, it's category 2
   (rate-cap).  If it NAKs at every rate including 1, it's
   category 3 (hardware-absent).
3. **Check `MON-HW3 vpManager_07` for F9T units**: bit 7 = 0
   means no 1176.45 MHz front-end, so every L5/E5a/B2a NAK is
   category 3 regardless of firmware string.

## Why this matters operationally

Without the taxonomy, a NAK looks like an error and either gets
retried in a loop (category 1 — wastes time, still NAKs), or
gets logged as a configuration failure (category 2 — when in
fact messages are flowing fine), or gets reported as a firmware
bug (category 3 — when the hardware is the actual constraint).

With the taxonomy, the NAK is a signal:
- Category 1 → confirm downstream, then ignore.
- Category 2 → adjust the rate in the driver class.
- Category 3 → adjust the driver profile or the request itself.

The PR #37 follow-up `f10tTimegpsRateDriverAware-main` is a
worked example of category 2 → per-driver attribute.

## References

- `docs/f9t-firmware-capabilities.md` — F9T family capability
  matrix; many of the hardware NAKs documented there are
  category 3 instances.
- `docs/f10t-firmware-capabilities.md` — F10T capability matrix;
  the canonical examples for all three categories.
- `docs/receiver-signals.md` — no-bundle CFG-VALSET policy,
  the wire-level prerequisite that makes per-key NAK identity
  recoverable.
- `scripts/peppar_fix/receiver.py` — `send_cfg()` /
  `_send_cfg_one()` implementation; `ReceiverDriver`
  subclasses encode hardware-specific category-2 / category-3
  workarounds as class attributes.
