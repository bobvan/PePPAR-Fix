# igc Kernel Patches — Inventory and Deployment Status

The Intel i225/i226 igc driver ships with two bugs that affect PePPAR
Fix.  We carry two independent patch sets in `drivers/` to fix them.
Both must be applied together for correct operation on any i226 host
that runs PHC discipline + ptp4l.

> ## ⚠ 2026-10-05 — A CONCRETE TRIGGER FOR THE TX HANG, AND IT IS NOT A SOFTWARE WORKLOAD
>
> **Measured by BBGresearch (Caleb) from TimeHat's dmesg, verified here.**
> On 2026-10-05 the i226 logged **23 `Detected Tx Unit Hang` lines across 6
> `Reset adapter` clusters** between 08:48 and 10:45 local — each within
> minutes of a **USB plug event** — with `hwmon3: Undervoltage detected!` at
> 10:45:16.  The host is a **Pi 5 with `usb_max_current_enable=0`** (USB capped
> at 600 mA) and `vcgencmd get_throttled` = **`0x50000`** (under-voltage *has*
> occurred, soft temperature limit *has* occurred).
>
> In the preceding 27 days there was **exactly one** other cluster: 2026-09-08
> 16:46, which is ~19 minutes before another project's tool ran on that host.
>
> **Why this reframes `igcWedgeAttribution`: the campaign's null result is now
> explained.**  Bravo's 2026-08-26/27 campaign ran ten arms varying adjfine
> rate, TX rate and driver version (patched vs stock), and **every single arm
> returned `rc=0` — no timeouts observed, on any arm, including stock.**  It
> hammered TX timestamping in software and could not reproduce the wedge.  The
> campaign window shows **no Tx Unit Hang at all** in dmesg.
>
> So the campaign varied the wrong axis.  **The correlate is a power/USB
> event, which a scripted software sweep never produces.**
>
> **This also resolves the self-contradiction flagged in this doc in
> September.**  The incident table said *"no adjfine running at time of first
> timeout cascade"* while its root-cause column said the strand happened
> *"during adjfine."*  Both can be true if **adjfine is irrelevant** — which is
> what a power-side trigger implies.  The contradiction was a symptom of
> attributing a power event to whatever software happened to be running.
>
> **Methodological consequence, and it is general: `/sys/class/ptp/ptpN/pins/`
> reports DRIVER CACHE, not hardware state.**  Every igc reset zeroes
> `TSAUXC`/`TSSDP`, so a PEROUT un-arms *in hardware* while sysfs keeps
> printing it armed.  Any decision of the form "the pins read as armed, so X is
> unsafe" is resting on a cached value.  Our own pin reads on this host in
> September were therefore not evidence of hardware state, and the PEROUT
> evidence we thought we were preserving had already been destroyed by resets
> before it was formally released.
>
> **Open:** Caleb has asked Bob for a 5 V 5 A supply.  If the trigger is
> supply margin, that is the fix — and it would mean the ~44 min MTBF this doc
> records is a property of an under-supplied host, not of the part.  **Do not
> quote that MTBF as an i226 characteristic until this is settled.**

> ## ⚠⚠ 2026-10-06 — eth1's RX HAS BEEN DEAD FOR 40 DAYS, AND IT CONFOUNDS THE CAMPAIGN
>
> Live wedged state captured before any recovery, at
> `gt:~/gt/peppar-fix-data/timehat-igc-wedge-live-20261006/`.  Reported by
> BBGresearch's Caleb as "dead receive path"; measured here:
>
> ```
> boot            2026-08-26 18:16:51
> undervoltage    2026-08-26 18:16:57    <- SIX SECONDS AFTER BOOT
> uptime          40.6 days
> rx_missed       30,656,721 = 8.74/s averaged over the WHOLE uptime
> RDH[0-3]        0 0 0 0    <- RX descriptor head never advanced, any ring
> ethtool rx_pkts 4477       <- hardware MAC accepted a few frames
> netdev RX pkts  0          <- driver delivered NONE upstream
> ```
>
> **The receive path never initialised.**  The NIC came up during a brownout six
> seconds after boot and `RDH` has sat at zero on all four rings for 40.6 days.
> This is **not** a progressive wedge, **not** the documented TX-timestamp/EXTTS
> failure, and **not** caused by the transmissions that revealed it.
>
> Note the diagnostic trap: `ip -s link` reports RX 0/0 while `ethtool -S`
> reports `rx_packets: 4477`.  **Hardware counters and netdev stats disagree**,
> because the former reads the MAC's register and the latter the driver's ring
> counters.  Reading only one of them gets you the wrong layer.
>
> **⚠ THIS CONFOUNDS `igcWedgeAttribution`.**  Bravo's ten-arm campaign ran
> 2026-08-26 21:28 → 2026-08-27 09:37 — about **three hours after this boot, on
> an adapter whose RX was already dead.**  Every arm ran on half-broken
> hardware.  That is a second, independent reason its `rc=0` results cannot
> support a conclusion, on top of its having varied the wrong axis.  **Any
> re-run must first verify the NIC is healthy** — `rx_missed_errors` near zero
> and netdev RX counting — which nobody checked, including me.
>
> **The power problem is worse and more autonomous than recorded yesterday.**
> Oct 5 alone: **36 `Reset adapter` and 40 `Undervoltage detected`**.  The
> evening events (19:07, 20:37–21:56) happened with **no USB activity**, so the
> brownouts are not merely USB-plug-triggered — the host browns out on its own.
> That strengthens the case for the 5 V 5 A supply from "likely trigger" to
> "this host's power is inadequate, full stop."

## Patch inventory

### Patch 1: TimeHAT PPS dual-edge fix ("ppsfix")

**Source directory**: `drivers/igc-timehat-edge/` (kernel 6.12, aarch64)
and `drivers/igc-timehat-edge-6.8/` (kernel 6.8, x86-64).

**What it fixes**:

1. **EXTTS dual-edge timestamping**: Stock igc delivers both rising
   and falling PPS edges to userspace.  With a 100 ms wide GPS PPS
   pulse, this gives 2 events/sec 100 ms apart, and the engine
   randomly picks the wrong edge half the time.  The patch adds a
   GPIO-level filter in the interrupt handler that drops falling edges
   before they reach userspace.

2. **PEROUT 1 Hz frequency-mode bug**: Stock igc has a special case
   that, when asked for a 1 Hz periodic output, switches to "frequency
   mode" producing a 50% duty-cycle square wave (500 ms HIGH / 500 ms
   LOW) instead of a proper PPS pulse.  The patch removes the special
   case so 1 Hz uses Target Time mode.

**Detection**: The patched driver exposes module parameters
`edge_check_delay_us` and `edge_check_invert` that the stock driver
does not have.  Check with:

```bash
cat /sys/module/igc/parameters/edge_check_delay_us  # → "20" if patched
```

**DKMS package version**: `igc/6.12.0-ppsfix.1` (aarch64) or
`igc/6.8.0-ppsfix.1` (x86-64).

### Patch 2: adjfine TX timestamp race fix

**Source directory**: `drivers/igc-adjfine-fix/`

**What it fixes**: `igc_ptp_adjfine_i225()` writes the `IGC_TIMINCA`
register without synchronization.  When the hardware captures a TX
timestamp while TIMINCA is changing, the captured value is corrupt.
After 15 seconds the driver logs `Tx timestamp timeout` and the skb
is freed.  Repeated timeouts wedge the EXTTS subsystem, breaking PPS
capture and causing ptp4l to cycle through MASTER → FAULTY endlessly.

**Trigger**: Any combination of PHC frequency discipline (peppar-fix
calling `adjfine()` at 1 Hz) and PTP hardware timestamping (ptp4l
sending Sync packets).  At `logSyncInterval -7` (128 Hz): ~30 min
MTBF.  At `logSyncInterval 0` (1 Hz): ~64 hour MTBF.

**The fix** (v3): Hold `tmreg_lock`, temporarily disable TX
timestamping via `TSYNCTXCTL`, write TIMINCA, re-enable.  The disable
window is ~1 µs — any TX packet in that window simply gets no hardware
timestamp (no corruption, no timeout, no wedge).

**Patch file**: `drivers/igc-adjfine-fix/0003-igc-disable-tx-tstamp-around-timinca.patch`

**Detection**: The patched `igc_ptp_adjfine_i225()` contains
`TSYNCTXCTL` references.  The simplest runtime check: if the driver is
patched with the ppsfix AND has adjfine fix, `igc_ptp.c` has 12+
`TSYNCTXCTL` references in the DKMS source (vs 9 for ppsfix-only).
At runtime, the definitive test is absence of `Tx timestamp timeout`
in `dmesg` during PHC discipline + ptp4l operation.

**Upstream status**: Submitted to intel-wired-lan list.  Maintainer
suggested `tmreg_lock` (incorporated in v3).  Discussion ongoing.  See
`drivers/igc-adjfine-fix/upstream-submission.txt` and
`upstream-reply-v3.txt`.

**Patch 2 limitations (discovered 2026-04-15)**: The v3 patch reduces
but does not eliminate TX timestamp failures.  The TSYNCTXCTL disable
window (~1 µs per adjfine call) can strand an in-flight TX timestamp
capture.  A single stranded slot sits occupied for 15 seconds
(IGC_PTP_TX_TIMEOUT).  During those 15 seconds, ptp4l continues
attempting TX timestamps which may also strand, cascading into a
permanent wedge where all 4 TX timestamp slots are occupied and every
subsequent attempt times out.  Observed MTBF on TimeHat: ~44 minutes
at 1 Hz adjfine + 1 Hz ptp4l sync.  Once wedged, the subsystem does
not self-recover — it stays broken until driver reload or reboot.

The timeout handler (`igc_ptp_tx_hang()`) clears stranded slots after
15 seconds, but the cascade happens faster than the recovery.  A more
robust fix would need to either: (a) make `igc_ptp_tx_hang()` clear
ALL pending slots atomically when any timeout fires, preventing the
cascade, or (b) add a brief delay after TSYNCTXCTL re-enable to let
any in-flight capture complete before the next TX attempt.

Additionally, EXTTS stops delivering events after the TX timestamp
subsystem wedges, even though EXTTS uses a different hardware path.
This kills PPS delivery and prevents the servo from running.  The
mechanism linking TX timestamp wedge to EXTTS failure is not yet
understood — it may be shared interrupt or register state.

## How patches are composed

Both patches target `igc_ptp.c` in the same kernel generation (6.12).
The ppsfix is a complete vendored igc source tree.  The adjfine fix is
a patch applied on top.  The intended layering:

```
Stock Raspberry Pi OS igc        (ships with kernel)
└── TimeHAT PPS fix              (drivers/igc-timehat-edge/)
    └── adjfine TX tstamp fix    (drivers/igc-adjfine-fix/0003-*)
```

The adjfine patch must be applied to the ppsfix vendored source tree in
`drivers/igc-timehat-edge/intel-igc-ppsfix/src/igc_ptp.c`, then the
combined tree is built and installed via DKMS as a single module.

## Host applicability

| Host | Has i226? | Needs igc patches? | Notes |
|------|-----------|---------------------|-------|
| TimeHat | Yes (TimeHAT board, PCIe) | **Yes** | Runs ptp4l as GM + PHC discipline |
| MadHat | Yes (TimeHAT board, PCIe) | **Yes** | Runs PHC discipline; ptp4l planned |
| clkPoC3 | No (Pi 4, bcmgenet only) | No | DKMS tree installed but igc never loads |
| ocxo | Yes (E810-XXVDA4T) | No — uses `ice` driver | Different driver, different bugs |

## Deployment verification checklist

Run on each i226 host after any driver install or kernel update:

```bash
# 1. Confirm DKMS-installed igc is loaded (not stock)
cat /sys/module/igc/srcversion
# Should match the DKMS-built srcversion, not the stock kernel module.

# 2. Confirm ppsfix patch (edge filter)
cat /sys/module/igc/parameters/edge_check_delay_us
# Expected: "20" — if missing, ppsfix is NOT installed.

# 3. Confirm adjfine patch (TSYNCTXCTL in adjfine)
sudo grep -c TSYNCTXCTL /usr/src/igc-6.12.0-ppsfix.1/src/igc_ptp.c
# Expected: 12+ — if 9, adjfine patch is NOT applied.
# (This checks the DKMS source; it implies the built module has it,
# but only if DKMS was rebuilt after patching.)

# 4. Runtime confirmation: no TX timestamp timeouts
sudo dmesg | grep -c "Tx timestamp timeout"
# Expected: 0 after reboot with both patches.

# 5. ptp4l is not cycling through FAULTY (if running)
sudo journalctl -u ptp4l --since "1 hour ago" | grep -c FAULTY
# Expected: 0 (or only at startup).
```

## Incident history

| Date | Host | Symptom | Root cause | Resolution |
|------|------|---------|------------|------------|
| 2026-04-01 | TimeHat | Tx timestamp timeout every ~30 min | adjfine race (Patch 2 missing) | Discovered bug, wrote patches |
| 2026-04-07 | MadHat | Dual-edge EXTTS, servo demands 3 Mppb | Missing ppsfix (Patch 1) | Installed ppsfix DKMS |
| 2026-04-14 | TimeHat | 1,266 Tx timestamp timeouts over 18h; ptp4l MASTER→FAULTY loop every 24s; AntPosEstThread correlation gate stall | adjfine race — ppsfix installed but adjfine patch never applied to ppsfix source tree | Applied adjfine patch to ppsfix tree (this fix) |
| 2026-04-15 | TimeHat | TX timestamp timeouts resume 44 min after boot despite v3 patch; EXTTS wedges; ptp4l MASTER→FAULTY every 24s; no adjfine running at time of first timeout cascade | **Not the adjfine race.** v3 patch protects adjfine but the TSYNCTXCTL stranding failure cascades: one stranded slot during adjfine sits for 15s, during which ptp4l TX attempts also strand, wedging the subsystem permanently. Once wedged, every subsequent TX timestamp fails even without adjfine. See "Patch 2 limitations" below. | Investigation ongoing |

## Old / stale DKMS trees

The Intel out-of-tree driver `igc/5.4.0-7642.46` was used for early
adjfine patch development.  It has the adjfine fix but NOT the ppsfix.
It should be removed from any host where it exists — it's half-patched
and the 5.4.0 source base predates the ppsfix changes, making it
unsuitable as a production driver.  The patch files in
`drivers/igc-adjfine-fix/` preserve the work for posterity.
