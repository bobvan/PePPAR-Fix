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

> ## ✅ 2026-10-07 — RESOLVED BY A POWER SUPPLY.  THE ~44 min MTBF IS NOT AN i226 PROPERTY.
>
> **A 5 V 5 A supply fixed it completely.**  Measured by BBGresearch's Caleb
> 45 s after a clean boot, and the before/after is unambiguous:
>
> | | before (3 A supply) | after (5 A supply) |
> |---|---|---|
> | `rx_missed_errors` | **32,099,512**, +17/s | **0**, +0 over 10 s |
> | netdev RX packets | **0** | **1252 → 1470 in 10 s — counting** |
> | `vcgencmd get_throttled` | `0x50000` | **`0x0`** |
> | firmware PSU `max_current` | 3000 mA | **5000 mA** |
> | `usb_max_current_enable` | 0 | **1** |
> | Reset adapter / Tx Hang / Undervoltage since boot | 42 / 85 / 40 | **0 / 0 / 0** |
>
> **The i226 is fine.**  The 42-day dead receive ring was a **boot-time brownout
> artifact** — the card came up during an undervoltage six seconds after boot
> and its RX ring never initialised.  On adequate supply it initialises
> correctly and the counters stay clean.
>
> **Therefore the ~44 min MTBF recorded in this document is a property of an
> under-supplied host, not of the part.**  Do not quote it as an i226
> characteristic.  Everything downstream of it needs re-examining — see
> `docs/split-clock-architecture.md` §3.3, where the i226 is disqualified for
> the rooftop-ear design on exactly that figure.
>
> **Honest limit on the evidence: N = 1 host.**  The obvious corroboration —
> does another i226 on a marginal supply show the same thing — **is not
> available**, because MadHat turns out to carry an **Intel I210** (reworked
> I210-T1, `igb`), not an i226, and it is already on an adequate supply
> (`get_throttled = 0x0`, `usb_max_current_enable=1`, 16 days up, zero
> undervoltage, zero resets).  So this is a strong single-host result — same
> host, same card, one variable changed, a 42-day fault cleared completely —
> and not yet a general claim about the part on any host.
>
> **⚠ WHAT THIS DOES *NOT* YET SHOW — and the test that would close it.**
> The before/after proves the **boot-time RX-ring fault** is power-related.  It
> does **not** prove the **TX-timestamp wedge** is, because the wedge's stated
> trigger is a *workload* — 1 Hz adjfine concurrent with 1 Hz ptp4l sync — and
> the post-swap host has been **idle**.  Confirmed healthy at 6 minutes uptime
> (`rx_missed` 0, RX counting 1470 → 7777, `get_throttled` 0x0, zero fault
> lines), but idle minutes are not the same experiment.
>
> **The closing test, which needs no new tooling:** run
> `tools/igc_tx_timeout_repro.py` — it already drives both sides of the race —
> on the healthy host under the production configuration, for **well past the
> old ~44 min MTBF**.  Clean for several hours ⇒ the wedge was power all along
> and the verdict in
> [`split-clock-architecture.md`](split-clock-architecture.md) §3.3 can be
> rewritten rather than merely hedged.  Wedges on good power ⇒ there are two
> distinct faults, the patches keep their justification, and only the *MTBF
> figure* was contaminated.
>
> Needs the i226, which is currently BBGresearch's — so it is Bob's call, not a
> thing to schedule unilaterally.
>
> **What the patches were for is now in question too.**  Both patch sets in
> this document address real, separately-observed defects (the dual-edge
> timestamping bug and the adjfine `TSYNCTXCTL` race).  But the *wedge* they
> were deployed against, and whose MTBF justified the urgency, was a power
> fault.  H1 (orphaned timestamp slots) and H2 (our v3 read-modify-write)
> were never needed to explain it.
>
> ---
>
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

## Which patches are deployed, and why — decision 2026-11-03

**Bob's call: ppsfix stays everywhere; adjfine is dropped while PTP runs slow.**
Recorded so nobody re-applies adjfine reflexively, and — more importantly — so
nobody drops ppsfix by treating the two as one decision.

### The two patches have different triggers

| | Patch 1 — ppsfix | Patch 2 — adjfine |
|---|---|---|
| Fixes | PPS **input** (EXTTS dual edge) + PEROUT 1 Hz frequency mode | TIMINCA write racing an in-flight **TX timestamp** capture |
| Triggered by | the GPS pulse itself | **PTP transmit rate** |
| Rate-dependent? | **No** | **Yes** |
| Verdict | **keep** | **drop, conditionally** |

**Why adjfine is droppable now.**  The race needs an adjfine call to land while
the hardware is capturing a TX timestamp, and TX timestamps come from PTP
transmit.  When the race was discovered the lab ran **6+ hosts at 128
updates/s** (~768 sync events/s).  Measured 2026-11-03:

```
TimeHat   ptp4l@eth1.service   active 3 weeks 5 days   serverOnly 1
          logSyncInterval 4  ->  one Sync every 16 SECONDS (0.0625/s)
          logAnnounceInterval 1, logMinDelayReqInterval 0
          clients: NONE (journal silent since the Oct 7 boot)
MadHat    ptp4l inactive
ptpmon / otcBob1 / ptBoat / PiFace / PiPuss / bbb : unreachable
```

That is **~12,000× below** the rate that produced the failure.  The engine still
calls adjfine at servo cadence, but there is nothing in flight to collide with.
Corroboration from 26 days of uptime on the patched driver:
`tx_hwtstamp_timeouts: 0`, `tx_hwtstamp_skipped: 0`, zero `Tx timestamp
timeout` in dmesg, and the only two `FAULTY` events are from link-up at boot.

**The decision is conditional on the rate, not permanent.**  `logMinDelayReqInterval
0` lets each client Delay_Req at 1/s, and every Delay_Resp is a TX timestamp —
so returning clients raise the rate directly.  Watch the tripwires in the
checklist below; if they fire, re-apply adjfine.

**Why ppsfix is not droppable.**  It is not a PTP fix at all.  Stock igc hands
userspace **both edges** of the 100 ms GPS pulse — 2 events/sec, 100 ms apart,
and the engine picks the wrong one about half the time — and separately turns a
1 Hz PEROUT request into a 500 ms square wave.  Both sit on the PPS measurement
path, which is the path the lab depends on below 10 ns.  PTP rate is irrelevant
to either.

> Worth noting the shape: "a 100 ms pulse reaches the consumer as two edges and
> the wrong one is taken ~50% of the time" is **the same bug as the TM2 +100 ms
> stray** chased on BBGresearch's hosts through late October — different
> subsystem (igc EXTTS vs F9T TIM-TM2), same 100 ms (the F9T timepulse width),
> same ~50%.  Two independent instances of one failure shape.

### Deployment after this decision

| Kernel line | Tree | Contents |
|---|---|---|
| 6.12 | `drivers/igc-timehat-edge/` | ppsfix + adjfine (TSYNCTXCTL = 12) |
| 6.18 | `drivers/igc-timehat-edge-6.18/` | **ppsfix only** (TSYNCTXCTL = 9, by design) |
| 6.8 (x86-64) | `drivers/igc-timehat-edge-6.8/` | ppsfix |

So the ppsfix-only 6.18 tree — recovered from MadHat and committed 2026-11-03,
see that directory's README for provenance — is **the intended module for a
6.18 host**, not a defective one.  On 6.18, `grep -c TSYNCTXCTL` returning 9 is
the expected result.

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
