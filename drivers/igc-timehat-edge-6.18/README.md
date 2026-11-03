# Intel igc PPS fix — kernel 6.18, aarch64 (RPi 5 + TimeHAT I226)

Patched igc driver for the 6.18 kernel line, carrying the **ppsfix** patch set
only.  See [`../../docs/igc-kernel-patches.md`](../../docs/igc-kernel-patches.md)
for the full patch inventory and the deployment checklist.

DKMS package: **`igc/6.18.34-ppsfix.1`**

## ⚠ This tree deliberately does NOT carry the adjfine patch

`drivers/igc-adjfine-fix/` (Patch 2) is **intentionally absent** here.  That is a
decision of Bob's, dated **2026-11-03**, not an oversight:

- The adjfine race needs a TIMINCA write to collide with an **in-flight TX
  timestamp capture**.  TX timestamps come from PTP transmit, so the collision
  probability scales with the PTP rate.
- The lab ran 6+ hosts at 128 updates/s when the race was discovered.  Today
  TimeHat is the only PTP host, `serverOnly`, at `logSyncInterval 4` — **one
  Sync every 16 seconds** — with no clients.  That is ~12,000× lower.
- Measured on TimeHat over 26 days of uptime: `tx_hwtstamp_timeouts: 0`,
  `tx_hwtstamp_skipped: 0`, zero `Tx timestamp timeout` in dmesg.

**The decision is conditional on that rate.**  If PTP clients return and start
sending Delay_Req at 1/s each, the TX-timestamp rate climbs and the adjfine
patch becomes relevant again.  The tripwires below are the ones to watch.

**ppsfix is not rate-dependent and must stay.**  It fixes the PPS *input* path
(stock igc delivers both edges of the 100 ms GPS pulse, so a consumer picks the
wrong one about half the time) and the PEROUT 1 Hz frequency-mode bug.  Neither
has anything to do with PTP traffic, and both sit on the measurement path the
lab relies on below 10 ns.

## Provenance

This tree was **not written here** — it was recovered, because it existed in
exactly one place and was not in version control:

| | |
|---|---|
| Found on | MadHat, `/usr/src/igc-6.18.34-ppsfix.1`, dated 2026-09-09 |
| Archived by | BBGresearch, 2026-11-03, to `gt:~/bbg-archive/igc-src/igc-6.18.34-ppsfix.1-from-MadHat-20261103.tar.gz` |
| Archive sha256 | `fb50dad0afe424d6f79f6bf1c79ee88c37049f4cece800171ff2e6d38befe783` (verified independently on gt and on MadHat) |
| Committed here | 2026-11-03, `src/` + `dkms.conf` byte-identical to the archive |

MadHat itself never exercised this module — its NIC is an **I210** (`igb`), so
`igc` never loads there.  That is why the missing adjfine patch went unnoticed
for two months: the tree was inert on the only host that had it.

The archive carried its own `README.md` titled *"for Kernel 6.12"* with
`6.12.0-ppsfix.1` install commands — a copy-paste carryover from the 6.12 tree.
It is not included; this file replaces it.

## Install

```bash
sudo ./build-and-install.sh
```

The script stages `intel-igc-ppsfix/` into `/usr/src/igc-6.18.34-ppsfix.1/`,
then runs `dkms add` / `build` / `install`.

To build against a kernel you are **not currently running** — the safe
pre-flight before a kernel upgrade:

```bash
sudo apt-get install -y linux-headers-<target version>   # versioned pkg, NOT the metapackage
sudo dkms build -m igc -v 6.18.34-ppsfix.1 -k <target version>
```

## Verification

After any install or kernel change, check the **signal, not the exit code**:

```bash
# ppsfix present — the two module parameters stock igc does not have
cat /sys/module/igc/parameters/edge_check_delay_us   # -> 20
cat /sys/module/igc/parameters/edge_check_invert     # -> 0

# the DKMS module is the loaded one, not stock
cat /sys/module/igc/srcversion
dkms status | grep igc

# adjfine deliberately ABSENT — this is expected to be 9, not 12
sudo grep -c TSYNCTXCTL /usr/src/igc-6.18.34-ppsfix.1/src/igc_ptp.c
```

### Tripwires for the adjfine decision

Watch these; if any fires, the rate assumption above has stopped holding:

```bash
sudo ethtool -S eth1 | grep -E 'tx_hwtstamp_timeouts|tx_hwtstamp_skipped'  # -> 0
sudo dmesg | grep -c 'Tx timestamp timeout'                                # -> 0
sudo journalctl -u ptp4l@eth1 | grep -c FAULTY   # startup only; 24 s cycling = the failure
```

## ⚠ Do not port the adjfine patch with `patch(1)`

If the decision is ever reversed, note that `patch` **reports success while
silently dropping the hunk that matters**.  Applying
`../igc-adjfine-fix/0003-igc-disable-tx-tstamp-around-timinca.patch` to this
tree gives:

```
patching file igc_ptp.c
Hunk #1 succeeded at 48 with fuzz 2 (offset 1 line).
$ echo $?   -> 0          $ ls *.rej   -> none
$ grep -c TSYNCTXCTL igc_ptp.c   -> still 9      # unchanged!
```

Hunk #1 adds only two local declarations, so its "success" means nothing on its
own; Hunk #2, which inserts the lock and the TX-timestamp disable, is dropped
without a word.  **This is the most likely way the 6.18 port lost the patch in
the first place.**  Verify by reading `igc_ptp_adjfine_i225()` and confirming it
contains `spin_lock_irqsave(&igc->tmreg_lock, flags)` and the
`IGC_TSYNCTXCTL_ENABLED` clear/restore around the `IGC_TIMINCA` write.
