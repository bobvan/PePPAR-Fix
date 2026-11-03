# ppsfix as a patch — rebase onto the stock igc of the kernel you will run

**This supersedes the per-kernel driver snapshots** in
`../igc-timehat-edge*/`.  Use `./rebase.sh` for any new kernel.

ppsfix is **9 hunks across 4 files** (`ppsfix.patch`, 216 lines).  The
snapshots carried a whole 27-file driver copy, which is why this matters.

## Why the snapshot approach was actively harmful

A snapshot is frozen at the upstream igc of the day it was taken.  Carry it
to a newer kernel and it **silently reverts every upstream igc fix made
since**, because the whole file is replaced.

Measured 2026-11-03 by diffing `igc-6.18.34-ppsfix.1` (recovered from MadHat)
against the exact stock igc of `linux-source-6.18 = 1:6.18.50-1+rpt1` — **25
hunks, of which only 9 are ppsfix.** The other 16 roll back upstream work:

| Upstream change the snapshot reverted | File |
|---|---|
| `igc_fpe_handle_mpacket(…, pktbuf + pkt_offset)` → `pktbuf` — a packet-offset bug | `igc_main.c` |
| `skb_put_padto(skb, 17)` → open-coded length check | `igc_main.c` |
| XSK TX timestamp cleanup on ring teardown — a leak fix | `igc_main.c` |
| IEEE 802.1Qbv schedule restore after queue reinit | `igc_main.c` |
| **TX-timestamp interrupt hardware-bug workaround** (`txstmpl_old` baseline; "future timestamp interrupts won't fire unless TXSTMPH_0 is read") | `igc_ptp.c` |
| `igc_ptp_clear_xsk_tx_tstamp_queue()` deleted outright | `igc_ptp.c` + `igc.h` |
| `IGC_TXPB0SIZE` 5 → 7, undoing a fix citing the I225/I226 manual §7.5.4 | `igc_defines.h` |
| TSN qdisc guard reverted to the older `strict_priority_enable` form | `igc_ethtool.c` |
| TX buffer type / error handling / `__netif_tx_unlock` | `igc_tsn.c` |

**PPS worked fine throughout**, which is exactly why nobody noticed for two
months.  The reverted TX-timestamp interrupt workaround is the one to care
about: it sits in the same path as `../igc-adjfine-fix/`, so the snapshot was
quietly removing an upstream mitigation while we debated whether to keep our
own.

The snapshot also *added* three things stock lacks — `napi_synchronize()`
before `napi_disable()`, and two `pci_save_state()` calls.  Not ppsfix, not
upstream.  The rebase drops them back to stock; if any turns out to matter,
add it deliberately as its own patch rather than as snapshot residue.

## What ppsfix actually is (the 9 hunks)

| Hunk | File | Change |
|---|---|---|
| 1 | `igc_main.c` | `edge_check_delay_us` / `edge_check_invert` module params |
| 2 | `igc_main.c` | `igc_sdp_val[]` — SDP data-value register table |
| 3 | `igc_main.c` | the EXTTS edge filter: read the GPIO level after a delay and drop falling edges |
| 4 | `igc_ptp.c` | EXTTS: bypass the strict dual-edge requirement, force rising-edge only |
| 5 | `igc_ptp.c` | per-channel `ts0/ts1` pin + flags tracking |
| 6 | `igc_ptp.c` | PEROUT: drop `500000000LL` from the frequency-mode condition so 1 Hz uses Target Time mode, not a 500 ms square wave |
| 7–8 | `igc.h`, `igc_defines.h` | `ts0_pin`/`ts0_flags`/`ts1_pin`/`ts1_flags` fields; `IGC_CTRL_SDP{0,1}_VAL` and `IGC_CTRL_EXT_SDP{2,3}_VAL` |

Hunks 1–5 and 7–8 are the PPS **input** path; hunk 6 is the PPS **output**
path.  Neither depends on PTP rate — see
`../../docs/igc-kernel-patches.md` for why that distinguishes ppsfix from the
adjfine patch, which *is* rate-dependent and is deliberately not applied.

## ⚠ A public git tag is not the kernel you are running

The first attempt used `raspberrypi/linux` at tag `stable_20260911`, whose
`Makefile` reads 6.18.50 — the exact version of the installed kernel package.
**It still did not build**: that tree's igc calls
`xsk_buff_get_metadata(pool, addr, …)` with three arguments, while the
installed `6.18.50+rpt` headers declare two.  The tag was ahead of the shipped
kernel.

`rebase.sh` therefore uses **`linux-source-<series>`**, which is the tarball
the kernel package was actually built from.  Verified: its igc calls the
2-argument form and matches the headers.

## Usage

```bash
sudo ./rebase.sh                     # for the running kernel
sudo ./rebase.sh 6.18.50+rpt-rpi-2712   # for a kernel you are about to boot
```

Step 4 of the script **verifies the result rather than trusting exit codes** —
it greps for each ppsfix marker and fails closed if any is missing, and
confirms the PEROUT special case is gone.  That guard exists because `patch`
has been observed here reporting success (exit 0, no `.rej`) while silently
dropping a hunk; see `../igc-timehat-edge-6.18/README.md`.

After rebooting, confirm on the live module:

```bash
cat /sys/module/igc/parameters/edge_check_delay_us   # -> 20
cat /sys/module/igc/parameters/edge_check_invert     # -> 0
```

## First use

Built and installed on TimeHat 2026-11-03 as `igc/6.18.50-ppsfix.2` for
`6.18.50+rpt-rpi-2712` and `-v8`, replacing the recovered snapshot.  Verified:
all 9 ppsfix hunks applied with zero rejects, the built module exposes both
module parameters, and every upstream fix listed above is present in the
rebased source.  `igc/6.12.0-ppsfix.1` is left installed for 6.12.75 as the
fallback kernel's module.
