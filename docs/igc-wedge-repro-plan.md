# igc TX-timestamp wedge — reproduction and attribution plan

**Status**: work item, not started.  Owner TBD (bravo or delta).
**Hardware**: `hw:TimeHat` (+ optionally `hw:MadHat`, `hw:ptpmon`).
**Motive**: i226 is currently written off for the split-clock architecture
([split-clock-architecture.md](split-clock-architecture.md) §3.3) on the
strength of a ~44 minute wedge MTBF.  i226 is too widely deployed — Timebeat
included — to condemn on an unattributed observation.  This plan attributes it.

---

## 1. Why this is being re-opened

[`igc-kernel-patches.md`](igc-kernel-patches.md)'s incident table contradicts
itself on the 2026-04-15 event.  Symptom column:

> …despite v3 patch; EXTTS wedges; **no adjfine running at time of first
> timeout cascade**

Root-cause column, same row:

> one stranded slot **during adjfine** sits for 15 s, during which ptp4l TX
> attempts also strand

Read strictly the row is not self-contradictory — it tells a **two-phase**
story: an earlier strand *during* adjfine wedges the subsystem, and the
*observed* cascade happens later with no adjfine running, because "once wedged,
every subsequent TX timestamp fails even without adjfine."  The contradiction
appears only if "first timeout cascade" is read as the initiating event.

So the honest statement is **ambiguity, not contradiction** — but the two-phase
mechanism it asserts has never been demonstrated, and the row does not say
which reading is meant.  Either way arm A settles it, and settles the *stronger*
reading too: if adjfine is off from boot and it still wedges, there was never an
adjfine phase to do the initiating strand, so the two-phase story dies with the
simple one.

The MTBF numbers in that doc are also not comparable: ~30 min (2026-04-01),
~64 h "at 1 Hz" (Patch 2 trigger section), ~44 min "at 1 Hz adjfine + 1 Hz
ptp4l" (2026-04-15) come from different sync rates, patch states, dates and
small event counts.  They cannot support a claim that the patch helped or hurt.
**Do not carry any of them into the writeup as a baseline.**

Two candidate mechanisms are on the table, both unverified:

- **H1 — orphaned slots.**  `igc_ptp_tx_hang()` does `rd32(IGC_TXSTMPH_0)`
  when any slot expires, which "clears all the equivalent bits in the
  TSYNCTXCTL register", but frees the *software* slot only for slots already
  past 15 s.  A slot occupied but not yet expired keeps its `skb` while the
  hardware state it was waiting on has been destroyed → guaranteed later
  timeout.  Predicts **bursts**, not singles.
- **H2 — our own v3 patch.**  It is the only read-modify-write of TSYNCTXCTL in
  the driver (upstream always writes clean values), and it clears
  `IGC_TSYNCTXCTL_ENABLED` for ~1 µs per call.  Either the RMW write-back
  disturbs status bits, or the disable window strands in-flight captures.

Both are dead if arm A (below) wedges.

---

## 2. The rig — no GNSS required

`tools/igc_tx_timeout_repro.py` already drives both sides: a thread hammering
`clock_adjtime(ADJ_FREQUENCY)` on the PHC, and a thread sending UDP with
`SO_TIMESTAMPING` hardware TX timestamps.  It uses `SO_BINDTODEVICE` and sends
to `224.0.0.1` — link-local multicast, so **no ARP, no peer, no IP config, no
route**.

**The DUT NIC needs link up and nothing else.**  Any switch port will do; it
does not need to reach anything.  No antenna, no PPS, no TICC, no ptp4l peer.
This test competes with nothing else in the lab.

### Networking — read this before wiring

**SSH must not come in over the DUT.**  Arms B/C/D swap DKMS modules and
reboot, and a wedged or unloaded igc would strand the host mid-run.  On
TimeHat, SSH over the Pi's **built-in NIC on the trusted LAN**; the i226 is the
DUT and carries only the repro's own multicast.

Putting the i226 on the PTP LAN is fine and arguably tidier (the traffic is
isolated and goes nowhere), but it is **not** required — an unused switch port
on any VLAN works identically, because nothing has to receive the packets.
What matters is only that the DUT is a different interface from the management
path.

### Stop ptp4l on the DUT first

Survey 2026-08-26 found `/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l.conf -i eth1`
already running on TimeHat — an *uncontrolled* hardware-TX-timestamp user on
the exact NIC under test.  It contaminates every arm **including A**: arm A is
"adjfine off, HW TX on", and a live ptp4l supplies uncontrolled TX timestamping
and, depending on what is disciplining, potentially the very adjfine the arm
exists to exclude.  Stop it and **verify** it is gone (`pgrep -a ptp4l`) before
each arm; do not assume a previous arm left it stopped.

### Tool changes needed first (small)

The tool is a *stress* reproducer — both loops run unthrottled, deliberately
("Don't sleep — maximize collision probability").  That answers "is the race
present" in seconds but cannot map onto production rates or separate the arms.
Add:

1. `--adjfine-hz N` (0 = off) — **`0` is what makes arm A possible** and is the
   single most important addition.
2. `--tx-hz N` — rate-limit the TX loop; keep an "unthrottled" setting for fast
   screening.
3. `--duration` already exists as `argv[3]`; promote to a flag while you are in
   there.
4. CSV output: once per second, wall-clock stamp +
   `ethtool -S <if>` counters (`tx_hwtstamp_timeouts`, `tx_hwtstamp_skipped` if
   present) + an EXTTS liveness indicator.  **The per-second cadence is the
   point** — see §4.

Keep the unthrottled default available; do not remove it.

---

## 2a. Driver identity — load known binaries, prove which one ran

**Bob's call, 2026-08-26, and the survey below says he is right**: do not infer
the driver under test from package state.  Unload and load *known binaries*
explicitly, and record which one actually ran.

### Why this is load-bearing, not ceremony

The 2026-04-15 record **cannot tell us which binary produced the observation**,
and three separate mechanisms conspire to hide it:

1. **The DKMS package name understates its contents.**  `dkms status` reports
   only `igc/6.12.0-ppsfix.1`, which names ppsfix and is silent about the v3
   adjfine patch — but commit `cc0ca17` ("apply adjfine TX timestamp race fix
   to *ppsfix vendored source*") folded both into that one package.  Verified
   2026-08-26: `/usr/src/igc-6.12.0-ppsfix.1/src/igc_ptp.c:76-79` contains the
   TSYNCTXCTL read-modify-write.  Anyone checking patch state the obvious way
   concludes the opposite of the truth.
2. **The timeline leaves a hole nobody logged.**  `cc0ca17` landed 15:01; the
   incident row (`a8441d5`) was written 17:38 the same day and describes a
   wedge "44 min after boot".  Whether TimeHat was rebooted onto the *rebuilt*
   module inside that window is recorded nowhere — so "despite v3 patch" rests
   on an assumption, not an observation.
3. **Two vendored trees exist** — `drivers/igc-timehat-edge/` and
   `drivers/igc-timehat-edge-6.8/`, both patched by `cc0ca17`.  Rebuilding the
   wrong one for the running kernel is silent.

This is very likely the source of the §1 ambiguity: not that anyone was
careless, but that patch state was never a *recorded* fact.  This section makes
it one.

### Protocol — every arm

```sh
sudo rmmod igc
sudo insmod /absolute/path/to/<arm>.ko          # NEVER modprobe — it searches
cat /sys/module/igc/srcversion                  # assert == expected; abort on mismatch
```

`/sys/module/igc/srcversion` is the authoritative *runtime* identity and is
independent of package names, file paths and DKMS bookkeeping.  Assert it after
every load and **stamp it into every CSV** alongside the binary's sha256, so no
results file can be misattributed later — which is precisely what went wrong in
2026-04.

### Staged binaries, measured on TimeHat 2026-08-26

Kernel `6.12.75+rpt-rpi-2712`, aarch64.  Both are already on the host; nothing
needs building.

| build | path | `srcversion` | sha256 (first 32) |
|---|---|---|---|
| **patched** (ppsfix + v3 adjfine) | `/var/lib/dkms/igc/6.12.0-ppsfix.1/6.12.75+rpt-rpi-2712/aarch64/module/igc.ko.xz` | `34A0F0BF20444879727CD8F` | `9ad35ea23506839cf969567d8040a71c` |
| **stock** (DKMS original) | `/var/lib/dkms/igc/original_module/6.12.75+rpt-rpi-2712/aarch64/igc.ko.xz` | `6AF1066A96C3EFD60610B4A` | `f7643ae7e5d17b286a091ea896af6508` |

The two `srcversion`s differ, so the check genuinely discriminates.  Running
module at survey time was `34A0F0BF20444879727CD8F` — **patched**, confirming
TimeHat sits in arm A/B state despite what the package name suggests.

### Safety — verified, and the check to repeat on any host

`rmmod igc` drops **only** the DUT: TimeHat's management path is `eth0` on
**`macb`** (onboard), the DUT is `eth1` on `igc`.  Before `rmmod igc` on *any*
host, run

```sh
readlink -f /sys/class/net/<mgmt-iface>/device/driver
```

and confirm it is not `igc` — a host whose only NIC is igc becomes unreachable
and needs a physical power cycle.

---

## 3. Arms

| arm | adjfine | HW TX tstamps | driver (`srcversion` asserted at load — §2a) | question |
|---|---|---|---|---|
| **A** | **off** | on | patched `34A0F0BF20444879727CD8F` | **does it still wedge with no adjfine?** |
| B | on, 1 Hz | on | patched `34A0F0BF20444879727CD8F` | production config |
| C | on, 1 Hz | on | stock `6AF1066A96C3EFD60610B4A` | does v3 help or hurt at a fixed rate? |
| D | off | on | stock `6AF1066A96C3EFD60610B4A` | control |

"Patched" and "stock" are the §2a binaries, loaded by absolute path and
verified by `srcversion` after load.  Do not identify an arm's driver by
package name, DKMS state, or "what was installed last" — that is the failure
mode §2a exists to close.

### Arm E — EXTTS liveness vs TX rate

Added 2026-08-26 (Bob's approval) after arm A produced a result §4 did not
anticipate: **EXTTS died ~14 s BEFORE the first TX timeout**, not with it and
not after.  §4 offered only "same instant" or "later"; the answer was
"earlier", so EXTTS death cannot be a downstream consequence of the timeout
cascade.  Both look downstream of TX-timestamp load itself.

But arm A ran TX **unthrottled** (`tx_hwtstamp_skipped` reached 8.7 million),
which is nothing like production.  The claim that survives is narrow:
*saturating* TX timestamping kills EXTTS within ~2 s.  Whether **1 Hz ptp4l**
does the same is the question the lab actually cares about, and it cannot be
inferred from one saturated data point.

| arm | adjfine | TX rate | driver | duration |
|---|---|---|---|---|
| E1 | off | 1 Hz | patched | 20 min |
| E2 | off | 5 Hz | patched | 15 min |
| E3 | off | 20 Hz | patched | 15 min |
| E4 | off | 100 Hz | patched | 10 min |
| E5 | off | 1000 Hz | patched | 10 min |
| E6 | off | unthrottled | patched | 10 min |

adjfine is **off throughout** so TX rate is the only variable.  For each rate
record: does EXTTS stop, how long after TX starts, and does the first TX
timeout precede or follow it.  A rate at which EXTTS survives indefinitely is
an operational answer on its own — it bounds how hard a host may drive TX
timestamping before losing PPS capture.

Requires a PPS source on the DUT's EXTTS pin.  Wired 2026-08-26: PiFace PPS
OUT → TimeHat PPS IN (SDP1), verified at exactly 1 Hz.  The `--extts-index`
pre-flight refuses to run without live edges, because an unrouted pin and a
dead EXTTS are indistinguishable from zero events and reporting the latter
would manufacture the very correlation being tested.

### Running the arms unattended

`tools/igc_arm_campaign.sh` runs the whole set back-to-back.  **Every arm
begins with a fresh `insmod` of its named binary by absolute path** — never
`modprobe` — which implements §2a and additionally guarantees each arm starts
from an identical un-wedged state, so no arm can contaminate the next.  It
asserts `srcversion` after every load, re-routes the EXTTS pin (routing does
not survive a reload), re-resolves the PHC index (it can change across a
reload — never hardcode `/dev/ptp0`), refuses to start if the management
interface is on igc, and restores the patched driver on exit.

**Run arm A first.  It is decisive.**

- If A wedges → adjfine is exonerated, H1 and H2 both die, Patch 2 is orthogonal
  to this failure, and the mechanism is somewhere else entirely.  That is a big
  result and it changes what we do about i226.
- If A does not wedge → adjfine is necessary to the failure, and B vs C tells us
  whether v3 is a net win at production rates.  If C beats B, the immediate
  operational answer is "stop shipping v3 at 1 Hz", with no new patch at all.

Run each arm long enough to see several events, not one.  Start unthrottled to
find the fast regime, then repeat the interesting arms at 1 Hz.  A single event
is not an MTBF; report event counts and the observation window, not a
derived MTBF, unless you have ≥5 events.

---

## 4. The discriminator: burst pattern

H1 predicts a specific signature.  When one slot expires, the
`rd32(IGC_TXSTMPH_0)` invalidates the others, so timeouts should arrive in
**bursts of up to four inside one 15 s window**, not as isolated singles.

Sampling `tx_hwtstamp_timeouts` once per second gives this directly.  Report
the inter-event time histogram, not just the total.

- Isolated singles, evenly spread → **refutes H1**.
- Clusters of 2–4 within 15 s → **supports H1**, and the patch is small: when
  `found`, free every occupied slot rather than only the expired ones.

If you want certainty rather than inference, a debug printk in
`igc_ptp_tx_hang()` logging the slot index `i` and `tstamp->start` at each
timeout settles it outright.  Cheap, and worth it if the counter cadence is
ambiguous.

Also record, for every arm: **does EXTTS die at the same instant as the first
timeout, or later?**  `igc-kernel-patches.md` notes the linkage is not
understood; the relative timing is a free clue and nobody has written it down.

---

## 5. Cross-host — is it the board or the driver?

TimeHat is the known i226 host, but the repo suggests two more:

- `config/madhat.toml` — `ptp_dev = "/dev/ptp_i226"` (disabled 2026-05-24),
  plus extensive i226 notes including its both-halves PEROUT quirk.
- `config/ocxo-i226.toml` — an i226 card in the box now called `ptpmon`
  (`/dev/ptp0 → i226 (this card)`).  **This file is stale.**

**ptpmon has no i226 — checked 2026-08-26, host powered on for this.**  It
carries an E810-C quad (`01:00.0-.3`, driver `ice`, as `e810p0..3`) plus an
onboard `e1000e` for management; `igc` is not even loaded, and the only
`igc.ko` present is the untouched in-kernel one.  So `config/ocxo-i226.toml`
describes hardware that is no longer in the box, and **ptpmon cannot provide
the x86-vs-aarch64 comparison this section wants.**  MadHat was unreachable at
survey time and remains unconfirmed.

Consequence: the driver-generic vs platform-specific question stays **open**,
and a TimeHat-only result must say so rather than imply generality.

If a second host is available it is worth using, for two reasons: arms run in
parallel and halve wall-clock, and **ptpmon is x86** while TimeHat is aarch64.
If the wedge reproduces on both, it is driver-generic; if only on the Pi, it is
platform- or board-specific, which is a different verdict for i226 as a
component choice.

---

## 6. Deliverables

1. Tool changes (§2) committed on a feature branch.
2. Raw CSVs pulled back to `~/gt/` per the lab storage rule.
3. **A results doc in `docs/`** — arms, event counts, observation windows,
   inter-event histograms, EXTTS timing, and a verdict on H1/H2.
4. **Corrections to [`igc-kernel-patches.md`](igc-kernel-patches.md)**: resolve
   the 2026-04-15 contradiction, and either attribute the wedge or state
   plainly that it remains unattributed.  Remove or qualify the
   non-comparable MTBF figures.
5. **Update [`split-clock-architecture.md`](split-clock-architecture.md) §3.3**
   — the i226 verdict there is explicitly contingent on this result.

Per the repo's acceptance rules this bead carries `hw:` labels, so code stays on
the branch and lab validation is reported separately.

---

## 7. What "success" means

Not "i226 is fine" or "i226 is bad".  Success is **an attributed mechanism**,
so the verdict rests on something understood.  A clean negative — "reproduces on
both hosts with adjfine off, cause unknown, here is the evidence" — is a good
outcome and better than the current state.

---

*Drafted 2026-08-26 (main).*
