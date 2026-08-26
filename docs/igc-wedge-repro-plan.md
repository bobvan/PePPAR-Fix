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

Both cannot be true.  If no adjfine was running when the cascade began, nothing
in the adjfine path can be the *initiating* strand — and Patch 2 may be
orthogonal to the failure that actually disqualifies the part.

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

## 3. Arms

| arm | adjfine | HW TX tstamps | driver | question |
|---|---|---|---|---|
| **A** | **off** | on | v3 patched | **does it still wedge with no adjfine?** |
| B | on, 1 Hz | on | v3 patched | production config |
| C | on, 1 Hz | on | unpatched | does v3 help or hurt at a fixed rate? |
| D | off | on | unpatched | control |

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
  (`/dev/ptp0 → i226 (this card)`).

**Confirm the hardware is still fitted before planning around it** — MadHat is
not in CLAUDE.md's host table and ptpmon was recommissioned 2026-06-14.

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
