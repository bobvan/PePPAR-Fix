# Split indoor/outdoor clock architecture — sketch and sanity test

**Status**: design sketch, not built.  Drafted 2026-08-26 (main) from Bob's
proposal: put the antenna + receiver + PTP GM in a rooftop "ear" on PoE, and
put the holdover oscillator indoors where the environment is benign, with
multiple ears per indoor unit and multiple indoor units.

This doc sketches it, names the standards precedent, and records the
sanity-test findings — several of which are already documented in this repo
and argue against parts of the sketch as drawn.

Companion: [two-clock-agreement-forward-model.md](two-clock-agreement-forward-model.md)
(§7 in particular — why PTP is a delivery leg, not part of the clock-agreement
budget).

---

## 1. The sketch

```
        ROOFTOP (harsh, replaceable)          INDOOR (benign, load-bearing)
   ┌──────────────────────────────┐
   │ ear A                        │
   │  survey-grade antenna        │      ┌─────────────────────────────────┐
   │  GNSS rx (F9T-class)         │      │ indoor unit 1                   │
   │  PTP GM  ─ i226 ─ PoE ───────┼──────┤ N ports, holdover DO (OCXO)     │
   │  PPS OUT (commissioning)     │      │ PHCs disciplined from the DO    │
   └──────────────────────────────┘      │ selection: intersection, not    │
   ┌──────────────────────────────┐      │            BMCA                 │
   │ ear B  ────────────────────────┬────┤ GM downstream, own clockClass   │
   └──────────────────────────────┘ │    └───────────┬─────────────────────┘
   ┌──────────────────────────────┐ │                │ cross-check
   │ ear C  ────────────────────────┼────┐ ┌──────────┴──────────┐
   └──────────────────────────────┘ │    └─┤ indoor unit 2       │
                                    │      └─────────────────────┘
                    NTS / provider PTP / PPS ─┘   (non-GNSS sanity feeds)
```

**The premise is sound and it is the right split.**  Failures concentrate
where the environment is harsh; the flywheel belongs where it is not.  Putting
an OCXO on a roof means its holdover spec is a fiction — the tempco term
dominates everything (see [two-site-sync-budget.md](two-site-sync-budget.md)
§3.3, and the MadHat ceiling-fan result: 4 ppb from *air movement indoors*).

---

## 2. Precedent — this is a standardized product class, just not an
   "enterprise switch"

Bob's doubt — "I don't think I've seen PTP switches used for holdover" — is
right about enterprise switches and wrong about telecom ones.

| standard | what it defines | relevance |
|---|---|---|
| **ITU-T G.8273.2** | T-BC classes A/B/C/D, incl. **holdover** requirements | a boundary clock with a real oscillator and a specified holdover is a standardized class, not an invention |
| **ITU-T G.8275.2 / G.8273.4** | APTS, and **T-BC-A**: "a boundary clock **assisted by a local time reference** (e.g. a PRTC or GNSS-based time source) as a primary source of time" | the indoor unit's job description, almost verbatim |
| **ITU-T G.8272.1** | ePRTC — caesium + GNSS, 100 ns over 14 days | the same philosophy: **the flywheel is deliberately not in the antenna** |

Commercial instances: Microchip TimeProvider 4100, Oscilloquartz OSA 54xx,
Meinberg — grandmasters that accept GNSS *or* PTP input and hold over on a
local oscillator.

**What is genuinely unusual here is the direction, not the holdover.**  APTS
puts GNSS *local* to the node with PTP as the backup.  This sketch inverts it:
GNSS is remote, delivered *over* PTP, with the local oscillator as flywheel.
The closest named thing is a T-BC-A whose "local time reference" arrives over
Ethernet instead of coax — and the entire cost of that inversion lands in §3.1.

---

## 3. Sanity-test findings

### 3.1 The ear→indoor link is now inside the timing path — but its error is static

PTP over copper carries **200–400 ns of PHY TX/RX latency asymmetry**, which
appears as a fixed offset of half that.  Delivered raw, that is the dominant
term and it swamps everything the clock chain does.

The saving grace: for a fixed ear, fixed port, fixed cable, fixed link speed,
**this asymmetry is a constant**.  It is calibratable exactly the way White
Rabbit calibrates a link, and it does not need WR hardware to do it — only a
one-time measurement.

**Proposed commissioning procedure**, using the PPS OUT the ears already have:
run a temporary coax from ear A's PPS OUT to the indoor unit's PPS IN, measure
`(PTP-derived time) − (PPS-derived time)` over a few minutes, store the
difference as that link's asymmetry constant, remove the coax.  Repeat per
(ear, port).  This is the single highest-leverage thing in the whole design:
it converts PTP delivery from ~100–200 ns of uncalibrated bias to something
bounded by the measurement, and it is a commissioning cost, not a running one.

Re-measure on any change of cable, port, SFP, or link speed — the constant is
a property of the whole physical path, not of the ear.

### 3.2 Three ears on one roof are NOT three independent sources

This is the finding that most threatens the design's stated purpose.

N-of-M voting only buys fault tolerance against **independent** faults.  Ears
sharing a rooftop share:

- the same sky, so the same spoofer, the same jammer, the same ionospheric storm
- the same multipath environment
- the same receiver model and firmware — so the same firmware bug, the same
  leap-second handling, the same week-rollover behaviour
- very likely the same SSR correction stream and the same NTRIP caster
- the same lightning transient and the same water ingress event

Against every one of those, three ears fail *together* and vote unanimously for
the wrong answer.  Voting is strongest against exactly the failures that are
already the least dangerous (one ear dies outright), and weakest against the
one that matters (an ear stays up and lies).

**Consequence: the non-GNSS feeds are not a nice-to-have, they are the actual
independence in the design.**  NTS over the internet and provider PTP are the
only sources in the sketch whose failure is uncorrelated with the rooftop.
Bob's instinct to include them is doing more work than the third ear.

Cheap ways to buy real independence between ears, in rough order of value:
different receiver vendor (u-blox vs Septentrio — the repo already runs both);
different antenna model and mounting point; different correction stream or
none; physically separated sites once the cable run allows it.

### 3.3 i226 is the wrong NIC for this — by our own measurements

`docs/igc-kernel-patches.md` documents, on TimeHat:

> TX timestamp timeouts cascade into "a permanent wedge where all 4 TX
> timestamp slots are occupied and every subsequent attempt times out.
> Observed MTBF on TimeHat: **~44 minutes at 1 Hz adjfine + 1 Hz ptp4l sync**.
> Once wedged, the subsystem does not self-recover — it stays broken until
> driver reload or reboot."

And: "EXTTS stops delivering events after the TX timestamp subsystem wedges…
This kills PPS delivery and prevents the servo from running."

The trigger is *PHC frequency discipline concurrent with PTP hardware
timestamping*.  That is not an edge case for this design — it is the indoor
unit's steady state, and the ear's too (a GM transmits Sync continuously).
The v3 adjfine patch reduces the rate but does not remove the failure.

Add the separately documented [i226 PEROUT 500 ms bug](i226-perout-500ms-bug.md)
— hardware, board-dependent, unfixable in software on affected boards.

**Verdict: do not put i226 in either the ear or the indoor unit until the
wedge is closed.**  This is the largest practical risk in the sketch, and it is
already sitting in our own docs.

#### Checked against Time-Appliances-Project/TimeHAT (2026-08-26) — nothing to adopt

[TAP's TimeHAT](https://github.com/Time-Appliances-Project/TimeHAT) ships
`intel-igc-ppsfix` DKMS trees for RPi5 kernels 6.6, 6.12 and 6.12.62.  Diffed
all three against `drivers/igc-timehat-edge/`:

- `igc_ptp.c` is the **only** file that differs anywhere in the src tree, and
  the entire 29-line diff is **our** adjfine `TSYNCTXCTL` guard, which none of
  their three variants has.
- Their README claims only EXTTS dual-edge, PEROUT 1PPS, and per-channel pin
  tracking — all of which our tree already carries (ppsfix is our base).
- Their git history is five commits, none of it PTP-path work.

**TimeHAT's driver is a strict subset of ours.**  There is nothing there we
missed, and i226 is not rehabilitated by it.

#### But the diff surfaced a concrete, untested hypothesis for the cascade

Both trees carry the upstream remedy this repo's `igc-kernel-patches.md`
proposed as fix (a) — `ptp_tx_lock` plus, in `igc_ptp_tx_hang()`:

```c
if (found) {
        /* Reading the high register of the first set of timestamp registers
         * clears all the equivalent bits in the TSYNCTXCTL register.
         */
        rd32(IGC_TXSTMPH_0);
}
```

So the ~44 min wedge was observed **with** that remedy in place.  Looking at
what it actually does suggests why it does not help, and may even hurt:

`rd32(IGC_TXSTMPH_0)` clears **all four** slots' hardware bits, but the loop
above it calls `igc_ptp_tx_timeout()` — and thus frees the skb and the software
slot — only for slots that have already **expired**.  A slot that is occupied
but not yet 15 s old keeps its `tstamp->skb`, while the hardware state it was
waiting on has just been destroyed underneath it.  That slot can now never
complete; it is guaranteed to sit until its own timeout fires, which is exactly
the cascade.

**Hypothesis: the upstream fix converts one expiry into up to three more
guaranteed expiries.**  If so, the correct patch is small — when `found`, free
*every* occupied slot, not just the expired ones, because the register read has
already invalidated them all.

Untested.  It is cheap to test (`tx_hwtstamp_timeouts` counter and dmesg on a
box doing 1 Hz adjfine + 1 Hz ptp4l), and worth doing before writing i226 off
permanently, because it would rehabilitate a lot of cheap hardware.

### 3.4 Multiple NICs means multiple clock domains — a real switch has one

Four i226 is four independent PHCs, each with its own oscillator.  A
purpose-built PTP switch has one clock domain across all ports; a multi-NIC box
does not.  Cross-PHC sync via `phc2sys` costs tens of ns, and that error lands
directly in the delivered time.

**But this one turns into a strength if inverted**: make the holdover DO the
box's time reference and discipline *every* PHC to it over PPS
(PEROUT/EXTTS on the SDP pins).  That is precisely what peppar-fix already
does for one PHC; N is a scaling problem, not a new one.  The DO stops being
"the holdover oscillator" and becomes the spine of the box.

Better still, avoid the problem: a **single 4-port NIC has one PHC**.  The
lab already owns an **E810-XXVDA4T** (currently `ptpmon`) — four ports, one
clock domain, onboard DPLL, SMA in/out, built for timing, and we have driver
experience with it.  It is x86-only (`ice`, see
[e810-cm5-research.md](e810-cm5-research.md)), which settles §5.

Caveat: E810-XXVDA4T is SFP28, so copper PoE to the ears needs media
conversion — and media converters are a classic asymmetry source.  That is
absorbable by §3.1's calibration, but it must be measured, not assumed.

### 3.5 No SyncE — one channel, not two

Telecom gets frequency from the **physical layer** (SyncE, G.8262) as a
channel independent of packet timing, and uses it both to improve holdover and
to sanity-check PTP.  i226 has no SyncE path.  So in the sketch as drawn there
is exactly one channel from ear to indoor — packets — and no independent
frequency reference to check it against except the local DO.

That is defensible (the local DO *is* the check, and that is the point of the
architecture), but it should be a conscious trade, not an oversight.  E810
class hardware restores the option.

### 3.6 Two indoor units is a harder problem than two ears

Two indoor units on one downstream network means two GMs.  BMCA picks one;
when it fails, the other takes over — and the handover is a **phase step**
of whatever the two disagreed by.  Everything downstream sees it.

The two indoor units must therefore be cross-disciplined to each other, not
merely both "correct".  The tools are already here: PPS OUT → PPS IN both
ways, measured continuously, which is the same two-clock agreement problem the
forward model already budgets.  Target the agreement bound, then failover is
bumpless by construction.

Also plan for the partition case: if the two indoor units lose sight of each
other but both still see ears, both will claim GM.  Needs an explicit tie-break
(static priority is fine and boring) rather than letting BMCA improvise.

---

## 4. Source selection: don't use BMCA for it

Bob's objection to BMCA is correct and worth stating precisely: **BMCA is a
configuration protocol, not a measurement protocol.**  It ranks *advertised
attributes* — priority1, clockClass, clockAccuracy — and the winner takes all.
Nothing in it measures whether the winner is telling the truth.  A node
announcing priority1=0, clockClass=6 wins instantly, whether it is a caesium
standard or a Raspberry Pi with a wrong leap-second table.

IEEE 1588-2019 added an optional AUTHENTICATION TLV, which closes the
*forgery* half.  It does nothing about a node that is authentic and wrong, and
nothing about a delay attack (which forges no message at all).  It is also
barely deployed.

**The architecture should not select an upstream with BMCA at all.**  Instead:

1. Each ear is a **measurement**: an offset with an uncertainty interval,
   `[θ − σ, θ + σ]`, where σ carries the ear's own reported quality (the
   engine already computes σ_total and maps it to clockClass — see
   [ptp4l-supervision.md](ptp4l-supervision.md)) plus the calibrated link
   asymmetry residual from §3.1.
2. Run **Marzullo / the intersection algorithm** over all sources, ears and
   non-GNSS feeds alike, to find the largest mutually-consistent set.  This is
   what NTP's select-and-cluster stage does, and it is a *measurement*
   decision, not a beauty contest.
3. The local DO is the flywheel that makes it safe to reject **everything**
   transiently rather than being forced to pick a least-bad source.
4. The indoor unit is then **always** the GM for the downstream network, with
   its own clockClass reflecting its actual state.  BMCA never selects
   anything downstream because there is only ever one candidate.

That structurally removes the failure Bob dislikes: downstream slaves cannot
blindly follow a bogus GM, because they never see a choice.  Upstream, a bogus
ear is one interval among four and gets voted out.

**Byzantine bound.**  Tolerating *f* arbitrary (not merely dead) faulty
sources requires **3f + 1**.  Three ears plus one local holdover = 4 → tolerates
one Byzantine source.  That is the real reason for "three or more ears", and it
is a good reason — subject entirely to §3.2's warning that the bound assumes
independence the rooftop does not provide.

---

### 3.7 Switchberry — most of the indoor unit already exists

[TAP's Switchberry](https://github.com/Time-Appliances-Project/Switchberry) is
a CM4-controlled 5-port managed switch built around a **Microchip KSZ9567**
switch and a **Renesas 8A34004 ClockMatrix DPLL**.  Asked narrowly — "could
that switch chip be a component of the indoor unit?" — the answer is yes, but
it undersells it: the board addresses four separate findings above at once.

| finding | Switchberry's answer |
|---|---|
| §3.3 i226 wedge | no i226 in the data path at all |
| §3.4 multi-PHC | KSZ9567 is **one** switch with one PTP clock domain across 5 ports |
| §3.5 no SyncE | **SyncE recovery from the switch into the DPLL**, as endpoint or boundary clock |
| §6 software forwarding | hardware forwarding at line rate; hardware TC available |

And the DPLL is the **same ClockMatrix family we already drive** on otcBob1 and
ptBoat — the register map, the combo bus, FCW steering at sub-ppt resolution
(`timebeat-otc-register-map.md`, `clockmatrix-bootstrap-plan.md`) all transfer.
Additional useful I/O: an OCP M.2 GNSS slot, 4× muxed rear SMA, and CM4↔DPLL
PPS routing explicitly wired for both grandmaster and client roles.

**The deepest benefit is one the feature list does not state.**  On a NIC-based
indoor unit, frequency steering happens through `adjfine()` on a Linux PHC —
which is precisely the trigger for §3.3's wedge.  On Switchberry, steering
happens **in the DPLL, in hardware**.  The architecture removes the trigger
rather than patching around it.

Two caveats, both real:

1. **The ClockMatrix's reference is a local TCXO.**  Holdover is exactly the
   indoor unit's reason for existing, and a TCXO is the wrong flywheel for it
   (`two-site-sync-budget.md` §6.1 classes TCXO as inadequate).  Fix: bring a
   good OCXO in as a DPLL input reference over one of the muxed SMAs, so the
   DPLL locks frequency to the OCXO and phase to GNSS/PTP — the two-tier
   pattern already explored in `clockmatrix-bootstrap-plan.md`.  This is the
   one modification the design actually requires.
2. **Its default mode is a hardware transparent clock, which is wrong for us.**
   A TC forwards PTP and preserves the end-to-end GM relationship, so
   downstream slaves would see the ears directly and run BMCA over them — the
   exact failure §4 exists to remove.  The indoor unit must **terminate** each
   ear's PTP and re-originate, which is the optional Linux DSA / boundary-clock
   mode, not the plug-and-play default.  Line-rate hardware forwarding remains
   available for non-PTP traffic.

Worth noting for §6: hardware TC is available here, so the "build a BC, never a
TC" rule was a consequence of *software* forwarding, not a universal one.  We
still want a BC — but for the selection-architecture reason in §4, not because
the hardware can't do TC.

---

## 5. Mesh PTP — PTP+Squared and PTPsec

**Timebeat PTP+Squared** ([vendor page](https://license.timebeat.app/squared.php))
is, by its own description, "classic PTP unicast sessions between hosts set up
automatically by a peer-to-peer network."  An overlay using a Distributed Hash
Table and PubSub handles source-capacity announcement, automatic unicast
session setup/teardown, and — the interesting part — **time-error dissemination
upstream and downstream so a node knows its true offset to source**.  Topology
is a directed acyclic graph, kept loop-free by increasing root distance per hop
and weighting by cumulative source error.

Its stated motivation is exactly Bob's: *"with only a single source of time you
cannot determine if it is correct,"* and with "three sources of time" you can
identify which is divergent.

**So the shape is right — but be precise about what it provides.**  PTP+Squared
is an *automation and topology* layer: discovery, session management, error
accounting.  It does not authenticate anything, and it does not itself reject a
bogus GM.  What it gives you is the thing rejection needs: N independent
measurements with propagated uncertainty.  **The rejection is still your
selection algorithm's job** (§4).  Adopting it would remove the unicast-mesh
plumbing work, not the decision logic.

**PTPsec** ([arXiv 2401.10664](https://arxiv.org/abs/2401.10664)) is the
academic backing for the mesh specifically.  It defends against **time delay
attacks** — where an attacker forges nothing and merely delays packets
asymmetrically, which authentication cannot detect — using **cyclic path
asymmetry analysis** over redundant paths, with a method for finding redundant
paths in arbitrary networks.  Reported to detect and mitigate all tested
attack scenarios with minimal detection time.

The two are complementary and both are relevant here: authentication for
forgery, intersection for wrong-but-authentic, cyclic-path analysis for
delay.  A mesh of ≥3 ears against a local holdover gives the redundant paths
all three need.

---

## 6. Can the indoor unit be a Pi with 3–4 NICs doing software forwarding?

**For the timing function: yes, and the reason is architectural rather than
performance.**

> **A boundary clock does not forward PTP packets.**  It terminates PTP on each
> port and re-originates it.  Timestamps are taken in the NIC's MAC/PHY.  So
> software forwarding latency, jitter, and scheduling never enter the timing
> path.

The distinction that matters:

| role | software forwarding |
|---|---|
| **Boundary clock / multi-port GM** | fine — PTP is terminated per port, not forwarded |
| **Transparent clock** | fatal — a TC must measure residence time of forwarded packets |

So: build a BC, never a TC, and line rate is irrelevant to accuracy.

Practical caveats, in order of how much they actually bite:

1. **§3.3 (the i226 wedge) is the blocker, not throughput.**
2. **§3.4** — 3–4 separate NICs means 3–4 PHCs; either discipline them all from
   the DO, or use one 4-port NIC.
3. **Pi 5 has a single PCIe 2.0 x1 lane.**  Three or four NICs need a PCIe
   switch on that lane.  Fine for PTP packet rates, badly oversubscribed for
   data.
4. **Don't make it the site's data switch.**  Let it be a timing appliance
   whose ports carry PTP and management only, and leave bulk traffic on an
   ordinary switch.  This sidesteps throughput entirely and shrinks the
   attack surface.

**Recommendation**: an x86 mini-PC (N100 class) rather than a Pi — because
§3.4's clean answer (E810-XXVDA4T, four ports, one PHC, timing-grade, already
in the lab) is x86-only, and because the whole peppar-fix analysis stack
already runs on x86 at `ptpmon`.  The commodity "4× i226 soft router" box is
the obvious cheap build and is exactly what §3.3 says to avoid.

---

## 7. What to prototype first

Ordered so each step can fail cheaply and independently:

1. **Link-asymmetry calibration (§3.1)** — one ear, one indoor unit, temporary
   PPS coax.  Answers "how good can PTP delivery actually get here?" and
   everything else is contingent on it.  No new hardware.
2. **NIC decision (§3.3/§3.4)** — reproduce or refute the i226 wedge under a
   *GM* workload specifically, and characterize the E810 as a 4-port BC.
   `ptpmon` already has the hardware.
3. **Intersection-based selection (§4)** — pure software, testable offline
   against recorded multi-source offsets, including a deliberately lying
   source.  This is where the design's central claim lives and it can be
   validated before any rooftop hardware exists.
4. **Two-ear disagreement measurement (§3.2)** — how correlated *are* two ears
   on one roof, in practice?  Measured, not assumed.  Determines whether ear
   count or source diversity is the better next investment.
5. Dual indoor units and bumpless failover (§3.6) — last, because it is the
   most work and depends on all of the above.

---

## References

- [two-clock-agreement-forward-model.md](two-clock-agreement-forward-model.md) §7 — PTP as delivery leg
- [two-site-sync-budget.md](two-site-sync-budget.md) — the excursion budget
- [igc-kernel-patches.md](igc-kernel-patches.md) — the i226 wedge, §3.3
- [i226-perout-500ms-bug.md](i226-perout-500ms-bug.md)
- [ptp4l-supervision.md](ptp4l-supervision.md) — clockClass from estimator state
- [receiver-clock-hierarchy.md](receiver-clock-hierarchy.md) — receiver clock tiers
- [wr-gm-research.md](wr-gm-research.md) — White Rabbit GM architecture
- [Timebeat PTP+Squared](https://license.timebeat.app/squared.php)
- [PTPsec, arXiv 2401.10664](https://arxiv.org/abs/2401.10664)
- ITU-T G.8275.2 (APTS), G.8273.2 / G.8273.4 (T-BC, T-BC-A), G.8272.1 (ePRTC)

---

*Drafted 2026-08-26 (main).  Sketch only — nothing here is built or measured.*
