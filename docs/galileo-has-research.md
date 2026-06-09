# Galileo High Accuracy Service (HAS) — Research

*Rewritten 2026-06-08 after the ZED-X20P bring-up on PiPuss.  The
previous version of this doc claimed HAS Phase 1 provides phase biases
for PPP-AR — **that is wrong** (see "Phase 1 has no phase biases"
below).  It also predated any E6-capable receiver in the lab; we now
have one (the X20P) and have empirically confirmed it streams the HAS
signal-in-space.*

## Bottom line (answers to the three questions)

1. **How could we use HAS in carrier-phase timing?**  Feed HAS orbit +
   clock + code-bias corrections into our existing **float** PPP filter
   (replacing/augmenting the broadcast-only or BKG SSR path).  Published
   work shows static **float**-PPP carrier-phase time transfer with HAS
   reaches **sub-nanosecond** precision (σ ≈ 0.1–0.3 ns) and tightens
   the real-time satellite clock from ~0.66 ns (broadcast) to ~0.17 ns.
   No integer ambiguity resolution is required for this — the constant
   carrier-phase bias is absorbed by the float ambiguity on a static
   receiver.  For PePPAR Fix this helps the **long-τ accuracy** of the
   receiver-clock solution (how faithfully dt_rx tracks GPS/Galileo
   time) and **cross-host agreement** (two hosts on the *same* HAS
   corrections share common-mode error), **not** the short-τ floor
   (that's rx-TCXO/DO limited and already below HAS clock noise).

2. **Should you sign up for Internet delivery (IDD)?**  **Not required**
   for the corrections — the X20P on PiPuss already streams the E6-B HAS
   pages (confirmed below), so we can decode HAS from the
   signal-in-space for free, with no registration and no internet.  IDD
   is still **worth registering** because it is *receiver-independent*
   (works on the F9T/F9P hosts that can't track E6), arrives as standard
   NTRIP that drops straight into our SSR ingestion path, and gives
   redundancy.  Registration is free but **application-gated** (EUSPA
   reviews the use case), so it isn't instant.  Recommendation: register
   IDD for fleet-wide convenience, but the **X20-E6 SIS path is the
   zero-dependency option and is already proven**.

3. **What positioning error could we get?**  HAS **Phase 1** (today):
   horizontal **≤ 20 cm (95%)**, vertical ≤ 40 cm, convergence target
   ≤ 300 s — measured results are commonly ~10–20 cm horizontal.  With
   phase biases + integer AR (HAS **Phase 2**, future) this drops to
   **cm-level** with faster convergence.  We pin the ARP from survey, so
   position accuracy isn't the deliverable for a fixed timing host — but
   HAS lets a *non-surveyed* host self-converge to dm autonomously, and
   the better orbit/clock reduces the (pos, ZTD, clock) null-drift we've
   been fighting on the float filter.

## What HAS is

Galileo HAS is a **free, worldwide** real-time PPP correction service,
broadcast on the **Galileo E6-B** signal-in-space and also distributed
over the internet (IDD).  Operational (Phase 1, "Initial Service")
since January 2023.  Covers **GPS + Galileo** (no BeiDou, no GLONASS).

### Phase 1 has NO phase biases (correction to the old doc)

This is the central fact the previous doc got wrong.  HAS **Phase 1 /
Service Level 1** currently broadcasts only:

| Correction | Phase 1 (now) | Phase 2 (future) |
|---|---|---|
| Orbit | yes | yes |
| Clock | yes | yes |
| Code bias | yes | yes |
| **Phase bias** | **no — not broadcast** | yes (SL1 upgraded) |
| Atmospheric (iono/tropo) | no | yes, Europe only (SL2) |

So **HAS today does not enable PPP-AR** — the same limitation as our
current BKG SSR stream (phase bias = 0).  The convergence-time target of
5 min was *not* met in the test campaign precisely because phase biases
weren't included.  What HAS gives us *over* broadcast-only/BKG today is
**much better real-time orbit + clock** (and code bias), which is what
makes the float-PPP **timing** result sub-ns.  PPP-AR with HAS is a
**Phase 2** prospect, not a today prospect.

(For PPP-AR *today* we'd still need a single-AC phase-bias source —
CNES SSRA00CNE0 on GAL E1/E5a, per `docs/correction-sources.md` and
`docs/ssr-mount-survey.md`.)

## Two delivery paths

### A. E6-B signal-in-space (SIS) — works on the X20P, no internet

HAS corrections are broadcast in the Galileo **E6-B C/NAV** pages.
Receiving them needs an **E6-capable receiver**.  Of the lab fleet,
**only the ZED-X20P tracks E6** (sigId 8, 1278.75 MHz — see
`docs/f9t-firmware-capabilities.md`).  F9T/F9P/F10T do **not**.

**Empirically confirmed on PiPuss 2026-06-08:** the base
**ZED-X20P-00B** streams the E6-B pages in **RXM-SFRBX** — GAL sigId 8
appeared **151 times in 20 s** (≈ 7.5 pages/s, the most frequent SFRBX
stream), alongside E1 (sigId 1) I/NAV and E5 (sigId 3).  So although
u-blox says the -00B "tracks E6 but doesn't *use* HAS internally" (and
u-center exposes no HAS option), the raw HAS signal-in-space **is
available to us** in the receiver's normal RXM-SFRBX output.  We decode
it ourselves.

- Latency: ~ a few seconds (E6-B page cadence; HAS messages are
  assembled over several pages).
- Cost / registration: **none**.

**Page decode validated on PiPuss 2026-06-08** (`tools/has_page_monitor.py`).
The X20-00B's RXM-SFRBX feeds the HAS decoder cleanly — the open risk is
closed at the page level:

- Each E6-B page arrives as **16 big-endian dwrds** (492-bit C/NAV page).
- The **HAS page header** sits at a **14-bit lead-in offset**, then the
  standard 24-bit layout: `HASS(2) rsv(2) MT(2) MID(5) MS(5) PID(8)`
  (determined by offset-scan; the false alignment at off=10 gave
  all-even MS and too few PIDs — discarded).
- **Real HAS pages: HASS=1 (Operational), MT=1**; in a 90 s capture, 450
  of 540 pages were operational.  Dummy pages are HASS=2 / MT=3 and
  byte-identical across satellites (easy to filter).
- **Complete, RS-decodable message sets are collected within seconds** —
  e.g. MID 14 (MS=11, 45 distinct Page-IDs), MID 20 (MS=11, 141 PIDs),
  MID 26 (MS=10, 110 PIDs).  Different satellites broadcast different
  Page-IDs of the same MID (E03/E16/E25 → PID 47/167/215 of MID 14) —
  the HAS Reed-Solomon page-diversity scheme, working as designed.

**Full RS + SSR decode DONE 2026-06-08** (`tools/has_decode_cssr.py`,
CSSRlib).  The X20's E6 pages decode end-to-end to real corrections:

```
X20 E6 ─→ RXM-SFRBX ─→ page reassembly ─→ HPVRS Reed-Solomon ─→ SSR
```

- CSSRlib's `cnav_msg.decode_cnav` **independently confirms the framing**
  reverse-engineered above — it also reads the HAS header at **bit 14**.
- gMat = HAS SIS ICD Annex B generator matrix (255×32, GF(256)),
  vendored at `support/has/has_gmat.csv`.
- Toolchain self-tested against the ICD's own Annex D worked example
  (decodes to 53 sats, orbit/clock/cbias/pbias), then run on a live
  120 s X20 capture (787 pages → 8 HAS messages decoded):
  **orbit + clock + code-bias for 53 SVs (26 GPS + 27 Galileo)**, e.g.
  G01 dclk +0.010 m, G02 −1.225 m, G03 +1.478 m, with sub-meter-to-few-
  meter orbit corrections — physically sensible SSR magnitudes.
- **No phase-bias block** appears — exactly consistent with HAS Phase 1
  (orbit/clock/code-bias only).  The decoder handles pbias when present
  (Phase 2 / the ICD example) but the live service doesn't send it yet.

Pipeline: `tools/has_page_monitor.py --raw-pages <file>` (Pi side, pyubx2)
→ `tools/has_decode_cssr.py <file>` (dev box, CSSRlib).  The collected
corrections are ready to adapt into our `SSRState` for the HAS-vs-broadcast
float-PPP time-transfer comparison.

### B. Internet Data Distribution (IDD) — NTRIP, receiver-independent

HAS corrections over **NTRIP** in an RTCM-like SSR format (RTCM
10410.1 / 10403.3).  Same transport we already use for BKG.

- Works with **any** receiver's RAWX (no E6 needed) — so it covers the
  F9T/F9P hosts.
- Registration: free but **application-reviewed** by EUSPA (use-case,
  EU-benefit, security) — not anonymous, not instant.  Register at the
  [GSC IDD page](https://www.gsc-europa.eu/galileo/services/galileo-high-accuracy-service-has/internet-data-distribution).
- Drops into our `ssr_corrections.py` ingestion (after format
  adaptation — the HAS SSR encoding differs from the standard RTCM-SSR
  message numbers we parse for BKG; HASlib can normalize it).

### -00B vs -01B (don't buy the -01B for this)

u-blox announced the **ZED-X20P-01B** (April 2026) with *native internal
HAS PPP*.  That is the **wrong tool for us**: it computes a HAS-corrected
**PVT inside the receiver** and would hand us a black-box position/clock,
not the raw SSR corrections our own filter needs.  Our architecture wants
the *corrections* (to feed our PPP/time filter), which the **base -00B
already provides via E6-B SFRBX**.  Stick with the -00B.

## Carrier-phase timing with HAS — what it does and doesn't do for us

The moonshot transfers GPS-time long-term stability to the DO while
preserving short-term DO/rx-TCXO stability.  Where HAS lands on that:

| τ regime | Limiter today | Does HAS help? |
|---|---|---|
| **Short τ (≤ few s)** | rx-TCXO + DO free-running noise; carrier-phase TD-CP ~5–10 ps, TDCP(1s) ~40–100 ps | **No.** Already far below HAS clock noise (~0.1–0.2 ns). Corrections don't touch this. |
| **Long τ (tracking GPST)** | real-time orbit/clock quality; float-ambiguity + ZTD + position null wander | **Yes.** HAS clock 0.17 ns vs 0.66 ns broadcast; sub-ns float-PPP time transfer; cleaner orbits reduce null drift. |
| **Cross-host agreement** | per-host correction differences | **Yes.** Two hosts on the *same* HAS stream share common-mode orbit/clock error → tighter Δ between their PPS (the 1 ns / 2 ns excursion goals). |

So HAS is a **long-τ accuracy and cross-host-agreement** lever, not a
short-τ one.  It also gives a non-surveyed host a clean autonomous
position, and (in Phase 2, with phase biases) integer AR would attack
the position/ZTD/clock null that float PPP leaves under-determined.

### Timing datum caveat

HAS clock corrections are referenced to a HAS-internal timescale tied to
Galileo System Time (GST), not directly to GPS time or UTC.  For
*disciplining a DO* and for *cross-host agreement* this is fine (both
sides share the same datum; the absolute GST–GPST–UTC offsets are
broadcast constants).  For absolute UTC traceability the datum offset
must be applied — a known, slowly-varying constant, not a noise term.

## Integration sketch for PePPAR Fix

```
X20P (PiPuss) ── RXM-SFRBX (E6-B C/NAV, sigId 8) ──┐
                                                    ├─ HASlib/decoder ─→ HAS SSR
HAS IDD (NTRIP, any host) ──────────────────────────┘                     │
                                                                          v
                                        our SSRState  <- (orbit/clock/code-bias)
                                                                          │
                          RXM-RAWX ─→ float PPP time filter (pinned ARP) ─┘
                                                │
                                                v  cleaner long-τ dt_rx → DO discipline
```

- **Decode**: re-assemble E6-B pages from RXM-SFRBX (or read IDD NTRIP),
  decode with [HASlib](https://github.com/nlsfi/HASlib) or CSSRlib to
  orbit/clock/code-bias SSR.
- **Ingest**: adapt into `SSRState`; our filter already consumes
  orbit/clock/code-bias.  No phase-bias path needed yet (Phase 1).
- **Validate**: `ssr.summary()` shows non-zero HAS orbit/clock counts;
  compare HAS-corrected float-PPP dt_rx long-τ TDEV vs broadcast-only,
  and vs the BKG SSR path, on the same X20 RAWX.
- **Measure**: the headline experiment is HAS-vs-broadcast float-PPP
  **time-transfer stability** on PiPuss (no DO needed — same DO-less
  setup as the TDCP comparison; see
  `reference_tdcp_log_no_do_receiver_comparison`).

## Positioning error — summary

| Configuration | Horizontal (95%) | Vertical (95%) | Convergence |
|---|---|---|---|
| HAS Phase 1, float PPP (today) | ≤ 20 cm (target); ~10–20 cm typical | ≤ 40 cm | ≤ 300 s target; ~few min |
| HAS Phase 2, PPP-AR (future) | cm-level | cm-level | ≤ 100 s (SL2, Europe) |
| Broadcast-only (our fallback) | 1–5 m | — | seconds |

Service availability ~99.9% GPS / 99.6% Galileo globally; ~9 GPS and ~8
Galileo SVs with valid HAS corrections at a time.

## Recommendation

1. **First, prove the free SIS path** on PiPuss: decode the X20's E6-B
   SFRBX HAS pages → SSR → compare HAS vs broadcast float-PPP time
   transfer (DO-less, like the TDCP comparison).  Zero cost, zero
   dependency, and it's the natural pairing with the E6-only X20.
2. **Also register for IDD** (free) so the F9T/F9P hosts can use HAS too
   and so we have an NTRIP fallback that needs no E6 decoding.  Start the
   application now since it's reviewed, not instant.
3. **Do not** buy the -01B for HAS — its internal PPP is the wrong
   interface for our filter; the -00B already exposes the SIS.
4. **PPP-AR with HAS is a Phase-2 item** — track the HAS phase-bias
   rollout; until then, AR (if pursued) stays on CNES GAL E1/E5a.

## References

- [Galileo HAS overview (GSC)](https://www.gsc-europa.eu/galileo/services/galileo-high-accuracy-service-has) ·
  [HAS Info Note (PDF)](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_HAS_Info_Note.pdf) ·
  [HAS Service Definition Document (PDF)](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo-HAS-SDD_v1.0.pdf)
- [HAS IDD + registration (GSC)](https://www.gsc-europa.eu/galileo/services/galileo-high-accuracy-service-has/internet-data-distribution)
- Phase-1 content (no phase bias) + accuracy:
  [HAS performance & anomaly mitigation (GPS Solutions)](https://link.springer.com/article/10.1007/s10291-023-01555-w) ·
  [HAS SSR product quality & PPP performance (ScienceDirect)](https://www.sciencedirect.com/science/article/abs/pii/S0273117724011438) ·
  [Performance evaluation of HAS for PPP-AR (GPS Solutions 2025)](https://link.springer.com/article/10.1007/s10291-025-01852-6)
- Timing / time transfer (the carrier-phase-timing case):
  [Evaluation of timing and time transfer with PPP using HAS (ScienceDirect)](https://www.sciencedirect.com/science/article/abs/pii/S0263224124000368) ·
  [Initial & comprehensive analysis of PPP time transfer based on HAS (GPS Solutions)](https://link.springer.com/article/10.1007/s10291-024-01633-7) ·
  [Real-time integrated precise kinematic time transfer based on HAS (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12115877/)
- u-blox X20P / HAS: [ZED-X20P product page](https://www.u-blox.com/en/zed-x20p) ·
  [ZED-X20P-01B native HAS announcement (Inside GNSS)](https://insidegnss.com/u-blox-launches-zed-x20p-01b-with-global-ppp-and-galileo-has-support/) ·
  forum "E6 tracked but not used" on the -00B (portal.u-blox.com)
- Decoders: [HASlib (NLS Finland, GitHub)](https://github.com/nlsfi/HASlib) · CSSRlib
- Internal: `docs/f9t-firmware-capabilities.md` (X20P E6 tracking),
  `docs/correction-sources.md`, `docs/ssr-mount-survey.md`,
  `docs/ssr-requirements-by-receiver.md`,
  memory `reference_zed_x20p_pipuss_bringup`,
  `reference_tdcp_log_no_do_receiver_comparison`
