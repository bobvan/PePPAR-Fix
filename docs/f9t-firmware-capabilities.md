# u-blox ZED-F9 Firmware Capability Matrix

Experimentally determined via CFG-VALSET probing on lab receivers
sharing a common antenna/splitter:
  - F9T variants: 2026-04-14 (factory reset on F9T-TOP, running
    state on F9T-BOT), follow-up on ptpmon 2026-04-18.
  - F9P variants: 2026-04-30 (per-key probe via /tmp/f9p_probe.py
    on clkPoC3, F9P-1 idle after the rawx-log overnight).

## Test receivers

| Name | Module | Firmware | PROTVER | SEC-UNIQID | Host | vpManager_07 |
|------|--------|----------|---------|------------|------|--------------|
| F9T-TOP | ZED-F9T | TIM 2.20 | 29.20 | 136395244089 | TimeHat | 1 |
| F9T-BOT | ZED-F9T-20B | TIM 2.25 | 29.25 | 262843023907 | MadHat | 1 |
| F9T-PTP | ZED-F9T | TIM 2.20 | 29.20 | 675836739647 | ptpmon | **0** |
| F9P-1 | ZED-F9P (HPG 1.51) | EXT CORE 1.00 | 27.50 | 904584649306 | clkPoC3 | (n/a) |
| F9P-2 | ZED-F9P (HPG 1.51) | EXT CORE 1.00 | 27.50 | 914202187869 | clkPoC3 | (n/a) |

`vpManager_07` is bit 7 of the virtual-pin manager bitmap returned by
UBX-MON-HW3.  On the tested F9T receivers it correlates perfectly
with the presence of the 1176.45 MHz (L5/E5a/B2a) RF front-end —
F9T units with vpManager_07=0 NAK every signal in that band.  The
firmware/module strings give no such hint: ptpmon and TimeHat
report identical `MOD=ZED-F9T`, `FWVER=TIM 2.20`, `PROTVER=29.20`,
and `ROM BASE 0x118B2060`.  F9P is a different module family
(HPG firmware, not TIM); the vpManager probe wasn't run on F9P
because the L5-band signal probe directly NAKed every 1176.45 MHz
key — the hardware front-end isn't there.

## Signal capability matrix

Tested by sending CFG-VALSET (RAM layer) for each key individually.
ACK = accepted, NAK = rejected by firmware.

| Capability | CFG key | ZED-F9T (2.20, L5-hw) | ZED-F9T (2.20, L2-only hw) | ZED-F9T-20B (2.25) | ZED-F9P-15 (HPG 1.51) |
|---|---|---|---|---|---|
| GPS L1 C/A | CFG_SIGNAL_GPS_L1CA_ENA | ACK | ACK | ACK | ACK |
| GPS L2C | CFG_SIGNAL_GPS_L2C_ENA | **ACK** | **ACK** | **NAK** | **ACK** |
| GPS L5 | CFG_SIGNAL_GPS_L5_ENA | **ACK** | **NAK** | ACK | **NAK** |
| GPS L5 health override | 0x10320001 | **ACK** | **ACK** | ACK | (not tested) |
| GAL E1 | CFG_SIGNAL_GAL_E1_ENA | ACK | ACK | ACK | ACK |
| GAL E5a | CFG_SIGNAL_GAL_E5A_ENA | ACK | **NAK** | ACK | **NAK** |
| GAL E5b | CFG_SIGNAL_GAL_E5B_ENA | **NAK** | **ACK** | **NAK** | **ACK** |
| GLONASS L1 | CFG_SIGNAL_GLO_L1_ENA | **NAK** | **NAK** | **NAK** | **ACK** |
| GLONASS L2 | CFG_SIGNAL_GLO_L2_ENA | **NAK** | **NAK** | **NAK** | **ACK** |
| NavIC | CFG_SIGNAL_NAVIC_ENA | **NAK** | (not tested) | **ACK** | (not tested) |
| BeiDou B1 | CFG_SIGNAL_BDS_B1_ENA | ACK | ACK | ACK | ACK |
| BeiDou B2 (B2I) | CFG_SIGNAL_BDS_B2_ENA | (not tested) | **ACK** | (not tested) | **ACK** |
| BeiDou B2a | CFG_SIGNAL_BDS_B2A_ENA | (not tested) | **NAK** | (not tested) | **NAK** |

## Key findings

### Neither receiver supports GLONASS

Both NAK `GLO_ENA`. The MON-VER extension string on TIM 2.20 lists
"GPS;GLO;GAL;BDS" but this appears to be a static ROM string, not a
reflection of actual capability. The -20B drops GLO from the string
entirely.

### ZED-F9T (TIM 2.20) ships in two RF variants — firmware can't tell

The plain `MOD=ZED-F9T` string covers two physically distinct parts:

- **L5-capable variant** (F9T-TOP on TimeHat) — 1176.45 MHz front-end
  present.  Accepts L5/E5a/B2a; NAKs E5b.  Can run either L2C or L5
  as the second GPS band (two-band RF limit — not simultaneous).
- **L2-only "classic" variant** (F9T on ptpmon) — no 1176.45 MHz
  front-end.  NAKs L5/E5a/B2a; accepts L2C + E5b + B2I as the
  second-band signals.  GPS L5 health override still ACKs (it's a
  firmware-only CFG key), but any attempt to enable an L5-band
  signal NAKs regardless of override or factory reset.

The variants report identical firmware/module/PROTVER strings.  The
only way to tell them apart from software is **MON-HW3 vpManager_07**
(1 = L5-capable, 0 = L2-only).  Configuration failures that look
like a firmware dependency-ordering problem on an "identical" unit
are almost always this hardware split.

The L5-capable ZED-F9T (TIM 2.20) **accepts both L2C and L5
configuration**. It can run either signal plan, just not
simultaneously (two-band RF chain limit).

However, there is a sequencing constraint when changing bands via
**individual** CFG-VALSET keys (not relevant when sending all keys
in a single VALSET, which is what `configure_signals()` does):

- Setting L5_ENA=1 when L2C is on: **ACK** — L2C auto-clears
- Setting L2C_ENA=1 when L5 is on: **ACK** — appears to succeed
- Setting L5_ENA=0 after L2C_ENA=1: **NAK** — even though L5 is
  already off (the receiver treats this as a conflicting request)
- After the NAK, both L2C and L5 read as 0 — the receiver is in
  L1-only mode

The safe pattern for individual key changes: always set the desired
band to 1 first (which auto-clears the other), then explicitly set
the other to 0. But the production code avoids this entirely —
`configure_signals()` sends a complete signal config in one VALSET
message, and the receiver applies it atomically.

### ZED-F9T-20B (TIM 2.25) lost L2C, gained NavIC

The -20B module **NAKs L2C**. This is a hard firmware restriction,
not a default preference. The -20B can only run L5 as its second
frequency.

In exchange, the -20B gained NavIC support.

### GAL E5 is hardware-dependent

E5a (1176.45 MHz) and E5b (1207.14 MHz) are gated by different RF
front-ends:

- **L5-capable hardware**: accepts E5a, NAKs E5b.
- **L2-only hardware**: accepts E5b, NAKs E5a.

The `F9TDriver` L2 profile originally specified E5b (classic L1+L2
u-blox F9T intent), was changed in commit 096dbdc to E5a after
observing E5b NAKs on L5-hardware units — but that commit's test
receivers were all L5-hardware.  Both profiles are needed:

- `F9TDriver` — L2 profile with E5a (for L5-hardware units running
  L2 as a diagnostic mode; GAL single-freq on second band)
- `F9TL2E5bDriver` — L2 profile with E5b (for classic L2-only
  hardware like ptpmon; full GAL dual-band)

CNES SSRA00CNE0 publishes both L5Q (E5a) and L7Q (E5b) phase biases,
so GAL dual-band AR works on either hardware variant with the existing
SSR pipeline.

### ZED-F9P-15 (HPG 1.51) is L1+L2 only — no L5/E5a/B2a-I path

The lab's two F9Ps (clkPoC3, post-DO-decommission slot) NAK every
1176.45 MHz signal: GPS L5, GAL E5a, BDS B2a-I.  The F9P-15 module's
RF front-end is L1+L2 band only.  Confirmed 2026-04-30 with
`/tmp/f9p_probe.py` against F9P-1.

The F9P signal set looks like the L2-only F9T variant, with one
addition: **GLONASS dual-band**.  GPS L1+L2C, GAL E1+E5b, BDS
B1I+B2I, GLO L1+L2.

There is a newer F9P-04 module that adds L5/E5a/B2a tracking
(announced post-2023) but our F9P-15 hardware predates it.
Configuration tweaks cannot make our F9P-15 produce L5-band
observations — the front-end isn't there.

**Practical implication for PPP-AR:** WUM rapid products
(used by PRIDE-PPPAR) provide observable-specific phase biases
keyed to L5/E5a/B2a-I codes, not E5b or B2I.  PRIDE on F9P-15
data therefore drops every GAL and BDS observation as
unmatched-OSB and falls back to GPS-only PPP-AR (~15 GPS NL
fixes, 0 GAL, 0 BDS) — the residual systematic biases land in
the GPS float ambiguities and position lands ~1.7 m off the
F9P-RTK ground truth, not the cm-level we'd see on F9T-20B.

This makes the **ZED-F9T-20B (2.25) better suited to PPP-AR
with WUM products than the ZED-F9P-15**, despite the F9P being
a more expensive RTK-marketed receiver.  The F9P advantage is
elsewhere: it does GLONASS, supports built-in RTK rover/base
modes, and runs a different firmware family (HPG vs TIM) with
millimetre-scale RTK accuracy when paired with a base station
that emits matching legacy RTCM 3 codes.  For PPP-AR third-leg
ARP work the L5-band-tracking F9T-20B is the right tool.

### Default config after factory reset

After CFG-CFG factory reset, both L2C_ENA and L5_ENA are **OFF** (0).
The receiver boots to L1-only mode. `ensure_receiver_ready()` detects
single-frequency and applies the L5 signal plan, which is why all
receivers end up on L5 regardless of hardware variant.

The F9P factory default ships with **L2C, E5b, B2I, and both GLONASS
bands enabled**, matching its L1+L2 hardware.  No reconfiguration
needed for normal RTK use.

### Engine `--systems` is software-side only — it does NOT reconfigure the receiver

A subtle but load-bearing fact, confirmed by code-path audit
2026-05-02:

  - The engine's ``--systems gps,gal[,bds]`` argument filters
    observations in two places: ``serial_reader`` drops obs from
    non-listed systems before they enter the obs queue, and
    ``PPPFilter.initialize(systems=)`` uses the set for ISB
    pinning of single-constellation runs.
  - It is **NOT** consumed by ``configure_signals()`` in
    ``peppar_fix/receiver.py``.  ``configure_signals`` writes a
    static ``signal_config`` dict per driver
    (``F9T_L5_SIGNAL_CONFIG`` etc.) which always has
    ``CFG_SIGNAL_BDS_ENA = 1`` (and analogous keys for other
    constellations) regardless of what ``--systems`` says.
  - Net: with ``--systems gps,gal``, the receiver still tracks
    BDS at the RF level; the engine just drops the resulting
    observations.

Implications:

  - **Channel-saturation hypotheses based on `--systems` are
    invalid.**  Switching the engine config does not change
    receiver tracking demand.  ZED-F9T and ZED-F9P each have
    184 acquisition+tracking channels; full multi-constellation
    L5-band tracking (GPS L1+L5 + GAL E1+E5a + BDS B1+B2a-I,
    plus GLO if hardware allows) is well under that ceiling.
  - To truly disable a constellation at the receiver, send
    ``CFG_SIGNAL_<sys>_ENA = 0`` via ``CFG-VALSET`` directly.
    No engine code path does this today.
  - For diagnostic A/B between "BDS receiving + engine using"
    vs "BDS receiving but engine ignoring" vs "BDS not received
    at all," the third case requires manual receiver
    reconfiguration outside the engine's normal ``--systems``
    flag.  The first two are accessible via ``--systems``.

### L5 SV count is identical across firmware versions

With L5 enabled and health override applied, all three lab receivers
(TIM 2.20 + two TIM 2.25) track the same GPS L5 and GAL E5a SVs
(7 GPS L5, 10-11 GAL E5a at the time of measurement). No SV
dropout difference between firmware versions.

## Summary: what each variant can actually do

| Feature | ZED-F9T 2.20 (L5-hw) | ZED-F9T 2.20 (L2-only hw) | ZED-F9T-20B (2.25) | ZED-F9P-15 (HPG 1.51) |
|---|---|---|---|---|
| L1 + L2C | Yes | Yes | **No** | Yes |
| L1 + L5 | **Yes** | **No** | Yes | **No** |
| L5 health override CFG key | Yes | Yes (but no effect) | Yes | (n/a) |
| GLONASS L1 + L2 | No | No | No | **Yes** |
| NavIC | No | (untested) | **Yes** | (untested) |
| GAL E5a | Yes | **No** | Yes | **No** |
| GAL E5b | **No** | **Yes** | No | **Yes** |
| BDS B2I | (untested) | Yes | (untested) | Yes |
| BDS B2a | (untested, expect Yes) | **No** | (untested) | **No** |
| Match for WUM PPP-AR phase biases | **Yes** (L5/E5a/B2a-I) | No | **Yes** (L5/E5a/B2a-I) | No |

The L5-hardware ZED-F9T (TIM 2.20) is the most capable: can run
either L2 or L5, GAL dual-band via E5a.  The classic L2-only
ZED-F9T (TIM 2.20) is locked to L2, GAL dual-band via E5b.  The
-20B is locked to L5, GAL dual-band via E5a, and adds NavIC.  The
-20B's NavIC support is not useful for PPP-AR (no SSR corrections
available for NavIC).

## Why L5 is preferred for PPP-AR

PePPAR Fix always configures L5 when the receiver accepts it.  This
is a deliberate choice driven by our SSR correction source (CNES)
and the receiver's tracking mode.

### Reason 1: CNES phase bias compatibility (decisive)

CNES SSR provides GPS phase biases for these tracking modes:

    GPS: L1C (hit), L2W, L5I
    GAL: L1C (hit), E5aQ (hit), E7Q (hit)

The F9T tracks GPS L2 as **L2CL** (civil L2C, L-code).  CNES
provides **L2W** (semi-codeless Z-tracking, used by geodetic
receivers).  L2CL and L2W are different signal processing approaches
with different hardware delay characteristics — the L2W phase bias
does not apply to L2CL observations.  Result: **GPS L2 AR silently
fails** because the bias lookup misses.

For L5, CNES provides **L5I** biases.  The F9T tracks **L5Q**.  L5I
and L5Q share the same carrier frequency (1176.45 MHz), so the phase
bias applies despite the tracking mode difference.  GPS L5 AR works.

**This is CNES-specific.**  An SSR provider that published **L2L or
L2X** phase biases (matching the F9T's civil L2C tracking) would make
GPS L2 AR viable.  The L5 preference is downstream of our SSR source
choice, not a fundamental limitation.  See `docs/correction-sources.md`
for SSR stream options.

### Reason 2: Lower code noise (significant)

L5 uses BPSK(10) modulation vs L2C's BPSK(1) — roughly 10× better
code precision.  This directly affects Melbourne-Wubbena averaging:
lower code noise means faster WL convergence (fewer epochs to fix
N_WL).  With L5, WL fixing completes in ~60 epochs; L2 would need
proportionally more.

### Reason 3: Fleet uniformity (practical)

Most of our fleet is L5-capable hardware.  Preferring L5 keeps the
majority of hosts on one signal plan, which simplifies AR tuning
and cross-host comparison.  The classic L2-only variant (ptpmon)
is the exception and runs the L2 profile by necessity.

### What would change with a different SSR source?

If we switched to an SSR provider with L2L biases:
- GPS L2 AR would work (bias lookup would hit)
- L2 has a **longer** wide-lane wavelength (86.2 cm vs 75.2 cm),
  which is actually easier to fix — wider tolerance for code noise
- But L2's higher code noise partly cancels that advantage
- L5 would still be preferred on balance (code noise + universality)
  but the margin would be "better" rather than "required"

The current policy: **always L5, documented as an SSR-driven choice,
not a hardware limitation.**

### Reason 4: TGD considerations for L1/L5 (under investigation)

The GPS broadcast satellite clock is referenced to the L1/L2
ionosphere-free combination.  When using L1/L5 instead, there is a
differential group delay between L2 and L5 hardware paths on the
satellite that is not corrected by the broadcast TGD parameter.  The
correction requires `ISC_L5` from the CNAV message (not broadcast in
LNAV) or an equivalent SSR code bias.

In practice, the SSR code bias for L5 (if provided) absorbs this
differential, so SSR-corrected processing should handle it.  Without
SSR code biases, the L5 group delay differential produces a ~3-5m
per-satellite pseudorange bias — small enough to be absorbed by the
receiver clock state in the filter.

**Status (2026-04-16)**: A systematic 50m PPP position bias is under
investigation.  The TGD handling was tested (removing TGD worsened the
bias from 50m to 100m, confirming the subtraction is needed in our
pipeline).  The bias appears on both CNES and BKG SSR streams,
pointing to a measurement model issue rather than SSR-specific error.
See `memory/project_50m_bias_investigation.md` for current findings.

### Diagnostic benefit of running one host on L2

TimeHat's ZED-F9T (TIM 2.20) can run L1/L2, while MadHat and clkPoC3
are locked to L1/L5.  Briefly running TimeHat on L2 while the others
stay on L5 would provide:

1. **TGD isolation**: If the 50m bias disappears on L2 but persists on
   L5, the bias is L5-specific (group delay, ISC_L5, or L5 code bias)
2. **Signal quality comparison**: L2 and L5 pseudorange residuals can
   be compared for systematic patterns
3. **Cross-frequency AR validation**: if an L2L-compatible SSR source
   is found, L2 AR results can be compared against L5 AR

This is a one-time diagnostic, not a permanent configuration.  After
the investigation, TimeHat should return to L5 for consistency.

## Full signal/correction chain for PPP-AR

The choice of L2 vs L5 cannot be made in isolation — it propagates
through every stage of the processing chain.  Here is the full
dependency:

```
Receiver hardware (ZED-F9T variant)
  ↓ determines available signals
Signal tracking mode (L2CL vs L5Q)
  ↓ determines RINEX observation codes
SSR code bias lookup (C2L vs C5Q)
  ↓ corrects pseudorange hardware delays
SSR phase bias lookup (L2L vs L5Q → L5I)
  ↓ makes carrier-phase ambiguities integer-valued
IF combination (L1/L2 vs L1/L5)
  ↓ determines noise amplification factor and wavelengths
Melbourne-Wubbena wide-lane (λ_WL depends on f2 choice)
  ↓ convergence speed depends on code noise
Narrow-lane / LAMBDA resolution
  ↓ integer validation depends on float convergence
Position fix
```

At each stage, L2 and L5 have different characteristics:

| Stage | L1/L2 | L1/L5 | Winner |
|---|---|---|---|
| Hardware support | TIM 2.20 only | Both firmwares | L5 |
| SSR code bias | C2L (rare in SSR) | C5Q (common) | L5 |
| SSR phase bias | L2W (CNES) ≠ L2CL (F9T) | L5I ≈ L5Q (same carrier) | L5 |
| IF noise amplification | α ≈ 2.55 | α ≈ 2.26 | L5 |
| Code precision | BPSK(1), ~3m | BPSK(10), ~0.3m | L5 |
| WL wavelength | 86.2 cm | 75.2 cm | L2 |
| WL convergence | ~600 epochs | ~60 epochs | L5 |
| Broadcast TGD | Referenced to L1/L2 IF | Needs ISC_L5 correction | L2 |

L5 wins on 6 of 8 criteria.  L2's only advantages are a slightly
wider WL wavelength (easier integer fixing) and native TGD
compatibility.  Neither advantage overcomes L5's decisive phase bias
match with CNES and 10× better code precision.

## Implications for PePPAR Fix

1. **F9T-TOP on TimeHat is the only receiver that can test L2 AR.**
   Force `--receiver f9t` to prevent `ensure_receiver_ready()` from
   auto-switching to L5.  Requires an SSR source with L2L biases
   to be meaningful (CNES L2W won't work).

2. **`F9TDriver` L2 profile E5b bug (fixed 096dbdc)**: the L2
   profile previously specified `GAL_E5B_ENA=1` which NAKs on all
   tested firmware.  Now uses E5a for Galileo in both profiles.

3. **`docs/receiver-signals.md` needs correction**: the claim that
   TIM 2.20 NAKs L5 is false. Both firmwares support L5.
