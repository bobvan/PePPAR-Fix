# GPSDO noise, and the external-clock receiver architecture

*Foundational note — the mental model behind why peppar-fix exists, and
what an external-clock receiver (Trimble NetRS / NetR9 / Septentrio
PolaRx …) could change about the short-τ floor.*

This started as three questions about DO and GNSS noise; it's written up
here so the reasoning survives a mangled tmux scrollback.

---

## 1. The GPSDO noise model — two oscillators and a crossover

A GPSDO is a control loop stitching together two very different noise
sources. Plot TDEV (or ADEV) vs τ on log-log axes:

- **The Disciplined Oscillator (DO)** — *rises* with τ. A good oscillator
  is a flywheel: intrinsically quiet short-term (a high-Q resonator, with
  nothing to average), but it drifts/ages long-term because it has no
  absolute reference.
- **The GNSS reference, as seen through the receiver** — *falls* with τ.
  It's a noisy-but-unbiased measurement: poor short-term (a statistical
  estimate from finite-SNR radio signals through a turbulent atmosphere,
  timed by a real clock), excellent long-term (anchored to the UTC
  ensemble with unlimited averaging).

They cross. The loop bandwidth is tuned to that crossover, and the
disciplined output is the **lower envelope** of the two curves:

- **Below crossover (short τ):** output = DO noise — the loop can't
  correct that fast, so the flywheel shows through.
- **Above crossover (long τ):** output = GNSS-transferred noise — the loop
  steers the DO to GPS, averaging out the sawtooth and PPS jitter.

The whole game is to put the crossover at the intersection so you get the
*best of both*: the DO's short-term quiet **and** GPS's long-term truth.

### 1a. Upgrading the oscillator: holdover vs normal running

A DO upgrade splits cleanly by *which part of its noise* you improve:

- Improving the DO's **short-τ noise floor** (the region τ < crossover,
  where the DO is the limiting curve) **does** lower the normal-running
  output there — and pulls the crossover inward, since a quieter DO now
  beats GPS out to a shorter τ.
- Improving only the DO's **long-τ drift / aging** changes **nothing** in
  normal locked operation — GPS discipline already masks it above the
  crossover. It helps **holdover only** (GPS lost → loop opens → DO
  free-runs; a better oscillator coasts longer).

So the intuition "an upgrade helps holdover but not normal running unless
it's below the GPS curve at τ < crossover" is right. The edge case it
captures: an oscillator so poor that GPS beats it even at short τ pushes
the crossover to zero — then the oscillator matters *only* for holdover.
In practice a better part (TCXO → OCXO → Rb) improves both regimes, but
they map to two distinct, separable benefits.

### 1b. Why even "ideal" GNSS is noisy at short τ

Because GNSS time is **estimated from noisy measurements**, not read off a
resonator. Three intrinsic contributors, all of which *average down* with
τ (so short τ = high, long τ = low):

1. **Per-epoch observation noise** — thermal (C/N₀-limited), residual
   tropospheric turbulence, multipath. A single epoch is limited by this;
   it beats down as √N over epochs and satellites. Short τ = few
   independent samples = high noise.
2. **The receiver's own clock (the rx TCXO)** — *every* observation is
   timestamped against it, so a single-receiver solution can't be pulled
   below the rx TCXO's short-τ noise. This is the hard floor in our world:
   PPS edges, qErr, PPP dt_rx all inherit it. It's *why* a cleaner
   receiver matters (§2), and the thing §3 attacks directly.
3. **The reference/corrections themselves** — satellite clocks and
   real-time SSR corrections update at finite rates and carry their own
   short-τ noise.

The contrast with an oscillator is the whole point: an oscillator is a
**flywheel** (great short-term, drifts long); GNSS is a **noisy absolute
reference** (poor short-term because it's a measurement, excellent long
because it's anchored to UTC). The crossover is exactly where "flywheel
coasting error" equals "measurement averaging error."

---

## 2. What peppar-fix actually buys — carrier phase, not PPS

A naïve intuition says "a receiver with lower short-τ noise gives a GPSDO
with lower short-τ noise." In a **PPS**-disciplined GPSDO that's mostly
false: at short τ the *DO* dominates the output, and the receiver's
short-τ noise sits **above** the loop bandwidth and gets filtered out.
What the receiver noise actually sets is the **achievable loop bandwidth /
crossover**: a noisy PPS forces a low loop bandwidth (crossover at
hundreds of seconds), stranding you on the DO alone across all of mid-τ.

That's the cleanest way to state peppar-fix's value versus "just average
the F9T PPS":

- The **F9T PPS is a lossy rendering** of the receiver clock — one edge
  per second, quantized to ~8 ns (125 MHz divider), plus sawtooth.
  Averaging it recovers long-τ GPS stability but is **floored by that ~ns
  measurement noise** at the handoff, and forces a slow loop.
- peppar-fix instead servos on the receiver's **carrier phase** — the
  *same* receiver clock, through a window ~100–300× cleaner than the PPS.
  The headline **~5–10 ps/epoch** figure is *time-differenced* carrier
  phase (TD-CP), which is a clean **frequency** observable; the absolute
  clock *phase* (`dt_rx` from PPP) is noisier per epoch because it's
  correlated with position/ZTD/ambiguities. Either way, removing the PPS
  quantization floor lets the crossover move far inward, so GPS stability
  is transferred faithfully down into **mid-τ (seconds to hundreds of
  seconds)** — exactly the band a PPS loop can't reach, and where the
  two-clock excursion budget lives.

So the "cleaner receiver → better GPSDO" intuition is right *if* you read
"cleaner" as "use the receiver's carrier phase instead of its PPS." That
is the move. **Honest caveat:** this is the *target*. Our own
de-sawtooth benchmarks show the theoretical mid-τ win isn't fully
realized yet (a loop-resonance hump at τ≈10–30 s); today we clearly win at
τ=1 s, and closing the mid-τ gap is the active servo-tuning work.

Even with carrier phase, note contributor #2 above: the observable is
still `(satellite − rx_TCXO)`, so the **rx TCXO short-τ noise is the
floor** peppar-fix cannot pull the DO below using GNSS inputs. That is the
premise §3 breaks.

---

## 3. The external-clock receiver — clocking the receiver *with the DO*

Some geodetic/timing receivers accept an **external frequency reference**
(typically 10 MHz, sometimes 5 MHz) in place of their internal TCXO.
Bob has a **Trimble NetRS** on order; the NetR9, Septentrio PolaRx, Javad,
and some NovAtel boards do the same.

**⚠ Load-bearing assumption to verify: the NetRS external 10 MHz input.**
An external frequency input is a definite NetR5/NetR9 feature, but is
*unconfirmed* on the 2003-era NetRS — check the rear panel for a `REF IN` /
10 MHz connector before relying on it. This is the single most load-bearing
assumption in this section: without it the NetRS is just an L1/L2 receiver,
not an external-clock PoC at all (the entire §3f-step-1 path rests on it).

### 3a. What it changes — eliminating the rx TCXO

Feed the **DO's own 10 MHz** into the receiver and something fundamental
changes: the receiver's code and carrier-phase measurements are now timed
against **the DO**, not a separate internal oscillator. The carrier-phase
observable becomes `(satellite − DO)` instead of `(satellite − rx_TCXO)`.
After the usual PPP/SSR correction of geometry, atmosphere, and satellite
clock, **the residual is the DO's phase error vs GPS time, measured
directly** — at carrier-phase (picosecond) precision, with **no separate
rx TCXO noise in the path.**

This collapses peppar-fix's two-oscillator problem into one:

| | rx on its own TCXO (today) | rx clocked by the DO |
|---|---|---|
| Servo observable | `(DO − rx_TCXO)` via TICC/EXTINT **plus** `(rx_TCXO − GPS)` via PPP dt_rx — combine, both carry rx TCXO noise | `(DO − GPS)` **directly** via PPP dt_rx (= dt of the DO) |
| Noise floor at short τ | rx TCXO (contributor #2) | GNSS measurement noise — thermal/atmo/multipath **and corrections/SSR** (§1b #1 & #3), which also survive clocking-by-DO |
| Extra hardware | TICC + a DO-PPS↔rx-PPS comparison to remove rx clock | none — the receiver *is* the GPS-vs-DO phase meter |

The receiver becomes a transparent **"GPS-minus-DO phase comparator."**
This is precisely how high-end timing receivers and UTC(k)-lab GPSDOs are
architected, and it directly attacks the moonshot's stated floor —
CLAUDE.md's *"we can't pull the DO below the rx TCXO's stability using
GNSS-based inputs."* Clock the receiver with the DO and that constraint is
gone: the rx TCXO is no longer in the loop. The only remaining short-τ
limits are the DO itself (which is what we're measuring and controlling)
and the GNSS measurement noise.

**So yes — clocking a NetRS-class receiver with our OCXO DO should help
short-τ performance by removing the rx TCXO as a noise source.** It's
arguably the single biggest architectural lever available to the moonshot.

### 3b. Caveats and subtleties

- **Pull range.** External frequency inputs expect ~10 MHz within a
  bounded tolerance (often ±a few ppm). Our DO is disciplined near
  nominal, so it stays in range — but a *cold* / undisciplined DO at
  power-on could be out of pull-in until it's steered. Bootstrap order
  matters.
- **Loss of an independent cross-check.** Today the DO and rx TCXO are
  independent, so a TICC comparison and the PPP solution are partially
  redundant — useful for catching measurement-chain faults (cf. the qErr
  qVIR sanity gate). With one clock feeding everything, DO noise and
  receiver-measurement noise are harder to separate. For *disciplining*
  that's fine (you want to see and control the DO); for *characterization*
  you'd keep a second, independently-clocked receiver or a TICC on hand.
- **Self-referenced, not circular.** The DO clocks the receiver *and* is
  disciplined by the receiver's output — but that's an ordinary
  sensor→plant feedback loop, not a logical circularity. The receiver
  tracks satellites and reports the clock bias of its time base regardless
  of the DO's exact frequency (within pull range); the servo steers the DO
  to null that bias.
- **It doesn't lower the GNSS measurement floor**, only removes the rx
  TCXO on top of it. Thermal/atmosphere/multipath still set the short-τ
  measurement noise; that's contributor #1, attacked only by C/N₀,
  geometry, multipath control, and averaging.

### 3c. Faster than 1 Hz?

Yes, and it matters. Raw-measurement rate is a real lever: faster carrier
phase lets the servo *react to DO noise sooner*, raising the achievable
loop bandwidth and pulling the crossover to shorter τ — helping exactly
the mid-τ band §2 targets. Nuance: at high rates the atmosphere and
multipath barely change between epochs, so the *independent-information*
rate is bounded by physics, not the sample rate; you get diminishing
returns on noise-averaging but a genuine gain in loop-bandwidth headroom
where the DO is the limiter. Typical ceilings (verify per unit/firmware):

- Older reference receivers (NetRS-class): often 1 Hz, sometimes up to
  ~5–20 Hz.
- NetR9 / Septentrio PolaRx / Javad: up to 50–100 Hz raw obs.

### 3d. Bands — the L1/L2-era trap, and better options

The catch with the **NetRS**: it is **GPS L1/L2 only** — no GLONASS, no
Galileo, no BeiDou, **no L5**. That's a poor match for our current stack,
which is built around F9T **L1 + L5**, **Galileo E1 + E5a**, and BeiDou
**B1I + B2a-I**, with SSR corrections and **GPS + Galileo E1/E5a PPP-AR**
plus BeiDou *tracking* (BeiDou is not AR'd — no SSR AC publishes B2a-I phase
biases, so BDS is dropped from the default `--systems gps,gal`; see
`docs/bds-b2a-phase-bias-survey-2026-05-09.md`). On a NetRS you'd be back to
dual-frequency GPS-only PPP: fewer satellites, weaker geometry, no
L5/Galileo AR, no multi-GNSS robustness.

The **same L1/L2-era limitation** applies to two other used receivers
worth naming, since they show up cheap:

- **NovAtel OEMV-3** (~2006–2011) — GPS L1/L2 **+ GLONASS**, but **no
  Galileo/BeiDou/L5** (L5 arrived with OEM6/7). A bare OEM *board* needing
  enclosure + power + interface integration. NovAtel boards do support an
  **external oscillator input** and PPS out, so it's usable as a
  GPS+GLONASS external-clock PoC.
- **Leica GRX1200** — L1/L2 (+ GLONASS on the GG/Pro variants), **no
  L5/Galileo/BeiDou**. Whether it accepts an external 10 MHz reference is
  **unconfirmed** — that's a definite feature of the newer Leica GR-series
  (GR10/25/30/50) and only *maybe* on GRX1200 Pro, so verify the rear panel
  for a "REF IN"/10 MHz connector before assuming it.

All three — **NetRS, OEMV-3, GRX1200** — are **cheap proof-of-concept
units only**: fine for "does clocking the receiver with the DO measurably
drop our short-τ floor?", but a step *back* in constellation/band coverage
that can't run our multi-GNSS L5 PPP-AR pipeline.

For a **production** external-clock timing receiver that keeps our bands,
in rough order of preference (all support an external 10 MHz input; verify
current-unit specs **and firmware options** before buying used):

- **Septentrio PolaRx5 / PolaRx5TR** — full multi-GNSS, all bands, up to
  100 Hz, external 10 MHz in. The **PolaRx5TR is a purpose-built time-&-
  frequency-transfer receiver** used in UTC(k) labs — the closest match to
  what we're actually doing. Priciest used, best fit.
- **Septentrio mosaic-T** — the *module-scale* Septentrio timing engine
  (same lineage as the PolaRx). Multi-frequency multi-GNSS (GPS L1/L2/L5,
  Galileo E1/E5a/E5b, BeiDou B1/B2a, GLONASS; verify the exact band set),
  native **SBF + RTCM 3 MSM** output, USB/UART/**Ethernet**, up to ~100 Hz,
  and an **external 10 MHz input**. Cheaper and far smaller than a PolaRx5TR
  while keeping our whole L5/E5a/B2a AR stack — a strong external-clock
  receiver candidate. **We already have one in the lab** — it's the GNSS
  engine inside the SparkFun/SparkPNT GNSSDO+ (see §3g).
- **Trimble NetR9** — multi-GNSS (GPS/GLO/GAL/BDS/QZSS), L1/L2/L5/E5/B2,
  external 10 MHz in, high rate. Widely available used, strong all-rounder.
  **Trimble option-lock caveat:** modern-signal tracking (L5 / E5a / B2a)
  is often a **paid, serial-locked firmware option** — verify the
  *installed option codes* on the actual unit (Receiver Status → Options),
  not a "Ti-N unlocked" claim in the listing prose.
- **Javad (Delta / TRE-3)** — multi-GNSS all bands, external frequency in.
- **NovAtel OEM7 / PwrPak7** — multi-GNSS all bands; external oscillator
  input on some variants (confirm the specific board).

**One practical gotcha across all these units: I/O is a cabling problem,
not a licensing one.** PPS output and the external frequency input are
hardware features — **no dongle/license** — but these receivers use
multi-pin circular (Lemo/ODU) connectors, so you need the correct
**breakout cable** to reach 1PPS-out (for cross-host TICC comparison) and
10 MHz-in (to feed the DO), and used units frequently **don't include**
them. Budget for sourcing/making the cables, and expect a one-time config
step (enable PPS, set cable delay) in the receiver UI.

### 3e. Getting the raw data out — interfaces & formats

Not F9T-over-USB — the trade is friendly USB enumeration for
**network-native streaming + a proprietary binary format**. Different, not
necessarily harder.

**Hardware.** The packaged reference receivers (NetRS, NetR9, PolaRx,
GRX1200) are **Ethernet-first**: raw data streams over a TCP socket,
config is a web UI, and most can act as an NTRIP server directly — once
set up that's *cleaner* than USB (the engine just reads a socket). All
also carry **RS-232 serial** (high baud, often Lemo/DB9); on the bare
**OEMV-3 board** a serial UART is the *primary* interface (wire to pins —
the least convenient). USB, where present (NetR9/PolaRx5), is usually
**USB-serial (CDC)** or thumb-drive host, not plug-and-play enumeration.
Most also log to **CF/internal memory** for post-processing.

**Format.** Each vendor speaks its own binary raw format, so you need a
decoder:

| Vendor | Native raw obs (+ PVT/clock) |
|---|---|
| Trimble (NetRS / NetR9) | **RT17 / RT27**; GSOF for PVT/clock |
| NovAtel (OEMV-3 / OEM7) | **OEM binary** (`RANGEB`/`RANGECMPB`, `RAWEPHEMB`) |
| Septentrio (PolaRx) | **SBF** (very well documented) |
| Leica (GRX1200) | Leica proprietary (LB2 — poorly documented; use RTCM) |

**The bridge that fits our stack.** The engine already parses RTCM
(pyrtcm, for the SSR streams), so two clean paths, best first:

1. **Configure the receiver to stream RTCM 3 MSM (MSM4/MSM7) over TCP** —
   the standard multi-GNSS raw code+phase+Doppler+CNR format, the
   cross-vendor lingua franca, landing on plumbing we already have. Least
   new code.
2. **Put RTKLIB in front as a universal translator** — `str2str` reads the
   native binary (RT27 / SBF / OEM / …) off TCP or serial and re-emits RTCM
   or RINEX; `convbin` does the same for files. Handles every vendor above.

For the external-clock experiment, either the raw obs (our PPP computes
`dt`) or the receiver's own clock-bias output (Trimble GSOF / NovAtel
`CLOCKMODEL` / Septentrio SBF) delivers `dt_DO` directly. Integration cost
= network/serial setup + a format bridge (prefer RTCM-MSM, else RTKLIB
`str2str`) + mapping their obs into our observation model — a mature
ecosystem, closer to "wire it up and configure a stream" than "write a
driver from scratch."

### 3f. How this would slot into peppar-fix

The engine already estimates `dt_rx` from carrier phase. In the
external-clock architecture, `dt_rx` simply *becomes* the DO's clock bias
vs GPS — the servo arm we most want, delivered directly by the receiver
with the rx TCXO removed from the path. (Note this is the absolute clock
*phase*, so it carries `dt_rx`'s per-epoch precision — not the tighter
TD-CP *frequency* figure from §2; the win is deleting the rx TCXO, not
changing which observable's noise applies.) The natural experiment:

1. Prove the concept cheaply on the NetRS (GPS L1/L2), OCXO DO feeding its
   external 10 MHz, and measure the short-τ floor of `dt_DO` vs our
   current F9T-on-its-own-TCXO arms (TICC chA baseline as the yardstick).
2. If it drops the short-τ floor toward the GNSS-measurement limit as
   predicted, move to a NetR9 / PolaRx5TR to regain L5 + Galileo + BeiDou
   and the SSR PPP-AR pipeline, now on a single, DO-referenced clock.

That is the cleanest path we have to the moonshot's "as good as the best
of (DO floor, rx TCXO floor) at every τ" — because it deletes the rx TCXO
floor from the problem entirely.

### 3g. The SparkFun/SparkPNT GNSSDO+ as a whole target platform

The GNSSDO+ we already use as the lab reference is *the entire external-clock
architecture in one commercial box*, and — with two changes — could become a
self-contained **peppar-fix node**:

- **DO:** a good **Rakon STP3593LF / ROX5242T1N OCXO** (10 MHz,
  ~1.5×10⁻¹² typ ADEV, −130 dBc/Hz — Rb-grade at short τ; specs +
  SparkPNT datasheet in `docs/pulsepuppy-ocxo-buying-guide.md`).
- **Receiver:** a **Septentrio mosaic-T** (§3d), multi-GNSS L5/E5a,
  RTCM-MSM-capable.
- **Already external-clocked:** the OCXO feeds the mosaic-T's external
  reference (`docs/do-characterization-architecture.md` flags this
  topology — currently treated as an out-of-scope black box), so the
  receiver's obs are already `(satellite − OCXO)`, rx-TCXO-free.
- **Steering today:** the shipped **ESP32 firmware** disciplines the OCXO
  from an error signal the mosaic-T produces — by default a multi-
  constellation **float PPP** solution, optionally refined by **AtomiChron**
  (Fugro's subscription *single-AC* correction service, which tightens
  accuracy but ties the timescale to Fugro's). Our PPS label
  "GNSSDO+AtomiChron" denotes this hardware running *with* that correction.
  (The OCXO is quartz, Rb-*class* — distinct from the lab's actual Rb
  standard, the FE-5680A, which is a separate 10 MHz reference / TICC clock.)

The two changes that make it a peppar-fix target:

1. **Feed the mosaic-T's raw obs (RTCM 3 MSM) into peppar-fix** — the ingest
   pipeline (`docs/rtcm-msm-obs-ingest.md`) already consumes exactly this.
2. **Replace/augment the ESP32 firmware so peppar-fix supplies the
   steering** — instead of the stock closed-loop discipline, the ESP32
   accepts an external frequency/DAC command from peppar-fix (which computes
   it from the mosaic-T obs), or peppar-fix drives the OCXO's EFC DAC
   directly. This also **swaps the correction source**: the stock loop runs
   Fugro's float-PPP/AtomiChron (their timescale, a subscription); peppar-fix
   would run its own **PPP-AR** against our SSR streams — ambiguity-resolved,
   carrier-phase-timing discipline, on our GPS/ITRF timescale, no
   subscription. That's a real reason to do it, not just re-plumbing.

The result is the moonshot architecture in a cheap off-the-shelf enclosure:
a multi-GNSS L5 receiver clocked by a good OCXO (rx TCXO deleted), disciplined
by our servo from carrier-phase obs. **Gating unknowns** (→ dayplan): (a) does
the enclosure expose the mosaic-T data/config port (Ethernet/UART) and let us
enable RTCM output; (b) is the ESP32 firmware replaceable/augmentable, and
what's the OCXO's EFC/steering interface (DAC vs the ESP32's control path).

---

*See also: `docs/two-site-sync-budget.md` (the excursion budget the mid-τ
floor feeds), `docs/ticc-baseline-2026-04-01.md` (the F9T PPS / EXTTS
quantization floors this contrasts with), `docs/glossary.md` (DO, rx TCXO,
gnss_pps/do_pps), and the moonshot framing in `CLAUDE.md`.*
