# DO characterization — 2026-05-15 first cut

Bob asked (2026-05-15) for per-host DO characterization across four lab
hosts.  This doc captures what we can know from currently-running engine
data (3 days continuous on PiFace and TimeHat, ~5 h on MadHat, ~5 min on
clkPoC3 post-restart) before stopping engines for actuator sweeps.

Methodology: TICC chA-chB analysis after accounting for per-host TICC
reference offset (sign-clean — see §1).

---

## 1. Sign convention (derived 2026-05-15)

TICC measures phase residuals against its 10 MHz reference input.  If
the TICC ref is fast by δ_ref ppb relative to GPS, ref ticks advance
faster than real time, so each PPS edge falls at a later ref-residual
position by δ_ref ns/s.

Definitions (positive = fast vs GPS):
- δ_ref      : TICC ref offset from GPS (ppb)
- δ_DO       : Disciplined-oscillator output offset from GPS (ppb)
- δ_F9T ≈ 0  : F9T PPS, GPS-locked, sub-µ-ppb

Then:
```
chA_slope (ppb) = -(δ_DO - δ_ref)
chB_slope (ppb) = -(δ_F9T - δ_ref) ≈ δ_ref
```
Solving:
```
δ_ref =  chB_slope
δ_DO  =  chB_slope - chA_slope
```

This is the canonical sign convention for the analysis below.  It is
NOT the same as `dac_ppb_per_code`'s sign convention, which is "positive
code increase → higher DO frequency" — that one is a hardware property
of the DAC↔EFC↔OCXO chain.  The two conventions live independently.

---

## 2. TICC reference offsets (chB slope)

Each host's TICC reads chB = F9T PPS (GNSS-locked) against its local
10 MHz reference.  chB slope = δ_ref (TICC ref vs GPS).

| Host    | TICC | Span | chB slope | TICC ref source (per topology.md) |
|---------|------|------|----------:|-----------------------------------|
| TimeHat | #1   | 16 h | **+0.36 ppb** | SV1AFN dist amp |
| MadHat  | #2   |  5 h | **-0.34 ppb** | SV1AFN dist amp |
| clkPoC3 | #5   |  3 h | **-1.01 ppb** | SV1AFN dist amp (per topology) |
| PiFace  | #3   | 69 h | **-0.04 ppb** | PiFace's own OCXO (self-referenced) |

**Anomaly**: topology.md says TimeHat #1, MadHat #2, lab bench #5 all
take 10 MHz from the SV1AFN dist amp.  If true, all three should
measure the same δ_ref.  Observed: +0.36, -0.34, -1.01 — a 1.4 ppb
spread.  Several possible causes (in order of plausibility):

  1. **Topology stale**.  Either the dist-amp chain has been
     restructured since the doc was last updated, or one or more TICCs
     run on a separate ref now.  Worth verifying physically.
  2. **Per-TICC internal delay-line drift**.  TAPR TICCs have coarse
     and fine timing chains internally; small ppb-level differences
     between two units are plausible at this measurement floor.
  3. **Distribution-amplifier port mismatch** — degraded gain or
     thermal coefficient differing between SV1AFN output ports.

For DO characterization the absolute TICC ref value cancels out in
chA-chB anyway, so we don't need to resolve this now — but it's a
flag for the next physical-rack inspection.

**PiFace's self-referenced TICC #3**: chB slope = -0.04 ppb means
PiFace OCXO is 0.04 ppb slow vs GPS, as measured by itself.  Sounds
circular but it isn't — we're using F9T PPS (GPS-locked) as the
truth signal, OCXO as the reference clock the measurement chain
uses to count ticks.  The slope is the OCXO frequency offset from
GPS.  This is a direct DO-vs-GPS measurement, no further correction
needed.

---

## 3. Per-host DO characterization snapshot

| Host    | DO chain                              | Current adj      | DO vs GPS¹    | State file last        |
|---------|---------------------------------------|------------------|---------------|------------------------|
| **PiFace** | CTI OCXO + AD5693R 16-bit + TADD-2 Mini | **+140.5 ppb** | **-0.05 ppb** | +140.2 (2026-05-13) ✓  |
| **TimeHat** | i226 PHC adjfine (NIC TCXO)        | **+39 ppb**    | **+0.33 ppb** | +30.8 (Apr 16, stale)   |
| **clkPoC3** | IsoTemp OCXO + AD5693R 16-bit + ? divider | **+788 ppb (was rail +1183)** | **-0.99 ppb²** | -17.3 (2026-05-12) ⚠ |
| **MadHat**  | i226 PHC adjfine (NIC TCXO)        | **+609 ppb**   | (chA broken)  | +152.2 (Apr 16, stale)  |

¹ chB slope − chA slope, longest smooth segment available
² Includes the rail-pinning period; not steady-state representative

### PiFace — excellent

The CTI OCXO + AD5693R chain on PiFace is operating exactly as
characterized.  Current adj = +140.5 ppb matches the 2026-05-13
state-file value of +140.2 ppb (sub-ppb drift over 2 days).
Residual after correction = -0.05 ppb, well within servo expectations.

The TOML comment block records the canonical PiFace characterization:
- ppb_per_code = 0.0361 (effective; gain factor 1.062 baked in)
- Linearity < 5 ppb over ±200 ppb of adj range
- Free-running at DAC center → -149 ppb (so center_code maps to roughly
  the natural OCXO offset, and the +140 ppb adj brings the DO right
  to GPS — small residual ~1 ppb apart from gain quirks)
- Sign verified 2026-04-14
- Pull range computed from 2^15 codes × 0.0361 ppb/code = **±1183 ppb**

### clkPoC3 — suspect

clkPoC3 uses **`dac_ppb_per_code = 0.0361`** — the SAME VALUE as PiFace,
with the comment block copy-pasted verbatim from PiFace's TOML.
**clkPoC3's OCXO is an IsoTemp, not a CTI** — different model entirely,
almost certainly different EFC gain.  This is the live form of the
`dacPpbSignClean-main` / I-010246 hazard.

Behavioral evidence supporting "ppb_per_code is wrong for IsoTemp":
- adj sat at the +1183 ppb rail for substantial stretches over the
  3-day run (per the engine-log adj timeline).  If 0.0361 were correct,
  the engine should hit equilibrium at ~150-200 ppb (typical OCXO
  offset).
- DO vs GPS = -0.99 ppb in the 3.3 h pre-restart segment we measured.
  At adj = -17.3 ppb (the stale state-file value), the OCXO should sit
  at ~-17 ppb + 0 = -17 ppb if ppb_per_code were correct.  Observed
  much smaller, which is consistent with — wait, this actually argues
  the OPPOSITE.  Hmm.

Actually the picture is muddier than a simple "ppb_per_code wrong":
- The state-file adj = -17.3 ppb implies the IsoTemp's natural offset
  is +17.3 ppb (slightly fast) and the servo pulls it down 17 ppb.
- But the live trace shows adj at +1183 ppb (rail!) — i.e., the engine
  is pulling +1183 ppb of correction.  That implies the OCXO's natural
  offset is -1183 ppb (very slow), not +17.3 ppb.
- A thermal transient + post-restart warm-start mismatch could explain
  some of this — clkPoC3 was rewired 2026-05-11 (F9T-BOT swap), so it
  may not have fully thermally settled, and its calibration is from
  2026-05-12.
- An incorrect ppb_per_code makes it worse — if the engine thinks it's
  commanding +1183 ppb but the chain delivers only some fraction of
  that, equilibrium might not exist within the actuator range.

The full diagnosis needs the actuator-sweep experiment in §5.

### TimeHat / MadHat — i226 PHC

These hosts have no separate DAC: the i226 NIC's TCXO is adjusted via
`clock_adjtime`, and adjfine_ppb is the literal frequency command.
Resolution = 1 ppb (kernel's adjfine units).  No ppb_per_code question
to resolve — the units are already physical.

The two NICs run with very different adjfine values:
- TimeHat i226: adj = +39 ppb (so NIC TCXO is naturally -39 ppb)
- MadHat  i226: adj = +609 ppb (so NIC TCXO is naturally -609 ppb)

Both are within the i226's pull range (≥10 ppm typically); both servos
have converged.  The NICs simply have different process-variation
offsets out of the factory, plus whatever thermal/aging accumulated
since each host's last warm-start.

MadHat's chA-channel is broken (residual std 1.6 ms over 5 h — not a
real PPS signal).  Per Bravo's diagnosis last session, this is a
downstream analog signal-path issue, not driver or PEROUT misconfig
or F10T.  Until that's fixed, we can't measure MadHat DO vs GPS.

---

## 4. What we still don't know

Per Bob's checklist, the gaps:

| Item | TimeHat | MadHat | clkPoC3 | PiFace |
|------|---------|--------|---------|--------|
| Pull range          | ✓ kernel-default (huge) | ✓ kernel-default | ✓ from TOML (±1183 ppb) **suspect if gain wrong** | ✓ ±1183 ppb |
| ppb_per_code        | N/A (1 ppb units)       | N/A              | **⚠ value lifted from PiFace, unverified**         | ✓ 0.0361 (verified) |
| Sign convention     | ✓ kernel               | ✓ kernel         | ⚠ assumed (+); not empirically verified           | ✓ verified 2026-04-14 |
| Offset at center    | -39 ppb (from current adj) | -609 ppb        | **⚠ unknown** (last_known says +17 but live behavior contradicts) | -149 ppb |
| Code for GPS agreement | adj = +39 ppb | adj = +609 ppb | **⚠ unknown** (live walks badly) | adj = +140 ppb |
| Noise ASD/PSD       | exists in state, stale  | exists, stale    | exists 2026-05-11                                  | exists 2026-05-11   |

---

## 5. Plan to close the gaps

Two classes of work:

### 5a.1 results — adjfine drift over 3 days

Computed 2026-05-15 from `[N] EKF: ... adj=...` log lines per host
(post-warm-up window, first 30 min excluded).

| Host    | n     | span | mean adj  | std    | trend (ppb/day) | range            |
|---------|------:|-----:|----------:|-------:|----------------:|------------------|
| PiFace  | 21602 | 70 h | +140.38   | **0.97** | +0.04          | [+137.4, +171.9] |
| TimeHat | 21317 | 70 h | +38.84    |   1.00 | +0.23          | [+31.2, +43.9]   |
| clkPoC3 | 21229 | 70 h | +1142.82  | **67.74** | +0.47          | [+401.5, +1182.9] |
| MadHat  | 21276 | 70 h | +471.69 (median +609.2) | 3527 | (artifact)¹ | [-45884, +44999]² |

¹ MadHat adj swings ±45000 ppb — engine reacting to broken chA
  measurements.  Numbers not physically meaningful.

² Bursts driven by junk-input feedback on the broken chA path.  See
  Bravo's diagnosis in `madhatPerout2ppsTicc-charlie`.

**Headline**: PiFace and TimeHat operate with **sub-1-ppb adj
steady-state noise** over 3 days — characteristic of well-tuned
servos on stable plants.  clkPoC3 runs with **68 ppb of adj std**
— 70× more wandering than PiFace, using the same code, same DAC
type, same `dac_ppb_per_code` value.  This is a clear empirical
signal that **clkPoC3's actuator chain is fundamentally
mis-configured** — the engine is fighting an effect the plant
model doesn't capture, most likely an incorrect ppb_per_code
scaling (the parameter cargo-culted from PiFace's TOML).

Aging rates of +0.04 / +0.23 / +0.47 ppb/day are all small and
consistent with weeks-to-months thermal/aging coefficients of
ovenized crystal oscillators.  None of these is causing the
clkPoC3 wandering.

### 5a. Things we can do from the existing 3-day data (no engine stop)

1. **Adjfine drift analysis** per host.  Time-series of adj over 3 days
   reveals aging / thermal patterns.  Already have the data extracted
   (`/tmp/day0515-adj/<host>.txt`).  Cheap to do.
2. **Noise ASD/PSD from chA-chB diff** per host on the long PiFace +
   TimeHat segments (69 h, 16 h).  Compare to the 2026-05-11 state-file
   ASD curves — confirms whether DO noise spectrum has changed.
3. **Cross-host PiFace-vs-clkPoC3 differential** from TICC#4 data
   (`ticc4-pifaceA-clkpoc3B-2026-05-XX.csv`).  Gives a direct
   DO-vs-DO comparison that's the moonshot success metric, AND
   doubly-verifies clkPoC3's DO offset since PiFace is the trusted
   reference.

### 5b. Things requiring an engine stop + actuator sweep

For **clkPoC3** specifically (the only known-suspect host):

1. Stop engine cleanly: graceful exit, save state.
2. Replace `dac_actuator.adjust_frequency_ppb()` calls with a controlled
   sweep script.  Step DAC code through e.g. [center − 16384,
   center − 8192, center − 4096, center − 2048, center − 1024,
   center − 512, center − 256, center, +256, +512, +1024, +2048, +4096,
   +8192, +16384] — sparse log-spaced to cover ±50% range without
   spending lab time on dense sweep.  Hold each code for 60 s.
3. At each code: capture 60 s of TICC chA-chB data, compute
   (chB - chA) slope = DO vs GPS at that code.
4. Plot DO_freq vs code.  Slope = actual ppb_per_code.  Intercept at
   center = OCXO free-running offset.  Sign of slope = sign convention.
5. Update `config/clkpoc3.toml` with correct values.
6. Restart engine; verify equilibrium adj matches expectation.

For **TimeHat + MadHat** (i226 hosts) — much simpler:
- adjfine IS the actuator value; no ppb_per_code to characterize.
- Sweep via `clock_adjtime` with PHC_FREQ to e.g. ±5000 ppb in 1000-ppb
  steps; measure resulting DO vs GPS at each.  Verify response is
  linear and slope = 1.0 ppb-out per ppb-in.  This is the i226
  **linearity** check, not a calibration.

For **PiFace** — well-characterized; can re-verify ppb_per_code with a
shorter sweep (±2000 ppb in 500-ppb steps over 10 min) as a
confidence-check before relying on it for the moonshot sync experiment.

### 5c. Other things worth measuring (Bob asked "anything else?")

1. **Temperature coefficient** — DO frequency vs ambient/oven temp.
   Requires a temp sensor per DO; not currently deployed everywhere.
   Defer until I/O is available.
2. **Aging rate** — long-term frequency drift, ppb/day.  Can extract
   from the 3-day adjfine timeseries already collected.  Quick to do.
3. **DAC linearity over full range** — does ppb_per_code stay constant
   from code=0 to code=65535, or does it bend near the rails?  The
   adjfine ≈ rail behavior on clkPoC3 suggests this may matter.  Tested
   incidentally in the 5b sweep if we use sparse-log spacing.
4. **Settling time** — how long after a step does DO frequency reach
   the new commanded value?  Useful for servo design.  Need a step
   experiment, which the sweep gives us for free at each code step.
5. **DAC hysteresis** — does ppb_per_code differ when commanding code
   up vs down?  Run the sweep both directions; compare.

---

## 6. Immediate proposed action

1. **5a.1 + 5a.2 (adjfine drift + noise ASD/PSD)** — I can do these
   right now from existing data, no engine impact.
2. **5b for clkPoC3** — requires Bob's go-ahead to stop clkPoC3
   engine.  Other three hosts can keep running.  Total lab time
   for the sweep: ~30-45 min (15 codes × 60 s = 15 min capture +
   bracket time + analysis).
3. **5c.2 (aging rate)** — falls out of 5a.1 for free.

Defer pull-range / linearity for TimeHat + MadHat until Bob can
stop one of those engines.  Lower priority — they're operationally
fine as-is.

---

## References

- `config/piface.toml` — canonical DAC characterization template
- `config/clkpoc3.toml` — comments lifted from PiFace; ppb_per_code suspect
- `state/dos/ocxo-piface.json` — 2026-05-11 PSD characterization, gold reference
- `state/dos/ocxo-clkpoc3-v2.json` — 2026-05-11 characterization
- `scripts/peppar_fix/dac_actuator.py` — actuator code + sign rules
- `docs/two-site-sync-budget.md` — why this characterization matters for the moonshot
- Dayplan item: `dacPpbSignClean-main` — the rename/cleanup precursor
- Memory: `dacWarmStart-main thermal-warmup blind-spot` — relevant to clkPoC3 rail behavior

---

*Drafted 2026-05-15 (main) under Bob's "limited attention" mode.*
