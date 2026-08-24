# Two-clock agreement — the forward model

**Status**: design tool + reference.  Drafted 2026-08-24 (main) in response to
Bob's question: *"I want to get an intuitive feel for the parameters that go
into two-clock agreement.  Is this just another way of looking at the budget?"*

**Answer: yes — it is the same budget, solved for the other variable.**
[`two-site-sync-budget.md`](two-site-sync-budget.md) runs **backwards**: start
from the 1 ns / 2 ns excursion targets, allocate a per-term budget, derive the
hardware.  This doc runs the same math **forwards**: start from a parts list
and a servo cadence, predict p95 |Δ|.  Same equation.  The forward direction is
what you want while speccing hardware; the backward direction is what you want
while grading a run.

Tool: [`tools/sync_agreement_model.py`](../tools/sync_agreement_model.py).

> **Expect this doc to change.**  The model is deliberately simple and its
> weakest input (σ_ref) is the one that dominates today's measurements.  It
> will be revised as v1 hardware produces real numbers.

---

## 1. The reframe — three bands, not four terms

The budget doc decomposes σ_clock into four additive terms.  That is correct,
but it hides the structure.  The useful mental model is **three bands separated
by the servo's loop time constant τ_c**:

| band | what lands on your PPS OUT | scaling |
|---|---|---|
| τ < τ_c | DO free-running noise the loop is too slow to remove | **grows** with τ_c |
| τ ≈ τ_c | measurement noise the loop *injects* onto the DO | **shrinks** as 1/√(f_s·τ_c) |
| τ > τ_c | reference-side error the loop faithfully **tracks** | independent of τ_c; grows with eval window |

```
σ_clock(W)² = [ADEV(τ_c)·τ_c]²  +  σ_z²/(f_s·τ_c)  +  σ_ref(W)²  +  σ_act²  +  bias²
              ────────┬────────     ──────┬──────     ────┬────     ───┬───     ──┬──
                  band 1              band 2           band 3      DAC LSB   uncalibrated
              DO above loop BW    observer noise    what the loop  ÷ √12 ×    per-unit
                                  averaged down     is chasing      T_act     offset

σ_Δ = √2 · σ_clock          p95 |Δ| ≈ 1.96 · σ_Δ
```

**Bands 1 and 2 fight.**  Slower loop → less injected measurement noise, more
free-running DO noise.  Their crossover *is* the optimum loop bandwidth, and it
is the single most important design number.  This is the same crossover
described qualitatively in
[`gpsdo-noise-and-external-clock.md`](gpsdo-noise-and-external-clock.md), made
numeric.

**Band 3 ignores the loop entirely.**  Below f_c the loop tracks its reference
faithfully — including the reference's errors.  You cannot tune your way out of
band 3; you can only get a better reference.  Empirically this is the term that
currently dominates the lab (§5).

---

## 2. Do we need a simulator?

**No, for the design surface.**  Five closed-form terms and a 1-D minimization
over τ_c.  `tools/sync_agreement_model.py` is ~250 lines and answers every
parameter-sweep question below in milliseconds.

**Yes, for four things the math cannot reach:**

1. **Tails past p95.**  The 1.96 factor is honest for p95 and useless past
   ~p99.  Historic worst excursions (TimeHat 1.85 µs, PiFace 2.3 µs) were not
   on any Gaussian tail — they were chi²-gate lockouts.
2. **Quantizer dead zones and limit cycles.**  Nonlinear, and the failure mode
   is a *bias*, not a variance (§4.1).
3. **Outlier-gate / coast logic.**  `two-site-sync-budget.md` §3.2.1 is the
   record of this being the binding term: 88 ps predicted, 3500 ps measured.
4. **Transients** — rooftop thermal steps, SSR outage and re-convergence,
   reboot.  A p95 taken over a day includes these.

Use [`scripts/peppar_fix/servo_sim.py`](../scripts/peppar_fix/servo_sim.py) for
those.  The two are complementary: the closed form sizes the hardware, the
simulator validates the loop logic that the closed form assumes is perfect.

---

## 3. The design surface

All figures at 1 Hz measurement, 16-bit DAC, **perfect reference** (σ_ref = 0),
so what is shown is the *measurement-chain and oscillator floor*.

```
$ python3 tools/sync_agreement_model.py matrix
```

| DO class | EXTINT/EXTTS 8 ns | TICC 60 ps | TDC7200 55 ps |
|---|---|---|---|
| TCXO i226 internal | 1.88 ns @ τ_c 23 s | 0.29 ns @ 1 s | 0.28 ns @ 1 s |
| MEMS SiT5358 | 1.33 ns @ 46 s | 0.15 ns @ 1 s | 0.15 ns @ 1 s |
| Hobby OCXO (CTI/IsoTemp) | **0.96 ns @ 67 s** | 0.07 ns @ 2 s | 0.07 ns @ 2 s |
| Good OCXO | 0.67 ns @ 139 s | 0.06 ns @ 6 s | 0.06 ns @ 5 s |
| Premium OCXO (OX-249) | 0.31 ns @ 647 s | 0.05 ns @ 25 s | 0.05 ns @ 23 s |

Three readings worth internalizing:

- **A better DO barely helps behind an 8 ns timestamper.**  50× better
  oscillator (hobby OCXO → OX-249) buys 3×, because at τ_c ≈ 67 s the
  measurement term dominates.  Behind a TICC, the same upgrade buys nothing
  at all — the actuator floor takes over.
- **The optimum τ_c moves with the DO.**  Better oscillator → you can afford a
  slower loop → you average the observer harder.  A fixed loop bandwidth is
  wrong for a fleet of mixed DO classes.
- **The bowl is shallow.**  For hobby OCXO + EXTINT, anything from τ_c ≈ 30 s
  to 150 s stays under 1.3 ns.  You do not have to nail the tuning.

---

## 4. Worked case — the 8 ns EXTINT question

Can a design that uses only the F9T's EXTINT pin (no TICC, no external TDC)
meet a 1 ns p95?

8 ns quantization ⇒ σ = 8/√12 = **2.31 ns per epoch**, white.  The loop averages
it as 1/√(f_s·τ_c).  Against a hobby OCXO (ADEV(1 s) = 1e-11, flicker floor
3e-12):

```
$ python3 tools/sync_agreement_model.py bands --do ocxo-hobby --ts extint-f9t
```

| τ_c | σ_DO | σ_meas | p95 \|Δ\| |
|---|---|---|---|
| 10 s | 32 ps | 730 ps | 2.03 ns |
| 30 s | 90 ps | 422 ps | 1.20 ns |
| **67 s** | **201 ps** | **282 ps** | **0.96 ns** ← optimum |
| 100 s | 300 ps | 231 ps | 1.05 ns |
| 300 s | 900 ps | 133 ps | 2.52 ns |
| 1000 s | 3000 ps | 73 ps | 8.32 ns |

**EXTINT-only, 1 Hz, hobby OCXO ⇒ p95 ≈ 1 ns at τ_c ≈ 60–100 s.**

Three independent routes agree on this number, which is why it is worth
trusting more than a single closed form usually deserves:

1. This model: 0.96 ns.
2. The 1-D closed-loop EKF scoping in
   [`doqerr-extint-tick-model.md`](doqerr-extint-tick-model.md) — x₂ RMSE
   0.361 ns with the raw quantized arm ⇒ 1.96·√2·0.361 = **1.00 ns**.
3. `two-site-sync-budget.md` §4.2's 354 ps/clock allocation — this model's
   σ_clock at the optimum is **347 ps**.

Raising the measurement rate is the cheapest lever, and needs no new hardware —
just TD-CP at full RAWX rate:

| f_s | optimum τ_c | p95 \|Δ\| |
|---|---|---|
| 1 Hz | 67 s | 0.96 ns |
| 5 Hz | 39 s | 0.56 ns |
| 10 Hz | 31 s | 0.45 ns |
| 20 Hz | 25 s | 0.35 ns |

### 4.1 The dither caveat — when 8 ns stops averaging down

σ_q/√N assumes the quantizer **dithers**.  On EXTINT it does, for a specific
reason: 1 s is an exact integer multiple of 8 ns (1 s = 125·10⁶ × 8 ns), so
whole seconds never walk the DO edge across the grid.  The *only* thing that
walks it is the rx TCXO's own wander — `z = x₂ − qerr(x₂ + x₀)` per the tick
model, with x₀ the rx TCXO phase.  x₀ wanders freely ⇒ good dither ⇒ white ⇒
averages down.

**Kill that wander and the model collapses.**  If the receiver is ever clocked
*from the DO* — which is exactly the external-reference-receiver architecture
recommended in
[`gpsdo-noise-and-external-clock.md`](gpsdo-noise-and-external-clock.md) and
[`receiver-clock-hierarchy.md`](receiver-clock-hierarchy.md) — the grid becomes
coherent with the signal, dither stops, and the 8 ns quantization becomes a
**standing bias up to ±4 ns that no averaging removes**.

We already have the empirical signature: **E810 EXTTS shows 77 % identical
adjacent timestamps; i226 shows 0 %** (CLAUDE.md, "EXTTS TDEV measurements are
unreliable").  *Fraction of identical adjacent timestamps* is the dither health
check — near zero is healthy, high means you are in a dead zone.  The other
tell is the qErr sawtooth period: a fast sawtooth means fast dither.

**Design consequence.**  8 ns EXTINT is adequate for a v1 clock whose receiver
keeps its own oscillator.  It stops being adequate the moment the receiver runs
on the DO.  Plan a real TDC — or deliberate dither — for that architecture.
Model that case with `--bias`, not `--ts extint-f9t`.

---

## 5. σ_ref — the term that actually binds, and that the math cannot predict

```
$ python3 tools/sync_agreement_model.py ref --do ocxo-hobby
```

| σ_ref (per clock, over eval window) | EXTINT 8 ns | TICC 60 ps |
|---|---|---|
| 0 | 0.96 ns | 0.07 ns |
| 0.10 ns | 1.00 ns | 0.29 ns |
| 0.25 ns | 1.19 ns | 0.70 ns |
| 0.50 ns | 1.69 ns | 1.39 ns |
| 1.0 ns | 2.93 ns | 2.77 ns |
| **2.9 ns** | **8.0 ns** | **8.0 ns** |

**Above ~0.5 ns of reference-side error, the choice of timestamper stops
mattering** — the columns converge.  That is the quantitative explanation for
why the TICC-vs-EXTINT A/B in
[`ticc-vs-extint-do-observer-experiment.md`](ticc-vs-extint-do-observer-experiment.md)
kept coming out ambiguous: both arms were downstream of a σ_ref that swamped
the difference.

That last row is **back-fit from measured data** — PiPuss ↔ PiFace, shared UFO1
antenna, 2026-07-01, p95 |Δ| = 8.04 ns:

```
$ python3 tools/sync_agreement_model.py backfit --p95 8.04 --do ocxo-hobby --ts ticc
  measured p95 |d| = 8.04 ns  ->  sigma_clock = 2901 ps per clock
  => implied independent sigma_ref = 2.90 ns per clock
  sigma_ref dominates: a better timestamper or a better DO will not help.
```

The 2026-07-24 analysis named the mechanism: **the servo steering the DO to
chase the per-receiver rx TCXO through the PPP clock arm.**  PiFace 6 % slewing
vs PiPuss 47 % on the *same antenna and the same corrections*, with
corr(PiFace slew, PiPuss slew) = +0.31 — per-unit, does not common-mode, lands
squarely in σ_ref.

**σ_ref is an input to this model, never an output.**  Nothing derives it from
datasheets.  `backfit` is how you get it, and it must be measured per receiver.

### 5.1 p95 depends on the evaluation window — always name it

σ_ref grows with the window because the underlying process is random-walk-ish.
The same 2026-07-01 run makes this vivid: **10-minute detrended σ was ~3 ns,
but the 2.5-hour CDF gave p95 = 8 ns**, because the offset crept +0.7 → +21 ns
over the run.  Low-frequency drift, not second-to-second jitter, busted the
budget.

CLAUDE.md's moonshot spec already names its window ("any 1-hour window").  Any
v1 spec must too, or the number is meaningless.

---

## 6. The other parameters, ranked by leverage

1. **σ_ref** — rx TCXO chase + correction-stream error + multipath.  Dominant
   today.  Levers: better rx oscillator, AR vs float, loop BW low enough not to
   chase it, external-reference receiver (§4.1's caveat applies).
2. **Measurement rate**, not just resolution.  1 → 10 Hz is worth more than
   8 ns → 60 ps once σ_ref > 0.5 ns, and costs nothing.
3. **Evaluation window** (§5.1).
4. **Constant per-unit bias.**  Enters undiminished — 0.5 ns of uncalibrated
   bias alone costs 1.39 ns p95 on an otherwise-perfect pair.  Cable, connector,
   PHY, antenna group delay, TICC/EXTINT offsets.  Per-unit calibration is a v1
   requirement, not a nicety.  `two-site-sync-budget.md` §4.3 already lists
   uncalibrated hosts as a live risk.
5. **PPS OUT edge placement.**  See
   [`gnssdo-plus-pps-alignment.md`](gnssdo-plus-pps-alignment.md) for the
   retiming case (0–100 ns per-box bias + metastability boundary steps).  Not
   an issue for a divider driven by the steered DO itself — see §7.
6. **Thermal.**  Every ADEV figure above assumes constant temperature.  A
   rooftop PoE enclosure is the worst environment in the project; slow diurnal
   drift is inside the loop bandwidth and tracked fine, but fast transients
   (cloud shadow, rain squall) are seen as frequency error.
7. **Actuator range and linearity**, not just LSB.  16-bit is 18 ps — a
   non-issue.  Running out of pull range (MadHat OCXO-33) is a real one.
8. **Outlier-gate architecture.**  Empirically the largest excursions by orders
   of magnitude, and absent from every datasheet.
9. **Antenna-to-receiver cable stability.**  Foam-PE coax ≈ 50 ppm/°C; 30 m
   through a 20 °C swing is ~150 ps of independent per-site drift.  **Putting
   the receiver at the antenna deletes this term** — a real argument for the
   rooftop-integrated design beyond packaging convenience.

---

## 7. PTP is not in this budget

For the v1 rooftop architecture — each unit independently GNSS-disciplined,
PTP as *delivery* — **PTP jitter does not bound two-clock agreement.**  The two
units agree via GPS.  PTP is a downstream leg that degrades what a *client*
sees.

```
unit A ──GNSS──► DO_A ──► PPS OUT ─┐
                    └──► PTP GM ──► network ──► client   (+ PTP error)
unit B ──GNSS──► DO_B ──► PPS OUT ─┘
                 ▲ agreement measured HERE is clock-limited
```

Rough delivery numbers over PoE:

| chain | static bias | jitter/wander p95 |
|---|---|---|
| PTP-unaware switches | uncalibrated | 1–100 µs, load-dependent |
| Transparent/boundary clocks, HW timestamping | 50–300 ns | 10–50 ns |
| Single hop, calibrated | 50–200 ns | 5–20 ns |
| White Rabbit | calibrated <1 ns | <100 ps |

The dominant PTP term at 1000BASE-T is not jitter — it is **PHY TX/RX latency
asymmetry**, typically 200–400 ns, appearing as a static offset of half that.
Stable and calibratable per port, but per-unit, so it does **not** cancel
between two units.  That is why a v1-over-PTP client pair cannot be graded
against a 1 ns target, and it is the main thing White Rabbit buys in v2:
calibrated asymmetry and a phase-tracked link, not merely better timestamps.

**Consequence — this is why each v1 unit needs PPS IN and PPS OUT.**  Clock
agreement and delivery quality are two different measurements with a 10–100×
gap between them; conflating them means chasing servo bugs that are actually
PHY asymmetry.  Budget and spec them separately.

Bonus: a *monitor* has no bandwidth requirement, so it can average as long as
it likes.  An 8 ns PPS-IN timestamper gives 2.31 ns single-shot but sub-100 ps
after a minute.  **Quantization hurts the servo far more than it hurts the
monitor** — you can measure agreement well below the level you can control it
to, with the same cheap part.

---

## 8. Tool reference

```sh
python3 tools/sync_agreement_model.py matrix                    # DO × timestamper
python3 tools/sync_agreement_model.py one --do ocxo-hobby --ts extint-f9t
python3 tools/sync_agreement_model.py bands --ts extint-f9t     # terms vs τ_c
python3 tools/sync_agreement_model.py ref --do ocxo-hobby       # σ_ref sensitivity
python3 tools/sync_agreement_model.py backfit --p95 8.04 --ts ticc
```

Common flags: `--do --ts --rate --dac --ref --white --bias`.
Component libraries (`DO_CLASSES`, `TIMESTAMPERS`, `DACS`) are dicts at the top
of the module — add parts there as they enter the lab.

---

## References

- [`two-site-sync-budget.md`](two-site-sync-budget.md) — the backward direction:
  targets → allocation → parts list.  **Read first.**
- [`doqerr-extint-tick-model.md`](doqerr-extint-tick-model.md) — the 8 ns EXTINT
  quantization mechanism and the 1-D EKF cross-check.
- [`gpsdo-noise-and-external-clock.md`](gpsdo-noise-and-external-clock.md) — the
  qualitative crossover this model makes numeric.
- [`ticc-vs-extint-do-observer-experiment.md`](ticc-vs-extint-do-observer-experiment.md)
  — the A/B whose ambiguity §5 explains.
- [`gnssdo-plus-pps-alignment.md`](gnssdo-plus-pps-alignment.md) — PPS OUT edge
  placement, the retiming case.
- [`receiver-clock-hierarchy.md`](receiver-clock-hierarchy.md) — why the
  external-reference receiver breaks §4.1's dither assumption.

---

*Drafted 2026-08-24 (main).  Expect revision as v1 hardware produces numbers.*
