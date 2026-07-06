# PePPAR Fix

> **Precise time for a GNSS antenna that doesn't move.**

PePPAR Fix turns a stationary GNSS antenna into a precision clock. If your antenna
isn't going anywhere, you can survey its position **once** and then spend the *entire*
GNSS solution on the one thing you actually want out of it: **time**. PePPAR Fix uses
carrier-phase Precise Point Positioning to measure a local oscillator's error against
GPS time and gently steers that oscillator to track it — without throwing away the
oscillator's own excellent short-term stability.

The name is a play on **PPP-AR** — *Precise Point Positioning with Ambiguity
Resolution*, the geodetic technique at its core. (The "arr" is also for pirates.)
"Fix" is the GNSS fix it stands on.

---

## Why a stationary antenna changes everything

A roving GNSS receiver has to solve for *where it is* **and** *what time it is*,
epoch after epoch. Position and clock trade off against each other, and the clock is
never better than the position solution allows.

Hold the antenna still and that tradeoff disappears. Survey the antenna reference
point once — to the centimeter — and pin it. Now every carrier-phase measurement the
receiver makes, normally spent chasing position, becomes a measurement of the
**receiver's clock** instead. Sub-centimeter position knowledge converts directly
into sub-nanosecond time knowledge. A GNSS antenna that doesn't move turns out to be
one of the best time references you can build.

```
  stationary GNSS antenna
        │   raw carrier phase  +  real-time SSR corrections (NTRIP)
        ▼
  ┌────────────────────────────────────────────────┐
  │  peppar-fix : fixed-position PPP(-AR) filter     │ ◀── surveyed position
  │              + multi-arm Kalman / LQR servo      │     (peppar-survey, once)
  └───────────────────────────┬──────────────────────┘
                              │   frequency / phase steer
                              ▼
  ┌────────────────────────────────────────────────┐
  │  disciplined oscillator  ──▶  PPS / 10 MHz out   │  locked to GPS time,
  │                                                  │  short-term stability kept
  └────────────────────────────────────────────────┘
```

---

## Two tools

**`peppar-fix`** — the real-time discipline engine. It ingests raw GNSS observations
plus a real-time correction stream, runs a fixed-position PPP(-AR) filter to estimate
the local clock continuously (1–10 Hz), and steers a **disciplined oscillator** (DO)
to hold it to GPS time. It fuses several independent error sources — PPS edges, the
receiver's fine time-pulse quantization error (qErr), carrier-phase `dt_rx`, and an
external time-interval counter — through a Kalman/LQR servo.

**`peppar-survey`** — the companion that establishes the ground truth `peppar-fix`
stands on. It determines the authoritative antenna reference point (ARP) offline from
a surveying-grade backend (PRIDE PPP-AR), to the centimeter. In the recommended
time-only mode this survey is load-bearing: the engine pins the position from it and
never estimates position at runtime, so all of its estimation power goes to the clock.

---

## How it works

**Time-only architecture.** With the position pinned from a survey, the engine skips
its own position filter entirely and runs a fixed-position clock solution. Everything
the receiver observes becomes a constraint on time.

**Carrier phase, not just PPS.** A 1 PPS edge from a timing receiver is quantized to a
few nanoseconds. The carrier phase is not — a fixed-position PPP filter can track the
receiver clock to a few *picoseconds* per epoch. PePPAR Fix treats carrier-phase
`dt_rx` as a first-class servo input and uses PPS/qErr as complementary arms.

**A two-oscillator budget.** The output can only ever be as good as the *better* of:

1. **The disciplined oscillator (DO)** — the crystal being steered. The servo can
   move its frequency but cannot erase its phase noise; below the loop bandwidth, the
   DO's own free-running floor is what you get.
2. **The GNSS receiver's oscillator (RX TCXO)** — every carrier-phase observation
   carries the receiver's clock noise, so any GNSS-derived correction inherits that
   floor.

The whole design is organized around respecting both floors and adding nothing of its
own in between.

---

## What we're reaching for

**The moonshot.** At *every* averaging time τ, the disciplined output should be as
stable as the *best* of (the DO's free-running floor, the receiver oscillator's
floor). At short τ the quieter oscillator shines through untouched; at long τ the
output tracks GPS time as faithfully as the receiver allows; and the discipline loop
guides the handoff so gently that it adds no noise, overshoot, or loop-bandwidth
artifact of its own. Beating a bare PPS reference is *not* the goal — that's limited
by receiver resolution, not by what the oscillators can actually deliver.

**Two clocks that agree.** Any two PePPAR Fix clocks should produce PPS edges that
agree in phase within a probability-bounded excursion:

| Configuration | Bound | Probability | Window |
|---|---|---|---|
| Shared antenna (same RF via splitter) | \|Δ\| ≤ 1 ns | 95% | any τ from 0.1 s to ~1000 s |
| Separate antennas, baseline ≤ 5 km | \|Δ\| ≤ 2 ns | 95% | any 1-hour window |

**Clocks that bootstrap each other.** A longer-term ambition is peer discipline —
PePPAR Fix nodes advertising and serving corrections to one another over the local
network, so a fresh clock can lean on a settled neighbor.

---

## Where we are

An honest snapshot — this is an active research system, not a finished product:

- ✅ **PPP-AR works** — GPS + Galileo ambiguity resolution, with sub-centimeter
  cross-host *position* agreement on a shared antenna.
- ✅ **Time-only architecture** — survey-pinned ARP + fixed-position clock solution,
  validated against the full position-filter baseline.
- ✅ **Carrier-phase discipline** — a time-differenced carrier-phase servo arm
  reaching ~15 ps TDEV(1 s) in prototype, well below what PPS can resolve.
- 🚧 **Cross-host agreement** — currently ~10–15 ns (95th percentile) between two
  settled clocks: roughly 10× from the 1 ns shared-antenna moonshot, and the present
  frontier.
- 🚧 **Robustness** — ongoing work on cross-stream measurement correlation,
  self-recovering fault handling, and honest confidence reporting.

---

## Supported hardware

PePPAR Fix runs on commodity Linux single-board computers and off-the-shelf timing
hardware.

| Role | Supported |
|---|---|
| **GNSS receiver** | u-blox ZED-F9T (timing, dual-freq L1/L5), ZED-F9P, ZED-X20P, NEO-F10T; Septentrio Mosaic-T (SBF) |
| **Disciplined oscillator / actuator** | Any Linux PTP Hardware Clock steered via `adjfine` (e.g. an Intel i226 NIC's TCXO); DAC-steered OCXOs (16/18-bit); external frequency-control-word oscillators. A generic actuator seam lets new DO hardware drop in. |
| **Measurement / grading** | TAPR TICC (~60 ps time-interval counter); PHC `EXTTS` PPS timestamping; on-chip TDCs |
| **Corrections** | Real-time SSR over NTRIP (IGS Real-Time Service and mirrors) + broadcast ephemeris |
| **Compute** | Raspberry Pi 4/5 or x86-64 Linux |

The oscillator you steer sets the ceiling. OCXO-class parts (ADEV(1 s) ≲ 1×10⁻¹¹) can
reach the sub-nanosecond cross-host target; TCXO-class parts are supported and useful
for exercising the measurement chain, but their free-running noise above the loop
bandwidth puts the moonshot out of reach.

> **Grade with a real counter.** Servo output should be graded with a time-interval
> counter (the TICC, ~60 ps). On-chip `EXTTS` timestamps are convenient but quantize
> at the receiver's clock period (~8 ns), which flatters short-τ results — fine for
> tracking, misleading for grading.

---

## Getting started

`peppar-fix` is driven by an orchestration wrapper; you normally don't invoke the
component scripts directly.

```sh
# One-time, per host:
python3 -m venv venv
venv/bin/pip install -e '.[analysis]'      # engine + analysis tools

# You need two things before a time run:
#   1. A surveyed antenna position   (peppar-survey, or --known-pos)
#   2. A real-time correction stream  (NTRIP SSR — see ntrip.conf.example)

# Discipline a PHC-based DO at a surveyed position, time-only:
scripts/peppar-fix --no-antposest --servo /dev/ptp0 --pps-pin 1
```

Real-time PPP needs SSR corrections; register for the free
[IGS Real-Time Service](https://igs.org/rts/access/) and put your credentials in
`ntrip.conf` (see `ntrip.conf.example`).

Design notes and research write-ups live in [`docs/`](docs/); contributor operating
knowledge is in [`CLAUDE.md`](CLAUDE.md).

---

## Repository layout

```
scripts/
  peppar-fix              orchestration wrapper — start here
  peppar_fix_engine.py    real-time PPP + servo engine
  peppar_fix/             engine internals: filter, servo, correlation, actuators
  ...                     analysis, capture, and configuration tools
docs/                     design notes and research write-ups
CLAUDE.md                 operating manual for contributors
```

---

## Status & license

PePPAR Fix is a research project, developed against real hardware in a working timing
lab. Interfaces change, targets are aspirational, and results are reported honestly —
including the gap between where we are and the moonshot. It is **not** a product and
carries no fitness or stability guarantees.

MIT-licensed. © Bob Van Valzah.
