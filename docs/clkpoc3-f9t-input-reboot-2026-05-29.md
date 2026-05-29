# clkPoC3 "F9T input integrity" re-bootstraps — root cause 2026-05-29

Dayplan item `clkpoc3F9tInputReboot` (owner charlie).  Overnight run
`day0528-xhost-clkpoc3` (no-gate / main) re-bootstrapped **12×** from
`30 consecutive catastrophic rejects … Exiting for re-bootstrap (exit
code 5)`, cadence ~1.5–2.3 h.

**Verdict: not receiver-side.**  The F9T input is healthy.  The trigger
is a ~32 s **engine observation-pipeline stall** every ~1.5 h; the
exit-5 cascade is an **engine gap-handling bug**, not lost F9T
integrity.  The dayplan's "F9T input loses integrity / RECEIVER-SIDE"
framing is wrong and was sending the investigation at the receiver.

## Evidence

Source: `~/peppar-fix/data/day0528-xhost-clkpoc3.log` on clkPoC3 (676 MB,
append-on-respawn) + `…-filter-state.csv`.

**1. The F9T clock is smooth across every burst — no jump, no clkReset.**
`[NAV-CLOCK]` logs clkB climbing steadily at clkD = **260 ns/s**,
tAcc = 2 ns, with no discontinuity at the burst onset.  `[CLK_REALIGN]`
fired **0 times** all night.  So the receiver clock did not glitch.

**2. Every burst (11 of 12) is preceded by a 31–39 s observation gap.**
`WARNING Gap dt=NN.Ns, resetting filter time (not skipping)` lands in
the same second as the first `[CATASTROPHIC_REJECT]`.  All 11 gaps map
1:1 onto burst onsets:

| burst onset (local) | gap dt | clkD·(gap−1s) predicted | measured median \|PR\| |
|---|---|---|---|
| 19:56:14 | 32 s | 2416 m | 2415 m |
| 21:41:32 | 35 s | 2650 m | 2609 m |
| 23:56:29 | 39 s | 2962 m | 2720 m |
| 01:53:13 | 32 s | 2416 m | 2158 m |
| 04:13:36 | 31 s | 2338 m | 2037 m |
| 05:57:32 | 32 s | 2416 m | 2062 m |
| 07:57:00 | 36 s | 2728 m | 2228 m |
| 09:46:08 | 31 s | 2338 m | 2207 m |
| 11:24:31 | 32 s | 2416 m | 2523 m |
| 12:56:33 | 32 s | 2416 m | 2568 m |
| 14:20:24 | 31 s | 2338 m | 2676 m |

The residual **is** the receiver-clock drift accumulated over the gap.
Exact for burst 1: clkD·(gap−1s) = 260 ns/s · 31 s = 8060 ns = **2416 m**
vs measured 2415 m.

**3. The residual is common-mode.**  The catastrophic gate trips on
`median |PR|` over ~17 SVs.  For the median to reach 2415 m, the
majority of SVs must share that residual — a clock-like offset, not
per-SV slips or multipath.

**4. The stall is process-wide.**  Log output drops from ~110 lines/5 s
to ~1 line/20 s during the gap; UBX iTOW then advances in catch-up
bursts (+5 s/+10 s/+15 s per processed message while wall-clock advances
+14 s/+7 s/+14 s).  The receiver kept emitting at 1 Hz; the engine's
**main loop blocked** ~14–32 s.  clkPoC3 was not CPU-saturated
(load 1.84/4, engine 38 %).

## Mechanism

```
~32 s main-loop stall  →  obs gap (receiver clock drifts ~8 µs un-tracked)
   │
   ▼  engine gap handler (peppar_fix_engine.py:4844)
 if dt > 30:  filt.predict(1.0)        # clamps clock advance to 1 s, NOT 32 s
   │                                    # → filter clock now ~8 µs behind truth
   ▼  filt.update(...)  pre-fit median |PR| ≈ clkD·(gap−1s) ≈ 2.4 km
   ▼  catastrophic gate: median |PR| > 20×5 m baseline  →  REJECT, state unchanged
   │                                    # the intended "re-anchor from PR" never runs
   ▼  every following epoch still ~2.4 km off (state frozen)  →  30 rejects
   ▼  exit code 5  →  wrapper relaunch + re-bootstrap  →  recovers (until next gap)
```

Two engine defects interact:

- **Gap-handler under-propagation** (`peppar_fix_engine.py:4849-4850`):
  `filt.predict(1.0)` advances the clock state by 1 s of drift, leaving
  the filter ~clkD·(gap−1s) ≈ 8 µs behind the true receiver clock.  The
  comment intends the *update* to re-anchor from pseudoranges — but:
- **The catastrophic gate blocks the re-anchor** (`solve_ppp.py:1685-1731`):
  the 2.4 km common-mode residual reads as a catastrophic PR blowup, so
  the epoch is rejected (`state unchanged`) and the re-anchor never
  happens.  The counter climbs to 30 → exit-5.

The clkReset / `[CLK_REALIGN]` absorb-path (`solve_ppp.py:1667-1678`)
already does the right thing for *flagged* integer-ms receiver
realignments — but a gap-induced drift carries no clkReset flag, so it
hits the gate instead.

## The one outlier: burst 5 (02:38:39, 21.000 ms)

Burst 5 is **not** preceded by a >10 s gap and its magnitude is a clean
21.000 ms (6,295,642 m), 4 orders larger than the gap-drift bursts.
This is a distinct, rarer event (a genuine ~21 ms receiver/time event —
cf. the 2026-05-12 F10T 21 ms slip).  Out of scope for the gap fix;
flagged for separate follow-up.

## Fixes

**Engine-side (the real fix — makes the loop robust to gaps).**  On a
gap > 30 s, absorb the receiver-clock drift instead of letting the gate
cascade.  Cleanest is to reuse the existing CLK_REALIGN path: on the
first post-gap epoch, re-seed `dt_rx` from the median PR and reset
`_consecutive_catastrophic_rejects` (exactly as the clkReset branch does
at `solve_ppp.py:1667-1678`), bypassing the catastrophic gate for that
one epoch.  Equivalent alternative: inflate `P[clk,clk]` by
`(clkD·dt)²` for the post-gap update so the gate tolerance covers the
gap drift and the update re-anchors.  Either turns a 30-reject exit-5
into a single clean re-anchor — no re-bootstrap, no fragmented long-tau
data.  **Needs lab validation on clkPoC3** (engine-core change).

**Stall-side (remove the trigger).**  Find what blocks the main loop
~32 s every ~1.5 h.  Prime suspect: SD/eMMC write stalls under the
**verbose `[NAV-SIG]` logging** (~50 lines/epoch into a 676 MB ext4
file on an 83 %-full card).  Cheapest test: drop NAV-SIG to a periodic
sample (or off) and confirm the gaps disappear.  If gaps persist, attach
`py-spy dump` to the engine during a live gap to catch the blocking
call (candidates: log/CSV fsync, a blocking NTRIP/SSR socket read).

## Why it matters

Each event costs a 30-epoch holdover + a full re-bootstrap, fragmenting
clkPoC3's long-tau record — which is why clkPoC3 is currently excluded
from the moonshot crossover analysis.  The engine-side fix alone
restores continuous long-tau data even if the stalls remain; fixing the
stalls additionally recovers the ~32 s of lost data per event.
