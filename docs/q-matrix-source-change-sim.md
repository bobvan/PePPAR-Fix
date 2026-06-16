# Q[3,3]-source change — validation gate

This is the empirical gate the DO-characterization design doc
(`docs/do-characterization-architecture.md` §"Q[3,3] design change vs
qFromCharPerActuator") requires *before* the engine-integration PR (step 3,
PR 2 of `doSchemaEngineIntegration` / I-192603-charlie) changes how
σ_do_freq is sourced.

**The change being validated**

| | σ_do_freq source |
|---|---|
| OLD (`qFromCharPerActuator`) | the disciplined `adjfine` / `freq_command` ASD — a control-loop signal.  On PiFace, channel-mixing made this `0.1975 ppb/√s`, 517× looser than correct. |
| NEW (this architecture) | the freerun-ADEV RWFM tail only (`0.000382 ppb/√s` on clkPoC3).  The disciplined `adjfine` ASD is reframed as σ_q under `[actuation_noise]` and never feeds Q[3,3]. |

The PiFace overnight cascade (I-100945-main) ran with the OLD over-loose
Q[3,3]; the actuator wobbled until the ResetBudget exhausted.  The question
this gate answers: does the NEW source calm that wobble **without**
introducing an over-tight / overconfident-coast failure in the other
direction?

Harness: `scripts/peppar_fix/qmatrix_source_ab.py`
(tests `test_qmatrix_source_ab.py`).


## Why the originally-agreed criterion was retired

The first-pass acceptance criterion (signed off in the dayplan) was:
*"PiFace's 60 ppb adjfine std reproduces under the OLD Q source and drops to
clkPoC3-class single-digit ppb under the NEW source."*  Prototyping against
the real traces showed **that absolute criterion is not reproducible by any
single tool**, for a load-bearing reason:

**The recorded 60 ppb was a *contained divergence*, not a property of
DOFreqEst + Q.**  The field wobble was held to 60 ppb by the full engine
reset cascade — ramp_reacquire EKF resets (actuator preserved) + ResetBudget
+ exit-5 + wrapper relaunch — repeatedly catching a loop that would
otherwise run away.  Strip that machinery and the underlying instability is
far larger.

Measured against the real `servo-overnight.csv` traces
(`~/gt/clock-stability-20260615/{piface,clkpoc3}/`; recorded adjfine matches
the postmortem exactly — PiFace std 60.62 ppb / p2p 1117.5, clkPoC3 std 3.95
ppb / p2p 378.7; `qerr_ns` empty on both = F9P/X20P HPG firmware; PiFace
`pps_err_ticc` median 40 ns vs clkPoC3 1.7 ns, so PiFace carried **both** a
517× loose Q **and** a 23× noisier measurement environment):

| | clkPoC3 (rec 3.95) | PiFace OLD Q | PiFace NEW Q |
|---|---|---|---|
| **open-loop replay** | **4.87 (PASS, 23%)** | 61 862 (diverges — no reset containment) | 1 442 |
| **synthetic closed-loop sim** | 0.15 (26× low) | 0.8 (χ² gate caps it) | 0.17 |

- Open-loop replay is faithful *exactly when the loop is stable* (clkPoC3)
  and unfaithful *exactly in the near-unstable regime* Arm 1 targets — it has
  no containment, so PiFace OLD Q diverges to 61 862 ppb (1000× the field 60).
- The synthetic sim's χ² gate (skips innovations with χ² > 100) caps the
  closed-loop wobble at ~0.8 ppb regardless of measurement noise, so it can
  never reach 60 ppb.

Neither reproduces the absolute 60 ppb, but the **direction** is
unambiguous in both, and the **relative reduction** under identical inputs is
robust to the open-loop caveat.  So the criterion was changed from an
absolute std to a relative ratio, with Bob's sign-off (2026-06-15).


## The three-arm gate (Option A)

Each arm uses the tool that is faithful for it.

### Arm 0 — faithfulness anchor (`faithfulness_anchor`)

Open-loop replay of clkPoC3 must reproduce its **recorded** adjfine std
within 30%.  Proves the CSV→DOFreqEst column mapping and wiring are correct
on a stable host before any A/B number is trusted.

> Measured: recorded 3.95 ppb → replay 4.87 ppb (23%). **PASS.**

### Arm 1 — over-loose (`over_loose_arm`)

Under PiFace's **recorded** measurement environment, the NEW Q must reduce
the replayed frequency-state chasing by ≥10× vs the OLD Q, **and** on clkPoC3
(clean env) the NEW Q must leave the healthy host within 30% of recorded
(no penalty).  A ratio under identical recorded inputs sidesteps the
open-loop divergence caveat.

> Measured: PiFace 61 862 → 1 442 ppb = **42.9× reduction**; clkPoC3 NEW vs
> recorded = 23%. **PASS.**

### Arm 2 — over-tight (two parts)

The regression the one-sided original criterion couldn't see: does the NEW
(tighter) Q cause an overconfident coast that diverges on GNSS return?

**2a `over_tight_arm_replay`** — clkPoC3 real-trace 15-min coast (injected
measurement gap) under NEW Q: √P[2,2] grows *bounded*, the first post-gap
measurements are *accepted* (χ² < 100), and the frequency estimate recovers
within a few epochs.  Faithful real-data PASS — but **not a sharp
discriminator**: clkPoC3's DO is so stable it barely wanders over a 15-min
coast, so an overconfident Q never gets punished on its real trace.

> Measured: √P22 0.167 → 7.2 ns (bounded); post-gap χ² = [0.3, 51.1, 28.2,
> 59.8, 27.0] all < 100; recovery 0 epochs. **PASS.**

**2b `over_tight_arm_synthetic`** — a controllable wandering-DO closed-loop
sim supplies the discriminating stress clkPoC3 can't.  A matched NEW Q must
recover from a measurement gap with a **no-larger** post-gap excursion than
an overconfident (100× tighter) Q, and **neither may diverge**.

> Measured: matched post-gap |innov| 444.8 ns vs overconfident 715.4 ns;
> neither diverges. **PASS.**

The reassuring finding behind 2b: the engine's adaptive-Q boost (during
pull-in) + χ² gate + sole-observer admit give real margin against Q
misspecification in the *tight* direction — which is *why* the original
one-sided criterion "couldn't catch" an over-tight problem: in this regime
there essentially isn't one.


## Running it

```sh
cd scripts
python -m peppar_fix.qmatrix_source_ab \
    --piface-csv  ~/gt/clock-stability-20260615/piface/servo-overnight.csv \
    --clkpoc3-csv ~/gt/clock-stability-20260615/clkpoc3/servo-overnight.csv
```

Exit 0 / `OVERALL: PASS` ⇒ the Q[3,3] source change is cleared for PR 2.
The unit tests (`./bin/test scripts/peppar_fix/test_qmatrix_source_ab.py`)
pin the replay mechanics, arm threshold logic, and the synthetic over-tight
arm without the (gt-only, ~10 MB) real traces.


## Verdict

All four checks PASS → **the NEW freerun-ADEV-only σ_do_freq is cleared.**
It cuts PiFace's over-loose chasing by ~43× while leaving the healthy clkPoC3
host unchanged and introducing no over-tight / overconfident-coast
regression.  PR 2 (engine integration) may proceed:

- engine reads `freerun_noise.sigma_do_freq_ppb` directly (drop
  `derive_do_process_noise`'s control-loop-source ranking),
- the disciplined `adjfine` ASD moves to `actuation_noise.sigma_q_ns` and no
  longer feeds Q[3,3].
