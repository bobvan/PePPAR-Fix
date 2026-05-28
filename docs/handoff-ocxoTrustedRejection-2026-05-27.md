# Hand-off: ocxoTrustedRejection — phases 3 + 4

**Status as of 2026-05-27 evening.**  Phases 1 + 2 landed on `main` (commits
`3b09dc2` + `6026f6b`).  Phases 3 + 4 remain.  Owner: main (whoever picks
up next session).

Cross-links: dayplan item `ocxoTrustedRejection`; related dayplan item
`madhatBootstrapStuckPostArm` (bravo); plot evidence in
`docs/3host-disc-*.png`; module + tests under `scripts/peppar_fix/`.

## Goal in one paragraph

Reject EKF Arm 4 (TICC) observations whose innovation magnitude implies
physically impossible DO frequency movement over the elapsed integration
interval.  Anchored on each DO's *characterized freerun short-τ noise
floor* (loaded from `state/dos/<label>.json`), independent of the
filter's R/Q tuning.  This complements (does not replace) the existing
chi² gate — that gate keys on `√S` which loosens whenever Q or P bloom;
the OCXO-trusted gate keys on physics, so it stays selective regardless
of filter state.

## Why we need it — twin findings from the 2026-05-27 disc-study

After 2.5 h of disciplined captures on PiFace + clkPoC3 + TimeHat with
both the morning's freerun baseline and the running engine:

**Continuous noise (PiFace pattern):**
- Innov std on TICC arm = **10 ns / epoch**.
- Filter's predicted √S = **1.1 ns** (R + state covariance too low).
- OCXO's *actual* freerun TDEV(1 s) = **54 ps**.
- Chi² gate (10σ, threshold 100) fires only 5.8 % of epochs; the other
  94 % pass and inject measurement noise into x3 → DAC → OCXO output.
- Result: disc TDEV(1 s) = 702 ps, **13× the freerun baseline of 54 ps**.

**Event-driven noise (clkPoC3 + TimeHat pattern):**
- clkPoC3 sat at +20 ppb for ~7000 s, then x3 oscillated 12–33 ppb
  between t≈7000 s and 8200 s — likely a cycle-slip / anchor-loss / NAV2
  bounce that the chi² gate let through.
- TimeHat showed 8 plateau-and-step events (`docs/3host-disc-dac-
  activity-2026-05-27.png`), residual accumulating linearly during each
  plateau because x3 is held constant.  **Original "TCXO frequency
  steps" diagnosis was wrong** — a real TCXO wouldn't toggle between
  random-walk and linear-drift 8 times, and Bob caught it on visual
  grounds.  The right reading is: between events the EKF tracks
  normally with small per-epoch updates; at each event a single
  anomalous innovation passes both gates, snaps x3 to a new level, and
  the tight P then holds x3 there until the next disturbance.

Both classes — continuous (PiFace) and event-driven (clkPoC3 / TimeHat)
— are the same underlying bug: the EKF is being told to believe
observations that the *physical OCXO can't support*.  The OCXO-trusted
gate makes "physical OCXO can't support" an explicit rejection criterion.

## What's done — phases 1 + 2

### Phase 1 — standalone module (commit `3b09dc2`)

`scripts/peppar_fix/ocxo_trusted_gate.py`:

```python
class OcxoTrustedGate:
    def __init__(self, *, sigma_short_tau_ns, k_sigma=10.0,
                 min_age_s=60.0, do_label=""):
        ...

    def evaluate(self, *, innov_ns, dt_s, age_s) -> (bool, str):
        if age_s < self.min_age_s:
            return True, "pre_min_age"   # don't fight bootstrap
        sigma_oc_dt = self.sigma_short_tau_ns * math.sqrt(max(1.0, dt_s))
        threshold_ns = self.k_sigma * sigma_oc_dt
        if abs(innov_ns) > threshold_ns:
            return False, f"ocxo_gate: |innov|=... > {threshold_ns:.2f} ns"
        return True, "accept"
```

Plus `load_sigma_short_tau_from_state(json_path)` that handles BOTH
schemas: `do_freerun_char.py`'s
`characterization.sources["DO PPS ..."].tdev_ns_by_tau_s` AND the
TimeHat offline analyzer's flat `characterization.tdev_ns`.

11 unit tests in `test_ocxo_trusted_gate.py`.

### Phase 2 — wired into `DOFreqEst` Arm 4 (commit `6026f6b`)

`DOFreqEst.__init__` gains `ocxo_trusted_gate=None`.  `update()`
accumulates `_total_age_s` and, when a gate is provided, evaluates it
*before* the chi² gate; either rejecting is enough to skip the Kalman
step.  Default `None` is bit-identical to pre-existing behavior.

5 integration tests in `test_ocxo_gate_integration.py` covering: no-gate
path preserved, gate rejects 10 ns innov vs 540 ps threshold, gate
disabled in pre-min-age window, gate accepts small innovations after
min-age, `_total_age_s` accumulates correctly across variable dt.

Full suite: **1338 passed, 1 skipped**.

## Phase 3 — engine CLI plumbing

### Files to touch

- `scripts/peppar_fix_engine.py` — add CLI args, construct gate, pass
  to DOFreqEst.
- Optionally `scripts/peppar-fix` if any wrapper-side flag muxing is
  needed (probably not; the engine's argparse will see args directly).

### CLI surface

```
--ocxo-trusted-gate          # boolean, default off
--ocxo-trusted-k-sigma  K    # float, default 10.0
--ocxo-trusted-min-age  N    # float seconds, default 60.0
```

Σ_DO loaded automatically from `state/dos/<do_uid>.json` at startup —
no need to expose σ as a CLI arg.  If the state JSON is missing or
the freerun-char section is absent, log a warning and skip the gate
(don't fail the engine — the gate is opt-in).

### Wiring point

Engine constructs DOFreqEst in `_setup_servo()` or
`_do_bootstrap_vcocxo()` / `_do_bootstrap_phc()` (search for
`DOFreqEst(` to confirm — there may be two construction sites).  Just
before the constructor call, build the gate:

```python
ocxo_gate = None
if args.ocxo_trusted_gate:
    state_path = Path('state') / 'dos' / f"{do_uid}.json"
    sigma_ns = load_sigma_short_tau_from_state(state_path, target_tau_s=1)
    if sigma_ns is not None:
        ocxo_gate = OcxoTrustedGate(
            sigma_short_tau_ns=sigma_ns,
            k_sigma=args.ocxo_trusted_k_sigma,
            min_age_s=args.ocxo_trusted_min_age,
            do_label=do_label)
        log.info("OCXO gate ENABLED: σ=%.4f ns, K=%.1f, min_age=%.1fs",
                 sigma_ns, ocxo_gate.k_sigma, ocxo_gate.min_age_s)
    else:
        log.warning("OCXO gate requested but state JSON has no usable "
                    "TDEV(1s) for %s — gate disabled", do_label)

dfe = DOFreqEst(..., ocxo_trusted_gate=ocxo_gate)
```

### Observability

Plumb gate stats into existing arm-state-log if practical — add columns
`ocxo_gate_rejected` (bool) and `ocxo_gate_reason` (string) alongside
the existing `arm_ticc_used`.  Will make post-run validation in phase 4
mechanical.

### Smoke test

`./bin/test scripts/peppar_fix/test_ocxo_gate_integration.py` should
still pass after the engine wiring (no test changes expected).

## Phase 4 — validation against today's disc-study captures

### Setup

Today's raw data lives at:

```
/home/bob/git/PePPAR-Fix/data/disc-study/day0527-disc2-{piface,clkpoc3,timehat}-ticc.csv
/home/bob/git/PePPAR-Fix/data/disc-study/day0527-disc2-{piface,clkpoc3,timehat}-arm-state.csv
/home/bob/git/PePPAR-Fix/data/disc-study/day0527-disc2-{piface,clkpoc3,timehat}-filter-state.csv
```

Plus the morning freerun JSONs:

```
/home/bob/git/PePPAR-Fix/data/freerun-day0527-2hb-comparison/{piface,clkpoc3,timehat}.json
```

### Two validation paths

**Path A — replay harness (preferred, fully controlled):**

Build a small replay script that feeds the captured TICC chA/chB stream
+ filter-state observations into a fresh `DOFreqEst` (one instance per
host, with the OCXO gate enabled).  Compare the replayed x3, adj_ppb,
and predicted output TDEV against the un-gated original.

Existing prior art: there's a replay harness pattern in
`scripts/peppar_fix/test_tdcp_slip_injection.py` — that one feeds
synthetic data, but the shape is similar to what's needed.

**Path B — live re-run on the lab hosts (faster but less controlled):**

Pull `state/dos/<label>.json` is already present on each host.  Edit
the host's `/tmp/restart-peppar.sh` (or equivalent) to add
`--ocxo-trusted-gate` and restart the engine.  Capture a fresh 2.5 h
disc-study run.  Compare TDEV against today's day0527-disc2 result.

Path A is preferred for reproducibility — replay can be run many times
with different K_σ / min_age values to find the sweet spot.

### Expected outcomes

If the gate works:
- **PiFace** TDEV(1 s) drops from 702 ps toward the 54 ps freerun
  baseline.  (May not reach 54 ps — some noise from cross-arm coupling
  remains — but should land < 200 ps.)
- **clkPoC3** event at t≈7000 s suppressed; x3 stays at +20 ppb
  through the late window.
- **TimeHat** 8 plateau events suppressed; x3 walks smoothly.
- All-host disc TDEV at long τ should fall to OCXO-class envelopes.

If the gate is too tight (rejects too much):
- The filter never updates → x3 drifts away from reality at long τ.
- Watch for TDEV(>100 s) curves climbing back up — that's
  under-discipline rather than over-discipline.

The K=10 default + min_age=60 should be a safe start.  If the validation
shows it's too loose, try K=5; too tight, try K=20.  Keep min_age=60 to
let bootstrap convergence proceed.

### Disc-study captures may still be running

PiFace + clkPoC3 + TimeHat engines were running disc-study captures
when this hand-off was written (started ~15:03 CDT, no `--duration`).
If they're still up, kill them cleanly before adding the gate so the
comparison baseline doesn't shift.  Use `sudo kill <PID>` not `pkill
-f` (see CLAUDE.md feedback memory on self-match).

## Risks + gotchas

1. **The chi² gate's threshold of 100 won't change.**  The OCXO-trusted
   gate is *additive* — it can REJECT updates the chi² gate would have
   accepted.  This is intentional.  But make sure both gates record
   their decisions in arm-state-log for post-mortem readability.

2. **σ_DO loaded from `state/dos/<label>.json` may be in cal-convention
   or engine-convention** depending on which schema and when the cal
   was run.  The magnitude is what matters for the gate; the sign
   doesn't.  But if you ever change the gate to use σ_DO in a signed
   computation, audit `do_freerun_char.py` and the schema-detection
   logic in `load_sigma_short_tau_from_state`.

3. **TimeHat is TCXO-class** with σ_short_tau ≈ 3 ns (vs OCXOs at
   ~50 ps).  The gate at K=10 → 30 ns threshold for TimeHat.  Most
   TimeHat innovations will pass.  That's correct — for a noisier DO,
   we tolerate more.  Don't be surprised if TimeHat improvement is
   smaller than PiFace.

4. **madhat is not in the captures** — engine couldn't bootstrap (see
   `madhatBootstrapStuckPostArm`, bravo's).  The OCXO-trusted gate
   *should* help madhat too: the 85 ns/s recovery rate during
   phase_setting is physically impossible for the OCXO, and the gate
   would reject those innovations.  But phase_setting uses
   `TiccTimestamper.measure_pps_frequency` (Path B), NOT the EKF.
   Adding gate logic to phase_setting is a v2 scope item, not in this
   phase 3.

5. **`_total_age_s` accumulates from DOFreqEst construction**, not
   from "bootstrap complete".  If the engine constructs the EKF early
   and bootstrap takes a while, `min_age_s=60` may fire before
   tracking begins.  Consider resetting `_total_age_s` when DOFreqEst
   enters `tracking` state, OR raising the default `min_age_s` to
   180 s to cover bootstrap.  The integration tests don't catch this.

## Pointers

- Module: `scripts/peppar_fix/ocxo_trusted_gate.py`
- Unit tests: `scripts/peppar_fix/test_ocxo_trusted_gate.py`
- Integration tests: `scripts/peppar_fix/test_ocxo_gate_integration.py`
- Wiring: `scripts/peppar_fix/do_freq_est.py` (around line 444 — the
  Arm 4 / chi² gate region)
- Plots motivating the design:
  - `docs/3host-disc-vs-freerun-tdev-2026-05-27.png`
  - `docs/3host-disc-per-window-tdev-2026-05-27.png`
  - `docs/3host-disc-residual-timeseries-2026-05-27.png`
  - `docs/3host-disc-dac-activity-2026-05-27.png`
- Dayplan: `ocxoTrustedRejection` (this work),
  `madhatBootstrapStuckPostArm` (bravo, related)
- Related design docs: `docs/two-site-sync-budget.md` (moonshot
  framing), `docs/ticc-baseline-2026-04-01.md` (freerun reference
  values), `docs/glossary.md` (DO vs PHC, sigma conventions).
