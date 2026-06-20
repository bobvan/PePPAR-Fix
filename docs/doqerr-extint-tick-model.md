# DOqErr — de-quantize the EXTINT (DO-phase) measurement with a tick model

**Status:** design / backlog (dayplan `doQErrExtintTickModel`).
**Owner:** charlie. **Origin:** Bob, 2026-06-19.
**Relationship:** the EXTINT-arm analogue of the existing TICC-arm software qErr;
a precision lever for [`tau-aware-extint-ticc-fusion`](tau-aware-extint-ticc-fusion.md).

## The problem

The EXTINT arm (Arm 3, TIM-TM2 → DO phase `x2`) takes the receiver's timestamp
**at face value**: a linear model `H_extint = [0,0,1,0]` with `R ≈ accEst`
(~8–30 ns). But that timestamp is **quantized to the rx TCXO's 125 MHz clock**
(8 ns grid) — the receiver latches the DO PPS edge at the nearest tick and
throws away the sub-tick residual. So we eat ±4 ns (~2.3 ns RMS) of pure
quantization noise on every EXTINT sample, *even though we could predict and
remove it*.

We already remove the analogous quantization on the **GNSS-PPS / TICC** side.
The TICC arm is nonlinear and **tick-aware**:

```
h_ticc(x) = −x2 − qerr(x0),   qerr(φ) = φ − round(φ / 8ns) · 8ns
```

`qerr(x0)` is the sub-tick residual of the **rx TCXO phase** `x0` (from PPP
carrier phase, ~0.1 ns). The filter *models* the quantization from the state
instead of swallowing it as noise. The EXTINT arm just never got the same
treatment — when EXTINT was a coarse backup to a precise TICC, it didn't
matter. For EXTINT-only / no-TICC hosts and the moonshot "lean on EXTINT"
lever, it matters a lot.

## The idea — DOqErr

The TIM-TM2 timestamp quantizes the **DO PPS edge** to the rx TCXO grid. The
sub-tick residual is deterministic given where the DO edge falls on that grid,
and we know both inputs:

- `x2` = DO phase (our EKF state)
- `x0` = rx TCXO phase (from PPP, sub-ns)

**Derivation** (rx-internal time = GPS + x0; DO edge at GPS `sec + x2`; 1 s is
an exact multiple of 8 ns since 1 s = 125·10⁶ · 8 ns, so whole seconds vanish
mod 8 ns):

```
DO edge in rx-internal time      = sec + x2 + x0
receiver latches nearest tick    = round((x2 + x0)/8ns) · 8ns
receiver reports in GPS time      = round((x2 + x0)/8ns) · 8ns − x0
  ⇒  z_extint = x2 − qerr(x2 + x0)
```

So define **DOqErr ≡ qerr(x2 + x0)** = `(x2 + x0) mod 8ns`, centered to ±4 ns,
and the tick-aware EXTINT model is:

```
h_extint(x) = x2 − qerr(x2 + x0)
```

— structurally identical to `h_ticc = −x2 − qerr(x0)`, with the DO edge
position `x2` folded into the tick argument. The EKF then models the
quantization (and its tick-boundary Jacobian) instead of eating it.

## Why it's worth doing

- **EXTINT effective noise drops from ~8 ns quantization (~2.3 ns RMS) toward
  sub-ns** → the EKF can weight EXTINT far more heavily (lower R), tracking the
  DO tighter and converging faster.
- **Biggest win where we need it:** EXTINT-only / no-per-clock-TICC hosts, and
  the `tauAwareExtintTiccFusion` crossover — a de-quantized EXTINT could reach
  TICC-class short-τ *without* a per-clock TICC, and it pushes the
  EXTINT-vs-TICC crossover to shorter τ.
- **Bonus de-contamination:** because DOqErr uses our PPP `x0` (sub-ns), it
  folds in our precise rx-clock-phase knowledge, partly cleaning the rx-clock
  contribution the receiver baked into the timestamp — the same "use PPP/TDCP
  to clean the references" idea from the fusion doc.

## Implementation

Mirror the TICC arm in `peppar_fix/do_freq_est.py`:

1. Make Arm 3 (EXTINT) a **nonlinear** update: `h_extint = x2 − qerr(x2 + x0)`,
   with the Jacobian linearized at the post-PPP state (run after Arm 1 so `x0`
   is tight, exactly like the TICC arm's ordering contract).
2. **R floor + R_lin inflation near tick boundaries** — reuse the TICC arm's
   state-dependent linearization-error inflation: when `σ(x2 + x0)` is well
   below the tick, `R_lin → 0` and the de-quantized EXTINT is sub-ns; when it
   approaches the tick, `R_lin → (tick/2)²` so a wrong-tick can't poison the
   update (graceful degrade to ~tick-floor, i.e. today's behavior).
3. **Convergence gate:** only engage the tick model once `x2` is known to
   < ~4 ns (else use the raw linear EXTINT). Cold-start uses raw EXTINT to
   acquire, then switches to tick-aware once locked.
4. **CLI flag** `--extint-tick-model` (default off initially) for clean A/B,
   matching how `--router-qvir` gates the TICC hardware-qErr path.

## Honest caveats

1. **Needs PPP (x0).** Same requirement as the existing TICC tick model — fine
   on PPP hosts (PiFace), impossible on a no-PPP host.
2. **Wrong-tick is the one failure mode** — handled by the convergence gate +
   R_lin, same as the TICC arm.
3. **Short-τ, not long-τ.** It removes per-epoch quantization noise; it does
   **not** fix the long-τ nav-clock drift (e.g. the X20P `no-ticc` → 225 ns
   over 3 h on 2026-06-19). That's the reference itself wandering — a separate
   floor that only a better receiver clock or TICC anchoring addresses.
4. **Must be the measurement *model*, not "correct z then feed as independent
   sub-ns."** The de-quantized value borrows its sub-tick from `x2_prior`, so
   feeding it as an independent sub-ns measurement would double-count the prior
   and make the filter overconfident. The nonlinear-h form (what the TICC arm
   already does) handles the correlation correctly.

## Validation plan

1. **Before baseline already captured:** the 2026-06-19 four-arm F9T/X20P run
   has raw-EXTINT arms (`no-ticc`, filter-off) on both hosts — the de-quantized
   version should be compared against these (chA-alone detrended TDEV vs τ).
2. **Sim** in `scripts/servo_sim.py`: inject 8 ns quantization on the EXTINT
   arm with/without the tick model; confirm short-τ TDEV drops toward the
   sub-ns floor and the loop stays stable through tick boundaries.
3. **Lab A/B** on PiFace (F9T, has PPP): `--no-ticc` with vs without
   `--extint-tick-model`; metric = detrended chA TDEV. Expect short/mid-τ
   improvement, long-τ unchanged (nav-clock floor).
4. Cross-check the wrong-tick guard during a deliberate cold start.

## Related

- [`tau-aware-extint-ticc-fusion.md`](tau-aware-extint-ticc-fusion.md) — a
  de-quantized EXTINT shifts the EXTINT-vs-TICC crossover; update its model
  once this lands.
- `peppar_fix/do_freq_est.py` — `_qerr()`, `h_ticc`, the R_lin inflation, and
  the sequential-update ordering contract to reuse.
- `docs/pps-ppp-error-source.md` — the original 125 MHz tick / PPP-precise
  correction model this extends.
