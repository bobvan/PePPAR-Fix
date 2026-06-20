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

So define **DOqErr ≡ qerr(x2 + x0)** = `(x2 + x0) mod 8ns`, centered to ±4 ns.
Conceptually the de-quantized measurement is `x2 = z_extint + DOqErr`.

### ⚠️ Why you can NOT just write it as a nonlinear `h` (the TICC arm's trick fails here)

It is tempting to mirror the TICC arm with `h_extint(x) = x2 − qerr(x2 + x0)`.
**Do not** — it silently produces zero EKF gain on `x2` (Main, PR #200 review).
The quantizer argument contains the *target* state `x2`, so:

```
h = x2 − qerr(x2 + x0) = −x0 + 8·round((x2 + x0)/8)
∂h/∂x2 = 0   almost everywhere   (round() is piecewise-constant)
```

The only `x2` sensitivity sits exactly at tick boundaries — which is precisely
where `R_lin` inflates the variance — so the filter would extract almost
nothing. This is the opposite of "lean on EXTINT harder." The TICC arm gets to
keep its nonlinear `h` only because *its* quantizer argument is `x0` (the
**other** state): `h_ticc = −x2 − qerr(x0)` ⇒ `∂h_ticc/∂x2 = −1`, full
sensitivity. EXTINT's quantizer is over `x2` itself, so the same structure
cancels.

### The working form — prior-evaluated dither offset

Evaluate the dither at the **prior** state `x2⁻`, treat it as a known
per-epoch additive constant, and keep a **linear** measurement:

```
c = qerr(x2⁻ + x0)               # computed once per epoch from the prior
z_corr = z_extint + c            # de-quantized measurement of x2
h(x) = x2,   H_extint = [0,0,1,0],   R = sub-ns (residual model below)
```

`∂h/∂x2 = 1` (full sensitivity) and `z_corr` is de-quantized to sub-ns. The
innovation `z_corr − x2⁻ ≈ (x2_true − x2⁻)` when `x2⁻` is within a tick of
truth, so the EKF gets a clean, fully-weighted correction. This is — and must
be — the **"correct z with the prior, then feed"** pattern; see the sharpened
caveat 4 below for why that's the *right* thing here, not a hazard.

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

In `peppar_fix/do_freq_est.py`, Arm 3 (EXTINT) stays a **linear** update on
`x2` (`H_extint = [0,0,1,0]`) — but with a prior-evaluated dither correction
applied to the measurement, NOT a nonlinear `h` (see the gotcha above):

1. After Arm 1 (so `x0` is tight — reuse the TICC arm's sequential-ordering
   contract), compute `c = qerr(x2⁻ + x0)` from the **prior** `x2⁻` and feed
   `z_corr = z_extint + c` as a linear measurement of `x2` with `H = [0,0,1,0]`.
2. **R model for `z_corr`** — this is where the rigor lives, since the literal
   `R_lin`-on-nonlinear-`h` reasoning does not apply. `R` must cover:
   `σ(x0)²` (PPP rx-phase error) + `σ(x2⁻ within-tick)²` (prior sub-tick
   uncertainty) + a **wrong-tick tail** (a `(tick/2)²`-weighted term that grows
   as `σ(x2⁻ + x0)` approaches the tick). When converged, `R` → sub-ns; near a
   tick boundary `R` → ~tick-floor (graceful degrade to today's behavior).
3. **Convergence gate:** only apply `c` once `x2` is known to < ~4 ns (else use
   the raw linear EXTINT). Cold-start acquires on raw EXTINT, then switches to
   dither-corrected once locked.
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
4. **It MUST be "correct z with the prior, then feed" — and that's fine here**
   (corrected per Main's PR #200 review; the earlier draft had this backwards).
   The literal nonlinear-`h` form *silently fails* via the Jacobian
   cancellation above; the prior-evaluated dither (`z_corr = z + qerr(x2⁻+x0)`,
   `H_x2 = 1`) is the working form. The sub-tick correction borrowed from the
   prior is **benign** — it's a ±4 ns, slowly-varying offset, so the
   correlation with the prior is negligible and must simply be *accepted*. The
   real hazard is **not** the correlation; it's setting `R` too tight —
   ignoring the residual `σ(x0)` / `σ(x2⁻ within-tick)` and the wrong-tick tail
   (item 2). Get `R` right and the filter is neither overconfident nor blind.

## Validation plan

1. **Before baseline already captured:** the 2026-06-19 four-arm F9T/X20P run
   has raw-EXTINT arms (`no-ticc`, filter-off) on both hosts — the de-quantized
   version should be compared against these (chA-alone detrended TDEV vs τ).
2. **Sim** in `scripts/servo_sim.py`: inject 8 ns quantization on the EXTINT
   arm with/without the tick model. The acceptance assertion is that **the EKF
   actually realizes sub-ns gain on `x2`** (e.g. `P[2,2]` / the EXTINT Kalman
   gain on `x2` reaches the sub-ns regime), not merely that `z_corr` is
   algebraically de-quantized — this is the specific failure mode the Jacobian
   cancellation would hide. Also confirm short-τ TDEV drops toward the sub-ns
   floor and the loop stays stable through tick boundaries.
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
