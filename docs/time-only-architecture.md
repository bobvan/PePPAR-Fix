# Time-only engine architecture — decoupled position via peppar-survey

The engine owns **time**; [`peppar-survey`](arp-survey-strategy.md) owns
**position**. They are two separate processes that meet at one file on disk.
The engine never free-estimates position in the timing loop — it pins a
position, disciplines the clock, and tracks how much its *time* confidence is
limited by how well that position is known. As `peppar-survey` refines the
position offline, the engine gently slews to it and tightens its time
confidence.

This is not an opt-in mode; for a **timing** deployment it is the
architecture. The `--no-antposest` flag is how you select it today.

## Why the engine doesn't solve position

The position filter (`PPPFilter`, ambiguity-resolution machinery, anchoring
state machine) was always a means to an end: get an accurate-enough ARP so the
time-side `FixedPosFilter` can compute `dt_rx + ZTD` correctly. Real-time,
forward-only, no-base PPP-AR is the *hardest* variant of position solving —
it is products- and phase-bias-limited (~dm), needs per-host tuning, and can
fail to converge. Offline tools already won this fight (PRIDE-PPP-AR absolute;
RTKLIB relative-baseline against a reference station), using full-arc
smoothing and better products. So the runtime position filter is just surface
area for known bugs:

- biased-equilibrium traps (day0422 + day0519 confirmed)
- false-fix lock-in (`--ar-mode full` pathology, 2026-05-20)
- ZTD/clock/ambiguity coupling under disturbance
- the (pos, ZTD, clk) null wander (position-drift-investigation-2026-06)
- ~30 % of engine complexity that never serves time output

`FixedPosFilter` has been silently correct throughout every bias-hunt round.
**Let peppar-survey own position; let the engine own time** — and, crucially,
because integer resolution is a survey concern, a *wrong* pinned position no
longer corrupts the clock loop; it only widens the time confidence (below).

`AntPosEst` is retained only for the cases timing doesn't have: **moving
platforms and unsurveyed real-time positioning**. It is never load-bearing for
a fixed-site time deployment. In those retained cases the `--nav2-floor` (#261)
bounds its free-position drift to ~NAV2 accuracy — the same "NAV2 as honest
coarse position" lever this architecture leans on at bootstrap.

## The core coupling: position error → time confidence

A pinned position wrong by **δr** makes each satellite's computed range wrong
by `δρ_i = −ê_i·δr` (ê_i = line-of-sight unit vector). The clock estimate
absorbs the common-mode part, so the induced time-confidence term is

```
σ_t(position) ≈ G · σ_r / c        G ≈ 0.3–0.7 (sky/geometry factor)
```

| Position state (σ_r) | → time-confidence term σ_t |
|---|---|
| NAV2 bootstrap (~10 m) | **~15–30 ns** |
| RTK-baseline / PPP-rapid (~cm–dm) | ~0.02–0.2 ns |
| PRIDE-final multi-day mean (~mm–cm) | tens of ps (negligible) |

Two properties make this the right coupling:

1. **Short-term stability is untouched by position error.** A fixed wrong
   position is quasi-static: as the geometry rotates it projects into the
   clock as a slow (diurnal) wander plus a DC bias. It limits **absolute
   accuracy and long-τ**, not TDEV(1 s). The DO's short-term stability is good
   from the first minute; only absolute time tightens as the survey converges.
2. **The engine is never blocked and never per-host-tuned.** It always emits
   time with an *honest* confidence that starts coarse (tens of ns on NAV2)
   and tightens automatically as σ_r shrinks. No convergence gate, no knob —
   σ_t just tracks σ_r.

σ_t is a real output: it drives the advertised **PTP `clockAccuracy` /
`clockClass`** ([ptp4l-supervision.md](ptp4l-supervision.md)) — coarse while
NAV2-bootstrapped, upgrading as the survey refines. A downstream consumer sees
the truth without reading any survey file.

## Startup: bootstrap from NAV2, never refuse

The engine starts from the NAV2 single-point fix (honest σ_r ≈ 10 m, absorbing
NAV2's 1.5–4 m receiver bias) and begins disciplining immediately. It does not
block on a survey. A better seed, when present at startup, is used directly:

| Seed source | σ_r | Use |
|---|---|---|
| `--known-pos` operator CLI | operator-stated | one-off runs |
| `state/positions/<uid>.survey.toml` | from the file's covariance | normal path — written by `peppar-survey` |
| `timelab/antennas.json[arp_label]` | ~12 mm | lab-shared antenna database |
| **NAV2 single-point** | ~10 m | **always-available bootstrap** — the engine starts here when nothing better exists |

> Build status: today the engine *refuses* (exit 1) under `--no-antposest`
> without a trusted seed. The target architecture starts from NAV2 with honest
> confidence so a fresh deployment is never blocked on a survey — it produces
> coarse-but-honest time from second one and tightens as the survey lands.

The engine writes **no** position state of its own — not `.survey.toml`
(survey-class, peppar-survey only) and not `.ppp.toml` (there is no position
filter to produce it). This keeps the survey/PPP write-ownership boundary
clean by construction.

## The refinement loop: poll, slew, re-confidence

The engine logs raw observations (`RXM-RAWX`/`SFRBX`) — the feed
`peppar-survey --auto` consumes — and watches `state/positions/<uid>.survey.toml`
for a *strictly better* estimate (lower `quality` σ / higher tier). The disk
contract:

```toml
# state/positions/<uid>.survey.toml — written atomically (temp + rename)
ecef_m  = [X, Y, Z]
cov_m2  = [...]              # feeds σ_r → σ_t
frame   = "ITRF2020@2026.50" # always ITRF2020@epoch; datum transform done in the backend
tier    = "pride-final"      # nav2 | rtk-baseline | ppp-rapid | pride-final
quality = 0.008              # 1σ 3D (m) — the monotonic "is this better?" scalar
session = { start = ..., span_s = ..., n_days = 12 }
product = "COD0MGXFIN wk2422"
converged = false            # true → engine may stop logging
```

On a better estimate the engine **slews** the pinned position `r_old → r_new`.
That move shifts the clock reference by up to `G·|Δr|/c` — a NAV2→cm upgrade is
~10 m → a **~15–30 ns** step. Applied raw the PPS OUT would jump, so the slew
is **rate-limited to the excursion budget** (spread over minutes–hours at ≤ a
few ps/s), reusing the existing **glide-slope** machinery
([phc-bootstrap.md](phc-bootstrap.md)). The DO glides to the corrected
reference; it never steps. Then σ_r → σ_t → clockClass update.

## Lifecycle

```
ACQUIRING  pos = NAV2,  σ_r ≈ 10 m,  σ_t ≈ tens-of-ns,  raw-log ON
   │  strictly-better estimate on disk
   ▼
REFINING   slew to each better tier,  σ_r ↓,  σ_t ↓,  clockClass ↑,  raw-log ON
   │  survey converged = true  (final multi-day mean stable)
   ▼
SURVEYED   pin final APC,  σ_t at floor,  raw-log OFF     ── steady state
   │  NAV2 watchdog: gross move (> 10 m)
   └────────────────────────────────────► ACQUIRING
```

`peppar-survey` typically runs for the **first ~month** at a new APC —
averaging rapid, then final, products into the authoritative sub-cm mean (the
OPUS multi-day pattern). When the final mean stops moving within its σ it flags
`converged = true`; the engine stops logging raw obs and holds the pin. The
`_check_nav2` watchdog (10 m gross-move detector) stays armed and re-triggers
the cycle on a real antenna move — matching NAV2's honest resolution (tighter
than 10 m is false-positive territory given its 1.5–4 m bias).

## What's unchanged

PHC bootstrap / DO characterization / servo loop; `DOFreqEst` clock state
machine; servo glide-slope, dead-zone, gain scheduling; TICC / EXTTS / qErr
correlation; RINEX writer; wrapper-respawn on exit 5.

## Validation

Lab-validated 2026-05-20 (PiFace canary, 1 h post-restart) that disabling the
position filter leaves the clock output unaffected — the empirical basis for
the whole split:

| τ | Time-only (`--no-antposest`) | Default (AntPosEst) | Ratio |
|---|---|---|---|
| 1 s | 0.55 ns | 0.47 ns | 1.17× |
| 10 s | 1.97 ns | 1.83 ns | 1.08× |
| 100 s | 2.79 ns | 1.56 ns | 1.79× (few independent samples) |
| 1000 s | **0.73 ns** | **0.72 ns** | **1.01× (1 ps match)** |

Same host/hardware/seed; only the engine mode changed. At τ = 1000 s (most
dispositive) the two match within 1 ps — the clock side is empirically
independent of the position filter, exactly as the architecture predicts.

`tests/test_no_antposest.py` covers flag parse/defaults, the forced
`pin_position` / `position_blend_source=none`, and `--help` advertising.

**Still owed** (target-architecture work, in build-priority order):
1. NAV2 bootstrap + honest σ_r (replace refuse-on-no-seed).
2. σ_t = G·σ_r/c confidence term → PTP clockAccuracy output.
3. Poll `.survey.toml` + rate-limited glide-slope slew on a better estimate.
4. `converged` handshake → stop raw-logging.
5. Synthetic ARP move (shift seed 15 m, restart) → confirm the watchdog trips.

## Related

- [arp-survey-strategy.md](arp-survey-strategy.md) — the position half: how
  `peppar-survey --auto` produces the progressively-refined estimates this
  engine consumes.
- [ptp4l-supervision.md](ptp4l-supervision.md) — how σ_t maps to advertised
  clock class/accuracy.
- [phc-bootstrap.md](phc-bootstrap.md) — the glide-slope the position slew
  reuses.
