# Weak antenna ZTD doom loop — Patch4 window-ledge observation

**2026-04-29 afternoon, PiFace travel rig, F9T-3RD on Patch4 (u-blox
ANA-MB2 indoor on a south-facing window ledge).**

First systematic observation of what the engine looks like under
sustained pseudorange-multipath bias too large for the EKF to
distinguish from real atmospheric delay.  Captured for future
comparison against the same receiver on Patch3 (same ANA-MB2 model,
outdoor sloped-roof mount).

## Setup

- **Receiver**: F9T-3RD (ZED-F9T-20B, TIM 2.25, SEC-UNIQID
  `394029318459`), on `/dev/ttyACM0` of PiFace (Pi 5, Trixie).
- **Antenna**: Patch4, u-blox ANA-MB2, mounted on a south-facing
  window ledge.  Sky view restricted to the southern hemisphere by
  building structure; near-field has glass + window frame.
- **DO stack**: AD5693R DAC (`0x4C` on `/dev/i2c-1`) → CTI
  OSC5A2B02 OCXO → TADD-2 Mini divider.
- **TICC #3** on `/dev/ticc3`.
- **Engine config**: `config/piface.toml` (forked from
  `config/clkpoc3.toml`; only `serial` by-path differs).
- **Engine commit**: 75b8295 (post Phase-1 σ default 0.02→3.0;
  pre-Phase-1-σ-inflation-on-save fix landed during the run).

## Observations

### Phase 1 graduates with multiple retries

```
attempt 1/3  REJECTED  horiz disp 5.5m vs NAV2
attempt 2/3  REJECTED  rms_pr=11.26m; horiz 9.6m
attempt 3/3  REJECTED  rms_pr=6.67m
CONVERGED epoch 254  σ=2.165m  rms=1.497m  pr_rms=2.12m  nav2_h=0.1m  retries=3
```

Three rejections (W2 + W1) before the LS finally converges within
the 5 m W2 horizontal gate.  The resolved position was actually
clean (nav2_h = 0.1 m at handoff) but Phase 1 took ~250 epochs to
get there and burned all three retries.  After a subsequent antenna
re-orientation, a fresh run could not re-acquire that quality —
suggesting the convergence basin is narrow at this site.

### Integrity-trip cadence is a mechanical limit cycle

```
18:17:37  ztd_impossible  ztd=+1.993m
18:23:55  ztd_cycling    ztd=−1.042m  +6m18s
18:47:21  ztd_impossible  ztd=+1.117m  (+23m gap; slip storm interlude)
18:53:39  ztd_cycling    ztd=+0.815m  +6m18s
18:59:57  ztd_impossible  ztd=+1.046m  +6m18s
19:06:16  ztd_cycling    ztd=+1.052m  +6m19s
19:12:34  ztd_impossible  ztd=+0.945m  +6m18s
19:18:52  ztd_cycling    ztd=+1.073m  +6m18s
```

Trips repeat at 6m18s ± 1s.  The mechanism reads as:

1. Trip → ZTD reset to 0 ± 500 mm (re-init).
2. ~5 minutes of EKF evolution → ZTD walks up to threshold (±700 mm).
3. ~60 seconds of sustained over-threshold → integrity counter arms.
4. Trip → repeat.

The bias source is **constant** (Patch4's multipath pattern doesn't
change between trips), so each re-init re-encounters the same
forcing function and walks back to threshold in the same time.  The
threshold-violation values land at +0.8 to +1.1 m most of the time
— a coherent, persistent pull on ZTD ~1 m above its true value.

### Cycle slip rate is ~9.6 per minute

**652 cycle slip flushes in 68 minutes** (9.6 / minute).  For
comparison, lab antennas (UFO1 + GUS #1) typically run < 1 / minute
on a clear sky.  Slip causes are mixed: `ubx_locktime_drop`,
`gf_jump`, and `mw_jump` all appear; mostly low-elevation SVs.

### WL fixes happen but never settle

Top SVs by re-fix count:

```
G23: 19 fixes      E06: 13      C31: 12
E10: 15            G18: 12      C22: 12
G10: 13            E04: 12      E36: 11
                   C34: 11
```

19 re-fixes for G23 in 68 minutes ≈ one re-fix every 3.6 min.  Each
fix lasts only as long as the next slip on that SV.  The fix set
never accumulates a stable membership — every integrity trip flushes
all current fixes anyway, so the maximum lifetime is bounded by the
6m18s cadence regardless.

### NL never attempts

Latest snapshot before the run was killed:

```
[AntPosEst 3840] σ=0.372m pos=(LAT,LON,ALT)
                 WL: 8/16 fixed  NL: 0 fixed  P=0.098
                 nav2Δ=4.6m  ZTD=+927±118mm  worstσ=0.5m
```

`P_IB = 0.098` — bootstrap success probability ~10%, far below the
0.97 LAMBDA threshold.  LAMBDA is dormant; rounding is dormant.  No
`[NL_ADMIT]`, no `[NL_ADMIT_BLOCK]`, no `[NL_EVICT]` over the entire
run.  `NlAdmissionTier` is uninvoked.

ZTD at +927 mm is already past the ±700 mm threshold — another
trip was imminent at kill time.

### What the EKF *did* manage

- **σ_position = 0.372 m** (down from initial seed of ~10 m)
- **nav2Δ = 4.6 m** (down from 9.5 m at startup) — agreement with
  NAV2 was *improving* despite the cycling
- **Position drift**: lat/lon barely moves epoch-to-epoch
- **Tide / windup / SSR streams**: all flowing, applied correctly

So **position converges; ZTD is what's poisoned**.  PR multipath is
a slow biased forcing function the EKF can't distinguish from real
neutral-atmosphere delay — both modify the L1+L5 IF pseudorange in
ways that look like clock + ZTD coupling.

## Mechanism: why ZTD eats the bias

Pseudorange measurement model (simplified IF):

```
ρ_IF = ρ_geo + c·dt_rx + tropo + ε_PR_IF
```

where `tropo` decomposes into `tropo_dry + ZTD·m(elev)` and ε_PR_IF
absorbs measurement noise + multipath.  Multipath has a slowly-
varying systematic component (multipath geometry depends on antenna
+ environment, not time) plus a fast random component.

The EKF's job is to attribute observed `ρ_IF − ρ_geo − c·dt_rx`
between **tropo** and **ε**.  It does this by leaning on:

1. **Multi-SV consistency** — atmospheric delay is shared (mostly);
   per-SV residuals after subtracting ZTD-induced delay should be
   noise-like.
2. **Elevation dependence** — tropospheric mapping function gives
   characteristic per-SV signature; multipath does *not* (or only
   weakly).

When the multipath bias has **per-SV elevation structure**, the EKF
mistakes it for trop and pumps the bias into ZTD.  Window-ledge
mounts have exactly this character: the southern wall + glass +
window frame produce direction-dependent multipath that aliases as
elevation-dependent delay.  ZTD absorbs the lot.

Once ZTD diverges past the engine's `ztd_impossible` threshold
(±700 mm from the prior, indicating physically-impossible neutral
atmosphere), `FixSetIntegrityMonitor` correctly trips and resets
ZTD.  The reset doesn't change the multipath geometry, so the
bias re-accumulates at the same rate.  Doom loop.

## Why the existing safeguards don't help here

| Safeguard | Why it's silent on Patch4 |
|---|---|
| W1 residual consistency at Phase 1 | PR residuals *are* large (rms 11 m peak) — W1 correctly rejects until the LS happens to land in the convergence basin |
| W2 NAV2 horizontal at Phase 1 | Same — rejects most candidates; eventually one lands inside the 5m gate |
| `SecondOpinionPosMonitor` | nav2Δ ≈ 6-9 m at runtime, but Bravo's hAcc gate (1.5 m, 30-ep window) holds the trip because NAV2 itself reports rolling hAcc ≈ 4.8 m — NAV2 isn't trustworthy enough as a witness |
| `NlAdmissionTier` (today's I-172719 work) | Never exercised — P_IB ≈ 0.1 means LAMBDA never even runs, so there are no NL admissions to gate |
| Cycle-slip detection | *Working* — 652 flushes is the system correctly detecting 9.6 slips/min |
| Phase-1 σ-inflation save (today's bug fix) | Will help on future runs after antenna moves, but doesn't help while running |

## Comparison hypothesis (Patch3 same-model rig)

Bob switched the F9T-3RD coax from Patch4 to Patch3 (same u-blox
ANA-MB2 model, outdoor sloped-roof mount, East roof slope).  If the
doom loop is genuinely multipath-driven (not antenna-component
driven), the Patch3 run should show:

- Phase 1 converges in 1-2 attempts (not 3 retries)
- Cycle slip rate < 2 / minute
- ZTD settles to a stable physical value (~2.0–2.5 m for the lab's
  sea-level pressure)
- WL fixes accumulate to 12-17 stable members
- NL admissions begin, P_IB rises past 0.97
- `NlAdmissionTier` exercises (TRUSTED tier accumulates over ~30 min)

If Patch3 *also* shows the doom loop, the antenna model itself is
the issue (not the mount environment) — a stronger result that
would push PePPAR-Fix toward antenna-quality requirements rather
than mount-location requirements.

The same-day Patch3 capture follows in the immediately-next engine
run on PiFace (post-state-reset, fresh cold-start).

## Practical takeaways

1. **Window-ledge antennas are not viable for sustained-anchor
   PPP-AR work**, even with ANA-MB2-quality hardware.  The
   multipath bias exceeds what the engine's ZTD-channel safeguards
   can compensate for.

2. **Outdoor mount is necessary** for Patch4-class antennas to
   produce the kind of fixes the engine is designed to refine.
   Position σ converges fine indoors; everything else (ZTD,
   integrity, NL) fails.

3. **Phase-1 σ-inflation-on-save** (today's `bad193f`) means a
   bad-environment run no longer permanently poisons future runs'
   trust gate — every restart re-validates.  Important defense
   against precisely this case.

4. **Multipath-aware PR weighting** is a future possible
   mitigation: detect per-SV PR-residual structure that doesn't
   match the elevation mapping function (i.e., looks like
   geometry-dependent multipath rather than zenith-coupled
   troposphere) and inflate that SV's measurement σ.  Out of scope
   for today; filed as a future-work pointer.

## Cross-references

- `config/piface.toml` — engine config for this rig
- `state/dos/ocxo-clkpoc3.json` — calibration carry-over from
  clkPoC3 (DAC ppb_per_code = 0.0361; "midscale ≈ +149 ppb" = the
  OCXO's pull *at the midscale code* — midscale is the neutral DAC
  command, not the GNSS-matching point, and the GNSS-matching code
  drifts with temperature; see noMagicCenterCode /
  `docs/do-characterization-architecture.md` §"Code symmetry vs pull
  symmetry")
- `docs/weak-antenna-doom-loop-2026-04-29.md` — this file
- Capture: `~/peppar-fix/data/piface-smoke5.log` on PiFace
- Engine commits: `75b8295` (σ default), `bad193f` (save floor)
