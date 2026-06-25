# ClockMatrix platform plan — OTC & Mini as first-class PePPAR-Fix platforms

**Status**: plan (2026-06-25). PoC complete; P1 next.
**Goal (Bob, 2026-06-25)**: make the Timebeat **Open Time Card (otcBob1)** and
**OTC Mini (ptBoat)** fully supported PePPAR-Fix platforms, so PePPAR's own
**PPP solution + servo** discipline the ClockMatrix DO (the OCXO) — via the
**combo bus**, with **on-chip phase**, **no external TICC and no wire** — and
**beat the chip's hardware DPLL**.

## Background: the two ClockMatrix discipline modes

Timebeat's firmware uses the 8A34002 two ways; only the second matters here:
1. **No GNSS PPS, PTP GM present** → timebeat's *software* servo writes FCW from
   PTP-GM errors (PTP slave). Not our concern.
2. **GNSS PPS present** → the **hardware DPLL** locks the OCXO to the F9T PPS;
   timebeat software just configures it and steps aside. **This is what we
   replace.** Its performance is the baseline we must beat.

The hardware DPLL disciplines to the **raw F9T PPS** (~ns, sawtooth-limited →
~150 ps flat chA-chB in our 2026-06-25 measurement). PePPAR disciplines to the
**carrier-phase PPP solution** (sub-ns). That delta is the whole point.

## PoC result (2026-06-25) — validated, see [[project_otc_output_tdc_recon_20260623]]

A software servo on a **single DPLL channel** (DPLL3) — PLL/auto (live
`PHASE_STATUS`) + `DPLL_BW=0` (loop filter contributes nothing) +
`COMBO_SLAVE_CFG_0=0x28` (SRC_ID=8 SW-combo, unfiltered) + writing
`COMBO_SW_VALUE` — steers the DO **bypassing the loop filter**. Confirmed on the
**physical PPS OUT** via TICC#4 (chA=GNSSDO+AtomiChron, chB=otcBob1 PPS OUT):
output stayed bounded (drift 1.7 ps/s) under the combo servo, no wire/TICC.
The combo path (not write-freq, not holdover) is the one mechanism that coexists
with live on-chip phase — proven across the full STATE_MODE map.

My ad-hoc PI was ~2.7× noisier than the hardware DPLL (std 949 vs 350 ps), all
from a **mid-τ loop-resonance hump (τ≈4–8 s)** = untuned gains, not a mechanism
limit. So: mechanism proven; the servo is the work.

## Where we expect to win vs the hardware DPLL (per-τ)

- **Short τ (≲ loop BW):** both ride the *same OCXO* free-running floor → ~tie.
- **Mid τ (around the loop band):** PePPAR should **win** — the carrier-phase
  PPP reference is far cleaner than the raw F9T PPS the hardware DPLL chases, and
  a smarter loop avoids the PPS-driven mid-τ noise. **This is the target.**
- **Long τ:** both ultimately track GPS, so it's GPS/products-limited → likely a
  tie or marginal. (Bob's intuition: winning long-τ would be a pleasant
  surprise. Plausible the cleaner carrier reference lowers the GPS-tracking floor
  somewhat, but don't bank on it.)

**P4 acceptance bar: beat (or at least match) the hardware DPLL's chA-TDEV, with
the clear win expected at mid-τ.**

## Plan

### P1 — ClockMatrix combo actuator (core new code)
New `ClockMatrixComboActuator(FrequencyActuator)` (sibling to
`scripts/peppar_fix/clockmatrix_actuator.py`, which is write-freq and can't carry
live phase):
- `setup()`: DO DPLL (DPLL3) → PLL/auto; `DPLL_BW`→~0 (1 µHz);
  `COMBO_SLAVE_CFG_0=0x28` (EN, SRC_ID=8, **unfiltered**); save originals.
- `adjust_frequency_ppb(ppb)`: write `COMBO_SW_VALUE_CNFG` (signed 48-bit FFO
  ×2⁻⁵³ ≈ ppb·2⁵³/1e9) × calibrated combo gain.
- `read_frequency_ppb()`: read back combo value (and/or `FILTER_STATUS`
  delta_freq, SELECT=3).
- `teardown()`: restore config → **hand the DO back to the hardware DPLL** (the
  safe fallback).
- Register map (DPLL3): MODE 0xC4B7, BW 0xC6C0, COMBO_SLAVE_CFG_0 0xC4B2,
  COMBO_SW_VALUE 0xC6E4, PHASE_STATUS 0xC130, FILTER_STATUS 0xC098 / cfg 0xC486.
  Full detail in `timebeat-otc-register-map.md` + `clockmatrix-input-tdc-plan.md`.
- **Combo-gain calibration** (the ~0.7× seen): a `do_steering_char`-style routine,
  per host.

### P2 — on-chip phase observer arm
`clockmatrix_phase.py` already reads `PHASE_STATUS` = DO-vs-F9T-PPS = the on-chip
do_pps/gnss_pps observer (~50 ps, the **TICC replacement**). Wire it into
`DOFreqEst` as a measurement arm (sibling to TICC/EXTINT/TDCP). The **PPP carrier
arm** remains the low-noise driver; PHASE_STATUS is the PPS-edge observer; the
OCXO is the short-τ floor — DOFreqEst fuses, the combo actuator applies.

### P3 — platform detection + integration + timebeat handoff
- **Runtime board probe**: 0x58 on **i2c-15 ⇒ OTC**, **i2c-16 ⇒ Mini** (pca954x
  mux channel = PCB wiring; confirmed 2026-06-25). Sets F9T-CLK = 2 (OTC) / 5
  (Mini). Encode in `config/otcbob1.toml` / `config/ptboat.toml` + auto-verify.
- Slot combo actuator + on-chip phase source into the engine platform
  abstraction (alongside DAC / i226-adjfine).
- **timebeat handoff**: PePPAR needs exclusive ClockMatrix + F9T-serial access →
  stop timebeat, take DPLL3, and on exit restore the hardware DPLL + restart
  timebeat. Decide: PePPAR becomes the timing service, or coordinates. **Crash
  safety: always fall back to the hardware DPLL so the OTC never loses
  discipline.**

### P4 — validate the PPP servo vs the hardware DPLL
Run PePPAR's real PPP servo on the OTC; TICC#4 cross-check (chA=GNSSDO+,
chB=PPS OUT) as in the PoC; grade chA-TDEV vs the hardware-DPLL baseline. Tune
the loop (flatten the mid-τ hump — gentler `Kp`; offset already nulls). Meet the
acceptance bar, then run **TICC-free**.

### P5 — productionize
OTC + Mini in the platform matrix + docs; then the two-host **PPS-agreement
moonshot** test using the OTC physical outputs.

## Key decisions / risks
1. **Beat-the-hardware-DPLL bar** (P4) — the hardware loop is already ~150 ps;
   confirm the PPP-carrier advantage early, expect the win at mid-τ.
2. **timebeat handoff + crash safety** — graceful handback to the hardware DPLL.
3. **PPP-vs-PHASE_STATUS arm weighting** in DOFreqEst (PPP-dominant).
4. **Combo gain calibration + actuator authority/range** per host.
5. **Mini parity** — same mechanism (DPLL3, CLK5); re-verify per-board specifics
   (don't assume OTC values).
