# ClockMatrix combo servo — which CLK input is the phase reference

**TL;DR (Timebeat, confirmed on hardware 2026-07-07):** the GNSS (F9T) 1-PPS is
on **CLK2** on **both** the Timebeat OTC and the OTC Mini.  The one-DPLL combo
servo must observe the DO against CLK2, so `clockmatrix_pps_clk = 2` on both,
and `ClockMatrixComboActuator.setup()` pins `REF_P0 = pps_clk` (PR #299).

## Why this matters

The combo servo is *steer-and-measure on one DPLL* (DPLL3): the combo bus sums a
software FFO into the DCO while the DPLL's PFD keeps measuring `PHASE_STATUS` =
(selected reference) − (DCO feedback).  That reference is `REF_P0`.  If `REF_P0`
points at the wrong CLK, the servo either observes nothing (dead input) or
disciplines the DO to the wrong signal.

## The two inputs (both GPS-derived, but not equivalent)

| CLK | Signal | Hops from GPS | Role |
|---|---|---|---|
| **CLK2** | F9T 1-PPS (raw receiver PPS) | 1 | **the intended DO-vs-F9T observer** |
| CLK5 | i226-SDP1 = the i226 PHC's PPS | 2 (GPS → PHC → PPS) | not the combo reference |

Both read *live* on the OTC (otcBob1) — the i226 PHC is itself GPS-disciplined,
so CLK5 tracks GPS too — which is why a CLK5-referenced servo "works" but carries
the extra PHC noise hop.  On the **Mini** (ptBoat) CLK5 is **dead** (its
i226-SDP1 isn't driven), so `PHASE_STATUS` against the default `REF_P0=CLK5`
reads a static zero and the servo is blind until the ref is pinned to CLK2.

## What was found (2026-07-07)

- **otcBob1** had been disciplining against `REF_P0=CLK5` — a **timebeat
  leftover**.  The pre-#299 combo actuator never set `REF_P0` from `pps_clk`; it
  read `PHASE_STATUS` read-only against whatever timebeat left.  Pinning CLK2 and
  restarting: combo disciplines cleanly on CLK2 (DOFreqEst tracking, confidence
  ~0.11 ns, 0 resets/cm_outlier, phase nulled ±4 ns).  So **#299 corrects
  otcBob1 CLK5→CLK2 — it is not a regression** (its config already said
  `pps_clk=2`).
- **ptBoat (Mini)** default `REF_P0=CLK5` is dead → #299's pin to CLK2 is what
  makes the on-chip observer live (`PHASE_STATUS` −8.85 ns, combo_gain 0.9936).

## Consequences

- `clockmatrix_pps_clk = 2` is correct on every OTC/Mini host; honest per-host
  config + #299's unconditional pin is the right design (no "only-pin-if-dead"
  heuristic needed).
- The `REF_P0`/`REF_MODE` save-restore in `setup()`/`teardown()` means the pin
  is fully reversible (a crash between them leaves `REF_MODE=manual` on the same
  CLK2 — benign).
- **Combo re-baselines** (e.g. the "combo beats timebeat 20–30% at mid-τ" A/B,
  measured on CLK5) should be redone on CLK2, since the reference the servo
  disciplines to changed.

See also `scripts/peppar_fix/clockmatrix_combo_actuator.py` (the ref-pin) and
`docs/timebeat-otc-register-map.md` (REF_P0 = DPLL base+0x0F, REF_MODE = +0x35).
