# ClockMatrix P4 — combo-servo validation runbook (otcBob1)

**Status**: runbook (2026-06-25).  First on-hardware test of the full P1–P3
stack: the combo actuator (P1) + on-chip PHASE_STATUS → DOFreqEst Arm 7 (P2) +
engine combo-actuator selection (P3).  Software-complete, **not yet lab-run**.

**Read first**: [`clockmatrix-platform-plan.md`](clockmatrix-platform-plan.md)
(the P1–P5 plan + acceptance bar), the project memory
`project_otc_output_tdc_recon_20260623` (the PoC + combo recipe + gotchas), and
CLAUDE.md "TICC stability metric" + "Cross-host PPS OUT agreement".

---

## 0. The one rule: ATTENDED ONLY

The crash-safe **hardware** revert (8A34002 write-timeout / chip watchdog) does
not exist yet (plan risk #6).  `teardown()` restores DPLL3 on clean / SIGTERM /
SIGHUP exit, but **SIGKILL / power-loss / a Pi lock-up leaves DPLL3 mid-combo
with timebeat stopped** — proven real on ptBoat 2026-06-25 (ssh drop → power
cycle).  So: **a human watches every combo run.**  No overnight, no `&`-and-walk-
away, until the hardware revert lands.

**timebeat ownership is manual by design** (Bob 2026-06-25): timebeat (hardware
DPLL) and peppar-fix (combo servo) are mutually exclusive on a host; the
operator chooses explicitly.  The engine never starts/stops timebeat.

---

## 1. Goal & acceptance bar

Show that PePPAR's combo servo — disciplining the OTC OCXO from the **PPP
carrier solution** with the **on-chip PHASE_STATUS** as the PPS-edge observer —
**matches or beats the hardware DPLL** that locks the OCXO to the raw F9T PPS.

| τ regime | expectation | why |
|---|---|---|
| short (≲ loop BW) | **tie** | same OCXO free-running floor |
| **mid (loop band)** | **WIN — the target** | carrier-phase reference ≫ the raw F9T PPS the hw DPLL chases |
| long | tie / marginal | both GPS-limited |

**Pass bar (P4): combo-servo chA−chB TDEV ≤ the hardware-DPLL baseline at every
τ, with a clear win at mid-τ.**  Baseline to beat (PoC 2026-06-25, hardware DPLL
= timebeat): std ≈ 350 ps, TDEV ≈ 150–190 ps flat (under the 354 ps per-clock
budget).  The PoC's *first* quick combo servo was ≈2.7× noisier — all a mid-τ
loop-resonance hump at τ≈4–8 s — so the work here is **tuning, not mechanism.**

---

## 2. Measurement setup — TICC#4 cross-check (the PoC rig)

Reuse the PoC's ref-immune differential rig.  One TICC, two channels, so the
TICC's own reference noise cancels in chA−chB (CLAUDE.md "shared-reference
TICC"):

```
GNSSDO+ (AtomiChron Rb) PPS ──→ TICC#4 chA   (the clean reference)
otcBob1 PPS OUT ─────────────→ TICC#4 chB   (the device under test)
TICC#4 on PiPuss
```

- chA = GNSSDO+AtomiChron (Rb-grade reference), chB = otcBob1 physical PPS OUT.
- **chA−chB, detrended, is the metric** — it grades otcBob1's PPS OUT against a
  clean reference, immune to the TICC's reference (CLAUDE.md: differential on
  one unit).  The combo servo disciplines the *internal DCO*; this rig confirms
  the discipline reaches the *physical SMA edge* (PoC closed that caveat:
  bounded chA−chB, 1.7 ps/s drift, under the combo servo).

---

## 3. Pre-flight checklist (otcBob1)

```sh
ssh otcBob1
cd ~/peppar-fix && git status && git fetch && \
  git checkout clockmatrix-combo-actuator && git pull --ff-only
```

1. **Deps**: `venv/bin/python -c "import smbus2,numpy,pyubx2,serial"` (otcBob1
   repo venv has them).
2. **DO registered as combo**: `state/dos/<uid>.toml` with
   `actuator_type = "ClockMatrix_combo"` (host-local, gitignored).  Use the same
   `<uid>` for `--do-label`.
3. **combo_gain measured**: `[steering].combo_gain` present (otcBob1: 0.978,
   R²=0.9996, 2026-06-25).  If absent → run
   `clockmatrix_combo_gain_char.py --uid <uid> --bus 15 --addr 0x58 --dpll 3`
   first (attended; it manages timebeat itself).
4. **Profile** `config/otcbob1.toml` (NOT committed by default — add for the
   run): `clockmatrix_bus = 15`, `clockmatrix_addr = "0x58"`,
   `clockmatrix_dpll_actuator = 3`, `do_label = "<uid>"`.
5. **PiPuss TICC#4**: confirm chA (GNSSDO+) and chB (otcBob1 PPS OUT) are wired
   and the TICC is live (`ssh PiPuss.local` — note PiPuss is `.local`-only and
   mDNS-flaky; retry).  Verify both channels report before steering anything.
6. **NTRIP** present (`ntrip.conf`) for the PPP carrier arm.
7. **No other users** of otcBob1's I2C / F9T (`fuser /dev/ttyAMA0`; `ps aux |
   grep -E 'peppar|combo'`).

---

## 4. The A/B protocol (interleaved, attended)

Interleave so slow environmental drift (temperature, sky) hits both arms
equally.  **2 cycles minimum** for reproducibility (the PoC standard).  Each arm
≈ 30–60 min; longer is better for long-τ but keep it attended.

For each arm, start a fresh **TICC#4 capture on PiPuss** spanning the arm, then
switch ownership on otcBob1:

### Arm A — hardware DPLL baseline (timebeat owns DPLL3)
```sh
ssh otcBob1 'sudo systemctl start timebeat'   # operator's explicit choice
# (ensure peppar-fix is NOT running)
```
Let it settle (DPLL3 re-locks to F9T PPS) before the capture window.

### Arm B — combo servo (peppar-fix owns DPLL3)
```sh
ssh otcBob1 'sudo systemctl stop timebeat'    # operator hands the host over
ssh otcBob1 'cd ~/peppar-fix && sudo venv/bin/python -u scripts/peppar-fix \
    --no-antposest --do-label <uid> 2>&1 | tee /tmp/p4-armB-$(date +%H%M).log'
```
- `--no-antposest` = the recommended time-only mode (ARP pinned via survey).
- The engine reads `clockmatrix_bus` from the profile → builds the
  **ClockMatrixComboActuator** (combo_gain=0.978), points the phase observer at
  **DPLL3** read-only, and feeds PHASE_STATUS into **DOFreqEst Arm 7** alongside
  the PPP carrier arm.  Watch the startup log for:
  `Using ClockMatrix COMBO actuator: bus=15 dpll=3 combo_gain=0.9777 (one-DPLL …)`
  and `ClockMatrix combo: reading PHASE_STATUS on DPLL_3 (read-only …)`.
- **--no-cm-phase** runs the PPP-only A/B (does the on-chip observer help?) —
  optional third arm.

### Capture (PiPuss, per arm)
```sh
ssh PiPuss.local 'cd ~/peppar-fix && venv/bin/python scripts/ticc_capture.py \
    --port <ticc4-port> --duration <s> --out /tmp/p4-<arm>-<cycle>.csv'
```
Pull every CSV + the engine log back to gt (lab eMMC is volatile):
`scp` to `bob@gt:~/gt/datasheets/p4-otcbob1-<date>/`.

---

## 5. Analysis & grading

Grade with `scripts/compare_clocks.py` (the campaign driver — ref-immune
chA−chB, graded on the moonshot excursion bounds).  Compare each combo arm
against the matched-cycle baseline arm:

```sh
# on gt, in the repo venv
python scripts/compare_clocks.py --chA <baseline-or-armB>.csv ...   # see --help
python scripts/tdev_step_c.py    # TDEV stack for the mid-τ story
python tools/plot_clock_stability_stack.py   # combo vs hw-DPLL TDEV overlay
```

**Gotchas (from the de-sawtooth + overnight memories — do not re-learn these):**
- **Detrend chA−chB vs `ref_sec`, NOT sample index** — per-channel
  `sec·1e12+ps` ramps 1e12/s; index-detrend blows up.
- **Pair chA/chB by `ref_sec`** (shared second), not by row order.
- compare_clocks CDF columns are **picoseconds**; the verdict line + TDEV are
  **ns** (CLAUDE.md feedback: check units, cross-check before interpreting).
- Guard any TDEV > 1 µs as a capture fault, not a real number.
- Grade **pass/fail on the CDF p95 excursion** (1 ns shared-antenna bound);
  use **TDEV for the "why"** (which τ regime won/lost).
- Stamp the analysis version (gitsha) on every plot (capture-recapture rule).

**Metric**: chA−chB detrended TDEV(τ), combo arm vs baseline arm, per cycle +
combined.  Plus the CDF p95 of |chA−chB| (the excursion bound).

---

## 6. Mid-τ hump tuning (reuse Charlie's STEP C)

The PoC's 2.7× was entirely a mid-τ (τ≈4–8 s) loop-resonance hump = untuned
gains, visible in BOTH the internal phase and the external chA−chB.  STEP C
(merged, `#218`/`#219`) is the fix, and it's a property of the DO + Q, not of
the actuator:

1. **Characterize the combo DO's free-run** → σ_do_freq:
   `scripts/do_freerun_char.py` on otcBob1 (timebeat off, DO free-running) →
   writes `[freerun_noise]`.  This is the honest σ the EKF needs.
2. **Apply the STEP C knobs** on Arm B:
   `--sigma-do-freq-override <σ>` (stiffer Q[3,3]) + `--coast-cap-k-sigma <k>`
   (longer coast between corrections).  STEP C cut clkPoC3 mid-τ ~23 % (2 h) /
   up to 64 % (short segments), short-τ not regressed.
3. Re-run the A/B; iterate σ/k against the chA−chB TDEV hump.
   `scripts/step_c_driver.py` + `tdev_step_c.py` drive the A/B + the plot.

Going *gentler* (lower effective loop BW so the loop stops chasing the per-sec
F9T sawtooth) is the intuition; STEP C's stiffer-Q + longer-coast is the
mechanism.  timebeat's flat ~150 ps proves the target is reachable on DPLL3.

---

## 7. Abort / recovery (attended)

Watch the engine log + the live chA−chB.  Abort if: chA−chB drifts unbounded
(combo not reaching the output), the engine logs repeated outlier resets /
state-sanity, or PHASE_STATUS reads go wild.

**To recover the host's timing at any time** — hand DPLL3 back to the hardware
DPLL:
```sh
ssh otcBob1 'sudo pkill -f peppar_fix_engine'   # engine teardown() restores DPLL3 regs
ssh otcBob1 'sudo systemctl start timebeat'     # operator re-takes ownership
```
If the engine was hard-killed (SIGKILL) or the Pi wedged and DPLL3 is stuck
mid-combo, `systemctl start timebeat` reconfigures DPLL3 from scratch (timebeat
owns it on start) — and a power cycle is the last-resort full reset (as on
ptBoat).  Confirm recovery: DPLL3_STATUS (0xC057) lock bit returns, chA−chB
re-stabilises.

---

## 8. Decision framework

- **PASS** (combo ≤ baseline all τ, win at mid-τ): the moonshot path is real on
  the OCXO class.  Then **run TICC-free** (drop the TICC#4 cross-check, trust
  the on-chip PHASE_STATUS) and proceed to **P5** (productionize + two-host PPS
  agreement).  Mid-τ win is the headline.
- **MID-τ STILL LOSES after STEP C**: characterize *why* — is it σ_do_freq
  (re-char the DO), the combo-gain (re-measure), the PPP arm noise floor, or a
  genuine combo-path artifact?  The internal-phase vs external chA−chB split
  localizes servo-dynamics (both) vs measurement (one).
- **SHORT-τ REGRESSES vs baseline**: the loop is injecting noise above its band
  — back off gain / lengthen coast (it must ride the OCXO floor at short τ).
- **PPP-only (`--no-cm-phase`) ≈ PPP+on-chip**: the on-chip observer isn't
  pulling its weight — investigate PHASE_STATUS glitch rate (Theil-Sen territory
  from the gain char) and arm weighting (PPP-dominant by design).

Record results in `docs/visual-stories.md` + a dayplan post; pull all captures
to gt.

---

## 9. Quick gotcha index

- otcBob1 ClockMatrix: **i2c-15** (behind a pca954x mux), addr 0x58, **DPLL3** =
  the DO (NOT the i226 25 MHz = a separate synth DPLL — safe to steer).
- The combo actuator owns DPLL3's config (PLL/auto + BW≈0 + SLAVE0=0x28); the
  phase observer reads PHASE_STATUS (0xC130) **read-only** — never `setup()` it
  on the combo path.
- PHASE_STATUS glitch reads (~0, ~every 5 s) — the gain char uses Theil-Sen;
  the EKF arm uses σ=50 ps and the chi²/OCXO gates absorb the rest.
- PiPuss is `.local`-only + mDNS-flaky — retry sshs.
- Launch the engine **detached but attended**: `( setsid … & )` if backgrounding
  on the host (the PoC's orchestration-hung lesson), but keep eyes on it.
- Pull captures to gt (`bob@gt:~/gt/`) — lab eMMC fails without warning.
