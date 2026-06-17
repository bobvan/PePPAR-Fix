# ClockMatrix Input TDC — TICC-free phase reference on Timebeat hardware

**Status**: design / proposal (2026-06-17). Review before any hardware poking.
**Prize (Bob, 2026-06-17)**: run the PePPAR-Fix servo on Timebeat OTC
(Renesas 8A34002 ClockMatrix) with **no external dependencies** — specifically
**no TICC**. A jumper wire on the board is acceptable; persuading Timebeat to
add one on future hardware is fine. The goal is self-contained discipline on
native Timebeat hardware.

Source: Renesas AN-1010 "ClockMatrix Time-to-Digital Converter (TDC)"
(`/tmp/REN_CM-TDC_APN_20211109.pdf`, 2021-11-09) + our prior reverse
engineering (`timebeat-otc-register-map.md`, `timebeat-integration-paths.md`,
`timebeat-status-2026-04-06.md`, memory `project_8a34012_register_map`).


## Where we got stuck before (confirmed)

1. **The TDC never produced usable phase.** Output `TDC_0` was EEPROM-frozen
   at 1704 ps and wouldn't reconfigure; `TDC_1..3` never measured; DPLL
   phase-measurement mode returned zeros; `DPLL_FILTER_STATUS` all zeros.
2. **The GNSS PPS is on a clock input, not a GPIO.** F9T PPS = **CLK5**. The
   thing we kept configuring — the **Output TDC** — can only target a DPLL
   output or **GPIO1/2/6/7**. It physically cannot see an input clock, so the
   2026-04-05 blocking question ("which GPIO is CLK5 wired to?") had no answer
   by construction.
3. We fell back to driving **FCW from PPP `dt_rx` / TDCP**, with an **external
   TICC** for verification. That works — but the TICC is the external
   dependency we now want to remove.


## The unlock: there are TWO TDC classes (AN-1010)

| | **Input TDC** | **Output TDC** |
|---|---|---|
| What it is | any DPLL channel's **PFD** | dedicated unit (×4) |
| Measures between | two **input clocks `CLKn`**, same frequency | two **DPLL outputs**, or a DPLL output vs **GPIO1/2/6/7** |
| Resolution | 50 ps (phase_status); **0.39 ps** (filter_status, high-precision) | 50 ps |
| Right for us | **GPS PPS (CLK5) vs disciplined-output PPS** | output-to-output skew |

We were fighting the **Output** TDC. The correct tool for a PPS-on-an-input is
the **Input TDC** (the DPLL's phase-frequency detector). Notably, our own logs
showed `DPLL_0_PHASE_STATUS` *already live at ~25 ps/count* — **that was the
Input TDC working**; it was never configured for the pair we wanted, and the
36-bit signed value was mis-decoded (read as 64-bit, hence the impossible
"~23 s" earlier).

Input TDC facts (AN-1010):
- Any DPLL PFD; supports **0.5 Hz–200 MHz** (so 1 PPS works), range **±0.86 s**.
- `phase_status` is a **signed 36-bit** value; default resolution **50 ps**
  (= (1/625 MHz)/32), refreshes every 100 µs (for <10 kHz inputs, updates each
  input edge — i.e. each PPS). Sign: +ve ⇒ feedback leads reference.
- **High-precision mode** (decimator adds 7 bits) → `filter_status` resolution
  **0.39 ps** (50/128). Requires `tdc_clk` be **not an integer multiple** of
  the input frequency (set via the `0xCD20–0xCD24` divider; enable bit
  `0xCD24[7]`).
- TDC self-noise ≤ **90 ps**; initial settle up to **15 s**, then a few s.


## The two real gotchas AN-1010 exposes (the design crux)

**A. Same-frequency constraint.** The Input TDC compares two inputs *of the
same frequency*. GPS PPS = 1 Hz (CLK5); the OCXO = MHz (CLK2) — they can't be
compared directly. To get the servo error we actually want — **GPS PPS vs the
disciplined output** — we need the **output's 1 PPS available as a CLK input**.
That's the **jumper**: `PPS OUT → a spare CLK IN`. Then Input TDC `ref = CLK5`
(GPS), `fb = CLK_loopback` (disciplined output), both 1 Hz → phase = the
discipline error directly. (This is exactly the kind of jumper Bob is willing
to add / request from Timebeat.)

**B. High-precision (0.39 ps) is mutually exclusive with FCW on the *same*
channel.** `phase_status` (50 ps) coexists with write-frequency (FCW);
`filter_status` (0.39 ps) requires the channel be synthesizer-only. **Solution:
use separate DPLL channels** — one channel as the Input TDC (phase-measurement,
high-precision), a *different* channel doing FCW on the output. There are 8
channels; this is free. The earlier work conflated measurement and steering on
one channel.


## Register path (AN-1010 concept + our 8A34002 map)

Measurement DPLL (a **spare** channel — *not* the one feeding the i226 25 MHz;
switching that one crashed otcBob1):
- `MODE` (DPLL base `+0x37`) → phase-measurement: `pll_mode = 5` in **bits
  [5:3]** ⇒ write **`0x28`**. (The old `timebeat-integration-paths` note "write
  0x05" predates the bitfield correction in `project_8a34012_register_map`:
  pll_mode is [5:3], not [2:0].)
- `PHASE_MEASUREMENT_CFG` (DPLL base `+0x36`) → select the pair (AN-1010
  Table 1): **`PFD_REF_CLK_SEL[3:0]`** = reference CLK and **`PFD_FB_CLK_SEL[3:0]`**
  = feedback CLK, each `0x0–0xF` = CLK0–CLK15. For us: `ref = CLK5` (GPS PPS),
  `fb = CLK_loopback` (disciplined output PPS). Two 4-bit fields; likely packed
  `REF[7:4]|FB[3:0]` in the `+0x36` byte — confirm the byte layout by read-back
  in P0.
- Read `PHASE_STATUS` (`0xC118` region) for the low-precision path: AN-1010
  Table 3 = **signed 36-bit × 50 ps**, +ve ⇒ feedback leads reference. Works
  alongside FCW. (The old "impossible ~23 s" decode was a 64-bit mis-parse of a
  36-bit signed value.) For the 0.39 ps path, enable high-precision (`0xCD24[7]`
  + `tdc_clk` divider `0xCD20–0xCD24`) and read `FILTER_STATUS`: AN-1010 Table 5
  = **signed 43-bit × 50/128 = 0.390625 ps** — synthesizer-only (no FCW on this
  channel).

Steering DPLL (the output channel): FCW via `DPLL_FREQ` write-frequency
(`MODE = 0x10`), the already-proven gain-1.000 / 0.11 fppb path
(`project_clockmatrix_fcw_discovery`). Unchanged.

DPLL bases (8A34002): DPLL_0 `0xC3B0`, DPLL_1 `0xC400`, DPLL_2 `0xC438`,
DPLL_3 `0xC480`. **DPLL_3 feeds the i226** (do not use as the measurement
channel). On ptBoat, DPLL_1/DPLL_2 are freerun → good measurement candidates.


## Phased plan (low-risk first)

- **P0 — no wiring, no risk.** On a *spare* DPLL, set phase-measurement mode
  and decode `PHASE_STATUS` against two existing same-frequency inputs (or a
  known injected offset). Goal: nail `PHASE_MEASUREMENT_CFG` and the 36-bit
  encoding (× 50 ps), resolving the years-old "phase encoding unknown." Pure
  register reads; the engine/servo untouched.
- **P1 — one jumper.** `PPS OUT → spare CLK IN`. Input TDC `ref = CLK5` (GPS),
  `fb = CLK_loopback` (output) → reads GPS-vs-disciplined-output in ps. Close
  the FCW loop from it (FCW on the output DPLL, measurement on the spare DPLL).
  **No TICC in the loop** — this is the prize.
- **P2 — high precision + retire the TICC.** Turn on `filter_status`
  (0.39 ps) on the measurement channel; cross-check against the external TICC
  for one run; then remove the TICC and run TICC-free.
- **P3 — productize.** Add a `ClockMatrix` phase-observer arm to the engine
  (sibling to TICC/EXTINT/TDCP), and a `do_*_char`-style bring-up. Document the
  jumper for Timebeat (request it on future boards so no field mod is needed).

**Host**: ptBoat (all DPLLs freerun → nothing to conflict). Never switch the
i226-feeding DPLL (otcBob1 crash history). Use a spare channel for measurement.

**Scope honesty**: the servo *already* runs on ClockMatrix via FCW + TDCP/PPP
(no internal TDC required). The Input TDC's unique value is **native on-chip
phase sensing that removes the external TICC** (the stated prize), gives a
*direct* disciplined-output-PPS monitor, and is a natural enabler for the
two-host PPS-agreement goal (each box measures its own output internally).
For GPS-PPS-vs-output the F9T PPS noise (~ns) dominates the 0.39 ps TDC floor,
so the win is *removing the box*, not better resolution per se.


## Open items

**Register decode is now resolved** from AN-1010 (rendered table images,
2026-06-17):
- `PHASE_MEASUREMENT_CFG` = `PFD_REF_CLK_SEL[3:0]` + `PFD_FB_CLK_SEL[3:0]`
  (CLK0–CLK15 each).
- `PHASE_STATUS` = signed 36-bit × 50 ps; `FILTER_STATUS` = signed 43-bit ×
  0.390625 ps (50/128); +ve ⇒ feedback leads reference.
- `PLL_MODE` phase-measurement = field value **5** (Table 4) → `0x28` at the
  MODE byte's `[5:3]` (matches our FCW-write finding: write_freq=2 → `0x10`).

Remaining (none block P0/P1 on the 50 ps low-precision path):
1. **Byte layout of `PHASE_MEASUREMENT_CFG`** at `+0x36` (`REF[7:4]|FB[3:0]`
   vs two adjacent bytes) — verify by read-back in P0.
2. **Which CLK inputs are spare** on ptBoat for the PPS-OUT loopback, and
   confirm `CLK5` (F9T PPS) is selectable as `PFD_REF_CLK_SEL`.
3. **`tdc_clk` divider for 1 Hz** (high-precision / P2 only):
   `tdc_clk = fref·(w + n/d)` with `w = 0xCD24[6:0]`, `n = 0xCD20:0xCD21`,
   `d = 0xCD22:0xCD23`, chosen so `tdc_clk` is **not** an integer multiple of
   1 Hz (redo AN-1010's 8 kHz "Fine Measurement Example" for 1 PPS). The
   50 ps `PHASE_STATUS` path needs none of this — it's the P0/P1 route.
4. Cross-check the register addresses/layout against the Linux
   `idt8a340`/`rsmu` driver or the Timebeat Go parser before writing live.
