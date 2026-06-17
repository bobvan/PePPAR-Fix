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
- `PHASE_MEASUREMENT_CFG` (DPLL base `+0x36`) → select `ref = CLK5` (GPS),
  `fb = CLK_loopback`. **(bit layout = AN-1010 Table 1 — still an OPEN ITEM,
  see below.)**
- Read `PHASE_STATUS` (`0xC118` region; 36-bit signed × 50 ps); or enable
  high-precision (`0xCD24[7]` + `tdc_clk` divider `0xCD20–0xCD24`) and read
  `FILTER_STATUS` (× 0.39 ps).

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


## Open items (close before P1)

1. **`PHASE_MEASUREMENT_CFG` bitfields** (ref-select vs fb-select bit
   positions) — AN-1010 Table 1 renders as an *image*; PyMuPDF text
   extraction can't read it. **Plan: render the table pages with
   `pdftoppm` (poppler-utils, being installed on gt) and read the bitfields
   directly**; cross-check against the Linux `idt8a340`/`rsmu` driver or the
   Timebeat Go parser.
2. **`PHASE_STATUS` / `FILTER_STATUS` decode** — confirm 36-bit sign extension
   and the ITDC_UI→ps scale (50 ps default; verify against an injected offset
   in P0).
3. **Which CLK inputs are spare** on ptBoat for the loopback, and the
   CLK→register-index mapping for `PHASE_MEASUREMENT_CFG`.
4. **tdc_clk divider** values for high-precision at 1 Hz inputs (the AN-1010
   "Fine Measurement Example" worked an 8 kHz case; redo the arithmetic for
   1 Hz so `tdc_clk` is non-integer-multiple of 1 Hz).
