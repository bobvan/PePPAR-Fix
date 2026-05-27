# i226 PHC adjfine resolution: enough for OCXO discipline?

Analysis filed 2026-05-24 by charlie at Bob's request, in the context of
evaluating whether to swap the TimeHat's SMD TCXO for a TXC OK series
OCXO without also redesigning the actuator path.

The question: **if we replace TimeHat's TCXO with an OCXO (~4 ps
intrinsic ADEV(1s)), would the i226 PHC's existing `adjfine()` be
fine-grained enough to take advantage of that stability, or is the
quantization the new limiter?**

## adjfine LSB = 0.073 ppb

The Linux `igc_ptp_adjfine_i225()` driver (drivers/net/ethernet/intel/igc/igc_ptp.c)
converts the `scaled_ppm` argument into a 31-bit hardware INCVAL register:

```c
static int igc_ptp_adjfine_i225(struct ptp_clock_info *ptp, long scaled_ppm)
{
    ...
    rate = scaled_ppm;
    rate <<= 14;                       // × 2^14
    rate = div_u64(rate, 78125);
    inca = rate & INCVALUE_MASK;       // 0x7fffffff (31 bits)
    ...
    wr32(IGC_TIMINCA, inca);
    return 0;
}
```

Working out the LSB:

```
INCVAL = (scaled_ppm × 2^14) / 78125
       = (ppm × 2^16 × 2^14) / 78125          since scaled_ppm = ppm × 2^16
       = (ppm × 2^30) / 78125
       = ppm × 13,743.895

1 INCVAL LSB = 78125 / 2^30 ppm
             = 7.276 × 10⁻⁵ ppm
             = 7.28 × 10⁻¹¹ fractional
             = 72.8 ppt
             = 0.073 ppb
```

The 78125 magic number comes from the i226's 25 MHz reference clock:
`(10⁹ ppb-units) / (2 × 25 × 10⁶)` arithmetic in the hardware's
fractional-increment representation.

Maximum frequency adjustment range (`ptp_caps.max_adj`) is **62,499,999
ppb ≈ ±62.5 ppm** — far larger than any OCXO's AFC pulling range
(TXC OK is ±3.6 ppm), so range is not a concern.  Resolution is.

## Quantization noise vs OCXO intrinsic

The actuator quantization error is uniform on `[−LSB/2, +LSB/2]`:

```
σ_y_quant = LSB / √12 = 72.8 / √12 = 21 ppt = 2.1 × 10⁻¹¹
```

Treating per-epoch rounding as independent (white-FM-like with 1 Hz
servo cadence), the contribution to ADEV(τ) scales as σ_y(τ) ∝ τ^(−1/2)
and the TDEV(τ) contribution is `τ × σ_y(τ) / √3`:

```
ADEV(1s)_quant ≈ 21 ppt = 2.1 × 10⁻¹¹
σ_phase(1s) ≈ σ_y × τ = 21 ps
TDEV(1s)_quant ≈ ADEV × τ / √3 ≈ 12 ps
```

Compare to a TXC OK series OCXO's intrinsic flicker-FM-dominated
ADEV(1s) ≈ 7 ppt (from datasheet phase noise integration; see
[`tdcp-literature-survey.md`](tdcp-literature-survey.md) for context
on the conversion).

RSS combination:

```
ADEV(1s)_total = √(21² + 7²) ≈ 22 ppt
TDEV(1s)_total ≈ 13 ps
```

So with a TXC OK OCXO disciplined through the existing i226 `adjfine`
path, the theoretical discipline floor is **~13 ps TDEV(1s)**:

- **Below the TICC + Rb measurement floor** (~50–60 ps).  The
  quantization noise is invisible to current measurement equipment.
- **Below the PiFace measured floor** of 85 ps.  TimeHat post-swap
  should land in PiFace-class territory or better.
- **~3× the OCXO intrinsic.**  We lose some of the OCXO's stability to
  quantization, but not catastrophically.

## What the i226 adjfine LSB *is* the limiter for

Not the OCXO swap.  The 0.073 ppb LSB is:

- **~1300× finer** than TimeHat's free-running TCXO noise at τ=1s
  (~2 ppb σ_y from 1.17 ns TDEV(1s)).
- **~10× coarser** than the OCXO's intrinsic ~7 ppt σ_y, but
- **~3× finer than the TICC+Rb measurement chain ceiling.**

The chain measurement floor matters more here than the actuator LSB.
Until we have a quieter reference (maser-class) and a higher-resolution
measurement (sub-50 ps TICC), we cannot resolve the OCXO + i226 chain's
true performance — only confirm it's "at least as good as PiFace."

## What's actually limiting TimeHat today (and will limit it post-swap)

The 2026-05-07 freerun measurement showed disturbing data:

- TimeHat free-running TCXO: TDEV(1s) = 1.17 ns
- TimeHat disciplined output: TDEV(1s) = 3.14 ns *(worse by 3×)*

Discipline *adds* noise.  That isn't an adjfine quantization story —
LSB-quantization can only inject ~12 ps RMS at τ=1s, well below
either number.  The candidates are:

1. **Servo loop bandwidth too low.**  The 2026-05-07 memo noted i226
   PHC loop BW ~0.005 Hz vs the optimal ~0.3 Hz given the GNSS-vs-DO
   crossover at ~2–3 s.  The servo undersamples the TCXO's noise
   spectrum and aliases short-τ noise into the disciplined output.
2. **Input-side noise** from Arm 1 (PPP dt_rx, TDEV ~1–3 ns) feeding
   into the discipline path.  Main's overnight finding: switching to
   TICC-only operation (`--no-ppp-arm --no-qerr-arm --no-extint`)
   drops chA TDEV(1s) on PiFace from 475 → 95 ps.
3. **adjfine application jitter** — *when* the write hits the
   hardware, not what value it carries.  Less likely to dominate but
   worth instrumenting if 1 + 2 are fixed and a residual remains.

For TimeHat post-OCXO-swap to land at the ~13 ps theoretical floor, all
three need to be addressed.  Specifically:

- Use Main's validated TICC-only operational config
  (`--no-ppp-arm --no-qerr-arm --no-extint`).
- Re-tune the servo loop bandwidth for the OCXO's noise spectrum
  (the optimal BW shifts to ~10–30 Hz given the OCXO's much-faster
  noise floor compared to the TCXO).
- Measure post-swap freerun and disciplined TDEV against the same
  TICC+Rb fixture used for the 1.17 ns baseline.

## Bottom line

**The i226 adjfine has enough resolution to make the OCXO swap
worthwhile.**  The 0.073 ppb LSB injects ~12 ps RMS into TDEV(1s),
below the OCXO's intrinsic floor by only ~3× and below the TICC+Rb
measurement chain by ~4×.  The OCXO's full intrinsic stability is
*mostly* preserved through the existing adjfine path.

This is a much simpler hardware migration than the OCXO+18-bit-DAC
path PiFace took.  PiFace's DAC LSB is ~27 ppt (2.7× finer than
i226 adjfine), but PiFace measures at 85 ps anyway because the
measurement chain dominates over both PiFace's and i226's
quantization noise.

**No DAC redesign needed for the OCXO swap.**  Servo tuning + arm
selection are the real work.

## Caveats

- **Update cadence matters.**  This analysis assumes 1 Hz servo
  cadence, with per-epoch rounding errors uncorrelated.  At slower
  cadence (e.g., 8 s scheduler interval), quantization noise integrates
  over longer τ and the TDEV(1s) contribution rises.  At faster
  cadence (10+ Hz), the LSB rounding noise averages out further but
  may exceed the i226 PHC's actual responsiveness.
- **Dithering** (deliberate sub-LSB noise injection) can improve
  effective resolution but requires care to avoid biasing the average.
  Not implemented in any current PePPAR-Fix servo path.
- **Multi-epoch holding** (only updating adjfine every N epochs) would
  make quantization noise correlated and the TDEV contribution
  proportionally larger.  Not currently a concern unless someone adds
  such logic.
- **OCXO intrinsic ADEV(1s) ≈ 7 ppt** is *inferred* from the datasheet
  phase noise integration.  Bench measurement against a Rb or maser
  reference would confirm the actual number, but is bounded by
  measurement-chain noise floor.

## Related docs

- [`tdcp-literature-survey.md`](tdcp-literature-survey.md) — context on
  TDCP timing applications and the σ_y conversion from phase noise.
- [`igc-kernel-patches.md`](igc-kernel-patches.md) — i226 driver patch
  history (ppsfix + adjfine patches).
- [`ticc-baseline-2026-04-01.md`](ticc-baseline-2026-04-01.md) — F9T
  PPS and i226 TCXO baselines that motivated the 1.17 ns TimeHat TCXO
  number.
- [`two-site-sync-budget.md`](two-site-sync-budget.md) — moonshot
  per-clock budget that the OCXO swap is meant to clear.

## Sources

- [Linux igc PTP driver source (igc_ptp.c)](https://github.com/torvalds/linux/blob/master/drivers/net/ethernet/intel/igc/igc_ptp.c)
- [Intel I225/I226 datasheet](https://www.mouser.com/datasheet/2/612/Intel-Corporation-2585974.pdf)
- [ptp_clock_info kernel API: adjfine and scaled_ppm](https://www.spinics.net/lists/netdev/msg850620.html)
- TXC OK Series ThermSymEros 5×3.2 OCXO datasheet (Rev. A 2024.06.13), sales-contact-gated; key specs summarized in `tdcp-literature-survey.md`.
