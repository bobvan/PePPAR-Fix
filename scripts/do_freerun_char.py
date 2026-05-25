#!/usr/bin/env python3
"""Free-run + phase-noise characterization of a DO via TICC chA (vs Rb).

Parks the DAC at a fixed code (default: code yielding ~0 ppb offset
from the existing dac_slope_cal.py output, if present in the JSON;
else mid-scale 32768), then captures TICC chA timestamps for a
configured duration.  The TICC's chA timestamp is referenced to the
TICC's internal Rb-disciplined timebase (sub-100 ps TDEV(1s) per the
lab's Rb characterization), so chA alone (detrended) measures the
DO's freerun noise vs the Rb — what you want for an apples-to-apples
oscillator comparison.

Per CLAUDE.md `### TICC stability metric: use chA alone, not chA-chB`.

Computes:

  - RMS of phase residual (ns)
  - PSD curve at standard offset frequencies (0.005–0.5 Hz from 1 Hz
    TICC; this is the *low-frequency* phase noise band — the spectral
    region that overlaps with the moonshot's 1-sec to ~200-sec
    timescale.  Higher-band L(f) requires a phase-noise analyzer.)
  - ASD at 0.1 Hz, 0.01 Hz
  - Spectral slope (log-log linear fit) + noise-type classification
  - ADEV / TDEV / MDEV at standard τ
  - Crossover frequency (TODO: vs ref like dt_rx — N/A here)

Output: a `characterization` section appended to the existing DO JSON
file (e.g. state/dos/<label>.json).  If the file has a slope-cal
section already (from dac_slope_cal.py), this script merges into it.

The DO PPS is read on TICC chA; GNSS PPS on chB.  Linear drift
(constant frequency offset from the parked code's residual) is
removed before spectral analysis so the PSD reflects the *noise*
floor, not the drift.

Usage:
    sudo ~/peppar-fix/venv/bin/python ~/peppar-fix/scripts/do_freerun_char.py \\
        --bus 1 --addr 0x4C --gain 1 \\
        --code 35000  \\
        --ticc-port /dev/ticc2 \\
        --duration-s 3600 \\
        --output state/dos/isotemp-ocxo-33-madhat.json

Safe to interrupt — restores DAC to mid-scale in `finally`.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

import numpy as np  # noqa: E402

from peppar_fix.dac_actuator import DacActuator  # noqa: E402
from ticc import Ticc  # noqa: E402


# Standard PSD offset frequencies (Hz).  Matches the PiFace
# characterization JSON's psd_curve grid.  Nulls fill where the
# capture duration is too short to resolve.
_PSD_FREQS_HZ = (0.001, 0.002, 0.003, 0.005, 0.007, 0.01, 0.02, 0.03,
                 0.05, 0.07, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5)
# Standard τ values (s) for ADEV/TDEV/MDEV.
_TAUS_S = (1, 2, 5, 10, 30, 100, 300, 1000)


def classify_noise_type(slope: float) -> str:
    """Map log-log PSD slope to phase-noise type (rough categories)."""
    if slope > -0.5:
        return "white_phase"
    if slope > -1.5:
        return "flicker_phase"
    if slope > -2.5:
        return "white_FM"
    if slope > -3.5:
        return "flicker_FM"
    return "random_walk_FM"


def collect_chA_series(ticc_port: str, duration_s: float):
    """Listen to TICC; collect all chA and chB edges separately.

    Returns (chA_records, chB_records, paired) where:
      - chA_records: list of (ref_sec, chA_phase_s) in arrival order
      - chB_records: list of (ref_sec, chB_phase_s)
      - paired: list of (ref_sec, chA_minus_chB_ns) for those seconds
                where both edges arrived (kept for offline re-analysis
                of how well the DO tracks GNSS, distinct from the DO's
                intrinsic noise).

    The PRIMARY freerun-noise metric uses chA alone, detrended against
    the TICC's Rb reference timebase (sub-100 ps TDEV(1s) per host).
    chA−chB would mix in F9T/F10T PPS jitter and tell us the DO+GNSS
    chain instead — that's the discipline-loop-tracking metric per
    CLAUDE.md `### TICC stability metric: use chA alone, not chA-chB`.
    """
    chA_dict: dict[int, float] = {}
    chB_dict: dict[int, float] = {}
    chA_records: list[tuple[int, float]] = []
    chB_records: list[tuple[int, float]] = []
    paired: list[tuple[int, float]] = []
    seen_pairs: set[int] = set()
    t0 = None
    with Ticc(ticc_port, wait_for_boot=False) as ticc:
        for ch, sec, ps in ticc:
            if t0 is None:
                t0 = time.monotonic()
            elif time.monotonic() - t0 > duration_s:
                break
            phase_s = sec + ps * 1e-12
            if ch == 'chA':
                chA_dict[sec] = phase_s
                chA_records.append((sec, phase_s))
            elif ch == 'chB':
                chB_dict[sec] = phase_s
                chB_records.append((sec, phase_s))
            for s in set(chA_dict) & set(chB_dict):
                if s not in seen_pairs:
                    paired.append((s, (chA_dict[s] - chB_dict[s]) * 1e9))
                    seen_pairs.add(s)
            cutoff = sec - 5
            for s in list(chA_dict):
                if s < cutoff and s not in chB_dict:
                    del chA_dict[s]
            for s in list(chB_dict):
                if s < cutoff and s not in chA_dict:
                    del chB_dict[s]
    return chA_records, chB_records, paired


def detrend_linear(t: np.ndarray, y: np.ndarray):
    """Linear-detrend y vs t.  Returns (residual, slope_per_sec, intercept)."""
    if len(t) < 2:
        return y - y.mean() if len(y) else y, 0.0, 0.0
    slope, intercept = np.polyfit(t, y, 1)
    return y - (slope * t + intercept), slope, intercept


def compute_asd(t: np.ndarray, y: np.ndarray, target_freqs: tuple):
    """Compute one-sided amplitude spectral density (units of y per √Hz).

    Welch-like single-FFT estimator since our duration is fixed.
    Returns dict freq → ASD value, and the full (freqs, asd) curves.

    Sample rate is derived from the MEDIAN inter-sample interval, not
    `t[1] − t[0]` — robust to occasional missed pairs at the start of
    the capture (one pair could lead to a multi-second gap in the
    first inter-sample interval, which would otherwise yield a wildly
    wrong fs and all-None PSD output).
    """
    n = len(y)
    if n < 4:
        return {f: None for f in target_freqs}, np.array([]), np.array([])
    dts = np.diff(t)
    median_dt = float(np.median(dts))
    if median_dt <= 0:
        return {f: None for f in target_freqs}, np.array([]), np.array([])
    fs = 1.0 / median_dt
    # Report non-uniform sampling if median != min/max within a few percent
    if n > 10 and (dts.max() - dts.min()) > 0.5 * median_dt:
        log_msg = (f"WARN: non-uniform sample spacing detected — "
                    f"min={dts.min():.3f}s median={median_dt:.3f}s "
                    f"max={dts.max():.3f}s. FFT assumes uniform; "
                    f"results may have aliasing artifacts.")
        print(log_msg)
    # Hann window to reduce spectral leakage
    win = np.hanning(n)
    yw = (y - y.mean()) * win
    Y = np.fft.rfft(yw)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    # Window correction factor for amplitude (Hann coherent gain = 0.5;
    # power gain = 0.375 of the rectangular).  ASD in y_units/√Hz:
    win_norm = (win ** 2).sum()
    psd = (np.abs(Y) ** 2) / (fs * win_norm) * 2  # one-sided
    asd = np.sqrt(np.maximum(psd, 0.0))
    # Pick the closest freq bin for each target
    out = {}
    for f in target_freqs:
        if f > freqs.max() or f < freqs.min():
            out[f] = None
            continue
        idx = int(np.argmin(np.abs(freqs - f)))
        out[f] = float(asd[idx])
    return out, freqs, asd


def loglog_slope(freqs: np.ndarray, asd: np.ndarray,
                  f_lo: float = 0.01, f_hi: float = 0.5):
    """Fit log-ASD vs log-freq over [f_lo, f_hi].  ASD ∝ f^(slope/2)
    so the PSD slope is 2× this.  Returns PSD slope (∝ f^slope_psd).
    """
    mask = (freqs >= f_lo) & (freqs <= f_hi) & (asd > 0)
    if mask.sum() < 3:
        return float('nan')
    lf = np.log10(freqs[mask])
    la = np.log10(asd[mask])
    slope_asd, _ = np.polyfit(lf, la, 1)
    return 2.0 * slope_asd  # PSD slope = 2 × ASD slope (since PSD = ASD²)


def compute_allantools_metrics(y_ns: np.ndarray, rate_hz: float, taus_s):
    """Compute ADEV / TDEV / MDEV at the requested τ values via allantools.

    Returns dicts τ → metric.
    """
    try:
        import allantools
    except ImportError:
        return None, None, None
    out_adev = {}
    out_tdev = {}
    out_mdev = {}
    valid_taus = [t for t in taus_s if t < len(y_ns) / rate_hz / 4]
    if not valid_taus:
        return out_adev, out_tdev, out_mdev
    # ADEV/MDEV on fractional frequency (dimensionless); TDEV on phase.
    phase_s = y_ns * 1e-9  # convert ns → s for allantools
    tau_arr = np.array(valid_taus, dtype=float)
    t_a, adev_arr, _, _ = allantools.adev(phase_s, rate=rate_hz,
                                          data_type='phase', taus=tau_arr)
    t_t, tdev_arr, _, _ = allantools.tdev(phase_s, rate=rate_hz,
                                          data_type='phase', taus=tau_arr)
    t_m, mdev_arr, _, _ = allantools.mdev(phase_s, rate=rate_hz,
                                          data_type='phase', taus=tau_arr)
    for τ, a in zip(t_a, adev_arr):
        out_adev[float(τ)] = float(a)
    for τ, td in zip(t_t, tdev_arr):
        out_tdev[float(τ)] = float(td) * 1e9  # back to ns for output
    for τ, m in zip(t_m, mdev_arr):
        out_mdev[float(τ)] = float(m)
    return out_adev, out_tdev, out_mdev


def pick_code_for_zero_ppb(json_path: Path, fallback: int = 32768) -> int:
    """If a slope-cal section is present in the JSON, return the DAC code
    nearest to ppb=0.  Otherwise return fallback (mid-scale).
    """
    if not json_path.exists():
        return fallback
    try:
        data = json.loads(json_path.read_text())
    except Exception:
        return fallback
    # The slope-cal output writes top-level keys
    slope = data.get('dac_ppb_per_code')
    icpt = data.get('dac_center_freq_offset_ppb')
    if slope is None or icpt is None:
        return fallback
    # ppb(code) = icpt + slope * (code - 32768) → solve for ppb=0
    if slope == 0:
        return fallback
    code = round(32768 - icpt / slope)
    code = max(0, min(65535, code))
    return code


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bus', type=int, default=1)
    ap.add_argument('--addr', type=lambda s: int(s, 0), default=0x4C)
    ap.add_argument('--gain', type=int, choices=[0, 1], default=0)
    ap.add_argument('--code', type=int, default=None,
                    help='DAC code to park at (default: derived from slope cal in --output JSON; else 32768)')
    ap.add_argument('--ticc-port', default='/dev/ticc2')
    ap.add_argument('--output', required=True,
                    help='JSON to merge characterization into')
    ap.add_argument('--do-label', default=None)
    ap.add_argument('--host', default=None,
                    help='Host name written into the characterization section')
    ap.add_argument('--duration-s', type=int, default=3600,
                    help='Capture duration in seconds (default: 3600)')
    ap.add_argument('--raw-csv', default=None,
                    help='Optional path for raw TICC pairs (ref_sec, chA_minus_chB_ns). '
                         'Lets us re-analyze offline without another capture.')
    args = ap.parse_args()

    out_path = Path(args.output)
    code = args.code if args.code is not None else pick_code_for_zero_ppb(out_path)
    do_label = args.do_label or out_path.stem

    print(f"DO freerun + phase-noise characterization for {do_label}")
    print(f"  DAC:        bus={args.bus} addr=0x{args.addr:02X} gain={args.gain} code={code}")
    print(f"  TICC:       {args.ticc_port}")
    print(f"  Duration:   {args.duration_s} s ({args.duration_s/60:.1f} min)")
    print()

    dac = DacActuator(args.bus, args.addr, bits=16, ppb_per_code=1.0,
                      dac_type='ad5693r', dac_gain=args.gain)
    dac.setup()
    dac._write_code(code)
    print(f"DAC parked at code {code}")

    chA_recs: list[tuple[int, float]] = []
    chB_recs: list[tuple[int, float]] = []
    paired: list[tuple[int, float]] = []
    try:
        print(f"\nWarmup 60s after DAC step (thermal settle)…")
        time.sleep(60)
        print(f"\nCapturing {args.duration_s} s of TICC chA + chB…")
        t_start = time.monotonic()
        chA_recs, chB_recs, paired = collect_chA_series(args.ticc_port,
                                                        args.duration_s)
        t_cap = time.monotonic() - t_start
        print(f"  → chA edges: {len(chA_recs)}  chB edges: {len(chB_recs)}  "
              f"paired-by-sec: {len(paired)}  ({t_cap:.0f} s wall)")
    finally:
        try:
            dac._write_code(32768)
        except Exception:
            pass
        dac.teardown()

    if len(chA_recs) < 60:
        print(f"\nToo few chA edges ({len(chA_recs)}) for meaningful statistics")
        return 1

    if args.raw_csv:
        raw_path = Path(args.raw_csv)
        raw_path.parent.mkdir(parents=True, exist_ok=True)
        # Save chA + chB separately + the (chA−chB) pairing.  All three
        # are useful: chA alone is the freerun DO noise; chA−chB is the
        # tracking metric; chB alone characterizes the GNSS PPS chain.
        chA_by_sec = dict(chA_recs)
        chB_by_sec = dict(chB_recs)
        secs_all = sorted(set(chA_by_sec) | set(chB_by_sec))
        with open(raw_path, 'w') as f:
            f.write("ref_sec,chA_s,chB_s,chA_minus_chB_ns\n")
            for s in secs_all:
                a = chA_by_sec.get(s)
                b = chB_by_sec.get(s)
                a_str = '' if a is None else f"{a:.12f}"
                b_str = '' if b is None else f"{b:.12f}"
                diff_str = '' if (a is None or b is None) else f"{(a-b)*1e9:.6f}"
                f.write(f"{s},{a_str},{b_str},{diff_str}\n")
        print(f"Raw chA + chB + pairs saved → {raw_path}")

    # ── PRIMARY: chA alone vs TICC's Rb reference ─────────────────
    # The TICC's chA timestamp is the DO PPS edge time on the TICC's
    # internal Rb-disciplined timebase.  Rb TDEV(1s) is <100 ps per
    # the lab's TICC characterization (sub-100 ps Rb reference floor).
    # Detrending the chA timestamps over the capture removes the
    # constant DO frequency offset; the residual is the DO's freerun
    # phase noise vs the Rb (DO_noise ⊕ Rb_noise ≈ DO_noise above the
    # ~100 ps floor).
    secs_a = np.array([r[0] for r in chA_recs], dtype=float)
    phase_a = np.array([r[1] for r in chA_recs], dtype=float)
    t_rel = secs_a - secs_a[0]
    # IMPORTANT: subtract the nominal 1 PPS cadence BEFORE detrend.
    # chA timestamps grow by ~1.0 s per sample (slope ≈ 1e9 ns/s).
    # If we polyfit float64 chA-in-ns directly, the 1e12-ns dynamic
    # range pollutes the residual with ~1 ns of numerical noise — we
    # saw this on the 4th MadHat run as non-monotonic ADEV/TDEV at
    # certain τ.  Subtract the nominal 1.0-s/sample component first
    # so polyfit operates on the small (µs-magnitude) deviation only.
    deviation_s = (phase_a - phase_a[0]) - t_rel  # seconds; ~µs scale
    phase_ns = deviation_s * 1e9  # ns; ~1e3–1e4 ns scale, precise
    dts = np.diff(t_rel)
    print(f"\n  chA sample spacing: min={dts.min():.3f}s median={float(np.median(dts)):.3f}s "
          f"max={dts.max():.3f}s  ({int((dts == 1.0).sum())}/{len(dts)} are exactly 1.0s)")

    # Detrend linear (= subtract constant frequency offset)
    residual, slope_per_s, intercept = detrend_linear(t_rel, phase_ns)
    freq_offset_ppb = slope_per_s  # ns/s == ppb (1 ns/s drift = 1 ppb)
    rms_ns = float(np.sqrt(np.mean(residual ** 2)))
    peak_ns = float(np.max(np.abs(residual)))

    # ASD at standard freqs + curve
    asd_at_targets, freqs_arr, asd_arr = compute_asd(t_rel, residual, _PSD_FREQS_HZ)
    asd_at_0p1 = asd_at_targets.get(0.1)
    asd_at_0p01 = asd_at_targets.get(0.01)
    psd_slope = loglog_slope(freqs_arr, asd_arr, f_lo=0.01, f_hi=0.5)
    noise_type = classify_noise_type(psd_slope) if not math.isnan(psd_slope) else "unknown"

    # ADEV / TDEV / MDEV
    adev_map, tdev_map, mdev_map = compute_allantools_metrics(residual, 1.0, _TAUS_S)

    psd_curve = [[round(f, 4),
                  None if asd_at_targets.get(f) is None else round(asd_at_targets[f], 4)]
                 for f in _PSD_FREQS_HZ]

    print(f"\n=== DO freerun characterization (chA vs TICC Rb reference) ===")
    print(f"  duration:        {len(chA_recs)} s = {len(chA_recs)/60:.1f} min")
    print(f"  freq offset:     {freq_offset_ppb:+.3f} ppb")
    print(f"  residual RMS:    {rms_ns:.3f} ns")
    print(f"  residual peak:   {peak_ns:.3f} ns")
    print(f"  ASD @ 0.01 Hz:   {asd_at_0p01:.4f} ns/√Hz" if asd_at_0p01 else "  ASD @ 0.01 Hz:   (not resolved)")
    print(f"  ASD @ 0.1 Hz:    {asd_at_0p1:.4f} ns/√Hz" if asd_at_0p1 else "  ASD @ 0.1 Hz:    (not resolved)")
    print(f"  PSD slope:       {psd_slope:+.3f}  → {noise_type}")
    if adev_map:
        print(f"\n  τ      ADEV          TDEV (ns)   MDEV")
        for τ in _TAUS_S:
            a = adev_map.get(τ)
            td = tdev_map.get(τ)
            md = mdev_map.get(τ)
            if a is None:
                continue
            print(f"  {τ:>4}s  {a:.3e}    {td:>9.3f}   {md:.3e}")

    # Merge into JSON
    existing = {}
    if out_path.exists():
        try:
            existing = json.loads(out_path.read_text())
        except Exception:
            existing = {}

    char_section = existing.setdefault('characterization', {})
    char_section.setdefault('host', args.host or '')
    char_section.setdefault('do_label', do_label)
    char_section['captured'] = datetime.now(tz=timezone.utc).isoformat()
    char_section['duration_s'] = len(chA_recs)
    char_section['n_samples'] = len(chA_recs)
    char_section['parked_dac_code'] = code
    char_section['parked_dac_gain'] = args.gain
    char_section['freq_offset_ppb_at_parked_code'] = freq_offset_ppb

    sources = char_section.setdefault('sources', {})
    sources['DO PPS (chA vs TICC Rb)'] = {
        'units': 'ns',
        'rms': round(rms_ns, 4),
        'peak': round(peak_ns, 4),
        'asd_at_0.1Hz': None if asd_at_0p1 is None else round(asd_at_0p1, 4),
        'asd_at_0.01Hz': None if asd_at_0p01 is None else round(asd_at_0p01, 4),
        'slope': None if math.isnan(psd_slope) else round(psd_slope, 3),
        'noise_type': noise_type,
        'psd_curve': psd_curve,
        'adev_by_tau_s': {str(τ): round(v, 6) for τ, v in (adev_map or {}).items()},
        'tdev_ns_by_tau_s': {str(τ): round(v, 4) for τ, v in (tdev_map or {}).items()},
        'mdev_by_tau_s': {str(τ): round(v, 6) for τ, v in (mdev_map or {}).items()},
    }
    char_section['notes'] = (
        "Source is chA alone, the DO PPS edge timestamp on the TICC's "
        "internal Rb-disciplined timebase (TDEV(1s) <100 ps reference). "
        "Linear-detrended over the capture removes constant DO frequency "
        "offset; residual is the DO's freerun phase noise vs Rb. "
        "PSD curve is one-sided amplitude spectral density (ns/√Hz) from "
        "1 Hz TICC samples; resolves 1/duration to 0.5 Hz. PSD slopes: "
        "~0=white_phase, -1=flicker_phase, -2=white_FM, -3=flicker_FM, "
        "<-3=random_walk_FM. Higher-band L(f) requires a phase-noise "
        "analyzer. Raw CSV (when --raw-csv set) saves chA + chB "
        "separately for re-analysis of either chain."
    )
    existing['updated'] = datetime.now(tz=timezone.utc).isoformat()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(existing, indent=2))
    print(f"\nSaved characterization → {out_path}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
