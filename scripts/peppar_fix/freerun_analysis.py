"""Shared analysis for DO freerun characterization.

The PARKING step is actuator-specific (DAC code via I2C, PHC adjfine
via clock_adjtime, ClockMatrix FCW via I2C register), but the CAPTURE
+ ANALYSIS + JSON MERGE are the same.  This module owns the shared
half; the per-actuator scripts own parking + CLI.

Used by:
  - ``scripts/do_freerun_char.py``     — DAC-parked DOs (PiFace,
    clkPoC3, MadHat OCXO-33).
  - ``scripts/do_freerun_char_phc.py`` — PHC-parked DOs (TimeHat i226
    TCXO).
  - (future) ``scripts/do_freerun_char_cm.py`` — ClockMatrix.

Design notes:
- Pure compute functions return data only — no I/O, no logging — so
  they are unit-testable on synthetic input.
- ``collect_phase_series`` does TICC I/O (necessarily); kept here so
  the capture method is identical across actuator scripts.
- ``analyze_samples`` is the high-level reduction (samples → result
  dict); ``merge_characterization_into_json`` writes the result; both
  the DAC and PHC scripts pipe their parking metadata into the same
  JSON merge with no actuator-specific logic in this library.

Reference: TICC's internal Rb-class timebase, not GNSS.  The DO PPS
is read on TICC chA and measured against the TICC's own Rb timebase
(each edge as <ref_sec, ref_ps> in its internal frame).  chB (GNSS
PPS) is **deliberately ignored** — a freerun characterization has
nothing to do with GNSS.  See dayplan ``freerunCharChAOnly`` for the
2026-05-26 → 27 MadHat incident that caught this.
"""

from __future__ import annotations

import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

# Tests need to import this module without dragging in the top-level
# ``ticc`` (which requires a real serial port).  Lazy-import inside
# ``collect_phase_series`` keeps the pure functions importable bare.

# Standard PSD offset frequencies (Hz).  Matches the PiFace
# characterization JSON's psd_curve grid.  Nulls fill where the
# capture duration is too short to resolve.
_PSD_FREQS_HZ = (0.001, 0.002, 0.003, 0.005, 0.007, 0.01, 0.02, 0.03,
                 0.05, 0.07, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5)

# Standard τ values (s) for ADEV/TDEV/MDEV.
_TAUS_S = (1, 2, 5, 10, 30, 100, 300, 1000)


# ── pure helpers ─────────────────────────────────────────────────── #

def _sig(x: float, n: int = 6):
    """Round to n significant figures.

    ADEV/MDEV are dimensionless fractional-frequency values that span
    orders of magnitude (OCXO-class σ_y ≈ 1e-11 at τ=1 s, dropping to
    ~1e-13 at long τ).  Fixed-decimal rounding (``round(v, 6)``) zeroes
    everything below 1e-6 — useless for these.  Sig-fig rounding keeps
    the value meaningful regardless of magnitude.
    """
    if x is None or x == 0 or not math.isfinite(x):
        return x
    return round(x, -int(math.floor(math.log10(abs(x)))) + (n - 1))


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


def detrend_linear(t: np.ndarray, y: np.ndarray):
    """Linear-detrend y vs t.  Returns ``(residual, slope_per_sec, intercept)``."""
    if len(t) < 2:
        return y - y.mean() if len(y) else y, 0.0, 0.0
    slope, intercept = np.polyfit(t, y, 1)
    return y - (slope * t + intercept), slope, intercept


def compute_asd(t: np.ndarray, y: np.ndarray, target_freqs: tuple):
    """One-sided amplitude spectral density (units of y per √Hz).

    Welch-like single-FFT estimator (fixed-duration capture).  Returns
    ``(dict freq → ASD value, freqs_array, asd_array)``.

    Sample rate is derived from the MEDIAN inter-sample interval, not
    ``t[1] − t[0]`` — robust to occasional missed pairs at the start
    of the capture (one pair could lead to a multi-second gap in the
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
        print(f"WARN: non-uniform sample spacing detected — "
              f"min={dts.min():.3f}s median={median_dt:.3f}s "
              f"max={dts.max():.3f}s. FFT assumes uniform; "
              f"results may have aliasing artifacts.")
    # Hann window to reduce spectral leakage
    win = np.hanning(n)
    yw = (y - y.mean()) * win
    Y = np.fft.rfft(yw)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    win_norm = (win ** 2).sum()
    psd = (np.abs(Y) ** 2) / (fs * win_norm) * 2  # one-sided
    asd = np.sqrt(np.maximum(psd, 0.0))
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
    """Fit log-ASD vs log-freq over ``[f_lo, f_hi]``.

    ASD ∝ f^(slope/2) so the returned PSD slope is 2× the log-log
    ASD slope (since PSD = ASD²).
    """
    mask = (freqs >= f_lo) & (freqs <= f_hi) & (asd > 0)
    if mask.sum() < 3:
        return float('nan')
    lf = np.log10(freqs[mask])
    la = np.log10(asd[mask])
    slope_asd, _ = np.polyfit(lf, la, 1)
    return 2.0 * slope_asd


def compute_allantools_metrics(y_ns: np.ndarray, rate_hz: float, taus_s):
    """ADEV / TDEV / MDEV at the requested τ values via allantools.

    Returns ``(adev_map, tdev_map, mdev_map)`` — each a dict τ →
    metric, or ``(None, None, None)`` if allantools isn't available.
    """
    try:
        import allantools
    except ImportError:
        return None, None, None
    out_adev: dict = {}
    out_tdev: dict = {}
    out_mdev: dict = {}
    valid_taus = [t for t in taus_s if t < len(y_ns) / rate_hz / 4]
    if not valid_taus:
        return out_adev, out_tdev, out_mdev
    phase_s = y_ns * 1e-9  # ns → s for allantools (phase-data input)
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
        out_tdev[float(τ)] = float(td) * 1e9  # back to ns
    for τ, m in zip(t_m, mdev_arr):
        out_mdev[float(τ)] = float(m)
    return out_adev, out_tdev, out_mdev


# ── capture (TICC I/O — actuator-agnostic, identical across scripts) ─ #

def collect_phase_series(ticc_port: str, duration_s: float):
    """Listen to TICC; return list of ``(ref_sec, ref_ps_chA)`` for the
    DO PPS edge timestamps in the TICC's internal Rb frame.

    chB (GNSS PPS) is deliberately ignored — see module docstring.

    Returns a list of ``(ref_sec: int, ref_ps: int)`` tuples in raw
    TICC units — both integers, no float-precision loss in the
    capture path.  Downstream analysis converts to ns while
    preserving precision (subtract ``ref_ps[0]`` while still integer,
    then ``* 1e-3`` → ns float).
    """
    # Lazy-import so the pure helpers above remain importable in test
    # environments without serial / pyserial.
    _scripts_dir = Path(__file__).resolve().parent.parent
    if str(_scripts_dir) not in sys.path:
        sys.path.insert(0, str(_scripts_dir))
    from ticc import Ticc  # noqa: E402

    samples: list[tuple[int, int]] = []
    t0 = None
    with Ticc(ticc_port, wait_for_boot=False) as ticc:
        for ch, sec, ps in ticc:
            if t0 is None:
                t0 = time.monotonic()
            elif time.monotonic() - t0 > duration_s:
                break
            if ch == 'chA':
                samples.append((sec, ps))
    return samples


# ── high-level reduction + JSON merge ────────────────────────────── #

def analyze_samples(samples, target_freqs=_PSD_FREQS_HZ, taus_s=_TAUS_S):
    """Reduce raw TICC chA samples to the full analysis dict.

    ``samples`` is a list of ``(ref_sec: int, ref_ps: int)`` tuples
    as returned by :func:`collect_phase_series`.

    Returns a dict (or ``None`` if too few samples) with keys:
        n_samples, t_rel, residual_ns, freq_offset_ppb,
        rms_ns, peak_ns, asd_at_targets, freqs_arr, asd_arr,
        psd_slope, noise_type, psd_curve,
        adev_map, tdev_map, mdev_map,
        sample_spacing_dts.

    The (residual_ns, t_rel) arrays are exposed so callers can pass
    them to alternative analysers without re-running the precision-
    preserving int→ns conversion.
    """
    if len(samples) < 60:
        return None

    # Precision-preserving conversion to ns: subtract ref_ps[0] while
    # still integer, then convert.  See CLAUDE.md "watch for floating
    # point error with TICC timestamps".
    secs = np.array([s for s, _ in samples], dtype=float)
    ps0 = samples[0][1]
    phase_ns = np.array(
        [(p - ps0) for _, p in samples], dtype=np.int64
    ).astype(float) * 1e-3
    t_rel = secs - secs[0]
    dts = np.diff(t_rel)

    residual, slope_per_s, _ = detrend_linear(t_rel, phase_ns)
    freq_offset_ppb = slope_per_s   # ns/s = ppb (1 ns/s = 1 ppb)
    rms_ns = float(np.sqrt(np.mean(residual ** 2)))
    peak_ns = float(np.max(np.abs(residual)))

    asd_at_targets, freqs_arr, asd_arr = compute_asd(
        t_rel, residual, target_freqs)
    psd_slope = loglog_slope(freqs_arr, asd_arr, f_lo=0.01, f_hi=0.5)
    noise_type = (classify_noise_type(psd_slope)
                  if not math.isnan(psd_slope) else "unknown")

    adev_map, tdev_map, mdev_map = compute_allantools_metrics(
        residual, 1.0, taus_s)

    psd_curve = [[round(f, 4),
                  None if asd_at_targets.get(f) is None
                  else round(asd_at_targets[f], 4)]
                 for f in target_freqs]

    return {
        'n_samples': len(samples),
        't_rel': t_rel,
        'residual_ns': residual,
        'sample_spacing_dts': dts,
        'freq_offset_ppb': freq_offset_ppb,
        'rms_ns': rms_ns,
        'peak_ns': peak_ns,
        'asd_at_targets': asd_at_targets,
        'freqs_arr': freqs_arr,
        'asd_arr': asd_arr,
        'psd_slope': psd_slope,
        'noise_type': noise_type,
        'psd_curve': psd_curve,
        'adev_map': adev_map or {},
        'tdev_map': tdev_map or {},
        'mdev_map': mdev_map or {},
    }


def print_summary(result: dict, taus_s=_TAUS_S) -> None:
    """Pretty-print analysis result to stdout (capture + reduce + this)."""
    n = result['n_samples']
    dts = result['sample_spacing_dts']
    print(f"\n  sample spacing: min={dts.min():.3f}s "
          f"median={float(np.median(dts)):.3f}s max={dts.max():.3f}s  "
          f"({int((dts == 1.0).sum())}/{len(dts)} are exactly 1.0s)")
    print(f"\n=== freerun characterization ===")
    print(f"  duration:        {n} s = {n/60:.1f} min")
    print(f"  freq offset:     {result['freq_offset_ppb']:+.3f} ppb")
    print(f"  residual RMS:    {result['rms_ns']:.3f} ns")
    print(f"  residual peak:   {result['peak_ns']:.3f} ns")
    asd_at = result['asd_at_targets']
    a01 = asd_at.get(0.01)
    a1 = asd_at.get(0.1)
    print(f"  ASD @ 0.01 Hz:   {a01:.4f} ns/√Hz" if a01 is not None
          else "  ASD @ 0.01 Hz:   (not resolved)")
    print(f"  ASD @ 0.1 Hz:    {a1:.4f} ns/√Hz" if a1 is not None
          else "  ASD @ 0.1 Hz:    (not resolved)")
    print(f"  PSD slope:       {result['psd_slope']:+.3f}  → "
          f"{result['noise_type']}")
    adev_map = result.get('adev_map') or {}
    if adev_map:
        tdev_map = result['tdev_map']
        mdev_map = result['mdev_map']
        print(f"\n  τ      ADEV          TDEV (ns)   MDEV")
        for τ in taus_s:
            a = adev_map.get(τ)
            td = tdev_map.get(τ)
            md = mdev_map.get(τ)
            if a is None:
                continue
            print(f"  {τ:>4}s  {a:.3e}    {td:>9.3f}   {md:.3e}")


def merge_characterization_into_json(out_path: Path, result: dict,
                                     parking_metadata: dict,
                                     do_label: str,
                                     host: str = '') -> None:
    """Merge the freerun characterization into a DO state JSON.

    Preserves all top-level keys (slope_cal output, dac_code_by_temperature,
    etc.).  The ``characterization`` section is set in full; ``parking_metadata``
    (actuator-specific fields like ``parked_dac_code``, ``parked_adjfine_ppb``)
    is added flat under ``characterization`` so existing consumers continue
    to work.

    ``host`` is preserved across reruns if already set and not supplied
    explicitly — same for ``do_label``.
    """
    existing: dict = {}
    if out_path.exists():
        try:
            existing = json.loads(out_path.read_text())
        except Exception:
            existing = {}

    # An earlier code path occasionally writes 'characterization': null;
    # treat that as absent (don't try to .setdefault into a None).
    char_section = existing.get('characterization')
    if not isinstance(char_section, dict):
        char_section = {}
        existing['characterization'] = char_section

    char_section.setdefault('host', host or '')
    char_section.setdefault('do_label', do_label)
    char_section['captured'] = datetime.now(tz=timezone.utc).isoformat()
    char_section['duration_s'] = result['n_samples']
    char_section['n_samples'] = result['n_samples']
    # Actuator-specific parking fields go in here verbatim.
    for k, v in parking_metadata.items():
        char_section[k] = v

    sources = char_section.setdefault('sources', {})
    sources['DO PPS (chA vs TICC Rb)'] = {
        'units': 'ns',
        'rms': round(result['rms_ns'], 4),
        'peak': round(result['peak_ns'], 4),
        'asd_at_0.1Hz': (None if result['asd_at_targets'].get(0.1) is None
                         else round(result['asd_at_targets'][0.1], 4)),
        'asd_at_0.01Hz': (None if result['asd_at_targets'].get(0.01) is None
                          else round(result['asd_at_targets'][0.01], 4)),
        'slope': (None if math.isnan(result['psd_slope'])
                  else round(result['psd_slope'], 3)),
        'noise_type': result['noise_type'],
        'psd_curve': result['psd_curve'],
        'adev_by_tau_s': {str(τ): _sig(v)
                          for τ, v in (result['adev_map'] or {}).items()},
        'tdev_ns_by_tau_s': {str(τ): round(v, 4)
                             for τ, v in (result['tdev_map'] or {}).items()},
        'mdev_by_tau_s': {str(τ): _sig(v)
                          for τ, v in (result['mdev_map'] or {}).items()},
    }
    char_section['notes'] = (
        "PSD curve is one-sided amplitude spectral density (ns/√Hz) "
        "from 1 Hz TICC chA-vs-Rb samples; resolves 1/duration to 0.5 Hz. "
        "PSD slopes: ~0=white_phase, -1=flicker_phase, -2=white_FM, "
        "-3=flicker_FM, <-3=random_walk_FM. "
        "Higher-band L(f) requires a phase-noise analyzer."
    )
    existing['updated'] = datetime.now(tz=timezone.utc).isoformat()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(existing, indent=2))


def write_raw_csv(raw_path: Path, samples) -> None:
    """Optional raw chA dump (lets us re-analyze offline without re-capture)."""
    raw_path.parent.mkdir(parents=True, exist_ok=True)
    with open(raw_path, 'w') as f:
        f.write("ref_sec,ref_ps\n")
        for s, p in samples:
            f.write(f"{s},{p}\n")
