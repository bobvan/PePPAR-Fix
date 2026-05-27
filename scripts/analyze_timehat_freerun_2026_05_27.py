"""One-off analyzer: TimeHat freerun chA vs TICC Rb from ticc_read.py CSV.

The 2026-05-27 2 h capture on TimeHat used timelab/scripts/ticc_read.py
(no DAC, since TimeHat is an i226 PHC host).  That tool writes both
chA + chB rows with channel column.  We filter chA and run the same
detrend/PSD/Allan analysis as do_freerun_char.py so the resulting
metrics are apples-to-apples with the DAC-host runs.

Outputs to stdout + writes a JSON next to the CSV.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / 'scripts'))

from do_freerun_char import (  # noqa: E402
    _PSD_FREQS_HZ,
    _TAUS_S,
    classify_noise_type,
    compute_allantools_metrics,
    compute_asd,
    detrend_linear,
    loglog_slope,
)


CSV = _REPO / 'data' / 'freerun-day0527-2h-timehat-raw.csv'
OUT_JSON = _REPO / 'data' / 'freerun-day0527-2h-timehat.json'


def load_chA(csv_path: Path):
    secs: list[int] = []
    pss: list[int] = []
    with open(csv_path) as f:
        r = csv.DictReader(f)
        for row in r:
            if row['channel'] != 'chA':
                continue
            secs.append(int(row['ref_sec']))
            pss.append(int(row['ref_ps']))
    return secs, pss


def main() -> int:
    secs_list, pss_list = load_chA(CSV)
    if len(secs_list) < 60:
        print(f"too few chA samples: {len(secs_list)}")
        return 1

    secs = np.asarray(secs_list, dtype=float)
    ps0 = pss_list[0]
    phase_ns = np.asarray(
        [p - ps0 for p in pss_list], dtype=np.int64
    ).astype(float) * 1e-3
    t_rel = secs - secs[0]
    dts = np.diff(t_rel)
    print(f"chA samples:       {len(secs_list)}")
    print(f"sample spacing:    min={dts.min():.3f}s "
          f"median={float(np.median(dts)):.3f}s max={dts.max():.3f}s  "
          f"({int((dts == 1.0).sum())}/{len(dts)} exactly 1.0s)")

    residual, slope_per_s, _ = detrend_linear(t_rel, phase_ns)
    freq_offset_ppb = slope_per_s
    rms_ns = float(np.sqrt(np.mean(residual ** 2)))
    peak_ns = float(np.max(np.abs(residual)))

    asd_at_targets, freqs_arr, asd_arr = compute_asd(t_rel, residual, _PSD_FREQS_HZ)
    asd_at_0p01 = asd_at_targets.get(0.01)
    asd_at_0p1 = asd_at_targets.get(0.1)
    psd_slope = loglog_slope(freqs_arr, asd_arr, f_lo=0.01, f_hi=0.5)
    noise_type = classify_noise_type(psd_slope) if not math.isnan(psd_slope) else "unknown"

    adev_map, tdev_map, mdev_map = compute_allantools_metrics(residual, 1.0, _TAUS_S)

    print(f"\n=== freerun characterization (chA vs TICC Rb) ===")
    print(f"  duration:        {len(secs_list)} s = {len(secs_list)/60:.1f} min")
    print(f"  freq offset:     {freq_offset_ppb:+.3f} ppb")
    print(f"  residual RMS:    {rms_ns:.3f} ns")
    print(f"  residual peak:   {peak_ns:.3f} ns")
    if asd_at_0p01 is not None:
        print(f"  ASD @ 0.01 Hz:   {asd_at_0p01:.4f} ns/√Hz")
    if asd_at_0p1 is not None:
        print(f"  ASD @ 0.1 Hz:    {asd_at_0p1:.4f} ns/√Hz")
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

    psd_curve = [[round(f, 4),
                  None if asd_at_targets.get(f) is None else round(asd_at_targets[f], 4)]
                 for f in _PSD_FREQS_HZ]

    out = {
        'characterization': {
            'host': 'TimeHat',
            'do_label': 'timehat-i226-tcxo',
            'captured': datetime.now(tz=timezone.utc).isoformat(),
            'source': 'chA vs TICC Rb (ticc_read.py + offline analyzer)',
            'duration_s': len(secs_list),
            'n_samples': len(secs_list),
            'freq_offset_ppb': freq_offset_ppb,
            'residual_rms_ns': rms_ns,
            'residual_peak_ns': peak_ns,
            'asd_at_0p01_Hz_ns_per_rtHz': asd_at_0p01,
            'asd_at_0p1_Hz_ns_per_rtHz': asd_at_0p1,
            'psd_slope': None if math.isnan(psd_slope) else psd_slope,
            'noise_type': noise_type,
            'psd_curve_freq_asd_ns_per_rtHz': psd_curve,
            'adev': {f"{int(t)}": v for t, v in (adev_map or {}).items()},
            'tdev_ns': {f"{int(t)}": v for t, v in (tdev_map or {}).items()},
            'mdev': {f"{int(t)}": v for t, v in (mdev_map or {}).items()},
        }
    }
    OUT_JSON.write_text(json.dumps(out, indent=2))
    print(f"\nWrote → {OUT_JSON}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
