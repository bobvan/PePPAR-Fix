#!/usr/bin/env python3
"""Per-host TDCP gate runner.

Drives TdcpEstimator against an arbitrary UBX file and reports
TDEV(τ) at τ ∈ {1, 2, 5, 10, 30 s}.  Mostly mirrors
prototypes/tdcp_validation.py but accepts the UBX path as a CLI
argument so we can iterate over multi-host captures from
`--ubx-out`.  Also returns the per-epoch df_f series so a caller
can do cross-host pair-excursion analysis.

Usage:
    python prototypes/tdcp_validation_host.py <UBX_FILE> [<NAV_FILE>]

NAV file defaults to /tmp/tdcp/BRDC_<DOY>.rnx where <DOY> is
derived from the UBX file's first epoch.
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent
sys.path.insert(0, str(_REPO / "scripts"))
sys.path.insert(0, str(_HERE))

from tdcp_proto import _SimpleEph, parse_rinex_nav, _iter_ubx_epochs  # noqa: E402
from peppar_fix.tdcp_estimator import TdcpEstimator  # noqa: E402

# CHOKE1 ECEF (all lab hosts share via splitter).
_RX_ECEF = np.array([157470.0, -4756189.5, 4232768.0])


def run(ubx_path: Path, nav_path: Path) -> dict:
    eph_dict: dict = {}
    for prefix in ("G", "E"):
        sub = parse_rinex_nav(nav_path, sysprefix=prefix)
        eph_dict.update(sub)
    eph = _SimpleEph(eph_dict)

    est = TdcpEstimator(eph, _RX_ECEF)
    results = []
    n_epochs = 0
    for t_dt, obs_list in _iter_ubx_epochs(ubx_path):
        n_epochs += 1
        obs_list = [o for o in obs_list if o["sv"][0] in "GE"]
        results.append(est.update(obs_list, t_dt))

    valid = [r for r in results if not np.isnan(r.df_f)]
    if len(valid) < 30:
        return {
            "ubx": str(ubx_path),
            "n_epochs": n_epochs,
            "n_valid": len(valid),
            "error": "too few valid solutions",
        }

    df_f = np.array([r.df_f for r in valid])
    t_gps = np.array([(r.t_dt - datetime(1980, 1, 6, tzinfo=timezone.utc)).total_seconds()
                      for r in valid])
    df_f_dc = df_f - df_f.mean()

    import allantools
    tau_out, tdev, _, _ = allantools.tdev(
        df_f_dc, rate=1.0, data_type="freq", taus=[1, 2, 5, 10, 30]
    )
    tdev_ps = {int(τ): float(td) * 1e12 for τ, td in zip(tau_out, tdev)}

    n_sv_used_median = int(np.median([r.n_used for r in valid]))
    mad_mm_median = float(np.median([r.mad_r_m for r in valid])) * 1000

    return {
        "ubx": str(ubx_path),
        "n_epochs": n_epochs,
        "n_valid": len(valid),
        "duration_s": int(t_gps[-1] - t_gps[0]),
        "n_sv_used_median": n_sv_used_median,
        "mad_mm_median": mad_mm_median,
        "tdev_ps": tdev_ps,
        "t_gps": t_gps,
        "df_f": df_f,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("ubx", type=Path)
    ap.add_argument("--nav", type=Path, default=None)
    ap.add_argument("--bound-ps", type=float, default=25.0,
                    help="TDEV(1s) bound for PASS/FAIL")
    args = ap.parse_args()

    nav = args.nav or Path("/tmp/tdcp/BRDC_143.rnx")
    if not args.ubx.exists():
        print(f"FAIL: UBX file missing: {args.ubx}")
        return 1
    if not nav.exists():
        print(f"FAIL: NAV file missing: {nav}")
        return 1

    out = run(args.ubx, nav)
    name = args.ubx.stem
    print(f"\n=== {name} ===")
    if "error" in out:
        print(f"FAIL: {out['error']}  (n_valid={out['n_valid']})")
        return 1
    print(f"epochs:        {out['n_epochs']} parsed, {out['n_valid']} valid")
    print(f"duration:      {out['duration_s']} s")
    print(f"median SVs:    {out['n_sv_used_median']}")
    print(f"median MAD:    {out['mad_mm_median']:.2f} mm")
    print(f"  {'τ (s)':>8}  {'TDEV (ps)':>12}")
    for τ, td_ps in sorted(out["tdev_ps"].items()):
        print(f"  {τ:>8}  {td_ps:>12.1f}")
    tdev_1s = out["tdev_ps"][1]
    if tdev_1s > args.bound_ps:
        print(f"FAIL: TDEV(1s) = {tdev_1s:.1f} ps > {args.bound_ps} ps bound")
        return 1
    print(f"PASS: TDEV(1s) = {tdev_1s:.1f} ps ≤ {args.bound_ps} ps bound")
    return 0


if __name__ == "__main__":
    sys.exit(main())
