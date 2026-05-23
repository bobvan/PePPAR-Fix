#!/usr/bin/env python3
"""Phase A validation: engine-facing TdcpEstimator vs prototype fixture.

Drives `scripts/peppar_fix/tdcp_estimator.py:TdcpEstimator` (the
engine-shaped class) against the same UBX + RINEX NAV fixture the
read-only prototype used in `tdcp-findings-2026-05-22.md`, computes
TDEV(τ) on the resulting df/f series, and asserts the headline 15 ps
floor reproduces.

This is the deterministic regression check the design doc named as
the Phase A → Phase B gate, run against the only 1 Hz fixture
available today (the prototype's archived TimeHat 2026-03-23
capture).  Once the lab hosts have a few hours of `--ubx-out`
output, switch to those — multi-host TDEV is the real validation
gate per docs/tdcp-servo-integration.md.

Not a unit test (depends on absolute paths to fixture files).  Run
it manually:

    /home/bob/git/PePPAR-Fix/venv/bin/python prototypes/tdcp_validation.py

Exit 0 ⇒ TdcpEstimator reproduces the prototype's TDEV(1s) within
tolerance.  Exit 1 ⇒ regression.
"""

from __future__ import annotations

import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent
sys.path.insert(0, str(_REPO / "scripts"))
sys.path.insert(0, str(_HERE))

# Reuse the prototype's RINEX NAV parser + UBX iter so we feed the
# engine class the same data the prototype consumed.
from tdcp_proto import (  # noqa: E402
    _SimpleEph, parse_rinex_nav, _iter_ubx_epochs,
)
from peppar_fix.tdcp_estimator import TdcpEstimator, GPS_EPOCH  # noqa: E402


_UBX_FIXTURE = Path(
    "/home/bob/gt/lab-archive/timehat-old-clone-2026-04-08/qerr-ticc-run/"
    "peppar_20260323T011546_raw.ubx"
)
_NAV_FIXTURE = Path("/tmp/tdcp/BRDC_082.rnx")

# Approximate lab ECEF (CHOKE1 era; sub-10 m is fine for TDCP).
_RX_ECEF = np.array([157470.0, -4756189.5, 4232768.0])

# Expected TDEV(1s) bound.  Prototype hit 15 ps; allow 5× headroom
# for engine-shape obs reformatting + statistical variation.
_TDEV_1S_MAX_PS = 75.0


def main() -> int:
    if not _UBX_FIXTURE.exists():
        print(f"FAIL: UBX fixture not found: {_UBX_FIXTURE}")
        return 1
    if not _NAV_FIXTURE.exists():
        print(f"FAIL: NAV fixture not found: {_NAV_FIXTURE}")
        print("Hint: download via curl -L -o /tmp/tdcp/BRDC_082.rnx.gz "
              "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2026/082/"
              "BRDC00IGS_R_20260820000_01D_MN.rnx.gz && gunzip /tmp/tdcp/BRDC_082.rnx.gz")
        return 1

    # Load broadcast nav for GPS + GAL (prototype's combo).
    eph_dict: dict = {}
    for prefix in ("G", "E"):
        sub = parse_rinex_nav(_NAV_FIXTURE, sysprefix=prefix)
        eph_dict.update(sub)
    eph = _SimpleEph(eph_dict)
    print(f"Loaded {len(eph_dict)} SVs of broadcast ephemeris")

    # Drive the engine-facing TdcpEstimator with the same obs the
    # prototype consumed.  The prototype's iter emits obs dicts with
    # 'sv', 'phi1_cyc', 'wl_f1', 'cno', 'gps_time', 'lli' — exactly
    # what TdcpEstimator wants (it falls back to lock_duration_ms=0
    # when missing, which is harmless because slip detection uses
    # lli alone in this path).
    est = TdcpEstimator(eph, _RX_ECEF)

    results: list = []
    n_epochs = 0
    for t_dt, obs_list in _iter_ubx_epochs(_UBX_FIXTURE):
        n_epochs += 1
        # Filter to GPS + GAL (BDS BDT/GPST handling is fragile per
        # the design doc).
        obs_list = [o for o in obs_list if o["sv"][0] in "GE"]
        r = est.update(obs_list, t_dt)
        results.append(r)

    print(f"Processed {n_epochs} epochs")
    valid = [r for r in results if not np.isnan(r.df_f)]
    print(f"Valid solutions: {len(valid)}")
    if len(valid) < 30:
        print("FAIL: too few valid solutions for meaningful TDEV")
        return 1
    n_sv_used_median = int(np.median([r.n_used for r in valid]))
    median_mad_mm = float(np.median([r.mad_r_m for r in valid])) * 1000
    print(f"Median SVs used per epoch: {n_sv_used_median}")
    print(f"Median per-epoch MAD: {median_mad_mm:.2f} mm")

    # Compute TDEV(1s) via allantools.
    try:
        import allantools
    except ImportError:
        print("FAIL: allantools not installed; cannot validate TDEV")
        return 1

    df_f = np.array([r.df_f for r in valid])
    df_f = df_f - df_f.mean()  # detrend DC offset
    rate = 1.0  # 1 Hz
    tau_out, tdev, _, _ = allantools.tdev(
        df_f, rate=rate, data_type="freq", taus=[1, 2, 5, 10, 30]
    )
    tdev_1s_ps = float(tdev[0]) * 1e12
    print()
    print(f"  {'τ (s)':>8}  {'TDEV (ps)':>12}")
    for τ, td in zip(tau_out, tdev):
        print(f"  {τ:>8.0f}  {td * 1e12:>12.1f}")
    print()

    if tdev_1s_ps > _TDEV_1S_MAX_PS:
        print(f"FAIL: TDEV(1s) = {tdev_1s_ps:.1f} ps > {_TDEV_1S_MAX_PS} ps bound")
        return 1
    print(f"PASS: TDEV(1s) = {tdev_1s_ps:.1f} ps "
          f"(prototype was 15.2 ps; bound was {_TDEV_1S_MAX_PS} ps)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
