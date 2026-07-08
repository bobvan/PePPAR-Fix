#!/usr/bin/env python3
"""Free-run characterization of a GNSSDO+ (SXT-D) OCXO from a TICC chA CSV.

Reads a captured TICC log (chA = GNSSDO+ PPS OUT vs Rb-ref timebase, from
gnssdo_freerun_hold.py + ticc_capture.py), selects the longest clean segment
(ref_sec / host-time gaps split it), **de-glitches the GNSSDO+ PPS
cycle-slips** (integer 10 MHz-cycle steps from the hardware's PPS-to-DO-edge
alignment — see freerun_analysis.deglitch_cycle_slips), and derives the DO's
Kalman-Q scalars via the canonical pipeline (analyze_samples +
_sigma_do_freq_ppb_from_adev + derive_coast_params).

Prints the char and the derived [freerun_noise]; with --write-toml, appends
[freerun_noise] to the DO's state/dos/<uid>.toml IF it beats the class default
(refuses to write a looser-than-default Q).

Example:
    tools/char_gnssdo_freerun.py gnssdo-freerun-ticc-2026-07-06.csv \\
        --do-uid gnssdo-sxtd-madhat --write-toml state/dos
"""
import argparse
import importlib.util
import os
import sys
from datetime import datetime, timezone

import numpy as np

_SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "scripts")
sys.path.insert(0, _SCRIPTS)

from peppar_fix.freerun_analysis import (  # noqa: E402
    analyze_samples, deglitch_cycle_slips)
from peppar_fix.do_state import _sigma_do_freq_ppb_from_adev  # noqa: E402
from peppar_fix import do_schema  # noqa: E402

_mig_spec = importlib.util.spec_from_file_location(
    "migrate_do_state", os.path.join(_SCRIPTS, "migrate_do_state.py"))
_mig = importlib.util.module_from_spec(_mig_spec)
_mig_spec.loader.exec_module(_mig)
derive_coast_params = _mig.derive_coast_params


def _read_chA(path, skip_before=None, skip_after=None):
    """Return list of (ts, ref_sec, ref_ps) for chA rows."""
    rows = []
    for line in open(path):
        if line[0] == "#" or line.startswith("ts_iso") or line.startswith("host_"):
            continue
        p = line.rstrip().split(",")
        if len(p) < 4 or p[1] != "chA":
            continue
        ts = datetime.fromisoformat(p[0].replace("Z", "+00:00"))
        if skip_before and ts < skip_before:
            continue
        if skip_after and ts >= skip_after:
            break
        rows.append((ts, int(p[2]), int(p[3])))
    return rows


def _longest_clean_segment(rows, gap_s=5.0):
    """Split at ref_sec/host-time gaps > gap_s; return the longest run."""
    if not rows:
        return []
    segs = [[rows[0]]]
    for prev, cur in zip(rows, rows[1:]):
        dsec = cur[1] - prev[1]
        dts = (cur[0] - prev[0]).total_seconds()
        if dsec <= 0 or dsec > gap_s or dts > gap_s:
            segs.append([])
        segs[-1].append(cur)
    return max(segs, key=len)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="TICC chA capture CSV")
    ap.add_argument("--skip-before", default=None,
                    help="ISO UTC; drop rows before this (settle/transient)")
    ap.add_argument("--skip-after", default=None,
                    help="ISO UTC; drop rows at/after this (e.g. free-run end)")
    ap.add_argument("--gap-s", type=float, default=5.0,
                    help="ref_sec/host gap (s) that splits a segment (default 5)")
    ap.add_argument("--do-uid", default="gnssdo-sxtd-madhat")
    ap.add_argument("--write-toml", default=None, metavar="DOS_DIR",
                    help="Append [freerun_noise] to <DOS_DIR>/<do-uid>.toml "
                         "if it beats the class default")
    args = ap.parse_args()

    skip = (datetime.fromisoformat(args.skip_before.replace("Z", "+00:00"))
            if args.skip_before else None)
    skip_a = (datetime.fromisoformat(args.skip_after.replace("Z", "+00:00"))
              if args.skip_after else None)
    rows = _read_chA(args.csv, skip, skip_a)
    seg = _longest_clean_segment(rows, args.gap_s)
    print(f"chA rows: {len(rows)}  → longest clean segment: {len(seg)} "
          f"(~{len(seg)/3600:.1f} h)")
    if len(seg) < 300:
        sys.exit("segment too short to characterize")

    samples = [(s, p) for _ts, s, p in seg]
    deglitched, n_slips = deglitch_cycle_slips(samples)
    print(f"GNSSDO+ PPS cycle-slips removed: {n_slips} "
          f"({n_slips/(len(seg)/3600):.1f}/h)")

    r = analyze_samples(deglitched)
    if r is None:
        sys.exit("analysis failed (too few samples)")
    print(f"freq_offset={r['freq_offset_ppb']:+.4f} ppb  rms={r['rms_ns']:.3f} ns  "
          f"noise={r['noise_type']}")
    print("TDEV(ns):", {int(k): round(v, 4)
                        for k, v in sorted(r["tdev_map"].items())})

    src = {"asd_at_0.1Hz": r["asd_at_targets"].get(0.1),
           "adev_by_tau_s": {str(k): v for k, v in r["adev_map"].items()},
           "tdev_ns_by_tau_s": {str(k): v for k, v in r["tdev_map"].items()}}
    phase = src["asd_at_0.1Hz"]
    sig_f, tau_f = _sigma_do_freq_ppb_from_adev(src["adev_by_tau_s"])
    coast = derive_coast_params(src["tdev_ns_by_tau_s"])
    dfl = do_schema.class_defaults("OCXO")
    print("\n[freerun_noise]  (OCXO class-default in parens)")
    print(f"  sigma_do_phase_ns = {phase:.5f}   ({dfl['sigma_do_phase_ns']})")
    print(f"  sigma_do_freq_ppb = {sig_f:.6f}   ({dfl['sigma_do_freq_ppb']})")
    print(f"  coast_tdev_ref_ns = {coast[0]:.4f}  coast_tdev_slope = {coast[1]:.4f}")
    beats = phase < dfl["sigma_do_phase_ns"] and sig_f < dfl["sigma_do_freq_ppb"]
    print(f"  beats class-default: {beats}")

    if args.write_toml:
        if not beats:
            sys.exit("REFUSING to write: Q is looser than the class default "
                     "(check for un-removed slips / a noisy capture)")
        path = os.path.join(args.write_toml, f"{args.do_uid}.toml")
        with open(path, "a") as f:
            f.write(f'\n# Measured {datetime.now(timezone.utc).date()} by '
                    f'char_gnssdo_freerun.py ({n_slips} PPS cycle-slips removed)\n'
                    '[freerun_noise]\nsource = "measured"\n'
                    'measurement_channel = "DO PPS (chA vs TICC Rb)"\n'
                    f'sigma_do_phase_ns = {phase:.5f}\n'
                    f'sigma_do_freq_ppb = {sig_f:.6f}\n'
                    f'coast_tdev_ref_ns = {coast[0]:.4f}\n'
                    f'coast_tdev_slope = {coast[1]:.4f}\n'
                    f'captured = "{datetime.now(timezone.utc).date()}"\n')
        print(f"\nappended [freerun_noise] → {path}")


if __name__ == "__main__":
    main()
