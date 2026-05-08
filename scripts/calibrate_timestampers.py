#!/usr/bin/env python3
"""Calibrate per-timestamper static biases against an external truth reference.

Each timestamper measuring our DO PPS has an inherent static offset
introduced by its physical chain — cable length from the DO output
to the timestamper's input, internal latency between edge capture and
GPS-time-anchored reading, reference-clock offset to "true GPS time",
and so on.  The DOFreqEst measurement ladder
(docs/dofreq-est-measurement-ladder.md) needs each measurement to be
unbiased before fusion; otherwise x[2] settles to a variance-weighted
compromise between biased reads, P[2,2] doesn't reflect σ-vs-truth,
and ablation runs aren't directly comparable.

This tool computes those biases from a calibration window of raw-stream
CSV logs and writes them to state/dos/<do_uid>.json.  The engine reads
that file at startup and subtracts the bias from each timestamper's
measurement before injecting it into the EKF.

Truth definition: an external GPS reference (typically otcBob1 PPS in
the PePPAR-Fix lab) on a TICC channel paired with the DO PPS.  Whatever
that reference's own offset to "true GPS time" is becomes baked into
the calibration — the result is "DO aligned to *this reference*", not
to a hypothetical perfect GPS time.  Same calibration philosophy as
any GPSDO with a chosen master reference.

Calibration is static.  If you change the reference, re-cable, swap a
cable, or move the DO output to a different connector, recalibrate.
The JSON keeps a `calibrated_at` timestamp for record-keeping but
does NOT auto-expire (per Bob's 2026-05-06 design call).  Time-varying
biases would graduate to additional EKF states, not recalibration.
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics
from bisect import bisect_left
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path


def _parse_ticc_csv(path):
    """Yield (recv_mono, ts, channel, ref_sec, ref_ps) per row.

    Tolerant of small format variations.  Discards rows that don't
    parse.
    """
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                ts_str = r.get("ts_iso") or r.get("host_timestamp")
                ts = datetime.fromisoformat(ts_str.replace("Z", "+00:00"))
                recv_mono = float(r.get("recv_mono") or r.get("host_monotonic"))
                ref_sec = int(r["ref_sec"])
                ref_ps = int(r["ref_ps"])
                channel = r["channel"]
                rows.append((recv_mono, ts, channel, ref_sec, ref_ps))
            except (KeyError, ValueError, AttributeError):
                continue
    return rows


def _pair_chA_chB(rows):
    """Return [(recv_mono, diff_ns), ...] for paired chA/chB rows."""
    by_sec = defaultdict(dict)
    for recv_mono, ts, ch, sec, ps in rows:
        by_sec[sec][ch] = (recv_mono, ps)
    pairs = []
    for sec in sorted(by_sec):
        if "chA" in by_sec[sec] and "chB" in by_sec[sec]:
            ra, psa = by_sec[sec]["chA"]
            rb, psb = by_sec[sec]["chB"]
            diff_ns = (psa - psb) * 1e-3
            if abs(diff_ns) < 1e6:
                pairs.append((max(ra, rb), diff_ns))
    return pairs


def _parse_extint_csv(path):
    """Yield (recv_mono, phase_residual_ns, acc_est_ns) per row."""
    out = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                recv_mono = float(r["host_monotonic"])
                phase = int(r["phase_residual_ns"])
                acc = int(r["acc_est_ns"])
                out.append((recv_mono, phase, acc))
            except (KeyError, ValueError):
                continue
    return out


def _match_to_truth(recv_mono, truth_recv_mono, truth_diff,
                    match_window_s=0.5):
    """Return the truth diff_ns for the row recv_mono is closest to,
    or None if no truth row is within match_window_s."""
    i = bisect_left(truth_recv_mono, recv_mono)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(truth_recv_mono):
            dt = abs(truth_recv_mono[j] - recv_mono)
            if dt < match_window_s and (best is None or dt < best[0]):
                best = (dt, truth_diff[j])
    return best[1] if best else None


def _summarize(values):
    if not values:
        return None
    return {
        "median_ns": float(statistics.median(values)),
        "mean_ns": float(statistics.mean(values)),
        "stdev_ns": float(statistics.stdev(values)) if len(values) > 1 else 0.0,
        "n_samples": len(values),
        "min_ns": float(min(values)),
        "max_ns": float(max(values)),
    }


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--truth-csv", required=True,
                   help="External-reference TICC csv (chA = DO PPS, "
                        "chB = trusted GPS reference like otcBob1).")
    p.add_argument("--ticc-csv", default=None,
                   help="Engine's --ticc-log csv (Arm 4 source).")
    p.add_argument("--extint-csv", default=None,
                   help="Engine's --extint-log csv (Arm 3 source).")
    p.add_argument("--output", required=True,
                   help="state/dos/<do_uid>.json path to write/update.")
    p.add_argument("--do-uid", required=True,
                   help="DO unique ID (e.g., 'ocxo-piface').")
    p.add_argument("--do-type", default="vcocxo",
                   help="DO type label for the JSON (e.g., vcocxo, ocxo, phc).")
    p.add_argument("--window-minutes", type=int, default=30,
                   help="Use the last N minutes of data for the calibration "
                        "window (default 30).")
    p.add_argument("--match-window-s", type=float, default=0.5,
                   help="Maximum host_monotonic gap (s) when matching engine "
                        "measurements to truth pairs (default 0.5).")
    p.add_argument("--ticc-name", default="ticc_diff:default",
                   help="Key under which the TICC bias is stored in the JSON.")
    p.add_argument("--extint-name", default="extint:default",
                   help="Key under which the EXTINT/TIM-TM2 bias is stored.")
    p.add_argument("--ticc-notes", default="DO PPS at TICC chA; F9T PPS at chB",
                   help="Free-form notes for the TICC entry.")
    p.add_argument("--extint-notes", default="DO PPS → F9T EXTINT pin",
                   help="Free-form notes for the EXTINT entry.")
    args = p.parse_args()

    truth_pairs = _pair_chA_chB(_parse_ticc_csv(args.truth_csv))
    if not truth_pairs:
        raise SystemExit("No paired truth measurements — empty or bad csv?")
    last_recv_mono = max(p[0] for p in truth_pairs)
    cutoff = last_recv_mono - args.window_minutes * 60
    truth_pairs = [p for p in truth_pairs if p[0] >= cutoff]
    print(f"# Truth window: {len(truth_pairs)} pairs over "
          f"last {args.window_minutes} min")
    if len(truth_pairs) < 60:
        print(f"# WARNING: thin truth sample ({len(truth_pairs)}). "
              f"Calibration may not be robust.")

    truth_recv_mono = [p[0] for p in truth_pairs]
    truth_diff = [p[1] for p in truth_pairs]
    print(f"# Truth diff (DO − reference) summary: "
          f"median={statistics.median(truth_diff):+.3f} ns, "
          f"std={statistics.stdev(truth_diff):.3f} ns")

    timestampers = {}

    if args.ticc_csv:
        # Engine's pps_err_ticc_ns = -(chA - chB).
        # Truth's chA-chB = (DO − reference) ≈ x[2]_truth.
        # We want: pps_err_ticc_ns − bias_ticc = -x[2]_truth
        #          = -(truth chA-chB)
        # ⇒ bias_ticc = pps_err_ticc_ns + truth_diff
        #             = -(arm4_chA-chB) + truth_diff
        ticc_pairs = _pair_chA_chB(_parse_ticc_csv(args.ticc_csv))
        ticc_pairs = [p for p in ticc_pairs if p[0] >= cutoff]
        biases = []
        for recv_mono, arm4_chA_chB in ticc_pairs:
            tdiff = _match_to_truth(recv_mono, truth_recv_mono, truth_diff,
                                    args.match_window_s)
            if tdiff is None:
                continue
            biases.append(-arm4_chA_chB + tdiff)
        s = _summarize(biases)
        if s is not None:
            timestampers[args.ticc_name] = {
                "type": "ticc_chA_minus_chB",
                "bias_ns": s["median_ns"],
                "sigma_ns_about_bias": s["stdev_ns"],
                "n_samples": s["n_samples"],
                "calibrated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                "calibration_window_s": args.window_minutes * 60,
                "truth_csv": str(args.truth_csv),
                "source_csv": str(args.ticc_csv),
                "raw_summary": s,
                "notes": args.ticc_notes,
            }
            print(f"# {args.ticc_name}: bias={s['median_ns']:+.3f} ns "
                  f"σ_about_bias={s['stdev_ns']:.3f} ns n={s['n_samples']}")

    if args.extint_csv:
        # Arm 3 directly observes x[2] in F9T's GPS-time frame:
        #   extint_phase_ns ≈ x[2]_F9T = (DO − F9T_GPS_time)
        # We want: extint_phase_ns − bias_extint = x[2]_truth = truth_diff
        # ⇒ bias_extint = extint_phase_ns − truth_diff
        extint_data = _parse_extint_csv(args.extint_csv)
        extint_data = [t for t in extint_data if t[0] >= cutoff]
        biases = []
        for recv_mono, phase, _acc in extint_data:
            tdiff = _match_to_truth(recv_mono, truth_recv_mono, truth_diff,
                                    args.match_window_s)
            if tdiff is None:
                continue
            biases.append(phase - tdiff)
        s = _summarize(biases)
        if s is not None:
            timestampers[args.extint_name] = {
                "type": "tim_tm2",
                "bias_ns": s["median_ns"],
                "sigma_ns_about_bias": s["stdev_ns"],
                "n_samples": s["n_samples"],
                "calibrated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                "calibration_window_s": args.window_minutes * 60,
                "truth_csv": str(args.truth_csv),
                "source_csv": str(args.extint_csv),
                "raw_summary": s,
                "notes": args.extint_notes,
            }
            print(f"# {args.extint_name}: bias={s['median_ns']:+.3f} ns "
                  f"σ_about_bias={s['stdev_ns']:.3f} ns n={s['n_samples']}")

    # Merge into existing JSON if present.
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if out_path.exists():
        existing = json.loads(out_path.read_text())
    else:
        existing = {"do_uid": args.do_uid, "type": args.do_type,
                    "timestampers": {}}
    existing.setdefault("do_uid", args.do_uid)
    existing.setdefault("type", args.do_type)
    existing.setdefault("timestampers", {})
    existing["timestampers"].update(timestampers)
    out_path.write_text(json.dumps(existing, indent=2) + "\n")
    print(f"# Wrote {out_path}")


if __name__ == "__main__":
    main()
