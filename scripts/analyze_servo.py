#!/usr/bin/env python3
"""
analyze_servo.py — Analyze PHC servo performance from TICC timestamps.

Reads TICC CSV (captured during servo run) and computes stability metrics.
TICC #3 on TimeHat: chA = disciplined PHC PPS OUT, chB = F9T-3RD raw GPS PPS.
The chA−chB difference measures how well the servo disciplines the PHC.

Can also capture live TICC data (--capture mode) for a specified duration.

Usage:
    # Analyze existing TICC CSV:
    python analyze_servo.py --ticc data/servo_ticc.csv --out data/servo

    # Capture + analyze in one step:
    python analyze_servo.py --capture --port /dev/ticc --duration 300 --out data/servo

    # Capture + analyze with servo log overlay:
    python analyze_servo.py --capture --port /dev/ticc --duration 300 \
        --servo-log /tmp/servo_test.csv --out data/servo

Outputs:
    _report.txt      — statistics, ADEV/TDEV at key taus
    _timeseries.png  — chA−chB time series
    _tdev.png        — TDEV(τ) plot
    _adev.png        — ADEV(τ) plot
"""

import argparse
import csv
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import allantools
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Import the shared TICC reader (preserves ps precision)
sys.path.insert(0, str(Path(__file__).parent))
from ticc import Ticc


# ── TICC capture ─────────────────────────────────────────────────────── #

def capture_ticc(port: str, duration: int, out_path: Path, baud: int = 115200):
    """Capture TICC timestamps to CSV for specified duration."""
    print(f"Capturing TICC from {port} for {duration}s → {out_path}")
    n_edges = 0
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["host_timestamp", "ref_sec", "ref_ps", "channel"])
        with Ticc(port, baud) as ticc:
            start = time.monotonic()
            for channel, ref_sec, ref_ps in ticc:
                now = datetime.now(tz=timezone.utc)
                w.writerow([now.isoformat(), ref_sec, ref_ps, channel])
                n_edges += 1
                elapsed = time.monotonic() - start
                if elapsed >= duration:
                    break
                if n_edges % 100 == 0:
                    print(f"  {n_edges} edges, {elapsed:.0f}s / {duration}s")
    print(f"  Captured {n_edges} edges in {elapsed:.0f}s")
    return out_path


# ── Load + pair ──────────────────────────────────────────────────────── #

def load_ticc(path: Path) -> pd.DataFrame:
    """
    Load TICC CSV and pair chA/chB edges by integer TICC second.

    Precision: ref_sec (int64) and ref_ps (int64) are kept as integers
    throughout. The difference chA−chB is computed in int64 picoseconds
    before any float conversion.

    Returns DataFrame with columns:
        integer_sec, chA_ref_sec, chA_ref_ps, chB_ref_sec, chB_ref_ps,
        raw_diff_ps (int64), raw_diff_s (float64), host_sec (int64)
    """
    df = pd.read_csv(path)

    df["ref_sec"] = df["ref_sec"].astype("int64")
    df["ref_ps"] = df["ref_ps"].astype("int64")
    df["integer_sec"] = df["ref_sec"]

    if "host_timestamp" in df.columns:
        host_ts = pd.to_datetime(df["host_timestamp"], utc=True)
        _epoch = pd.Timestamp("1970-01-01", tz="UTC")
        df["host_sec"] = (host_ts - _epoch).dt.total_seconds().astype("int64")

    # Pivot: pair chA and chB by integer second
    piv_sec = (df.pivot_table(index="integer_sec", columns="channel",
                               values="ref_sec", aggfunc="first")
                 .rename(columns={"chA": "chA_ref_sec", "chB": "chB_ref_sec"}))
    piv_ps = (df.pivot_table(index="integer_sec", columns="channel",
                              values="ref_ps", aggfunc="first")
                .rename(columns={"chA": "chA_ref_ps", "chB": "chB_ref_ps"}))
    piv = (pd.concat([piv_sec, piv_ps], axis=1)
             .dropna()
             .reset_index()
             .sort_values("integer_sec")
             .reset_index(drop=True))
    for col in ("chA_ref_sec", "chB_ref_sec", "chA_ref_ps", "chB_ref_ps"):
        piv[col] = piv[col].astype("int64")

    # Difference in int64 picoseconds (no float precision loss)
    piv["raw_diff_ps"] = ((piv["chA_ref_sec"] - piv["chB_ref_sec"])
                          * 1_000_000_000_000
                          + piv["chA_ref_ps"] - piv["chB_ref_ps"])
    piv["raw_diff_s"] = piv["raw_diff_ps"].astype(float) * 1e-12

    # Map host_sec for UTC time axis
    if "host_sec" in df.columns:
        hs_map = df.groupby("integer_sec")["host_sec"].first()
        piv["host_sec"] = piv["integer_sec"].map(hs_map)

    return piv


# ── Stability metrics ────────────────────────────────────────────────── #

def compute_stability(phase_s: np.ndarray, taus="decade") -> dict:
    """
    Compute ADEV and TDEV from a 1-Hz phase time series (seconds).
    phase_s = cumulative phase error, one sample per second.
    """
    phase_s = phase_s[~np.isnan(phase_s)]
    if len(phase_s) < 8:
        return {}
    taus_a, adev, _, _ = allantools.adev(
        phase_s, rate=1.0, data_type="phase", taus=taus)
    taus_t, tdev, _, _ = allantools.tdev(
        phase_s, rate=1.0, data_type="phase", taus=taus)
    return {"taus_adev": taus_a, "adev": adev,
            "taus_tdev": taus_t, "tdev": tdev}


def individual_stability(piv: pd.DataFrame, taus="decade") -> dict:
    """
    Compute per-channel TDEV/ADEV plus the difference.

    Each channel's phase series: x[i] = (ref_ps[i] - ref_ps[0]) * 1e-12 seconds.
    This is the phase residual relative to the TICC's 10 MHz reference.
    The ref_sec part cancels in the subtraction (all same epoch within a channel).

    Returns dict with keys: 'chA', 'chB', 'diff'
    Each value is a stability dict from compute_stability().

    chA = disciplined PHC PPS OUT (what we're evaluating)
    chB = F9T-3RD raw GPS PPS (reference)
    diff = chA - chB (differential measurement)
    """

    def _phase_ps_to_s(ps_arr: np.ndarray) -> np.ndarray:
        """Phase residual from int64 ps array → float64 seconds."""
        return (ps_arr - ps_arr[0]).astype(float) * 1e-12

    result = {}

    # Individual channels: phase = cumulative ps relative to first sample
    for ch, label in [("chA", "chA"), ("chB", "chB")]:
        ps = piv[f"{ch}_ref_ps"].values
        phase_s = _phase_ps_to_s(ps)
        result[label] = compute_stability(phase_s, taus=taus)

    # Difference (already computed)
    result["diff"] = compute_stability(piv["raw_diff_s"].values, taus=taus)

    return result


# ── Report ───────────────────────────────────────────────────────────── #

def write_report(piv: pd.DataFrame, ind_stab: dict, out_stem: Path) -> None:
    lines = []
    a = lines.append

    diff_ns = piv["raw_diff_s"] * 1e9
    n = len(piv)
    dur_s = n  # 1 Hz data

    a("=" * 62)
    a("  PePPAR Fix M5 — PHC Servo TDEV Report")
    a("=" * 62)
    a(f"  TICC pairs : {n}")
    a(f"  Duration   : {dur_s}s ({dur_s/3600:.2f}h)")
    a(f"  chA        : TimeHat PHC PPS OUT (disciplined)")
    a(f"  chB        : F9T-3RD PPS (raw GPS)")
    a(f"  ref        : 10 MHz OCXO (SV1AFN dist amp)")
    a("")

    a("── chA−chB difference (PHC − GPS) ────────────────────────")
    a(f"  Mean : {diff_ns.mean():+.3f} ns  (cable delay + PHC offset)")
    a(f"  Std  : {diff_ns.std():.3f} ns")
    a(f"  Peak : {diff_ns.max():+.1f} / {diff_ns.min():+.1f} ns")
    a("")

    key_taus = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000]

    # Per-channel TDEV comparison (the key metric)
    a("── TDEV(τ) — per channel ──────────────────────────────────")
    a(f"  {'τ (s)':>8s}  {'chA disc (ns)':>14s}  {'chB raw (ns)':>14s}  {'diff (ns)':>12s}")
    for tau in key_taus:
        vals = []
        for key in ("chA", "chB", "diff"):
            stab = ind_stab.get(key, {})
            if stab and "taus_tdev" in stab:
                idx = np.searchsorted(stab["taus_tdev"], tau)
                if idx < len(stab["tdev"]):
                    actual_tau = stab["taus_tdev"][idx]
                    if abs(actual_tau - tau) / max(tau, 1) < 0.3:
                        vals.append(f"{stab['tdev'][idx]*1e9:>14.3f}")
                    else:
                        vals.append(f"{'—':>14s}")
                else:
                    vals.append(f"{'—':>14s}")
            else:
                vals.append(f"{'—':>14s}")
        a(f"  {tau:>8d}  {vals[0]}  {vals[1]}  {vals[2]}")
    a("")

    # Per-channel ADEV
    a("── ADEV(τ) — per channel ──────────────────────────────────")
    a(f"  {'τ (s)':>8s}  {'chA disc (ns)':>14s}  {'chB raw (ns)':>14s}  {'diff (ns)':>12s}")
    for tau in key_taus:
        vals = []
        for key in ("chA", "chB", "diff"):
            stab = ind_stab.get(key, {})
            if stab and "taus_adev" in stab:
                idx = np.searchsorted(stab["taus_adev"], tau)
                if idx < len(stab["adev"]):
                    actual_tau = stab["taus_adev"][idx]
                    if abs(actual_tau - tau) / max(tau, 1) < 0.3:
                        vals.append(f"{stab['adev'][idx]*1e9:>14.3f}")
                    else:
                        vals.append(f"{'—':>14s}")
                else:
                    vals.append(f"{'—':>14s}")
            else:
                vals.append(f"{'—':>14s}")
        a(f"  {tau:>8d}  {vals[0]}  {vals[1]}  {vals[2]}")
    a("")

    a("=" * 62)

    path = out_stem.parent / (out_stem.name + "_report.txt")
    path.write_text("\n".join(lines) + "\n")
    print(f"Report  → {path}")
    for line in lines:
        print(line)


# ── Plots ────────────────────────────────────────────────────────────── #

def plot_timeseries(piv: pd.DataFrame, out_stem: Path,
                    servo_log: pd.DataFrame = None) -> None:
    """chA−chB time series with optional servo log overlay."""
    fig, ax1 = plt.subplots(figsize=(14, 5))

    diff_ns = piv["raw_diff_s"] * 1e9
    # Remove mean to show jitter around zero
    diff_centered = diff_ns - diff_ns.mean()
    t = np.arange(len(diff_centered))

    ax1.plot(t, diff_centered, color="steelblue", linewidth=0.5,
             alpha=0.8, label="chA−chB (centered)")
    ax1.axhline(0, color="navy", linewidth=0.5, linestyle="--")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("PHC − GPS PPS (ns, centered)")
    ax1.set_title(f"PHC Servo: chA−chB  |  std={diff_ns.std():.2f} ns  "
                  f"mean={diff_ns.mean():+.1f} ns  N={len(piv)}")
    ax1.grid(True, alpha=0.3)

    if servo_log is not None and "phc_error_ns" in servo_log.columns:
        ax2 = ax1.twinx()
        ax2.plot(servo_log.index, servo_log["phc_error_ns"],
                 color="tomato", linewidth=0.5, alpha=0.6, label="servo phc_err")
        ax2.set_ylabel("Servo PHC error (ns)", color="tomato")
        ax2.tick_params(axis="y", labelcolor="tomato")

    ax1.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    path = out_stem.parent / (out_stem.name + "_timeseries.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print(f"Plot    → {path}")
    plt.close(fig)


def plot_tdev(ind_stab: dict, out_stem: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 5))

    colors = {"chA": "steelblue", "chB": "tomato", "diff": "gray"}
    labels = {"chA": "chA: disciplined PHC PPS",
              "chB": "chB: F9T-3RD raw GPS PPS",
              "diff": "chA−chB (differential)"}

    for key in ("chA", "chB", "diff"):
        stab = ind_stab.get(key, {})
        if stab and "taus_tdev" in stab:
            style = "o-" if key != "diff" else "s--"
            ax.loglog(stab["taus_tdev"], np.array(stab["tdev"]) * 1e9,
                      style, color=colors[key], markersize=4, linewidth=1.5,
                      label=labels[key], alpha=0.9 if key != "diff" else 0.6)

    ax.set_xlabel("τ (s)")
    ax.set_ylabel("TDEV (ns)")
    ax.set_title("PePPAR Fix M5 — Time Deviation: disciplined vs raw PPS")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, which="both")
    fig.tight_layout()
    path = out_stem.parent / (out_stem.name + "_tdev.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print(f"Plot    → {path}")
    plt.close(fig)


def plot_adev(ind_stab: dict, out_stem: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 5))

    colors = {"chA": "steelblue", "chB": "tomato", "diff": "gray"}
    labels = {"chA": "chA: disciplined PHC PPS",
              "chB": "chB: F9T-3RD raw GPS PPS",
              "diff": "chA−chB (differential)"}

    for key in ("chA", "chB", "diff"):
        stab = ind_stab.get(key, {})
        if stab and "taus_adev" in stab:
            style = "o-" if key != "diff" else "s--"
            ax.loglog(stab["taus_adev"], np.array(stab["adev"]) * 1e9,
                      style, color=colors[key], markersize=4, linewidth=1.5,
                      label=labels[key], alpha=0.9 if key != "diff" else 0.6)

    ax.set_xlabel("τ (s)")
    ax.set_ylabel("ADEV (ns)")
    ax.set_title("PePPAR Fix M5 — Allan Deviation: disciplined vs raw PPS")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, which="both")
    fig.tight_layout()
    path = out_stem.parent / (out_stem.name + "_adev.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print(f"Plot    → {path}")
    plt.close(fig)


# ── Main ─────────────────────────────────────────────────────────────── #

def main():
    ap = argparse.ArgumentParser(
        description="Analyze PHC servo performance from TICC timestamps (M5)")

    ap.add_argument("--ticc", type=Path,
                    help="Input TICC CSV file (if not capturing)")
    ap.add_argument("--capture", action="store_true",
                    help="Capture live TICC data before analysis")
    ap.add_argument("--port", default="/dev/ticc",
                    help="TICC serial port (default: /dev/ticc)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="TICC baud rate (default: 115200)")
    ap.add_argument("--duration", type=int, default=300,
                    help="Capture duration in seconds (default: 300)")
    ap.add_argument("--servo-log", type=Path,
                    help="Servo CSV log for overlay")
    ap.add_argument("--out", type=Path, required=True,
                    help="Output stem (e.g. data/servo → data/servo_report.txt)")
    ap.add_argument("--taus", default="all", choices=["all", "decade", "octave"],
                    help="Tau selection for ADEV/TDEV (default: all)")

    args = ap.parse_args()

    # Capture if requested
    if args.capture:
        ticc_path = args.out.parent / (args.out.name + "_ticc.csv")
        ticc_path.parent.mkdir(parents=True, exist_ok=True)
        capture_ticc(args.port, args.duration, ticc_path, args.baud)
    elif args.ticc:
        ticc_path = args.ticc
    else:
        ap.error("Either --ticc or --capture is required")

    # Load and pair
    print(f"Loading TICC data from {ticc_path}")
    piv = load_ticc(ticc_path)
    print(f"  {len(piv)} paired chA/chB seconds")

    if len(piv) < 8:
        print("ERROR: Too few paired samples for analysis")
        sys.exit(1)

    # Compute per-channel and differential stability
    print("Computing per-channel ADEV/TDEV...")
    ind_stab = individual_stability(piv, taus=args.taus)

    # Load servo log if provided
    servo_log = None
    if args.servo_log and args.servo_log.exists():
        servo_log = pd.read_csv(args.servo_log)

    # Output
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_report(piv, ind_stab, args.out)
    plot_timeseries(piv, args.out, servo_log)
    plot_tdev(ind_stab, args.out)
    plot_adev(ind_stab, args.out)

    print("\nDone.")


if __name__ == "__main__":
    main()
