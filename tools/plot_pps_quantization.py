#!/usr/bin/env python3
"""plot_pps_quantization.py — measure PPS quantization noise EXTERNALLY on a
TICC, for any receiver (whether or not it reports qErr in TIM-TP).

Bob's method: a receiver makes its 1 PPS by dividing an internal clock, so the
edge lands on a clock grid — that quantization rides on the PPS regardless of
whether the receiver exposes it.  Measure the receiver's PPS on TICC chB, take
the sub-second phase (ref_ps), detrend the slow drift of the TICC reference,
and the residual is the quantization (plus the TICC's ~60 ps single-shot
noise).  Do it on an F9T, an F9P and an X20P and the magnitude differences in
the per-receiver PPS jitter show up directly — even though F9P/X20P report
qErr=0 (HPG firmware; see f9t-firmware-capabilities.md).

Integer-safe per Bob's caveat: ref_sec/ref_ps are kept as Python ints and the
sub-second residual is `total_ps - n_pulse*PS_PER_S` (the round() handles a
drift crossing a second boundary) — never a float seconds timestamp (which
would lose ps resolution).  Same reconstruction as
scripts/analyze_discipline_noise.py.

Usage:
  plot_pps_quantization.py --source "F9T=f9t.ticc.csv" \
      --source "F9P=f9p.ticc.csv" --source "X20P=x20.ticc.csv" \
      --out-dir plots [--channel chB] [--window 250]
"""
import argparse
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PS_PER_S = 1_000_000_000_000
TICC_FLOOR_NS = 0.060  # TAPR TICC single-shot resolution

plt.rcParams.update({
    "font.size": 17, "axes.titlesize": 21, "axes.labelsize": 19,
    "xtick.labelsize": 16, "ytick.labelsize": 16, "legend.fontsize": 15,
})


def load_chan(path, channel="chB"):
    """Detrended PPS-edge phase residual (ns) for one TICC channel.
    Integer-safe: ref_sec/ref_ps stay ints; residual = total_ps - n_pulse*1e12."""
    mono, sec, ps = [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            if row.get("channel") != channel:
                continue
            # engine .ticc.csv uses host_monotonic; ticc_capture.py uses recv_mono
            mraw = row.get("host_monotonic") or row.get("recv_mono")
            try:
                mono.append(float(mraw))
                sec.append(int(row["ref_sec"]))
                ps.append(int(row["ref_ps"]))
            except (ValueError, KeyError, TypeError):
                continue
    if len(sec) < 8:
        raise SystemExit(f"too few {channel} samples in {path} ({len(sec)})")
    mono = np.asarray(mono)
    sec = np.asarray(sec, dtype=np.int64)
    ps = np.asarray(ps, dtype=np.int64)
    back = np.where(np.diff(sec) < -10)[0]          # drop pre-TICC-reset
    if len(back):
        cut = back[-1] + 1
        mono, sec, ps = mono[cut:], sec[cut:], ps[cut:]
    # total_ps relative to first sample via object dtype → no float overflow
    total_ps = (sec - sec[0]).astype(object) * PS_PER_S + (ps - ps[0]).astype(object)
    total_ps = np.array([int(v) for v in total_ps], dtype=np.int64)
    n_pulse = np.round(total_ps / PS_PER_S).astype(np.int64)   # handles sec-boundary wrap
    resid_ps = total_ps - n_pulse * PS_PER_S
    phase_ns = resid_ps.astype(float) * 1e-3
    # linear-detrend vs monotonic time: remove the ref/receiver freq offset
    t = mono - mono[0]
    slope, intercept = np.polyfit(t, phase_ns, 1)
    phase_ns = phase_ns - (slope * t + intercept)
    return t, phase_ns, slope


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", action="append", required=True, metavar="LABEL=CSV")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--channel", default="chB")
    ap.add_argument("--window", type=int, default=250, help="seconds shown in the time panels")
    args = ap.parse_args()

    series = []
    for spec in args.source:
        label, path = spec.split("=", 1)
        t, ph, slope = load_chan(path, args.channel)
        series.append((label, t, ph, slope))

    ylim = max(np.percentile(np.abs(ph), 99.5) for _, _, ph, _ in series) * 1.15
    ylim = max(ylim, 1.0)
    n = len(series)
    fig, axes = plt.subplots(n, 1, figsize=(16, 9), sharex=True)
    if n == 1:
        axes = [axes]
    for ax, (label, t, ph, slope) in zip(axes, series):
        m = t <= args.window
        # metrics over the common window so different capture lengths compare fairly
        rms = float(np.sqrt(np.mean(ph[m] ** 2)))
        pp = float(ph[m].max() - ph[m].min())
        # quantization estimate with the TICC 60 ps floor removed in quadrature
        q = (rms ** 2 - TICC_FLOOR_NS ** 2) ** 0.5 if rms > TICC_FLOOR_NS else 0.0
        ax.plot(t[m], ph[m], ".-", ms=3, lw=0.7, color="tab:blue")
        ax.axhline(0, color="gray", lw=0.5)
        ax.set_ylim(-ylim, ylim)
        ax.set_ylabel("phase  [ns]")
        ax.grid(alpha=0.3)
        ax.text(0.012, 0.93,
                f"{label}:  RMS {rms:.2f} ns   p-p {pp:.1f} ns   "
                f"≈quant {q:.2f} ns   (drift {slope*1e3:+.2f} ps/s)",
                transform=ax.transAxes, va="top", fontsize=16, fontweight="bold",
                bbox=dict(boxstyle="round", fc="#eef3fb", ec="0.6"))
    axes[-1].set_xlabel(f"elapsed time  [s]   (window {args.window} s)")
    axes[0].set_title("PPS quantization measured externally on TICC chB — detrended\n"
                      "(residual = receiver PPS quantization ⊕ ~60 ps TICC single-shot noise)")
    fig.tight_layout()
    os.makedirs(args.out_dir, exist_ok=True)
    out = os.path.join(args.out_dir, "pps_quantization")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    for label, t, ph, slope in series:
        rms = float(np.sqrt(np.mean(ph ** 2)))
        print(f"  {label}: n={len(t)} RMS={rms:.3f} ns p-p={ph.max()-ph.min():.2f} ns")
    print(f"wrote {out}.pdf/.png")


if __name__ == "__main__":
    sys.exit(main())
