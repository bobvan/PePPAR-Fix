#!/usr/bin/env python3
"""Analyse a GNSSDO+ steering-comparison CSV (--gnssdo-compare-log).

Compares PePPAR-Fix's carrier-phase steering against the mosaic-T's own
PVTGeodetic clock solution.  Reports the metrologically VALID comparisons
(see docs/gnssdo-plus-integration.md §8):

  - TDEV of the DIFFERENCE (our dt_rx − their RxClkBias), linearly detrended:
    the two ESTIMATORS' relative stability (common OCXO motion cancels) —
    short τ shows their code/SPP noise vs our carrier precision, long τ the
    carrier-PPP-vs-AtomiChron datum drift.  This is the valid one.
  - TDEV of each series individually is printed FOR CONTEXT ONLY: our dt_rx is
    a closed-loop residual (the signal we minimise — suppressed, not a clock),
    theirs is an open-loop estimate.  Not a fair head-to-head; use the TICC chA
    for the honest disciplined-output grade.
  - Correction signals (our freq_ppb, their RxClkDrift): PSD + std + epoch-to-
    epoch roughness — the right tools for "servo smoothness", NOT TDEV (TDEV is
    a phase metric).

CSV columns: mono_s, our_dt_rx_ns, our_control_word, our_freq_ppb,
             mosaic_rxclkbias_ns, mosaic_rxclkdrift_ppb, mosaic_mode
"""
import argparse
import numpy as np
import allantools


def _detrend(x):
    n = np.arange(len(x))
    return x - np.polyval(np.polyfit(n, x, 1), n)


def _tdev(x_ns, taus):
    taus_out, td, _, _ = allantools.tdev(_detrend(x_ns) * 1e-9, rate=1.0,
                                         data_type="phase", taus=taus)
    return taus_out, td


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--skip-s", type=float, default=60.0,
                    help="Drop the first N s (acquisition transient); default 60")
    ap.add_argument("--psd", action="store_true",
                    help="Also print a Welch PSD of the two correction signals")
    args = ap.parse_args()

    r = np.genfromtxt(args.csv, delimiter=",", names=True)
    t = r["mono_s"] - r["mono_s"][0]
    m = t >= args.skip_s
    our_dt, their_b = r["our_dt_rx_ns"][m], r["mosaic_rxclkbias_ns"][m]
    our_f, their_d = r["our_freq_ppb"][m], r["mosaic_rxclkdrift_ppb"][m]
    n = int(m.sum())
    print(f"{args.csv}\nsettled window: {n} s "
          f"(TDEV reliable only to τ≈{max(1, n//8)} s; τ>{n//4}s is noise)\n")

    taus = [tt for tt in (1, 2, 4, 8, 16, 32, 64, 128, 256, 512) if tt <= n // 4]
    print("TDEV (linearly detrended):")
    for label, x in (("difference (ours−theirs) ★", our_dt - their_b),
                     ("ours dt_rx (loop resid)", our_dt),
                     ("theirs RxClkBias", their_b)):
        to, td = _tdev(x, taus)
        print(f"  {label:26s} " +
              " ".join(f"τ{int(a)}={v*1e9:6.3f}ns" for a, v in zip(to, td)))
    print("  (★ = the valid comparison; the other two are context — see docstring)")

    print("\nCorrection signals (servo smoothness — PSD/roughness, NOT TDEV):")
    for name, x in (("ours   freq_ppb", our_f), ("theirs RxClkDrift", their_d)):
        rough = np.sqrt(np.mean(np.diff(x) ** 2))
        print(f"  {name:20s} std={np.std(x):.4f} ppb  "
              f"epoch-to-epoch RMS(Δ)={rough:.4f} ppb")

    if args.psd:
        from scipy import signal
        print("\nWelch PSD (ppb²/Hz) of the corrections:")
        for name, x in (("ours   freq_ppb", our_f), ("theirs RxClkDrift", their_d)):
            f, pxx = signal.welch(x - np.mean(x), fs=1.0,
                                  nperseg=min(64, len(x)))
            print(f"  {name:20s} " +
                  " ".join(f"{ff:.3f}Hz={pp:.2e}"
                           for ff, pp in zip(f[1:6], pxx[1:6])))


if __name__ == "__main__":
    main()
