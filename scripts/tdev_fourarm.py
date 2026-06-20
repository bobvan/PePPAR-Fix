#!/usr/bin/env python3
"""Four-arm F9T/X20P chA TDEV — the real (detrended chA-alone) comparison.

Metric per CLAUDE.md: TICC chA alone, detrended.  Sub-decade tau, chi^2
confidence bands from the independent-window count (these are 3 h arms).

Arms: default (TICC+EXTINT) / no-ticc (EXTINT only) / ticc-swqerr (TICC sw qErr)
      / ticc-hwqerr (TICC hw qErr [PiFace=ext, clkPoC3=raw]).
"""
import csv
import sys
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np
import allantools
from scipy.stats import chi2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
from plot_chA_tdev_goldilocks import load_chA_phase_s  # noqa: E402

HOSTS = {
    "PiFace(F9T)":  "/home/bob/gt/fourarm-piface-20260619-142528",
    "clkPoC3(X20P)": "/home/bob/gt/fourarm-clkpoc3-20260619-142530",
}
ARMS = ["default", "no-ticc", "ticc-swqerr", "ticc-hwqerr"]
SKIP_S = 600
TAUS = np.array([1, 2, 5, 10, 20, 50, 100, 200, 500, 1000], dtype=float)


def first_ts(path):
    for row in csv.DictReader(open(path)):
        if row["channel"] == "chA":
            return datetime.fromisoformat(row["host_timestamp"].replace("Z", "+00:00"))
    raise ValueError("no chA")


def chi2_ci(dev, edf, ci=0.6827):
    p = (1 - ci) / 2
    return dev * np.sqrt(edf / chi2.ppf(1 - p, edf)), dev * np.sqrt(edf / chi2.ppf(p, edf))


res = {}
for host, base in HOSTS.items():
    for arm in ARMS:
        p = f"{base}/{arm}/ticc.csv"
        try:
            ph = load_chA_phase_s(Path(p), first_ts(p) + timedelta(seconds=SKIP_S))
        except Exception as e:
            print(f"# {host}/{arm}: {e}", file=sys.stderr); continue
        N = len(ph)
        taus, td, _, _ = allantools.tdev(ph, rate=1.0, data_type="phase", taus=TAUS)
        edf = np.maximum(1.0, N / taus - 2.0)
        lo, hi = chi2_ci(td, edf)
        res[(host, arm)] = dict(taus=taus, td=td, lo=lo, hi=hi, n=N)

# ---- table (ps) ----
for host in HOSTS:
    print(f"\n=== {host} — chA TDEV (ps), detrended, {SKIP_S}s skip ===")
    hdr = "  tau_s | " + " | ".join(f"{a.replace('ticc-',''):>12s}" for a in ARMS)
    print(hdr); print("  " + "-" * (len(hdr) - 2))
    for i, t in enumerate(TAUS):
        cells = []
        for a in ARMS:
            r = res.get((host, a))
            cells.append(f"{r['td'][i]*1e12:7.0f}     " if r and i < len(r['td']) else f"{'-':>12s}")
        print(f"  {t:>5.0f} | " + " | ".join(cells))

# ---- plot ----
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, axes = plt.subplots(1, 2, figsize=(15, 6))
col = {"default": "C0", "no-ticc": "C1", "ticc-swqerr": "C2", "ticc-hwqerr": "C3"}
for ax, host in zip(axes, HOSTS):
    for a in ARMS:
        r = res.get((host, a))
        if not r: continue
        ax.loglog(r["taus"], r["td"]*1e12, "o-", color=col[a], label=a, ms=4)
        ax.fill_between(r["taus"], r["lo"]*1e12, r["hi"]*1e12, color=col[a], alpha=0.15)
    ax.set_xlabel("τ (s)"); ax.set_ylabel("chA TDEV (ps)")
    ax.set_title(f"{host} — four-arm chA TDEV\n2026-06-19/20, 68% χ² bands")
    ax.grid(True, which="both", alpha=0.3); ax.legend()
fig.tight_layout()
out = "/home/bob/gt/fourarm-analysis-tdev.png"
fig.savefig(out, dpi=110)
print(f"\nwrote {out}")
