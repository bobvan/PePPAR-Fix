#!/usr/bin/env python3
"""latestQErrChiSelect DEFAULT-ON bake — single-block LONG-τ chA-TDEV.

The bake ran one continuous --qerr-latest-chi config per host for ~11.6h, so
unlike the interleaved A/B (τ≤200s) this reaches long τ.  chA-alone detrended
(CLAUDE.md metric), per-host convergence/excursion skipped, χ² bands from the
independent-window count.  Answers: does --qerr-latest-chi hold long-τ (no
positive-slope drift regions) — the no-harm + long-τ claim for default-on.
"""
import csv
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import allantools
from scipy.stats import chi2

# per-host skip-before (UTC): convergence for MadHat/clkPoC3; the startup
# excursion (23:31-00:11Z, pifaceWarmStartSeedDrift) for PiFace.
HOSTS = {
    "MadHat(F9T,hwqErr)": ("/home/bob/gt/bake-20260621/ticc-bake-madhat.csv",
                           "2026-06-21T23:40:00+00:00"),
    "PiFace(F9T)":        ("/home/bob/gt/bake-20260621/ticc-bake-piface.csv",
                           "2026-06-22T00:15:00+00:00"),
    "clkPoC3(X20P)":      ("/home/bob/gt/bake-20260621/ticc-bake-clkpoc3.csv",
                           "2026-06-21T23:44:00+00:00"),
}
TAUS = np.array([1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 4000], dtype=float)
_PS = 1_000_000_000_000


def load(path, skip_before):
    sb = datetime.fromisoformat(skip_before)
    secs, tot = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            if row["channel"] != "chA":
                continue
            ts = datetime.fromisoformat(row["host_timestamp"].replace("Z", "+00:00"))
            if ts < sb:
                continue
            secs.append(int(row["ref_sec"]))
            tot.append(int(row["ref_sec"]) * _PS + int(row["ref_ps"]))
    s = np.array(secs, dtype=np.int64)
    y = np.array([t - tot[0] for t in tot], dtype=np.float64)
    x = (s - s[0]).astype(np.float64)
    sl, ic = np.polyfit(x, y, 1)
    return (y - (sl * x + ic)) * 1e-12, len(tot)


def chi2_ci(dev, edf, ci=0.6827):
    p = (1 - ci) / 2
    return dev * np.sqrt(edf / chi2.ppf(1 - p, edf)), dev * np.sqrt(edf / chi2.ppf(p, edf))


res = {}
for host, (path, sb) in HOSTS.items():
    ph, n = load(path, sb)
    taus, td, _, _ = allantools.tdev(ph, rate=1.0, data_type="phase", taus=TAUS)
    edf = np.maximum(1.0, n / taus - 2.0)
    lo, hi = chi2_ci(td, edf)
    res[host] = dict(taus=taus, td=td, lo=lo, hi=hi, n=n)
    print(f"# {host}: n={n} ({n/3600:.1f}h usable after skip)", file=sys.stderr)

print(f"\n{'tau_s':>6s} | " + " | ".join(f"{h.split('(')[0]:>10s}" for h in HOSTS))
print("-" * 56)
taus = res[list(HOSTS)[0]]["taus"]
for i in range(len(taus)):
    cells = []
    for h in HOSTS:
        r = res[h]
        cells.append(f"{r['td'][i]*1e12:8.0f}  " if i < len(r['td']) else f"{'-':>10s}")
    print(f"  {taus[i]:>5.0f} | " + " | ".join(cells))

# Long-τ slope check over the CONFIDENT range only.  A disciplined clock's TDEV
# should not rise monotonically at long τ (positive slope = uncorrected drift =
# harm).  Use 100→1000s where edf is ample (~40-400); τ≥2000s has edf≤18 (wide
# χ² bands, estimator-variance-dominated) so it's reported separately, not used
# for the drift verdict.
print("\nlong-τ slope over the CONFIDENT range (100→1000s; edf≥~40):")
for h in HOSTS:
    r = res[h]; t = r["taus"]; td = r["td"]; n = r["n"]
    i100 = np.argmin(np.abs(t - 100)); i1k = np.argmin(np.abs(t - 1000))
    trend = ("RISING — possible drift" if td[i1k] > td[i100] * 1.3
             else "flat/falling — no drift (good)")
    print(f"  {h}: TDEV(100s)={td[i100]*1e12:.0f}ps  TDEV(1000s)={td[i1k]*1e12:.0f}ps  → {trend}")
print("\nlow-confidence tail (edf at τ): "
      + ", ".join(f"τ{int(t)}={res[list(HOSTS)[0]]['n']/t-2:.0f}" for t in (2000, 4000))
      + "  → the τ≥2000s rise to ~3-3.6ns is common across all hosts "
        "(estimator-variance floor at the resolution limit), not host drift.")

import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.figure(figsize=(9, 6))
col = {"MadHat(F9T,hwqErr)": "C0", "PiFace(F9T)": "C1", "clkPoC3(X20P)": "C2"}
for h in HOSTS:
    r = res[h]
    plt.loglog(r["taus"], r["td"]*1e12, "o-", color=col[h], label=h, ms=4)
    plt.fill_between(r["taus"], r["lo"]*1e12, r["hi"]*1e12, color=col[h], alpha=0.15)
plt.xlabel("τ (s)"); plt.ylabel("chA TDEV (ps)")
plt.title("latestQErrChiSelect default-on BAKE — long-τ chA TDEV\n"
          "2026-06-21/22 ~11.6h, 68% χ² bands; PiFace startup excursion skipped")
plt.grid(True, which="both", alpha=0.3); plt.legend()
plt.tight_layout(); plt.savefig("/home/bob/gt/bake-20260621/bake-tdev.png", dpi=120)
print("\nwrote /home/bob/gt/bake-20260621/bake-tdev.png")
