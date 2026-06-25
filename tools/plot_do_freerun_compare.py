#!/usr/bin/env python3
"""Overlay free-run ADEV + TDEV across the lab DOs.

Reads the dense free-run curves from each DO's <uid>.noise.toml sidecar
([adev_by_tau_s] + [tdev_ns_by_tau_s]) and overlays them, so the OCXO-class
DOs (MadHat / clkPoC3 / PiFace / otcBob1) and the TCXO (TimeHat) sit on one
log-log plot.  A raw TICC capture (chB=DO vs chA=Rb) can be added inline for a
DO that doesn't have a sidecar yet (otcBob1 now; ptBoat soon) via
--extra LABEL:CSV[:CLASS].

  python tools/plot_do_freerun_compare.py \
    --sidecar-dir ~/gt/datasheets/do-freerun-compare \
    --extra "otcBob1:~/gt/datasheets/p4-otcbob1-freerun/p4freerun-2026-06-25.csv:OCXO" \
    --out ~/gt/datasheets/do-freerun-compare/do_freerun_compare.png

Caveat surfaced on the plot: the sidecars were measured vs the TICC's internal
Rb (a free Rb); an --extra capture vs the GNSS-disciplined GNSSDO+ is comparable
at short/mid τ but its long-τ (τ≳60 s) tracks GPS, not the free DO.
"""
import argparse, glob, os, subprocess, tomllib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# DO class hint by uid substring (for color/marker grouping + the title).
def _do_class(uid, override=None):
    if override:
        return override
    u = uid.lower()
    if "tcxo" in u or u.startswith("54-49"):  # TimeHat i226 = MAC-named TCXO
        return "TCXO"
    return "OCXO"


def _load_sidecar(path):
    d = tomllib.load(open(path, "rb"))
    name = os.path.splitext(os.path.basename(path))[0].replace(".noise", "")
    adev = {float(k): float(v) for k, v in d.get("adev_by_tau_s", {}).items()}
    tdev = {float(k): float(v) for k, v in d.get("tdev_ns_by_tau_s", {}).items()}
    return name, _do_class(d.get("do_uid", name)), adev, tdev, d.get("measurement_channel", "")


def _from_capture(label, csv, cls):
    import allantools
    A, B = {}, {}
    for line in open(os.path.expanduser(csv)):
        if line[:1] == "#" or line.startswith("ts_iso"):
            continue
        p = line.split(",")
        if len(p) < 4:
            continue
        try:
            ch, sec, ps = p[1], int(p[2]), int(p[3])
        except ValueError:
            continue
        (A if ch == "chA" else B)[sec] = ps
    secs = sorted(A.keys() & B.keys())
    diff = np.array([B[s] - A[s] for s in secs], float)
    x = np.array(secs, float); x -= x[0]
    dd = diff - np.polyval(np.polyfit(x, diff, 1), x)        # detrend freq offset
    ph = dd * 1e-12
    taus = [1, 2, 5, 10, 30, 100, 300]
    t, ad, _, _ = allantools.adev(ph, rate=1.0, data_type="phase", taus=taus)
    t2, td, _, _ = allantools.tdev(ph, rate=1.0, data_type="phase", taus=taus)
    return label, cls, dict(zip(t, ad)), {k: v * 1e9 for k, v in zip(t2, td)}, "DO via ClockMatrix DCO output"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sidecar-dir", required=True)
    ap.add_argument("--extra", action="append", default=[],
                    help="LABEL:CSV[:CLASS] — add a DO from a raw TICC capture")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    dos = []
    for f in sorted(glob.glob(os.path.join(os.path.expanduser(args.sidecar_dir), "*.noise.toml"))):
        dos.append(_load_sidecar(f))
    for e in args.extra:
        parts = e.split(":")
        label, csv = parts[0], parts[1]
        cls = parts[2] if len(parts) > 2 else None
        dos.append(_from_capture(label, csv, cls or "OCXO"))

    cmap = plt.get_cmap("tab10")
    fig, (axA, axT) = plt.subplots(1, 2, figsize=(13, 5.5))
    for i, (name, cls, adev, tdev, chan) in enumerate(dos):
        c = cmap(i)
        ls = "--" if cls == "TCXO" else "-"
        if adev:
            ta = sorted(adev); axA.loglog(ta, [adev[t] * 1e9 for t in ta], ls, marker="o",
                                          color=c, label=f"{name} ({cls})", alpha=0.85)
        if tdev:
            tt = sorted(tdev); axT.loglog(tt, [tdev[t] for t in tt], ls, marker="o",
                                          color=c, label=f"{name} ({cls})", alpha=0.85)
    for ax, ylab, title in ((axA, "ADEV (ppb)", "Free-run ADEV"),
                            (axT, "TDEV (ns)", "Free-run TDEV")):
        ax.set_xlabel("τ (s)"); ax.set_ylabel(ylab); ax.set_title(title)
        ax.grid(True, which="both", alpha=0.3); ax.legend(fontsize=8)
    axA.axhline(0.001, ls=":", color="grey", lw=1)
    axA.text(1, 0.0011, "class-default σ_do_freq", fontsize=7, color="grey")
    fig.suptitle("Lab DO free-run noise — OCXO-class (solid) vs TCXO (dashed).  CAVEAT: "
                 "sidecars are DAC-parked DO vs TICC-Rb (truly open-loop); --extra (otcBob1) "
                 "is the ClockMatrix DCO output, combo-armed BW≈0 — NOT truly open-loop, so its "
                 "τ≳60 s drop is residual discipline, not the DO floor.",
                 fontsize=8.5)
    sha = subprocess.getoutput("git -C %s rev-parse --short HEAD" % os.path.dirname(__file__))
    fig.text(0.5, 0.005, f"ANALYSIS_VERSION=do-freerun-compare-1  git={sha}",
             ha="center", fontsize=7, color="grey")
    out = os.path.expanduser(args.out)
    fig.tight_layout(rect=[0, 0.02, 1, 0.95]); fig.savefig(out, dpi=120)
    print("wrote", out)


if __name__ == "__main__":
    main()
