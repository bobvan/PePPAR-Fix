#!/usr/bin/env python3
"""analyze_chokering_pcv.py — mean & peak-to-peak PCV (relative to the PCO) for
every choke-ring antenna in an IGS ANTEX file.

ANTEX models a receiver antenna's phase pattern as PCO (a constant offset, the
mean phase centre) + PCV(zenith[,azimuth]) (the residual *relative to the PCO*,
mm).  This computes, per antenna per band, the **mean** and **peak-to-peak** of
the NOAZI (elevation-only) PCV — i.e. the residual swing around the PCO.

Choke rings are identified by name token (no clean ANTEX field exists); the
token list is documented + overridable, and the matched list is printed so the
classification is auditable.  A few Ashtech models (ASH700228/701933/701941)
are flagged as uncertain (may be plain geodetic, not choke).

Usage:
  analyze_chokering_pcv.py [igs20.atx] --out-dir plots [--csv out.csv]
"""
import argparse
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
import antex

DEFAULT_ATX = os.path.expanduser("~/PRIDE-PPPAR/table/igs20.atx")
# Name tokens for choke-ring families (auditable; tweak with --tokens).
TOKENS = ("CHOKE", "RINGANT", "AOAD/M", "_DM", "ASH700936", "ASH701945",
          "ASH701946", "ASH700228", "ASH701933", "ASH701073", "ASH701941",
          "TRM29659", "LEIAR25", "JNSCR", "NOV750", "NOV503", "TPSCR")
UNCERTAIN = ("ASH700228", "ASH701933", "ASH701941")   # may be geodetic, not choke

FAMILY = [("AOAD/M", "AOA/Dorne-Margolin"), ("ASH7009", "Ashtech-DM"),
          ("ASH70194", "Ashtech-CR"), ("ASH7002", "Ashtech-700228?"),
          ("ASH70193", "Ashtech-701933?"), ("ASH70107", "Ashtech-701073"),
          ("JAVRINGANT", "Javad-RingAnt"), ("JAV_RINGANT", "Javad-RingAnt"),
          ("JNSCR", "Javad-CR"), ("LEIAR25", "Leica-AR25"),
          ("NOV", "NovAtel-CR"), ("SEPCHOKE", "Septentrio-CR"),
          ("TPSCR", "Topcon-CR"), ("TRM29659", "Trimble-CR")]


def family_of(name):
    u = name.upper()
    for pre, fam in FAMILY:
        if u.startswith(pre):
            return fam
    return "other"


def band_noazi(ax, ant, freq):
    p = ax.recv_patterns.get((ant, freq))
    if p is None:
        return None
    z = np.arange(p.zen_start, p.zen_end + 1e-6, p.zen_step)
    pcv = np.asarray(p.noazi_m) * 1000.0      # mm
    n = min(len(z), len(pcv))
    return z[:n], pcv[:n]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("atx", nargs="?", default=DEFAULT_ATX)
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--csv", default=None)
    ap.add_argument("--tokens", default=None, help="comma list to override choke-ring tokens")
    ap.add_argument("--out-name", default="chokering_pcv", help="output basename (no ext)")
    ap.add_argument("--title", default="Choke-ring antenna PCV (relative to PCO)",
                    help="plot title prefix")
    args = ap.parse_args()

    toks = tuple(t.strip().upper() for t in args.tokens.split(",")) if args.tokens else TOKENS
    ax = antex.ANTEXParser(args.atx)
    names = sorted(set(k[0] for k in ax.recv_patterns))
    ck = [n for n in names if any(t in n.upper() for t in toks)]
    print(f"{os.path.basename(args.atx)}: {len(names)} recv antenna types, "
          f"{len(ck)} choke-ring type+radome entries\n")

    rows = []
    for ant in ck:
        b1 = band_noazi(ax, ant, "G01")
        if not b1:
            continue
        b2 = band_noazi(ax, ant, "G02")
        l1 = b1[1]; l2 = b2[1] if b2 else np.array([np.nan])
        rows.append(dict(antenna=ant, family=family_of(ant),
                         uncertain=any(u in ant.upper() for u in UNCERTAIN),
                         l1_mean=float(l1.mean()), l1_pp=float(l1.max() - l1.min()),
                         l2_mean=float(np.nanmean(l2)), l2_pp=float(np.nanmax(l2) - np.nanmin(l2)),
                         zen=b1[0], l1_curve=l1, l2_curve=(b2[1] if b2 else None)))

    A = np.array([(r["l1_mean"], r["l1_pp"], r["l2_mean"], r["l2_pp"]) for r in rows], float)
    cert = np.array([not r["uncertain"] for r in rows])
    print(f"=== AGGREGATE over {len(rows)} entries (mm) ===")
    for lbl, mask in (("ALL matched", np.ones(len(rows), bool)),
                      ("confident-only", cert)):
        a = A[mask]
        print(f"  [{lbl}, n={mask.sum()}]")
        print(f"    L1 mean-PCV avg {np.nanmean(a[:,0]):+.2f} ({np.nanmin(a[:,0]):+.2f}..{np.nanmax(a[:,0]):+.2f}); "
              f"p-p avg {np.nanmean(a[:,1]):.2f} ({np.nanmin(a[:,1]):.2f}..{np.nanmax(a[:,1]):.2f})")
        print(f"    L2 mean-PCV avg {np.nanmean(a[:,2]):+.2f} ({np.nanmin(a[:,2]):+.2f}..{np.nanmax(a[:,2]):+.2f}); "
              f"p-p avg {np.nanmean(a[:,3]):.2f} ({np.nanmin(a[:,3]):.2f}..{np.nanmax(a[:,3]):.2f})")

    # CSV
    csv_path = args.csv or os.path.join(args.out_dir, args.out_name + ".csv")
    os.makedirs(args.out_dir, exist_ok=True)
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["antenna", "family", "uncertain", "L1_mean_mm", "L1_pp_mm", "L2_mean_mm", "L2_pp_mm"])
        for r in rows:
            w.writerow([r["antenna"], r["family"], int(r["uncertain"]),
                        f"{r['l1_mean']:.2f}", f"{r['l1_pp']:.2f}",
                        f"{r['l2_mean']:.2f}", f"{r['l2_pp']:.2f}"])
    print(f"\nwrote {csv_path}  ({len(rows)} rows)")

    # Plot: NOAZI PCV vs zenith, one curve per MODEL (radome NONE if present),
    # coloured by family, L1 + L2 panels.
    seen = {}
    for r in rows:
        model = r["antenna"].split()[0]
        if model not in seen or r["antenna"].endswith("NONE"):
            seen[model] = r
    fams = sorted(set(r["family"] for r in seen.values()))
    cmap = {f: plt.cm.tab20(i % 20) for i, f in enumerate(fams)}
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(16, 9), sharey=True)
    for ax_, band, lbl in ((a1, "l1_curve", "GPS L1"), (a2, "l2_curve", "GPS L2")):
        for r in seen.values():
            c = r[band]
            if c is None:
                continue
            ax_.plot(r["zen"], c, lw=1.3, color=cmap[r["family"]], alpha=0.8)
        ax_.axhline(0, color="gray", lw=0.6)
        ax_.set_xlabel("zenith angle  [deg]  (0 = boresight, 90 = horizon)")
        ax_.grid(alpha=0.3)
        ax_.set_title(lbl)
    a1.set_ylabel("NOAZI PCV relative to PCO  [mm]")
    handles = [plt.Line2D([], [], color=cmap[f], lw=3, label=f) for f in fams]
    a1.legend(handles=handles, loc="lower left", fontsize=11)
    fig.suptitle(f"{args.title} — IGS {os.path.basename(args.atx)}\n"
                 f"{len(seen)} model(s) · L1 p-p avg {np.nanmean(A[:,1]):.0f} mm, "
                 f"L2 p-p avg {np.nanmean(A[:,3]):.0f} mm", fontsize=18, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out = os.path.join(args.out_dir, args.out_name)
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png  ({len(seen)} models plotted)")


if __name__ == "__main__":
    sys.exit(main())
