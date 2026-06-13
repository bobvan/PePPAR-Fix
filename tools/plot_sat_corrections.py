#!/usr/bin/env python3
"""plot_sat_corrections.py — "the mountains the F9T subtracts to reach 15 ns".

Story 2 of the corrections narrative (Story 1 = the sub-ns onion, e.g.
plot_solid_tide.py).  Starting from raw RAWX, a receiver must compute and
apply a stack of physical corrections *just to produce an SPP fix good to
~15 ns* — the work an F9T does silently behind its PPS.  This tool draws
them from real broadcast ephemeris (BCEP, collected via NTRIP) evaluated at
the surveyed ARP:

  FIGURE 1 — satellite clock correction, ALL BY ITSELF, because it dwarfs
    everything else.  Per SV: dt_sv(t) = af0 + af1·(t−toc) + af2·(t−toc)²,
    the SV atomic clock's *offset from GNSS system time* — up to a few
    hundred µs (the broadcast af0 field spans ±0.977 ms).  NOT an error:
    a known, broadcast bias you subtract exactly.

  FIGURE 2 — the rest, on one log-|ns| axis with a 15 ns reference line:
    relativistic eccentricity (±46 ns), Sagnac/Earth-rotation (±100+ ns),
    troposphere (slant, 8–80 ns), ionosphere (representative slant), group
    delay TGD (a few ns).  Almost every line towers over the 15 ns mark —
    the point: mountains subtracted to leave a molehill.

Input: a BCEP ephemeris JSON ({prn: [eph_dict, ...]}, latest used), e.g.
collected with BroadcastEphemeris over an NTRIP BCEP00BKG0 stream.

Usage:
    python3 tools/plot_sat_corrections.py /tmp/bcep_ephs.json --out-dir plots
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "scripts"))
sys.path.insert(0, os.path.join(_REPO, "tools"))
from broadcast_eph import _kepler_ecef, F_REL, OMEGA_E, C
from plot_solid_tide import load_arp, enu_basis  # ARP from gitignored antennas.json

M2NS = 1e9 / C  # 3.3356 ns/m
RE = 6371000.0
H_ION = 350000.0
TECU_M_L1 = 0.162  # metres of L1 delay per TECU
KM_PER_US = 0.299792458  # 1 µs of clock error = 299.79 m of range
SYS_NAME = {"G": "GPS", "E": "Galileo", "C": "BeiDou"}
SYS_TINT = {"G": "#dbe7f3", "E": "#dcefdc", "C": "#ffe9d6"}

plt.rcParams.update({
    "font.size": 18, "axes.titlesize": 23, "axes.labelsize": 22,
    "xtick.labelsize": 20, "ytick.labelsize": 20, "legend.fontsize": 16,
})


def latest(ephs, prn):
    v = ephs.get(prn)
    return v[-1] if v else None


def elev_rad(sat_ecef, up, rx_ecef):
    los = sat_ecef - rx_ecef
    return math.asin(float(los @ up) / np.linalg.norm(los))


def iono_slant_ns(elev, tecu=20.0):
    """Representative slant ionosphere (single-frequency), thin-shell mapping."""
    z = math.pi / 2 - elev
    zp = math.asin(RE / (RE + H_ION) * math.sin(z))
    mf = 1.0 / math.cos(zp)
    return TECU_M_L1 * tecu * mf * M2NS


def saastamoinen_zhd_m(lat, h):
    p = 1013.25 * (1 - 2.2557e-5 * h) ** 5.2568
    return 0.0022768 * p / (1 - 0.00266 * math.cos(2 * lat) - 2.8e-7 * h)


def sagnac_ns(sat_ecef, rx_ecef):
    d_m = OMEGA_E * (sat_ecef[0] * rx_ecef[1] - sat_ecef[1] * rx_ecef[0]) / C
    return d_m * M2NS


def fig_satclock_table(ephs, out):
    """Per-constellation satellite-clock-offset table (the offset is ~constant
    in time, so a table beats a time series).  Makes the point: you can't do
    naïve math on raw observations — apply at least the broadcast clock."""
    rows, tints = [], []
    for sysc in ["G", "E", "C"]:
        svs = [p for p in ephs if p[0] == sysc and latest(ephs, p)]
        if not svs:
            continue
        off = [latest(ephs, p)["af0"] * 1e6 for p in svs]                       # µs
        drift = [abs(latest(ephs, p)["af1"] * 86400
                     + latest(ephs, p).get("af2", 0.0) * 86400 ** 2) * 1e6       # µs/day
                 for p in svs]
        lo, hi = min(off), max(off)
        worst = max(abs(lo), abs(hi))
        worst_str = f"{worst/1000:.2f} ms" if worst >= 1000 else f"{worst:.0f} µs"
        rows.append([SYS_NAME[sysc], str(len(svs)), f"{lo:+.0f} … {hi:+.0f} µs",
                     f"{worst_str}  ≈ {worst*KM_PER_US:.0f} km", f"≤ {max(drift):.1f} µs"])
        tints.append(SYS_TINT[sysc])
    cols = ["Constellation", "SVs", "clock-offset range",
            "worst error  (≡ position)", "drift / 24 h"]

    fig, ax = plt.subplots(figsize=(16, 9))
    fig.subplots_adjust(top=0.78, bottom=0.24)
    ax.axis("off")
    tbl = ax.table(cellText=rows, colLabels=cols, loc="center", cellLoc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(21)
    tbl.scale(1, 3.0)
    tbl.auto_set_column_width(col=list(range(len(cols))))
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("0.65")
        if r == 0:
            cell.set_facecolor("#333333")
            cell.get_text().set_color("white")
            cell.get_text().set_fontweight("bold")
        else:
            cell.set_facecolor(tints[r - 1])
            if c in (0, 3):
                cell.get_text().set_fontweight("bold")
    fig.suptitle("Satellite clock offset from system time —\nyou must apply the broadcast clock correction",
                 fontsize=26, fontweight="bold", y=0.93)
    fig.text(0.5, 0.17,
             "The SV clock isn't synced to system time — it free-runs at a large, known, broadcast offset\n"
             "(here ~constant: drift ≪ offset, so no time axis needed).  Skip it and naïve math on the raw\n"
             "observations is wrong by hundreds of km.  Apply it and the residual is ≈ 0.5–1 ns.",
             ha="center", fontsize=17)
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png   rows={[r[0] for r in rows]}")


def fig_combined(ephs, arp, lat, lon, out, span_h=2.5):
    up, _, _ = enu_basis(lat, lon)
    tk = np.linspace(-span_h * 3600, span_h * 3600, 151)
    th = tk / 3600.0
    # pick the GPS SV with the largest elevation arc in-window (best visual)
    best, best_range = None, -1
    for prn in [p for p in ephs if p[0] == "G" and latest(ephs, p)]:
        eph = latest(ephs, prn)
        els = []
        for t in tk:
            try:
                els.append(math.degrees(elev_rad(_kepler_ecef(eph, t, eph["gm"])[0], up, arp)))
            except Exception:
                els.append(-90)
        els = np.array(els)
        vis = els[els > 5]
        if len(vis) > 10 and (vis.max() - vis.min()) > best_range:
            best, best_range = prn, vis.max() - vis.min()
    eph = latest(ephs, best)
    e = float(eph["e"])
    sat, Ek = [], []
    for t in tk:
        p, ek = _kepler_ecef(eph, t, eph["gm"])
        sat.append(p); Ek.append(ek)
    rel = np.array([F_REL * e * eph["sqrt_a"] * math.sin(k) for k in Ek]) * 1e9
    elev = np.array([math.degrees(elev_rad(s, up, arp)) for s in sat])
    sag = np.array([sagnac_ns(s, arp) for s in sat])
    zhd = saastamoinen_zhd_m(lat, math.sqrt(arp @ arp) - RE)
    trop = np.array([zhd / math.sin(math.radians(max(el, 3))) * M2NS for el in elev])
    iono = np.array([iono_slant_ns(math.radians(max(el, 3))) for el in elev])
    tgd = abs(eph.get("tgd", 0.0)) * 1e9
    mask = elev > 5

    fig, ax = plt.subplots(figsize=(16, 9))
    series = [("troposphere (slant)", np.abs(trop), "tab:red", "-"),
              ("ionosphere (≈20 TECU, repr.)", np.abs(iono), "tab:cyan", "-"),
              ("Sagnac / Earth rotation", np.abs(sag), "tab:purple", "-"),
              ("relativistic eccentricity", np.abs(rel), "tab:orange", "-"),
              (f"group delay TGD ({tgd:.1f} ns)", np.full(tk.shape, max(tgd, 1e-3)), "tab:brown", "--")]
    for lbl, y, col, ls in series:
        ax.plot(th[mask], y[mask], lw=2.6, color=col, ls=ls, label=lbl)
    ax.axhline(15, color="black", lw=3.0, ls=":", label="F9T PPS ≈ 15 ns")
    ax.set_yscale("log")
    ax.set_ylim(0.01, 1000)
    ax.set_xlim(th[mask].min(), th[mask].max())
    ax.set_xlabel(f"hours from reference epoch   (SV {best}, elev {elev[mask].min():.0f}–{elev[mask].max():.0f}°)")
    ax.set_ylabel("|correction applied|  [ns]")
    ax.grid(alpha=0.3, which="both")
    ax.legend(loc="upper right", ncol=2)
    ax.set_title("Corrections the F9T applies to reach 15 ns (one SV over a pass)\n"
                 "log scale — almost every correction towers over the 15 ns PPS floor")
    fig.tight_layout(); fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png   SV={best}  peaks(ns): "
          f"trop={np.abs(trop[mask]).max():.0f} iono={np.abs(iono[mask]).max():.0f} "
          f"sagnac={np.abs(sag[mask]).max():.0f} rel={np.abs(rel[mask]).max():.0f} tgd={tgd:.1f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("eph_json", help="BCEP ephemeris JSON ({prn:[eph,...]})")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--arp-label", default="ufo1")
    ap.add_argument("--antennas-json", default=None)
    args = ap.parse_args()

    ephs = json.load(open(args.eph_json))
    arp, lat, lon = load_arp(args.arp_label, args.antennas_json)
    os.makedirs(args.out_dir, exist_ok=True)
    fig_satclock_table(ephs, os.path.join(args.out_dir, "satcorr_clock_table"))
    fig_combined(ephs, arp, lat, lon, os.path.join(args.out_dir, "satcorr_combined"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
