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

plt.rcParams.update({
    "font.size": 18, "axes.titlesize": 23, "axes.labelsize": 22,
    "xtick.labelsize": 20, "ytick.labelsize": 20, "legend.fontsize": 16,
})


def latest(ephs, prn):
    v = ephs.get(prn)
    return v[-1] if v else None


def clock_poly_us(eph, dt):
    """SV clock polynomial (no relativistic/TGD) in microseconds."""
    return (eph["af0"] + eph["af1"] * dt + eph["af2"] * dt * dt) * 1e6


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


def fig_satclock(ephs, out, span_h=2.0):
    svs = [p for p in sorted(ephs) if p[0] in "GE" and latest(ephs, p)]
    tk = np.linspace(-span_h * 3600, span_h * 3600, 121)
    th = tk / 3600.0
    fig, ax = plt.subplots(figsize=(16, 9))
    peak = {"G": 0.0, "E": 0.0}
    for prn in svs:
        eph = latest(ephs, prn)
        y = np.array([clock_poly_us(eph, t) for t in tk])  # µs
        peak[prn[0]] = max(peak[prn[0]], abs(y).max())
        ax.plot(th, y, lw=1.4, color="tab:blue" if prn[0] == "G" else "tab:green", alpha=0.7)
    ax.axhline(0, color="gray", lw=0.6)
    ax.set_xlim(-span_h, span_h)
    ax.set_xlabel("hours from clock reference epoch (toc)")
    ax.set_ylabel("satellite clock offset from system time  [µs]")
    ax.grid(alpha=0.3)
    ax.plot([], [], color="tab:blue", lw=3, label="GPS")
    ax.plot([], [], color="tab:green", lw=3, label="Galileo")
    ax.legend(loc="upper right")
    gms, ems = peak["G"] / 1e3, peak["E"] / 1e3  # ms
    ax.set_title("Satellite clock correction — each SV's clock offset from system time\n"
                 f"GPS within ±{gms:.2f} ms · Galileo to ±{ems:.1f} ms "
                 f"(≈ ±{peak['E']*0.3:.0f} km uncorrected) — known & broadcast, subtracted exactly")
    ax.text(0.012, 0.045,
            "dt_sv(t) = af0 + af1·(t−toc) + af2·(t−toc)²   ·   1 µs ≈ 300 m of range\n"
            "GPS af0 range ±0.98 ms · Galileo af0 range ±62.5 ms — Galileo doesn't steer SV phase\n"
            "NOT an error: a known broadcast offset; residual after applying ≈ 0.5–1 ns",
            transform=ax.transAxes, fontsize=14, family="monospace",
            bbox=dict(boxstyle="round", fc="white", ec="0.6", alpha=0.92))
    fig.tight_layout(); fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png   GPS peak={peak['G']:.0f}us  GAL peak={peak['E']:.0f}us  ({len(svs)} SVs)")


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
    fig_satclock(ephs, os.path.join(args.out_dir, "satcorr_clock_alone"))
    fig_combined(ephs, arp, lat, lon, os.path.join(args.out_dir, "satcorr_combined"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
