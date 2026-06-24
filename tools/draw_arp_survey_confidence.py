#!/usr/bin/env python3
"""draw_arp_survey_confidence.py — two-panel ARP survey-confidence slide (16:9).

Best/final survey consensus for an ARP, drawn at 1σ:

  LEFT  — plan (top-down) view: ARP at centre, four cardinal points at ±1σ
          N/S and ±1σ E/W (the side-midpoints of the 1σ box), σ as dimension
          arrows in mm.
  RIGHT — elevation view from the south (looking north): ARP at centre, ±1σ
          vertical and ±1σ E/W, σ as dimension arrows in mm.

Both panels share ONE millimetre scale (identical limits + equal aspect).

LLA labels: the first two digits after the decimal are masked (``XX``) so the
exact site isn't disclosed.  Within each coordinate group the common leading
digits are drawn in a normal weight and only the trailing digits that *change*
across the σ range are **bold** — emphasising how many digits the survey
precision actually pins down.

Source: ``timelab/antennas.json`` ⟨uid⟩ (default ``ufo1``: the current
operational ITRF2020 final-orbit OPUS 6-day mean).  1σ = the across-day spread
of the 6 × 24 h solutions (``across_day_sigma_mm``).

Usage:
  draw_arp_survey_confidence.py [--uid ufo1] [--antennas-json PATH] --out-dir DIR
"""
import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

DEFAULT_JSON = "/home/bob/git/timelab/antennas.json"
M_PER_DEG_LAT = 111320.0


# ── digit-emphasis: common digits normal, changing digits bold ──────────────
def _common_prefix(strs):
    s1, s2 = min(strs), max(strs)
    i = 0
    while i < len(s1) and i < len(s2) and s1[i] == s2[i]:
        i += 1
    return s1[:i]


def _mathlabel(value_str, group_strs, suffix_math=""):
    """mathtext: common leading run (normal) + changing tail (bold) + suffix.

    group_strs are the (masked) sibling strings the value is compared against;
    the longest shared prefix is rendered normal, the rest bold."""
    pre = _common_prefix(group_strs)
    tail = value_str[len(pre):]
    pre_m = rf"\mathrm{{{pre}}}" if pre else ""
    tail_m = rf"\mathbf{{{tail}}}" if tail else ""
    return f"${pre_m}{tail_m}{suffix_math}$"


def _mask(value, decimals):
    """abs(value) to `decimals` places with the first two decimals -> 'XX'."""
    s = f"{abs(value):.{decimals}f}"
    ip, fp = s.split(".")
    return f"{ip}.XX{fp[2:]}"


# ── geometry helpers ────────────────────────────────────────────────────────
def _sigma_arrow(ax, p0, p1, label, lx, ly, rot=0, ha="left"):
    """double-headed σ dimension arrow p0->p1 with a boxed mm label."""
    ax.annotate("", xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle="<|-|>", color="#222", lw=1.8,
                                mutation_scale=13), zorder=6)
    ax.text(lx, ly, label, rotation=rot, ha=ha, va="center", fontsize=10.5,
            color="#222", weight="bold", zorder=7,
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.92))


def _panel(ax, L):
    ax.set_xlim(-L, L); ax.set_ylim(-L, L)
    ax.set_aspect("equal")
    ax.grid(True, which="major", color="0.85", lw=0.7)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)


C_ARP = "#c0392b"      # ARP marker (red)
C_PT = "#1f4e79"       # sigma points (blue)
C_BOX = "#1f4e79"      # 1σ box


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uid", default="ufo1")
    ap.add_argument("--antennas-json", default=DEFAULT_JSON)
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--out-name", default="arp_survey_confidence")
    args = ap.parse_args()

    with open(args.antennas_json) as f:
        db = json.load(f)
    e = db[args.uid]
    lat, lon, alt = e["lat"], e["lon"], e["alt_m"]
    sig = e["across_day_sigma_mm"]
    sNS, sEW, sV = sig["lat"], sig["lon"], sig["alt"]      # mm

    # mm -> deg for the cardinal-point LLA labels
    dlat = (sNS / 1000.0) / M_PER_DEG_LAT
    dlon = (sEW / 1000.0) / (M_PER_DEG_LAT * math.cos(math.radians(lat)))
    DEC = 8                                                # decimals shown
    # masked strings per group (for prefix detection + display)
    lat_c, lat_n, lat_s = (_mask(v, DEC) for v in (lat, lat + dlat, lat - dlat))
    lon_c, lon_e, lon_w = (_mask(v, DEC) for v in (lon, lon + dlon, lon - dlon))
    LATG = [lat_c, lat_n, lat_s]
    LONG = [lon_c, lon_e, lon_w]
    ADEC = 4
    alt_c, alt_u, alt_d = (f"{v:.{ADEC}f}" for v in (alt, alt + sV / 1000, alt - sV / 1000))
    ALTG = [alt_c, alt_u, alt_d]
    deg_N = r"^{\circ}\,\mathrm{N}"
    deg_W = r"^{\circ}\,\mathrm{W}"
    m_u = r"\,\mathrm{m}"

    plt.rcParams.update({"font.size": 12})
    fig, (axL, axR) = plt.subplots(1, 2, figsize=(16, 9))
    L = max(sNS, sEW, sV) + 3.2                            # shared half-range, mm
    _panel(axL, L); _panel(axR, L)

    # ════ LEFT: plan view ═══════════════════════════════════════════════════
    axL.add_patch(Rectangle((-sEW, -sNS), 2 * sEW, 2 * sNS, fill=False,
                            ec=C_BOX, lw=1.6, ls=(0, (5, 3)), zorder=3))
    # four cardinal points (side-midpoints of the box)
    axL.scatter([0, 0, sEW, -sEW], [sNS, -sNS, 0, 0], s=46, color=C_PT, zorder=5)
    axL.scatter([0], [0], marker="*", s=320, color=C_ARP, ec="white",
                lw=1.0, zorder=6)
    # cardinal LLA labels (lat for N/S, lon for E/W)
    axL.text(0, sNS + 0.55, _mathlabel(lat_n, LATG, deg_N) + f"\n+{sNS:.2f} mm  N",
             ha="center", va="bottom", fontsize=11)
    axL.text(0, -sNS - 0.55, _mathlabel(lat_s, LATG, deg_N) + f"\n−{sNS:.2f} mm  S",
             ha="center", va="top", fontsize=11)
    axL.text(sEW + 0.4, 0, _mathlabel(lon_e, LONG, deg_W) + f"\n+{sEW:.2f} mm  E",
             ha="left", va="center", fontsize=11)
    axL.text(-sEW - 0.4, 0, _mathlabel(lon_w, LONG, deg_W) + f"\n−{sEW:.2f} mm  W",
             ha="right", va="center", fontsize=11)
    # dimension arrows (σ = centre→point), drawn on the axes through the ARP
    _sigma_arrow(axL, (0, 0), (0, sNS), f"σ N/S\n{sNS:.2f} mm", 0.35, sNS * 0.55)
    _sigma_arrow(axL, (0, 0), (sEW, 0), f"σ E/W = {sEW:.2f} mm",
                 sEW * 0.5, -0.7, ha="center")
    # ARP corner block
    arp_block = ("ARP  (1σ box centre)\n"
                 + _mathlabel(lat_c, LATG, deg_N) + "\n"
                 + _mathlabel(lon_c, LONG, deg_W) + "\n"
                 + _mathlabel(alt_c, ALTG, m_u))
    axL.text(-L + 0.4, L - 0.4, arp_block, ha="left", va="top", fontsize=10.5,
             bbox=dict(boxstyle="round,pad=0.4", fc="#fdf3f2", ec=C_ARP, lw=1.2))
    axL.text(0, 0.0, "  ✦ ARP", ha="left", va="bottom", fontsize=10,
             color=C_ARP, weight="bold")
    axL.set_xlabel("East  →   [mm]"); axL.set_ylabel("North  ↑   [mm]")
    axL.set_title("Plan view  (top-down)", fontsize=14, weight="bold")
    axL.text(L - 0.3, -L + 0.3, "N ↑   E →", ha="right", va="bottom",
             fontsize=10, color="0.4", style="italic")

    # ════ RIGHT: elevation view from the south ══════════════════════════════
    axR.add_patch(Rectangle((-sEW, -sV), 2 * sEW, 2 * sV, fill=False,
                            ec=C_BOX, lw=1.6, ls=(0, (5, 3)), zorder=3))
    axR.scatter([0, 0, sEW, -sEW], [sV, -sV, 0, 0], s=46, color=C_PT, zorder=5)
    axR.scatter([0], [0], marker="*", s=320, color=C_ARP, ec="white",
                lw=1.0, zorder=6)
    axR.text(0, sV + 0.55, _mathlabel(alt_u, ALTG, m_u) + f"\n+{sV:.2f} mm  up",
             ha="center", va="bottom", fontsize=11)
    axR.text(0, -sV - 0.55, _mathlabel(alt_d, ALTG, m_u) + f"\n−{sV:.2f} mm  down",
             ha="center", va="top", fontsize=11)
    axR.text(sEW + 0.4, 0, _mathlabel(lon_e, LONG, deg_W) + f"\n+{sEW:.2f} mm  E",
             ha="left", va="center", fontsize=11)
    axR.text(-sEW - 0.4, 0, _mathlabel(lon_w, LONG, deg_W) + f"\n−{sEW:.2f} mm  W",
             ha="right", va="center", fontsize=11)
    _sigma_arrow(axR, (0, 0), (0, sV), f"σ vert\n{sV:.2f} mm", 0.35, sV * 0.55)
    _sigma_arrow(axR, (0, 0), (sEW, 0), f"σ E/W = {sEW:.2f} mm",
                 sEW * 0.5, -0.7, ha="center")
    arp_block_r = ("ARP  (1σ box centre)\n"
                   + _mathlabel(alt_c, ALTG, m_u) + "  ellipsoidal")
    axR.text(-L + 0.4, L - 0.4, arp_block_r, ha="left", va="top", fontsize=10.5,
             bbox=dict(boxstyle="round,pad=0.4", fc="#fdf3f2", ec=C_ARP, lw=1.2))
    axR.text(0, 0.0, "  ✦ ARP", ha="left", va="bottom", fontsize=10,
             color=C_ARP, weight="bold")
    axR.set_xlabel("East  →   [mm]"); axR.set_ylabel("Up / elevation  ↑   [mm]")
    axR.set_title("Elevation view  (from the south, looking north)",
                  fontsize=14, weight="bold")

    # ── title + emphasis note + provenance ──────────────────────────────────
    fig.suptitle(f"{args.uid.upper()} antenna reference point — survey confidence (1σ)",
                 fontsize=20, weight="bold")
    fig.text(0.5, 0.905,
             "common leading digits in normal weight · "
             r"$\mathbf{bold}$ = digits that change across the 1σ range · "
             "first two decimals masked (XX)",
             ha="center", va="center", fontsize=11.5, color="#444")

    frame = e.get("frame", "").replace("@", " @ ")
    method = e.get("method", "")
    ant = e.get("antenna", "")
    cc = e.get("cross_check", {})
    pride = next((v.get("horiz_mm") for k, v in cc.items() if "PRIDE" in k), None)
    f9p = next((v.get("horiz_mm") for k, v in cc.items() if "F9P" in k), None)
    xchk = []
    if pride is not None:
        xchk.append(f"PRIDE PPP-AR {pride:g} mm horiz")
    if f9p is not None:
        xchk.append(f"F9P CORS-RTK {f9p:g} mm horiz")
    prov = (f"{frame}  ·  {method}  ·  Leica GRX1200 GG Pro reference + "
            f"{ant} (NGS-calibrated PCO/PCV)\n"
            f"1σ = across-day spread of 6 × 24 h OPUS-Static solutions  "
            f"(N/S {sNS:.2f} mm, E/W {sEW:.2f} mm, vertical {sV:.2f} mm)"
            + ("   ·   cross-checks: " + ", ".join(xchk) if xchk else ""))
    fig.text(0.5, 0.045, prov, ha="center", va="center", fontsize=11,
             color="#333")

    fig.tight_layout(rect=[0, 0.085, 1, 0.89])
    os.makedirs(args.out_dir, exist_ok=True)
    out = os.path.join(args.out_dir, args.out_name)
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png   "
          f"(σ N/S {sNS} / E/W {sEW} / vert {sV} mm, L={L:.1f} mm)")


if __name__ == "__main__":
    main()
