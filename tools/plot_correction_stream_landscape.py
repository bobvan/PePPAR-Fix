#!/usr/bin/env python3
"""Plot the real-time GNSS correction landscape: how often a correction arrives
vs how well it pins the satellite clock + orbit.

The thesis: **real-time accuracy is all about the corrections.**  A GNSS receiver's
raw observations are identical no matter what you feed it; what separates a
100 ns consumer fix from a sub-ns disciplined clock is entirely the quality and
freshness of the satellite clock/orbit/bias corrections applied on top.

Axes
----
x : correction update rate, as the interval between updates (log seconds),
    **reversed** so that more
    frequent updates are to the RIGHT.
y : residual line-of-sight satellite clock+orbit time error (log ns), with the
    equivalent range error in metres on the twin axis.  Down = better.
    (1 ns = 0.2998 m; c-equivalence, so the twin axis is a pure unit relabel of
    the SAME measure -- not a second measure.  That is the one legitimate use of
    a twin y-axis.)

So the good corner is **bottom-right**, and the eye should travel down-and-right
from the broadcast cluster to the real-time-correction cluster.

Why two regions and not one fitted trend line
---------------------------------------------
A regression through all seven points would assert something false.  Within the
broadcast tier, update rate is NOT what sets accuracy -- Galileo updates
every 10 min and lands at 0.22 ns while GLONASS updates every 30 min and lands
at 3.82 ns, a 17x gap driven by the onboard frequency standard (Galileo passive
H-maser vs GLONASS caesium) and the ground segment, not by update rate.  The real
effect is BETWEEN tiers: leaving the broadcast nav message for a real-time
correction stream buys both an order of magnitude in freshness and a tighter,
more complete correction.  So the trend is drawn as two soft regions plus one
arrow, which is the claim the data actually supports.

Measured inputs (all from captures archived on gt)
--------------------------------------------------
- gt/ssr-capture/cnes-2026-06-11/  -- 24 h CNES SSRA00CNE0 capture.  Per-SV
  clock c0 + orbit radial, detrended, gives the broadcast error each
  constellation carries (the correction IS the broadcast error it removes).
- gt/ssr-capture/has-compare-20260613/  -- concurrent BKG SSR + Galileo HAS
  E6-B signal-in-space: HAS arrives every 10 s, BKG/CNES every 5 s.

Scope: real-time sources only.  Post-processed IGS rapid/final products are
deliberately absent -- they are better than anything here but arrive hours to
weeks late, so they are not options for a live clock.

Output: docs/correction-stream-landscape.{png,svg}
"""
from __future__ import annotations

import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.ticker import LogLocator, NullFormatter

VERSION = "plot_correction_stream_landscape v4 (2026-08-11)"
SOURCES = "data: gt/ssr-capture/{cnes-2026-06-11, has-compare-20260613}"

C_MPS = 2.99792458e8          # m/s -- ns <-> m relabel of the same axis
NS_PER_M = 1e9 / C_MPS        # 3.3356 ns per metre

# Correction completeness -- sequential, one hue, light -> dark.
COMPLETENESS = {
    "nav":   ("#bfdbfe", "Broadcast nav message only\n(orbit + clock, coarse)"),
    "oc_cb": ("#3b82f6", "Precise orbit + clock + code bias\n(float PPP)"),
    "oc_pb": ("#172554", "…and phase bias\n(single AC → PPP-AR capable)"),
}

DELIVERY = {
    "sis": ("^", "Over the air — no internet needed"),
    "net": ("o", "Internet NTRIP"),
}

EDGE = "#1e293b"
INK = "#0f172a"

# ---------------------------------------------------------------------------
# x   : interval between new corrections, seconds
# y   : residual LOS clock+orbit time error, ns (p50)
# y95 : p95 of the same -> vertical whisker
# lx,ly,ha,va : label anchor in data coords; a leader line joins it to the point
# ---------------------------------------------------------------------------
POINTS = [
    # Broadcast navigation messages.  y from the 24 h CNES capture (per-SV
    # detrended clock c0 + orbit radial); x is the ICD ephemeris update rate.
    dict(label="GPS LNAV\nbroadcast", x=7200, y=0.44, y95=1.52,
         comp="nav", deliv="sis", lx=7200, ly=0.285, ha="center", va="top"),
    dict(label="BeiDou-3\nbroadcast", x=3600, y=0.69, y95=2.70,
         comp="nav", deliv="sis", lx=3600, ly=1.30, ha="center", va="bottom"),
    dict(label="GLONASS\nbroadcast", x=1800, y=3.82, y95=28.8,
         comp="nav", deliv="sis", lx=1000, ly=6.5, ha="left", va="center"),
    dict(label="Galileo I/NAV\nbroadcast", x=600, y=0.22, y95=0.73,
         comp="nav", deliv="sis", lx=600, ly=0.128, ha="center", va="top"),

    # Real-time correction streams.
    dict(label="Galileo HAS\n(E6-B signal)", x=10, y=0.17,
         comp="oc_cb", deliv="sis", lx=70, ly=0.30, ha="right", va="center"),
    dict(label="IGS-RTS combined\n(no phase bias)", x=5, y=0.15,
         comp="oc_cb", deliv="net", lx=70, ly=0.143, ha="right", va="center"),
    dict(label="Single-AC SSR\n(CNES · CAS · WHU · AtomiChron)", x=5, y=0.09,
         comp="oc_pb", deliv="net", lx=70, ly=0.070, ha="right", va="center"),
]

# The two regimes, as soft background regions.
REGIONS = [
    dict(x=(440, 11000), y=(0.155, 6.4), colour="#64748b",
         label="Broadcast navigation message\nupdated every 10 min – 2 h",
         tx=5000, ty=0.100, va="top"),
    dict(x=(3.5, 17.0), y=(0.066, 0.255), colour="#2563eb",
         label="Real-time correction stream\nupdated every 5–10 s",
         tx=11.0, ty=0.42, va="bottom"),
]

XLIM = (1.3e4, 3.1)      # reversed: staler on the left, fresher on the right
YLIM = (0.055, 45.0)


def _fmt_interval(s: float) -> str:
    if s < 60:
        return f"{s:g} s"
    if s < 3600:
        return f"{s/60:.0f} min"
    return f"{s/3600:.0f} h"


def _log_blob(x_lo, x_hi, y_lo, y_hi, n=200):
    """Soft-cornered region (superellipse) spanning a box in log-log space."""
    th = np.linspace(0, 2 * np.pi, n)
    xc, yc = np.sqrt(x_lo * x_hi), np.sqrt(y_lo * y_hi)
    xw, yh = np.log10(x_hi / x_lo) / 2, np.log10(y_hi / y_lo) / 2
    c, s = np.cos(th), np.sin(th)
    e = 0.32          # smaller -> boxier; corners stay soft
    x = 10 ** (np.log10(xc) + xw * np.sign(c) * np.abs(c) ** e)
    y = 10 ** (np.log10(yc) + yh * np.sign(s) * np.abs(s) ** e)
    return x, y


def main() -> None:
    fig, ax = plt.subplots(figsize=(13.0, 8.6))
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(*XLIM)
    ax.set_ylim(*YLIM)

    # ---- the two regimes -------------------------------------------------
    for r in REGIONS:
        x, y = _log_blob(*r["x"], *r["y"])
        ax.fill(x, y, facecolor=r["colour"], alpha=0.10, edgecolor=r["colour"],
                linewidth=1.2, linestyle=(0, (6, 4)), zorder=0)
        ax.text(r["tx"], r["ty"], r["label"], ha="center", va=r["va"],
                fontsize=10, color=r["colour"], fontweight="bold",
                linespacing=1.35, zorder=1)

    # ---- the trend the data actually supports ----------------------------
    ax.annotate("", xy=(26, 0.31), xytext=(300, 1.75),
                arrowprops=dict(arrowstyle="-|>,head_width=0.42,head_length=0.8",
                                color="#0f766e", lw=3.2, alpha=0.5,
                                shrinkA=0, shrinkB=0), zorder=1)
    ax.text(72, 1.22,
            "more frequent, more complete\n→  better time",
            ha="center", va="center", fontsize=12, color="#0f766e",
            fontweight="bold", linespacing=1.4, rotation=-25, zorder=2)

    # ---- the points ------------------------------------------------------
    for p in POINTS:
        colour = COMPLETENESS[p["comp"]][0]
        marker = DELIVERY[p["deliv"]][0]

        if p.get("y95"):
            ax.plot([p["x"], p["x"]], [p["y"], p["y95"]], color=colour, lw=1.5,
                    alpha=0.55, solid_capstyle="butt", zorder=3)
            ax.plot([p["x"] * 0.88, p["x"] * 1.14], [p["y95"], p["y95"]],
                    color=colour, lw=1.5, alpha=0.55, zorder=3)

        ax.annotate("", xy=(p["x"], p["y"]), xytext=(p["lx"], p["ly"]),
                    arrowprops=dict(arrowstyle="-", color="#94a3b8", lw=0.8,
                                    shrinkA=2, shrinkB=10), zorder=2)

        ax.plot(p["x"], p["y"], marker=marker, ms=15,
                markerfacecolor=colour, markeredgecolor=EDGE,
                markeredgewidth=1.7, zorder=5, linestyle="none")

        ax.text(p["lx"], p["ly"], p["label"], ha=p["ha"], va=p["va"],
                fontsize=9.4, color=INK, linespacing=1.3, zorder=6)

    # ---- axes ------------------------------------------------------------
    xticks = [10800, 7200, 3600, 1800, 600, 300, 60, 30, 10, 5]
    ax.set_xticks(xticks)
    ax.set_xticklabels([_fmt_interval(t) for t in xticks], fontsize=10)
    ax.xaxis.set_minor_locator(LogLocator(base=10, subs=range(2, 10)))
    ax.xaxis.set_minor_formatter(NullFormatter())

    yticks = [0.06, 0.1, 0.3, 1.0, 3.0, 10.0, 30.0]
    ax.set_yticks(yticks)
    ax.set_yticklabels([f"{v:g}" for v in yticks], fontsize=10)
    ax.yaxis.set_minor_locator(LogLocator(base=10, subs=range(2, 10)))
    ax.yaxis.set_minor_formatter(NullFormatter())

    ax.set_xlabel("Correction update rate          "
                  "more frequent  ▶", fontsize=12, labelpad=10)
    ax.set_ylabel("Satellite clock + orbit time error  (ns)\nlower is better",
                  fontsize=12, labelpad=10)
    ax.grid(True, which="major", alpha=0.4, lw=0.8, color="#94a3b8")
    ax.grid(True, which="minor", alpha=0.1, lw=0.4, ls=":", color="#94a3b8")
    ax.set_axisbelow(True)

    # Twin y: the SAME measure, relabelled in metres of range.
    ax2 = ax.twinx()
    ax2.set_yscale("log")
    ax2.set_ylim(YLIM[0] / NS_PER_M, YLIM[1] / NS_PER_M)
    m_ticks = [0.02, 0.05, 0.1, 0.3, 1.0, 3.0, 10.0]
    ax2.set_yticks(m_ticks)
    ax2.set_yticklabels([(f"{v*100:g} cm" if v < 1 else f"{v:g} m")
                         for v in m_ticks], fontsize=10)
    ax2.set_ylabel("equivalent range error   (1 ns = 30 cm)", fontsize=11,
                   color="#475569", labelpad=10)
    ax2.tick_params(axis="y", colors="#475569")
    ax2.yaxis.set_minor_locator(LogLocator(base=10, subs=range(2, 10)))
    ax2.yaxis.set_minor_formatter(NullFormatter())

    ax.set_title("Real-time accuracy is all about the corrections",
                 fontsize=16, fontweight="bold", pad=14)

    # ---- legends, below the axes so they never sit on data ---------------
    comp_handles = [
        Line2D([], [], marker="o", ls="none", ms=12, markerfacecolor=c,
               markeredgecolor=EDGE, markeredgewidth=1.4, label=lbl)
        for c, lbl in COMPLETENESS.values()
    ]
    leg1 = fig.legend(handles=comp_handles, title="What the correction carries",
                      loc="upper left", bbox_to_anchor=(0.085, 0.205),
                      fontsize=9, title_fontsize=9.8, framealpha=0.0,
                      handletextpad=0.7, labelspacing=0.6, borderpad=0.4)
    leg1.get_title().set_fontweight("bold")

    deliv_handles = [
        Line2D([], [], marker=m, ls="none", ms=12, markerfacecolor="#94a3b8",
               markeredgecolor=EDGE, markeredgewidth=1.4, label=lbl)
        for m, lbl in DELIVERY.values()
    ]
    leg2 = fig.legend(handles=deliv_handles, title="How it reaches you",
                      loc="upper left", bbox_to_anchor=(0.60, 0.205),
                      fontsize=9, title_fontsize=9.8, framealpha=0.0,
                      handletextpad=0.7, labelspacing=0.6, borderpad=0.4)
    leg2.get_title().set_fontweight("bold")

    fig.text(0.5, 0.014,
             f"{VERSION}   ·   {SOURCES}\n"
             "Real-time sources only.  Error bars run p50→p95 across all SVs of "
             "the constellation over 24 h.",
             ha="center", fontsize=7.6, color="#64748b", linespacing=1.5)

    out_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), os.pardir, "docs"))
    base = os.path.join(out_dir, "correction-stream-landscape")
    fig.subplots_adjust(left=0.085, right=0.905, top=0.925, bottom=0.265)
    for ext, kw in (("png", dict(dpi=150)), ("svg", {})):
        fig.savefig(f"{base}.{ext}", **kw)
        print(f"wrote {base}.{ext}")


if __name__ == "__main__":
    main()
