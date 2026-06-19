#!/usr/bin/env python3
"""Plot the EXTINT ringing mechanism (Bob 2026-06-19) from the three-arm data.

Two panels:
  (left)  bimodal phase-residual histogram: primary mode = normal slightly-
          delayed edge; secondary mode = the trumping edge one ringing period
          later (one-directional, always LATE).
  (right) a zoom of the excursion onset: late EXTINT edges (red) -> adjfine
          cranks up -> chA (pps_err_ticc) ramps off.
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ext = np.genfromtxt(sys.argv[1], delimiter=",", names=True)
srv = np.genfromtxt(sys.argv[2], delimiter=",", names=True)
out = sys.argv[3]

m = ext["count"] > 0
te = ext["host_timestamp"][m]
ph = ext["phase_residual_ns"][m].astype(float)

# detrend with rolling median to expose the per-edge ringing offset
h = 30
trend = np.array([np.median(ph[max(0, i-h):i+h+1]) for i in range(len(ph))])
res = ph - trend

fig, (axh, axt) = plt.subplots(1, 2, figsize=(14, 5))

# --- left: bimodal histogram (symmetric, log y) -------------------------
axh.hist(res[np.abs(res) < 400], bins=160, color="#3b6", log=True)
axh.axvline(0, color="k", lw=0.8)
med_late = np.median(res[res > 30])
axh.axvline(med_late, color="r", ls="--",
            label=f"late mode +{med_late:.0f} ns (ring period)")
axh.set_xlabel("detrended EXTINT phase residual (ns)")
axh.set_ylabel("edges (log)")
axh.set_title("Bimodal EXTINT edges: normal vs trumping (late) edge\n"
              f"baseline delay {np.median(ph):.0f} ns, "
              f"{100*np.mean(res>30):.1f}% late edges, one-directional")
axh.legend()

# --- right: excursion-onset zoom ----------------------------------------
ticc = srv["pps_err_ticc_ns"].astype(float)
adj = srv["adjfine_ppb"].astype(float)
# servo has no host_timestamp; use its own row index scaled to 1 Hz from the
# first/last extint wall time for a rough shared axis.
si = np.arange(len(ticc))
# find the excursion onset (first |ticc| > 1000)
on = np.argmax(np.abs(ticc) > 1000)
lo, hi = max(0, on-400), min(len(ticc), on+800)
ax2 = axt.twinx()
axt.plot(si[lo:hi], ticc[lo:hi]/1000.0, color="#06c", label="chA pps_err_ticc (µs)")
ax2.plot(si[lo:hi], adj[lo:hi], color="#d62", lw=0.8, label="adjfine (ppb)")
axt.set_xlabel("servo epoch (~1 Hz)")
axt.set_ylabel("chA DO error (µs)", color="#06c")
ax2.set_ylabel("adjfine (ppb)", color="#d62")
axt.set_title("Late edges crank adjfine up -> DO excursion")
axt.legend(loc="upper left"); ax2.legend(loc="lower right")

fig.tight_layout()
fig.savefig(out, dpi=110)
print(f"wrote {out}")
print(f"late-edge fraction: {100*np.mean(res>30):.2f}%  "
      f"positive-large/negative-large ratio (>100ns): "
      f"{np.sum(res>100)}/{np.sum(res<-100)}")
