#!/usr/bin/env python3
"""Verify Bob's EXTINT ringing mechanism in the three-arm default-arm data.

Mechanism (Bob, 2026-06-19): the F9P EXTINT input-shaping network adds a
fixed delay AND rings; when the ring dips below logic-low and re-crosses,
EXTINT sees a SECOND edge one ringing-period later.  The later edge trumps
(last-write-wins), so the EKF sees the DO PPS as late -> thinks the DO
slowed -> cranks frequency up.

Predicted signatures:
  1. phase_residual baseline ~ the fixed shaping delay.
  2. bimodal phase: a primary mode (normal edge) + a secondary mode offset
     by the ringing period (the trumping edge).
  3. an adjfine up-crank right after late edges.
"""
import sys
import numpy as np

ext = np.genfromtxt(sys.argv[1], delimiter=",", names=True)
srv = np.genfromtxt(sys.argv[2], delimiter=",", names=True)

# EXTINT: drop the count==0 firmware-duplicate rows (same phase, not ringing).
m = ext["count"] > 0
t = ext["host_timestamp"][m]
ph = ext["phase_residual_ns"][m].astype(float)

# Detrend the slow DO-drift ramp with a centered rolling median (~61 s).
def roll_med(x, w=61):
    h = w // 2
    out = np.empty_like(x)
    for i in range(len(x)):
        out[i] = np.median(x[max(0, i - h):i + h + 1])
    return out

trend = roll_med(ph)
res = ph - trend

print(f"rows (count>0): {len(ph)}")
print(f"phase baseline (median): {np.median(ph):.0f} ns  <- fixed shaping delay")
print(f"detrended residual: median={np.median(res):.1f}  std={np.std(res):.1f} ns")

# Bimodality: histogram the detrended residual, find the secondary (late) mode.
hi = res[res > 30]          # candidate "late" edges, well above normal jitter
print(f"\nlate edges (detrended residual > 30 ns): {len(hi)} "
      f"({100*len(hi)/len(res):.1f}% of edges)")
if len(hi):
    # cluster the late residuals to estimate the ringing period
    print(f"  late-edge residual: median={np.median(hi):.0f} ns "
          f"p10={np.percentile(hi,10):.0f} p90={np.percentile(hi,90):.0f}")
    print(f"  => ringing period estimate ~ {np.median(hi):.0f} ns "
          f"({1e9/np.median(hi)/1e6:.2f} MHz ring)")

# Histogram (text) of detrended residual to show bimodality.
print("\ndetrended phase residual histogram (ns bucket: count):")
edges = [-50, -20, -10, -5, 0, 5, 10, 20, 50, 100, 200, 500, 1000, 1e9]
h, _ = np.histogram(res, bins=edges)
for i in range(len(h)):
    if h[i]:
        bar = "#" * min(60, int(60 * h[i] / h.max()))
        print(f"  [{edges[i]:>6.0f},{edges[i+1]:>7.0f}) {h[i]:5d} {bar}")

# Correlate: adjfine change right after a late edge.  Match servo rows to
# EXTINT late-edge times via the wall clock (servo 'timestamp' is a string;
# use the monotonic-free gps_second alignment is hard, so use index proximity
# on the shared host wall clock from the servo log's own epoch col if present).
# Simpler: report adjfine slope during late-edge-dense vs sparse windows.
adj = srv["adjfine_ppb"].astype(float)
ticc = srv["pps_err_ticc_ns"].astype(float)
print(f"\nservo: adjfine range [{np.nanmin(adj):.1f}, {np.nanmax(adj):.1f}] ppb"
      f"   |pps_err_ticc| max = {np.nanmax(np.abs(ticc)):.0f} ns")
