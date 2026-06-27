#!/usr/bin/env python3
"""Servo INPUT precision vs realized OUTPUT vs DO free-run — one honest axis.

Overlays, as TDEV(τ), the three things that bound a disciplined clock, so the
moonshot framing is visible at a glance:

  1. DO free-run (OCXO, DAC-parked vs TICC-Rb)  — the open-loop oscillator floor
  2. TDCP carrier-phase servo INPUT precision    — per-epoch, NO DO ("tens of ps")
  3. Realized PPS OUTPUT (chA, disciplined)      — where we actually land

Plus two context curves: our PPP time solution (chA−φDO) and the de-sawtoothed
F9T SPP+carrier-smoothing reference (chB+qErr) — the honest mid-τ bar.

The caveats are printed ON the figure on purpose: the "tens of ps" is a
measurement PRECISION floor (averages down with τ), NOT disciplined-clock
stability; the rx TCXO the carrier phase measures itself walks.  chA is the
honest output (DO + actuator + loop); its mid-τ hump is the realization
residual (midTauTrackingResidual / softGateMidTau).

Data (all on gt):
  - DO free-run:  ~/gt/datasheets/do-freerun-compare/<host>.noise.toml
  - realized/PPP/de-sawtooth:  ~/gt/stepb-qerr-20260623/<host>-{ticc,arm,qerr}.csv
  - TDCP per-epoch σ_trim band: docs/receiver-comparison-2026-06-01.md (37–109 ps)

TDEV method is the validated stepb analyzer (linear-detrend, 300 s startup
trim, last 1800 s, octave τ).
"""
import csv, os, subprocess, tomllib
import numpy as np
import allantools
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ANALYSIS_VERSION = "servo-io-overlay-1"
STEPB = os.path.expanduser("~/gt/stepb-qerr-20260623")
NOISE = os.path.expanduser("~/gt/datasheets/do-freerun-compare")
OUT = os.path.expanduser("~/gt/datasheets/servo-io-overlay.png")
HOST = "madhat"            # F9T-20-00 + OCXO — cleanest representative
HOST_RX = "MadHat — F9T-20-00 (L1/L5) + OCXO/AD5693R"
SIGMA_TRIM_PS = (37, 109)  # TDCP per-epoch precision range across receivers
BUDGET_SHARED_PS = 354      # per-clock budget — shared antenna (1 ns excursion bound)
BUDGET_SEPARATE_PS = 1000   # per-clock budget — separate antennas (2 ns excursion bound)

HEAD_SKIP, WIN = 300, 1800


def _grid(sec, phase_ps):
    sec = np.asarray(sec); phase_ps = np.asarray(phase_ps, float)
    o = np.argsort(sec); sec, phase_ps = sec[o], phase_ps[o]
    keep = np.concatenate(([True], np.diff(sec) > 0))
    sec, phase_ps = sec[keep], phase_ps[keep]
    sec0 = sec.min() + HEAD_SKIP
    m = sec >= sec0; sec, phase_ps = sec[m], phase_ps[m]
    if sec.size and (sec.max() - sec.min()) > WIN:
        m = sec >= sec.max() - WIN; sec, phase_ps = sec[m], phase_ps[m]
    grid = np.arange(sec.min(), sec.max() + 1)
    ph = np.interp(grid, sec, phase_ps)
    ph -= np.polyval(np.polyfit(grid - grid[0], ph, 1), grid - grid[0])
    return grid, ph


def _tdev(phase_ps):
    taus, td, _, _ = allantools.tdev(phase_ps * 1e-12, rate=1.0,
                                     data_type="phase", taus="octave")
    return taus, td * 1e12  # ps


def load_ticc(path):
    d = {"chA": ([], [], []), "chB": ([], [], [])}
    for r in csv.DictReader(open(path)):
        ch = r["channel"]
        if ch in d:
            d[ch][0].append(int(r["ref_sec"])); d[ch][1].append(int(r["ref_ps"]))
            d[ch][2].append(float(r["host_monotonic"]))
    out = {}
    for ch, (s, p, m) in d.items():
        s = np.array(s); out[ch] = dict(sec=s, mono=np.array(m),
                                         phase_ps=s.astype(float) * 1e12 + np.array(p, float))
    return out


def load_arm(path):
    mono, phi = [], []
    for r in csv.DictReader(open(path)):
        try:
            mono.append(float(r["host_monotonic"])); phi.append(float(r["x2_phi_do_ns"]) * 1e3)
        except (ValueError, KeyError):
            pass
    return np.array(mono), np.array(phi)


def load_qerr(path):
    sec, ps, qm = [], [], []
    for r in csv.DictReader(open(path)):
        try:
            sec.append(int(r["chb_ref_sec"])); ps.append(int(r["chb_ref_ps"]))
            qm.append(float(r["qerr_matched_ns"]))
        except (ValueError, KeyError):
            pass
    sec = np.array(sec)
    return sec, sec.astype(float) * 1e12 + np.array(ps, float) + np.array(qm, float) * 1e3


# --- build curves -------------------------------------------------------- #
t = load_ticc(f"{STEPB}/{HOST}-ticc.csv")
am, aphi = load_arm(f"{STEPB}/{HOST}-arm.csv")

curves = []  # (label, taus, td_ps, style)
g, chA = _grid(t["chA"]["sec"], t["chA"]["phase_ps"])
curves.append(("Realized PPS output  (chA, disciplined DO)", *_tdev(chA),
               dict(color="tab:red", marker="o", ls="-", lw=2.2)))

phi_at = np.interp(t["chA"]["mono"], am, aphi)
g2, sw = _grid(t["chA"]["sec"], t["chA"]["phase_ps"] - phi_at)
curves.append(("Our PPP time solution  (chA − φDO)", *_tdev(sw),
               dict(color="tab:green", marker="s", ls="-", lw=1.6)))

qs, qph = load_qerr(f"{STEPB}/{HOST}-qerr.csv")
gq, ds = _grid(qs, qph)
curves.append(("F9T SPP + carrier-smooth  (de-sawtoothed, no DO)", *_tdev(ds),
               dict(color="tab:orange", marker="^", ls="--", lw=1.6)))

nd = tomllib.load(open(f"{NOISE}/{HOST.replace('madhat','MadHat')}.noise.toml", "rb"))
dt = nd["tdev_ns_by_tau_s"]
do_t = np.array([float(k) for k in dt]); do_v = np.array([float(dt[k]) for k in dt]) * 1e3
order = np.argsort(do_t)
curves.append(("DO free-run  (OCXO, DAC-parked vs Rb)", do_t[order], do_v[order],
               dict(color="tab:blue", marker="o", ls="-", lw=2.2)))

# --- plot (16:9 projection slide; larger fonts except caveats/footer) ---- #
fig, ax = plt.subplots(figsize=(13.33, 7.5))
for label, taus, td, st in curves:
    st = dict(st); st["lw"] = st["lw"] + 0.8
    ax.loglog(taus, td, label=label, alpha=0.9, markersize=8, **st)

# TDCP per-epoch input-precision band ("the tens of ps")
ax.axhspan(*SIGMA_TRIM_PS, color="tab:green", alpha=0.12, zorder=0)
ax.axhline(SIGMA_TRIM_PS[0], color="tab:green", ls=":", lw=1.3)
ax.axhline(SIGMA_TRIM_PS[1], color="tab:green", ls=":", lw=1.3)
ax.text(1.05, SIGMA_TRIM_PS[1] * 1.04,
        "TDCP carrier-phase per-epoch precision (no DO): %d–%d ps  ← the “wow” number"
        % SIGMA_TRIM_PS, fontsize=14, color="darkgreen")

ax.axhline(BUDGET_SHARED_PS, color="gray", ls="--", lw=1.3)
ax.text(120, BUDGET_SHARED_PS * 1.04,
        "354 ps per-clock budget (shared antenna)", fontsize=14, color="gray")
ax.axhline(BUDGET_SEPARATE_PS, color="gray", ls=":", lw=1.6)
ax.text(120, BUDGET_SEPARATE_PS * 1.04,
        "1.0 ns per-clock budget (separate antennas)", fontsize=14, color="gray")

ax.set_xlabel("τ (s)", fontsize=18)
ax.set_ylabel("TDEV (ps)", fontsize=18)
ax.set_title("Servo input precision vs realized output vs DO free-run\n%s" % HOST_RX,
             fontsize=20)
ax.tick_params(which="both", labelsize=15)
ax.grid(True, which="both", alpha=0.3)
ax.legend(fontsize=14, loc="upper left", framealpha=0.95)
ax.set_ylim(15, 3e4)

caveats = (
    "CAVEATS — state these with the number:\n"
    "• TDCP band & chA−φDO are receiver carrier-phase, NO DO / NO TICC — the servo INPUT\n"
    "  precision, not a disciplined output. White-PM floor (averages DOWN with τ); the rx\n"
    "  TCXO it measures itself WALKS to tens of ns (not shown).\n"
    "• chA is the honest realized OUTPUT (DO + actuator + loop). The mid-τ hump = servo\n"
    "  realization residual — midTauTrackingResidual (I-102153) / softGateMidTau.\n"
    "• DO free-run = DAC-parked OCXO vs TICC-Rb (truly open-loop floor).\n"
    "• Moonshot target ≈ lower envelope of (DO free-run, input precision); the gap above it\n"
    "  — especially mid-τ — is the work that remains."
)
ax.text(0.985, 0.025, caveats, transform=ax.transAxes, fontsize=7.4,
        ha="right", va="bottom", family="monospace",
        bbox=dict(boxstyle="round", fc="#fffbe6", ec="gray", alpha=0.95))

sha = subprocess.getoutput("git -C %s rev-parse --short HEAD" % os.path.dirname(__file__))
fig.text(0.5, 0.005, "ANALYSIS_VERSION=%s  git=%s  data: stepb-qerr-20260623 + "
         "do-freerun-compare + receiver-comparison-2026-06-01" % (ANALYSIS_VERSION, sha),
         ha="center", fontsize=6.5, color="gray")
fig.tight_layout(rect=(0, 0.02, 1, 1))
fig.savefig(OUT, dpi=125)
OUT_PDF = os.path.splitext(OUT)[0] + ".pdf"
fig.savefig(OUT_PDF)
print("wrote", OUT, "and", OUT_PDF)
for label, taus, td, _ in curves:
    g1 = lambda T: td[int(np.argmin(np.abs(taus - T)))]
    print("  %-46s τ1=%4.0f τ10=%5.0f τ100=%6.0f ps" % (label, g1(1), g1(10), g1(100)))
