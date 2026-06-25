#!/usr/bin/env python3
"""pos_sim — synthetic closed-loop simulator for the POSITION filter.

Sibling of ``servo_sim`` (the time/servo filter): a constructed ground
truth — a static ARP, a residual ZTD, a receiver clock, and per-SV float
ambiguities — emits synthetic IF code+carrier observations, and the
**real** :class:`solve_ppp.FixedPosFilter` runs against them.  Truth is
exact, so we can **dial the (pos, ZTD, clk) null** and watch whether the
filter holds the answer or slides along the unobservable direction with
shrinking (false-confident) σ.

Faithfulness (same principle as ``servo_sim``): the emitter calls the
filter's *own* ``geometric_range()`` / ``tropo_delay()`` / ``wet_mapping()``,
so the observation it emits is ``z = h(truth)`` for the **same** ``h()``
the filter inverts.  Only the plant (truth + sky geometry) and the loop
are new.  A bug in ``h()`` is shared by emitter and filter and is
therefore invisible here — that is intended: ``pos_sim`` tests the
ESTIMATOR given the model; catching a wrong *model* is ``pos_replay``'s
job.  See docs/simulators-and-replay.md.

First-cut scope (per the design doc): static ARP, synthetic sky geometry
(dialable strong/weak), IF-float (no ambiguity resolution), no SSR biases
(obs are emitted already-corrected).  The null lives in the
geometry+clk+tropo+ZTD core, which is exactly what this exercises.

The canonical demo is the **(up, ZTD, clock) degeneracy**: a sky of only
high-elevation satellites gives every line-of-sight ≈ straight up, so the
up-position, the zenith wet delay (m_wet≈1), and the receiver clock become
nearly collinear.  Seed the position with an up-offset and a
high-elevation-only sky cannot correct it; a sky with low-elevation
satellites can.
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone

import numpy as np

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from solve_ppp import (  # noqa: E402
    PPPFilter, C, IDX_CLK, IDX_ZTD, IDX_ISB_GAL, IDX_ISB_BDS)

GPS_RADIUS_M = 20_200_000.0 + 6_378_137.0   # ECEF radius of a GPS-ish SV
_ISB_IDX = {"gps": None, "gal": IDX_ISB_GAL, "bds": IDX_ISB_BDS}


# ── local frame ─────────────────────────────────────────────────────── #

def enu_basis(arp_ecef):
    """East/North/Up unit vectors (ECEF) at an ARP."""
    up = arp_ecef / np.linalg.norm(arp_ecef)
    east = np.array([-arp_ecef[1], arp_ecef[0], 0.0])
    east /= np.linalg.norm(east)
    north = np.cross(up, east)
    return east, north, up


def azel_to_ecef(arp_ecef, az_deg, el_deg, radius=GPS_RADIUS_M):
    """ECEF position of a satellite placed at (az, el) seen from the ARP.

    The realized elevation the filter computes is slightly below ``el_deg``
    (finite range), which is fine — placement only needs to put SVs in the
    sky; emitter and filter agree because both use ``geometric_range``.
    """
    east, north, up = enu_basis(arp_ecef)
    az, el = math.radians(az_deg), math.radians(el_deg)
    direction = (math.cos(el) * (math.sin(az) * east + math.cos(az) * north)
                 + math.sin(el) * up)
    return arp_ecef + radius * direction


# ── synthetic sky (the sp3 provider the filter consumes) ────────────── #

class SyntheticSky:
    """Static synthetic constellation exposing the ``sp3`` contract
    (``sat_position(sv, t) -> (ecef, clk_seconds)``) the filter calls."""

    def __init__(self, arp_ecef, sats, sat_clk_s=0.0):
        # sats: list of (sv_name, sys, az_deg, el_deg)
        self.sys_of = {}
        self._pos = {}
        self.sat_clk_s = float(sat_clk_s)
        for name, sysname, az, el in sats:
            self.sys_of[name] = sysname
            self._pos[name] = azel_to_ecef(np.asarray(arp_ecef, float), az, el)

    @property
    def svs(self):
        return list(self.sys_of)

    def sat_position(self, sv, t):
        pos = self._pos.get(sv)
        if pos is None:
            return None, None
        return pos, self.sat_clk_s


# ── ground truth ────────────────────────────────────────────────────── #

@dataclass
class Truth:
    """Exact truth the emitter realises each epoch."""
    arp_ecef: np.ndarray                       # static ARP (ECEF, m)
    clk0_m: float = 0.0                        # receiver clock at t0 (m)
    clk_rate_m_s: float = 0.0                  # clock drift (m/s) — a null axis
    ztd0_m: float = 0.0                        # residual wet ZTD at t0 (m)
    ztd_rate_m_s: float = 0.0                  # ZTD drift (m/s) — a null axis
    isb_m: dict = field(default_factory=dict)  # sys -> ISB (m), gps implicitly 0
    amb_m: dict = field(default_factory=dict)  # sv -> float ambiguity (m)

    def clk(self, t_s):
        return self.clk0_m + self.clk_rate_m_s * t_s

    def ztd(self, t_s):
        return self.ztd0_m + self.ztd_rate_m_s * t_s


def emit(filt, sky, truth, t_s, t_dt, rng, sigma_code_m=0.3, sigma_carrier_m=0.005,
         cno=45.0, bias_fn=None):
    """Emit one epoch of synthetic obs via the filter's OWN forward model.

    ``bias_fn(sv, sys, elev_deg) -> metres`` injects an UNMODELED per-SV range
    error (orbit/clock/correction error or multipath) that the filter does
    NOT know about — i.e. a deliberate forward-MODEL error.  This breaks the
    shared-h() faithfulness on purpose, to study how the estimator routes an
    observation-side error (the realm pos_replay carries for real).  Default
    None = faithful (no injected error).
    """
    obs = []
    clk = truth.clk(t_s)
    ztd = truth.ztd(t_s)
    for sv in sky.svs:
        sat_pos, sat_clk = sky.sat_position(sv, t_dt)
        rho, _e_los, elev, _sat_rot = filt.geometric_range(truth.arp_ecef, sat_pos)
        if elev < 0.0:
            continue
        tropo = filt.tropo_delay(elev)
        m_wet = filt.wet_mapping(elev)
        sysname = sky.sys_of[sv]
        isb = truth.isb_m.get(sysname, 0.0) if sysname != "gps" else 0.0
        # The exact observable the filter inverts (h(truth)):
        rho_pred = rho + clk + isb - sat_clk * C + tropo + ztd * m_wet
        if bias_fn is not None:
            rho_pred += bias_fn(sv, sysname, elev)   # unmodeled obs-side error
        amb = truth.amb_m.setdefault(sv, rng.uniform(-1e6, 1e6))
        obs.append({
            "sv": sv,
            "sys": sysname,
            "cno": cno,
            "pr_if": rho_pred + rng.normal(0.0, sigma_code_m),
            "phi_if_m": rho_pred + amb + rng.normal(0.0, sigma_carrier_m),
        })
    return obs


# ── truth-relative divergence monitor (shared scoring layer) ────────── #

class DivergenceMonitor:
    """Watch error-vs-σ over time; fire when far AND still moving farther.

    Per docs/simulators-and-replay.md: the verdict is the *trajectory*, not
    one epoch.  Fires when the deviation exceeds ``k_sigma·σ`` AND has a
    positive trend sustained across ``window`` epochs — "far and diverging",
    the "no point continuing" signal.  Far-but-settling does not fire.
    """

    def __init__(self, k_sigma=3.0, window=120, slope_thresh=0.0):
        self.k_sigma = float(k_sigma)
        self.window = int(window)
        self.slope_thresh = float(slope_thresh)
        self._err = []
        self._sig = []
        self.fired_epoch = None

    def update(self, epoch, error, sigma):
        self._err.append(float(error))
        self._sig.append(float(sigma))
        if self.fired_epoch is not None or len(self._err) < self.window:
            return False
        w_err = np.array(self._err[-self.window:])
        w_sig = np.array(self._sig[-self.window:])
        # (1) far: above the corridor across the whole window
        far = bool(np.all(w_err > self.k_sigma * w_sig))
        # (2) diverging: positive least-squares slope over the window
        x = np.arange(self.window, dtype=float)
        slope = np.polyfit(x, w_err, 1)[0]
        if far and slope > self.slope_thresh:
            self.fired_epoch = epoch
            return True
        return False

    @property
    def verdict(self):
        return {"fired": self.fired_epoch is not None,
                "fired_epoch": self.fired_epoch}


# ── the run loop (truth-evolves / filter-tracks) ────────────────────── #

def run(sky, truth, *, n_epochs=600, dt=1.0, seed=0,
        seed_pos_offset_enu=(0.0, 0.0, 0.0), seed_pos_sigma_m=5.0,
        seed_ztd_offset_m=0.0, ztd_sigma_m=0.2, clock_model="random_walk",
        monitor=None, sigma_code_m=0.3, sigma_carrier_m=0.005, systems=None,
        bias_fn=None):
    """Drive the real PPPFilter (position estimator) against the synthetic truth.

    Seeds the filter at truth + an optional ENU position offset / ZTD
    offset, then tracks.  Returns per-epoch error and the filter's own σ
    for position (ECEF magnitude) and ZTD, plus the monitor verdict.
    """
    rng = np.random.default_rng(seed)
    east, north, up = enu_basis(truth.arp_ecef)
    de, dn, du = seed_pos_offset_enu
    seed_pos = truth.arp_ecef + de * east + dn * north + du * up

    filt = PPPFilter(clock_model=clock_model)
    # systems: which constellations the filter EXPECTS — drives ISB pinning
    # (a present non-reference system's ISB is pinned to 0 only if its
    # reference is absent).  Default: derive from the sky (the engine's
    # behavior).  Override to study the rank effect of (un)pinning.
    if systems is None:
        systems = sorted({sky.sys_of[s] for s in sky.svs})
    filt.initialize(seed_pos, truth.clk0_m, 0.0, 0.0, systems=systems,
                    pos_sigma_m=seed_pos_sigma_m, ztd_sigma_m=ztd_sigma_m,
                    init_ztd_m=truth.ztd0_m + seed_ztd_offset_m)
    filt.prev_clock = 0.0

    t0 = datetime(2026, 1, 1, tzinfo=timezone.utc)
    if monitor is None:
        monitor = DivergenceMonitor()

    rec = {"t_s": [], "pos_err_m": [], "pos_sigma_m": [], "up_err_m": [],
           "ztd_err_m": [], "ztd_sigma_m": [], "n_used": []}
    for i in range(n_epochs):
        t_s = i * dt
        t_dt = t0 + timedelta(seconds=t_s)
        if i > 0:
            filt.predict(dt)
        obs = emit(filt, sky, truth, t_s, t_dt, rng,
                   sigma_code_m=sigma_code_m, sigma_carrier_m=sigma_carrier_m,
                   bias_fn=bias_fn)
        # caller manages the per-SV float ambiguity lifecycle (as the engine does)
        seen = {o["sv"] for o in obs}
        for o in obs:
            if o["sv"] not in filt.sv_to_idx:
                filt.add_ambiguity(o["sv"], o["phi_if_m"] - o["pr_if"])
        for sv in list(filt.sv_to_idx):
            if sv not in seen:
                filt.remove_ambiguity(sv)
        n_used, _resid, _ = filt.update(obs, sky, t_dt, clk_file=None)

        err_vec = np.asarray(filt.x[:3], float) - truth.arp_ecef
        pos_err = float(np.linalg.norm(err_vec))
        pos_sigma = float(math.sqrt(max(0.0, np.mean(np.diag(filt.P)[:3]))))
        ztd_err = float(filt.x[IDX_ZTD] - truth.ztd(t_s))
        ztd_sigma = float(math.sqrt(max(0.0, filt.P[IDX_ZTD, IDX_ZTD])))

        rec["t_s"].append(t_s)
        rec["pos_err_m"].append(pos_err)
        rec["pos_sigma_m"].append(pos_sigma)
        rec["up_err_m"].append(float(np.dot(err_vec, up)))
        rec["ztd_err_m"].append(ztd_err)
        rec["ztd_sigma_m"].append(ztd_sigma)
        rec["n_used"].append(int(n_used))
        monitor.update(i, pos_err, pos_sigma)

    rec = {k: np.array(v) for k, v in rec.items()}
    rec["verdict"] = monitor.verdict
    return rec


# ── sky presets + demo ──────────────────────────────────────────────── #

def strong_sky(arp):
    """Well-conditioned: 8 SVs spread in azimuth AND elevation (incl. low)."""
    sats = [(f"G{i+1:02d}", "gps", az, el) for i, (az, el) in enumerate([
        (20, 15), (70, 55), (140, 25), (200, 70),
        (250, 18), (300, 45), (340, 35), (110, 80)])]
    return SyntheticSky(arp, sats)


# Single-constellation (Galileo): observable, but high-elevation-heavy so the
# up/ZTD/clock axes are poorly separated.
_GAL_SATS = [(40, 78), (120, 72), (210, 80), (300, 75), (150, 40), (20, 30)]
# Low-elevation GPS "anchors" — the cross-elevation lines of sight that break
# the up/ZTD degeneracy when the second constellation is added.
_GPS_ANCHORS = [(60, 18), (160, 22), (250, 15), (330, 30)]


def gal_only_sky(arp):
    """Galileo only — 6 SVs, high-elevation-heavy (up/ZTD weakly separated)."""
    return SyntheticSky(arp, [(f"E{i+1:02d}", "gal", az, el)
                              for i, (az, el) in enumerate(_GAL_SATS)])


def gps_gal_sky(arp):
    """GPS + Galileo — the Galileo set plus low-elevation GPS anchors."""
    sats = [(f"E{i+1:02d}", "gal", az, el) for i, (az, el) in enumerate(_GAL_SATS)]
    sats += [(f"G{i+1:02d}", "gps", az, el)
             for i, (az, el) in enumerate(_GPS_ANCHORS)]
    return SyntheticSky(arp, sats)


def gps_elev_bias(amplitude_m=0.8):
    """An unmodeled, elevation-dependent GPS-only range error (stands in for a
    bad GPS orbit/clock correction, L5 datum offset, or multipath) — the kind
    of obs-side error pos_replay carries for real.  GAL is untouched."""
    def _bias(sv, sysname, elev_deg):
        if sysname != "gps":
            return 0.0
        return amplitude_m / math.sin(math.radians(max(elev_deg, 5.0)))
    return _bias


def moderate_sky(arp):
    """Partially-degenerate: 7 SVs at mid-high elevation, full azimuth spread.

    σ still collapses (enough spread to observe position) BUT up and ZTD are
    collinear enough that a drifting ZTD leaks into up-position — the
    confident-but-wrong regime.
    """
    sats = [(f"G{i+1:02d}", "gps", az, el) for i, (az, el) in enumerate([
        (30, 55), (90, 68), (150, 60), (210, 72),
        (270, 58), (330, 65), (180, 78)])]
    return SyntheticSky(arp, sats)


def weak_sky(arp):
    """(up,ZTD,clk)-degenerate: 5 SVs all at HIGH elevation → LOS≈vertical."""
    sats = [(f"G{i+1:02d}", "gps", az, el) for i, (az, el) in enumerate([
        (10, 82), (90, 85), (180, 80), (270, 84), (340, 86)])]
    return SyntheticSky(arp, sats)


def main():
    # A valid ECEF ARP (mid-latitude, ~ Chicago-ish); magnitude is what matters.
    arp = np.array([-2_730_000.0, -4_440_000.0, 3_975_000.0])
    print("pos_sim — the (pos,ZTD,clk) null vs the divergence monitor\n")
    print(f"{'sky':>9s} | {'n':>2s} | {'up_err':>8s} | {'σ_pos':>6s} | "
          f"{'err/σ':>6s} | {'monitor':>16s}")
    print("-" * 64)
    # Each row drifts the truth ZTD (a null axis); the question is where the
    # misfit lands and whether the filter stays honest about it.
    scenarios = [
        ("strong", strong_sky(arp), 2e-4),    # ZTD separable → tracks truth
        ("moderate", moderate_sky(arp), 2e-3),  # ZTD leaks into up, σ collapses
        ("weak", weak_sky(arp), 1e-3),        # unobservable → honest large σ
    ]
    for label, sky, ztd_rate in scenarios:
        truth = Truth(arp_ecef=arp, ztd0_m=0.05, ztd_rate_m_s=ztd_rate)
        rec = run(sky, truth, n_epochs=600, seed=1, ztd_sigma_m=0.05,
                  monitor=DivergenceMonitor(k_sigma=3.0, window=120))
        v = rec["verdict"]
        ue, sp = rec["up_err_m"][-1], rec["pos_sigma_m"][-1]
        msg = f"FIRE @ ep {v['fired_epoch']}" if v["fired"] else "in corridor"
        print(f"{label:>9s} | {len(sky.svs):>2d} | {ue:+7.3f} | {sp:6.3f} | "
              f"{abs(ue)/sp:6.2f} | {msg:>16s}")
    print("\nstrong  → ZTD separable, error small, honest σ → no fire")
    print("moderate→ confident (σ collapses) BUT ZTD drift leaks into up and")
    print("          GROWS past 3σ → monitor fires: 'no point continuing'")
    print("weak    → unobservable, but σ stays honestly large → no fire")
    print("(The monitor needs confident AND wrong AND diverging — not just far.)")
    constellation_demo(arp)


def constellation_demo(arp):
    """The GPS-vs-Galileo question: does adding GPS help or hurt, and why?

    Answer (clean obs): more constellations only HELP — adding GPS's
    low-elevation anchors breaks the up/ZTD degeneracy a high-elevation-heavy
    Galileo-only sky suffers from.  So "Galileo converges but GPS+GAL doesn't"
    cannot come from rank/geometry; it needs a GPS-side OBSERVATION error.
    Inject one and GPS+GAL degrades — the asymmetry is observation-side, which
    is exactly what pos_sim (shared h()) can't carry and pos_replay can.
    """
    print("\n\npos_sim — single-constellation (GPS vs Galileo) rank story\n")
    truth_kw = dict(ztd0_m=0.05, ztd_rate_m_s=1e-3)
    rows = [
        ("Galileo only", gal_only_sky(arp), None),
        ("GPS+GAL (clean)", gps_gal_sky(arp), None),
        ("GPS+GAL (GPS biased)", gps_gal_sky(arp), gps_elev_bias()),
    ]
    print(f"{'config':>22s} | {'n':>2s} | {'pos_err':>8s} | {'σ_pos':>6s}")
    print("-" * 48)
    for label, sky, bias in rows:
        truth = Truth(arp_ecef=arp, **truth_kw)
        rec = run(sky, truth, n_epochs=400, seed=1, ztd_sigma_m=0.05, bias_fn=bias)
        print(f"{label:>22s} | {len(sky.svs):>2d} | "
              f"{rec['pos_err_m'][-1]:7.3f} | {rec['pos_sigma_m'][-1]:6.3f}")
    print("\nGalileo only → high-el-heavy, up/ZTD weakly separated → biased")
    print("GPS+GAL clean→ low-elev GPS anchors break the degeneracy → RESCUED")
    print("GPS+GAL biased→ an unmodeled GPS obs-error degrades it → 'adds GPS")
    print("               hurts' is OBSERVATION-side, not rank → pos_replay's job")


if __name__ == "__main__":
    main()
