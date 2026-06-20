#!/usr/bin/env python3
"""Prototype: latestQErrChiSelect — does pure-latest qErr + chi2 selection work,
and does the gate have to be COMPARATIVE (Main's PR #203 must-resolve)?

Closed-loop sim driving the REAL peppar_fix.do_freq_est.DOFreqEst.  Two routers:
  ABSOLUTE  = stock DOFreqEst(routed_qerr=True): ext accepted if chi2 <= ~100.
  COMPARATIVE = CompDOFreqEst override: ext wins only if it beats raw's chi2
                (argmin-chi2 with a small margin) — the proposed fix.

qErr feed modes:
  matched = qerr() of the CURRENT edge (what the matched router achieves).
  latest  = qerr() of the PREVIOUS edge (pure-latest, no matching). Equal to
            matched when the rx phase doesn't cross a tick between edges;
            wrong by ~a full tick (off-by-edge ~3.3ns RMS) when it does.

Regime = lab TICC-only with poorly-known rx phase (no PPP / GNSS-PPS rejected),
so the internal qerr(x0) model is weak and the receiver's ext qErr is the real
value-add (the qVIR-high case, e.g. MadHat 75-130 / PiFace 28-80).

Metric: chA = realized DO phase (x2_truth) RMSE after convergence; route mix;
and P[2,2] sanity (covariance must not collapse).
"""
import logging
import sys
from pathlib import Path

import numpy as np

logging.disable(logging.CRITICAL)  # silence DOFreqEst per-epoch chatter

sys.path.insert(0, str(Path(__file__).resolve().parent))
from peppar_fix.do_freq_est import DOFreqEst, _qerr, _CHI2_GATE_THRESHOLD  # noqa: E402

TICK = 8.0


class CompDOFreqEst(DOFreqEst):
    """Comparative-gate variant: ext must BEAT raw's chi2 (argmin-chi2 with a
    margin), instead of merely clearing the absolute _CHI2_GATE_THRESHOLD."""
    EXT_MARGIN = 0.0  # pure argmin-chi2: ext wins iff it reduces normalized innov

    def _route_ticc_arm(self, x_pred, P_pred, ticc_diff_ns, ticc_qerr_ns,
                        R_base, R_lin):
        H_x2 = np.array([[0.0, 0.0, -1.0, 0.0]])
        cands = []
        if ticc_qerr_ns is not None:
            cands.append(('ext', ticc_diff_ns + ticc_qerr_ns, -x_pred[2], H_x2, R_base))
        cands.append(('int', ticc_diff_ns, self._h_ticc(x_pred),
                      self._H_ticc(x_pred), R_base + R_lin))
        cands.append(('raw', ticc_diff_ns, -x_pred[2], H_x2,
                      R_base + (self.tick_ns ** 2) / 12.0))
        scored = []
        for name, z, h, H, R in cands:
            S = (H @ P_pred @ H.T).item() + R
            chi2 = (z - h) ** 2 / S if S > 0 else 0.0
            scored.append((chi2, name, z, h, H, R))
        chi2_raw = next(c[0] for c in scored if c[1] == 'raw')
        # Comparative: prefer the smallest chi2, but a non-raw candidate must
        # beat raw by EXT_MARGIN to be chosen (else fall to the robust floor).
        best = min(scored, key=lambda c: c[0])
        if best[1] != 'raw' and best[0] > chi2_raw - self.EXT_MARGIN:
            best = next(c for c in scored if c[1] == 'raw')
        _, name, z, h, H, R = best
        return z, h, H, R, name


def run(router, qerr_mode, n=20000, seed=1, ppp_sigma=1.5):
    rng = np.random.default_rng(seed)
    Est = CompDOFreqEst if router == 'comparative' else DOFreqEst
    ekf = Est(routed_qerr=True, sigma_ticc_ns=0.060)
    dt = 1.0
    # truth
    phi_do = 0.0; f_do = 0.0
    x0 = 0.0   # rx TCXO phase (ns), wanders & sweeps ticks
    adjfine = 0.0
    prev_qerr = 0.0
    errs = []; routes = {'ext': 0, 'int': 0, 'raw': 0, None: 0}; p22 = []
    for k in range(n):
        # truth propagate
        f_do += rng.normal(0, 0.002)              # DO freq random walk
        phi_do += (f_do + adjfine) * dt + rng.normal(0, 0.05)
        x0 += rng.normal(0.7, 0.2)                # rx phase wanders, sweeps ticks
        # measurements
        v_ticc = rng.normal(0, 0.060)
        ticc_diff = -phi_do - _qerr(x0, TICK) + v_ticc
        cur_qerr = _qerr(x0, TICK) + rng.normal(0, 0.1)   # receiver report (correct edge)
        if qerr_mode == 'matched':
            fed_qerr = cur_qerr
        elif qerr_mode == 'latest':
            fed_qerr = prev_qerr
        else:                       # 'none' = no ext qErr (raw/int only) — the no-harm floor
            fed_qerr = None
        prev_qerr = cur_qerr
        kw = dict(dt=dt, ticc_diff_ns=ticc_diff, ticc_sigma_ns=0.060,
                  ticc_qerr_ns=fed_qerr)
        # PPP arm: tracks x0 to ~ppp_sigma ns — stable, but NOT to sub-tick
        # precision, so internal qerr(x0) is imperfect and ext qErr is the
        # value-add (the lab qVIR-high regime).
        kw.update(dt_rx_ns=x0 + rng.normal(0, ppp_sigma), dt_rx_sigma_ns=ppp_sigma)
        adjfine = -ekf.update(**kw)
        routes[getattr(ekf, 'last_ticc_route', None)] = \
            routes.get(getattr(ekf, 'last_ticc_route', None), 0) + 1
        if k > 2000:
            errs.append(ekf.x[2] - phi_do)        # filter's DO-phase error vs truth
            p22.append(ekf.P[2, 2])
    errs = np.array(errs)
    rmse = float(np.sqrt(np.mean(errs ** 2)))
    tot = sum(routes.values())
    mix = {kk: f"{100*vv/tot:.0f}%" for kk, vv in routes.items() if vv and kk}
    return rmse, mix, float(np.median(p22)), float(np.max(p22))


def main():
    print("regime: TICC-only, no PPP (rx phase loosely known) — the qVIR-high lab case\n")
    print(f"{'router':>12s} {'qErr':>8s} | {'x2 RMSE':>9s} | {'route mix':>26s} | "
          f"{'P22 med':>8s} {'P22 max':>8s}")
    print("-" * 88)
    rows = [
        ('comparative', 'none'),     # NO-HARM FLOOR: no ext qErr at all
        ('absolute', 'matched'),     # control: matched router (today), should win
        ('absolute', 'latest'),      # Main's claim: absolute gate poisons on lagged
        ('comparative', 'latest'),   # the fix: should beat the floor, not poison
        ('comparative', 'matched'),  # fix must not hurt the matched case
    ]
    res = {}
    for router, qmode in rows:
        rmse, mix, p22med, p22max = run(router, qmode)
        res[(router, qmode)] = rmse
        mixs = " ".join(f"{k}={v}" for k, v in mix.items())
        print(f"{router:>12s} {qmode:>8s} | {rmse*1e3:7.1f}ps | {mixs:>26s} | "
              f"{p22med:7.2f} {p22max:7.2f}")
    floor = res[('comparative', 'none')]
    print()
    print(f"  NO-HARM FLOOR (no ext qErr)               : {floor*1e3:.1f} ps")
    print(f"  matched/absolute (today's behavior)       : {res[('absolute','matched')]*1e3:.1f} ps")
    print(f"  latest/ABSOLUTE  (Main: poisons?)         : {res[('absolute','latest')]*1e3:.1f} ps"
          f"   ({res[('absolute','latest')]/floor:.1f}x the floor — {'HARM' if res[('absolute','latest')]>floor else 'ok'})")
    print(f"  latest/COMPARATIVE (the fix)              : {res[('comparative','latest')]*1e3:.1f} ps"
          f"   ({res[('comparative','latest')]/floor:.2f}x the floor — {'HARM' if res[('comparative','latest')]>floor*1.05 else 'no harm'})")
    print(f"  matched/COMPARATIVE (fix doesn't hurt?)   : {res[('comparative','matched')]*1e3:.1f} ps")


if __name__ == "__main__":
    main()
