#!/usr/bin/env python3
"""Forward model: design parameters -> two-clock PPS-OUT agreement (p95 |d|).

`docs/two-site-sync-budget.md` runs BACKWARDS -- it starts from the 1 ns / 2 ns
excursion targets and allocates a per-term budget.  This module runs the same
math FORWARDS: given a DO class, a DO-phase timestamper, a servo cadence and an
evaluation window, what p95 |d| between two such clocks should we expect?  It
exists so hardware decisions (which OCXO, TICC-or-EXTINT, 1 Hz or 10 Hz) can be
sized before anything is soldered.

Companion doc: `docs/two-clock-agreement-forward-model.md`.

Model -- three bands separated by the loop time constant tau_c
--------------------------------------------------------------
    tau < tau_c   DO free-running noise the loop is too slow to remove
                  -> GROWS with tau_c
    tau ~ tau_c   measurement noise the loop *injects* onto the DO
                  -> SHRINKS as 1/sqrt(fs*tau_c)
    tau > tau_c   reference-side error the loop faithfully *tracks*
                  -> independent of tau_c

    sigma_clock(W)^2 = [ADEV(tau_c)*tau_c]^2 + sigma_z^2/(fs*tau_c)
                       + sigma_ref(W)^2 + sigma_act^2 + bias^2
    sigma_delta      = sqrt(2) * sigma_clock   (two independent twins)
    p95 |d|          = 1.96 * sigma_delta      (Gaussian)

Bands 1 and 2 fight; their crossover sets the optimum tau_c.  Band 3 ignores the
loop entirely and is, empirically, the term that currently dominates the lab.

KNOWN LIMITS -- read before quoting a number
--------------------------------------------
* Gaussian.  The 1.96 factor is honest for p95 and useless past ~p99.  Real
  tails are set by gate lockouts and transients (`two-site-sync-budget.md`
  section 3.2.1), not by this variance.  For tails use
  `scripts/peppar_fix/servo_sim.py`.
* sigma_ref is an INPUT, not a prediction.  Nothing here derives rx-TCXO chase,
  correction-stream error, or multipath from datasheet numbers -- they must be
  measured per receiver.  See `backfit_sigma_ref()`.
* Linear loop.  No quantizer dead zones, no chi2 gating, no coast, no actuator
  saturation, no thermal transients.
* sigma_meas assumes a DITHERED quantizer.  The 1/sqrt(N) averaging is only
  valid while the timestamper's grid walks across the DO edge.  On EXTINT the
  walk comes from the rx TCXO's own wander; if the receiver is ever clocked
  *from* the DO, the grid goes coherent, dither stops, and the quantization
  becomes a standing bias of up to +/- q/2 that no averaging removes.  Feed
  that case in as `bias_ns`, not as `q_ns`.
"""
from __future__ import annotations

import argparse
import math

# -- Component libraries ---------------------------------------------------
# a1 = ADEV(1 s) white-FM coefficient, floor = flicker-FM floor
DO_CLASSES = {
    'tcxo-i226':    dict(a1=1e-10, floor=1e-11, label='TCXO i226 internal'),
    'mems-sit5358': dict(a1=5e-11, floor=5e-12, label='MEMS SiT5358'),
    'ocxo-hobby':   dict(a1=1e-11, floor=3e-12, label='Hobby OCXO (CTI/IsoTemp)'),
    'ocxo-good':    dict(a1=3e-12, floor=1e-12, label='Good OCXO'),
    'ocxo-premium': dict(a1=2e-13, floor=1e-13, label='Premium OCXO (OX-249)'),
}

# Quantization step of the DO-phase observer, in ns.
TIMESTAMPERS = {
    'extint-f9t': dict(q_ns=8.0,   label='F9T EXTINT/TIM-TM2'),
    'extts-phc':  dict(q_ns=8.0,   label='PHC EXTTS (effective)'),
    'ticc':       dict(q_ns=0.060, label='TAPR TICC'),
    'tdc7200':    dict(q_ns=0.055, label='TDC7200 Click'),
}

# Actuator LSB in ppb over a +/-2 ppm pull range.
DACS = {
    '16-bit': 0.061,   # AD5693R (current fleet)
    '18-bit': 0.015,   # AD5781
    '20-bit': 0.004,   # AD5791
    'fcw':    0.001,   # Renesas 8A34002 ClockMatrix frequency control word
}

P95_K = 1.96           # two-sided Gaussian 95%


# -- Terms -----------------------------------------------------------------
def adev(tau, a1, floor, rwfm=0.0):
    """ADEV(tau): white-FM a1/sqrt(tau), flicker floor, optional random-walk FM."""
    return math.hypot(max(a1 / math.sqrt(tau), floor), rwfm * math.sqrt(tau))


def sigma_do_ns(tau_c, a1, floor, rwfm=0.0):
    """Phase the DO accumulates free-running over one loop time constant."""
    return adev(tau_c, a1, floor, rwfm) * tau_c * 1e9


def sigma_meas_ns(q_ns, tau_c, fs=1.0, white_ns=0.0):
    """Per-epoch observer noise after the loop averages it.

    q_ns/sqrt(12) is the uniform-quantization RMS; `white_ns` carries any extra
    per-epoch white noise (PPP clock-arm scatter, TICC trigger noise).
    """
    return math.hypot(q_ns / math.sqrt(12.0), white_ns) / math.sqrt(fs * tau_c)


def sigma_act_ns(dac_lsb_ppb, t_act):
    """Phase accumulated per actuation step from frequency quantization."""
    return (dac_lsb_ppb / math.sqrt(12.0)) * t_act


def sigma_clock_ns(tau_c, *, a1, floor, q_ns, rwfm=0.0, fs=1.0, white_ns=0.0,
                   sigma_ref_ns=0.0, dac_lsb_ppb=DACS['16-bit'], bias_ns=0.0):
    """One clock's RMS phase error vs GPS -- the part that does NOT common-mode
    with a co-located twin.  Returns (sigma_clock, per-term dict), all in ns."""
    t = {
        'DO':   sigma_do_ns(tau_c, a1, floor, rwfm),
        'meas': sigma_meas_ns(q_ns, tau_c, fs, white_ns),
        'ref':  sigma_ref_ns,
        'act':  sigma_act_ns(dac_lsb_ppb, 1.0 / fs),
        'bias': bias_ns,
    }
    return math.sqrt(sum(v * v for v in t.values())), t


def p95_ns(tau_c, **kw):
    """p95 |d| between two independent clocks of this design.
    Returns (p95, per-term dict)."""
    sc, terms = sigma_clock_ns(tau_c, **kw)
    return P95_K * math.sqrt(2.0) * sc, terms


def optimize_tau(*, tau_lo=1.0, tau_hi=3000.0, n=600, **kw):
    """Best loop time constant and the p95 it buys.  Returns (p95, tau_c, terms)."""
    best = None
    for i in range(n + 1):
        tau = tau_lo * (tau_hi / tau_lo) ** (i / n)
        p, terms = p95_ns(tau, **kw)
        if best is None or p < best[0]:
            best = (p, tau, terms)
    return best


def backfit_sigma_ref(p95_measured_ns, tau_c, **kw):
    """Invert the model: given a MEASURED p95 |d| and everything else, how much
    independent reference-side error must each clock be carrying?

    This is the intended way to get sigma_ref -- it is not predictable a priori.
    Returns ns, or 0.0 if the measurement is already at/below the modelled floor.
    """
    kw = dict(kw)
    kw['sigma_ref_ns'] = 0.0
    floor, _ = sigma_clock_ns(tau_c, **kw)
    need = p95_measured_ns / (P95_K * math.sqrt(2.0))
    return math.sqrt(max(0.0, need * need - floor * floor))


# -- CLI -------------------------------------------------------------------
def _cfg(args):
    do = DO_CLASSES[args.do]
    return dict(a1=do['a1'], floor=do['floor'], q_ns=TIMESTAMPERS[args.ts]['q_ns'],
                fs=args.rate, white_ns=args.white, sigma_ref_ns=args.ref,
                dac_lsb_ppb=DACS[args.dac], bias_ns=args.bias)


def cmd_one(args):
    p, tau, t = optimize_tau(**_cfg(args))
    sc = math.sqrt(sum(v * v for v in t.values()))
    print(f"{DO_CLASSES[args.do]['label']}  +  {TIMESTAMPERS[args.ts]['label']}")
    print(f"  {args.rate:g} Hz, {args.dac} DAC, sigma_ref={args.ref:g} ns, "
          f"bias={args.bias:g} ns")
    print(f"  optimum tau_c = {tau:.0f} s")
    for k in ('DO', 'meas', 'ref', 'act', 'bias'):
        print(f"    sigma_{k:<5} {t[k] * 1000:8.1f} ps")
    print(f"  sigma_clock = {sc * 1000:.0f} ps")
    print(f"  p95 |d|     = {p:.2f} ns   "
          f"({'PASS' if p <= args.target else 'FAIL'} vs {args.target:g} ns target)")


def cmd_matrix(args):
    print(f"p95 |d| [ns] at optimum tau_c -- {args.rate:g} Hz, {args.dac} DAC, "
          f"sigma_ref={args.ref:g} ns")
    names = list(TIMESTAMPERS)
    print(f"{'DO class':<26}" + "".join(f"{n:>14}" for n in names))
    for do in DO_CLASSES.values():
        row = f"{do['label']:<26}"
        for tk in names:
            p, tau, _ = optimize_tau(
                a1=do['a1'], floor=do['floor'], q_ns=TIMESTAMPERS[tk]['q_ns'],
                fs=args.rate, sigma_ref_ns=args.ref,
                dac_lsb_ppb=DACS[args.dac], bias_ns=args.bias)
            row += f"{p:>9.2f} @{tau:>3.0f}s"
        print(row)
    print("\n  cell = p95 |d| @ optimum loop time constant")


def cmd_bands(args):
    print(f"Term breakdown vs loop time constant -- {DO_CLASSES[args.do]['label']}"
          f" + {TIMESTAMPERS[args.ts]['label']}, sigma_ref={args.ref:g} ns")
    print(f"{'tau_c':>7}{'s_DO':>10}{'s_meas':>10}{'s_ref':>10}{'s_act':>10}"
          f"{'p95 |d|':>11}")
    for tau in (1, 3, 10, 30, 60, 100, 200, 300, 600, 1000, 3000):
        p, t = p95_ns(float(tau), **_cfg(args))
        print(f"{tau:>6}s{t['DO']*1000:>9.0f}p{t['meas']*1000:>9.0f}p"
              f"{t['ref']*1000:>9.0f}p{t['act']*1000:>9.0f}p{p:>10.2f}n")


def cmd_ref(args):
    print(f"p95 |d| vs reference-side error -- {DO_CLASSES[args.do]['label']}, "
          f"{args.rate:g} Hz, {args.dac} DAC")
    names = list(TIMESTAMPERS)
    print(f"{'sigma_ref':>10}" + "".join(f"{n:>13}" for n in names))
    for ref in (0.0, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 3.0, 5.0):
        row = f"{ref:>9.2f}n"
        for tk in names:
            p, _, _ = optimize_tau(
                a1=DO_CLASSES[args.do]['a1'], floor=DO_CLASSES[args.do]['floor'],
                q_ns=TIMESTAMPERS[tk]['q_ns'], fs=args.rate, sigma_ref_ns=ref,
                dac_lsb_ppb=DACS[args.dac], bias_ns=args.bias)
            row += f"{p:>10.2f} n"
        print(row)
    print("\n  sigma_ref = per-clock INDEPENDENT reference-side error over the\n"
          "  eval window (rx TCXO chase + correction error + multipath).  Above\n"
          "  ~0.5 ns the timestamper columns converge -- the observer stops\n"
          "  mattering.  Get sigma_ref from `backfit`, never from a datasheet.")


def cmd_backfit(args):
    kw = _cfg(args)
    kw['sigma_ref_ns'] = 0.0
    tau = args.tau if args.tau else optimize_tau(**kw)[1]
    ref = backfit_sigma_ref(args.p95, tau, **kw)
    print(f"measured p95 |d| = {args.p95:g} ns  ->  sigma_clock = "
          f"{args.p95 / (P95_K * math.sqrt(2.0)) * 1000:.0f} ps per clock")
    print(f"  at tau_c = {tau:.0f} s, modelled floor (sigma_ref=0) gives "
          f"p95 = {p95_ns(tau, **kw)[0]:.2f} ns")
    print(f"  => implied independent sigma_ref = {ref:.2f} ns per clock")
    if ref > 0.5:
        print("  sigma_ref dominates: a better timestamper or a better DO will "
              "not help.")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)

    def common(p, ts=True):
        p.add_argument('--do', choices=DO_CLASSES, default='ocxo-hobby')
        if ts:
            p.add_argument('--ts', choices=TIMESTAMPERS, default='extint-f9t')
        p.add_argument('--rate', type=float, default=1.0, help='measurement Hz')
        p.add_argument('--dac', choices=DACS, default='16-bit')
        p.add_argument('--ref', type=float, default=0.0, help='sigma_ref, ns')
        p.add_argument('--white', type=float, default=0.0,
                       help='extra per-epoch white observer noise, ns')
        p.add_argument('--bias', type=float, default=0.0,
                       help='uncalibrated per-unit constant bias, ns')

    p = sub.add_parser('one', help='one configuration, full breakdown')
    common(p)
    p.add_argument('--target', type=float, default=1.0)
    p.set_defaults(func=cmd_one)

    p = sub.add_parser('matrix', help='DO class x timestamper')
    common(p, ts=False)
    p.set_defaults(func=cmd_matrix)

    p = sub.add_parser('bands', help='term breakdown vs loop time constant')
    common(p)
    p.set_defaults(func=cmd_bands)

    p = sub.add_parser('ref', help='sensitivity to reference-side error')
    common(p, ts=False)
    p.set_defaults(func=cmd_ref)

    p = sub.add_parser('backfit', help='measured p95 -> implied sigma_ref')
    common(p)
    p.add_argument('--p95', type=float, required=True)
    p.add_argument('--tau', type=float, default=None)
    p.set_defaults(func=cmd_backfit)

    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
