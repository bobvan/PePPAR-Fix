"""e2e sign lock for the GNSSDO+ engine glue (Bravo c / Main test-gap, #303).

The dt_rx→$W path in _dt_rx_servo_epoch has TWO negations —
    do_phase_err_ns = -dt_rx          (feed the negated dt_rx to Arm 8)
    freq_ppb        = -servo.update() (adjfine sign contract)
— and a wrong flip anywhere is positive feedback straight to the rail.  The
EKF-internal sign is unit-tested elsewhere; this locks the ENGINE-GLUE double
negation + the actuator write, end to end, with a real DOFreqEst + a recording
mock actuator.  Property: sustained positive dt_rx (DO ahead/fast) must produce
a NEGATIVE actuator command (slow the DO) — negative feedback.
"""
import os
import sys
import types

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SCRIPTS = os.path.join(_ROOT, "scripts")
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import peppar_fix_engine as E
from peppar_fix.do_freq_est import DOFreqEst


class _MockActuator:
    max_adj_ppb = 386.0
    current_word = 489540

    def __init__(self):
        self.cmds = []

    def adjust_frequency_ppb(self, ppb):
        self.cmds.append(ppb)


class _MockScheduler:
    def record_actuation(self, *a, **k):
        pass

    def compute_adaptive_interval(self, *a, **k):
        pass


def _fresh_ctx():
    return {
        "servo": DOFreqEst(),
        "scheduler": _MockScheduler(),
        "actuator": _MockActuator(),
        "dt_rx_phase": True,
    }


def _args():
    return types.SimpleNamespace(
        track_outlier_ns=None, track_max_ppb=None, freerun=False,
        gnssdo_compare_log=None, _pvt_store=None)


def _run(ctx, args, dt_rx, n0, n_epochs):
    """Drive n_epochs of _dt_rx_servo_epoch at a fixed dt_rx; return commands."""
    for i in range(n_epochs):
        E._dt_rx_servo_epoch(ctx, args, n0 + i, dt_rx, 0.16)
    return ctx["actuator"].cmds


def _settled_command(dt_rx):
    """Lock at dt_rx≈0, then hold a sustained dt_rx; return the last command."""
    ctx, args = _fresh_ctx(), _args()
    _run(ctx, args, 0.0, 0, 30)          # acquire/lock at zero
    cmds = _run(ctx, args, dt_rx, 30, 15)  # sustained offset
    return cmds[-1]


def test_positive_dt_rx_gives_negative_command():
    # DO ahead (dt_rx>0, fast) → slow it → negative ppb command.
    assert _settled_command(+10.0) < 0.0


def test_negative_dt_rx_gives_positive_command():
    # DO behind (dt_rx<0, slow) → speed it up → positive ppb command.
    assert _settled_command(-10.0) > 0.0


def test_sign_is_antisymmetric():
    # The two directions must be opposite (no dead-zone / sign asymmetry).
    assert _settled_command(+10.0) * _settled_command(-10.0) < 0.0


def test_actuator_actually_written():
    # The guarded write reaches the actuator (not silently skipped).
    ctx, args = _fresh_ctx(), _args()
    _run(ctx, args, 0.0, 0, 5)
    assert len(ctx["actuator"].cmds) == 5


def test_none_or_zero_sigma_epoch_is_skipped():
    # bravo #303 review (b): an epoch with no valid sigma must NOT actuate
    # (else the servo steers from a stale DO-phase estimate).
    ctx, args = _fresh_ctx(), _args()
    _run(ctx, args, 0.0, 0, 3)
    n = len(ctx["actuator"].cmds)
    assert E._dt_rx_servo_epoch(ctx, args, 3, 10.0, None) == "no_sigma"
    assert E._dt_rx_servo_epoch(ctx, args, 4, 10.0, 0.0) == "no_sigma"
    assert len(ctx["actuator"].cmds) == n   # no new commands


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
