"""Lock the single-oscillator safety property (Main's PR #303 review).

The GNSSDO+ do_phase arm (Arm 8) is fed ONLY by _dt_rx_servo_epoch, which
_servo_epoch reaches ONLY when ctx['dt_rx_phase'] is set (= actuator_type
'gnssdo').  A two-oscillator host (ClockMatrix combo/FCW etc.) sets
ctx['cm_phase_source'] and MUST route to _cm_servo_epoch — never touching the
do_phase arm.  Main flagged that this was verified by reading, not by a test.
This locks the dispatch: cm_phase wins, and it wins even if both flags are set
(a mis-config can't fall through to Arm 8).
"""
import os
import sys
import types

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SCRIPTS = os.path.join(_ROOT, "scripts")
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import peppar_fix_engine as E


def _call(ctx, monkeypatched):
    """Invoke _servo_epoch's dispatch with sentinel servo-epoch fns."""
    calls = []
    orig_cm, orig_dt = E._cm_servo_epoch, E._dt_rx_servo_epoch
    E._cm_servo_epoch = lambda *a, **k: calls.append("cm") or "cm"
    E._dt_rx_servo_epoch = lambda *a, **k: calls.append("dt_rx") or "dt_rx"
    try:
        E._servo_epoch(ctx, types.SimpleNamespace(), None, None, None, 0,
                       1.0, 0.1, 0, None, 0.0, 0.0, 0.0)
    finally:
        E._cm_servo_epoch, E._dt_rx_servo_epoch = orig_cm, orig_dt
    return calls


def test_two_osc_routes_to_cm_never_arm8():
    # cm_phase_source set (ClockMatrix / two-osc) → _cm_servo_epoch only.
    assert _call({"cm_phase_source": object()}, True) == ["cm"]


def test_gnssdo_routes_to_dt_rx():
    # dt_rx_phase set, no cm → _dt_rx_servo_epoch (Arm 8).
    assert _call({"dt_rx_phase": True}, True) == ["dt_rx"]


def test_cm_wins_when_both_flags_set():
    # Safety: even a mis-config that sets BOTH must route to cm (two-osc),
    # NOT fall through to the single-osc do_phase arm.
    assert _call({"cm_phase_source": object(), "dt_rx_phase": True}, True) \
        == ["cm"]


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
