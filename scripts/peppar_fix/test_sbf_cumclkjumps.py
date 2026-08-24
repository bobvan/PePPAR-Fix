"""SBF MeasEpoch CumClkJumps monitor (I-054229 fast-follow).

A change in the mosaic's cumulative clock-jump count = a ~1 ms receiver-clock
step that lands in the carrier phase.  The reader must surface it (so it isn't
an invisible downstream glitch) — and must NOT warn on the steady no-jump case.
Uses a private handler on the module logger (robust to the full-suite caplog
pollution) and monkeypatches the obs conversion so no real MeasEpoch internals
are needed.
"""
import logging
import os
import queue
import sys
import types

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SCRIPTS = os.path.join(_ROOT, "scripts")
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import peppar_fix.sbf_obs_source as S


class _Capture(logging.Handler):
    def __init__(self):
        super().__init__()
        self.records = []

    def emit(self, record):
        self.records.append(record.getMessage())


def _meas(ccj, tow=1000):
    return types.SimpleNamespace(identity="MeasEpoch", CumClkJumps=ccj,
                                 TOW=tow, WNc=2300)


def _run(measepochs, monkeypatch):
    # No real obs needed — stub the conversion so the loop reaches the monitor.
    # meas_epoch_to_raw_obs is a module import; raw_obs_to_if_observations is
    # imported inside sbf_obs_reader from realtime_ppp, so patch it at source.
    import realtime_ppp
    monkeypatch.setattr(S, "meas_epoch_to_raw_obs", lambda *a, **k: [])
    monkeypatch.setattr(realtime_ppp, "raw_obs_to_if_observations",
                        lambda *a, **k: ([], None, 0, 0))
    # The module logger inherits root's level by default; pin it so the
    # capture handler below is guaranteed to see our WARNING.
    S.log.setLevel(logging.WARNING)
    cap = _Capture()
    cap.setLevel(logging.WARNING)
    S.log.addHandler(cap)
    try:
        msgs = [(b"", m) for m in measepochs]
        S.sbf_obs_reader(iter(msgs), queue.Queue(), None, {}, systems=("gps",))
    finally:
        S.log.removeHandler(cap)
    return [m for m in cap.records if "clock JUMP" in m]


def test_jump_is_warned(monkeypatch):
    warns = _run([_meas(0), _meas(0), _meas(1)], monkeypatch)
    assert len(warns) == 1
    assert "CumClkJumps 0→1" in warns[0]


def test_no_jump_no_warning(monkeypatch):
    assert _run([_meas(3), _meas(3), _meas(3)], monkeypatch) == []


def test_multi_step_jump_reports_delta(monkeypatch):
    warns = _run([_meas(5), _meas(7)], monkeypatch)  # +2 in one step
    assert len(warns) == 1 and "2-step" in warns[0]


def test_first_epoch_never_warns(monkeypatch):
    # No prior count on the very first MeasEpoch → no spurious jump.
    assert _run([_meas(9)], monkeypatch) == []


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
