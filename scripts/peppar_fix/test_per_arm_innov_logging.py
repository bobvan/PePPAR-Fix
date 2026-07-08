"""Unit tests for per-arm innovation logging in DOFreqEst.

Verifies that `self.last_arm_innov[name]` and `self.last_arm_S[name]`
are populated for each arm that fires in a given `update()` call, and
left at None for arms that didn't fire.  This data backs the
`innov_{name}` / `s_{name}` columns in `--arm-state-log` and is the
key post-mortem aid for state-excursion debugging
(see docs/piface-x1-excursion-2026-05-24.md, dayplan I-074410-bravo).

Recording happens BEFORE the state update so a future chi-squared
gate (ekfInnovationGate-bravo) that skips the update can still report
the rejected innov via the log.
"""

from __future__ import annotations

import math

import numpy as np

from peppar_fix.do_freq_est import DOFreqEst


_ARMS = ('ppp', 'qerr', 'tdcp', 'extint', 'do_phase', 'pseudo', 'ticc',
         'cm_phase')


def _baseline():
    """A DOFreqEst seeded for repeatable arm-isolation testing.

    Mirror of `test_do_freq_est_arm5.py:_baseline()`.
    """
    f = DOFreqEst(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0, sigma_tcxo_freq_ppb=0.1,
        initial_dt_rx_ns=0.0, base_freq=0.0,
    )
    f.P = np.diag([1.0, 1.0, 100.0, 1.0])
    f.x = np.array([0.0, 0.0, 0.0, 0.0])
    return f


def test_dicts_initialized_with_all_arm_keys():
    """Both dicts have all expected arm keys, all None at construction."""
    f = _baseline()
    assert set(f.last_arm_innov.keys()) == set(_ARMS)
    assert set(f.last_arm_S.keys()) == set(_ARMS)
    assert all(v is None for v in f.last_arm_innov.values())
    assert all(v is None for v in f.last_arm_S.values())


def test_no_arms_fired_dicts_stay_none():
    """If no arm provides an observation, all dict entries stay None."""
    f = _baseline()
    f.update(dt=1.0)  # predict-only, no measurements
    assert all(v is None for v in f.last_arm_innov.values())
    assert all(v is None for v in f.last_arm_S.values())


def test_ppp_records_innov_and_s():
    f = _baseline()
    f.update(dt=1.0, dt_rx_ns=1.5, dt_rx_sigma_ns=0.1)
    assert f.last_arm_innov['ppp'] is not None
    assert f.last_arm_S['ppp'] is not None
    assert f.last_arm_S['ppp'] > 0
    # Other arms didn't fire
    for arm in ('qerr', 'tdcp', 'extint', 'pseudo', 'ticc'):
        assert f.last_arm_innov[arm] is None, f"{arm} should be None"
        assert f.last_arm_S[arm] is None, f"{arm} should be None"


def test_qerr_records_innov_and_s():
    f = _baseline()
    f.update(dt=1.0, qerr_freq_ppb=0.3, qerr_freq_sigma_ppb=0.30)
    assert f.last_arm_innov['qerr'] is not None
    assert f.last_arm_S['qerr'] is not None
    # Innovation is z − Hx = 0.3 − 0 = 0.3 (state seeded at zero).
    assert math.isclose(f.last_arm_innov['qerr'], 0.3, abs_tol=1e-9)
    # All other arms None
    for arm in ('ppp', 'tdcp', 'extint', 'pseudo', 'ticc'):
        assert f.last_arm_innov[arm] is None


def test_tdcp_records_innov_and_s():
    f = _baseline()
    f.update(dt=1.0, tdcp_freq_ppb=0.5, tdcp_freq_sigma_ppb=0.026)
    assert f.last_arm_innov['tdcp'] is not None
    assert f.last_arm_S['tdcp'] is not None
    assert math.isclose(f.last_arm_innov['tdcp'], 0.5, abs_tol=1e-9)


def test_extint_records_innov_and_s():
    f = _baseline()
    f.update(dt=1.0, extint_phase_ns=12.0, extint_sigma_ns=5.0)
    assert f.last_arm_innov['extint'] is not None
    assert f.last_arm_S['extint'] is not None


def test_ticc_records_innov_and_s():
    f = _baseline()
    f.update(dt=1.0, ticc_diff_ns=-44.0, ticc_sigma_ns=0.060)
    assert f.last_arm_innov['ticc'] is not None
    assert f.last_arm_S['ticc'] is not None
    # TICC is the only path that sets `last_innov` / `last_S` (legacy
    # singletons used by InnovControlMonitor) — verify the dict and
    # the singletons agree.
    assert f.last_arm_innov['ticc'] == f.last_innov
    assert f.last_arm_S['ticc'] == f.last_S


def test_multiple_arms_record_independently():
    """When several arms fire in one epoch, each records its own innov+S."""
    f = _baseline()
    f.update(
        dt=1.0,
        qerr_freq_ppb=0.2, qerr_freq_sigma_ppb=0.30,
        tdcp_freq_ppb=0.25, tdcp_freq_sigma_ppb=0.026,
        extint_phase_ns=8.0, extint_sigma_ns=5.0,
    )
    for arm in ('qerr', 'tdcp', 'extint'):
        assert f.last_arm_innov[arm] is not None, f"{arm} should fire"
        assert f.last_arm_S[arm] is not None
    for arm in ('ppp', 'pseudo', 'ticc'):
        assert f.last_arm_innov[arm] is None, f"{arm} should not fire"


def test_dicts_reset_each_epoch():
    """A new epoch where the arm doesn't fire clears the prior entry."""
    f = _baseline()
    f.update(dt=1.0, qerr_freq_ppb=0.5, qerr_freq_sigma_ppb=0.30)
    assert f.last_arm_innov['qerr'] is not None
    # Next epoch: no arms fire.
    f.update(dt=1.0)
    assert f.last_arm_innov['qerr'] is None
    assert f.last_arm_S['qerr'] is None


def test_arm_recording_survives_zero_innovation():
    """Zero innovation is still a recorded value, not None.

    Distinguishes 'arm fired with innov=0' from 'arm didn't fire'.
    """
    f = _baseline()
    f.x = np.array([0.0, 0.5, 0.0, 0.0])  # state already at z
    f.update(dt=1.0, qerr_freq_ppb=0.5, qerr_freq_sigma_ppb=0.30)
    # innovation should be ~0 (state pre-update is 0.5 from x[1] seed
    # minus tiny predict-step drift)
    assert f.last_arm_innov['qerr'] is not None
    # Must be a real (small) float, not None or NaN.
    assert isinstance(f.last_arm_innov['qerr'], float)
    assert not math.isnan(f.last_arm_innov['qerr'])


def test_ticc_innov_recorded_before_state_update():
    """The TICC innov is computed against the PRE-update state (x_pred),
    and `last_arm_innov['ticc']` reflects that pre-update value.  This
    is what the post-mortem needs: 'what innov did the gate see?', not
    'what's the state now?'.
    """
    f = _baseline()
    f.x = np.array([0.0, 0.0, 0.0, 0.0])
    f.P = np.diag([1.0, 1.0, 100.0, 1.0])
    # Skip the constructor's first-TICC seed step (which deliberately
    # absorbs the initial innov to avoid a startup shock).  After this,
    # subsequent TICC observations produce real (non-seeded) innovs.
    f._need_phc_seed = False
    # h_ticc(x_pred) at this state ≈ -x[2] - qerr(x[0]) = 0 - 0 = 0.
    # Innov = z - 0 = z.
    z = 25.0
    f.update(dt=1.0, ticc_diff_ns=z, ticc_sigma_ns=0.060)
    # The recorded innov should match z (within predict-step drift).
    assert f.last_arm_innov['ticc'] is not None
    assert abs(f.last_arm_innov['ticc'] - z) < 1.0, (
        f"recorded innov {f.last_arm_innov['ticc']:.3f} should be ≈ {z}"
    )
