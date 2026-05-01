"""Tests for surgical ZTD-only reset on ztd_impossible/ztd_cycling
trips (I-133619-main P3).

Validates the change to AntPosEstThread._apply_integrity_trip: for
the two local-ZTD-threshold trip reasons (impossible / cycling),
the recovery is now ZTD-state-only — NL fixes, MW state, SV states,
and AntPosEst state machine are all untouched.

Why: 2026-04-30 overnight evidence (37+62+82 trips fleet-wide)
showed |Δalt| at recovery = 0.05-0.06 m, meaning the position +
ambiguity scaffold was correct at trip time and only ZTD needed the
reset.  Legacy NL-only-revert was throwing away genuinely-good NL
fixes and ANCHORED state on every trip, costing the trust scaffold
~7 minutes of accumulation each cycle.

ztd_consensus is intentionally exempt — peer-cohort agreement is a
stronger external signal that ZTD-only-reset may not be sufficient
for; legacy NL-revert behavior preserved there until evidence says
otherwise.
"""

from __future__ import annotations

import os
import sys
import unittest
from unittest.mock import MagicMock

import numpy as np

_SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)


class _StubFilter:
    """Minimal stand-in with .x and .P arrays.  Includes _ztd_sigma_m
    instance attribute since AntPosEstThread._apply_integrity_trip
    reads filt._ztd_sigma_m for the post-trip ZTD σ."""

    IDX_ZTD = 6  # matches solve_ppp.IDX_ZTD

    def __init__(self, ztd_value=2.5, ztd_sigma=0.20):
        # 7 base states + 5 ambiguities; only x[IDX_ZTD] is meaningful
        # for the surgical-reset assertion.
        self.x = np.zeros(7 + 5)
        self.x[6] = ztd_value
        self.P = np.eye(7 + 5) * 0.01
        self.P[6, 6] = 0.5 ** 2
        self._ztd_sigma_m = float(ztd_sigma)


def _build_thread_with_ztd_state(reason='ztd_impossible',
                                  ztd_value=2.5, ztd_sigma=0.20,
                                  n_nl_fixes=3, n_mw_svs=10):
    """Construct enough of AntPosEstThread to exercise
    _apply_integrity_trip without running the full thread."""
    import peppar_fix_engine

    # Don't call AntPosEstThread.__init__ — too many real
    # dependencies.  Instead, instantiate the class without __init__
    # and fill in just the attributes the method touches.
    ape = peppar_fix_engine.AntPosEstThread.__new__(
        peppar_fix_engine.AntPosEstThread)

    ape._n_epochs = 100
    ape._fix_set_integrity = MagicMock()
    ape._sv_state = MagicMock()
    # state(sv) returns a SvAmbState; ANCHORED so we can assert
    # that surgical reset does NOT transition.
    from peppar_fix.sv_state import SvAmbState
    ape._sv_state.state = MagicMock(return_value=SvAmbState.ANCHORED)
    ape._sv_state.svs_in = MagicMock(return_value=[])
    ape._sv_state.transition = MagicMock()
    ape._ape_sm = MagicMock()
    from peppar_fix.states import AntPosEstState
    ape._ape_sm.state = AntPosEstState.ANCHORED  # so the legacy code
    # would have transitioned to CONVERGING; surgical reset must not.

    filt = _StubFilter(ztd_value=ztd_value, ztd_sigma=ztd_sigma)
    mw = MagicMock()
    # Populate mw._state with N entries so reset() iteration is
    # observable.
    mw._state = {f'G{i:02d}': object() for i in range(n_mw_svs)}
    mw.reset = MagicMock()
    nl = MagicMock()
    nl._fixed = {f'E{i:02d}': object() for i in range(n_nl_fixes)}
    nl.unfix = MagicMock()

    ev = {
        'reason': reason,
        'ztd_m': ztd_value,
        'threshold_m': 0.7,
        'sustained_epochs': 60,
        'recent_trip_count': 1,
    }
    return ape, filt, mw, nl, ev


class SurgicalZtdResetTest(unittest.TestCase):

    def test_ztd_impossible_resets_only_ztd(self):
        ape, filt, mw, nl, ev = _build_thread_with_ztd_state(
            reason='ztd_impossible', ztd_value=2.5, ztd_sigma=0.20)
        ape._apply_integrity_trip(filt, mw, nl, ev)

        # ZTD state reset
        self.assertEqual(filt.x[filt.IDX_ZTD], 0.0)
        self.assertAlmostEqual(filt.P[filt.IDX_ZTD, filt.IDX_ZTD],
                               0.20 ** 2)

        # NL fixes preserved (no unfix)
        self.assertEqual(nl.unfix.call_count, 0,
                         "ztd_impossible must NOT unfix NL ambiguities "
                         "(surgical reset preserves trust scaffold)")

        # MW state preserved (no reset)
        self.assertEqual(mw.reset.call_count, 0,
                         "ztd_impossible must NOT reset MW state")

        # SV state preserved
        self.assertEqual(ape._sv_state.transition.call_count, 0,
                         "ztd_impossible must NOT transition SVs out of "
                         "their current state")

        # AntPosEst state machine NOT transitioned
        self.assertEqual(ape._ape_sm.transition.call_count, 0,
                         "ztd_impossible must NOT transition AntPosEstState")

        # Trip still recorded for diagnostic purposes
        ape._fix_set_integrity.record_trip.assert_called_once_with(100)

    def test_ztd_cycling_resets_only_ztd(self):
        """ztd_cycling shares the ztd_impossible code path now (no
        WL flush escalation — the legacy escalation hypothesis was
        invalidated by the SSR-PB-gap analysis)."""
        ape, filt, mw, nl, ev = _build_thread_with_ztd_state(
            reason='ztd_cycling', ztd_value=-1.5, ztd_sigma=0.20)
        ape._apply_integrity_trip(filt, mw, nl, ev)

        self.assertEqual(filt.x[filt.IDX_ZTD], 0.0)
        self.assertAlmostEqual(filt.P[filt.IDX_ZTD, filt.IDX_ZTD],
                               0.20 ** 2)
        self.assertEqual(nl.unfix.call_count, 0)
        self.assertEqual(mw.reset.call_count, 0)
        self.assertEqual(ape._sv_state.transition.call_count, 0)
        self.assertEqual(ape._ape_sm.transition.call_count, 0)
        ape._fix_set_integrity.record_trip.assert_called_once_with(100)

    def test_ztd_consensus_keeps_legacy_revert(self):
        """ztd_consensus is intentionally exempt from surgical-only
        reset — peer-cohort signal is stronger evidence that ZTD-only
        reset may not be sufficient.  Legacy NL-revert preserved."""
        ape, filt, mw, nl, ev = _build_thread_with_ztd_state(
            reason='ztd_consensus', ztd_value=0.4, ztd_sigma=0.20)
        ev['delta_m'] = 0.4   # ztd_consensus param shape
        ape._apply_integrity_trip(filt, mw, nl, ev)

        # ZTD state reset
        self.assertEqual(filt.x[filt.IDX_ZTD], 0.0)
        # NL fixes ARE reverted (legacy behavior)
        self.assertEqual(nl.unfix.call_count, 3,
                         "ztd_consensus must still unfix NL ambiguities "
                         "(peer-cohort signal stronger than local)")
        # AntPosEst state machine IS transitioned (legacy)
        ape._ape_sm.transition.assert_called_once()

    def test_ztd_sigma_m_default_when_filter_lacks_attr(self):
        """If filter has no _ztd_sigma_m attribute (e.g., pre-Charlie
        commit f67871d), fall back to 0.2m so the reset is well-defined."""
        ape, filt, mw, nl, ev = _build_thread_with_ztd_state(
            reason='ztd_impossible', ztd_value=2.0)
        del filt._ztd_sigma_m  # simulate older filter
        ape._apply_integrity_trip(filt, mw, nl, ev)
        self.assertAlmostEqual(filt.P[filt.IDX_ZTD, filt.IDX_ZTD],
                               0.2 ** 2)


if __name__ == '__main__':
    unittest.main()
