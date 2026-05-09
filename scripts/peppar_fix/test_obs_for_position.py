"""Unit tests for obs_for_position() — the per-filter gate that
drops MISS-phase-bias SVs from PPPFilter / MW tracker / WL bootstrap
while leaving the FixedPosFilter feed unfiltered.

Background: I-175645-charlie + docs/bds-b2a-phase-bias-survey-2026-05-09.md.
TD-CP cancels phase-bias constants between epochs, so the time
filter (FixedPosFilter via TD-CP) is insensitive to MISS phase
biases.  The position filter (PPPFilter, MW/WL math) IS sensitive —
half-corrected MW combination places WL integer search on a wrong
cycle → m-scale per-SV bias → multi-meter position basin.

The gate is one helper function called at the four position-side
consumer sites in peppar_fix_engine.py.  These tests cover the
helper itself; the engine integration is exercised by the per-host
overnight cohort runs at validation time.
"""
from __future__ import annotations

import unittest

from peppar_fix.obs_routing import obs_for_position


def _obs(sv, ar_phase_bias_ok=None, **extra):
    """Synthetic obs dict.  Only the fields obs_for_position cares
    about are set; the engine consumers expect more, but that's not
    this test's concern.
    """
    o = {'sv': sv}
    if ar_phase_bias_ok is not None:
        o['ar_phase_bias_ok'] = ar_phase_bias_ok
    o.update(extra)
    return o


class ObsForPositionTest(unittest.TestCase):

    def test_default_true_passes_obs_with_no_field(self):
        """Legacy / replay paths (no SSR stream → no ``ar_phase_bias_ok``
        key on obs) must pass the gate unchanged.  The default in
        ``obs.get('ar_phase_bias_ok', True)`` carries that promise.
        """
        obs = [_obs('G01'), _obs('G02'), _obs('E01')]
        out = obs_for_position(obs)
        self.assertEqual([o['sv'] for o in out], ['G01', 'G02', 'E01'])

    def test_drops_false_keeps_true(self):
        """Mixed list — explicit True passes, explicit False drops."""
        obs = [
            _obs('G01', ar_phase_bias_ok=True),
            _obs('C29', ar_phase_bias_ok=False),     # BDS B2a-I MISS
            _obs('G02', ar_phase_bias_ok=True),
            _obs('C42', ar_phase_bias_ok=False),     # BDS B2a-I MISS
            _obs('E01', ar_phase_bias_ok=True),
        ]
        out = obs_for_position(obs)
        self.assertEqual([o['sv'] for o in out], ['G01', 'G02', 'E01'])

    def test_default_true_mixes_with_explicit_false(self):
        """Realistic case: SSR stream populates the field for SVs whose
        biases were looked up, leaves it absent for SVs not yet seen.
        Absent → True (default), present-False → drop.
        """
        obs = [
            _obs('G01'),  # default True (newly admitted, bias not yet checked)
            _obs('C29', ar_phase_bias_ok=False),
            _obs('G05', ar_phase_bias_ok=True),
        ]
        out = obs_for_position(obs)
        self.assertEqual([o['sv'] for o in out], ['G01', 'G05'])

    def test_empty_list(self):
        self.assertEqual(obs_for_position([]), [])

    def test_all_dropped(self):
        """If every SV has MISS phase bias, the gate returns []."""
        obs = [_obs(s, ar_phase_bias_ok=False) for s in ('C29', 'C42')]
        self.assertEqual(obs_for_position(obs), [])

    def test_returns_same_dict_objects_not_copies(self):
        """The gate is a list comprehension that returns the original
        dicts (no deep-copy).  Important: callers that mutate obs in
        place (e.g., the AntPosEstThread PCV-correction path at
        ``peppar_fix_engine.py:2880-2885`` writes to ``obs['pr_if']``
        before the filter call) need their writes to land on the same
        dict the filter then consumes — not a copy.
        """
        o1 = _obs('G01', ar_phase_bias_ok=True)
        out = obs_for_position([o1])
        self.assertIs(out[0], o1)

    def test_preserves_order(self):
        """Filter is a comprehension; order is preserved.  Some
        downstream consumers (notably the MW tracker iteration) rely
        on stable per-epoch SV ordering for the resid_labels alignment.
        """
        obs = [
            _obs('G05', ar_phase_bias_ok=True),
            _obs('G01', ar_phase_bias_ok=False),
            _obs('G09', ar_phase_bias_ok=True),
            _obs('G03', ar_phase_bias_ok=True),
        ]
        out = obs_for_position(obs)
        self.assertEqual([o['sv'] for o in out], ['G05', 'G09', 'G03'])


if __name__ == '__main__':
    unittest.main()
