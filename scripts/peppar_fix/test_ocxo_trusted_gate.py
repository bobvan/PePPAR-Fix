"""Tests for OcxoTrustedGate and load_sigma_short_tau_from_state."""
from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.dirname(_HERE)
for p in (_SCRIPTS, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from peppar_fix.ocxo_trusted_gate import (  # noqa: E402
    OcxoTrustedGate,
    load_sigma_short_tau_from_state,
)


class OcxoTrustedGateBasicTests(unittest.TestCase):

    def test_construction_rejects_bad_args(self):
        with self.assertRaises(ValueError):
            OcxoTrustedGate(sigma_short_tau_ns=0.0)
        with self.assertRaises(ValueError):
            OcxoTrustedGate(sigma_short_tau_ns=-1.0)
        with self.assertRaises(ValueError):
            OcxoTrustedGate(sigma_short_tau_ns=0.05, k_sigma=0.0)

    def test_accepts_small_innovation(self):
        # σ=0.054 ns (PiFace freerun), k=10 → threshold = 0.54 ns at τ=1 s
        g = OcxoTrustedGate(sigma_short_tau_ns=0.054, k_sigma=10.0, min_age_s=0.0)
        accept, reason = g.evaluate(innov_ns=0.3, dt_s=1.0, age_s=1000.0)
        self.assertTrue(accept, f"0.3 ns innov should pass; got {reason}")
        self.assertEqual(g.n_accepted, 1)
        self.assertEqual(g.n_rejected, 0)

    def test_rejects_large_innovation(self):
        # 10 ns innov vs 540 ps threshold → rejected
        g = OcxoTrustedGate(sigma_short_tau_ns=0.054, k_sigma=10.0, min_age_s=0.0)
        accept, reason = g.evaluate(innov_ns=10.0, dt_s=1.0, age_s=1000.0)
        self.assertFalse(accept)
        self.assertIn('ocxo_gate', reason)
        self.assertEqual(g.n_rejected, 1)

    def test_pre_min_age_always_accepts(self):
        # Same large innov but age=10s < min_age=60s → accept
        g = OcxoTrustedGate(sigma_short_tau_ns=0.054, k_sigma=10.0, min_age_s=60.0)
        accept, reason = g.evaluate(innov_ns=10.0, dt_s=1.0, age_s=10.0)
        self.assertTrue(accept)
        self.assertEqual(reason, "pre_min_age")
        self.assertEqual(g.n_skipped_pre_age, 1)
        self.assertEqual(g.n_rejected, 0)

    def test_dt_scales_threshold(self):
        # At dt=100s, threshold should be sqrt(100)=10× the 1s threshold
        g = OcxoTrustedGate(sigma_short_tau_ns=0.054, k_sigma=10.0, min_age_s=0.0)
        # 4 ns innov: rejected at dt=1 (threshold 0.54), accepted at dt=100
        # (threshold = 0.054 * 10 * sqrt(100) = 5.4 ns)
        accept1, _ = g.evaluate(innov_ns=4.0, dt_s=1.0, age_s=1000.0)
        accept100, _ = g.evaluate(innov_ns=4.0, dt_s=100.0, age_s=1000.0)
        self.assertFalse(accept1)
        self.assertTrue(accept100)

    def test_stats(self):
        g = OcxoTrustedGate(sigma_short_tau_ns=0.054, k_sigma=10.0, min_age_s=60.0,
                            do_label="ocxo-piface")
        g.evaluate(innov_ns=10.0, dt_s=1.0, age_s=30.0)   # pre-age
        g.evaluate(innov_ns=10.0, dt_s=1.0, age_s=300.0)  # rejected
        g.evaluate(innov_ns=0.3, dt_s=1.0, age_s=300.0)   # accepted
        s = g.stats
        self.assertEqual(s['n_skipped_pre_age'], 1)
        self.assertEqual(s['n_rejected'], 1)
        self.assertEqual(s['n_accepted'], 1)
        self.assertEqual(s['do_label'], "ocxo-piface")


class LoadSigmaTests(unittest.TestCase):

    def test_loads_from_freerun_char_schema(self):
        # Schema 1: do_freerun_char.py output
        data = {
            'characterization': {
                'sources': {
                    'DO PPS (chA-chB)': {
                        'tdev_ns_by_tau_s': {
                            '1.0': 0.054,
                            '10.0': 0.101,
                        },
                    },
                },
            },
        }
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(data, f)
            p = Path(f.name)
        try:
            v = load_sigma_short_tau_from_state(p, target_tau_s=1)
            self.assertEqual(v, 0.054)
        finally:
            p.unlink()

    def test_loads_from_timehat_analyzer_schema(self):
        # Schema 2: TimeHat offline analyzer
        data = {
            'characterization': {
                'tdev_ns': {
                    '1': 3.179,
                    '10': 9.987,
                },
            },
        }
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(data, f)
            p = Path(f.name)
        try:
            v = load_sigma_short_tau_from_state(p, target_tau_s=1)
            self.assertEqual(v, 3.179)
        finally:
            p.unlink()

    def test_prefers_chA_vs_Rb_when_both_present(self):
        # If both stale "chA-chB" and correct "chA vs TICC Rb" keys are
        # present, prefer the correctly-named one.
        data = {
            'characterization': {
                'sources': {
                    'DO PPS (chA-chB)':           {'tdev_ns_by_tau_s': {'1.0': 9.999}},
                    'DO PPS (chA vs TICC Rb)':    {'tdev_ns_by_tau_s': {'1.0': 0.045}},
                },
            },
        }
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(data, f)
            p = Path(f.name)
        try:
            v = load_sigma_short_tau_from_state(p, target_tau_s=1)
            self.assertEqual(v, 0.045)
        finally:
            p.unlink()

    def test_returns_none_when_file_missing(self):
        v = load_sigma_short_tau_from_state(
            Path("/nonexistent/path/state.json"), target_tau_s=1)
        self.assertIsNone(v)

    def test_returns_none_when_no_tdev_at_tau(self):
        data = {'characterization': {}}
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(data, f)
            p = Path(f.name)
        try:
            v = load_sigma_short_tau_from_state(p, target_tau_s=1)
            self.assertIsNone(v)
        finally:
            p.unlink()


class OcxoTrustedGateRegimeTests(unittest.TestCase):
    """v2 acquiring/locked regime state machine (regime_aware=True)."""

    def _gate(self, **kw):
        defaults = dict(sigma_short_tau_ns=0.054, k_sigma=10.0, min_age_s=0.0,
                        regime_aware=True, lock_bias_ratio=0.4,
                        unlock_bias_ratio=0.7, ema_alpha=0.05,
                        lock_dwell_s=5.0, min_lock_samples=10,
                        do_label="ocxo-test")
        defaults.update(kw)
        return OcxoTrustedGate(**defaults)

    def _feed(self, g, n, mean, noise, rng, dt=1.0, age0=0.0, age_step=1.0):
        """Feed n innovations ~ N(mean, noise); return (last_accept, last_reason)."""
        last = (True, "")
        for i in range(n):
            x = mean + rng.gauss(0.0, noise)
            last = g.evaluate(innov_ns=x, dt_s=dt, age_s=age0 + i * age_step)
        return last

    def test_disabled_is_v1(self):
        # regime_aware=False → large innov rejected after min_age (v1).
        g = OcxoTrustedGate(sigma_short_tau_ns=0.054, k_sigma=10.0,
                            min_age_s=0.0, regime_aware=False)
        accept, reason = g.evaluate(innov_ns=10.0, dt_s=1.0, age_s=1000.0)
        self.assertFalse(accept)
        self.assertEqual(g.regime, "acquiring")  # never transitions in v1

    def test_acquiring_passes_through_large_innov(self):
        # While acquiring (biased innovations), even huge innovations
        # pass — defer to chi²/√S.  This is the clkPoC3 fix.
        import random
        g = self._gate()
        accept, reason = self._feed(g, 40, mean=5.0, noise=0.5,
                                    rng=random.Random(1))
        self.assertEqual(g.regime, "acquiring")
        self.assertTrue(accept)
        self.assertEqual(reason, "acquiring")
        self.assertEqual(g.n_rejected, 0)  # nothing rejected while acquiring

    def test_locks_after_zero_mean_dwell(self):
        import random
        g = self._gate()
        self._feed(g, 50, mean=0.0, noise=1.0, rng=random.Random(2))
        self.assertEqual(g.regime, "locked")
        self.assertLess(g.bias_ratio, g.lock_bias_ratio)

    def test_locked_rejects_large_innov(self):
        import random
        g = self._gate()
        self._feed(g, 50, mean=0.0, noise=1.0, rng=random.Random(3))
        self.assertEqual(g.regime, "locked")
        # One large innovation at dt=1 (threshold 0.54 ns) → tight reject,
        # and one outlier must NOT flip us out of locked.
        accept, reason = g.evaluate(innov_ns=10.0, dt_s=1.0, age_s=200.0)
        self.assertFalse(accept)
        self.assertIn("locked", reason)
        self.assertEqual(g.regime, "locked")

    def test_unlocks_on_sustained_bias(self):
        import random
        g = self._gate()
        self._feed(g, 50, mean=0.0, noise=1.0, rng=random.Random(4))
        self.assertEqual(g.regime, "locked")
        # Sustained biased innovations (loop fell behind) → re-acquire.
        self._feed(g, 60, mean=8.0, noise=0.5, rng=random.Random(5),
                   age0=100.0)
        self.assertEqual(g.regime, "acquiring")

    def test_pre_min_age_stays_acquiring(self):
        g = self._gate(min_age_s=60.0)
        accept, reason = g.evaluate(innov_ns=10.0, dt_s=1.0, age_s=30.0)
        self.assertTrue(accept)
        self.assertEqual(reason, "pre_min_age")
        self.assertEqual(g.regime, "acquiring")

    def test_acquire_loose_gate_when_k_acquire_set(self):
        # k_sigma_acquire set → loose threshold during acquisition
        # instead of pass-through.
        import random
        g = self._gate(k_sigma_acquire=1000.0)
        # huge K → 0.054*1000*1 = 54 ns threshold; 10 ns passes, 100 ns fails
        a1, r1 = g.evaluate(innov_ns=10.0, dt_s=1.0, age_s=1.0)
        a2, r2 = g.evaluate(innov_ns=100.0, dt_s=1.0, age_s=2.0)
        self.assertEqual(g.regime, "acquiring")
        self.assertTrue(a1)
        self.assertFalse(a2)
        self.assertIn("acquiring", r1)

    def test_bias_ratio_bounds(self):
        import random
        g = self._gate()
        # zero-mean → low ratio
        self._feed(g, 60, mean=0.0, noise=1.0, rng=random.Random(6))
        self.assertLess(g.bias_ratio, 0.4)
        # heavily biased → high ratio
        g2 = self._gate()
        self._feed(g2, 60, mean=10.0, noise=0.3, rng=random.Random(7))
        self.assertGreater(g2.bias_ratio, 0.7)

    def test_stats_exposes_regime(self):
        g = self._gate()
        s = g.stats
        self.assertIn('regime', s)
        self.assertIn('bias_ratio', s)
        self.assertTrue(s['regime_aware'])


if __name__ == '__main__':
    unittest.main()
