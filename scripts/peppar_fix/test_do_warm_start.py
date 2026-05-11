"""Tests for is_do_warm_startable (dacWarmStart-main)."""

from __future__ import annotations

import json
import os
import tempfile
import time
import unittest

from peppar_fix.do_state import is_do_warm_startable, save_do_state


class _WarmStartGate(unittest.TestCase):

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = self._tmp.name
        self.uid = "ocxo-test"

    def tearDown(self):
        self._tmp.cleanup()

    def _write(self, freq_ppb, age_s=0.0):
        state = {
            "unique_id": self.uid,
            "label": self.uid,
            "last_known_freq_offset_ppb": freq_ppb,
        }
        save_do_state(state, state_dir=self.dir)
        if age_s > 0:
            path = os.path.join(self.dir, f"{self.uid}.json")
            target = time.time() - age_s
            os.utime(path, (target, target))

    def test_missing_state_file(self):
        ok, info = is_do_warm_startable(self.uid, state_dir=self.dir)
        self.assertFalse(ok)
        self.assertEqual(info["reason"], "no_state_file")

    def test_fresh_state_passes(self):
        self._write(150.0)
        ok, info = is_do_warm_startable(self.uid, state_dir=self.dir)
        self.assertTrue(ok)
        self.assertAlmostEqual(info["freq_ppb"], 150.0)
        self.assertLess(info["age_s"], 5.0)

    def test_too_old_fails(self):
        self._write(150.0, age_s=100_000)
        ok, info = is_do_warm_startable(self.uid, max_age_s=86400,
                                        state_dir=self.dir)
        self.assertFalse(ok)
        self.assertIn("too_old", info["reason"])

    def test_envelope_violation_fails(self):
        self._write(5000.0)
        ok, info = is_do_warm_startable(self.uid, max_ppb=500.0,
                                        state_dir=self.dir)
        self.assertFalse(ok)
        self.assertIn("out_of_envelope", info["reason"])

    def test_negative_freq_within_envelope_passes(self):
        self._write(-141.0)
        ok, info = is_do_warm_startable(self.uid, max_ppb=500.0,
                                        state_dir=self.dir)
        self.assertTrue(ok)
        self.assertAlmostEqual(info["freq_ppb"], -141.0)

    def test_no_last_known_freq_fails(self):
        save_do_state({"unique_id": self.uid, "label": self.uid},
                      state_dir=self.dir)
        ok, info = is_do_warm_startable(self.uid, state_dir=self.dir)
        self.assertFalse(ok)
        self.assertEqual(info["reason"], "no_last_known_freq")

    def test_non_numeric_freq_fails(self):
        save_do_state({"unique_id": self.uid, "label": self.uid,
                       "last_known_freq_offset_ppb": "not a number"},
                      state_dir=self.dir)
        ok, info = is_do_warm_startable(self.uid, state_dir=self.dir)
        self.assertFalse(ok)
        self.assertIn("not_numeric", info["reason"])

    def test_infinite_freq_fails(self):
        save_do_state({"unique_id": self.uid, "label": self.uid,
                       "last_known_freq_offset_ppb": float("inf")},
                      state_dir=self.dir)
        ok, info = is_do_warm_startable(self.uid, state_dir=self.dir)
        self.assertFalse(ok)
        self.assertIn("not_finite", info["reason"])

    def test_malformed_json_fails(self):
        path = os.path.join(self.dir, f"{self.uid}.json")
        with open(path, "w") as f:
            f.write("{not valid json")
        ok, info = is_do_warm_startable(self.uid, state_dir=self.dir)
        self.assertFalse(ok)
        self.assertEqual(info["reason"], "load_failed")


if __name__ == "__main__":
    unittest.main()
