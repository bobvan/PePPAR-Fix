"""Tests for InBandNoiseEstimator's last-feed exposure (I-162848 Step 1).

The noise-buffer CSV writer in the engine reads `last_mono`,
`last_residual_ns`, and `last_channel` after each `feed()` call.
This file pins the exposure semantics:

  - Both attributes populated on every feed() call
  - channel='gap' when corrected_this_epoch=False
  - channel='correction' when corrected_this_epoch=True
  - residual_ns matches what got appended to the internal buffers
  - mono matches the timestamp passed (or time.monotonic() default)
"""
from __future__ import annotations

import os
import sys
import time
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.noise_estimator import InBandNoiseEstimator


class LastFeedExposureTest(unittest.TestCase):

    def test_pre_feed_attributes_are_none(self):
        est = InBandNoiseEstimator()
        self.assertIsNone(est.last_mono)
        self.assertIsNone(est.last_residual_ns)
        self.assertIsNone(est.last_channel)

    def test_gap_epoch_sets_channel_gap(self):
        est = InBandNoiseEstimator()
        est.feed(phase_error_ns=1.0, adjfine_ppb=0.0,
                 corrected_this_epoch=False, mono=100.0)
        self.assertEqual(est.last_mono, 100.0)
        self.assertAlmostEqual(est.last_residual_ns, 1.0)  # no detrending yet
        self.assertEqual(est.last_channel, 'gap')

    def test_correction_epoch_sets_channel_correction(self):
        est = InBandNoiseEstimator()
        est.feed(phase_error_ns=2.0, adjfine_ppb=0.0,
                 corrected_this_epoch=True, mono=100.0)
        self.assertEqual(est.last_mono, 100.0)
        self.assertAlmostEqual(est.last_residual_ns, 2.0)
        self.assertEqual(est.last_channel, 'correction')

    def test_residual_matches_buffer_append(self):
        # The exposed last_residual_ns must equal what got pushed into
        # the appropriate channel's buffer (gap goes into both buffers,
        # correction goes into corr only).
        est = InBandNoiseEstimator()
        # Seed prev_mono so detrending kicks in on next call.
        est.feed(phase_error_ns=0.0, adjfine_ppb=10.0,
                 corrected_this_epoch=False, mono=100.0)
        # Second feed at +1s with same adjfine: expected drift = 10ns.
        # Phase_error_ns=10 → residual = 10 - 10 = 0.
        est.feed(phase_error_ns=10.0, adjfine_ppb=10.0,
                 corrected_this_epoch=False, mono=101.0)
        self.assertAlmostEqual(est.last_residual_ns, 0.0)
        # Buffer's most recent entry should equal the exposed value.
        self.assertAlmostEqual(est._gap_phases[-1], est.last_residual_ns)
        self.assertAlmostEqual(est._corr_phases[-1], est.last_residual_ns)

    def test_correction_resets_phase_accumulator(self):
        # After a correction, _phase_acc_ns resets to 0 — next feed's
        # residual is just phase_error_ns (no accumulated drift).
        est = InBandNoiseEstimator()
        est.feed(0.0, 10.0, corrected_this_epoch=False, mono=100.0)
        est.feed(10.0, 10.0, corrected_this_epoch=True, mono=101.0)
        # Correction reset _phase_acc_ns; on next gap epoch with same
        # adjfine, residual will start fresh.  But also _last_correction_mono
        # is set, so we know correction took effect.
        self.assertEqual(est.last_channel, 'correction')
        # Phase accumulator was reset.
        self.assertEqual(est._phase_acc_ns, 0.0)

    def test_alternating_gap_correction_sequence(self):
        est = InBandNoiseEstimator()
        sequence = [
            (False, 'gap'),
            (False, 'gap'),
            (True, 'correction'),
            (False, 'gap'),
            (True, 'correction'),
        ]
        for i, (corrected, expected_channel) in enumerate(sequence):
            est.feed(phase_error_ns=float(i), adjfine_ppb=0.0,
                     corrected_this_epoch=corrected, mono=100.0 + i)
            self.assertEqual(est.last_channel, expected_channel,
                             f"row {i} channel mismatch")
            self.assertEqual(est.last_mono, 100.0 + i)

    def test_mono_default_uses_time_monotonic(self):
        est = InBandNoiseEstimator()
        before = time.monotonic()
        est.feed(0.0, 0.0, corrected_this_epoch=False)
        after = time.monotonic()
        self.assertGreaterEqual(est.last_mono, before)
        self.assertLessEqual(est.last_mono, after)


if __name__ == "__main__":
    unittest.main()
