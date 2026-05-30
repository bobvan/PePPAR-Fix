"""Tests for the shared freerun analysis library (used by both
``do_freerun_char.py`` and ``do_freerun_char_phc.py``).  Covers the
pure-compute parts that don't need real TICC hardware: ``_sig``
sig-fig rounding, ``classify_noise_type`` PSD-slope buckets, the
``detrend_linear`` + ``compute_asd`` + ``loglog_slope`` pipeline on
synthetic phase data, ``analyze_samples`` end-to-end on a synthetic
sample stream, and ``merge_characterization_into_json`` JSON merge
semantics (preserves slope_cal, populates honest adev, etc.).

``collect_phase_series`` is not tested — it requires a real TICC.
"""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.dirname(_HERE)
for p in (_SCRIPTS, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from peppar_fix.freerun_analysis import (  # noqa: E402
    _PSD_FREQS_HZ,
    _TAUS_S,
    _sig,
    analyze_samples,
    classify_noise_type,
    compute_allantools_metrics,
    compute_asd,
    detrend_linear,
    loglog_slope,
    merge_characterization_into_json,
)


class SigFigRoundingTests(unittest.TestCase):
    """The clkPoC3-style ADEV bug motivator: fixed-decimal rounding
    zeroes the OCXO-class 1e-11..1e-13 ADEV values.  Sig-fig rounding
    must keep them meaningful."""

    def test_small_adev_value_preserved(self):
        # OCXO-class ADEV(1s) ~ 1e-11.  Fixed-decimal would zero it.
        self.assertNotEqual(_sig(8.72391e-11), 0.0)
        self.assertAlmostEqual(_sig(8.72391e-11), 8.72391e-11, places=15)

    def test_zero_is_preserved(self):
        self.assertEqual(_sig(0.0), 0.0)

    def test_none_is_preserved(self):
        self.assertIsNone(_sig(None))

    def test_nan_preserved(self):
        v = _sig(float('nan'))
        self.assertTrue(math.isnan(v))

    def test_large_value_rounded(self):
        self.assertEqual(_sig(123456.789, n=4), 123500.0)


class ClassifyNoiseTypeTests(unittest.TestCase):

    def test_white_phase(self):
        self.assertEqual(classify_noise_type(0.0), "white_phase")
        self.assertEqual(classify_noise_type(-0.4), "white_phase")

    def test_flicker_phase(self):
        self.assertEqual(classify_noise_type(-1.0), "flicker_phase")
        self.assertEqual(classify_noise_type(-0.7), "flicker_phase")

    def test_white_fm(self):
        self.assertEqual(classify_noise_type(-2.0), "white_FM")

    def test_flicker_fm(self):
        self.assertEqual(classify_noise_type(-3.0), "flicker_FM")

    def test_random_walk_fm(self):
        self.assertEqual(classify_noise_type(-4.0), "random_walk_FM")
        self.assertEqual(classify_noise_type(-5.0), "random_walk_FM")


class DetrendLinearTests(unittest.TestCase):

    def test_constant_drift_removed(self):
        # phase = 0.5 * t + noise → detrend removes the 0.5 slope.
        t = np.arange(100.0)
        y = 0.5 * t  # pure linear drift
        residual, slope, intercept = detrend_linear(t, y)
        self.assertAlmostEqual(slope, 0.5, places=10)
        # Residual should be ~zero (no noise added)
        self.assertLess(np.max(np.abs(residual)), 1e-9)

    def test_single_sample(self):
        # Edge case: too few samples for polyfit.
        t = np.array([0.0])
        y = np.array([5.0])
        residual, slope, intercept = detrend_linear(t, y)
        # Returns y - y.mean(), slope=0
        self.assertEqual(slope, 0.0)


class ComputeAsdTests(unittest.TestCase):
    """compute_asd on a known white-noise signal: ASD should be ~flat
    across frequencies, slope ~= 0 (white_phase)."""

    def test_white_noise_flat_asd(self):
        rng = np.random.default_rng(42)
        n = 600
        t = np.arange(n, dtype=float)
        y = rng.standard_normal(n)  # white phase noise, σ=1
        asd_at, freqs, asd = compute_asd(t, y, (0.01, 0.1, 0.3))
        # All target frequencies should be resolved at this duration
        self.assertIsNotNone(asd_at[0.1])
        # White → log-log slope on PSD should be near 0
        slope = loglog_slope(freqs, asd, f_lo=0.05, f_hi=0.4)
        self.assertGreater(slope, -0.7)   # not flicker territory
        self.assertLess(slope, 0.7)

    def test_too_few_samples(self):
        asd_at, freqs, asd = compute_asd(
            np.array([0.0, 1.0]), np.array([1.0, 2.0]),
            (0.1,))
        self.assertIsNone(asd_at[0.1])
        self.assertEqual(len(asd), 0)


class LoglogSlopeTests(unittest.TestCase):

    def test_constant_asd_gives_zero_slope(self):
        # Flat ASD = white phase.  Slope ~ 0.
        freqs = np.array([0.01, 0.05, 0.1, 0.3, 0.5])
        asd = np.ones_like(freqs)
        slope = loglog_slope(freqs, asd, f_lo=0.01, f_hi=0.5)
        self.assertAlmostEqual(slope, 0.0, places=6)

    def test_too_few_points(self):
        freqs = np.array([0.01, 0.1])
        asd = np.array([1.0, 1.0])
        slope = loglog_slope(freqs, asd, f_lo=0.01, f_hi=0.5)
        self.assertTrue(math.isnan(slope))


class ComputeAllantoolsTests(unittest.TestCase):

    def test_zero_phase_zero_adev(self):
        # Synthetic: constant phase → zero ADEV/TDEV.
        n = 1200
        y = np.zeros(n)
        adev, tdev, mdev = compute_allantools_metrics(y, 1.0, _TAUS_S)
        if adev is None:
            self.skipTest("allantools not available")
        for τ, a in adev.items():
            # Constant phase ⇒ ADEV is 0 (or numerically tiny)
            self.assertLess(a, 1e-12)


class AnalyzeSamplesEndToEndTest(unittest.TestCase):
    """End-to-end on a synthetic sample stream: TICC int-pair list →
    full analysis dict.  Validates the precision-preserving int-to-ns
    path and that the dict shape is what merge expects."""

    def _make_samples(self, n=600, freq_offset_ppb=0.0, noise_std_ps=1000.0):
        """Build a synthetic (ref_sec, ref_ps) list.

        TICC ref_sec is the integer part of the chA edge time
        (increments by 1 each second at 1 Hz).  ref_ps is the
        sub-second offset (0..999_999_999_999 ps).  The DO PPS edge
        wanders within the second according to its drift; 1 ppb of
        frequency offset = 1 ns/s of phase drift = 1000 ps/s
        accumulation in ref_ps.

        ``noise_std_ps`` adds zero-mean Gaussian noise to ref_ps per
        sample (TICC quantization + DO short-tau jitter).
        """
        rng = np.random.default_rng(1234)
        samples = []
        base_ps = 500_000_000_000  # mid-second start point
        for i in range(n):
            # Drift accumulates in the SUB-SECOND ref_ps; ref_sec
            # increments by 1 each sample (nominal 1 Hz cadence).
            ps_drift = int(round(i * freq_offset_ppb * 1000.0))
            noise = int(rng.normal(0.0, noise_std_ps))
            samples.append((i, base_ps + ps_drift + noise))
        return samples

    def test_too_few_samples_returns_none(self):
        samples = self._make_samples(n=30)  # below the 60-sample floor
        self.assertIsNone(analyze_samples(samples))

    def test_zero_drift_zero_freq_offset(self):
        samples = self._make_samples(n=600, freq_offset_ppb=0.0,
                                     noise_std_ps=0.0)
        result = analyze_samples(samples)
        self.assertIsNotNone(result)
        self.assertEqual(result['n_samples'], 600)
        self.assertAlmostEqual(result['freq_offset_ppb'], 0.0, places=6)
        self.assertLess(result['rms_ns'], 0.01)

    def test_nonzero_freq_offset_recovered(self):
        # Add 0.5 ppb drift over the capture.  freq_offset_ppb should
        # match (up to noise).
        samples = self._make_samples(n=600, freq_offset_ppb=0.5,
                                     noise_std_ps=10.0)
        result = analyze_samples(samples)
        self.assertIsNotNone(result)
        # Recovered slope vs commanded — same sign, same order of magnitude
        self.assertAlmostEqual(result['freq_offset_ppb'], 0.5, delta=0.05)

    def test_result_dict_shape(self):
        samples = self._make_samples(n=600)
        result = analyze_samples(samples)
        expected_keys = {
            'n_samples', 't_rel', 'residual_ns', 'sample_spacing_dts',
            'freq_offset_ppb', 'rms_ns', 'peak_ns',
            'asd_at_targets', 'freqs_arr', 'asd_arr',
            'psd_slope', 'noise_type', 'psd_curve',
            'adev_map', 'tdev_map', 'mdev_map',
        }
        self.assertEqual(set(result.keys()), expected_keys)


class MergeIntoJsonTests(unittest.TestCase):
    """``merge_characterization_into_json`` must preserve other
    top-level fields the DO state JSON carries (slope_cal output,
    dac_code_by_temperature, etc.)."""

    def _result_stub(self):
        # Minimal but valid result dict for the merge.
        return {
            'n_samples': 600,
            'freq_offset_ppb': 0.123,
            'rms_ns': 1.5,
            'peak_ns': 4.7,
            'asd_at_targets': {f: 0.1 for f in _PSD_FREQS_HZ},
            'psd_slope': -2.0,
            'noise_type': 'white_FM',
            'psd_curve': [[f, 0.1] for f in _PSD_FREQS_HZ],
            'adev_map': {1.0: 8.72e-11, 10.0: 2.74e-11},
            'tdev_map': {1.0: 0.05, 10.0: 0.15},
            'mdev_map': {1.0: 7.5e-11, 10.0: 2.3e-11},
        }

    def test_preserves_slope_cal_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "state.json"
            # Existing slope-cal output is at top level
            p.write_text(json.dumps({
                'dac_ppb_per_code': -0.0111,
                'dac_center_freq_offset_ppb': 85.6,
                'dac_code_by_temperature': {'25C': 41000},
                'characterization': {'old': 'value to be overwritten'},
            }))
            merge_characterization_into_json(
                p, self._result_stub(),
                parking_metadata={'parked_dac_code': 41000, 'parked_dac_gain': 1,
                                  'freq_offset_ppb_at_parked_code': 0.123},
                do_label='isotemp-ocxo-33-madhat',
                host='MadHat')
            data = json.loads(p.read_text())
            # slope_cal preserved
            self.assertAlmostEqual(data['dac_ppb_per_code'], -0.0111)
            self.assertAlmostEqual(data['dac_center_freq_offset_ppb'], 85.6)
            self.assertEqual(data['dac_code_by_temperature'], {'25C': 41000})
            # parking metadata in characterization
            self.assertEqual(data['characterization']['parked_dac_code'], 41000)

    def test_adev_nonzero_in_json(self):
        # The clkPoC3-bug invariant: adev values come out nonzero
        # (sig-fig rounding, not fixed-decimal).
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "state.json"
            merge_characterization_into_json(
                p, self._result_stub(),
                parking_metadata={'parked_dac_code': 32768, 'parked_dac_gain': 0,
                                  'freq_offset_ppb_at_parked_code': 0.0},
                do_label='ocxo-test', host='lab')
            data = json.loads(p.read_text())
            src = data['characterization']['sources']['DO PPS (chA vs TICC Rb)']
            for τ_str, a in src['adev_by_tau_s'].items():
                self.assertGreater(a, 0.0,
                                   f"adev[{τ_str}] = {a} is zero — sig-fig "
                                   f"rounding regression?")

    def test_null_characterization_section_handled(self):
        # Some older state JSONs carry 'characterization': null; the
        # merge must not crash on that.
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "state.json"
            p.write_text(json.dumps({'characterization': None, 'unique_id': 'x'}))
            merge_characterization_into_json(
                p, self._result_stub(),
                parking_metadata={'parked_dac_code': 32768, 'parked_dac_gain': 0,
                                  'freq_offset_ppb_at_parked_code': 0.0},
                do_label='ocxo-x', host='lab')
            data = json.loads(p.read_text())
            self.assertIsInstance(data['characterization'], dict)
            # unique_id preserved
            self.assertEqual(data['unique_id'], 'x')

    def test_phc_parking_metadata_in_json(self):
        # PHC variant writes adjfine fields instead of DAC fields.
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "state.json"
            merge_characterization_into_json(
                p, self._result_stub(),
                parking_metadata={
                    'parked_adjfine_ppb': 0.0,
                    'prior_adjfine_ppb_restored': -41.2,
                    'ptp_device': '/dev/ptp_i226',
                    'freq_offset_ppb_at_parked_state': 0.123,
                },
                do_label='i226-timehat-tcxo', host='TimeHat')
            data = json.loads(p.read_text())
            char = data['characterization']
            self.assertEqual(char['parked_adjfine_ppb'], 0.0)
            self.assertEqual(char['prior_adjfine_ppb_restored'], -41.2)
            self.assertEqual(char['ptp_device'], '/dev/ptp_i226')


if __name__ == '__main__':
    unittest.main()
