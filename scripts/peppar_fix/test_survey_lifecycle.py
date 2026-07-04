"""Survey-provenance lifecycle + raw-log stop seam (I-071400 E3a).

ACQUIRING (coarse bootstrap σ_r) → REFINING (survey-class σ_r, not converged)
→ SURVEYED (survey converged → stop raw-obs logging).  Covers the pure
classifier, the transition machine, the RawCaptureBundle stop seam, and the
forward-compatible `.survey.toml` converged reader.
"""
import os
import sys
import tempfile
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.survey_lifecycle import (  # noqa: E402
    classify, SurveyLifecycle, SurveyLifecycleMachine)
from peppar_fix.raw_capture import RawCaptureBundle  # noqa: E402
import peppar_fix_engine as eng  # noqa: E402


class ClassifyTest(unittest.TestCase):
    def test_bootstrap_floor_is_acquiring(self):
        # σ_r at the 10 m trust floor (a --nav2-bootstrap seed) → ACQUIRING
        self.assertIs(classify(10.0, False), SurveyLifecycle.ACQUIRING)
        self.assertIs(classify(12.0, False), SurveyLifecycle.ACQUIRING)

    def test_survey_sigma_not_converged_is_refining(self):
        self.assertIs(classify(0.05, False), SurveyLifecycle.REFINING)
        self.assertIs(classify(9.99, False), SurveyLifecycle.REFINING)

    def test_converged_is_surveyed_regardless_of_sigma(self):
        self.assertIs(classify(0.008, True), SurveyLifecycle.SURVEYED)
        self.assertIs(classify(10.0, True), SurveyLifecycle.SURVEYED)

    def test_invalid_sigma_is_acquiring_unless_converged(self):
        for bad in (None, float('nan'), -1, 0, "x"):
            self.assertIs(classify(bad, False), SurveyLifecycle.ACQUIRING)
        self.assertIs(classify(None, True), SurveyLifecycle.SURVEYED)

    def test_custom_trusted_sigma(self):
        self.assertIs(classify(3.0, False, trusted_sigma_m=2.0),
                      SurveyLifecycle.ACQUIRING)
        self.assertIs(classify(1.0, False, trusted_sigma_m=2.0),
                      SurveyLifecycle.REFINING)


class MachineTest(unittest.TestCase):
    def test_full_progression_and_changed_edges(self):
        m = SurveyLifecycleMachine()
        s, changed = m.update(10.0, False)          # ACQUIRING (first tick)
        self.assertIs(s, SurveyLifecycle.ACQUIRING)
        self.assertTrue(changed)                     # first seed counts as edge
        s, changed = m.update(10.0, False)          # steady — no edge
        self.assertFalse(changed)
        s, changed = m.update(0.05, False)          # survey landed → REFINING
        self.assertIs(s, SurveyLifecycle.REFINING)
        self.assertTrue(changed)
        s, changed = m.update(0.008, True)          # converged → SURVEYED
        self.assertIs(s, SurveyLifecycle.SURVEYED)
        self.assertTrue(changed)
        s, changed = m.update(0.008, True)          # steady SURVEYED
        self.assertFalse(changed)

    def test_starts_surveyed_when_seed_already_converged(self):
        m = SurveyLifecycleMachine()
        s, changed = m.update(0.008, True)
        self.assertIs(s, SurveyLifecycle.SURVEYED)
        self.assertTrue(changed)                     # act on the edge → stop log


class RawCaptureStopTest(unittest.TestCase):
    def test_stop_recording_halts_all_streams_idempotent(self):
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.record("ubx", b"A", 1.0)
            b.record_line("ticc", "line", 2.0)
            self.assertEqual(b.counts.get("ubx"), 1)
            self.assertEqual(b.counts.get("ticc"), 1)
            self.assertTrue(b.recording)
            self.assertTrue(b.stop_recording())       # first call stops
            self.assertFalse(b.recording)
            b.record("ubx", b"B", 3.0)                 # no-op now
            b.record_line("ticc", "line2", 4.0)
            self.assertEqual(b.counts.get("ubx"), 1)   # unchanged
            self.assertEqual(b.counts.get("ticc"), 1)
            self.assertFalse(b.stop_recording())       # idempotent → False
            b.close()

    def test_engine_log_not_gated_by_stop(self):
        # Group-B engine_log is NOT raw obs — stop_recording must not gate it.
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.stop_recording()
            b.engine_log("ppp_state.log", "[PPP_STATE] x")
            b.close()                                  # flush buffered fh
            with open(os.path.join(d, "engine", "ppp_state.log")) as f:
                self.assertIn("PPP_STATE", f.read())


def _write_survey(path, *, converged=None):
    lines = [
        'ecef_m = [3979160.466, -4257.850, 4968043.152]',
        'sigma_m = 0.008',
        'mount_sn = 1',
        'updated = "2026-07-04T00:00:00+00:00"',
        'source = "pride-final"',
        'frame = "ITRF2020@2026.50"',
    ]
    if converged is not None:
        lines.append(f'converged = {"true" if converged else "false"}')
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


class ReadSurveyConvergedTest(unittest.TestCase):
    def test_converged_true_false_absent(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "s.survey.toml")
            _write_survey(p, converged=True)
            self.assertTrue(eng._read_survey_converged(p))
            _write_survey(p, converged=False)
            self.assertFalse(eng._read_survey_converged(p))
            _write_survey(p, converged=None)            # field absent (old file)
            self.assertFalse(eng._read_survey_converged(p))

    def test_missing_or_none_path_is_false(self):
        self.assertFalse(eng._read_survey_converged(None))
        self.assertFalse(eng._read_survey_converged("/no/such/file.survey.toml"))


if __name__ == "__main__":
    unittest.main()
