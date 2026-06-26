"""Tests for pos_replay_compare — the offline position comparison that scores
a captured [PPP_STATE] trajectory against a static truth via the pos_sim
divergence monitor (the shared scoring layer)."""
import os
import sys
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix import pos_replay_compare as prc  # noqa: E402

_TRUTH = (-2_730_000.0, -4_440_000.0, 3_975_000.0)


def _ppp_line(epoch, ecef, sigma_pos, ztd=0.05, sigma_ztd=0.02, n=8):
    # mirror peppar_fix_engine's [PPP_STATE] format, with a logger prefix
    return ("2026-06-25 12:00:00,000 INFO  "
            "[PPP_STATE] epoch=%d n=%d ecef=%.4f,%.4f,%.4f "
            "sigma_pos=%.4fm ztd=%+.4fm sigma_ztd=%.4fm"
            % (epoch, n, ecef[0], ecef[1], ecef[2], sigma_pos, ztd, sigma_ztd))


class TestParse(unittest.TestCase):
    def test_parses_ppp_state_lines_ignoring_others(self):
        lines = [
            "2026-06-25 INFO [STATUS] unrelated",
            _ppp_line(5, (1.0, 2.0, 3.0), 0.15, ztd=-0.012, sigma_ztd=0.03, n=9),
            "noise",
        ]
        rows = prc.parse_ppp_state(lines)
        self.assertEqual(len(rows), 1)
        r = rows[0]
        self.assertEqual(r.epoch, 5)
        self.assertEqual(r.n_used, 9)
        self.assertEqual(r.ecef_m, (1.0, 2.0, 3.0))
        self.assertAlmostEqual(r.sigma_pos_m, 0.15)
        self.assertAlmostEqual(r.ztd_m, -0.012)
        self.assertAlmostEqual(r.sigma_ztd_m, 0.03)


class TestCompare(unittest.TestCase):
    def test_converged_trajectory_does_not_fire(self):
        # estimates sit ~5 cm from truth with honest σ → in corridor
        lines = [_ppp_line(i, (_TRUTH[0] + 0.05, _TRUTH[1], _TRUTH[2]), 0.15)
                 for i in range(200)]
        res = prc.compare_position(prc.parse_ppp_state(lines),
                                   prc.StaticTruth(_TRUTH))
        self.assertFalse(res["verdict"]["fired"])
        self.assertLess(res["final_error_m"], 0.1)

    def test_confident_divergence_fires(self):
        # error grows linearly while σ is small (confident) → fires
        lines = [_ppp_line(i, (_TRUTH[0] + 0.01 * i, _TRUTH[1], _TRUTH[2]), 0.12)
                 for i in range(300)]
        res = prc.compare_position(prc.parse_ppp_state(lines),
                                   prc.StaticTruth(_TRUTH))
        self.assertTrue(res["verdict"]["fired"])
        self.assertGreater(res["final_error_m"], 1.0)

    def test_honest_large_sigma_does_not_fire(self):
        # far from truth but σ honestly large (uncertain, not confidently wrong)
        lines = [_ppp_line(i, (_TRUTH[0] + 0.6, _TRUTH[1], _TRUTH[2]), 2.0)
                 for i in range(200)]
        res = prc.compare_position(prc.parse_ppp_state(lines),
                                   prc.StaticTruth(_TRUTH))
        self.assertFalse(res["verdict"]["fired"])

    def test_truth_sigma_added_in_quadrature(self):
        rows = prc.parse_ppp_state([_ppp_line(0, _TRUTH, 0.4)])
        # filter σ 0.4 ⊕ truth σ 0.3 = 0.5
        res = prc.compare_position(rows, prc.StaticTruth(_TRUTH, sigma_m=0.3))
        self.assertAlmostEqual(res["sigmas_m"][0], 0.5, places=6)
        res2 = prc.compare_position(rows, prc.StaticTruth(_TRUTH, sigma_m=0.3),
                                    include_truth_sigma=False)
        self.assertAlmostEqual(res2["sigmas_m"][0], 0.4, places=6)


class TestTruthHelpers(unittest.TestCase):
    def test_truth_from_ecef(self):
        t = prc.truth_from_ecef("-2730000,-4440000,3975000", sigma_m=0.012)
        self.assertEqual(t.ecef_m, (-2730000.0, -4440000.0, 3975000.0))
        self.assertAlmostEqual(t.sigma_m, 0.012)


if __name__ == "__main__":
    unittest.main()
