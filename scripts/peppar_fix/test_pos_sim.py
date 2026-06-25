"""Tests for pos_sim — the synthetic position-filter simulator + the
truth-relative divergence monitor (docs/simulators-and-replay.md)."""
import os
import sys
import unittest

import numpy as np

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix import pos_sim as ps  # noqa: E402

ARP = np.array([-2_730_000.0, -4_440_000.0, 3_975_000.0])


class TestEmitterFaithfulness(unittest.TestCase):
    def test_noiseless_seeded_at_truth_holds_truth(self):
        """With zero noise, no drift, and the filter seeded AT truth, the
        emitted obs are exactly h(truth) so the filter must not move — proves
        the emitter and filter share the same forward model."""
        truth = ps.Truth(arp_ecef=ARP, ztd0_m=0.05)
        rec = ps.run(ps.strong_sky(ARP), truth, n_epochs=120, seed=0,
                     seed_pos_offset_enu=(0, 0, 0), seed_ztd_offset_m=0.0,
                     sigma_code_m=0.0, sigma_carrier_m=0.0)
        self.assertLess(abs(rec["up_err_m"][-1]), 1e-2)
        self.assertLess(rec["pos_err_m"][-1], 1e-2)


class TestStrongGeometryConverges(unittest.TestCase):
    def test_pulls_back_a_seed_offset(self):
        """Well-conditioned geometry pulls a seeded up-offset back to truth."""
        truth = ps.Truth(arp_ecef=ARP, ztd0_m=0.05)
        rec = ps.run(ps.strong_sky(ARP), truth, n_epochs=300, seed=1,
                     seed_pos_offset_enu=(0, 0, 1.0),
                     monitor=ps.DivergenceMonitor(k_sigma=3.0, window=120))
        self.assertLess(abs(rec["up_err_m"][-1]), 0.3)
        self.assertFalse(rec["verdict"]["fired"])


class TestModerateGeometryDiverges(unittest.TestCase):
    def test_confident_divergence_fires_monitor(self):
        """Moderate geometry: σ collapses (confident) but a drifting ZTD leaks
        into up-position and grows past 3σ → the monitor must fire."""
        truth = ps.Truth(arp_ecef=ARP, ztd0_m=0.05, ztd_rate_m_s=2e-3)
        rec = ps.run(ps.moderate_sky(ARP), truth, n_epochs=600, seed=1,
                     ztd_sigma_m=0.05,
                     monitor=ps.DivergenceMonitor(k_sigma=3.0, window=120))
        self.assertTrue(rec["verdict"]["fired"])
        # confident: σ small; wrong: error large → genuine false confidence
        self.assertLess(rec["pos_sigma_m"][-1], 0.5)
        self.assertGreater(abs(rec["up_err_m"][-1]), 1.0)


class TestWeakGeometryStaysHonest(unittest.TestCase):
    def test_unobservable_keeps_large_sigma_no_fire(self):
        """Weak (degenerate) geometry must NOT fire: the error stays inside an
        honestly-large σ — uncertain, not confidently-wrong."""
        truth = ps.Truth(arp_ecef=ARP, ztd0_m=0.05, ztd_rate_m_s=1e-3)
        rec = ps.run(ps.weak_sky(ARP), truth, n_epochs=600, seed=1,
                     ztd_sigma_m=0.05,
                     monitor=ps.DivergenceMonitor(k_sigma=3.0, window=120))
        self.assertFalse(rec["verdict"]["fired"])
        self.assertGreater(rec["pos_sigma_m"][-1], 0.5)  # honestly uncertain


class TestDivergenceMonitorLogic(unittest.TestCase):
    """Direct unit tests of the corridor + early-abort rule, independent of
    the filter — far AND growing fires; far-but-settling and within-corridor
    do not."""

    def test_fires_when_far_and_growing(self):
        m = ps.DivergenceMonitor(k_sigma=3.0, window=10)
        fired = False
        for i in range(40):
            err = 0.1 + 0.5 * i      # grows well past 3σ
            fired = m.update(i, err, sigma=0.1) or fired
        self.assertTrue(fired)
        self.assertIsNotNone(m.verdict["fired_epoch"])

    def test_no_fire_when_settling(self):
        """Large but DECREASING error (transient settling) must not fire."""
        m = ps.DivergenceMonitor(k_sigma=3.0, window=10)
        for i in range(40):
            err = 10.0 / (1 + i)     # far early, decaying — settling
            m.update(i, err, sigma=0.1)
        self.assertFalse(m.verdict["fired"])

    def test_no_fire_within_corridor(self):
        """Error inside the ±3σ corridor must not fire even if it grows."""
        m = ps.DivergenceMonitor(k_sigma=3.0, window=10)
        for i in range(40):
            err = 0.01 * i           # grows but stays << 3σ (σ=1.0)
            m.update(i, err, sigma=1.0)
        self.assertFalse(m.verdict["fired"])

    def test_no_fire_before_window_full(self):
        m = ps.DivergenceMonitor(k_sigma=3.0, window=100)
        for i in range(50):          # fewer than `window` samples
            m.update(i, 100.0, sigma=0.1)
        self.assertFalse(m.verdict["fired"])


class TestGeometryHelpers(unittest.TestCase):
    def test_azel_roundtrips_through_filter_geometry(self):
        """A satellite placed at high elevation reads back as high-elevation
        through the filter's own geometric_range (shared forward model)."""
        from solve_ppp import PPPFilter
        filt = PPPFilter()
        sat = ps.azel_to_ecef(ARP, az_deg=45.0, el_deg=80.0)
        _rho, _e, elev, _sr = filt.geometric_range(ARP, sat)
        self.assertGreater(elev, 60.0)   # finite range lowers it, still high
        sat_low = ps.azel_to_ecef(ARP, az_deg=45.0, el_deg=10.0)
        _r2, _e2, elev_low, _s2 = filt.geometric_range(ARP, sat_low)
        self.assertLess(elev_low, elev)


class TestConstellationRank(unittest.TestCase):
    """The GPS-vs-Galileo story: with clean obs, adding a constellation only
    HELPS (rank/geometry); making GPS+GAL *worse* than Galileo-only requires a
    GPS-side observation error, not a filter-rank effect."""

    def _err(self, sky, bias=None):
        truth = ps.Truth(arp_ecef=ARP, ztd0_m=0.05, ztd_rate_m_s=1e-3)
        rec = ps.run(sky, truth, n_epochs=400, seed=1, ztd_sigma_m=0.05,
                     bias_fn=bias)
        return rec["pos_err_m"][-1]

    def test_adding_gps_rescues_galileo_only(self):
        gal = self._err(ps.gal_only_sky(ARP))
        dual = self._err(ps.gps_gal_sky(ARP))
        # clean dual must be clearly better than single (rank/geometry helps)
        self.assertLess(dual, gal)
        self.assertLess(dual, 0.5 * gal)

    def test_asymmetry_needs_an_obs_side_error(self):
        dual_clean = self._err(ps.gps_gal_sky(ARP))
        dual_biased = self._err(ps.gps_gal_sky(ARP), bias=ps.gps_elev_bias())
        # only an unmodeled GPS obs-error makes "adding GPS" hurt
        self.assertGreater(dual_biased, dual_clean + 0.05)

    def test_gps_bias_leaves_galileo_untouched(self):
        bias = ps.gps_elev_bias()
        self.assertEqual(bias("E01", "gal", 80.0), 0.0)
        self.assertNotEqual(bias("G01", "gps", 20.0), 0.0)


if __name__ == "__main__":
    unittest.main()
