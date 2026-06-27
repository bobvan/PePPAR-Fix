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


def _ppp_line(epoch, ecef, sigma_pos, ztd=0.05, sigma_ztd=0.02, n=8, gps=None):
    # mirror peppar_fix_engine's [PPP_STATE] format, with a logger prefix
    gps_field = f"gps={gps} " if gps else ""
    return ("2026-06-25 12:00:00,000 INFO  "
            "[PPP_STATE] %sepoch=%d n=%d ecef=%.4f,%.4f,%.4f "
            "sigma_pos=%.4fm ztd=%+.4fm sigma_ztd=%.4fm"
            % (gps_field, epoch, n, ecef[0], ecef[1], ecef[2],
               sigma_pos, ztd, sigma_ztd))


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


class TestGpsKey(unittest.TestCase):
    def test_parses_gps_field_when_present(self):
        ln = _ppp_line(1, _TRUTH, 0.1, gps="2026-06-25T12:00:01+00:00")
        r = prc.parse_ppp_state([ln])[0]
        self.assertIsNotNone(r.gps)
        self.assertEqual(r.gps.year, 2026)

    def test_back_compat_no_gps_field(self):
        r = prc.parse_ppp_state([_ppp_line(1, _TRUTH, 0.1)])[0]
        self.assertIsNone(r.gps)            # old logs (no gps=) still parse


class TestZtdCompare(unittest.TestCase):
    def _pt(self, t, ztd, sig=0.01):
        return prc.ZtdPoint(t_s=float(t), ztd_m=ztd, sigma_ztd_m=sig)

    def test_constant_offset_is_absorbed_no_fire(self):
        # our = truth + 0.30 everywhere (lag + apriori) → offset removed,
        # detrended ≈ 0 → no fire.
        truth = [self._pt(i, 0.10) for i in range(200)]
        our = [self._pt(i, 0.10 + 0.30) for i in range(200)]
        res = prc.compare_ztd(our, truth)
        self.assertAlmostEqual(res["offset_m"], 0.30, places=6)
        self.assertLess(res["final_detrended_m"], 1e-6)
        self.assertFalse(res["verdict"]["fired"])

    def test_growing_departure_fires(self):
        # a time-varying departure on top of a constant offset → detrended
        # grows → fires (the real ZTD-misallocation signal).
        truth = [self._pt(i, 0.10) for i in range(300)]
        our = [self._pt(i, 0.10 + 0.20 + 0.002 * i, sig=0.01)
               for i in range(300)]
        res = prc.compare_ztd(our, truth)
        self.assertTrue(res["verdict"]["fired"])

    def test_alignment_tolerance(self):
        # truth points 1000 s away → nothing aligns
        truth = [self._pt(10000 + i, 0.10) for i in range(50)]
        our = [self._pt(i, 0.10) for i in range(50)]
        res = prc.compare_ztd(our, truth, align_tol_s=60.0)
        self.assertEqual(res["n_aligned"], 0)

    def test_short_series_is_inconclusive(self):
        # fewer aligned pairs than the window can never fire → inconclusive,
        # not a falsely reassuring "stayed in corridor".
        truth = [self._pt(i, 0.10) for i in range(30)]
        our = [self._pt(i, 0.10) for i in range(30)]
        res = prc.compare_ztd(our, truth, window=120)
        self.assertEqual(res["n_aligned"], 30)
        self.assertTrue(res["inconclusive"])
        self.assertFalse(res["verdict"]["fired"])

    def test_long_series_is_not_inconclusive(self):
        truth = [self._pt(i, 0.10) for i in range(200)]
        our = [self._pt(i, 0.10) for i in range(200)]
        res = prc.compare_ztd(our, truth, window=120)
        self.assertFalse(res["inconclusive"])

    def test_empty_is_inconclusive(self):
        res = prc.compare_ztd([], [])
        self.assertTrue(res["inconclusive"])
        self.assertEqual(res["n_aligned"], 0)

    def test_fired_epoch_is_gps_time_not_pair_index(self):
        # the fired epoch should be the aligned GPS-time (unix seconds), a
        # large timestamp — not a small pair index.
        base = 1_750_000_000
        truth = [self._pt(base + i, 0.10) for i in range(300)]
        our = [self._pt(base + i, 0.10 + 0.20 + 0.002 * i, sig=0.01)
               for i in range(300)]
        res = prc.compare_ztd(our, truth)
        self.assertTrue(res["verdict"]["fired"])
        self.assertGreater(res["verdict"]["fired_epoch"], base)

    def test_series_from_ppp_uses_gps_keyed_rows_only(self):
        rows = prc.parse_ppp_state([
            _ppp_line(1, _TRUTH, 0.1, ztd=0.07,
                      gps="2026-06-25T12:00:01+00:00"),
            _ppp_line(2, _TRUTH, 0.1, ztd=0.08),   # no gps → skipped
        ])
        series = prc.ztd_series_from_ppp(rows)
        self.assertEqual(len(series), 1)
        self.assertAlmostEqual(series[0].ztd_m, 0.07)


class TestZtdApriori(unittest.TestCase):
    """logZtdApriori (Charlie #235 finding 2): the engine logs its ZTD apriori
    so the total assembly uses the run's ACTUAL value, not the current constant
    — old captures stay correct across an apriori retune."""

    def test_parses_logged_apriori(self):
        self.assertAlmostEqual(prc.parse_ztd_apriori(
            ["2026-06-25 INFO [ZTD_APRIORI] m=2.3000", "noise"]), 2.3)

    def test_returns_none_when_absent(self):
        self.assertIsNone(prc.parse_ztd_apriori(["[PPP_STATE] gps=x epoch=1"]))

    def test_parses_retuned_apriori(self):
        # a future retune → the compare reads THAT value, not the constant
        self.assertAlmostEqual(
            prc.parse_ztd_apriori(["[ZTD_APRIORI] m=2.2500"]), 2.25)

    def test_total_assembly_uses_parsed_apriori(self):
        rows = prc.parse_ppp_state([_ppp_line(
            1, _TRUTH, 0.1, ztd=0.07, gps="2026-06-25T12:00:01+00:00")])
        series = prc.ztd_total_series_from_ppp(rows, apriori_m=2.25)
        self.assertAlmostEqual(series[0].ztd_m, 2.32)   # 2.25 + 0.07, not 2.37


class TestTotalZtdAssembly(unittest.TestCase):
    def test_total_is_apriori_plus_residual(self):
        rows = prc.parse_ppp_state([
            _ppp_line(1, _TRUTH, 0.1, ztd=0.07,
                      gps="2026-06-25T12:00:01+00:00")])
        series = prc.ztd_total_series_from_ppp(rows, apriori_m=2.3)
        self.assertAlmostEqual(series[0].ztd_m, 2.37, places=6)   # 2.3 + 0.07

    def test_total_default_apriori_is_engine_constant(self):
        from peppar_fix.saastamoinen import ENGINE_ZTD_APRIORI_M
        rows = prc.parse_ppp_state([
            _ppp_line(1, _TRUTH, 0.1, ztd=0.05,
                      gps="2026-06-25T12:00:01+00:00")])
        series = prc.ztd_total_series_from_ppp(rows)
        self.assertAlmostEqual(series[0].ztd_m, ENGINE_ZTD_APRIORI_M + 0.05,
                               places=6)

    def test_total_skips_rows_without_gps(self):
        rows = prc.parse_ppp_state([_ppp_line(1, _TRUTH, 0.1, ztd=0.05)])
        self.assertEqual(prc.ztd_total_series_from_ppp(rows), [])

    def test_engine_applies_the_same_apriori_the_compare_assumes(self):
        # Finding 1: the engine's tropo apriori (solve_ppp) and the constant
        # the compare assembles from MUST be one value.  At zenith the engine
        # tropo IS the apriori (mapping=1) — assert they match so a future
        # retune in one place can't silently mis-assemble every total.
        import sys
        sys.path.insert(0, _SCRIPTS)
        from solve_ppp import PPPFilter
        from peppar_fix.saastamoinen import ENGINE_ZTD_APRIORI_M
        f = PPPFilter.__new__(PPPFilter)
        self.assertAlmostEqual(f.tropo_delay(90.0), ENGINE_ZTD_APRIORI_M,
                               places=9)


class TestAbsoluteBias(unittest.TestCase):
    def _pt(self, t, ztd, sig=0.01):
        return prc.ZtdPoint(t_s=float(t), ztd_m=ztd, sigma_ztd_m=sig)

    def test_large_constant_bias_warns(self):
        # our_total sits 0.2 m above truth, flat → detrended verdict can't see
        # it, but the absolute-bias check warns.
        truth = [self._pt(i, 2.45) for i in range(200)]
        our = [self._pt(i, 2.65) for i in range(200)]
        res = prc.compare_ztd(our, truth, abs_bias_warn_m=0.10)
        self.assertFalse(res["verdict"]["fired"])     # flat → no divergence
        self.assertTrue(res["abs_bias_warned"])        # but biased
        self.assertAlmostEqual(res["offset_m"], 0.20, places=6)

    def test_small_bias_does_not_warn(self):
        truth = [self._pt(i, 2.45) for i in range(200)]
        our = [self._pt(i, 2.47) for i in range(200)]   # 2 cm, within tol
        res = prc.compare_ztd(our, truth, abs_bias_warn_m=0.10)
        self.assertFalse(res["abs_bias_warned"])

    def test_abs_bias_disabled_by_default(self):
        truth = [self._pt(i, 2.45) for i in range(200)]
        our = [self._pt(i, 4.75) for i in range(200)]   # huge (residual-style)
        res = prc.compare_ztd(our, truth)               # no warn arg
        self.assertFalse(res["abs_bias_warned"])


class TestMetarParse(unittest.TestCase):
    def test_parses_metar_line(self):
        ln = ("2026-06-25 INFO [METAR] epoch=42 KDPA age=12min T=21.0C "
              "dewp=10.0C altim=1013.2hPa Pstn=1000.0hPa e=12.3hPa "
              "ZHD=2.3000m ZWD=0.1500m ZTD=2.4500m")
        rows = prc.parse_metar([ln, "noise"])
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0].epoch, 42)
        self.assertAlmostEqual(rows[0].zhd_m, 2.30, places=6)
        self.assertAlmostEqual(rows[0].zwd_m, 0.15, places=6)
        self.assertAlmostEqual(rows[0].ztd_m, 2.45, places=6)


class TestInterpolateZtd(unittest.TestCase):
    def _pt(self, t, ztd, sig=0.01):
        return prc.ZtdPoint(t_s=float(t), ztd_m=ztd, sigma_ztd_m=sig)

    def test_linear_interp_onto_dense_times(self):
        # sparse truth at 0 and 300 s; interpolate onto a mid time
        truth = [self._pt(0, 2.450), self._pt(300, 2.456)]
        out = prc.interpolate_ztd(truth, [150.0])
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(out[0].ztd_m, 2.453, places=6)  # midpoint
        self.assertAlmostEqual(out[0].t_s, 150.0)

    def test_exact_time_passes_through(self):
        truth = [self._pt(0, 2.450), self._pt(300, 2.456)]
        out = prc.interpolate_ztd(truth, [300.0])
        self.assertAlmostEqual(out[0].ztd_m, 2.456, places=6)

    def test_outside_span_is_dropped_not_extrapolated(self):
        truth = [self._pt(100, 2.450), self._pt(400, 2.456)]
        out = prc.interpolate_ztd(truth, [50.0, 250.0, 500.0])
        self.assertEqual([round(p.t_s) for p in out], [250])  # only in-span kept

    def test_empty_truth(self):
        self.assertEqual(prc.interpolate_ztd([], [1.0, 2.0]), [])

    def test_interp_makes_whole_dense_series_usable(self):
        # 5-min truth, 1 Hz ours: nearest-tol would drop most; interp keeps all
        truth = [self._pt(300 * i, 2.45 + 0.001 * i) for i in range(5)]   # 0..1200
        our_t = [float(t) for t in range(0, 1201)]
        out = prc.interpolate_ztd(truth, our_t)
        self.assertEqual(len(out), len(our_t))   # every 1 Hz point covered


class TestZtdSeriesFromTro(unittest.TestCase):
    def test_adapts_tro_to_ztdpoints(self):
        import tempfile
        text = ("+TROP/SOLUTION\n"
                "*SITE ____EPOCH___ TROTOT STDDEV\n"
                " ABCD 26:001:00000 2451.6    1.2\n"
                " ABCD 26:001:00300 2452.0    1.1\n"
                "-TROP/SOLUTION\n")
        with tempfile.NamedTemporaryFile("w", suffix=".tro", delete=False) as f:
            f.write(text)
            path = f.name
        try:
            series = prc.ztd_series_from_tro(path)
        finally:
            os.unlink(path)
        self.assertEqual(len(series), 2)
        self.assertAlmostEqual(series[0].ztd_m, 2.4516, places=6)
        self.assertIsInstance(series[0], prc.ZtdPoint)


if __name__ == "__main__":
    unittest.main()
