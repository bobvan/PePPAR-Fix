"""Tests for SSRState.update_from_records — the decoder-neutral SSR
ingestion path used to feed Galileo HAS corrections (decoded off the X20
E6 signal) into the same filter that runs on BKG RTCM SSR.

cssrlib-free: exercises the SSRState side only (the cssrlib->records
adapter lives in tools/has_ssr_adapter.py).
"""
import unittest

from ssr_corrections import SSRState


class UpdateFromRecordsTest(unittest.TestCase):

    def _records(self):
        return [
            {"prn": "G01", "iode": 42, "orbit": (0.21, 0.25, -0.32),
             "clock": 0.010, "code_bias": {"C1C": 1.04, "C2L": 2.16}},
            {"prn": "E03", "iode": 7, "orbit": (0.13, -0.31, 0.77),
             "clock": -1.225, "code_bias": {"C1C": 0.5, "C5Q": -0.8}},
            {"prn": "G05", "clock": 0.83},          # clock-only
        ]

    def test_counts_and_storage(self):
        ssr = SSRState()
        counts = ssr.update_from_records(self._records(), epoch_s=1217.0)
        self.assertEqual(counts, {"orbit": 2, "clock": 3, "code_bias": 4})
        self.assertEqual(ssr.n_orbit, 2)
        self.assertEqual(ssr.n_clock, 3)

    def test_orbit_clock_readback_units_and_prn(self):
        ssr = SSRState()
        ssr.update_from_records(self._records(), epoch_s=1217.0)
        oc = ssr.get_orbit("G01")
        self.assertIsNotNone(oc)
        self.assertEqual(oc.iod, 42)
        self.assertAlmostEqual(oc.radial, 0.21)
        self.assertAlmostEqual(oc.along, 0.25)
        self.assertAlmostEqual(oc.cross, -0.32)
        self.assertAlmostEqual(ssr.get_clock("E03").c0, -1.225)

    def test_code_bias_keyed_by_rinex_observable(self):
        ssr = SSRState()
        ssr.update_from_records(self._records(), epoch_s=1217.0)
        self.assertAlmostEqual(ssr.get_code_bias("G01", "C1C"), 1.04)
        self.assertAlmostEqual(ssr.get_code_bias("G01", "C2L"), 2.16)
        self.assertAlmostEqual(ssr.get_code_bias("E03", "C5Q"), -0.8)
        self.assertIsNone(ssr.get_code_bias("G01", "C5Q"))

    def test_clock_only_record_has_no_orbit(self):
        ssr = SSRState()
        ssr.update_from_records(self._records(), epoch_s=1217.0)
        self.assertIsNone(ssr.get_orbit("G05"))
        self.assertIsNotNone(ssr.get_clock("G05"))

    def test_freshness_stamped(self):
        ssr = SSRState()
        ssr.update_from_records(self._records(), epoch_s=1217.0, rx_mono=123.0)
        self.assertEqual(ssr.last_orbit_update_mono, 123.0)
        self.assertEqual(ssr.last_clock_update_mono, 123.0)

    def test_empty_records_no_writes(self):
        ssr = SSRState()
        counts = ssr.update_from_records([], epoch_s=0.0)
        self.assertEqual(counts, {"orbit": 0, "clock": 0, "code_bias": 0})
        self.assertEqual(ssr.n_orbit, 0)


class LoadSsrRecordsFileTest(unittest.TestCase):
    """The external records-file loader (realtime_ppp.load_ssr_records) +
    round-trip through update_from_records — the --ssr-records-file path
    used to feed Galileo HAS into the live engine."""

    def _write(self, obj):
        import json
        import tempfile
        f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        json.dump(obj, f)
        f.close()
        return f.name

    def test_load_and_apply_roundtrip(self):
        from realtime_ppp import load_ssr_records
        path = self._write({
            "epoch_s": 1217.0,
            "records": [{"prn": "E03", "iode": 53,
                         "orbit": [0.235, -0.352, -0.176], "clock": -0.168,
                         "code_bias": {"C1C": -0.92, "C5Q": -1.64}}]})
        epoch_s, recs = load_ssr_records(path)
        self.assertEqual(epoch_s, 1217.0)
        ssr = SSRState()
        counts = ssr.update_from_records(recs, epoch_s)
        self.assertEqual(counts, {"orbit": 1, "clock": 1, "code_bias": 2})
        self.assertAlmostEqual(ssr.get_clock("E03").c0, -0.168)
        self.assertAlmostEqual(ssr.get_code_bias("E03", "C5Q"), -1.64)

    def test_missing_or_empty_file_returns_none(self):
        from realtime_ppp import load_ssr_records
        self.assertEqual(load_ssr_records("/nonexistent/xyz.json"), (None, None))
        self.assertEqual(load_ssr_records(self._write({"records": []})),
                         (None, None))

    def test_garbled_file_returns_none(self):
        from realtime_ppp import load_ssr_records
        import tempfile
        f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        f.write('{"records": [half written')   # truncated JSON
        f.close()
        self.assertEqual(load_ssr_records(f.name), (None, None))


if __name__ == "__main__":
    unittest.main()
