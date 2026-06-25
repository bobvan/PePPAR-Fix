"""Tests for raw_capture — the pos_replay reference-capture bundle core
(docs/pos-replay-capture-manifest.md §6)."""
import os
import sys
import tomllib
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix import raw_capture as rc  # noqa: E402


class TestRoundTrip(unittest.TestCase):
    def test_binary_payloads_round_trip_exactly(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            with rc.RawCaptureBundle(d) as b:
                # binary-safe incl. NUL + high bytes (UBX/RTCM can be anything)
                b.record("ubx", b"\xb5\x62\x00\x01\xff", recv_mono=10.0)
                b.record("ubx", b"\x00\x00", recv_mono=11.5)
            recs = list(rc.read_stream(os.path.join(d, "raw", "ubx.cap")))
            self.assertEqual(recs, [(10.0, b"\xb5\x62\x00\x01\xff"),
                                    (11.5, b"\x00\x00")])

    def test_record_line_appends_newline(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            with rc.RawCaptureBundle(d) as b:
                b.record_line("ticc", "chA 0.123", recv_mono=1.0)
                b.record_line("ticc", b"chB 0.456\n", recv_mono=2.0)
            recs = list(rc.read_stream(os.path.join(d, "raw", "ticc.cap")))
            self.assertEqual(recs[0][1], b"chA 0.123\n")
            self.assertEqual(recs[1][1], b"chB 0.456\n")

    def test_counts(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            with rc.RawCaptureBundle(d) as b:
                b.record("ubx", b"a", 1.0)
                b.record("ubx", b"b", 2.0)
                b.record("ssr", b"c", 1.5)
                self.assertEqual(b.counts, {"ubx": 2, "ssr": 1})


class TestMergeOrder(unittest.TestCase):
    def test_merged_records_in_global_recv_mono_order(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            with rc.RawCaptureBundle(d) as b:
                b.record("ubx", b"u1", 1.0)
                b.record("ssr", b"s1", 1.5)
                b.record("ubx", b"u2", 3.0)
                b.record("ssr", b"s2", 2.0)
                b.record("ticc", b"t1\n", 2.5)
            merged = [(rm, stream, pl)
                      for rm, stream, pl in rc.merged_records(d)]
            self.assertEqual([rm for rm, _, _ in merged], [1.0, 1.5, 2.0, 2.5, 3.0])
            self.assertEqual([s for _, s, _ in merged],
                             ["ubx", "ssr", "ssr", "ticc", "ubx"])


class TestManifest(unittest.TestCase):
    def test_manifest_round_trips_through_tomllib(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            with rc.RawCaptureBundle(d) as b:
                b.record("ubx", b"x", 1.0)
                b.record("ssr", b"y", 1.0)
                path = b.write_manifest(
                    host="clkpoc3", started_iso="2026-06-25T00:00:00Z",
                    conventions={"arp_ecef_m": [-2730000.0, -4440000.0, 3975000.0],
                                 "antenna_height_m": 200.5, "ztd_station": "KORD",
                                 "mapping_function": "GMF"},
                    notes="first capture")
            with open(path, "rb") as f:
                m = tomllib.load(f)
            self.assertEqual(m["schema_version"], "1")
            self.assertEqual(m["capture"]["host"], "clkpoc3")
            self.assertEqual(m["capture"]["record_counts"], {"ubx": 1, "ssr": 1})
            self.assertEqual(m["conventions"]["ztd_station"], "KORD")
            self.assertEqual(m["conventions"]["arp_ecef_m"][0], -2730000.0)
            self.assertIn("git_rev", m["software"])


class TestRobustness(unittest.TestCase):
    def test_truncated_tail_stops_cleanly(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            with rc.RawCaptureBundle(d) as b:
                b.record("ubx", b"good", 1.0)
            cap = os.path.join(d, "raw", "ubx.cap")
            # append a header claiming 100 bytes but no payload (killed capture)
            with open(cap, "ab") as f:
                f.write(rc._REC_HDR.pack(2.0, 100))
                f.write(b"short")
            recs = list(rc.read_stream(cap))
            self.assertEqual(recs, [(1.0, b"good")])   # complete record only

    def test_empty_stream_file(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            os.makedirs(os.path.join(d, "raw"))
            cap = os.path.join(d, "raw", "ubx.cap")
            open(cap, "wb").close()
            self.assertEqual(list(rc.read_stream(cap)), [])


if __name__ == "__main__":
    unittest.main()
