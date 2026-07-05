"""Tests for the SBF engine obs-source seam (I-030423).

Exercises the seam that turns a Septentrio SBF stream into engine obs_queue
items: the WNc+TOW → GPS-time mapping, the reader loop over pysbf2 messages,
and the TCP source thread target (with a mocked socket serving the real
mosaic-T fixture bytes).  The decode itself is covered by test_sbf_obs.py.
"""
import io
import os
import queue
import socket
import sys
import threading
import unittest
from datetime import datetime, timezone
from types import SimpleNamespace
from unittest import mock

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from pysbf2 import SBFReader  # noqa: E402

from peppar_fix import sbf_obs_source as src  # noqa: E402

_FIXTURE = os.path.join(os.path.dirname(__file__), "testdata",
                        "mosaic_measepoch.sbf")


def _fixture_bytes():
    return open(_FIXTURE, "rb").read()


def _load_meas_epoch():
    for _raw, parsed in SBFReader(io.BytesIO(_fixture_bytes()), quitonerror=0):
        if parsed is not None and parsed.identity == "MeasEpoch":
            return parsed
    raise AssertionError("no MeasEpoch in fixture")


class SbfGpsTimeTest(unittest.TestCase):
    def test_epoch(self):
        self.assertEqual(src.sbf_gps_time(0, 0),
                         datetime(1980, 1, 6, tzinfo=timezone.utc))

    def test_week_and_tow(self):
        # one week + 54341.0 s past the GPS epoch
        t = src.sbf_gps_time(54341000, 1)
        self.assertEqual((t - src._GPS_EPOCH).total_seconds(),
                         604800.0 + 54341.0)

    def test_returns_aware_datetime(self):
        self.assertIsNotNone(src.sbf_gps_time(123, 2300).tzinfo)


class SbfObsReaderTest(unittest.TestCase):
    def setUp(self):
        self.msg = _load_meas_epoch()
        self.sig_lookup = __import__(
            "peppar_fix.rtcm_msm_obs", fromlist=["default_sig_lookup"]
        ).default_sig_lookup({"gps", "gal"})

    def test_queues_observation_event(self):
        q = queue.Queue()
        messages = [(b"", self.msg)]
        n = src.sbf_obs_reader(
            messages, q, threading.Event(), self.sig_lookup,
            systems={"gps", "gal"}, ssr=None,
            now_fn=lambda: datetime(2026, 7, 5, tzinfo=timezone.utc),
            mono_fn=lambda: 1234.5)
        self.assertEqual(n, 1)
        self.assertEqual(q.qsize(), 1)
        ev = q.get_nowait()
        self.assertGreaterEqual(len(ev.observations), 1)
        self.assertEqual(ev.recv_mono, 1234.5)
        # gps_time comes from the block's own WNc+TOW
        self.assertEqual(ev.gps_time, src.sbf_gps_time(self.msg.TOW, self.msg.WNc))

    def test_skips_non_measepoch(self):
        q = queue.Queue()
        junk = SimpleNamespace(identity="PVTGeodetic")
        n = src.sbf_obs_reader([(b"", junk)], q, threading.Event(),
                               self.sig_lookup, systems={"gps"})
        self.assertEqual(n, 0)
        self.assertTrue(q.empty())

    def test_stop_event_halts(self):
        q = queue.Queue()
        stop = threading.Event()
        stop.set()
        n = src.sbf_obs_reader([(b"", self.msg)], q, stop, self.sig_lookup,
                               systems={"gps", "gal"})
        self.assertEqual(n, 0)


class ParseHostportTest(unittest.TestCase):
    def test_host_and_port(self):
        self.assertEqual(src._parse_hostport("10.101.101.153:28784"),
                         ("10.101.101.153", 28784))

    def test_default_port(self):
        self.assertEqual(src._parse_hostport("host.local"),
                         ("host.local", 28784))


class ExtObsSourcePredicateTest(unittest.TestCase):
    """The run()/run_bootstrap shared predicate — SBF must count as an external
    obs source so the Phase-1 NAV2 seed-wait is skipped (delta #288 MEDIUM: the
    generalization must reach run_bootstrap, not just run())."""

    def setUp(self):
        import peppar_fix_engine as eng
        self.eng = eng

    def _args(self, **kw):
        base = dict(obs_ntrip_mount=None, obs_sbf_tcp=None)
        base.update(kw)
        return SimpleNamespace(**base)

    def test_sbf_is_ext_source(self):
        a = self._args(obs_sbf_tcp="10.101.101.153:28784")
        self.assertTrue(self.eng._is_sbf_source(a))
        self.assertFalse(self.eng._is_msm_source(a))
        self.assertTrue(self.eng._has_ext_obs_source(a))

    def test_msm_is_ext_source(self):
        a = self._args(obs_ntrip_mount="ALIC00AUS0")
        self.assertTrue(self.eng._is_msm_source(a))
        self.assertTrue(self.eng._has_ext_obs_source(a))

    def test_serial_is_not_ext_source(self):
        a = self._args()
        self.assertFalse(self.eng._has_ext_obs_source(a))


def _args():
    return SimpleNamespace(obs_sbf_tcp="10.101.101.153:28784",
                           systems="gps,gal", msm_l2_sig="GPS-L2W")


def _fake_sock():
    s = mock.Mock()
    s.makefile.return_value = io.BytesIO(_fixture_bytes())
    return s


class RunSbfTcpSourceTest(unittest.TestCase):
    def test_reads_from_mocked_socket(self):
        # a fake socket whose makefile() serves the real fixture bytes; the real
        # SBFReader parses them end-to-end.  stop_event is set on the 2nd connect
        # so the reconnect supervisor exits after one full session.
        stop = threading.Event()
        calls = []

        def fake_conn(addr, timeout=None):
            calls.append(addr)
            if len(calls) >= 2:
                stop.set()
            return _fake_sock()

        q = queue.Queue()
        with mock.patch.object(socket, "create_connection",
                               side_effect=fake_conn) as cc:
            src.run_sbf_tcp_source(_args(), q, stop, ssr=None,
                                   reconnect_delay=0.01)
        self.assertEqual(cc.call_args_list[0].args[0], ("10.101.101.153", 28784))
        # the fixture holds ≥1 MeasEpoch → ≥1 queued event
        self.assertGreaterEqual(q.qsize(), 1)

    def test_reconnects_after_stream_end(self):
        # the reader returns on stream EOF; the supervisor must reconnect and
        # keep queuing.  Stop once 2 sessions have each produced an event, so we
        # prove events flow ACROSS a reconnect boundary.
        stop = threading.Event()
        q = queue.Queue()
        calls = []

        def fake_conn(addr, timeout=None):
            calls.append(addr)
            if q.qsize() >= 2:            # 2 productive sessions already
                stop.set()
            return _fake_sock()

        with mock.patch.object(socket, "create_connection",
                               side_effect=fake_conn):
            src.run_sbf_tcp_source(_args(), q, stop, reconnect_delay=0.01)
        self.assertGreaterEqual(len(calls), 3)   # ≥2 reconnects
        self.assertEqual(q.qsize(), 2)           # 1 event per session, 2 sessions

    def test_connect_failure_retries_then_stops(self):
        # transient connect failures must NOT kill the feed — retry with backoff,
        # honoring stop_event (here set on the 3rd attempt) for prompt shutdown.
        stop = threading.Event()
        attempts = []

        def fail_conn(addr, timeout=None):
            attempts.append(addr)
            if len(attempts) >= 3:
                stop.set()
            raise OSError("connection refused")

        q = queue.Queue()
        with mock.patch.object(socket, "create_connection",
                               side_effect=fail_conn):
            # must not raise — a thread target logs and retries, then stops
            src.run_sbf_tcp_source(_args(), q, stop, reconnect_delay=0.01)
        self.assertGreaterEqual(len(attempts), 3)
        self.assertTrue(q.empty())

    def test_already_stopped_never_connects(self):
        # prompt shutdown: a stop_event set before entry means no connect at all.
        stop = threading.Event()
        stop.set()
        with mock.patch.object(socket, "create_connection") as cc:
            src.run_sbf_tcp_source(_args(), queue.Queue(), stop)
        cc.assert_not_called()

    def test_backoff_wait_is_interruptible(self):
        # a stop_event that fires DURING the post-failure backoff wait must break
        # the loop, not sleep out the full delay.
        stop = threading.Event()

        def fail_conn(addr, timeout=None):
            raise OSError("refused")

        q = queue.Queue()
        with mock.patch.object(socket, "create_connection",
                               side_effect=fail_conn):
            with mock.patch.object(stop, "wait", return_value=True) as w:
                src.run_sbf_tcp_source(_args(), q, stop, reconnect_delay=30.0)
        # wait() returned True (stopped) → loop broke after a single attempt,
        # without sleeping the 30 s delay.
        w.assert_called_once()


if __name__ == "__main__":
    unittest.main()
