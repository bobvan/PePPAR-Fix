"""Unit tests for METAR fetch + retry + cache (peppar_fix.metar)."""
from __future__ import annotations

import json
import os
import tempfile
import unittest
from datetime import datetime, timezone, timedelta
from unittest import mock

import peppar_fix.metar as metar


_GOOD_API_BODY = json.dumps([{
    'reportTime': '2026-05-12T18:00:00.000Z',
    'temp': 22.0,
    'dewp': 12.0,
    'altim': 1013.0,
    'slp': 1013.5,
    'rawOb': 'KDPA 121800Z ...',
    'icaoId': 'KDPA',
}]).encode()


class _FakeUrlOpen:
    """Replace urllib.request.urlopen for tests.

    Each call pops one response from ``responses``; each is either an
    Exception instance (raised) or bytes (returned as body).
    """

    def __init__(self, responses):
        self.responses = list(responses)
        self.call_count = 0

    def __call__(self, *args, **kwargs):
        self.call_count += 1
        if not self.responses:
            raise RuntimeError("FakeUrlOpen exhausted")
        item = self.responses.pop(0)
        if isinstance(item, Exception):
            raise item
        return _FakeResp(item)


class _FakeResp:
    def __init__(self, body):
        self.body = body

    def __enter__(self):
        return self

    def __exit__(self, *a):
        pass

    def read(self):
        return self.body


def _tmpdir_cache():
    """Return a TemporaryDirectory + a patcher that points the cache at it."""
    td = tempfile.TemporaryDirectory()
    return td, mock.patch.dict(os.environ, {'XDG_CACHE_HOME': td.name})


class FetchLatestMetarTest(unittest.TestCase):

    def setUp(self):
        self.td, self.patch_env = _tmpdir_cache()
        self.patch_env.start()
        self.addCleanup(self.patch_env.stop)
        self.addCleanup(self.td.cleanup)
        # No-op sleep to keep tests fast.
        self.sleeps: list[float] = []
        self._sleep = self.sleeps.append

    def test_success_first_try_no_retry(self):
        fake = _FakeUrlOpen([_GOOD_API_BODY])
        with mock.patch.object(metar.urllib.request, 'urlopen', fake):
            rec = metar.fetch_latest_metar(
                'KDPA', _sleep=self._sleep)
        self.assertEqual(fake.call_count, 1)
        self.assertEqual(rec['station'], 'KDPA')
        self.assertEqual(rec['temp_C'], 22.0)
        self.assertEqual(self.sleeps, [])

    def test_success_after_retry_writes_cache(self):
        # 2 timeouts then success on 3rd attempt
        timeout = TimeoutError("connection timed out")
        fake = _FakeUrlOpen([timeout, timeout, _GOOD_API_BODY])
        with mock.patch.object(metar.urllib.request, 'urlopen', fake):
            rec = metar.fetch_latest_metar(
                'KDPA', _sleep=self._sleep,
                retry_delays_s=(0.1, 0.2, 0.5))
        self.assertEqual(fake.call_count, 3)
        self.assertEqual(rec['temp_C'], 22.0)
        # Sleeps fired between attempts 1→2 and 2→3 (not after the success)
        self.assertEqual(self.sleeps, [0.1, 0.2])
        # Cache file written
        cache_path = metar._cache_path('KDPA')
        self.assertTrue(cache_path.exists())
        cached = json.loads(cache_path.read_text())
        self.assertEqual(cached['temp_C'], 22.0)

    def test_all_retries_fail_falls_back_to_cache(self):
        # Pre-populate cache with a recent record
        recent_ts = datetime.now(timezone.utc) - timedelta(minutes=30)
        cache_rec = {
            'report_ts': recent_ts.isoformat(),
            'temp_C': 18.5, 'dewp_C': 9.0, 'altim_hPa': 1012.0,
            'slp_hPa': None, 'raw_metar': '', 'station': 'KDPA',
        }
        metar._cache_path('KDPA').write_text(json.dumps(cache_rec))

        # All 4 attempts fail
        timeout = TimeoutError("connection timed out")
        fake = _FakeUrlOpen([timeout] * 4)
        with mock.patch.object(metar.urllib.request, 'urlopen', fake):
            rec = metar.fetch_latest_metar(
                'KDPA', _sleep=self._sleep,
                retry_delays_s=(0.1, 0.2, 0.5))
        self.assertEqual(fake.call_count, 4)
        self.assertEqual(rec['temp_C'], 18.5)  # cached value, not 22.0

    def test_stale_cache_does_not_satisfy_failure(self):
        # Cache 3h old — older than 2h gate
        old_ts = datetime.now(timezone.utc) - timedelta(hours=3)
        cache_rec = {
            'report_ts': old_ts.isoformat(),
            'temp_C': 18.5, 'dewp_C': 9.0, 'altim_hPa': 1012.0,
            'slp_hPa': None, 'raw_metar': '', 'station': 'KDPA',
        }
        metar._cache_path('KDPA').write_text(json.dumps(cache_rec))

        timeout = TimeoutError("connection timed out")
        fake = _FakeUrlOpen([timeout] * 4)
        with mock.patch.object(metar.urllib.request, 'urlopen', fake):
            with self.assertRaises(metar.MetarReadError):
                metar.fetch_latest_metar(
                    'KDPA', _sleep=self._sleep,
                    retry_delays_s=(0.1, 0.2, 0.5))

    def test_no_cache_no_retry_left_raises(self):
        timeout = TimeoutError("connection timed out")
        fake = _FakeUrlOpen([timeout] * 4)
        with mock.patch.object(metar.urllib.request, 'urlopen', fake):
            with self.assertRaises(metar.MetarReadError) as ctx:
                metar.fetch_latest_metar(
                    'KDPA', _sleep=self._sleep,
                    retry_delays_s=(0.1, 0.2, 0.5))
        self.assertIn("4 attempts", str(ctx.exception))

    def test_use_cache_false_skips_cache_path(self):
        # Pre-populate cache — should NOT be used when use_cache=False
        recent_ts = datetime.now(timezone.utc) - timedelta(minutes=30)
        cache_rec = {
            'report_ts': recent_ts.isoformat(),
            'temp_C': 18.5, 'dewp_C': 9.0, 'altim_hPa': 1012.0,
            'slp_hPa': None, 'raw_metar': '', 'station': 'KDPA',
        }
        metar._cache_path('KDPA').write_text(json.dumps(cache_rec))
        timeout = TimeoutError("connection timed out")
        fake = _FakeUrlOpen([timeout] * 4)
        with mock.patch.object(metar.urllib.request, 'urlopen', fake):
            with self.assertRaises(metar.MetarReadError):
                metar.fetch_latest_metar(
                    'KDPA', use_cache=False,
                    _sleep=self._sleep,
                    retry_delays_s=(0.1, 0.2, 0.5))

    def test_url_error_treated_as_retryable(self):
        import urllib.error
        err = urllib.error.URLError("connection refused")
        fake = _FakeUrlOpen([err, _GOOD_API_BODY])
        with mock.patch.object(metar.urllib.request, 'urlopen', fake):
            rec = metar.fetch_latest_metar(
                'KDPA', _sleep=self._sleep,
                retry_delays_s=(0.1, 0.2, 0.5))
        self.assertEqual(fake.call_count, 2)
        self.assertEqual(rec['temp_C'], 22.0)

    def test_oserror_treated_as_retryable(self):
        err = OSError(101, "Network is unreachable")
        fake = _FakeUrlOpen([err, _GOOD_API_BODY])
        with mock.patch.object(metar.urllib.request, 'urlopen', fake):
            rec = metar.fetch_latest_metar(
                'KDPA', _sleep=self._sleep,
                retry_delays_s=(0.1, 0.2, 0.5))
        self.assertEqual(fake.call_count, 2)
        self.assertEqual(rec['temp_C'], 22.0)


class CacheRoundtripTest(unittest.TestCase):

    def setUp(self):
        self.td, self.patch_env = _tmpdir_cache()
        self.patch_env.start()
        self.addCleanup(self.patch_env.stop)
        self.addCleanup(self.td.cleanup)

    def test_write_then_read(self):
        # Use a fresh report_ts so the 2h staleness gate doesn't
        # reject the record after the test's hardcoded date slips
        # behind wall-clock.
        recent_ts = datetime.now(timezone.utc) - timedelta(minutes=5)
        rec = {
            'report_ts': recent_ts,
            'temp_C': 22.0, 'dewp_C': 12.0, 'altim_hPa': 1013.0,
            'slp_hPa': 1013.5, 'raw_metar': 'KDPA latest',
            'station': 'KDPA',
        }
        metar._write_cache('KDPA', rec)
        got = metar._read_cache('KDPA')
        self.assertIsNotNone(got)
        self.assertEqual(got['temp_C'], 22.0)
        self.assertEqual(got['station'], 'KDPA')
        self.assertEqual(got['report_ts'], rec['report_ts'])

    def test_read_missing_returns_none(self):
        self.assertIsNone(metar._read_cache('NONEXIST'))

    def test_read_corrupt_cache_returns_none(self):
        path = metar._cache_path('KDPA')
        path.write_text("{not valid json")
        self.assertIsNone(metar._read_cache('KDPA'))

    def test_read_stale_returns_none(self):
        old_ts = datetime.now(timezone.utc) - timedelta(hours=3)
        rec = {
            'report_ts': old_ts,
            'temp_C': 18.5, 'dewp_C': 9.0, 'altim_hPa': 1012.0,
            'slp_hPa': None, 'raw_metar': '', 'station': 'KDPA',
        }
        metar._write_cache('KDPA', rec)
        # Default 2h ttl — 3h-old record stale.
        self.assertIsNone(metar._read_cache('KDPA'))

    def test_write_atomic_via_rename(self):
        # The cache write goes through a tmpfile + rename so a reader
        # never sees a partial file.  Implementation detail — verify
        # only that the post-write file content is complete.
        rec = {
            'report_ts': datetime.now(timezone.utc),
            'temp_C': 1.0, 'dewp_C': 1.0, 'altim_hPa': 1000.0,
            'slp_hPa': None, 'raw_metar': '', 'station': 'KORD',
        }
        metar._write_cache('KORD', rec)
        text = metar._cache_path('KORD').read_text()
        # Round-trip must succeed
        parsed = json.loads(text)
        self.assertEqual(parsed['station'], 'KORD')


if __name__ == "__main__":
    unittest.main()
