"""Unit tests for tools/analysis/nav_sig_disagree_analysis.py."""
from __future__ import annotations

import tempfile
import unittest
from datetime import datetime
from pathlib import Path

# Add tools/analysis to path
import sys
sys.path.insert(0, str(Path(__file__).parent))

from nav_sig_disagree_analysis import (
    parse_log, aggregate_disagrees, correlate_cat_rejects,
    DisagreeEvent, CatRejectEvent,
    _readiness_verdict, _parse_ts, _parse_kv, _host_from_path,
)


def _write_log(text: str) -> Path:
    """Write a temp log file containing engine-style lines + return Path."""
    fd, path = tempfile.mkstemp(suffix='.log', prefix='test_navsig_')
    p = Path(path)
    p.write_text(text)
    return p


class TimestampParseTest(unittest.TestCase):

    def test_space_separator(self):
        self.assertEqual(
            _parse_ts("2026-05-12 18:30:15,123 INFO foo"),
            datetime(2026, 5, 12, 18, 30, 15))

    def test_t_separator(self):
        self.assertEqual(
            _parse_ts("2026-05-12T18:30:15.123 INFO foo"),
            datetime(2026, 5, 12, 18, 30, 15))

    def test_no_subsecond(self):
        self.assertEqual(
            _parse_ts("2026-05-12 18:30:15 INFO foo"),
            datetime(2026, 5, 12, 18, 30, 15))

    def test_bad_line_returns_none(self):
        self.assertIsNone(_parse_ts("garbage"))
        self.assertIsNone(_parse_ts(""))

    def test_kv_parse(self):
        self.assertEqual(
            _parse_kv("lock_ms=64500 cno_eng=39 health=1"),
            {'lock_ms': '64500', 'cno_eng': '39', 'health': '1'})


class HostFromPathTest(unittest.TestCase):

    def test_match_known(self):
        for host in ('madhat', 'timehat', 'piface', 'clkpoc3'):
            self.assertEqual(
                _host_from_path(Path(f'/data/day0512overnight-{host}.log')),
                host)

    def test_unknown_falls_back_to_stem(self):
        self.assertEqual(
            _host_from_path(Path('/data/something-else.log')),
            'something-else')


class ParseLogTest(unittest.TestCase):

    def test_parses_single_disagree(self):
        line = (
            "2026-05-12 18:30:15,123 INFO [NAV-SIG_DISAGREE ep=42 sv=E19 "
            "sig=GAL-E5aQ] receiver=excluded our=admit lock_ms=64500 "
            "cno_rcv=39 health=1\n"
        )
        p = _write_log(line)
        try:
            d, c = parse_log(p)
            self.assertEqual(len(d), 1)
            self.assertEqual(len(c), 0)
            ev = d[0]
            self.assertEqual(ev.sv, 'E19')
            self.assertEqual(ev.sig, 'GAL-E5aQ')
            self.assertEqual(ev.epoch, 42)
            self.assertEqual(ev.direction, 'receiver=excluded our=admit')
            self.assertTrue(ev.receiver_excludes)
            self.assertEqual(ev.fields['lock_ms'], '64500')
            self.assertEqual(ev.fields['cno_rcv'], '39')
        finally:
            p.unlink()

    def test_parses_both_directions(self):
        text = (
            "2026-05-12 18:30:15 INFO [NAV-SIG_DISAGREE ep=1 sv=G07 sig=GPS-L1CA] "
            "receiver=excluded our=admit\n"
            "2026-05-12 18:30:16 INFO [NAV-SIG_DISAGREE ep=2 sv=C42 sig=BDS-B2aI] "
            "receiver=admitted our=exclude\n"
        )
        p = _write_log(text)
        try:
            d, _ = parse_log(p)
            self.assertEqual(len(d), 2)
            self.assertTrue(d[0].receiver_excludes)
            self.assertFalse(d[1].receiver_excludes)
        finally:
            p.unlink()

    def test_parses_cat_reject_with_chips(self):
        line = (
            "2026-05-12 18:30:20,456 ERROR [CATASTROPHIC_REJECT] median |PR|="
            "6295637.0m (21482.10 L1 chips, 21.001 ms) > 20×baseline 5.0m"
            " sustained 30 epochs\n"
        )
        p = _write_log(line)
        try:
            _, c = parse_log(p)
            self.assertEqual(len(c), 1)
            self.assertAlmostEqual(c[0].median_m, 6295637.0)
            self.assertAlmostEqual(c[0].chips, 21482.10)
            self.assertAlmostEqual(c[0].ms, 21.001)
        finally:
            p.unlink()

    def test_cat_reject_pulls_nearby_svs_from_slip_ringbuffer(self):
        text = (
            "2026-05-12 18:30:10 INFO [SLIP] sv=G03 reasons=mw_jump\n"
            "2026-05-12 18:30:12 INFO [SLIP] sv=E19 reasons=gf_step\n"
            "2026-05-12 18:30:20,456 ERROR [CATASTROPHIC_REJECT] "
            "median |PR|=789.0m > 20×baseline 5.0m sustained 30 epochs\n"
        )
        p = _write_log(text)
        try:
            _, c = parse_log(p)
            self.assertEqual(len(c), 1)
            # Both slip SVs are within 30 s of the cat-reject
            self.assertIn('G03', c[0].nearby_svs)
            self.assertIn('E19', c[0].nearby_svs)
        finally:
            p.unlink()

    def test_old_slips_outside_window_not_attached(self):
        text = (
            "2026-05-12 17:30:10 INFO [SLIP] sv=G03 reasons=mw_jump\n"
            # 1 hour gap before cat-reject — out of 30 s window
            "2026-05-12 18:30:20,456 ERROR [CATASTROPHIC_REJECT] "
            "median |PR|=789.0m > 20×baseline 5.0m\n"
        )
        p = _write_log(text)
        try:
            _, c = parse_log(p)
            self.assertEqual(c[0].nearby_svs, [])
        finally:
            p.unlink()

    def test_host_inferred_from_filename(self):
        text = "2026-05-12 18:30:15 INFO [NAV-SIG_DISAGREE ep=1 sv=G07 sig=GPS-L1CA] receiver=excluded our=admit\n"
        # Write with madhat-shaped name
        p = Path(tempfile.gettempdir()) / 'day0512overnight-madhat.log'
        p.write_text(text)
        try:
            d, _ = parse_log(p)
            self.assertEqual(d[0].host, 'madhat')
        finally:
            p.unlink()

    def test_explicit_host_overrides_filename(self):
        text = "2026-05-12 18:30:15 INFO [NAV-SIG_DISAGREE ep=1 sv=G07 sig=GPS-L1CA] receiver=excluded our=admit\n"
        p = _write_log(text)
        try:
            d, _ = parse_log(p, host='custom-host')
            self.assertEqual(d[0].host, 'custom-host')
        finally:
            p.unlink()


class AggregationTest(unittest.TestCase):

    def _ev(self, sv, sig, direction='receiver=excluded our=admit', host='madhat'):
        return DisagreeEvent(
            ts=datetime(2026, 5, 12, 18, 30, 15),
            host=host, epoch=1, sv=sv, sig=sig, direction=direction)

    def test_aggregate_counts(self):
        events = [
            self._ev('E19', 'GAL-E5aQ'),
            self._ev('E19', 'GAL-E5aQ'),
            self._ev('G07', 'GPS-L5Q'),
            self._ev('C42', 'BDS-B2aI',
                     direction='receiver=admitted our=exclude'),
        ]
        s = aggregate_disagrees(events)
        self.assertEqual(s['total'], 4)
        self.assertEqual(s['by_direction']['receiver=excluded our=admit'], 3)
        self.assertEqual(s['by_direction']['receiver=admitted our=exclude'], 1)
        self.assertEqual(
            s['by_sv_sig']['E19|GAL-E5aQ']['receiver=excluded our=admit'], 2)

    def test_aggregate_empty(self):
        s = aggregate_disagrees([])
        self.assertEqual(s['total'], 0)
        self.assertEqual(s['by_direction'], {})


class CorrelateCatRejectsTest(unittest.TestCase):

    def _disagree(self, ts_sec, sv, host='madhat',
                  direction='receiver=excluded our=admit'):
        return DisagreeEvent(
            ts=datetime(2026, 5, 12, 18, 30, ts_sec),
            host=host, epoch=1, sv=sv, sig='GAL-E5aQ',
            direction=direction)

    def _catreject(self, ts_sec, host='madhat', nearby_svs=()):
        return CatRejectEvent(
            ts=datetime(2026, 5, 12, 18, 30, ts_sec),
            host=host, median_m=1000.0, chips=3.5, ms=0.003,
            nearby_svs=list(nearby_svs))

    def test_hit_when_receiver_excluded_in_window(self):
        # Receiver excluded E19 at :10, cat-reject at :30 with E19 nearby
        d = [self._disagree(10, 'E19')]
        c = [self._catreject(30, nearby_svs=['E19'])]
        result = correlate_cat_rejects(c, d, window_s=60.0)
        self.assertEqual(result['n_cat_rejects'], 1)
        self.assertEqual(result['n_hit'], 1)
        self.assertEqual(result['hit_rate'], 1.0)
        self.assertEqual(result['per_reject'][0]['matching_svs'], ['E19'])

    def test_miss_when_disagree_outside_window(self):
        # Receiver excluded E19 a full 5 min before cat-reject
        d = [self._disagree(0, 'E19')]  # 18:30:00
        d[0] = DisagreeEvent(
            ts=datetime(2026, 5, 12, 18, 25, 0),
            host='madhat', epoch=1, sv='E19', sig='GAL-E5aQ',
            direction='receiver=excluded our=admit')
        c = [self._catreject(30, nearby_svs=['E19'])]  # 18:30:30
        result = correlate_cat_rejects(c, d, window_s=60.0)
        self.assertEqual(result['n_hit'], 0)

    def test_miss_when_disagree_sv_not_in_nearby(self):
        # Receiver excluded G07; cat-reject's nearby SVs is [E19]
        d = [self._disagree(10, 'G07')]
        c = [self._catreject(30, nearby_svs=['E19'])]
        result = correlate_cat_rejects(c, d, window_s=60.0)
        self.assertEqual(result['n_hit'], 0)

    def test_hit_when_no_nearby_svs_recorded(self):
        # Cat-reject with empty nearby_svs falls back to "any disagreement
        # in the window" — graceful behavior when slip-context lookup
        # didn't find anything.
        d = [self._disagree(10, 'G07')]
        c = [self._catreject(30, nearby_svs=[])]
        result = correlate_cat_rejects(c, d, window_s=60.0)
        self.assertEqual(result['n_hit'], 1)

    def test_only_receiver_excluded_direction_counts(self):
        # receiver=admitted our=exclude in the window — should NOT
        # count as a Phase B3 hit (Phase B excludes based on
        # prUsed=0, not on our-side rejection).
        d = [self._disagree(10, 'E19',
                            direction='receiver=admitted our=exclude')]
        c = [self._catreject(30, nearby_svs=['E19'])]
        result = correlate_cat_rejects(c, d, window_s=60.0)
        self.assertEqual(result['n_hit'], 0)

    def test_cross_host_no_match(self):
        # Receiver excluded on madhat, cat-reject on timehat — different
        # host, no match.
        d = [self._disagree(10, 'E19', host='madhat')]
        c = [self._catreject(30, host='timehat', nearby_svs=['E19'])]
        result = correlate_cat_rejects(c, d, window_s=60.0)
        self.assertEqual(result['n_hit'], 0)

    def test_no_cat_rejects_returns_indeterminate(self):
        result = correlate_cat_rejects([], [], window_s=60.0)
        self.assertEqual(result['n_cat_rejects'], 0)
        self.assertIsNone(result['hit_rate'])


class ReadinessVerdictTest(unittest.TestCase):

    def test_no_cat_rejects(self):
        v = _readiness_verdict(None, 0)
        self.assertIn("no cat-rejects", v)

    def test_ready(self):
        v = _readiness_verdict(0.95, 20)
        self.assertIn("READY", v)
        self.assertIn("95%", v)

    def test_partial(self):
        v = _readiness_verdict(0.65, 20)
        self.assertIn("PARTIAL", v)

    def test_not_ready(self):
        v = _readiness_verdict(0.25, 20)
        self.assertIn("NOT READY", v)


class EndToEndIntegrationTest(unittest.TestCase):

    def test_full_pipeline(self):
        # Mock a realistic log: 3 receiver-excluded events; 2 cat-rejects
        # — one preceded by a matching SV's exclusion, one not.
        text = (
            "2026-05-12 18:30:00 INFO [NAV-SIG_DISAGREE ep=1 sv=E19 sig=GAL-E5aQ] "
            "receiver=excluded our=admit lock_ms=64500\n"
            "2026-05-12 18:30:15 INFO [SLIP] sv=E19 reasons=mw_jump\n"
            "2026-05-12 18:30:25 ERROR [CATASTROPHIC_REJECT] median |PR|=789.0m "
            "(2.69 L1 chips, 0.003 ms) > 20×baseline 5.0m\n"
            "2026-05-12 19:00:00 INFO [SLIP] sv=G03 reasons=mw_jump\n"
            "2026-05-12 19:00:20 ERROR [CATASTROPHIC_REJECT] median |PR|=520.0m "
            "(1.77 L1 chips, 0.002 ms) > 20×baseline 5.0m\n"
        )
        p = _write_log(text)
        try:
            d, c = parse_log(p)
            self.assertEqual(len(d), 1)  # 1 disagree
            self.assertEqual(len(c), 2)  # 2 cat-rejects
            # First cat-reject's nearby_svs includes E19 (slip at :15
            # within 30 s of :25)
            self.assertIn('E19', c[0].nearby_svs)
            # Second cat-reject has G03 (no E19), so should NOT match
            # the disagree on E19
            self.assertNotIn('E19', c[1].nearby_svs)

            corr = correlate_cat_rejects(c, d, window_s=60.0)
            self.assertEqual(corr['n_cat_rejects'], 2)
            self.assertEqual(corr['n_hit'], 1)
            self.assertEqual(corr['hit_rate'], 0.5)
        finally:
            p.unlink()


if __name__ == '__main__':
    unittest.main()
