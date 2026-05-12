"""Tests for the NAV-SIG L0 admission gate (slipDetectUnified Phase B)."""

from __future__ import annotations

import unittest
from dataclasses import dataclass

from peppar_fix.nav_sig_gate import (
    GateResult,
    NavSigGateCounters,
    REASON_ADMIT_BOTH_USED,
    REASON_ADMIT_NAV_SIG_STALE,
    REASON_ADMIT_GATE_OFF,
    REASON_DROP_NAV_SIG_F1_EXCL,
    REASON_DROP_NAV_SIG_F2_EXCL,
    REASON_DROP_NAV_SIG_UNHEALTHY,
    decide,
)


# Lightweight stand-in for the real SigStatus.  The gate consumes only
# .pr_used, .health, and .host_mono — duck-typing keeps tests
# decoupled from the realtime_ppp parser shape.
@dataclass
class _FakeSig:
    pr_used: bool = True
    health:  int  = 1     # 0=unknown, 1=healthy, 2=unhealthy
    host_mono: float = 100.0


class _FakeStore:
    """Map (sv, sig_name) → SigStatus or None."""

    def __init__(self, entries=None):
        self._entries = dict(entries or {})

    def get(self, sv, sig_name):
        return self._entries.get((sv, sig_name))


NOW_MONO = 100.0
MAX_AGE = 2.0


class _GateDisabled(unittest.TestCase):
    """Gate-off mode is a no-op: always admit, never consult."""

    def test_gate_off_admits_even_with_excluding_store(self):
        store = _FakeStore({('G07', 'L1CA'): _FakeSig(pr_used=False)})
        r = decide(store, 'G07', 'L1CA', 'L5Q',
                   gate_enabled=False, max_age_s=MAX_AGE, now_mono=NOW_MONO)
        self.assertTrue(r.admit)
        self.assertEqual(r.reason, REASON_ADMIT_GATE_OFF)
        self.assertFalse(r.receiver_consulted)


class _DegradedModes(unittest.TestCase):
    """When NAV-SIG is unavailable, fall through to admit."""

    def test_none_store_admits(self):
        r = decide(None, 'G07', 'L1CA', 'L5Q',
                   gate_enabled=True, max_age_s=MAX_AGE, now_mono=NOW_MONO)
        self.assertTrue(r.admit)
        self.assertEqual(r.reason, REASON_ADMIT_NAV_SIG_STALE)
        self.assertFalse(r.receiver_consulted)

    def test_empty_store_admits(self):
        r = decide(_FakeStore(), 'G07', 'L1CA', 'L5Q',
                   gate_enabled=True, max_age_s=MAX_AGE, now_mono=NOW_MONO)
        self.assertTrue(r.admit)
        self.assertEqual(r.reason, REASON_ADMIT_NAV_SIG_STALE)

    def test_stale_data_admits(self):
        # SigStatus host_mono is 5s old; max_age 2s → treated as missing
        old = _FakeSig(pr_used=False, host_mono=NOW_MONO - 5.0)
        store = _FakeStore({('G07', 'L1CA'): old, ('G07', 'L5Q'): old})
        r = decide(store, 'G07', 'L1CA', 'L5Q',
                   gate_enabled=True, max_age_s=MAX_AGE, now_mono=NOW_MONO)
        self.assertTrue(r.admit)
        self.assertEqual(r.reason, REASON_ADMIT_NAV_SIG_STALE)


class _GateActive(unittest.TestCase):
    """Gate-on mode honors receiver verdict."""

    def test_both_bands_used_admits(self):
        store = _FakeStore({
            ('G07', 'L1CA'): _FakeSig(pr_used=True),
            ('G07', 'L5Q'):  _FakeSig(pr_used=True),
        })
        r = decide(store, 'G07', 'L1CA', 'L5Q',
                   gate_enabled=True, max_age_s=MAX_AGE, now_mono=NOW_MONO)
        self.assertTrue(r.admit)
        self.assertEqual(r.reason, REASON_ADMIT_BOTH_USED)
        self.assertTrue(r.receiver_consulted)
        self.assertFalse(r.receiver_excluded)

    def test_f1_excluded_drops(self):
        store = _FakeStore({
            ('G07', 'L1CA'): _FakeSig(pr_used=False),
            ('G07', 'L5Q'):  _FakeSig(pr_used=True),
        })
        r = decide(store, 'G07', 'L1CA', 'L5Q',
                   gate_enabled=True, max_age_s=MAX_AGE, now_mono=NOW_MONO)
        self.assertFalse(r.admit)
        self.assertEqual(r.reason, REASON_DROP_NAV_SIG_F1_EXCL)
        self.assertTrue(r.receiver_excluded)

    def test_f2_excluded_drops(self):
        store = _FakeStore({
            ('G07', 'L1CA'): _FakeSig(pr_used=True),
            ('G07', 'L5Q'):  _FakeSig(pr_used=False),
        })
        r = decide(store, 'G07', 'L1CA', 'L5Q',
                   gate_enabled=True, max_age_s=MAX_AGE, now_mono=NOW_MONO)
        self.assertFalse(r.admit)
        self.assertEqual(r.reason, REASON_DROP_NAV_SIG_F2_EXCL)

    def test_both_excluded_drops_with_f1_reason(self):
        # f1 check fires first per implementation order.
        store = _FakeStore({
            ('G07', 'L1CA'): _FakeSig(pr_used=False),
            ('G07', 'L5Q'):  _FakeSig(pr_used=False),
        })
        r = decide(store, 'G07', 'L1CA', 'L5Q',
                   gate_enabled=True, max_age_s=MAX_AGE, now_mono=NOW_MONO)
        self.assertFalse(r.admit)
        self.assertEqual(r.reason, REASON_DROP_NAV_SIG_F1_EXCL)

    def test_unhealthy_overrides_pr_used(self):
        store = _FakeStore({
            ('G07', 'L1CA'): _FakeSig(pr_used=True, health=2),
            ('G07', 'L5Q'):  _FakeSig(pr_used=True),
        })
        r = decide(store, 'G07', 'L1CA', 'L5Q',
                   gate_enabled=True, max_age_s=MAX_AGE, now_mono=NOW_MONO)
        self.assertFalse(r.admit)
        self.assertEqual(r.reason, REASON_DROP_NAV_SIG_UNHEALTHY)

    def test_one_band_stale_other_used_admits(self):
        # f1 missing entirely, f2 used → admit (the cohort F10T case
        # where SOME signals report and others don't)
        store = _FakeStore({
            ('G07', 'L5Q'): _FakeSig(pr_used=True),
        })
        r = decide(store, 'G07', 'L1CA', 'L5Q',
                   gate_enabled=True, max_age_s=MAX_AGE, now_mono=NOW_MONO)
        self.assertTrue(r.admit)
        self.assertEqual(r.reason, REASON_ADMIT_BOTH_USED)

    def test_one_band_stale_other_excluded_drops(self):
        # f1 stale, f2 actively excluded — drop on f2 evidence
        store = _FakeStore({
            ('G07', 'L5Q'): _FakeSig(pr_used=False),
        })
        r = decide(store, 'G07', 'L1CA', 'L5Q',
                   gate_enabled=True, max_age_s=MAX_AGE, now_mono=NOW_MONO)
        self.assertFalse(r.admit)
        self.assertEqual(r.reason, REASON_DROP_NAV_SIG_F2_EXCL)


class _NoneSignalNames(unittest.TestCase):
    """Tolerate missing f1_sig_name / f2_sig_name without crashing."""

    def test_both_none_falls_through_to_stale(self):
        store = _FakeStore({('G07', 'L1CA'): _FakeSig(pr_used=False)})
        r = decide(store, 'G07', None, None,
                   gate_enabled=True, max_age_s=MAX_AGE, now_mono=NOW_MONO)
        self.assertTrue(r.admit)
        self.assertEqual(r.reason, REASON_ADMIT_NAV_SIG_STALE)


class _Counters(unittest.TestCase):

    def test_records_each_reason_category(self):
        c = NavSigGateCounters()
        c.record(GateResult(True,  REASON_ADMIT_BOTH_USED,     False, True))
        c.record(GateResult(True,  REASON_ADMIT_BOTH_USED,     False, True))
        c.record(GateResult(False, REASON_DROP_NAV_SIG_F1_EXCL, True, True))
        c.record(GateResult(True,  REASON_ADMIT_NAV_SIG_STALE, False, False))
        d = c.as_dict()
        self.assertEqual(d['admit_both_used'],  2)
        self.assertEqual(d['drop_f1_excluded'], 1)
        self.assertEqual(d['admit_stale'],      1)

    def test_reset_epoch_clears_counts(self):
        c = NavSigGateCounters()
        c.record(GateResult(True, REASON_ADMIT_BOTH_USED, False, True))
        c.reset_epoch()
        d = c.as_dict()
        self.assertEqual(d['admit_both_used'], 0)


if __name__ == '__main__':
    unittest.main()
