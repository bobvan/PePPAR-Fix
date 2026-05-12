"""Tests for the NAV-SIG / NAV-CLOCK / NAV-TIMEGPS parsers + stores.

Background: f9tClockTelemetry-bravo + slipDetectUnified-main Phase A.

Nav2SignalStore consumes UBX-NAV-SIG and exposes:
  - polling: get(sv_label, sig_name) → SigStatus or None
  - bulk:    snapshot() → dict[(sv, sig)] → SigStatus
  - pub/sub: on_transition(cb) — fires when prUsed flips
  - meta:    epoch() — incremented per NAV-SIG message

Charlie's Phase A.5 disagreement instrumentation consumes
on_transition() callbacks; engine-side admission logic can poll
get() or snapshot().

These tests use plain Python objects in lieu of pyubx2 decoded
messages — the store reads via getattr() so anything with the
right attribute names works.
"""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from realtime_ppp import (   # noqa: E402
    Nav2SignalStore, NavClockStore, NavTimeGpsStore, SigStatus,
    _sv_label,
)


# pyubx2 stub: attributes named like 'gnssId_01', 'svId_01', etc.
class _FakeNavSig:
    def __init__(self, sigs):
        """sigs: list of dicts {gnssId, svId, sigId, sigFlags, cno,
        qualityInd, prRes}.  Numbered 01..NN as the wire format."""
        self.numSigs = len(sigs)
        for i, s in enumerate(sigs, start=1):
            i2 = f"{i:02d}"
            for k, v in s.items():
                setattr(self, f"{k}_{i2}", v)


# Lab signal-name mapping (subset for tests).
_SIGNAL_NAMES = {
    (0, 0): "GPS-L1CA",
    (0, 6): "GPS-L5I",
    (0, 7): "GPS-L5Q",
    (2, 0): "GAL-E1C",
    (2, 4): "GAL-E5aQ",
    (3, 0): "BDS-B1I",
}


def _sig(gnss_id, sv_id, sig_id, pr_used, cno=44, pr_res=3,
          health=1, cr_used=False, do_used=False, pr_smoothed=False):
    """Build a NAV-SIG entry dict with the right sigFlags bitfield."""
    flags = health & 0x03
    if pr_smoothed: flags |= 0x04
    if pr_used:     flags |= 0x08
    if cr_used:     flags |= 0x10
    if do_used:     flags |= 0x20
    return {'gnssId': gnss_id, 'svId': sv_id, 'sigId': sig_id,
            'sigFlags': flags, 'cno': cno, 'qualityInd': 7,
            'prRes': pr_res}


class _SvLabelTest(unittest.TestCase):
    def test_gps_gal_bds_glo(self):
        self.assertEqual(_sv_label(0, 7), 'G07')
        self.assertEqual(_sv_label(2, 19), 'E19')
        self.assertEqual(_sv_label(3, 42), 'C42')
        self.assertEqual(_sv_label(6, 12), 'R12')

    def test_unknown_system(self):
        self.assertEqual(_sv_label(99, 1), 'sys99_sv1')


class _SignalStoreBasic(unittest.TestCase):

    def setUp(self):
        self.store = Nav2SignalStore()
        self.store.set_signal_names(_SIGNAL_NAMES)

    def test_empty_store_returns_none(self):
        self.assertIsNone(self.store.get('G07', 'GPS-L1CA'))
        self.assertEqual(self.store.snapshot(), {})
        self.assertEqual(self.store.epoch(), 0)

    def test_single_update_populates(self):
        msg = _FakeNavSig([
            _sig(0, 7, 0, pr_used=True, cno=44, pr_res=3),
        ])
        self.store.update(msg)
        st = self.store.get('G07', 'GPS-L1CA')
        self.assertIsNotNone(st)
        self.assertTrue(st.pr_used)
        self.assertEqual(st.cno, 44)
        self.assertEqual(st.health, 1)
        self.assertEqual(self.store.epoch(), 1)

    def test_pr_res_scaling_heuristic(self):
        """prRes raw value > 100 assumed to be 0.1m units (per UBX
        spec); value ≤ 100 assumed already decoded."""
        # Raw 0.1m units: pr_res=4127 → 412.7m
        msg = _FakeNavSig([_sig(2, 19, 4, pr_used=False, pr_res=4127)])
        self.store.update(msg)
        st = self.store.get('E19', 'GAL-E5aQ')
        self.assertAlmostEqual(st.pr_res_m, 412.7, places=2)
        # Already-decoded: pr_res=3 → 3.0m
        msg2 = _FakeNavSig([_sig(0, 7, 0, pr_used=True, pr_res=3)])
        self.store.update(msg2)
        st2 = self.store.get('G07', 'GPS-L1CA')
        self.assertAlmostEqual(st2.pr_res_m, 3.0, places=2)

    def test_unknown_signal_name_falls_back(self):
        """Receiver may emit a (gnssId, sigId) the driver map doesn't
        list (firmware advance).  Store should fall back to a generic
        label, not crash."""
        msg = _FakeNavSig([
            {'gnssId': 0, 'svId': 7, 'sigId': 99,
             'sigFlags': 0x09, 'cno': 30, 'qualityInd': 1, 'prRes': 5},
        ])
        self.store.update(msg)
        st = self.store.get('G07', 'sys0/sig99')
        self.assertIsNotNone(st)

    def test_snapshot_full_map(self):
        msg = _FakeNavSig([
            _sig(0, 7, 0, pr_used=True),
            _sig(2, 19, 4, pr_used=False),
            _sig(3, 42, 0, pr_used=True),
        ])
        self.store.update(msg)
        snap = self.store.snapshot()
        self.assertEqual(len(snap), 3)
        self.assertIn(('G07', 'GPS-L1CA'), snap)
        self.assertIn(('E19', 'GAL-E5aQ'), snap)
        self.assertIn(('C42', 'BDS-B1I'), snap)


class _PrUsedTransitions(unittest.TestCase):

    def setUp(self):
        self.store = Nav2SignalStore()
        self.store.set_signal_names(_SIGNAL_NAMES)

    def test_pr_used_flip_returned(self):
        # Epoch 1: prUsed=True
        msg1 = _FakeNavSig([_sig(0, 7, 0, pr_used=True)])
        transitions1 = self.store.update(msg1)
        self.assertEqual(transitions1, [],
                          "first observation is not a transition")

        # Epoch 2: prUsed=False — transition fires
        msg2 = _FakeNavSig([_sig(0, 7, 0, pr_used=False)])
        transitions2 = self.store.update(msg2)
        self.assertEqual(transitions2, [('G07', 'GPS-L1CA', True, False)])

        # Epoch 3: same state — no transition
        msg3 = _FakeNavSig([_sig(0, 7, 0, pr_used=False)])
        transitions3 = self.store.update(msg3)
        self.assertEqual(transitions3, [])

        # Epoch 4: flip back
        msg4 = _FakeNavSig([_sig(0, 7, 0, pr_used=True)])
        transitions4 = self.store.update(msg4)
        self.assertEqual(transitions4, [('G07', 'GPS-L1CA', False, True)])

    def test_on_transition_callback_fires(self):
        events = []
        def cb(sv, sig, prev, new, status):
            events.append((sv, sig, prev, new, status.cno))
        self.store.on_transition(cb)

        # Establish baseline.
        self.store.update(_FakeNavSig([_sig(0, 7, 0, pr_used=True, cno=44)]))
        # Flip prUsed → False.
        self.store.update(_FakeNavSig([_sig(0, 7, 0, pr_used=False, cno=39)]))

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0][:4], ('G07', 'GPS-L1CA', True, False))
        self.assertEqual(events[0][4], 39)  # status's NEW cno

    def test_callback_exception_does_not_break_others(self):
        events = []
        def bad_cb(*args):
            raise RuntimeError("oops")
        def good_cb(sv, sig, prev, new, status):
            events.append(sv)
        self.store.on_transition(bad_cb)
        self.store.on_transition(good_cb)
        self.store.update(_FakeNavSig([_sig(0, 7, 0, pr_used=True)]))
        self.store.update(_FakeNavSig([_sig(0, 7, 0, pr_used=False)]))
        self.assertEqual(events, ['G07'])  # good_cb still fires


class _GetSignalContract(unittest.TestCase):
    """get_signal() + iter_signals() per the Phase A.5 monitor contract
    (charlie/secondOpinionPinPos @ b42569f).
    """

    def setUp(self):
        self.store = Nav2SignalStore()
        self.store.set_signal_names(_SIGNAL_NAMES)

    def test_get_signal_returns_dict_with_required_keys(self):
        self.store.update(_FakeNavSig([
            _sig(0, 7, 0, pr_used=True, cno=44, pr_res=3),
        ]))
        d = self.store.get_signal('G07', 'GPS-L1CA')
        self.assertIsInstance(d, dict)
        # Required keys per the spec:
        for k in ('prUsed', 'prRes', 'cno', 'health'):
            self.assertIn(k, d)
        # Optional keys:
        for k in ('crUsed', 'doUsed', 'prSmoothed', 'qualityInd',
                  'hostMono', 'ageS'):
            self.assertIn(k, d)
        self.assertTrue(d['prUsed'])
        self.assertEqual(d['cno'], 44)
        self.assertEqual(d['health'], 1)

    def test_get_signal_missing_returns_none(self):
        # Never seen: not in store yet
        self.assertIsNone(self.store.get_signal('G07', 'GPS-L1CA'))

    def test_get_signal_stale_returns_none(self):
        """If the SigStatus is older than max_age_s, return None."""
        import time as _time
        self.store.update(_FakeNavSig([
            _sig(0, 7, 0, pr_used=True),
        ]))
        # Force the stored host_mono back so the entry is "stale".
        st = self.store.get('G07', 'GPS-L1CA')
        st.host_mono = _time.monotonic() - 10.0  # 10 s old
        self.assertIsNone(
            self.store.get_signal('G07', 'GPS-L1CA', max_age_s=5.0))
        # But within a larger window it returns:
        self.assertIsNotNone(
            self.store.get_signal('G07', 'GPS-L1CA', max_age_s=30.0))

    def test_iter_signals_yields_fresh_pairs(self):
        self.store.update(_FakeNavSig([
            _sig(0, 7, 0, pr_used=True),
            _sig(2, 19, 4, pr_used=False),
            _sig(3, 42, 0, pr_used=True),
        ]))
        pairs = list(self.store.iter_signals())
        self.assertEqual(set(pairs), {
            ('G07', 'GPS-L1CA'),
            ('E19', 'GAL-E5aQ'),
            ('C42', 'BDS-B1I'),
        })

    def test_iter_signals_filters_stale(self):
        import time as _time
        self.store.update(_FakeNavSig([
            _sig(0, 7, 0, pr_used=True),
            _sig(2, 19, 4, pr_used=False),
        ]))
        # Make E19 stale:
        st = self.store.get('E19', 'GAL-E5aQ')
        st.host_mono = _time.monotonic() - 10.0
        pairs = list(self.store.iter_signals(max_age_s=5.0))
        self.assertEqual(pairs, [('G07', 'GPS-L1CA')])

    def test_get_signal_camelcase_keys_match_spec(self):
        """Dict keys are UBX-aligned camelCase to match consumer
        expectations — NOT snake_case used internally."""
        self.store.update(_FakeNavSig([
            _sig(0, 7, 0, pr_used=True, cr_used=True, do_used=False),
        ]))
        d = self.store.get_signal('G07', 'GPS-L1CA')
        # camelCase spec:
        self.assertIn('prUsed', d)
        self.assertIn('crUsed', d)
        self.assertIn('doUsed', d)
        # NOT snake_case:
        self.assertNotIn('pr_used', d)
        self.assertNotIn('cr_used', d)


class _NavClockStoreTest(unittest.TestCase):
    def test_empty_then_update(self):
        store = NavClockStore()
        self.assertIsNone(store.get())

        class M:
            clkB = 1234
            clkD = -56
            tAcc = 100
            fAcc = 200
            iTOW = 123456789
        store.update(M())
        snap = store.get()
        self.assertEqual(snap['clk_b_ns'], 1234)
        self.assertEqual(snap['clk_d_ns_per_s'], -56)
        self.assertEqual(snap['t_acc_ns'], 100)
        self.assertEqual(snap['f_acc_ps_per_s'], 200)
        self.assertEqual(snap['itow_ms'], 123456789)
        self.assertEqual(snap['n_updates'], 1)


class _NavTimeGpsStoreTest(unittest.TestCase):
    def test_valid_flags(self):
        store = NavTimeGpsStore()
        class M:
            iTOW = 100
            fTOW = 200
            week = 2300
            leapS = 18
            tAcc = 5
            valid = 0x07  # all three valid bits set
        store.update(M())
        snap = store.get()
        self.assertTrue(snap['valid_tow'])
        self.assertTrue(snap['valid_week'])
        self.assertTrue(snap['valid_leap_s'])
        self.assertEqual(snap['week'], 2300)
        self.assertEqual(snap['leap_s'], 18)

    def test_invalid_flags(self):
        store = NavTimeGpsStore()
        class M:
            iTOW = 100
            fTOW = 0
            week = 0
            leapS = 0
            tAcc = 999
            valid = 0  # nothing valid
        store.update(M())
        snap = store.get()
        self.assertFalse(snap['valid_tow'])
        self.assertFalse(snap['valid_week'])
        self.assertFalse(snap['valid_leap_s'])


if __name__ == '__main__':
    unittest.main()
