"""Unit tests for receiver_state dt_rx seed freshness (ptboatStartupTransient).

A receiver clock bias (dt_rx) is not persistent across a reboot / long gap.
Seeding the DOFreqEst rx-TCXO phase from a stale value blows up the filter at
startup (ptBoat 2026-07-06).  These tests pin the freshness guard and the
save→seed round-trip that keeps a healthy start seeding from a fresh dt_rx.
"""

import os
import tempfile
import unittest
from datetime import datetime, timedelta, timezone

from peppar_fix.receiver_state import (
    DT_RX_SEED_MAX_AGE_S,
    dt_rx_seed_if_fresh,
    load_receiver_state,
    save_dt_rx_to_receiver,
    save_receiver_state,
)


def _state(dt_rx_ns=None, updated=None):
    tcxo = {}
    if dt_rx_ns is not None:
        tcxo["last_known_dt_rx_ns"] = dt_rx_ns
    if updated is not None:
        tcxo["updated"] = updated
    return {"tcxo": tcxo}


def _iso(dt):
    return dt.strftime("%Y-%m-%dT%H:%M:%SZ")


class TestDtRxSeedFreshness(unittest.TestCase):
    def setUp(self):
        self.now = datetime(2026, 7, 6, 9, 40, 0, tzinfo=timezone.utc)

    def test_fresh_seed_accepted(self):
        saved = self.now - timedelta(seconds=30)
        val, age = dt_rx_seed_if_fresh(
            _state(2557956.3, _iso(saved)), now=self.now)
        self.assertAlmostEqual(val, 2557956.3)
        self.assertAlmostEqual(age, 30.0, places=3)

    def test_stale_seed_rejected(self):
        # The ptBoat case: 8 days stale.
        saved = self.now - timedelta(days=8)
        val, age = dt_rx_seed_if_fresh(
            _state(-15886713.5, _iso(saved)), now=self.now)
        self.assertIsNone(val)
        self.assertGreater(age, DT_RX_SEED_MAX_AGE_S)

    def test_boundary_just_within_and_just_beyond(self):
        just_in = self.now - timedelta(seconds=DT_RX_SEED_MAX_AGE_S - 1)
        just_out = self.now - timedelta(seconds=DT_RX_SEED_MAX_AGE_S + 1)
        self.assertIsNotNone(
            dt_rx_seed_if_fresh(_state(1.0, _iso(just_in)), now=self.now)[0])
        self.assertIsNone(
            dt_rx_seed_if_fresh(_state(1.0, _iso(just_out)), now=self.now)[0])

    def test_missing_value(self):
        self.assertEqual(
            dt_rx_seed_if_fresh(_state(None, _iso(self.now)), now=self.now),
            (None, None))

    def test_missing_timestamp_rejected(self):
        # No timestamp ⇒ can't establish freshness ⇒ reject (don't trust it).
        self.assertEqual(
            dt_rx_seed_if_fresh(_state(1.0, None), now=self.now),
            (None, None))

    def test_unparseable_timestamp_rejected(self):
        self.assertEqual(
            dt_rx_seed_if_fresh(_state(1.0, "not-a-date"), now=self.now),
            (None, None))

    def test_negative_age_rejected(self):
        # Clock stepped backward since save ⇒ future timestamp ⇒ reject.
        future = self.now + timedelta(seconds=120)
        val, age = dt_rx_seed_if_fresh(
            _state(1.0, _iso(future)), now=self.now)
        self.assertIsNone(val)
        self.assertLess(age, 0)

    def test_empty_and_none_state(self):
        self.assertEqual(dt_rx_seed_if_fresh({}, now=self.now), (None, None))
        self.assertEqual(dt_rx_seed_if_fresh(None, now=self.now), (None, None))


class TestSaveDtRxRoundTrip(unittest.TestCase):
    def test_save_then_seed_is_fresh(self):
        with tempfile.TemporaryDirectory() as d:
            uid = "940005540004"
            # A receiver state file must exist for the save to attach to.
            save_receiver_state({"unique_id": uid, "tcxo": {}}, state_dir=d)
            save_dt_rx_to_receiver(uid, 2557956.3,
                                   freq_offset_ppb=-4127.5, state_dir=d)
            st = load_receiver_state(uid, state_dir=d)
            self.assertAlmostEqual(
                st["tcxo"]["last_known_dt_rx_ns"], 2557956.3)
            self.assertAlmostEqual(
                st["tcxo"]["last_known_freq_offset_ppb"], -4127.5)
            # Just-written ⇒ seed accepts it.
            val, age = dt_rx_seed_if_fresh(st)
            self.assertAlmostEqual(val, 2557956.3)
            self.assertLess(age, 5.0)

    def test_save_no_state_file_is_noop(self):
        with tempfile.TemporaryDirectory() as d:
            # No file for this uid — must not raise.
            save_dt_rx_to_receiver("nonexistent", 1.0, state_dir=d)
            self.assertIsNone(load_receiver_state("nonexistent", state_dir=d))

    def test_freq_only_save_does_not_refresh_stale_dt_rx(self):
        # A freq-only save must NOT bump the freshness timestamp, else it would
        # mark a stale dt_rx as fresh and defeat the seed guard.
        with tempfile.TemporaryDirectory() as d:
            uid = "940005540004"
            stale = "2026-06-28T09:00:00Z"
            save_receiver_state(
                {"unique_id": uid,
                 "tcxo": {"last_known_dt_rx_ns": -15886713.5,
                          "updated": stale}},
                state_dir=d)
            save_dt_rx_to_receiver(uid, None, freq_offset_ppb=-4127.5,
                                   state_dir=d)
            st = load_receiver_state(uid, state_dir=d)
            self.assertEqual(st["tcxo"]["updated"], stale)  # unchanged
            self.assertAlmostEqual(
                st["tcxo"]["last_known_freq_offset_ppb"], -4127.5)
            # The stale dt_rx must still be rejected by the seed guard.
            self.assertIsNone(dt_rx_seed_if_fresh(st)[0])


if __name__ == "__main__":
    unittest.main()
