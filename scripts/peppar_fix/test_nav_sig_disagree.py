"""Unit tests for NavSigDisagreeMonitor (Phase A.5 logger)."""
from __future__ import annotations

import logging
import unittest

from peppar_fix.nav_sig_disagree import (
    NavSigDisagreeMonitor,
    StubSignalStore,
    engine_sig_to_ubx_sigid,
)


class NavSigDisagreeMonitorTest(unittest.TestCase):

    def setUp(self):
        self.monitor = NavSigDisagreeMonitor()
        self.store = StubSignalStore()
        # Capture log records emitted by the monitor.
        self.log_records: list[logging.LogRecord] = []
        h = logging.Handler()
        h.emit = lambda rec: self.log_records.append(rec)
        h.setLevel(logging.INFO)
        log = logging.getLogger("peppar_fix.nav_sig_disagree")
        log.addHandler(h)
        log.setLevel(logging.INFO)
        self.addCleanup(lambda: log.removeHandler(h))

    def _log_messages(self) -> list[str]:
        return [r.getMessage() for r in self.log_records]

    # ─── Agreement (no log) ──────────────────────────────────────── #

    def test_both_admit_no_log(self):
        self.store.set("G07", "GPS-L1CA", prUsed=True, cno=44, prRes=0.3)
        admit = {("G07", "GPS-L1CA"): {"lock_time_ms": 64500, "cno": 43}}
        n = self.monitor.check_epoch(100, admit, self.store)
        self.assertEqual(n, 0)
        self.assertEqual(self._log_messages(), [])

    def test_both_exclude_no_log(self):
        self.store.set("E19", "GAL-E5aQ", prUsed=False, cno=22, prRes=-3.1)
        admit = {}  # we also exclude
        n = self.monitor.check_epoch(100, admit, self.store)
        self.assertEqual(n, 0)
        self.assertEqual(self._log_messages(), [])

    # ─── Disagreement: we_admit + receiver_excludes ─────────────── #

    def test_we_admit_receiver_excludes_logs(self):
        self.store.set("E19", "GAL-E5aQ", prUsed=False, cno=39,
                       prRes=412.7, health=1)
        admit = {("E19", "GAL-E5aQ"): {
            "lock_time_ms": 64500, "cno": 39,
            "gf_phase_m": 0.3, "mw_cycles_smoothed": -2.71,
            "elev_deg": 21.0, "az_deg": 146.0,
            "last_slip_reason": "mw_jump_LOW",
            "our_admission_reason": "ar_phase_bias_ok",
        }}
        self.monitor.check_epoch(100, admit, self.store)
        msgs = self._log_messages()
        self.assertEqual(len(msgs), 1)
        m = msgs[0]
        self.assertIn("sv=E19", m)
        self.assertIn("sig=GAL-E5aQ", m)
        self.assertIn("receiver=excluded our=admit", m)
        self.assertIn("ep=100", m)
        # Diagnostic fields present
        self.assertIn("lock_ms=64500", m)
        self.assertIn("gf_m=0.300", m)
        self.assertIn("mw_c=-2.710", m)
        self.assertIn("pr_res_rcv_m=412.700", m)
        self.assertIn("health=1", m)
        self.assertIn("slip=mw_jump_LOW", m)
        self.assertIn("our_reason=ar_phase_bias_ok", m)

    # ─── Disagreement: we_exclude + receiver_admits ─────────────── #

    def test_we_exclude_receiver_admits_logs(self):
        self.store.set("C42", "BDS-B2aI", prUsed=True, cno=37, prRes=0.5)
        admit = {}  # we excluded — perhaps PB_GAP_DROP
        self.monitor.check_epoch(100, admit, self.store)
        msgs = self._log_messages()
        self.assertEqual(len(msgs), 1)
        self.assertIn("receiver=admitted our=exclude", msgs[0])
        self.assertIn("sv=C42", msgs[0])
        self.assertIn("sig=BDS-B2aI", msgs[0])

    # ─── Transition detection (no log spam) ─────────────────────── #

    def test_no_log_on_steady_disagreement(self):
        # Persistent disagreement: log on FIRST epoch only.
        self.store.set("E19", "GAL-E5aQ", prUsed=False, cno=39)
        admit = {("E19", "GAL-E5aQ"): {"lock_time_ms": 64500}}
        n1 = self.monitor.check_epoch(100, admit, self.store)
        n2 = self.monitor.check_epoch(101, admit, self.store)
        n3 = self.monitor.check_epoch(102, admit, self.store)
        self.assertEqual(n1, 1)
        self.assertEqual(n2, 0)
        self.assertEqual(n3, 0)
        self.assertEqual(len(self._log_messages()), 1)

    def test_log_again_on_transition_back(self):
        # Disagreement → agreement → disagreement: log twice.
        admit = {("E19", "GAL-E5aQ"): {"lock_time_ms": 64500}}
        self.store.set("E19", "GAL-E5aQ", prUsed=False)
        self.monitor.check_epoch(100, admit, self.store)
        # Receiver now admits — agreement
        self.store.set("E19", "GAL-E5aQ", prUsed=True)
        self.monitor.check_epoch(101, admit, self.store)
        # Receiver excludes again — disagreement returns
        self.store.set("E19", "GAL-E5aQ", prUsed=False)
        self.monitor.check_epoch(102, admit, self.store)
        msgs = self._log_messages()
        # First and third epochs should log; second is agreement.
        self.assertEqual(len(msgs), 2)
        self.assertIn("ep=100", msgs[0])
        self.assertIn("ep=102", msgs[1])

    def test_log_on_direction_flip(self):
        # we_admit + recv_excludes → we_exclude + recv_admits
        # Both are disagreements but different direction — log twice.
        self.store.set("E19", "GAL-E5aQ", prUsed=False)
        admit_with = {("E19", "GAL-E5aQ"): {"lock_time_ms": 64500}}
        self.monitor.check_epoch(100, admit_with, self.store)
        # Flip both
        self.store.set("E19", "GAL-E5aQ", prUsed=True)
        admit_without: dict = {}
        self.monitor.check_epoch(101, admit_without, self.store)
        msgs = self._log_messages()
        self.assertEqual(len(msgs), 2)
        self.assertIn("receiver=excluded our=admit", msgs[0])
        self.assertIn("receiver=admitted our=exclude", msgs[1])

    # ─── Edge cases ──────────────────────────────────────────────── #

    def test_none_signal_store_no_op(self):
        admit = {("G07", "GPS-L1CA"): {}}
        n = self.monitor.check_epoch(100, admit, None)
        self.assertEqual(n, 0)
        self.assertEqual(self._log_messages(), [])

    def test_missing_receiver_sig_no_log(self):
        # Engine admits a signal the receiver doesn't have a record for.
        # Don't classify (signal-store gap, not a real disagreement).
        admit = {("G99", "GPS-L1CA"): {}}  # G99 not in store
        n = self.monitor.check_epoch(100, admit, self.store)
        self.assertEqual(n, 0)

    def test_engine_sig_to_ubx_known_mappings(self):
        self.assertEqual(engine_sig_to_ubx_sigid("GPS-L1CA"), (0, 0))
        self.assertEqual(engine_sig_to_ubx_sigid("GPS-L5Q"), (0, 7))
        self.assertEqual(engine_sig_to_ubx_sigid("GAL-E1C"), (2, 0))
        self.assertEqual(engine_sig_to_ubx_sigid("GAL-E5aQ"), (2, 4))
        self.assertEqual(engine_sig_to_ubx_sigid("BDS-B2aI"), (3, 5))
        # Unknown signal returns None
        self.assertIsNone(engine_sig_to_ubx_sigid("NOPE-X1Z"))

    def test_state_size_tracking(self):
        self.store.set("E19", "GAL-E5aQ", prUsed=False)
        self.store.set("G07", "GPS-L1CA", prUsed=True)
        admit = {("G07", "GPS-L1CA"): {}}
        self.monitor.check_epoch(100, admit, self.store)
        self.assertEqual(self.monitor.state_size(), 2)
        self.monitor.reset()
        self.assertEqual(self.monitor.state_size(), 0)


if __name__ == "__main__":
    unittest.main()
