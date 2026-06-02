"""Tests for ResetBudget — the windowed in-process-reset cap.

Mirrors decision (c) of docs/exit-five-to-servo-reset.md: at most N
resets in any rolling W-second window, shared across all reset
reasons, recovers after a quiet window.  The clock is injected so the
window logic is deterministic without sleeping.
"""
import unittest

from peppar_fix.reset_budget import ResetBudget


class ResetBudgetTests(unittest.TestCase):

    def test_construction_validates(self):
        with self.assertRaises(ValueError):
            ResetBudget(max_resets=0)
        with self.assertRaises(ValueError):
            ResetBudget(window_s=0)
        with self.assertRaises(ValueError):
            ResetBudget(window_s=-1)

    def test_disabled_budget_always_denies(self):
        # The contract holds even if a caller reaches request() directly
        # (bypassing the engine funnel).  enabled=False → always deny.
        b = ResetBudget(max_resets=3, window_s=300.0, enabled=False)
        self.assertFalse(b.request("ekf_outlier", now=1000.0))
        self.assertFalse(b.request("ekf_outlier", now=1010.0))
        self.assertEqual(b.total_allowed, 0)
        self.assertEqual(b.total_denied, 2)

    def test_sub_budget_all_allowed(self):
        b = ResetBudget(max_resets=3, window_s=300.0, enabled=True)
        # N requests inside the window → all allowed.
        self.assertTrue(b.request("ekf_outlier", now=1000.0))
        self.assertTrue(b.request("ekf_outlier", now=1010.0))
        self.assertTrue(b.request("ekf_outlier", now=1020.0))
        self.assertEqual(b.resets_in_window, 3)
        self.assertEqual(b.total_allowed, 3)
        self.assertEqual(b.total_denied, 0)

    def test_budget_exhaustion_denies(self):
        b = ResetBudget(max_resets=3, window_s=300.0, enabled=True)
        for t in (1000.0, 1010.0, 1020.0):
            self.assertTrue(b.request("ekf_outlier", now=t))
        # 4th inside the window → denied.
        self.assertFalse(b.request("ekf_outlier", now=1030.0))
        self.assertEqual(b.total_allowed, 3)
        self.assertEqual(b.total_denied, 1)
        # Denied request did not consume window slots beyond the 3.
        self.assertEqual(b.resets_in_window, 3)

    def test_recovery_after_quiet_window(self):
        b = ResetBudget(max_resets=2, window_s=300.0, enabled=True)
        self.assertTrue(b.request("phc_restep", now=1000.0))
        self.assertTrue(b.request("phc_restep", now=1100.0))
        self.assertFalse(b.request("phc_restep", now=1200.0))  # full
        # After > window_s of quiet, the old resets age out.
        self.assertTrue(b.request("phc_restep", now=1000.0 + 300.0 + 1))
        self.assertTrue(b.request("phc_restep", now=1402.0))

    def test_boundary_prune_requires_exceeding_window(self):
        # Pruning is strict (ts < now - window_s), so an entry exactly
        # window_s old is still counted; the window must be *exceeded*
        # to age it out.  This matches test_recovery_after_quiet_window.
        b = ResetBudget(max_resets=1, window_s=300.0, enabled=True)
        self.assertTrue(b.request("x", now=1000.0))
        self.assertFalse(b.request("x", now=1299.0))      # 299 s later: full
        self.assertFalse(b.request("x", now=1300.0))      # exactly 300 s: retained
        self.assertTrue(b.request("x", now=1300.001))     # just past 300 s: aged out

    def test_budget_is_shared_across_reasons(self):
        # The load-bearing test for decision (c): alternating reasons
        # draw from ONE window, not per-reason windows.
        b = ResetBudget(max_resets=3, window_s=300.0, enabled=True)
        self.assertTrue(b.request("ekf_outlier", now=1000.0))
        self.assertTrue(b.request("cm_outlier", now=1010.0))
        self.assertTrue(b.request("binary_layer", now=1020.0))
        # A 4th from yet another reason is still denied — shared budget.
        self.assertFalse(b.request("phc_restep", now=1030.0))
        self.assertEqual(b.cumulative_by_reason,
                         {"ekf_outlier": 1, "cm_outlier": 1,
                          "binary_layer": 1})

    def test_per_reason_cumulative_counts(self):
        b = ResetBudget(max_resets=10, window_s=300.0, enabled=True)
        b.request("ekf_outlier", now=1000.0)
        b.request("ekf_outlier", now=1010.0)
        b.request("cm_outlier", now=1020.0)
        self.assertEqual(b.cumulative_by_reason["ekf_outlier"], 2)
        self.assertEqual(b.cumulative_by_reason["cm_outlier"], 1)

    def test_stats_shape(self):
        b = ResetBudget(max_resets=3, window_s=300.0, enabled=True)
        b.request("ekf_outlier", now=1000.0)
        s = b.stats
        self.assertEqual(s["max_resets"], 3)
        self.assertEqual(s["window_s"], 300.0)
        self.assertTrue(s["enabled"])
        self.assertEqual(s["resets_in_window"], 1)
        self.assertEqual(s["total_allowed"], 1)


if __name__ == "__main__":
    unittest.main()
