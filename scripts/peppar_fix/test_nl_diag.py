"""Unit tests for the NL attempt diagnostic (scripts/peppar_fix/nl_diag.py).

No hardware, no resolver, no filter — exercises the logger's API directly
by capturing its log output and asserting that the expected fields and
result strings land on each line.
"""

from __future__ import annotations

import logging
import unittest

from peppar_fix.nl_diag import (
    NlDiagLogger,
    RESULT_CAND, RESULT_FIXED_LAMBDA, RESULT_FIXED_ROUNDING,
    RESULT_SKIP_ELEV, RESULT_SKIP_BLACKLIST, RESULT_SKIP_NO_WL,
    RESULT_SKIP_PRESCREEN,
    RESULT_REJ_LAMBDA_RATIO, RESULT_REJ_CORNER,
)


class _CaptureHandler(logging.Handler):
    """Collect every emitted log record's message."""

    def __init__(self):
        super().__init__()
        self.messages: list[str] = []

    def emit(self, record):
        self.messages.append(self.format(record))


def _attach_capture():
    handler = _CaptureHandler()
    logger = logging.getLogger("peppar_fix.nl_diag")
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    return handler, logger


def _detach_capture(handler, logger):
    logger.removeHandler(handler)


class DisabledLoggerTest(unittest.TestCase):
    """Disabled logger must be a no-op and emit nothing."""

    def test_no_output_when_disabled(self):
        handler, logger = _attach_capture()
        try:
            d = NlDiagLogger(enabled=False)
            d.begin(100)
            d.record(sv="G01", result=RESULT_CAND, n1_frac=0.1)
            d.set_lambda_batch_summary(n=5, ratio=2.0, p_bootstrap=0.9,
                                       result=RESULT_FIXED_LAMBDA)
            d.emit()
            self.assertEqual(handler.messages, [])
        finally:
            _detach_capture(handler, logger)


class EnabledLoggerTest(unittest.TestCase):
    def setUp(self):
        self.handler, self.logger = _attach_capture()
        self.d = NlDiagLogger(enabled=True)

    def tearDown(self):
        _detach_capture(self.handler, self.logger)

    def test_single_record_emits_line(self):
        self.d.begin(42)
        self.d.record(sv="E23", elev_deg=72.0, az_deg=178.0,
                      n1_frac=0.083, sigma_n1_cyc=0.124,
                      wl_fixed_count=6, result=RESULT_CAND)
        self.d.emit()
        self.assertEqual(len(self.handler.messages), 1)
        line = self.handler.messages[0]
        self.assertIn("[NL_DIAG]", line)
        self.assertIn("epoch=42", line)
        self.assertIn("sv=E23", line)
        self.assertIn("elev=72", line)
        self.assertIn("frac=0.083", line)
        self.assertIn("sigma=0.124", line)
        self.assertIn("wl_fixed=6", line)
        self.assertIn("result=CAND", line)

    def test_record_upsert_preserves_unset_fields(self):
        self.d.begin(99)
        self.d.record(sv="G01", elev_deg=45.0, result=RESULT_CAND,
                      n1_frac=0.05, sigma_n1_cyc=0.1)
        # Later in attempt: LAMBDA rejected this SV's batch.  The update
        # merges the ratio without wiping elev/frac.
        self.d.update("G01", lambda_ratio=1.7, lambda_p_bootstrap=0.88,
                      result=RESULT_REJ_LAMBDA_RATIO)
        self.d.emit()
        self.assertEqual(len(self.handler.messages), 1)
        line = self.handler.messages[0]
        self.assertIn("elev=45", line)
        self.assertIn("frac=0.050", line)
        self.assertIn("sigma=0.100", line)
        self.assertIn("ratio=1.700", line)
        self.assertIn("p_bootstrap=0.8800", line)
        self.assertIn("result=REJECT_LAMBDA_RATIO", line)

    def test_batch_summary_emits_separate_line(self):
        self.d.begin(100)
        self.d.record(sv="E01", elev_deg=50.0, result=RESULT_CAND,
                      n1_frac=0.1, sigma_n1_cyc=0.1)
        self.d.set_lambda_batch_summary(
            n=5, ratio=4.2, p_bootstrap=0.9972, result=RESULT_FIXED_LAMBDA,
        )
        self.d.emit()
        self.assertEqual(len(self.handler.messages), 2)
        # First line: per-SV record
        self.assertIn("[NL_DIAG]", self.handler.messages[0])
        self.assertIn("sv=E01", self.handler.messages[0])
        # Second line: batch summary
        batch = self.handler.messages[1]
        self.assertIn("[NL_DIAG_BATCH]", batch)
        self.assertIn("epoch=100", batch)
        self.assertIn("n=5", batch)
        self.assertIn("ratio=4.200", batch)
        self.assertIn("p_bootstrap=0.9972", batch)
        self.assertIn("result=FIXED_LAMBDA", batch)

    def test_set_lambda_batch_result_mass_updates(self):
        self.d.begin(200)
        for sv in ("E01", "E02", "E03"):
            self.d.record(sv=sv, elev_deg=60.0, result=RESULT_CAND,
                          n1_frac=0.05, sigma_n1_cyc=0.1)
        self.d.record(sv="E99", elev_deg=15.0, result=RESULT_SKIP_ELEV)
        self.d.set_lambda_batch_result(
            ["E01", "E02", "E03"],
            ratio=4.0, p_bootstrap=0.999, result=RESULT_FIXED_LAMBDA,
        )
        self.d.emit()
        msgs = self.handler.messages
        self.assertEqual(len(msgs), 4)  # no batch summary this test
        by_sv = {m.split("sv=", 1)[1].split()[0]: m for m in msgs}
        for sv in ("E01", "E02", "E03"):
            self.assertIn("result=FIXED_LAMBDA", by_sv[sv])
            self.assertIn("ratio=4.000", by_sv[sv])
        # E99 was a SKIP_ELEV record and must stay that way.
        self.assertIn("result=SKIP_ELEV", by_sv["E99"])

    def test_emit_clears_buffer(self):
        self.d.begin(1)
        self.d.record(sv="G01", result=RESULT_CAND, elev_deg=45.0,
                      n1_frac=0.1, sigma_n1_cyc=0.1)
        self.d.emit()
        self.d.begin(2)
        # Without this begin call's clear, the G01 record would re-emit
        # at epoch=2.  Assert we got only one line from each emit.
        self.d.record(sv="G02", result=RESULT_CAND, elev_deg=45.0,
                      n1_frac=0.1, sigma_n1_cyc=0.1)
        self.d.emit()
        self.assertEqual(len(self.handler.messages), 2)
        self.assertIn("sv=G01", self.handler.messages[0])
        self.assertIn("sv=G02", self.handler.messages[1])
        # Epoch field moved forward between begins.
        self.assertIn("epoch=1", self.handler.messages[0])
        self.assertIn("epoch=2", self.handler.messages[1])

    def test_skip_blacklist_records_remaining_epochs(self):
        self.d.begin(10)
        self.d.record(sv="E13", elev_deg=30.0, wl_fixed_count=4,
                      blacklist_remaining=45, result=RESULT_SKIP_BLACKLIST)
        self.d.emit()
        line = self.handler.messages[0]
        self.assertIn("result=SKIP_BLACKLIST", line)
        self.assertIn("bl_rem=45", line)

    def test_rounding_rejection_carries_corner_margin(self):
        self.d.begin(50)
        self.d.record(sv="G17", result=RESULT_CAND, elev_deg=40.0,
                      n1_frac=0.09, sigma_n1_cyc=0.11,
                      corner_margin_sum=1.82, reason="corner")
        self.d.update("G17", result=RESULT_REJ_CORNER)
        self.d.emit()
        line = self.handler.messages[0]
        self.assertIn("corner=1.820", line)
        self.assertIn("result=REJECT_CORNER", line)


if __name__ == "__main__":
    unittest.main()
