"""Tests for the no-gnss-pps experimental source-competition lockout.

The branch hypothesis is that F9T PPS is not needed for clock discipline
once the Carrier (PPP dt_rx) source is wired into the source competition.
The lockout makes this concrete: PPS, PPS+qErr, TICC (qErr-corrected),
and PPS+PPP all get hardwired huge confidence so Carrier wins whenever
it is initialized and within sigma bounds.

These tests pin the lockout's contract:
  * Carrier wins over every PPS-edge-derived source when both are
    available, regardless of the deprecated source's nominal confidence.
  * When Carrier is not available (uninitialized tracker, sigma too
    high, or no ppp_cal), the source list still contains the
    deprecated sources at huge confidence — preserving the engine's
    downstream consumers from an empty list, but at a confidence level
    that the scheduler treats as effectively unusable.

If this test starts failing, either the lockout was unwound (good if
the experiment succeeded and we removed the deprecated sources entirely
in the merge) or someone narrowed the suppression — neither should
happen silently.
"""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.error_sources import compute_error_sources


class _FakeCarrierTracker:
    def __init__(self, initialized=True, offset_ns=0.0):
        self.initialized = initialized
        self._offset = offset_ns

    def compute_error(self, dt_rx_ns):
        return dt_rx_ns + self._offset


class _FakePppCal:
    def __init__(self, calibrated=True, offset_ns=0.0):
        self.calibrated = calibrated
        self.offset_ns = offset_ns


class CarrierWinsTest(unittest.TestCase):

    def test_carrier_beats_pps_qerr_even_when_pps_qerr_looks_excellent(self):
        # Realistic post-calibration numbers: PPS+qErr nominally tight
        # (~0.2 ns) and Carrier in the few-ns range.  Without the
        # lockout, PPS+qErr would win.
        sources = compute_error_sources(
            pps_error_ns=0.0,
            qerr_ns=1.0,
            dt_rx_ns=42.0,
            dt_rx_sigma_ns=2.5,
            qerr_confidence=0.2,           # ignored by lockout
            pps_confidence=20.0,           # ignored by lockout
            carrier_tracker=_FakeCarrierTracker(),
            ticc_error_ns=0.5,
            ticc_confidence=0.3,           # ignored by lockout
        )
        self.assertEqual(sources[0].name, "Carrier")
        # All other sources are at the suppression value.
        for s in sources[1:]:
            self.assertGreater(s.confidence_ns, 1e5)

    def test_carrier_wins_over_ticc_even_with_tiny_ticc_confidence(self):
        sources = compute_error_sources(
            pps_error_ns=0.0,
            qerr_ns=None,
            dt_rx_ns=10.0,
            dt_rx_sigma_ns=5.0,
            carrier_tracker=_FakeCarrierTracker(),
            ticc_error_ns=0.1,
            ticc_confidence=0.05,          # ignored
        )
        self.assertEqual(sources[0].name, "Carrier")

    def test_carrier_loses_only_to_itself(self):
        # When carrier sigma > carrier_max_sigma, Carrier source is
        # not added.  Remaining sources are all suppressed.  Best
        # source then has the suppression confidence — engine
        # downstream treats that as "no useful steering input".
        sources = compute_error_sources(
            pps_error_ns=0.0,
            qerr_ns=1.0,
            dt_rx_ns=10.0,
            dt_rx_sigma_ns=100.0,           # > default carrier_max_sigma=50
            carrier_max_sigma=50.0,
            carrier_tracker=_FakeCarrierTracker(),
        )
        names = [s.name for s in sources]
        self.assertNotIn("Carrier", names)
        self.assertGreater(sources[0].confidence_ns, 1e5,
                           "best (suppressed) source should signal no-steering")

    def test_carrier_uninitialized_no_carrier_in_list(self):
        sources = compute_error_sources(
            pps_error_ns=0.0,
            qerr_ns=1.0,
            dt_rx_ns=10.0,
            dt_rx_sigma_ns=2.0,
            carrier_tracker=_FakeCarrierTracker(initialized=False),
        )
        names = [s.name for s in sources]
        self.assertNotIn("Carrier", names)
        # Suppressed sources still listed for fallback bookkeeping.
        self.assertIn("PPS", names)
        self.assertIn("PPS+qErr", names)


class SuppressedSourcesStillListedTest(unittest.TestCase):
    """The lockout must not remove sources from the list — engine
    downstream code (source-change logging, scheduler ingest, status
    emit) reads them."""

    def test_pps_always_listed(self):
        sources = compute_error_sources(
            pps_error_ns=0.0, qerr_ns=None,
            dt_rx_ns=0.0, dt_rx_sigma_ns=None,
        )
        self.assertIn("PPS", [s.name for s in sources])

    def test_pps_qerr_listed_when_qerr_present(self):
        sources = compute_error_sources(
            pps_error_ns=0.0, qerr_ns=2.0,
            dt_rx_ns=0.0, dt_rx_sigma_ns=None,
        )
        self.assertIn("PPS+qErr", [s.name for s in sources])

    def test_ticc_listed_when_provided(self):
        sources = compute_error_sources(
            pps_error_ns=0.0, qerr_ns=None,
            dt_rx_ns=0.0, dt_rx_sigma_ns=None,
            ticc_error_ns=0.0, ticc_confidence=0.1,
        )
        self.assertIn("TICC", [s.name for s in sources])

    def test_pps_ppp_listed_when_calibrated(self):
        sources = compute_error_sources(
            pps_error_ns=0.0, qerr_ns=None,
            dt_rx_ns=0.0, dt_rx_sigma_ns=2.0,
            ppp_cal=_FakePppCal(),
        )
        self.assertIn("PPS+PPP", [s.name for s in sources])


if __name__ == "__main__":
    unittest.main()
