#!/usr/bin/env python3
"""NAV-PVT seed fallback — for receivers without NAV2.

When NAV2 (the secondary nav engine) is unavailable — e.g. non-timing
u-blox firmware that NAKs CFG-NAV2, or any receiver not emitting
NAV2-PVT — the engine seeds position from the primary engine's NAV-PVT
fix instead.  These tests pin:

  1. Nav2PositionStore.has_data() flips on first ingest.
  2. wait_for_nav2_seed gives up fast (no_data_grace_s) when the store
     emits no PVT at all — instead of burning the full timeout.
  3. wait_for_nav2_seed succeeds on a populated store regardless of
     which PVT stream feeds it (NAV2-PVT or NAV-PVT).
  4. _apply_position_seed returns a consistent (ecef, sigma, source)
     and honours the honest-seed-σ floor.

Run: ./bin/test tests/test_nav_pvt_seed_fallback.py
"""
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'scripts'))

import peppar_fix_engine as engine
from realtime_ppp import Nav2PositionStore


def _pvt(lat=41.879, lon=-87.636, height_mm=200000.0, hacc_mm=620.0,
         fix_type=3, gnss_ok=1, num_sv=27):
    """A minimal stand-in for a decoded UBX-NAV-PVT / NAV2-PVT message
    (Nav2PositionStore.update reads these attributes)."""
    return SimpleNamespace(lat=lat, lon=lon, height=height_mm, hAcc=hacc_mm,
                           vAcc=900.0, pDOP=1.2, fixType=fix_type,
                           gnssFixOk=gnss_ok, numSV=num_sv)


class TestHasData(unittest.TestCase):
    def test_has_data_flips_on_ingest(self):
        s = Nav2PositionStore()
        self.assertFalse(s.has_data())
        s.update(_pvt())
        self.assertTrue(s.has_data())


class TestWaitForSeed(unittest.TestCase):
    def test_fast_giveup_when_no_pvt(self):
        """Empty store (engine emitting no PVT) → returns None within the
        grace window, not the full timeout."""
        store = Nav2PositionStore()
        stop = threading.Event()
        t0 = time.time()
        out = engine.wait_for_nav2_seed(store, stop, timeout_s=30.0,
                                        no_data_grace_s=0.5,
                                        source_label="NAV-PVT")
        elapsed = time.time() - t0
        self.assertIsNone(out)
        self.assertLess(elapsed, 5.0, "should give up fast, not wait 30s")

    def test_seeds_from_populated_store(self):
        """A populated store (fixType=3, good hAcc) yields a seed — works
        for a NAV-PVT-fed store just as for NAV2-PVT."""
        store = Nav2PositionStore()
        store.update(_pvt(hacc_mm=620.0, num_sv=27))
        out = engine.wait_for_nav2_seed(store, threading.Event(),
                                        timeout_s=5.0, source_label="NAV-PVT")
        self.assertIsNotNone(out)
        ecef, h_acc, n_sv = out
        self.assertEqual(len(ecef), 3)
        self.assertAlmostEqual(h_acc, 0.62, places=2)   # mm → m
        self.assertEqual(n_sv, 27)

    def test_stop_event_aborts(self):
        store = Nav2PositionStore()
        stop = threading.Event()
        stop.set()
        self.assertIsNone(engine.wait_for_nav2_seed(
            store, stop, timeout_s=30.0, no_data_grace_s=10.0))


class TestApplyPositionSeed(unittest.TestCase):
    def _ape(self):
        calls = []
        return SimpleNamespace(transition=lambda *a, **k: calls.append(a)), calls

    def test_returns_ecef_sigma_source(self):
        ape, calls = self._ape()
        args = SimpleNamespace(nav2_seed_sigma_floor_m=0.0)
        seed = ([1.0, 2.0, 3.0], 0.62, 27)
        ecef, sigma, source = engine._apply_position_seed(
            seed, "NAV-PVT", args, ape)
        self.assertEqual(ecef, [1.0, 2.0, 3.0])
        self.assertAlmostEqual(sigma, 0.62)
        self.assertIn("NAV-PVT", source)
        self.assertEqual(len(calls), 1)  # transitioned to VERIFYING

    def test_sigma_floor_applied(self):
        ape, _ = self._ape()
        args = SimpleNamespace(nav2_seed_sigma_floor_m=2.0)
        ecef, sigma, source = engine._apply_position_seed(
            ([1.0, 2.0, 3.0], 0.62, 27), "NAV2", args, ape)
        self.assertEqual(sigma, 2.0)  # floored up from 0.62

    def test_label_distinguishes_source(self):
        ape, _ = self._ape()
        args = SimpleNamespace(nav2_seed_sigma_floor_m=0.0)
        _, _, src_nav2 = engine._apply_position_seed(
            ([0, 0, 0], 0.5, 10), "NAV2", args, ape)
        _, _, src_pvt = engine._apply_position_seed(
            ([0, 0, 0], 0.5, 10), "NAV-PVT", args, ape)
        self.assertIn("NAV2", src_nav2)
        self.assertIn("NAV-PVT", src_pvt)


if __name__ == "__main__":
    unittest.main()
