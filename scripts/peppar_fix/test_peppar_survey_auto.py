"""Tests for the peppar-survey --auto backend selector (S3 of I-071401).

The selector is pure predicates + a ranking fn over injected inputs, so
these are deterministic with no network/tool/filesystem dependence.  The
core invariant under test: heuristics only ever change *which* (faster)
backend runs — the PRIDE floor is always the fallback.
"""
from __future__ import annotations

import json
import os
import sys
import unittest
from tempfile import TemporaryDirectory

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.peppar_survey_auto import (
    BackendPlan, capabilities_for_module, capabilities_for_uid,
    ecef_to_latlon, plan_auto, select_backend,
)
from peppar_fix.peppar_survey_discovery import BaseDescriptor, RegionSource


def _region(name="NGS CORS", kind="ngs_cors", realization="NAD83(2011)"):
    return RegionSource(name, kind, (15.0, 72.0, -170.0, -50.0), realization)


def _base(station="DSP1", km=12.0, realization="NAD83(2011)"):
    return BaseDescriptor(station, km, _region(realization=realization),
                          realization)


class CapabilitiesTest(unittest.TestCase):
    def test_f9p_is_dual_freq_rtk(self):
        c = capabilities_for_module("ZED-F9P")
        self.assertTrue(c.dual_freq)
        self.assertTrue(c.rtk_fw)

    def test_f9t_dual_freq_no_rtk(self):
        c = capabilities_for_module("ZED-F9T")
        self.assertTrue(c.dual_freq)
        self.assertFalse(c.rtk_fw)

    def test_lea_f9t_variant_matches_substring(self):
        self.assertTrue(capabilities_for_module("LEA-F9T-11B").dual_freq)

    def test_x20p_has_l5(self):
        self.assertIn("L5", capabilities_for_module("ZED-X20P").bands)

    def test_unknown_is_conservative(self):
        c = capabilities_for_module("SOME-NEW-CHIP")
        self.assertFalse(c.dual_freq)
        self.assertFalse(c.rtk_fw)

    def test_none_module_is_conservative(self):
        self.assertFalse(capabilities_for_module(None).dual_freq)


class EcefToLatLonTest(unittest.TestCase):
    def test_round_trips_a_known_point(self):
        from pyproj import Transformer
        # Chicago-ish lat/lon/h -> ECEF -> back.
        fwd = Transformer.from_crs(4979, 4978, always_xy=True)
        x, y, z = fwd.transform(-88.10, 41.85, 200.0)
        lat, lon = ecef_to_latlon((x, y, z))
        self.assertAlmostEqual(lat, 41.85, places=4)
        self.assertAlmostEqual(lon, -88.10, places=4)


class SelectBackendTest(unittest.TestCase):
    """The ranking fn: fastest viable, else the PRIDE floor."""

    def _f9p(self):
        return capabilities_for_module("ZED-F9P")

    def test_dual_freq_with_region_and_base_picks_baseline(self):
        plan = select_backend(
            caps=self._f9p(), region_source=_region(), base_desc=_base(),
            have_pride=True, have_rtklib=True)
        self.assertEqual(plan.backend, "baseline")
        self.assertEqual(plan.base.station, "DSP1")
        self.assertEqual(plan.base_realization, "NAD83(2011)")

    def test_no_base_falls_back_to_pride_floor(self):
        plan = select_backend(
            caps=self._f9p(), region_source=_region(), base_desc=None,
            have_pride=True, have_rtklib=True)
        self.assertEqual(plan.backend, "pride")
        self.assertIn("no fixed base", plan.reason)

    def test_no_region_falls_back_to_pride(self):
        plan = select_backend(
            caps=self._f9p(), region_source=None, base_desc=None,
            have_pride=True, have_rtklib=True)
        self.assertEqual(plan.backend, "pride")

    def test_single_freq_never_baselines(self):
        single = capabilities_for_module("UNKNOWN-CHIP")
        plan = select_backend(
            caps=single, region_source=_region(), base_desc=_base(),
            have_pride=True, have_rtklib=True)
        self.assertEqual(plan.backend, "pride")
        self.assertIn("not dual-frequency", plan.reason)

    def test_rtklib_only_when_pride_missing(self):
        plan = select_backend(
            caps=self._f9p(), region_source=None, base_desc=None,
            have_pride=False, have_rtklib=True)
        self.assertEqual(plan.backend, "rtklib")

    def test_names_pride_floor_even_when_nothing_installed(self):
        # Correctness floor is always named so --plan-only can tell the
        # operator what to install; a real run errors honestly.
        plan = select_backend(
            caps=self._f9p(), region_source=None, base_desc=None,
            have_pride=False, have_rtklib=False)
        self.assertEqual(plan.backend, "pride")
        self.assertIn("install", plan.reason.lower())

    def test_f9t_can_baseline_offline(self):
        # F9T lacks RTK positioning fw, but offline relative-baseline
        # post-processing only needs dual-freq carrier phase.
        plan = select_backend(
            caps=capabilities_for_module("ZED-F9T"),
            region_source=_region(), base_desc=_base(),
            have_pride=True, have_rtklib=True)
        self.assertEqual(plan.backend, "baseline")
        self.assertIn("RTK fw not required", plan.reason)


def _write_receiver(d, uid, module, ecef=None):
    state = {"unique_id": uid, "module": module}
    if ecef is not None:
        state["last_known_position"] = {"ecef_m": list(ecef), "sigma_m": 0.5,
                                        "source": "test", "updated": "now"}
    with open(os.path.join(d, f"{uid}.json"), "w") as f:
        json.dump(state, f)


class CapabilitiesForUidTest(unittest.TestCase):
    def test_reads_module_and_position(self):
        from pyproj import Transformer
        fwd = Transformer.from_crs(4979, 4978, always_xy=True)
        ecef = fwd.transform(-88.10, 41.85, 200.0)
        with TemporaryDirectory() as d:
            _write_receiver(d, "U1", "ZED-F9P", ecef=ecef)
            caps, latlon = capabilities_for_uid("U1", d)
        self.assertTrue(caps.dual_freq)
        self.assertIsNotNone(latlon)
        self.assertAlmostEqual(latlon[0], 41.85, places=3)

    def test_no_position_returns_none_latlon(self):
        with TemporaryDirectory() as d:
            _write_receiver(d, "U2", "ZED-F9T", ecef=None)
            caps, latlon = capabilities_for_uid("U2", d)
        self.assertTrue(caps.dual_freq)
        self.assertIsNone(latlon)


class PlanAutoTest(unittest.TestCase):
    """The glue: resolve caps+region+base via injected I/O, rank, return."""

    def _receivers(self, d, uid="U", module="ZED-F9P", at=(41.85, -88.10)):
        from pyproj import Transformer
        fwd = Transformer.from_crs(4979, 4978, always_xy=True)
        ecef = fwd.transform(at[1], at[0], 200.0)
        _write_receiver(d, uid, module, ecef=ecef)

    def test_baseline_when_region_and_base_found(self):
        with TemporaryDirectory() as d:
            self._receivers(d)
            plan = plan_auto(
                uid="U", receivers_dir=d, caster_host="caster.example",
                have_pride=True, have_rtklib=True,
                source_for_position_fn=lambda lat, lon: _region(),
                discover_base_fn=lambda lat, lon, **k: _base())
        self.assertEqual(plan.backend, "baseline")

    def test_near_override_used(self):
        seen = {}

        def fake_region(lat, lon):
            seen["latlon"] = (lat, lon)
            return None
        with TemporaryDirectory() as d:
            self._receivers(d, at=(41.85, -88.10))
            plan_auto(uid="U", receivers_dir=d, near=(51.49, -0.06),
                      have_pride=True, have_rtklib=True,
                      source_for_position_fn=fake_region,
                      discover_base_fn=lambda *a, **k: None)
        self.assertAlmostEqual(seen["latlon"][0], 51.49)

    def test_no_caster_means_no_base_discovery(self):
        called = {"discover": False}

        def fake_discover(*a, **k):
            called["discover"] = True
            return _base()
        with TemporaryDirectory() as d:
            self._receivers(d)
            plan = plan_auto(
                uid="U", receivers_dir=d, caster_host=None,
                have_pride=True, have_rtklib=True,
                source_for_position_fn=lambda lat, lon: _region(),
                discover_base_fn=fake_discover)
        self.assertFalse(called["discover"])
        self.assertEqual(plan.backend, "pride")

    def test_no_position_falls_back_to_pride(self):
        with TemporaryDirectory() as d:
            _write_receiver(d, "U", "ZED-F9P", ecef=None)
            plan = plan_auto(uid="U", receivers_dir=d,
                             caster_host="caster.example",
                             have_pride=True, have_rtklib=True)
        self.assertEqual(plan.backend, "pride")
        self.assertIsInstance(plan, BackendPlan)


if __name__ == "__main__":
    unittest.main()
