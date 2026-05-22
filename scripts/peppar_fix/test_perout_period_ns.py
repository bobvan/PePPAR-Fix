"""Tests for the perout_period_ns plumb-through chain.

The MadHat both-halves-firing workaround is a per-host setting that
needs to flow from TOML config → argparse → bootstrap → setup_perout
→ enable_perout without being silently dropped at any boundary.
This test holds that chain together.
"""
from __future__ import annotations

import unittest
from unittest import mock


class SetupPeroutPassesPeriodTest(unittest.TestCase):
    """phc_bootstrap._enable_pps_out plumbs args.perout_period_ns
    through to setup_perout()."""

    def _run_enable(self, args_override: dict | None = None):
        from types import SimpleNamespace
        import phc_bootstrap

        defaults = dict(
            pps_out_pin=0,
            pps_out_channel=0,
            program_pin=True,
            ptp_dev="/dev/ptp_i226",
            ticc_port=None,
        )
        if args_override:
            defaults.update(args_override)
        args = SimpleNamespace(**defaults)
        captured = {}

        def fake_setup_perout(ptp, **kw):
            captured.update(kw)
            return True

        with mock.patch(
            "peppar_fix.perout_setup.setup_perout",
            side_effect=fake_setup_perout,
        ):
            phc_bootstrap._enable_pps_out(ptp=mock.Mock(), args=args)
        return captured

    def test_default_period_is_1_second(self):
        # Argparse default is None so _apply_host_config can override.
        # _enable_pps_out must fall back to 1 Hz when nothing's set.
        kw = self._run_enable({"perout_period_ns": None})
        self.assertEqual(kw.get("period_ns"), 1_000_000_000)

    def test_period_doubling_overrides_default(self):
        kw = self._run_enable({"perout_period_ns": 2_000_000_000})
        self.assertEqual(kw.get("period_ns"), 2_000_000_000)

    def test_argparse_default_is_none_so_toml_overrides_can_apply(self):
        """The actual lab-validated bug: argparse default for
        --perout-period-ns must be None, NOT 1_000_000_000.  Otherwise
        _apply_host_config in peppar_fix_engine sees a non-None value
        already on args and SKIPS the TOML override, silently leaving
        MadHat at the buggy 1 Hz period.  Empirically caught
        2026-05-22 when the engine bootstrap landed at chA=2.40 Hz
        on MadHat despite madhat.toml saying 2_000_000_000."""
        import os, ast
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        engine = os.path.join(here, "peppar_fix_engine.py")
        # Crude but robust: find the literal "--perout-period-ns" in the
        # source and walk forward to the default kwarg.
        with open(engine) as f:
            src = f.read()
        idx = src.find('"--perout-period-ns"')
        self.assertGreater(idx, 0, "--perout-period-ns flag not found")
        snippet = src[idx:idx + 400]
        # Require default=None in the same add_argument call.  If a
        # future edit puts an int default back, this test fires.
        self.assertIn("default=None", snippet,
                      "--perout-period-ns must use default=None so "
                      "_apply_host_config can override from per-host TOML; "
                      "found instead:\n" + snippet[:200])

    def test_pin_disabled_skips_setup_entirely(self):
        from types import SimpleNamespace
        import phc_bootstrap

        args = SimpleNamespace(pps_out_pin=-1, pps_out_channel=0,
                               perout_period_ns=2_000_000_000)
        called = []
        with mock.patch(
            "peppar_fix.perout_setup.setup_perout",
            side_effect=lambda *a, **kw: called.append(True),
        ):
            phc_bootstrap._enable_pps_out(ptp=mock.Mock(), args=args)
        self.assertEqual(called, [], "setup_perout called despite pin<0")


class SetupPeroutPassesPeriodToEnableTest(unittest.TestCase):
    """setup_perout in turn must pass period_ns to enable_perout."""

    def test_setup_perout_forwards_period(self):
        from peppar_fix.perout_setup import setup_perout

        captured_periods = []

        class FakePtp:
            path = "/dev/ptp_i226"

            def set_pin_function(self, pin, func, ch):
                pass

            def enable_perout(self, channel, period_ns=1_000_000_000,
                              start_nsec_override=None):
                captured_periods.append(period_ns)

            def disable_perout(self, channel):
                pass

        with mock.patch("peppar_fix.ptp_device.PTP_PF_PEROUT", 2, create=True):
            ok = setup_perout(
                FakePtp(),
                pin_index=0,
                channel=0,
                period_ns=2_000_000_000,
                program_pin=True,
                ptp_dev_path="/dev/ptp_i226",
                verify_via_ticc_port=None,  # skip the TICC verify retry loop
                max_attempts=1,
            )
        self.assertTrue(ok)
        # At least one enable_perout call with the requested period.
        self.assertIn(2_000_000_000, captured_periods)


class HostConfigMapHasPeroutPeriodNsTest(unittest.TestCase):
    """The host-config map in peppar_fix_engine.py must include
    perout_period_ns so TOML overrides reach argparse.  We can't
    import the engine top-level (it has heavyweight side effects),
    so introspect the source text — same pattern other host-config
    tests use."""

    def test_field_in_engine_host_config_map(self):
        import os
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        engine = os.path.join(here, "peppar_fix_engine.py")
        with open(engine) as f:
            src = f.read()
        self.assertIn('"perout_period_ns":', src,
                      "perout_period_ns missing from engine's TOML→argparse map")


class MadHatConfigSetsPeriodDoublingTest(unittest.TestCase):
    """MadHat's per-host TOML should set the override; other hosts
    (TimeHat, PiFace, clkPoC3) should NOT, to match the documented
    "only MadHat needs this" finding."""

    def _read(self, name):
        import os
        here = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        path = os.path.join(here, "config", name)
        if not os.path.exists(path):
            self.skipTest(f"{path} not present")
        with open(path) as f:
            return f.read()

    def test_madhat_overrides_to_2s(self):
        src = self._read("madhat.toml")
        self.assertIn("perout_period_ns = 2_000_000_000", src,
                      "madhat.toml missing the period override")

    def test_timehat_does_not_override(self):
        src = self._read("timehat.toml")
        # TimeHat is the reference good board; no override needed.
        # Allow a commented mention, but no active assignment.
        for line in src.splitlines():
            stripped = line.strip()
            if stripped.startswith("#"):
                continue
            self.assertFalse(
                stripped.startswith("perout_period_ns"),
                f"timehat.toml unexpectedly overrides perout_period_ns: "
                f"{stripped!r}")


if __name__ == "__main__":
    unittest.main()
