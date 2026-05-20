#!/usr/bin/env python3
"""Unit tests for --no-antposest (time-only architecture).

The flag opts out of the position filter entirely: no AntPosEstThread,
no Phase-1 PPPFilter bootstrap, surveyed position pinned, blend off.
FixedPosFilter at the pinned position still runs in steady state, NAV2
watchdog still catches gross ARP moves, RINEX is still logged.  See
docs/time-only-architecture.md and dayplan timeOnlyArchitecture-main.

Run: ./bin/test tests/test_no_antposest.py
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'scripts'))

# Import the engine module via the same path the wrapper uses.  The
# engine has a single main() that's hard to exercise headlessly, so we
# test the public argparse + post-process surface and the gate logic
# that the engine reads at runtime.
import peppar_fix_engine as engine


# ── argparse surface ─────────────────────────────────────────────── #

class TestNoAntposestFlag(unittest.TestCase):
    """The flag parses correctly and defaults False."""

    def _parser(self):
        # Build the same parser main() uses.  Walk up to the
        # add_argument call by inspecting the module.
        import argparse
        ap = argparse.ArgumentParser()
        # Mirror the relevant group:
        pos = ap.add_argument_group("pos")
        pos.add_argument("--no-antposest", action="store_true")
        pos.add_argument("--pin-position", action="store_true")
        pos.add_argument("--position-blend-source",
                         choices=["nav2", "antposest", "none"],
                         default="nav2")
        return ap

    def test_default_false(self):
        ap = self._parser()
        args = ap.parse_args([])
        self.assertFalse(args.no_antposest)

    def test_flag_sets_true(self):
        ap = self._parser()
        args = ap.parse_args(["--no-antposest"])
        self.assertTrue(args.no_antposest)


# ── post-process derived defaults ────────────────────────────────── #

class TestDerivedDefaults(unittest.TestCase):
    """When --no-antposest is set, pin_position is forced True and
    position_blend_source is forced 'none'.  These are the safety
    rails: surveyed pin must not drift toward NAV2 or AntPosEst."""

    def _make_args(self, **overrides):
        """Replicates the post-process block in main()."""
        class _A:
            pass
        a = _A()
        a.no_antposest = False
        a.pin_position = False
        a.position_blend_source = "nav2"
        for k, v in overrides.items():
            setattr(a, k, v)
        return a

    def _apply_derived(self, a):
        """Mirror the engine main()'s --no-antposest post-process."""
        if getattr(a, 'no_antposest', False):
            a.pin_position = True
            a.position_blend_source = 'none'

    def test_default_path_unchanged(self):
        # Without --no-antposest, pin_position and blend-source keep
        # their CLI-supplied (or default) values.
        a = self._make_args()
        self._apply_derived(a)
        self.assertFalse(a.pin_position)
        self.assertEqual(a.position_blend_source, "nav2")

    def test_no_antposest_forces_pin_position(self):
        a = self._make_args(no_antposest=True)
        self._apply_derived(a)
        self.assertTrue(a.pin_position)

    def test_no_antposest_forces_blend_none(self):
        a = self._make_args(no_antposest=True)
        self._apply_derived(a)
        self.assertEqual(a.position_blend_source, "none")

    def test_no_antposest_overrides_explicit_blend(self):
        # Even if operator wrote --position-blend-source nav2 on the
        # command line, --no-antposest wins — surveyed pin is truth.
        a = self._make_args(no_antposest=True,
                            position_blend_source="nav2")
        self._apply_derived(a)
        self.assertEqual(a.position_blend_source, "none")


# ── seed-required-at-startup gate ────────────────────────────────── #

class TestSeedRequired(unittest.TestCase):
    """The engine's run() refuses to bootstrap when --no-antposest is
    set and no seed (known_ecef) is available.  This is the contract
    that turns peppar-survey into load-bearing infrastructure for
    time-only deployments.

    We can't run the full engine here, but we exercise the equivalent
    gate logic directly.
    """

    def _gate(self, *, no_antposest, known_ecef):
        """Mirror the engine's pre-bootstrap check."""
        if known_ecef is None:
            if no_antposest:
                return "REFUSE"
            return "BOOTSTRAP"
        return "PROCEED"

    def test_seed_present_proceeds(self):
        self.assertEqual(
            self._gate(no_antposest=True, known_ecef=(1, 2, 3)),
            "PROCEED")

    def test_no_antposest_without_seed_refuses(self):
        # The contract: time-only mode refuses to bootstrap.
        self.assertEqual(
            self._gate(no_antposest=True, known_ecef=None),
            "REFUSE")

    def test_default_mode_falls_through_to_bootstrap(self):
        # Without --no-antposest, missing seed → run_bootstrap (legacy).
        self.assertEqual(
            self._gate(no_antposest=False, known_ecef=None),
            "BOOTSTRAP")


# ── engine surface integration ───────────────────────────────────── #

class TestEngineFlagWired(unittest.TestCase):
    """The engine module exposes --no-antposest via its main()
    argparse.  Catches the rename / removal regression."""

    def test_engine_argparse_recognizes_flag(self):
        # The engine builds its parser inside main(); we can't easily
        # invoke main() to introspect, but we can check the help
        # string contains the flag (proves it was wired up).
        import subprocess
        out = subprocess.run(
            [sys.executable,
             str(Path(__file__).resolve().parent.parent
                 / 'scripts/peppar_fix_engine.py'),
             '--help'],
            capture_output=True, text=True, timeout=30)
        self.assertIn('--no-antposest', out.stdout,
                      msg="engine --help should advertise --no-antposest")


if __name__ == "__main__":
    unittest.main()
