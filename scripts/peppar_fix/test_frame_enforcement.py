"""Frame-discipline enforcement (coordFrameAudit step 5).

Pins the invariants that keep the reference-frame migration from
silently regressing:

  1. ``PositionState.frame`` is a *required* field — no default, so a
     coordinate can never be constructed without declaring its frame
     (design §4.1 "no implicit ITRF").
  2. Readers reject an untagged or unparseable-frame state file rather
     than guessing a default (design §4.3 hard enforcement).
  3. ``load_arp_from_antennas`` returns a *canonical ITRF2020* point —
     guards against reverting the step-4 1.71 m conversion.
  4. A static lint: every ``PositionState(...)`` construction site in
     the modules that build them passes a ``frame=`` keyword.  This
     catches a new construction site that forgets the frame *before*
     runtime, including paths no test exercises.

A fully general "no bare (X, Y, Z) crosses any module boundary" static
lint is deliberately NOT attempted — it is too false-positive-prone to
be useful.  Instead we pin the typed carrier (PositionState requires a
Frame; the boundary type is GeoPoint) and the known constructor sites.
"""
import ast
import dataclasses
import json
import os
import tempfile
import unittest
from pathlib import Path

from peppar_fix.position_state import (
    PositionState, load_arp_from_antennas, load_ppp_state)

_PKG_DIR = Path(__file__).resolve().parent

# Modules that construct PositionState (the engine only *reads* .ecef_m).
_CONSTRUCTOR_MODULES = [
    _PKG_DIR / "position_state.py",
    _PKG_DIR / "peppar_survey_pride.py",
    _PKG_DIR / "peppar_survey_rtklib.py",
]


def _positionstate_calls_without_frame(path: Path):
    """Return [lineno, …] for every ``PositionState(...)`` call in
    ``path`` that lacks a ``frame=`` keyword (and isn't using ``**``
    unpacking, which can't be checked statically)."""
    tree = ast.parse(path.read_text())
    offenders = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = (func.id if isinstance(func, ast.Name)
                else func.attr if isinstance(func, ast.Attribute) else None)
        if name != "PositionState":
            continue
        kw_names = {k.arg for k in node.keywords}
        if None in kw_names:      # **kwargs unpacking — can't verify here
            continue
        if "frame" not in kw_names:
            offenders.append(node.lineno)
    return offenders


class TestFrameRequiredField(unittest.TestCase):
    def test_frame_is_required_no_default(self):
        """PositionState.frame must have no default and no
        default_factory — constructing without it is a TypeError."""
        fields = {f.name: f for f in dataclasses.fields(PositionState)}
        self.assertIn("frame", fields)
        frame_field = fields["frame"]
        self.assertIs(frame_field.default, dataclasses.MISSING,
                      "frame must not have a default (no implicit ITRF)")
        self.assertIs(frame_field.default_factory, dataclasses.MISSING,
                      "frame must not have a default_factory")

    def test_constructing_without_frame_raises(self):
        with self.assertRaises(TypeError):
            PositionState(mount_sn=0, ecef_m=(1.0, 2.0, 3.0),
                          sigma_m=0.01, updated="x", source="y")


class TestReadersRejectUntaggedCoords(unittest.TestCase):
    def test_missing_frame_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "u.ppp.toml"), "w") as f:
                f.write('mount_sn = 0\necef_m = [1.0, 2.0, 3.0]\n'
                        'sigma_m = 0.01\nupdated = "2025-01-01T00:00:00Z"\n'
                        'source = "x"\n')
            self.assertIsNone(load_ppp_state("u", positions_dir=d))

    def test_garbage_frame_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "g.ppp.toml"), "w") as f:
                f.write('mount_sn = 0\necef_m = [1.0, 2.0, 3.0]\n'
                        'sigma_m = 0.01\nupdated = "2025-01-01T00:00:00Z"\n'
                        'source = "x"\nframe = "ZZ99@notanumber"\n')
            self.assertIsNone(load_ppp_state("g", positions_dir=d))


class TestArpLoaderStaysCanonical(unittest.TestCase):
    def test_arp_loader_returns_itrf2020(self):
        """Guards against reverting the step-4 conversion: a NAD83 ARP
        must come back tagged canonical ITRF2020."""
        with tempfile.TemporaryDirectory() as d:
            ap = os.path.join(d, "antennas.json")
            with open(ap, "w") as f:
                json.dump({"a": {"ecef_m": [10.0, 20.0, 30.0],
                                 "sigma_m": 0.01,
                                 "frame": "NAD_83(2011) EPOCH:2010.0000"}}, f)
            s = load_arp_from_antennas("a", mount_sn=0, antennas_path=ap)
            self.assertIsNotNone(s)
            self.assertEqual(s.frame.realization, "ITRF2020")


class TestPositionStateConstructorsTagFrame(unittest.TestCase):
    """Static lint: no PositionState(...) site forgets frame=."""

    def test_all_constructor_sites_pass_frame(self):
        for path in _CONSTRUCTOR_MODULES:
            self.assertTrue(path.exists(), f"missing source file: {path}")
            offenders = _positionstate_calls_without_frame(path)
            self.assertEqual(
                offenders, [],
                f"{path.name}: PositionState(...) without frame= at "
                f"line(s) {offenders} — every coordinate must declare "
                f"its frame (coordFrameAudit).")

    def test_lint_detects_a_missing_frame(self):
        """Sanity-check the lint itself: it flags a frameless call and
        passes a framed one (so a future no-op refactor can't silently
        defang it)."""
        with tempfile.TemporaryDirectory() as d:
            bp, gp = Path(d) / "bad.py", Path(d) / "good.py"
            bp.write_text("PositionState(mount_sn=0, ecef_m=(1, 2, 3))\n")
            gp.write_text("PositionState(ecef_m=(1, 2, 3), frame=f)\n")
            self.assertEqual(_positionstate_calls_without_frame(bp), [1])
            self.assertEqual(_positionstate_calls_without_frame(gp), [])


if __name__ == "__main__":
    unittest.main()
