#!/usr/bin/env python3
"""Regression test for disciplineModeFsm ctx threading.

PRs #92/#95/#103/#107 introduced ``_convergence`` (DisciplineConvergence
signal) and ``_binary_layer`` (gross-fault detector) as local variables
in ``_setup_servo`` AND added per-epoch references to them in
``_servo_epoch``, but did NOT thread them through the returned
``servo_ctx`` dict.  Bare-name references in ``_servo_epoch`` raised

    NameError: name '_binary_layer' is not defined

on the first servo epoch any time ``--graded-taper`` was on
(lab repro 2026-05-30 PiFace + MadHat — engine crashes within seconds
of reaching Phase 2 steady state).  Unit tests covered the new classes
in isolation; sim A/Bs covered DOFreqEst-level wiring; neither
exercised the engine's per-epoch retrieval — gap that PR #107 review
explicitly flagged as ``NO ENGINE-LEVEL E2E TEST``.

Fix: ``_setup_servo`` adds both objects to the returned ctx dict;
``_servo_epoch`` retrieves them via ``ctx.get`` (matching the older
``_ho = ctx.get('holdover_state')`` pattern five lines above).

This test pins the ctx contract textually so a future refactor can't
silently drop either key.  A full engine-level e2e test (start engine,
run a servo epoch under ``--graded-taper``, assert no exception)
remains the longer-term ask flagged in the #107 review.
"""

import unittest
from pathlib import Path

_ENGINE = (
    Path(__file__).resolve().parent.parent
    / "scripts" / "peppar_fix_engine.py"
)


class DisciplineModeFsmCtxThreadingTests(unittest.TestCase):
    """Pin the ctx contract for the disciplineModeFsm objects."""

    @classmethod
    def setUpClass(cls):
        cls.src = _ENGINE.read_text()

    def _slice_function(self, name):
        """Extract a function body (best-effort textual)."""
        # split on ``def <name>`` then take everything up to the next
        # top-level ``def `` (column-0 def).  Good enough for textual
        # contract checks; not a parser.
        after = self.src.split(f"def {name}", 1)[1]
        # next top-level def starts at column 0 — first occurrence of
        # ``\ndef `` (newline + def + space).
        next_def_idx = after.find("\ndef ")
        return after[:next_def_idx] if next_def_idx != -1 else after

    def test_setup_servo_return_includes_convergence(self):
        body = self._slice_function("_setup_servo")
        self.assertIn(
            "'convergence': _convergence", body,
            "_setup_servo's return dict must include 'convergence' "
            "(else _servo_epoch raises NameError on --graded-taper)")

    def test_setup_servo_return_includes_binary_layer(self):
        body = self._slice_function("_setup_servo")
        self.assertIn(
            "'binary_layer': _binary_layer", body,
            "_setup_servo's return dict must include 'binary_layer' "
            "(else _servo_epoch raises NameError on --graded-taper)")

    def test_servo_epoch_retrieves_convergence_from_ctx(self):
        body = self._slice_function("_servo_epoch")
        self.assertIn(
            "ctx.get('convergence')", body,
            "_servo_epoch must read _convergence via ctx.get; bare-name "
            "reference raises NameError")

    def test_servo_epoch_retrieves_binary_layer_from_ctx(self):
        body = self._slice_function("_servo_epoch")
        self.assertIn(
            "ctx.get('binary_layer')", body,
            "_servo_epoch must read _binary_layer via ctx.get; bare-name "
            "reference raises NameError")


if __name__ == "__main__":
    unittest.main()
