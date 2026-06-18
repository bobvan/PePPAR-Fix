"""Tests for the engine's _dac_build_kwargs merge/refuse glue (PR #187).

measuredSteeringIsAuthoritative wires measured [steering] into the actuator at
three sites via _dac_build_kwargs, which merges measured params OVER config
(measured authoritative), refuses (SystemExit) a DAC with no measurement, and
falls back to config only for the no-registered-DO case.  The underlying
resolve_dac_actuator_params is unit-tested in test_do_char_resolve; this locks
the engine-side merge/refuse logic (Charlie #187 finding 2 — the integration
risk that was untested).  The resolver is patched so these tests need no
state/dos files or hardware.
"""
from __future__ import annotations

import argparse
import os
import sys
import unittest
from unittest import mock

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix_engine import _dac_build_kwargs  # noqa: E402
from peppar_fix.do_schema import SchemaError  # noqa: E402

# _dac_build_kwargs does `from peppar_fix.do_char_resolve import
# resolve_dac_actuator_params` at call time, so patching the attribute on that
# module is picked up.
_RESOLVE = "peppar_fix.do_char_resolve.resolve_dac_actuator_params"


def _args(**kw):
    base = dict(dac_ppb_per_code=0.0346, dac_center_code=None,
                dac_code_min=None, dac_code_max=None, dac_gain=0)
    base.update(kw)
    return argparse.Namespace(**base)


class TestDacBuildKwargs(unittest.TestCase):

    def test_measured_overrides_config_and_fills_range(self):
        measured = {"ppb_per_code": 0.025, "center_code": 32768,
                    "code_min": 16000, "code_max": 50000, "dac_gain": 1}
        with mock.patch(_RESOLVE, return_value=measured):
            out = _dac_build_kwargs(_args(), "ocxo-x")
        self.assertEqual(out["ppb_per_code"], 0.025)   # measured wins over 0.0346
        self.assertEqual(out["center_code"], 32768)
        self.assertEqual(out["code_min"], 16000)
        self.assertEqual(out["code_max"], 50000)
        self.assertEqual(out["dac_gain"], 1)

    def test_measured_without_gain_keeps_config_gain(self):
        # Back-compat: [steering] lacking dac_gain → config gain survives the
        # merge (resolve omits the key; engine must not clobber it to None).
        measured = {"ppb_per_code": 0.025}
        with mock.patch(_RESOLVE, return_value=measured):
            out = _dac_build_kwargs(_args(dac_gain=1), "ocxo-x")
        self.assertEqual(out["ppb_per_code"], 0.025)
        self.assertEqual(out["dac_gain"], 1)           # from config, not clobbered

    def test_refuse_raises_systemexit(self):
        # A registered DAC with no measured steering → resolve raises
        # SchemaError → _dac_build_kwargs converts to a clean SystemExit
        # (no uncaught exception, no default-slope fallback).
        with mock.patch(_RESOLVE, side_effect=SchemaError("not measured")):
            with self.assertRaises(SystemExit):
                _dac_build_kwargs(_args(), "ocxo-x")

    def test_no_registered_do_uses_config(self):
        # resolve returns None (no-DO / intrinsic actuator) → config path,
        # unchanged.
        with mock.patch(_RESOLVE, return_value=None):
            out = _dac_build_kwargs(
                _args(dac_ppb_per_code=0.0346, dac_code_min=100,
                      dac_code_max=900), None)
        self.assertEqual(out["ppb_per_code"], 0.0346)
        self.assertEqual(out["code_min"], 100)
        self.assertEqual(out["code_max"], 900)


if __name__ == "__main__":
    unittest.main()
