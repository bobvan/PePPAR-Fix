"""Tests for the --tdcp-log plumbing (receiver-clock noise-floor capture).

--tdcp-log writes the read-only TDCP estimator's per-epoch output to a
CSV so the X20-vs-F9T-vs-F9P short-τ TDEV comparison can run on a host
with NO DO and NO TICC (e.g. PiPuss).  The chain is: TOML tdcp_log →
argparse --tdcp-log (default None so TOML can override) → estimator
activation → CSV sink.  The engine has heavyweight import side effects,
so (matching test_perout_period_ns) we introspect the source text.
"""
from __future__ import annotations

import os
import unittest


def _engine_src():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(here, "peppar_fix_engine.py")) as f:
        return f.read()


def _config_src(name):
    here = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    path = os.path.join(here, "config", name)
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return f.read()


class TdcpLogPlumbingTest(unittest.TestCase):

    def test_arg_exists_with_default_none(self):
        # default=None is load-bearing: a non-None argparse default would
        # make _apply_host_config skip the TOML override (see the
        # perout_period_ns lesson).
        src = _engine_src()
        idx = src.find('add_argument("--tdcp-log"')
        self.assertGreater(idx, 0, "--tdcp-log add_argument not found")
        self.assertIn("default=None", src[idx:idx + 400],
                      "--tdcp-log must use default=None so the host-config "
                      "TOML can override it")

    def test_field_in_host_config_map(self):
        self.assertIn('"tdcp_log":', _engine_src(),
                      "tdcp_log missing from engine's TOML→argparse map")

    def test_tdcp_log_activates_estimator(self):
        # The estimator-activation condition must include tdcp_log, so
        # --tdcp-log alone (no --print-tdcp, no servo) runs the estimator.
        src = _engine_src()
        idx = src.find("tdcp_arm5_active =")
        self.assertGreater(idx, 0)
        window = src[idx:idx + 600]
        self.assertIn("tdcp_log", window,
                      "estimator activation must consider tdcp_log so "
                      "--tdcp-log alone enables the read-only TDCP estimator")


class PiPussConfigCollectsTdcpTest(unittest.TestCase):
    """PiPuss (no DO) is the TDCP-collection host: its config sets
    tdcp_log and must NOT configure a DO/servo/TICC."""

    def setUp(self):
        self.src = _config_src("pipuss.toml")
        if self.src is None:
            self.skipTest("config/pipuss.toml not present")

    def test_sets_tdcp_log(self):
        active = [ln.strip() for ln in self.src.splitlines()
                  if ln.strip().startswith("tdcp_log")]
        self.assertTrue(active, "pipuss.toml should set tdcp_log")

    def test_no_do_or_ticc(self):
        for ln in self.src.splitlines():
            s = ln.strip()
            if s.startswith("#"):
                continue
            for forbidden in ("ptp_dev", "do_label", "do_type", "dac_",
                              "ticc_port"):
                self.assertFalse(
                    s.startswith(forbidden),
                    f"pipuss.toml has no DO/TICC but sets {s!r}")


if __name__ == "__main__":
    unittest.main()
