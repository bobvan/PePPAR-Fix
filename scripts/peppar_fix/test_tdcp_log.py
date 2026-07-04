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


# NOTE: the former PiPussConfigCollectsTdcpTest (asserting pipuss.toml sets
# tdcp_log and has no DO/servo/TICC) was retired 2026-07-04: PiPuss was
# recommissioned from the TDCP-collection host to a full F9T-20B + IsoTemp
# clock host (do_label=ocxo-clkpoc3, DAC + TICC #5), so its config no longer
# sets tdcp_log. The config is current; the test encoded the old role. The
# generic --tdcp-log feature tests above remain the coverage. (I-024907)


if __name__ == "__main__":
    unittest.main()
