"""Guard: the shipped madhat-sxtd.toml actually turns the servo fixes ON.

Bravo's PR #303 review (MAJOR 2) caught that the headline knob-2 fix (wno) was
NOT wired into the config — clock_model was absent from the host-config
allowlist and the arg defaulted to the cascaded random_walk, so running via
`--host-config config/madhat-sxtd.toml` silently got the UNFIXED behaviour.
This test locks the wiring: loading the shipped config must enable wno + the
knob-1 gate + the clamp.  It relies on the argparse defaults being None so the
host-config _MAP can fill them (the mechanism that was broken).
"""
import argparse
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SCRIPTS = os.path.join(_ROOT, "scripts")
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import peppar_fix_engine as E

_CFG = os.path.join(_ROOT, "config", "madhat-sxtd.toml")


def _apply():
    ns = argparse.Namespace(host_config=_CFG)
    # Seed the servo knobs at their CLI "unset" default (None) so the
    # host-config _MAP fills them — the exact path MAJOR 2 was about.
    for d in ("fixedpos_clock_model", "innov_gate_nsigma", "do_freq_clamp_ppb",
              "lqr_phase_gain", "gnssdo_transport", "gnssdo_serial",
              "gnssdo_ppb_per_controlword", "gnssdo_watchdog_s",
              "ptp_profile", "do_label", "do_type"):
        setattr(ns, d, None)
    E._apply_host_config(ns)
    return ns


def test_config_enables_wno():
    assert _apply().fixedpos_clock_model == "wno"


def test_config_enables_knob1_gate():
    assert _apply().innov_gate_nsigma == 5.0


def test_config_sets_do_freq_clamp():
    assert _apply().do_freq_clamp_ppb == 30.0


def test_config_carries_calibrated_gain_sign():
    # The sign-confirmed control-word gain must be present (positive here).
    g = _apply().gnssdo_ppb_per_controlword
    assert g is not None and g > 0


# NOTE: test_config_enables_wno also guards the arg default indirectly — the
# host-config _MAP only fills args still at None, so if --fixedpos-clock-model
# ever regains a non-None default (the MAJOR-2 trap), wno would stop applying
# and that test would fail.


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
