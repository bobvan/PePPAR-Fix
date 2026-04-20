"""Unit tests for the phase-bias-availability gate in NarrowLaneResolver.

When the active SSR stream(s) lack matched phase biases for both signals
of an SV's IF combination, that SV must not become an integer-fix
candidate — its float IF ambiguity carries an unknown bias and any
"integer" fix lands on a biased value.  This was the root of ptpmon's
2026-04-20 overnight altitude drift: GPS SVs (tracked as L2W; CNES
publishes L2L) were quietly entering LAMBDA batches anchored by GAL
and being accepted under the relaxed P_bootstrap=0.97 threshold.

The gate: `ar_phase_bias_ok` kwarg on `NarrowLaneResolver.attempt()`.
None → gate disabled (legacy).  dict → per-SV bool; False excludes from
candidacy with `RESULT_SKIP_NO_PHASE_BIAS`.
"""

from __future__ import annotations

import logging
import os
import sys
import unittest

import numpy as np

# ppp_ar lives in scripts/ (not scripts/peppar_fix/) — add it to the path
# so the test can import it directly.  Matches the convention used by
# other peppar_fix tests that reach into scripts/ modules.
_SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from ppp_ar import NarrowLaneResolver  # noqa: E402
from solve_ppp import N_BASE           # noqa: E402

from peppar_fix.nl_diag import (       # noqa: E402
    NlDiagLogger,
    RESULT_SKIP_NO_PHASE_BIAS,
)


class _FakeMwTracker:
    """Minimal MW tracker stub for NarrowLaneResolver.attempt()."""

    def __init__(self, wl_by_sv, freqs_by_sv):
        self._wl = dict(wl_by_sv)
        self._freqs = dict(freqs_by_sv)
        self.n_fixed = len(self._wl)

    def get_wl(self, sv):
        return self._wl.get(sv)

    def get_freqs(self, sv):
        return self._freqs.get(sv)


class _FakeFilter:
    """Minimal PPPFilter stub with sv_to_idx, x, P."""

    def __init__(self, sv_to_idx):
        self.sv_to_idx = dict(sv_to_idx)
        # State vector: N_BASE + N_ambiguities.  Initialize with
        # deliberately non-integer IF ambiguities so the pre-screen
        # gate would admit them if the phase-bias gate let them
        # through.
        n_amb = len(sv_to_idx)
        self.x = np.zeros(N_BASE + n_amb)
        for sv, idx in self.sv_to_idx.items():
            self.x[N_BASE + idx] = 100.0  # arbitrary IF ambiguity, meters
        self.P = np.eye(N_BASE + n_amb) * 1e-6


class _CaptureHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.messages: list[str] = []

    def emit(self, record):
        self.messages.append(self.format(record))


class PhaseBiasGateTest(unittest.TestCase):
    """The gate excludes SVs lacking phase biases; others pass through."""

    # Frequencies for GPS L1/L2 (Hz) — exact values don't matter for the
    # gate test, but need to be realistic so lambda_WL/lambda_NL are
    # sensible.
    F1_GPS = 1575.42e6
    F2_GPS = 1227.60e6
    # GAL L1/L5
    F1_GAL = 1575.42e6
    F5_GAL = 1176.45e6

    def setUp(self):
        self.handler = _CaptureHandler()
        self.logger = logging.getLogger("peppar_fix.nl_diag")
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)
        self.diag = NlDiagLogger(enabled=True)

    def tearDown(self):
        self.logger.removeHandler(self.handler)

    def _make_stack(self, svs):
        """Build matched filter + mw_tracker for a list of SVs, assigning
        plausible GPS or GAL frequencies based on the SV prefix."""
        sv_to_idx = {sv: i for i, sv in enumerate(svs)}
        wl_by_sv = {sv: 0 for sv in svs}  # WL integer 0 is fine for the gate test
        freqs_by_sv = {}
        for sv in svs:
            if sv.startswith('G'):
                freqs_by_sv[sv] = (self.F1_GPS, self.F2_GPS)
            elif sv.startswith('E'):
                freqs_by_sv[sv] = (self.F1_GAL, self.F5_GAL)
            else:
                raise ValueError(f"unhandled prefix: {sv!r}")
        filt = _FakeFilter(sv_to_idx)
        mw = _FakeMwTracker(wl_by_sv, freqs_by_sv)
        return filt, mw

    def _resolver(self):
        return NarrowLaneResolver(
            ar_elev_mask_deg=0.0,  # disable elev gate in tests
            nl_diag=self.diag,
        )

    def _skip_no_phase_bias_svs(self):
        """Return the set of SVs the diag logger reported as
        SKIP_NO_PHASE_BIAS in the most recent attempt."""
        result = set()
        for line in self.handler.messages:
            if f"result={RESULT_SKIP_NO_PHASE_BIAS}" in line:
                for token in line.split():
                    if token.startswith("sv="):
                        result.add(token.split("=", 1)[1])
        return result

    # ── Core behavior ────────────────────────────────────────────── #

    def test_gate_excludes_flagged_sv(self):
        """ar_phase_bias_ok[G01] = False → G01 is skipped with
        RESULT_SKIP_NO_PHASE_BIAS; E05 (True) is not."""
        filt, mw = self._make_stack(["G01", "E05"])
        resolver = self._resolver()
        self.diag.begin(epoch=1)
        resolver.attempt(filt, mw,
                         elevations={"G01": 60.0, "E05": 60.0},
                         ar_phase_bias_ok={"G01": False, "E05": True})
        self.diag.emit()
        skipped = self._skip_no_phase_bias_svs()
        self.assertEqual(skipped, {"G01"},
                         "only G01 should be skipped for missing bias")

    def test_gate_disabled_when_kwarg_is_none(self):
        """ar_phase_bias_ok=None → legacy path; no SV is skipped for
        bias reasons even if it would be under a dict."""
        filt, mw = self._make_stack(["G01", "E05"])
        resolver = self._resolver()
        self.diag.begin(epoch=2)
        resolver.attempt(filt, mw,
                         elevations={"G01": 60.0, "E05": 60.0},
                         ar_phase_bias_ok=None)
        self.diag.emit()
        self.assertEqual(self._skip_no_phase_bias_svs(), set(),
                         "None must bypass the gate entirely")

    def test_missing_sv_defaults_to_admit(self):
        """An SV not present in the dict is admitted (default True).
        This covers the boundary where an observation arrived late or
        the flag couldn't be computed — we don't want a missing key
        to silently exclude a legitimate candidate."""
        filt, mw = self._make_stack(["G01", "E05"])
        resolver = self._resolver()
        self.diag.begin(epoch=3)
        # Only E05 has a flag; G01 missing → treated as True.
        resolver.attempt(filt, mw,
                         elevations={"G01": 60.0, "E05": 60.0},
                         ar_phase_bias_ok={"E05": True})
        self.diag.emit()
        self.assertEqual(self._skip_no_phase_bias_svs(), set(),
                         "missing key must default to admit")

    def test_ptpmon_profile_excludes_gps_keeps_gal(self):
        """End-to-end scenario matching ptpmon 2026-04-20 configuration.

        F9T-L2 hardware tracks L2W for GPS; CNES publishes L2L phase
        biases; no match for GPS.  GAL uses L1C + L7Q, both matched by
        CNES.  The flag captures this: all GPS flagged False, GAL True.
        """
        filt, mw = self._make_stack(["G10", "G23", "G28", "E05", "E36"])
        resolver = self._resolver()
        elev = {sv: 45.0 for sv in ("G10", "G23", "G28", "E05", "E36")}
        # ptpmon reality: GPS never passes, GAL always passes
        ar_flag = {
            "G10": False, "G23": False, "G28": False,
            "E05": True,  "E36": True,
        }
        self.diag.begin(epoch=4)
        resolver.attempt(filt, mw, elevations=elev,
                         ar_phase_bias_ok=ar_flag)
        self.diag.emit()
        skipped = self._skip_no_phase_bias_svs()
        self.assertEqual(skipped, {"G10", "G23", "G28"},
                         "ptpmon L2 profile must exclude all GPS SVs")
        # And GAL SVs must not be skipped by this gate (they may still
        # fail later gates like pre-screen, but not this one).
        for sv in ("E05", "E36"):
            self.assertNotIn(sv, skipped)


if __name__ == "__main__":
    unittest.main()
