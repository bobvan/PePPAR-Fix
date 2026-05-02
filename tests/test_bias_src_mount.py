"""Tests for src_mount tagging on BiasCorrection (I-122350-main P1).

Validates the plumbing that lets per-(SV, signal) applied bias values
be attributed to the NTRIP mount that delivered them.  Used by the
[CB_APPLIED] / [PB_APPLIED] log lines for the WHU dual-mount stepwise
bringup diagnostic (I-003751-main).

Three assertions:
  1. BiasCorrection accepts and stores src_mount.
  2. SSRState plumbs src_mount through update_from_rtcm to the
     parse / store path for code biases.
  3. Last-write-wins: two writes to the same (sv, signal) from
     different src_mounts end with the second one's src_mount —
     this is the "WHU overwrites CNES" case the bringup will see
     in the dual-mount config.
"""

from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from ssr_corrections import BiasCorrection, SSRState  # noqa: E402


class BiasCorrectionSrcMountTest(unittest.TestCase):
    """Direct unit test of the dataclass."""

    def test_default_src_mount_is_none(self):
        bc = BiasCorrection(signal_code='C1C', bias_m=0.123)
        self.assertIsNone(bc.src_mount)

    def test_src_mount_stored(self):
        bc = BiasCorrection(signal_code='C1C', bias_m=0.123,
                            src_mount='SSR')
        self.assertEqual(bc.src_mount, 'SSR')

    def test_src_mount_in_slots(self):
        """__slots__ must include src_mount or attribute set will
        raise AttributeError."""
        bc = BiasCorrection(signal_code='C1C', bias_m=0.0)
        bc.src_mount = 'SSR-BIAS'  # would raise on missing slot
        self.assertEqual(bc.src_mount, 'SSR-BIAS')


class _FakeRtcmCodeBiasMsg:
    """Minimal fake of a pyrtcm RTCM 1059 (GPS code bias) message
    for SSRState._parse_code_bias.  Standard RTCM SSR uses DF-prefixed
    fields per the RTCM 3.x tables."""

    identity = '1059'

    def __init__(self, prn_int=1, sig_id=0, bias_m=0.123):
        # Header
        self.DF385 = 0       # epoch
        self.DF413 = 0       # IOD SSR
        self.DF387 = 1       # n_sats
        # Per-sat (i=01)
        setattr(self, 'DF068_01', prn_int)        # GPS PRN
        setattr(self, 'DF379_01', 1)              # n_biases
        setattr(self, 'DF380_01_01', sig_id)      # signal ID (0 = L1CA → C1C)
        setattr(self, 'DF383_01_01', bias_m)      # bias (m)
        self.recv_mono = 0.0


class SSRStatePlumbingTest(unittest.TestCase):
    """Verify update_from_rtcm passes src_mount through to the stored
    BiasCorrection."""

    def test_code_bias_carries_src_mount(self):
        ssr = SSRState()
        msg = _FakeRtcmCodeBiasMsg(prn_int=1, sig_id=0, bias_m=0.456)
        result = ssr.update_from_rtcm(msg, src_mount='SSR')
        self.assertEqual(result, 'code_bias')
        bc = ssr._code_bias.get('G01', {}).get('C1C')
        self.assertIsNotNone(bc, "code bias not stored")
        self.assertEqual(bc.src_mount, 'SSR')
        self.assertAlmostEqual(bc.bias_m, 0.456, places=4)

    def test_default_src_mount_when_not_passed(self):
        """Backwards-compat: callers that don't supply src_mount get
        None, not an exception."""
        ssr = SSRState()
        msg = _FakeRtcmCodeBiasMsg(prn_int=1, sig_id=0, bias_m=0.1)
        result = ssr.update_from_rtcm(msg)  # no src_mount kw
        self.assertEqual(result, 'code_bias')
        bc = ssr._code_bias.get('G01', {}).get('C1C')
        self.assertIsNotNone(bc)
        self.assertIsNone(bc.src_mount)

    def test_last_write_wins_dual_mount(self):
        """The dual-mount bug we're hunting: CNES and WHU both publish
        a code bias for the same (sv, signal); the later parse
        overwrites the earlier one and src_mount records the winner.
        For the bringup diagnostic this is exactly the signal we want
        to expose in [CB_APPLIED] — when a value flips, we know which
        mount caused it."""
        ssr = SSRState()
        # First write from CNES OC mount
        msg_cnes = _FakeRtcmCodeBiasMsg(prn_int=1, sig_id=0, bias_m=0.10)
        ssr.update_from_rtcm(msg_cnes, src_mount='SSR')
        bc = ssr._code_bias['G01']['C1C']
        self.assertEqual(bc.src_mount, 'SSR')
        self.assertAlmostEqual(bc.bias_m, 0.10, places=4)
        # Second write from WHU bias mount (different value)
        msg_whu = _FakeRtcmCodeBiasMsg(prn_int=1, sig_id=0, bias_m=0.20)
        ssr.update_from_rtcm(msg_whu, src_mount='SSR-BIAS')
        bc = ssr._code_bias['G01']['C1C']
        # Last write wins
        self.assertEqual(bc.src_mount, 'SSR-BIAS')
        self.assertAlmostEqual(bc.bias_m, 0.20, places=4)


if __name__ == '__main__':
    unittest.main()
