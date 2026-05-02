"""Tests for the dual-mount gap-fill filter (I-122350-main P3 fix).

Validates that ``SSRState.update_from_rtcm(..., gap_fill_only=True)``
restricts bias writes to the ``GAP_FILL_SIGNALS`` allow-list.  This
is what makes the dual-mount sound: with the secondary stream
filtered to gap signals only, the bias cache becomes single-source-
per-(SV, signal) by construction, and the cross-AC datum mismatch
documented in the 2026-05-02 twin-WHU run can't poison shared-signal
ambiguities.

See ``docs/ac-datum-mixing.md`` for the framing.
"""

from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from ssr_corrections import GAP_FILL_SIGNALS, SSRState  # noqa: E402


class _FakeRtcmCodeBiasMsg:
    """Standard RTCM SSR (RTCM 1059 = GPS, 1065 = GLONASS, 1242 = GAL,
    1260 = BDS) — one satellite, one bias.  Identity-and-fields
    pattern matches the existing test_bias_src_mount fixture."""

    def __init__(self, identity, sat_field, prn_int, sig_id, bias_m):
        self.identity = identity
        self.DF385 = 0
        self.DF413 = 0
        self.DF387 = 1
        # Per-constellation sat_id field (DF068=GPS, DF384=GLO,
        # DF252=GAL, DF488=BDS).
        setattr(self, f'{sat_field}_01', prn_int)
        setattr(self, 'DF379_01', 1)
        setattr(self, 'DF380_01_01', sig_id)
        setattr(self, 'DF383_01_01', bias_m)
        self.recv_mono = 0.0


class GapFillSignalsListTest(unittest.TestCase):
    """The static list itself."""

    def test_gap_fill_is_frozenset(self):
        """Read-only by construction so a stray runtime mutation can't
        silently broaden the gap-fill scope."""
        self.assertIsInstance(GAP_FILL_SIGNALS, frozenset)

    def test_gps_l5_in_gap_fill(self):
        """GPS L5Q is the canonical example — CNES SSRA00CNE0 omits L5
        phase bias; this gap is what the dual-mount design is
        ultimately for."""
        self.assertIn(('G', 'C5Q'), GAP_FILL_SIGNALS)
        self.assertIn(('G', 'L5Q'), GAP_FILL_SIGNALS)

    def test_bds_b2a_in_gap_fill(self):
        """BDS B2a-I (1176.45 MHz, RINEX band 5) — CNES covers B1I+B3I
        only; this is the F9T-20B's gap signal."""
        self.assertIn(('C', 'C5X'), GAP_FILL_SIGNALS)
        self.assertIn(('C', 'L5X'), GAP_FILL_SIGNALS)

    def test_glonass_in_gap_fill(self):
        """CNES does not publish GLO biases at all."""
        self.assertIn(('R', 'C1C'), GAP_FILL_SIGNALS)
        self.assertIn(('R', 'L1C'), GAP_FILL_SIGNALS)
        self.assertIn(('R', 'C2C'), GAP_FILL_SIGNALS)
        self.assertIn(('R', 'L2C'), GAP_FILL_SIGNALS)

    def test_shared_signals_not_in_gap_fill(self):
        """The whole point: signals CNES already covers must NOT be
        in the allow-list, or secondary-mount writes will collide
        on shared keys and re-introduce the merge bug."""
        # GPS L1 C/A — CNES covers, must not be gap-fill.
        self.assertNotIn(('G', 'C1C'), GAP_FILL_SIGNALS)
        self.assertNotIn(('G', 'L1C'), GAP_FILL_SIGNALS)
        # GPS L2C — CNES covers (L2W / L2L variants).
        self.assertNotIn(('G', 'C2X'), GAP_FILL_SIGNALS)
        self.assertNotIn(('G', 'L2X'), GAP_FILL_SIGNALS)
        # GAL E1, E5a — CNES covers fully.
        self.assertNotIn(('E', 'C1C'), GAP_FILL_SIGNALS)
        self.assertNotIn(('E', 'L1C'), GAP_FILL_SIGNALS)
        self.assertNotIn(('E', 'C5Q'), GAP_FILL_SIGNALS)
        self.assertNotIn(('E', 'L5Q'), GAP_FILL_SIGNALS)
        # BDS B1I — CNES covers.
        self.assertNotIn(('C', 'C2I'), GAP_FILL_SIGNALS)
        self.assertNotIn(('C', 'L2I'), GAP_FILL_SIGNALS)


class GapFillFilterCodeBiasTest(unittest.TestCase):
    """The filter behaviour on code biases."""

    def test_shared_signal_blocked_when_gap_fill_only(self):
        """GPS L1 C/A is shared — secondary mount must NOT write."""
        ssr = SSRState()
        msg = _FakeRtcmCodeBiasMsg(
            identity='1059', sat_field='DF068',
            prn_int=1, sig_id=0,  # sig_id=0 → L1CA → C1C
            bias_m=0.456)
        ssr.update_from_rtcm(msg, src_mount='SSR-BIAS', gap_fill_only=True)
        self.assertNotIn('C1C', ssr._code_bias.get('G01', {}),
                         "shared signal C1C must not be stored from "
                         "secondary mount when gap_fill_only=True")

    def test_gap_signal_allowed_when_gap_fill_only(self):
        """GPS L5Q IS gap-fill — secondary mount writes through."""
        ssr = SSRState()
        # RTCM 1059 GPS sig_id 14 maps to L5I (per RTCM 3.x table); the
        # closest gap-fill we can hit via the standard sig_id table is
        # to use IGS SSR sig_id mapping.  Use the GAL message instead —
        # RTCM 1242 sig_id table differs.  Simpler: directly inject a
        # phase-bias path via the binary parser, which uses the
        # _RTCM_SSR_SIGNAL_MAP we control via signal-id semantics.
        # Easiest: call _store_phase_bias directly — that's the unit
        # under test for the gap-fill filter.
        from ssr_corrections import _SSR_SIGNAL_MAP
        # Pick a sig_id that maps to L5Q in the IGS map for GPS.
        # IGS map keys: ('G', N) -> 'L5Q' for some N.  Find one.
        l5q_sig_ids = [
            sig_id for (sys_p, sig_id), code
            in _SSR_SIGNAL_MAP.items()
            if sys_p == 'G' and code == 'L5Q']
        self.assertTrue(l5q_sig_ids, "expected L5Q in _SSR_SIGNAL_MAP for GPS")
        sig_id = l5q_sig_ids[0]
        n = ssr._store_phase_bias(
            'G15', 'G', sig_id, 0.123, _SSR_SIGNAL_MAP,
            src_mount='SSR-BIAS', gap_fill_only=True)
        self.assertEqual(n, 1)
        self.assertIn('L5Q', ssr._phase_bias.get('G15', {}))

    def test_default_path_unchanged(self):
        """Without gap_fill_only, every signal is written (back-compat
        with the primary-mount writer)."""
        ssr = SSRState()
        msg = _FakeRtcmCodeBiasMsg(
            identity='1059', sat_field='DF068',
            prn_int=1, sig_id=0,  # GPS L1 C/A — shared signal
            bias_m=0.456)
        ssr.update_from_rtcm(msg, src_mount='SSR', gap_fill_only=False)
        self.assertIn('C1C', ssr._code_bias.get('G01', {}),
                      "primary-mount writes must not be filtered")


class GapFillFilterPhaseBiasTest(unittest.TestCase):
    """The filter behaviour on phase biases via _store_phase_bias —
    the unit consumes the same allow-list."""

    def test_gap_signal_stored(self):
        ssr = SSRState()
        from ssr_corrections import _SSR_SIGNAL_MAP
        l5q = next(
            sig_id for (sys_p, sig_id), code in _SSR_SIGNAL_MAP.items()
            if sys_p == 'G' and code == 'L5Q')
        n = ssr._store_phase_bias(
            'G15', 'G', l5q, 0.05, _SSR_SIGNAL_MAP,
            src_mount='SSR-BIAS', gap_fill_only=True)
        self.assertEqual(n, 1)
        self.assertIn('L5Q', ssr._phase_bias['G15'])

    def test_shared_signal_dropped(self):
        ssr = SSRState()
        from ssr_corrections import _SSR_SIGNAL_MAP
        l1c = next(
            sig_id for (sys_p, sig_id), code in _SSR_SIGNAL_MAP.items()
            if sys_p == 'G' and code == 'L1C')
        n = ssr._store_phase_bias(
            'G15', 'G', l1c, 0.05, _SSR_SIGNAL_MAP,
            src_mount='SSR-BIAS', gap_fill_only=True)
        self.assertEqual(n, 0, "shared L1C must be dropped under gap_fill_only")
        self.assertNotIn('L1C', ssr._phase_bias.get('G15', {}))

    def test_unmapped_signal_returns_zero_regardless(self):
        """If the sig_id is unmapped, gap_fill_only doesn't matter —
        return 0 either way (no rinex_code to check)."""
        ssr = SSRState()
        from ssr_corrections import _SSR_SIGNAL_MAP
        n = ssr._store_phase_bias(
            'G15', 'G', 999, 0.05, _SSR_SIGNAL_MAP,
            src_mount='SSR-BIAS', gap_fill_only=True)
        self.assertEqual(n, 0)


class DualMountMergeProtectionTest(unittest.TestCase):
    """The end-to-end protection: with gap_fill_only=True on the
    secondary mount, the cache holds only one source per shared
    (SV, signal) — no last-write-wins flip possible."""

    def test_primary_writes_then_secondary_blocked_on_shared(self):
        ssr = SSRState()
        # Primary mount writes shared L1 C/A bias (real path; no filter).
        msg_primary = _FakeRtcmCodeBiasMsg(
            identity='1059', sat_field='DF068',
            prn_int=1, sig_id=0, bias_m=0.10)
        ssr.update_from_rtcm(
            msg_primary, src_mount='SSR', gap_fill_only=False)
        bc = ssr._code_bias['G01']['C1C']
        self.assertEqual(bc.src_mount, 'SSR')
        self.assertAlmostEqual(bc.bias_m, 0.10, places=4)
        # Secondary mount tries to write the same shared signal — must
        # be blocked by gap_fill_only filter.
        msg_secondary = _FakeRtcmCodeBiasMsg(
            identity='1059', sat_field='DF068',
            prn_int=1, sig_id=0, bias_m=0.20)
        ssr.update_from_rtcm(
            msg_secondary, src_mount='SSR-BIAS', gap_fill_only=True)
        bc = ssr._code_bias['G01']['C1C']
        # Cache STILL holds primary's value — secondary write was
        # filtered.  This is the merge-bug protection.
        self.assertEqual(bc.src_mount, 'SSR',
                         "secondary mount overwrote shared signal "
                         "despite gap_fill_only=True")
        self.assertAlmostEqual(bc.bias_m, 0.10, places=4)


if __name__ == '__main__':
    unittest.main()
