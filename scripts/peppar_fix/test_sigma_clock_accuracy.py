"""σ_total → advertised clockAccuracy (I-071400 E2 / I-060548).

The advertised PTP clockAccuracy byte was a fixed per-clock-class constant, so
the 100 ns / 250 ns buckets never emitted and a ~33 ns σ_t jumped to 1 µs. E2
quantizes σ_total → the smallest enum bucket ≥ σ (decoupled from clockClass),
with asymmetric hysteresis to avoid flapping the byte near a boundary.
"""
import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix import pmc  # noqa: E402
from peppar_fix.pmc import (clock_accuracy_for_sigma_ns, PmcClient,  # noqa: E402
                            ACCURACY_25NS, ACCURACY_100NS, ACCURACY_250NS,
                            ACCURACY_1US, ACCURACY_UNKNOWN)
import peppar_fix_engine as eng  # noqa: E402


class QuantizerTest(unittest.TestCase):
    """clock_accuracy_for_sigma_ns — smallest bucket whose bound ≥ σ."""

    def test_bucket_boundaries(self):
        cases = [
            (5, ACCURACY_25NS), (25, ACCURACY_25NS),
            (25.0001, ACCURACY_100NS), (30, ACCURACY_100NS),
            (100, ACCURACY_100NS), (100.1, ACCURACY_250NS),
            (250, ACCURACY_250NS), (250.1, ACCURACY_1US),
            (1000, ACCURACY_1US), (1000.1, ACCURACY_UNKNOWN),
        ]
        for sigma, want in cases:
            self.assertEqual(clock_accuracy_for_sigma_ns(sigma), want,
                             f"σ={sigma}")

    def test_invalid_sigma_is_unknown(self):
        for bad in (None, -1, 0, float('nan'), "x"):
            self.assertEqual(clock_accuracy_for_sigma_ns(bad), ACCURACY_UNKNOWN)

    def test_nav2_bootstrap_33ns_lands_at_100ns(self):
        # 10 m NAV2 bootstrap → σ_t ≈ 33 ns → 100 ns bucket (not 1 µs)
        self.assertEqual(clock_accuracy_for_sigma_ns(33.0), ACCURACY_100NS)


class SetGrandmasterAccuracyOverrideTest(unittest.TestCase):
    """set_grandmaster_class(state, clock_accuracy=…) patches ONLY the accuracy
    byte, leaving clockClass (and the rest of the preset) intact."""

    def _client(self):
        c = PmcClient("/tmp/does-not-open")
        c._sent = []
        c.set_grandmaster_settings = lambda data: (c._sent.append(data), True)[1]
        return c

    def test_override_patches_accuracy_keeps_clockclass(self):
        c = self._client()
        self.assertTrue(c.set_grandmaster_class("locked",
                                                clock_accuracy=ACCURACY_100NS))
        sent = c._sent[-1]
        self.assertEqual(sent[0], 6)                 # clockClass 6 (locked) kept
        self.assertEqual(sent[1], ACCURACY_100NS)    # accuracy byte overridden
        # rest identical to the preset with only byte 1 differing
        preset = pmc.gm_settings_locked()
        self.assertEqual(sent[:1] + preset[1:2] + sent[2:], preset)

    def test_none_is_byte_identical_to_preset(self):
        c = self._client()
        c.set_grandmaster_class("initialized", clock_accuracy=None)
        self.assertEqual(c._sent[-1], pmc.gm_settings_initialized())


class HysteresisTest(unittest.TestCase):
    """_sigma_clock_accuracy_hysteretic — degrade fast, upgrade with margin."""

    def test_first_advertisement_is_raw(self):
        self.assertEqual(
            eng._sigma_clock_accuracy_hysteretic({}, 33.0), ACCURACY_100NS)

    def test_degrade_is_immediate(self):
        ctx = {'pmc_announced_accuracy': ACCURACY_25NS}
        # σ worsened to 200 ns → coarsen to 250 ns right away (stay honest)
        self.assertEqual(
            eng._sigma_clock_accuracy_hysteretic(ctx, 200.0), ACCURACY_250NS)

    def test_upgrade_blocked_near_boundary(self):
        ctx = {'pmc_announced_accuracy': ACCURACY_250NS}
        # σ=95 (raw would be 100 ns) but inflated 118.75 → 250 ns: hold, no flap
        self.assertEqual(
            eng._sigma_clock_accuracy_hysteretic(ctx, 95.0), ACCURACY_250NS)

    def test_upgrade_allowed_with_margin(self):
        ctx = {'pmc_announced_accuracy': ACCURACY_250NS}
        # σ=70 → inflated 87.5 ≤ 100 → comfortably in the finer bucket → upgrade
        self.assertEqual(
            eng._sigma_clock_accuracy_hysteretic(ctx, 70.0), ACCURACY_100NS)


class SetClockClassDedupTest(unittest.TestCase):
    """_set_clock_class re-announces on class OR accuracy change, dedups both."""

    def _ctx(self):
        calls = []
        pmc_stub = type("P", (), {
            "set_grandmaster_class":
                lambda self, state, clock_accuracy=None:
                    (calls.append((state, clock_accuracy)), True)[1]})()
        return {'pmc': pmc_stub}, calls

    def test_dedup_same_state_and_accuracy(self):
        ctx, calls = self._ctx()
        eng._set_clock_class(ctx, "locked", clock_accuracy=ACCURACY_25NS)
        eng._set_clock_class(ctx, "locked", clock_accuracy=ACCURACY_25NS)
        self.assertEqual(calls, [("locked", ACCURACY_25NS)])   # second deduped

    def test_reannounce_on_accuracy_change_same_state(self):
        ctx, calls = self._ctx()
        eng._set_clock_class(ctx, "locked", clock_accuracy=ACCURACY_25NS)
        eng._set_clock_class(ctx, "locked", clock_accuracy=ACCURACY_100NS)
        self.assertEqual(len(calls), 2)                        # bucket changed
        self.assertEqual(ctx['pmc_announced_accuracy'], ACCURACY_100NS)

    def test_pmc_announced_stays_a_string(self):
        # other code reads ctx['pmc_announced'] as the class string
        ctx, _ = self._ctx()
        eng._set_clock_class(ctx, "locked", clock_accuracy=ACCURACY_25NS)
        self.assertEqual(ctx['pmc_announced'], "locked")


if __name__ == "__main__":
    unittest.main()
