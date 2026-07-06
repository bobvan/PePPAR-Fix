"""fixWarmRateChange (I-041905): the forced-driver warm path must apply
--measurement-rate-ms, not silently keep the receiver's prior rate.

Regression for the bug where ensure_receiver_ready's forced-driver
`dual >= 4` fast-path returned BEFORE configure_rate_ms, so a receiver
already delivering dual-freq kept its old measurement rate and the flag
was dropped (engine's meas_rate_hz and the receiver then disagree).
"""
import unittest
from unittest import mock

from peppar_fix.receiver import ensure_receiver_ready, F9TL5Driver


def _identity_patches(module="ZED-F9T"):
    identity = {"module": module, "unique_id": 1, "unique_id_hex": "01",
                "firmware": "TIM 2.25", "protver": "y"}
    return (
        mock.patch("peppar_fix.receiver.query_receiver_identity",
                   return_value=identity),
        mock.patch("peppar_fix.receiver_state.check_receiver_change",
                   return_value=(None, "new")),
        mock.patch("peppar_fix.receiver_state.new_receiver_state",
                   return_value={}),
        mock.patch("peppar_fix.receiver_state.save_receiver_state"),
    )


class TestForcedDriverWarmRate(unittest.TestCase):

    def test_warm_path_applies_measurement_rate(self):
        """A warm forced-F9T (already dual-freq) must still assert the
        requested measurement rate via configure_rate_ms."""
        p_id, p_chg, p_new, p_save = _identity_patches()
        with p_id, p_chg, p_new, p_save, \
                mock.patch("peppar_fix.receiver._check_dual_freq",
                           return_value=(6, 10, set())), \
                mock.patch("peppar_fix.receiver.open_receiver",
                           return_value=(mock.MagicMock(), mock.MagicMock())), \
                mock.patch("peppar_fix.receiver.configure_rate_ms",
                           return_value=True) as rate, \
                mock.patch("peppar_fix.receiver.configure_signals") as sig:
            driver, _ident = ensure_receiver_ready(
                "/dev/x", 115200, port_type="USB",
                forced_driver=F9TL5Driver(), measurement_rate_ms=500)
        self.assertIsInstance(driver, F9TL5Driver)
        rate.assert_called_once()
        # third positional arg to configure_rate_ms is the rate in ms
        self.assertEqual(rate.call_args.args[2], 500)
        # warm path — no signal reconfigure dance
        sig.assert_not_called()

    def test_warm_path_no_rate_when_unset(self):
        """measurement_rate_ms=None (defensive) → no rate write, still
        returns the forced driver."""
        p_id, p_chg, p_new, p_save = _identity_patches()
        with p_id, p_chg, p_new, p_save, \
                mock.patch("peppar_fix.receiver._check_dual_freq",
                           return_value=(6, 10, set())), \
                mock.patch("peppar_fix.receiver.open_receiver",
                           return_value=(mock.MagicMock(), mock.MagicMock())), \
                mock.patch("peppar_fix.receiver.configure_rate_ms",
                           return_value=True) as rate:
            driver, _ident = ensure_receiver_ready(
                "/dev/x", 115200, port_type="USB",
                forced_driver=F9TL5Driver(), measurement_rate_ms=None)
        self.assertIsInstance(driver, F9TL5Driver)
        rate.assert_not_called()


if __name__ == "__main__":
    unittest.main()
