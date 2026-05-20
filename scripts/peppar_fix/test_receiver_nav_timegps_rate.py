"""Tests for per-driver NAV-TIMEGPS rate (eliminates F10T rate-cap NAK).

F10T NAKs CFG_MSGOUT_UBX_NAV_TIMEGPS_* at rate > 1; messages flow
at 1 anyway but the NAK shows up on every engine restart.  Per-
driver `nav_timegps_rate` attribute lets each receiver class pick
its own decimation.
"""

import unittest
from unittest import mock

from peppar_fix.receiver import (
    F9TDriver,
    F9TL2E5bDriver,
    F9TL5Driver,
    F10TDriver,
    ReceiverDriver,
    configure_messages,
)


class TestNavTimegpsRate(unittest.TestCase):

    def test_base_driver_default_is_5(self):
        self.assertEqual(ReceiverDriver().nav_timegps_rate, 5)

    def test_f9t_inherits_5(self):
        for cls in (F9TDriver, F9TL5Driver, F9TL2E5bDriver):
            with self.subTest(cls=cls.__name__):
                self.assertEqual(cls().nav_timegps_rate, 5)

    def test_f10t_overrides_to_1(self):
        self.assertEqual(F10TDriver().nav_timegps_rate, 1)


class TestConfigureMessagesUsesDriverRate(unittest.TestCase):
    """configure_messages plumbs the driver attribute into the VALSET."""

    def _captured_messages(self, driver):
        captured = {}

        def fake_send_cfg(ser, ubr, key_values, description="", layers=7):
            captured.update(key_values)
            return True, []

        with mock.patch("peppar_fix.receiver.send_cfg",
                        side_effect=fake_send_cfg):
            configure_messages(ser=None, ubr=None, port_id=3,
                               sfrbx_rate=1, driver=driver)
        return captured

    def test_f9t_emits_rate_5(self):
        msgs = self._captured_messages(F9TDriver())
        self.assertEqual(msgs["CFG_MSGOUT_UBX_NAV_TIMEGPS_USB"], 5)

    def test_f10t_emits_rate_1(self):
        msgs = self._captured_messages(F10TDriver())
        self.assertEqual(msgs["CFG_MSGOUT_UBX_NAV_TIMEGPS_USB"], 1)

    def test_no_driver_falls_back_to_base_default(self):
        msgs = self._captured_messages(None)
        self.assertEqual(msgs["CFG_MSGOUT_UBX_NAV_TIMEGPS_USB"], 5)

    def test_sfrbx_zero_still_disables_regardless_of_driver(self):
        captured = {}

        def fake_send_cfg(ser, ubr, key_values, description="", layers=7):
            captured.update(key_values)
            return True, []

        with mock.patch("peppar_fix.receiver.send_cfg",
                        side_effect=fake_send_cfg):
            configure_messages(ser=None, ubr=None, port_id=3,
                               sfrbx_rate=0, driver=F10TDriver())
        self.assertEqual(captured["CFG_MSGOUT_UBX_NAV_TIMEGPS_USB"], 0)


if __name__ == "__main__":
    unittest.main()
