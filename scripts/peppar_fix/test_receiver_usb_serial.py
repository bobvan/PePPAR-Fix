"""Tests for peppar_fix.receiver.usb_serial_for_tty.

The happy path requires a real /sys tree backing a USB device, so
it's covered by lab observation rather than unit test.  Here we
cover the failure modes — bad paths, missing attributes — which
must always return None without raising.
"""

import unittest

from peppar_fix.receiver import usb_serial_for_tty


class TestUsbSerialForTtyFailures(unittest.TestCase):

    def test_nonexistent_path_returns_none(self):
        self.assertIsNone(usb_serial_for_tty("/dev/nope-this-does-not-exist"))

    def test_non_tty_path_returns_none(self):
        # /dev/null is a real device but not a tty under /sys/class/tty.
        self.assertIsNone(usb_serial_for_tty("/dev/null"))

    def test_empty_string_returns_none(self):
        self.assertIsNone(usb_serial_for_tty(""))

    def test_arbitrary_file_returns_none(self):
        # A regular file resolves but sysfs lookup fails.
        self.assertIsNone(usb_serial_for_tty("/etc/hostname"))


if __name__ == "__main__":
    unittest.main()
