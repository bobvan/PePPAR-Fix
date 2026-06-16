"""discover_phc_mac / phc_unique_id must follow udev symlinks
(doUidTomlPathSanitization part 2): /dev/ptp_i226 → real /dev/ptpN → MAC,
so PHC hosts key DO state on the stable MAC, not the device path."""
import os
from unittest import mock

from peppar_fix import do_state


def test_discover_phc_mac_follows_symlink(tmp_path):
    # Symlink whose OWN basename ("gnss-clk") would fail the "ptp" prefix
    # check; only after realpath → "ptp9" does the sysfs walk proceed.
    real = tmp_path / "ptp9"
    real.write_text("")
    link = tmp_path / "gnss-clk"
    os.symlink(real, link)

    def fake_isdir(p):
        return p == "/sys/class/ptp/ptp9/device/net"

    def fake_listdir(p):
        return ["eth0"] if p == "/sys/class/ptp/ptp9/device/net" else []

    m = mock.mock_open(read_data="aa:bb:cc:dd:ee:ff\n")
    with mock.patch("os.path.isdir", fake_isdir), \
            mock.patch("os.listdir", fake_listdir), \
            mock.patch("builtins.open", m):
        mac = do_state.discover_phc_mac(str(link))
    assert mac == "aa:bb:cc:dd:ee:ff"   # without realpath this would be None


def test_discover_phc_mac_nonptp_returns_none(tmp_path):
    # A device that realpaths to a non-ptp node yields None (no crash).
    f = tmp_path / "notaclock"
    f.write_text("")
    assert do_state.discover_phc_mac(str(f)) is None


def test_phc_unique_id_falls_back_to_path_when_no_mac(tmp_path):
    f = tmp_path / "ptp_dead"
    f.write_text("")
    # No sysfs → MAC None → phc_unique_id returns the (realpath'd) path,
    # never crashes.
    uid = do_state.phc_unique_id(str(f))
    assert uid == str(f)
