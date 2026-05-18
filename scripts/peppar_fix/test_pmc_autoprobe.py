"""Tests for the PMC UDS auto-probe in peppar_fix_engine."""

import os
import socket
import sys
import tempfile
import unittest

_SCRIPTS = os.path.abspath(
    os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix_engine import _autodetect_pmc_path  # noqa: E402


class TestAutodetectPmcPath(unittest.TestCase):

    def test_no_candidates_returns_none(self):
        self.assertIsNone(_autodetect_pmc_path([]))

    def test_nonexistent_paths_return_none(self):
        self.assertIsNone(_autodetect_pmc_path([
            "/nope/nope/a", "/nope/nope/b",
        ]))

    def test_regular_file_skipped(self):
        with tempfile.NamedTemporaryFile() as f:
            self.assertIsNone(_autodetect_pmc_path([f.name]))

    def test_directory_skipped(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(_autodetect_pmc_path([d]))

    def test_real_unix_socket_returned(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "fake-ptp4l")
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            try:
                sock.bind(path)
                # File now exists and is a socket.
                self.assertEqual(_autodetect_pmc_path([path]), path)
            finally:
                sock.close()

    def test_first_existing_socket_wins(self):
        """When multiple candidates exist, return the first one in
        priority order (so callers can rank by preference)."""
        with tempfile.TemporaryDirectory() as d:
            path_a = os.path.join(d, "a")
            path_b = os.path.join(d, "b")
            sock_a = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            sock_b = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            try:
                sock_a.bind(path_a)
                sock_b.bind(path_b)
                # Order matters — first match wins.
                self.assertEqual(
                    _autodetect_pmc_path([path_a, path_b]), path_a)
                self.assertEqual(
                    _autodetect_pmc_path([path_b, path_a]), path_b)
            finally:
                sock_a.close()
                sock_b.close()

    def test_skips_to_next_candidate_when_first_missing(self):
        with tempfile.TemporaryDirectory() as d:
            path_real = os.path.join(d, "real")
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            try:
                sock.bind(path_real)
                self.assertEqual(
                    _autodetect_pmc_path([
                        "/nope/nope", path_real,
                    ]), path_real)
            finally:
                sock.close()


if __name__ == "__main__":
    unittest.main()
