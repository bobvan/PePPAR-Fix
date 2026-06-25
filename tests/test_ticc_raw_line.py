"""TiccEvent.raw_line — the raw TICC wire line carried for the pos_replay
raw-capture tap, so replay re-feeds the exact text through the same parser."""
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import ticc  # noqa: E402
from peppar_fix.timebase_estimator import TimebaseRelationEstimator  # noqa: E402


class _FakeSer:
    in_waiting = 0

    def __init__(self, lines):
        self._lines = lines

    def __iter__(self):
        return iter(self._lines)


def _ticc(lines):
    t = ticc.Ticc.__new__(ticc.Ticc)         # bypass __init__ (no serial port)
    t._ser = _FakeSer(lines)
    t._recv_estimator = TimebaseRelationEstimator()
    t._last_chb_recv_mono = None
    return t


class TestTiccRawLine(unittest.TestCase):
    def test_raw_line_is_the_exact_parsed_line(self):
        evs = list(_ticc([b"1.123456789012 chA\n",
                          b"2.000000000000 chB\n"]).iter_events())
        self.assertEqual([e.raw_line for e in evs],
                         ["1.123456789012 chA", "2.000000000000 chB"])
        # re-parsing the captured raw_line reproduces the fields → replay path
        m = re.match(ticc._LINE_RE, evs[0].raw_line)
        self.assertEqual((int(m.group(1)), m.group(3)), (1, "chA"))

    def test_malformed_line_yields_no_event(self):
        self.assertEqual(list(_ticc([b"garbage\n"]).iter_events()), [])


if __name__ == "__main__":
    unittest.main()
