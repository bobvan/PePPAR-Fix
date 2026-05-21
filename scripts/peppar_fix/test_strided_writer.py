"""Tests for StridedWriter."""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.strided_writer import StridedWriter


class _RecordingWriter:
    """Captures writerow calls into a list."""
    def __init__(self):
        self.rows = []
    def writerow(self, row):
        self.rows.append(list(row))


class StrideOneTest(unittest.TestCase):
    """stride=1 passes every row through (today's behavior)."""

    def test_writes_all_rows(self):
        rec = _RecordingWriter()
        sw = StridedWriter(rec, stride=1)
        for i in range(10):
            sw.writerow([i])
        self.assertEqual(len(rec.rows), 10)
        self.assertEqual(rec.rows[0], [0])
        self.assertEqual(rec.rows[-1], [9])
        self.assertEqual(sw.n_seen, 10)
        self.assertEqual(sw.n_emitted, 10)


class StrideNTest(unittest.TestCase):
    """stride=N writes every Nth row starting at index 0."""

    def test_stride_10(self):
        rec = _RecordingWriter()
        sw = StridedWriter(rec, stride=10)
        for i in range(35):
            sw.writerow([i])
        # Expect rows at counter 0, 10, 20, 30 → 4 rows
        self.assertEqual(len(rec.rows), 4)
        self.assertEqual([r[0] for r in rec.rows], [0, 10, 20, 30])
        self.assertEqual(sw.n_seen, 35)
        self.assertEqual(sw.n_emitted, 4)

    def test_stride_30(self):
        rec = _RecordingWriter()
        sw = StridedWriter(rec, stride=30)
        for i in range(100):
            sw.writerow([i])
        # 0, 30, 60, 90 → 4 rows
        self.assertEqual([r[0] for r in rec.rows], [0, 30, 60, 90])

    def test_writerows_respects_stride(self):
        rec = _RecordingWriter()
        sw = StridedWriter(rec, stride=5)
        sw.writerows([[i] for i in range(20)])
        # 0, 5, 10, 15 → 4 rows
        self.assertEqual([r[0] for r in rec.rows], [0, 5, 10, 15])


class StrideEdgeCasesTest(unittest.TestCase):

    def test_zero_stride_coerced_to_one(self):
        """Operator wanting 'off' should omit the path flag; if they
        somehow pass stride=0 we don't silently drop everything."""
        rec = _RecordingWriter()
        sw = StridedWriter(rec, stride=0)
        for i in range(5):
            sw.writerow([i])
        self.assertEqual(len(rec.rows), 5)
        self.assertEqual(sw.stride, 1)

    def test_negative_stride_coerced_to_one(self):
        rec = _RecordingWriter()
        sw = StridedWriter(rec, stride=-7)
        sw.writerow([1])
        sw.writerow([2])
        self.assertEqual(rec.rows, [[1], [2]])
        self.assertEqual(sw.stride, 1)

    def test_float_stride_truncates_to_int(self):
        """Operator passes 30.7 by accident — int() truncation, not crash."""
        rec = _RecordingWriter()
        sw = StridedWriter(rec, stride=30.7)
        self.assertEqual(sw.stride, 30)


class IndependenceTest(unittest.TestCase):
    """Two StridedWriter instances with different strides don't share state."""

    def test_independent_counters(self):
        a_rec, b_rec = _RecordingWriter(), _RecordingWriter()
        a = StridedWriter(a_rec, stride=10)
        b = StridedWriter(b_rec, stride=3)
        for i in range(15):
            a.writerow([i])
            b.writerow([i])
        # a writes at 0, 10
        self.assertEqual([r[0] for r in a_rec.rows], [0, 10])
        # b writes at 0, 3, 6, 9, 12
        self.assertEqual([r[0] for r in b_rec.rows], [0, 3, 6, 9, 12])


if __name__ == "__main__":
    unittest.main()
