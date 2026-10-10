"""Pin that TIM-TM2's `ch` (EXTINT input index) reaches the CSV log.

`ch` is the only TM2 field that distinguishes a stray edge arriving on a
*different physical input* from a mistimed edge on the expected one.  We
logged TM2 for months without it, which is why the 18 archived PiFace
strays cannot be checked for channel provenance retrospectively.  See
docs/tm2-latency-bimodality.md.

It is appended LAST in the row so existing positional readers and
csv.DictReader consumers of older logs stay valid.
"""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.extint_reader import TimTm2Store


class _FakeMsg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class _RecordingWriter:
    def __init__(self):
        self.rows = []

    def writerow(self, row):
        self.rows.append(row)


def _msg(ch, tow_ms=322391000):
    """A TM2 message as pyubx2 presents it: named flag attributes.

    The flag values are the ones actually captured on PiFace (packing to
    0xcd: mode=1, run=0, newFallingEdge=1, timeBase=1 (GPS), utc=0,
    time=1, newRisingEdge=1), so this fake stays anchored to a real
    message rather than an invented bit pattern.
    """
    return _FakeMsg(
        ch=ch, wnR=2300, towMsR=tow_ms, towSubMsR=0, accEst=10, count=1,
        mode=1, run=0, newFallingEdge=1, timeBase=1, utc=0, time=1,
        newRisingEdge=1,
    )


class ExtintChColumnTest(unittest.TestCase):

    def test_ch_is_logged_and_is_the_last_column(self):
        w = _RecordingWriter()
        store = TimTm2Store(log_writer=w)
        store.update(_msg(ch=0))
        self.assertEqual(len(w.rows), 1)
        self.assertEqual(w.rows[0][-1], 0)

    def test_ch_1_is_preserved_not_coerced(self):
        """EXTINT1 must be distinguishable from EXTINT0 in the log."""
        w = _RecordingWriter()
        store = TimTm2Store(log_writer=w)
        store.update(_msg(ch=1))
        self.assertEqual(w.rows[0][-1], 1)

    def test_missing_ch_defaults_to_zero_rather_than_raising(self):
        """A TM2 variant without `ch` must not kill the reader thread."""
        w = _RecordingWriter()
        store = TimTm2Store(log_writer=w)
        m = _msg(ch=0)
        del m.ch
        store.update(m)
        self.assertEqual(w.rows[0][-1], 0)

    def test_appending_ch_did_not_shift_the_earlier_columns(self):
        """Older positional consumers index 0..8; those must not move."""
        w = _RecordingWriter()
        store = TimTm2Store(log_writer=w)
        store.update(_msg(ch=1, tow_ms=322391001))
        row = w.rows[0]
        self.assertEqual(len(row), 10)
        self.assertEqual(row[2], 2300)        # wn
        self.assertEqual(row[3], 322391001)   # tow_ms
        self.assertEqual(row[4], 0)           # tow_sub_ms_ns
        self.assertEqual(row[5], 10)          # acc_est_ns
        self.assertEqual(row[6], 1_000_000)   # phase_residual_ns
        self.assertEqual(row[7], 1)           # count
        self.assertEqual(row[8], "0xcd")      # flags


if __name__ == "__main__":
    unittest.main()
