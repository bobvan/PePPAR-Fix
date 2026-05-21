"""Thin wrapper around csv.writer (or any object with .writerow + .flush)
that drops all but every Nth row, controlled by a constructor-time
stride.  Lets the engine's existing log writers stay ignorant of
operator-chosen decimation — the stride lives entirely at the
construction site.

Used by --ticc-log-stride / --qerr-log-stride / --extint-log-stride /
--dt-rx-log-stride / --arm-state-log-stride in peppar_fix_engine.py.
"""
from __future__ import annotations


class StridedWriter:
    """csv.writer adapter that emits 1-of-every-N rows.

    Example: stride=10 → rows with counter 0, 10, 20, ... are written.
    stride=1 → all rows pass through.  stride <= 0 is coerced to 1
    (we never silently drop everything; operators wanting "off" should
    omit the path flag instead).
    """

    def __init__(self, inner_writer, stride: int = 1):
        self._w = inner_writer
        self._stride = max(1, int(stride))
        self._counter = 0
        # Tally for diagnostics.
        self.n_seen = 0
        self.n_emitted = 0

    def writerow(self, row) -> None:
        if (self._counter % self._stride) == 0:
            self._w.writerow(row)
            self.n_emitted += 1
        self._counter += 1
        self.n_seen += 1

    def writerows(self, rows) -> None:
        for row in rows:
            self.writerow(row)

    @property
    def stride(self) -> int:
        return self._stride
