"""TIM-TM2 ingest for the gnss-phase-experiment EKF arm 3.

UBX-TIM-TM2 reports the F9T's GPS-time view of a rising/falling edge
on its EXTINT pin.  Wire DO PPS → F9T EXTINT and each DO PPS edge
generates one TIM-TM2 message carrying:
  - wnR/towMsR/towSubMsR : rising-edge GPS time
  - accEst                : F9T's own σ on the timestamp (ns)
  - count                 : running edge counter
  - flags                 : edge polarity / new-edge flags

This module turns that into the EKF's Arm 3 measurement:
  z_extint = signed offset (ns) of the rising edge from its
             nearest integer GPS-second tick
  σ_extint = accEst from the message itself

The EKF's H_extint = [0, 0, 1, 0] observes x[2] (DO phase from
GPS) directly.  No edge-FIFO matching, no rx_tcxo correlation —
just the freshest TIM-TM2 sample per epoch.

See docs/dofreq-est-measurement-ladder.md for context.
"""
from __future__ import annotations

import threading
import time

# 1 GPS second in nanoseconds.
_NS_PER_S = 1_000_000_000


def phase_residual_ns(tow_ms, tow_sub_ms_ns):
    """Signed nanoseconds from the nearest integer GPS-second tick.

    The F9T's TIM-TM2 reports an edge time as `towMsR` (milliseconds
    of GPS week) plus `towSubMsR` (sub-millisecond ns).  Reduce to
    "where in the GPS second did the edge fall, signed".

    Returns:
      An int.  Positive: edge was after the second tick.  Negative:
      edge was before the next second tick (i.e., closer to the next
      one than the previous).
    """
    ms_in_second = int(tow_ms) % 1000
    in_second_ns = ms_in_second * 1_000_000 + int(tow_sub_ms_ns)
    # Wrap (-500ms, +500ms].  An edge at +999.999 ms reads as -1 µs,
    # which is what the EKF wants — small offset from the *nearest*
    # second, not the *previous* second.
    if in_second_ns > _NS_PER_S // 2:
        in_second_ns -= _NS_PER_S
    return in_second_ns


class TimTm2Store:
    """Thread-safe latest-rising-edge holder fed by the UBX reader.

    The realtime_ppp.py UBX thread parses each TIM-TM2 message and
    calls update(); the engine main loop calls consume_latest() at
    each servo update to retrieve the freshest unconsumed sample
    (or None if none is fresh).

    Consumption is one-shot per sample: once the engine reads a
    sample, the store returns None until the next TIM-TM2 message
    arrives.  This prevents the engine from feeding the same EKF
    measurement twice.
    """

    def __init__(self, log_writer=None, log_file=None,
                 max_acc_est_ns=200):
        """
        Args:
          log_writer: optional csv.writer.  Header expected:
              host_timestamp,host_monotonic,wn,tow_ms,tow_sub_ms_ns,
              acc_est_ns,phase_residual_ns,count,flags
          log_file: file handle for flush() after each row.
          max_acc_est_ns: accEst values above this are recorded but
              not exposed to the engine — the F9T's nav engine is
              degraded and the timestamp is not trustworthy.
        """
        self._lock = threading.Lock()
        self._latest = None  # (recv_mono, phase_ns, acc_est_ns) or None
        self._log_writer = log_writer
        self._log_file = log_file
        self._max_acc_est_ns = max_acc_est_ns
        # Stats — for [STATUS] line and post-hoc diagnosis.
        self.n_received = 0
        self.n_dropped_acc_est = 0
        self.n_consumed = 0

    def update(self, parsed):
        """Push a parsed TIM-TM2 message.

        `parsed` must have rising-edge fields:
            wnR, towMsR, towSubMsR, accEst, count, flags
        """
        host_mono = time.monotonic()
        host_wall = time.time()
        wnR = int(getattr(parsed, "wnR", 0))
        tow_ms = int(getattr(parsed, "towMsR", 0))
        tow_sub_ms_ns = int(getattr(parsed, "towSubMsR", 0))
        acc_est = int(getattr(parsed, "accEst", -1))
        count = int(getattr(parsed, "count", 0))
        flags = int(getattr(parsed, "flags", 0))

        residual_ns = phase_residual_ns(tow_ms, tow_sub_ms_ns)

        if self._log_writer is not None:
            try:
                self._log_writer.writerow([
                    f"{host_wall:.6f}",
                    f"{host_mono:.9f}",
                    wnR, tow_ms, tow_sub_ms_ns,
                    acc_est, residual_ns, count, f"0x{flags:02x}",
                ])
                if self._log_file is not None:
                    self._log_file.flush()
            except (OSError, ValueError):
                pass

        self.n_received += 1
        if acc_est > self._max_acc_est_ns:
            self.n_dropped_acc_est += 1
            return

        with self._lock:
            self._latest = (host_mono, residual_ns, acc_est)

    def consume_latest(self, max_age_s=2.0):
        """Return the freshest unconsumed sample as
        (phase_ns, acc_est_ns), or None if none is available or it
        is older than `max_age_s` seconds.

        After a successful read, the store returns None on the next
        call until update() is called again.  This keeps the engine
        from feeding the same TIM-TM2 measurement to the EKF twice.
        """
        with self._lock:
            if self._latest is None:
                return None
            recv_mono, phase_ns, acc_est_ns = self._latest
            self._latest = None
        if time.monotonic() - recv_mono > max_age_s:
            return None
        self.n_consumed += 1
        return phase_ns, acc_est_ns
