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

import collections
import statistics
import threading
import time

# 1 GPS second in nanoseconds.
_NS_PER_S = 1_000_000_000


def _wrap_ns(d):
    """Wrap a phase difference into (-0.5 s, +0.5 s] nanoseconds."""
    d %= _NS_PER_S
    if d > _NS_PER_S // 2:
        d -= _NS_PER_S
    return d


class ExtintLateEdgeFilter:
    """Reject spurious LATE EXTINT edges caused by input-network ringing.

    Mechanism (PiFace F9P, characterised 2026-06-19):
    the ArduSimple F9P EXTINT input-shaping network adds a fixed delay
    (~425 ns on PiFace) AND rings.  When the ring dips back below the
    EXTINT logic-low threshold and re-crosses, the receiver sees a SECOND
    edge one ringing-period later (~180-400 ns).  TIM-TM2 reports the
    later edge, which TRUMPS the real one (the store keeps the freshest
    sample), so the EKF reads the DO PPS as *late* -> concludes the DO
    slowed -> cranks the actuator frequency UP -> a real DO excursion
    (measured 29 us on chA; 308 us when EXTINT is the sole observer).

    Empirically the spurious edges are **strictly one-directional** (169
    late / 0 early over a quiet 3000-edge window) and jump discretely
    above the running trend by >> the network's nominal-delay jitter
    (jitter ~+/-10 ns; ringing +180-400 ns).  So:

    - Track a running trend of *accepted* edge phases (median per-edge
      step) and predict the next phase.  This follows the slow DO drift
      at any rate, so a fast-but-genuine drift is NOT filtered.
    - Reject an edge only if it is LATER than the prediction by more than
      ``reject_ns`` (one-sided: an early edge is never ringing, and a
      genuine DO speed-up must pass).
    - After ``max_consecutive_rejects`` rejects in a row, force-accept and
      re-baseline: a *persistent* late offset is a real DO phase step, not
      transient ringing, and must not blind EXTINT forever (load-bearing
      on ``--no-ticc`` hosts where EXTINT is the sole DO observer).

    This is the TIM-TM2/EXTINT analogue of :class:`ptp_device.DualEdgeFilter`
    (which rejects i226 EXTTS dual-edge artifacts on the GNSS-PPS path).
    """

    def __init__(self, reject_ns=50.0, window=16, warmup=4,
                 max_consecutive_rejects=8):
        if reject_ns <= 0:
            raise ValueError(f"reject_ns must be > 0, got {reject_ns}")
        self.reject_ns = float(reject_ns)
        self.window = int(window)
        self.warmup = int(warmup)
        self.max_consecutive_rejects = int(max_consecutive_rejects)
        self._accepted = collections.deque(maxlen=self.window)
        self._consec_rej = 0
        self.n_accepted = 0
        self.n_rejected = 0
        self.n_rebaseline = 0
        self.last_reject_ns = None

    def accept(self, phase_ns):
        """Return True if this EXTINT edge phase should be used, else False.

        Mutates internal trend state on accept; a rejected edge is not
        added to the trend so the prediction keeps extrapolating from the
        last good edge.
        """
        acc = self._accepted
        if (len(acc) >= self.warmup
                and self._consec_rej < self.max_consecutive_rejects):
            steps = [_wrap_ns(acc[i + 1] - acc[i]) for i in range(len(acc) - 1)]
            step = statistics.median(steps) if steps else 0.0
            pred = acc[-1] + step
            dev = _wrap_ns(phase_ns - pred)
            if dev > self.reject_ns:        # one-sided: only LATE edges
                self.n_rejected += 1
                self._consec_rej += 1
                self.last_reject_ns = dev
                return False
        # Accept: passed the gate, still warming up, or forced after a run of
        # rejects (a persistent offset is a real step -> re-baseline to it).
        if self._consec_rej >= self.max_consecutive_rejects:
            acc.clear()
            self.n_rebaseline += 1
        self._consec_rej = 0
        acc.append(phase_ns)
        self.n_accepted += 1
        return True

    def reset(self):
        """Forget the trend (e.g. after a servo reset / PHC step)."""
        self._accepted.clear()
        self._consec_rej = 0


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


# Sentinel so callers can disable the late-edge filter explicitly.
_NO_FILTER = object()


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
                 max_acc_est_ns=200, late_edge_filter=_NO_FILTER):
        """
        Args:
          log_writer: optional csv.writer.  Header expected:
              host_timestamp,host_monotonic,wn,tow_ms,tow_sub_ms_ns,
              acc_est_ns,phase_residual_ns,count,flags
          log_file: file handle for flush() after each row.
          max_acc_est_ns: accEst values above this are recorded but
              not exposed to the engine — the F9T's nav engine is
              degraded and the timestamp is not trustworthy.
          late_edge_filter: an :class:`ExtintLateEdgeFilter` to drop
              ringing-induced late edges before they reach the EKF.
              Default constructs one (the fix is on by default — it is a
              no-op on clean inputs); pass ``None`` to disable.  Rejected
              edges are still written to ``log_writer`` (the raw stream is
              preserved for diagnosis); only the EKF-facing ``_latest`` is
              gated.
        """
        self._lock = threading.Lock()
        self._latest = None  # (recv_mono, phase_ns, acc_est_ns) or None
        self._log_writer = log_writer
        self._log_file = log_file
        self._max_acc_est_ns = max_acc_est_ns
        if late_edge_filter is _NO_FILTER:
            late_edge_filter = ExtintLateEdgeFilter()
        self._late_filter = late_edge_filter
        # Stats — for [STATUS] line and post-hoc diagnosis.
        self.n_received = 0
        self.n_dropped_acc_est = 0
        self.n_dropped_late_edge = 0
        self.n_consumed = 0

    def update(self, parsed, recv_mono=None):
        """Push a parsed TIM-TM2 message.

        `parsed` must have rising-edge fields:
            wnR, towMsR, towSubMsR, accEst, count, flags

        ``recv_mono`` overrides the ingest stamp (None = live).  Pass the
        captured per-frame recv_mono when replaying so freshness is a pure
        function of the captured stream (pos_replay milestone-0b /
        docs/pos-replay-capture-manifest.md §6).
        """
        host_mono = time.monotonic() if recv_mono is None else recv_mono
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

        # Ringing late-edge rejection (one-sided).  The raw row was already
        # logged above, so the diagnostic stream keeps the dropped edge;
        # only the EKF-facing _latest is gated.
        if self._late_filter is not None and not self._late_filter.accept(
                residual_ns):
            self.n_dropped_late_edge += 1
            return

        with self._lock:
            self._latest = (host_mono, residual_ns, acc_est)

    def consume_latest(self, max_age_s=2.0, now_mono=None):
        """Return the freshest unconsumed sample as
        (phase_ns, acc_est_ns), or None if none is available or it
        is older than `max_age_s` seconds.

        After a successful read, the store returns None on the next
        call until update() is called again.  This keeps the engine
        from feeding the same TIM-TM2 measurement to the EKF twice.

        ``now_mono`` overrides the freshness reference (None = live) —
        pos_replay milestone-0b virtual clock.
        """
        now = time.monotonic() if now_mono is None else now_mono
        with self._lock:
            if self._latest is None:
                return None
            recv_mono, phase_ns, acc_est_ns = self._latest
            self._latest = None
        if now - recv_mono > max_age_s:
            return None
        self.n_consumed += 1
        return phase_ns, acc_est_ns
