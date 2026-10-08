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

    Mechanism (PiFace F9P, characterised 2026-06-19; **decomposed into TWO
    independent defects 2026-09-20**, per Bob via BBGresearch):

    1. **~425 ns FIXED delay** — the ArduSimple carrier board places an **RC
       network between its breakout pin and the module's EXTINT pin**.  Its
       own documentation calls this pulse shaping; it costs microseconds.
       Cure is to bypass it.  A calibration problem, not a noise one.

    2. **+180-400 ns spurious LATE edges (24.8 % of edges)** — the **TADD-2
       Mini's PIC defaults to 5 V**, so it drives a 5 V PPS into a 3V3 EXTINT
       input.  That overdrive rings; when the ring dips below logic-low and
       re-crosses, the receiver sees a SECOND edge.  TIM-TM2 reports the later
       one, which TRUMPS the real edge (the store keeps the freshest sample),
       so the EKF reads the DO PPS as *late* -> concludes the DO slowed ->
       cranks the actuator frequency UP -> a real DO excursion (measured 29 us
       on chA; 308 us when EXTINT is the sole observer).

    **This filter addresses (2) only, and cannot touch (1).**  An earlier
    version of this docstring attributed both to one "input-shaping network",
    which accounted for the delay and left the intermittency unexplained — a
    *fixed* delay and an *intermittent* extra edge are different kinds of
    symptom and should not have shared a cause.  That was the tell that was
    walked past for three months.

    **ONE-SIDEDNESS IS IMMUNE TO HALF THE RE-ACQUISITION HAZARD, NOT ALL OF
    IT** (recorded 2026-09-20 after BBGresearch measured an antenna pull).

    When a receiver loses its fix and re-acquires, it corrects its own clock
    in a SINGLE SAMPLE — they measured a −174 ns step at the moment the fix
    returned, with accEst improving 108 → 68 at the same sample.  That is
    exactly the shape this filter exists to reject, and rejecting it would be
    wrong: it is the one genuinely correct sample in the sequence.

    Their step was *early*, so our late-only rule passed it.  **But the sign
    is not a property of re-acquisition — it is set by which way the clock
    drifted during the outage.**  A DO or rx TCXO with a positive free-run
    offset drifts its PPS *late*, and the correction then arrives as a large
    LATE step, which this filter rejects.  Our fleet has both signs on record
    (PiFace −135 ppb, clkPoC3 +21 ppb), so do not record this filter as immune
    to re-acquisition — it is immune to whichever half the drift sign spares.

    The discriminator that survives BOTH signs is accEst: a legitimate step
    carries a simultaneous accEst *improvement*; a glitch does not.  **That
    rule is not expressible here — `accept()` takes only `phase_ns` and cannot
    see accEst.**  A structural limitation, not a tuning question.  Unvalidated
    on either project's hardware, so not implemented; recorded so the gap is
    visible before someone assumes coverage that does not exist.

    Related, and quantified: `ExtintReader.update()` drops any sample with
    `accEst > max_acc_est_ns` (default 200 ns) BEFORE this filter sees it.
    BBGresearch measured accEst climbing 30 → 127 ns over a six-minute
    outage, decelerating to 0.17 ns/s — so **200 ns is reached at roughly a
    13–20 minute outage.**  The margin is minutes, not hours, and past it we
    discard exactly the samples carrying the receiver's own correction.

    **The cheapest cure for (2) is at the source and makes this filter
    unnecessary for that fault: the TADD-2's PIC is configurable — strap it
    for 3.3 V** and the overdrive is gone for every downstream device at once.
    Keep the filter as a safety net for interface faults not yet found, but
    check the logic levels first: no software filter removes an artefact at
    source, and (1) is invisible to any filter — only a schematic shows it.

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

    **Known limitation — the servo bends the trajectory this filter predicts.**
    The predictor is linear: a median per-edge step extrapolated forward.  It
    tracks a *constant* drift exactly (at any skip count, since the fix of
    2026-09-19), but it cannot track a *changing* drift — and a servo changes
    the DO frequency every epoch by construction, so the phase trajectory is
    curved on purpose.  The median over a `window`-sample history lags the
    current step by roughly ``a * window / 2`` for a frequency slew ``a``, so
    genuine edges start being rejected at about::

        a > 2 * reject_ns / window   ==  2*50/16  ==  6.25 ppb/s

    Measured against this class with **no ringing present at all**, 1 Hz:

        0-6 ppb/s   ->  0/60 genuine edges rejected
        10 ppb/s    -> 24/60 rejected, 3 forced re-baselines
        25 ppb/s    -> 35/60 rejected, 4 forced re-baselines
       100 ppb/s    -> 40/60 rejected, 4 forced re-baselines

    So a tightly-slewing servo makes this filter reject the signal it is
    supposed to be cleaning, and **the tighter the servo the worse it gets**.
    Hypothesis raised by BBGresearch 2026-09-19 from a 31% rejection rate that
    survived their own predictor fix; the threshold above is ours, measured.

    NOT ACTED ON YET, deliberately: we have shown the mechanism exists above
    ~6 ppb/s but have NOT measured whether our servo actually slews that fast
    in normal operation.  `stats()['reject_rate']` is the number that settles
    it — a persistently high rate with no known ringing source means the
    predictor is failing to track, not that the signal is bad.  Candidate
    fixes if it is real: scale `reject_ns` with the commanded frequency-slew
    rate, or extrapolate with curvature instead of a median step.  Do not
    simply widen `reject_ns` — that trades this failure for blindness to the
    ringing the filter exists to catch.
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
        last good edge.  The extrapolation **advances with the skipping** —
        see the ``_consec_rej`` term below; without it the predictor eats
        itself whenever the DO drifts faster than ``reject_ns`` per edge.
        """
        acc = self._accepted
        if (len(acc) >= self.warmup
                and self._consec_rej < self.max_consecutive_rejects):
            steps = [_wrap_ns(acc[i + 1] - acc[i]) for i in range(len(acc) - 1)]
            step = statistics.median(steps) if steps else 0.0
            # Advance the prediction by one step PER EDGE ELAPSED, not one
            # step per call.  `acc[-1]` is the last *accepted* edge, so after
            # `_consec_rej` rejections the next edge is (1 + _consec_rej)
            # edges later and its true phase has drifted that many steps.
            #
            # Without the multiplier the predictor compounds: one rejection
            # leaves the prediction one step low, which makes the next
            # GENUINE edge look one step late, which rejects it, which puts
            # the prediction two steps low.  It self-sustains as soon as the
            # per-edge drift exceeds reject_ns (50 ns/s = 50 ppb) — i.e. on
            # any free-running or re-acquiring DO, which is exactly when
            # EXTINT matters most on a --no-ticc host.
            #
            # Measured before the fix, single ringing event at 1 Hz:
            #   drift    2 ns/s -> 1 rejection   (harmless)
            #   drift   60 ns/s -> 8 rejections + forced re-baseline
            #   drift 1200 ns/s -> 8 rejections + forced re-baseline
            # After: 1 rejection at every drift rate, and a genuine 5 us
            # phase step still re-baselines as intended.
            #
            # Found by BBGresearch 2026-09-19, who hit the same compounding
            # in their own predictor — where it did NOT self-limit, because
            # theirs fitted a window and interleaved rejects with accepts so
            # the run-length hatch never fired.  Ours was being masked by
            # max_consecutive_rejects, which was silently doing this job
            # rather than the one its comment claims.
            pred = acc[-1] + step * (1 + self._consec_rej)
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

    def stats(self):
        """Counters, so "what is this filter actually catching?" is answerable.

        These were write-only until 2026-09-19.  That mattered: the predictor
        compounding fixed in the same commit had been driving `n_rebaseline`
        on any drifting DO, meaning the escape hatch was mostly absorbing a
        predictor artefact rather than catching real phase steps as its
        docstring claims — and no counter was exported that could have shown
        it.  A mechanism whose observed effect nobody can see is a mechanism
        whose stated purpose nobody can check.

        Raised by BBGresearch, whose framing is the one to keep: for any
        gate, ask *what does this exist to catch, and what is it catching
        most of the time?*  The second half needs instrumentation.

        `reject_rate` is the number to watch.  A persistently high rate with
        no known ringing source means the predictor is failing to track the
        signal, not that the signal is bad — see the servo-curvature note in
        the class docstring.
        """
        seen = self.n_accepted + self.n_rejected
        return dict(
            n_accepted=self.n_accepted,
            n_rejected=self.n_rejected,
            n_rebaseline=self.n_rebaseline,
            last_reject_ns=self.last_reject_ns,
            reject_rate=(self.n_rejected / seen) if seen else 0.0,
            reject_ns=self.reject_ns,
            window=self.window,
        )

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
        host_wall = time.time()  # wallclock-ok: extint log wall stamp; correlation uses host_mono (recv_mono)
        wnR = int(getattr(parsed, "wnR", 0))
        tow_ms = int(getattr(parsed, "towMsR", 0))
        tow_sub_ms_ns = int(getattr(parsed, "towSubMsR", 0))
        acc_est = int(getattr(parsed, "accEst", -1))
        count = int(getattr(parsed, "count", 0))
        # pyubx2 does NOT expose a raw `flags` byte for TIM-TM2 — it expands
        # the bitfield into named attributes.  `getattr(parsed, "flags", 0)`
        # therefore returned the DEFAULT 0 on every message ever logged, and
        # the `flags` column in every extint CSV we hold is a constant zero
        # (verified: 758,017 samples across 137 captures, all 0x00).  Worse
        # than missing, because a column of zeros looks like data.
        # Reconstruct the real byte from the named attributes, u-blox layout:
        #   bit0 mode  bit1 run  bit2 newFallingEdge  bits3-4 timeBase
        #   bit5 utc   bit6 time (timestamp VALID)    bit7 newRisingEdge
        flags = (
            (int(getattr(parsed, "mode", 0)) & 1)
            | ((int(getattr(parsed, "run", 0)) & 1) << 1)
            | ((int(getattr(parsed, "newFallingEdge", 0)) & 1) << 2)
            | ((int(getattr(parsed, "timeBase", 0)) & 3) << 3)
            | ((int(getattr(parsed, "utc", 0)) & 1) << 5)
            | ((int(getattr(parsed, "time", 0)) & 1) << 6)
            | ((int(getattr(parsed, "newRisingEdge", 0)) & 1) << 7)
        )

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
