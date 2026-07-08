"""Engine obs-source seam for Septentrio SBF (I-030423 — geodetic bridge).

Turns a stream of SBF blocks (from a Septentrio receiver's IP-server TCP port,
e.g. the mosaic-T) into the engine's ``(gps_time, observations)`` obs_queue
items — the same contract ``serial_reader`` fills from UBX-RXM-RAWX and
``msm_obs_source`` fills from RTCM MSM.  So the engine can position/discipline
from a Septentrio receiver's raw obs with NO change to the PPP/ZTD/AR/clock
pipeline.

SBF is simpler than RTCM MSM here: each **MeasEpoch** block is one complete
epoch and carries **WNc (week) + TOW** directly, so there is no multi-message
epoch assembly and no wall-clock week derivation — ``sbf_gps_time`` maps
WNc+TOW straight to a GPS-time datetime.

Uses ``pysbf2`` for framing; the obs decode lives in ``sbf_obs``.  GPS/GAL/BDS
(CDMA); GLONASS FDMA is a follow-on.  Broadcast ephemeris is fed by the
separate ``--eph-mount`` NTRIP thread (as with the MSM path).
"""
from __future__ import annotations

import logging
from datetime import datetime, timedelta, timezone

from peppar_fix.rtcm_msm_obs import default_sig_lookup
from peppar_fix.sbf_obs import meas_epoch_to_raw_obs

log = logging.getLogger("peppar-fix.sbf_source")

_GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
_WEEK_S = 604800.0


def sbf_gps_time(tow_ms, wnc):
    """SBF MeasEpoch TOW (ms) + WNc (full GPS week) → GPS-time datetime.
    SBF carries the week, so unlike the MSM path this needs no wall clock."""
    return _GPS_EPOCH + timedelta(seconds=int(wnc) * _WEEK_S + int(tow_ms) / 1000.0)


import threading as _threading


class PvtClockStore:
    """Latest mosaic-T OWN clock solution, from SBF PVTGeodetic.

    RxClkBias (ms) and RxClkDrift (ppm) are the receiver's own clock-error
    and frequency-offset estimates.  On an AtomiChron / PPP-timing mosaic-T
    these are its *corrected* solution — so logging them next to PePPAR-Fix's
    carrier-phase dt_rx and control word lets us compare our steering against
    theirs (docs/gnssdo-plus-integration.md).  Thread-safe: the SBF reader
    thread writes, the servo thread reads.
    """

    _DNU = -1.0e10   # SBF Do-Not-Use sentinel is ~-2e10; anything below is invalid

    def __init__(self):
        self._lock = _threading.Lock()
        self._latest = None   # (tow_ms, rxclkbias_ms, rxclkdrift_ppm, mode)

    def update_from_sbf(self, msg):
        bias = getattr(msg, "RxClkBias", None)
        if bias is None or bias <= self._DNU:
            return
        drift = getattr(msg, "RxClkDrift", None)
        mode = getattr(msg, "Mode", None)
        tow = getattr(msg, "TOW", None)
        snap = (tow,
                float(bias),
                float(drift) if drift is not None else float("nan"),
                int(mode) if mode is not None else -1)
        with self._lock:
            self._latest = snap

    def latest(self):
        """Return (tow_ms, rxclkbias_ms, rxclkdrift_ppm, mode) or None."""
        with self._lock:
            return self._latest


def sbf_obs_reader(messages, obs_queue, stop_event, sig_lookup, *,
                   systems=None, ssr=None, now_fn=None, mono_fn=None,
                   pvt_store=None):
    """Reader loop: consume pysbf2-parsed SBF ``messages`` (an iterator, e.g.
    ``SBFReader`` over a socket), decode each MeasEpoch into IF observations via
    the SHARED former, and push ``ObservationEvent``\\ s onto ``obs_queue`` —
    the serial_reader contract.  Returns the count of epochs queued; stops when
    ``stop_event`` is set or the stream ends.  ``now_fn``/``mono_fn`` are
    injectable receipt clocks for deterministic tests.
    """
    import time as _time

    from peppar_fix.event_time import ObservationEvent
    from realtime_ppp import raw_obs_to_if_observations
    now_fn = now_fn or (lambda: datetime.now(timezone.utc))  # wallclock-todo: GPS week from system clock; should derive from ephemeris
    mono_fn = mono_fn or _time.monotonic
    n_epochs = 0
    _prev_ccj = None   # last MeasEpoch CumClkJumps count (mosaic clock-jump monitor)
    for _raw, parsed in messages:
        if stop_event is not None and stop_event.is_set():
            break
        if parsed is None:
            continue
        _ident = getattr(parsed, "identity", "")
        if _ident == "PVTGeodetic":
            if pvt_store is not None:
                pvt_store.update_from_sbf(parsed)
            continue
        if _ident != "MeasEpoch":
            continue
        # CumClkJumps monitor (I-054229): the mosaic is free-running, so an
        # increment in the cumulative clock-jump count means the receiver
        # applied a ~1 ms clock STEP that lands in the carrier phase / dt_rx.
        # Surface it — rare when the DO is locked (bias stays << 0.5 ms),
        # common-mode and removable, and NOT a DO event — so it isn't misread
        # downstream as a silent phase glitch.  (Auto-realign wiring to the
        # servo is the remaining half of the bead.)
        _ccj = getattr(parsed, "CumClkJumps", None)
        if _ccj is not None:
            if _prev_ccj is not None and _ccj != _prev_ccj:
                log.warning("SBF mosaic clock JUMP: CumClkJumps %d→%d "
                            "(%d-step ~1 ms discontinuity in the carrier phase "
                            "at TOW=%s) — rare when locked, common-mode, not a "
                            "DO event", _prev_ccj, _ccj, abs(_ccj - _prev_ccj),
                            getattr(parsed, "TOW", "?"))
            _prev_ccj = _ccj
        raw_obs = meas_epoch_to_raw_obs(parsed, sig_lookup)
        obs, _r, _no, _ns = raw_obs_to_if_observations(raw_obs, systems, ssr)
        if obs:
            obs_queue.put(ObservationEvent(
                gps_time=sbf_gps_time(parsed.TOW, parsed.WNc),
                observations=obs,
                recv_mono=mono_fn(),
                recv_utc=now_fn()))
            n_epochs += 1
    return n_epochs


def _parse_hostport(spec, default_port=28784):
    host, _sep, port = str(spec).partition(":")
    return host, int(port) if port else default_port


def _stopped(stop_event):
    return stop_event is not None and stop_event.is_set()


def _interruptible_wait(stop_event, secs):
    """Sleep ``secs``, but wake immediately if ``stop_event`` fires.  Returns
    True if we should stop (event set), False if the wait elapsed normally."""
    if stop_event is None:
        import time as _time
        _time.sleep(secs)
        return False
    return stop_event.wait(secs)


def run_sbf_tcp_source(args, obs_queue, stop_event, *, ssr=None,
                       systems=None, sig_lookup=None, pvt_store=None,
                       reconnect_delay=5.0, max_reconnect_delay=60.0):
    """Engine obs-source thread target: read SBF from a receiver's IP-server TCP
    port (``args.obs_sbf_tcp`` as ``host:port``) and fill ``obs_queue`` with
    ObservationEvents — in place of ``serial_reader``.  Broadcast ephemeris is
    fed by the separate ``--eph-mount`` thread.

    Supervises the connection: on a transient TCP drop or a stream end
    (receiver reboot, network blip) it RECONNECTS with exponential backoff
    (``reconnect_delay`` → ``max_reconnect_delay``, reset on a successful
    connect), so an outage shows downstream as growing obs age rather than a
    dead feed — parity with the MSM path's NtripStream (I-110209).  The backoff
    is interruptible: ``stop_event`` breaks the loop promptly at every point
    (before connecting, during the read, and during the backoff wait).  The
    15 s socket timeout also bounds a stalled read so a wedged stream reconnects
    rather than hanging forever.
    """
    import socket

    from pysbf2 import SBFReader
    host, port = _parse_hostport(args.obs_sbf_tcp)
    if systems is None:
        systems = (set(args.systems.split(",")) if getattr(args, "systems", None)
                   else {"gps"})
    if sig_lookup is None:
        sig_lookup = default_sig_lookup(systems,
                                        gps_l2=getattr(args, "msm_l2_sig", "GPS-L2W"))
    backoff = reconnect_delay
    total = 0
    while not _stopped(stop_event):
        sock = None
        try:
            sock = socket.create_connection((host, port), timeout=15)
            backoff = reconnect_delay        # reset on a successful connect
            stream = sock.makefile("rb")
            log.info("SBF obs source connected: %s:%d", host, port)
            n = sbf_obs_reader(SBFReader(stream, quitonerror=0),
                               obs_queue, stop_event, sig_lookup,
                               systems=systems, ssr=ssr, pvt_store=pvt_store)
            total += n
            if _stopped(stop_event):
                break
            # Reader returned without a stop → the stream ended (EOF/reset).
            log.warning("SBF obs source stream ended (%d epochs this session) "
                        "— reconnecting to %s:%d", n, host, port)
        except Exception as e:                # noqa: BLE001 - thread must log, not crash
            log.warning("SBF obs source connection error (%s) — retry in %.0fs",
                        e, backoff)
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:             # noqa: BLE001
                    pass
        # Interruptible backoff before the next connect attempt.
        if _interruptible_wait(stop_event, backoff):
            break
        backoff = min(backoff * 2, max_reconnect_delay)
    log.info("SBF obs source stopped (%d epochs queued total)", total)
