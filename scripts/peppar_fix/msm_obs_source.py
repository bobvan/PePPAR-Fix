"""Engine obs-source seam for RTCM MSM (I-030423 step 3).

Turns a stream of RTCM messages (from an NTRIP obs mount, or a receiver's TCP
stream) into the engine's ``(gps_time, observations)`` obs_queue items — the
same contract ``realtime_ppp.serial_reader`` fills from UBX-RXM-RAWX.  So the
engine can discipline against / position from a geodetic receiver's obs with
NO change to the PPP / ZTD / AR / clock pipeline downstream.

Pieces:
- :class:`MsmEpochAssembler` groups the MSM messages of one epoch (GPS, and
  later GAL/BDS) by time-of-week and forms IF observations at the epoch
  boundary via the shared former.
- :func:`msm_ntrip_reader` is the reader loop: route broadcast-eph messages to
  ``beph`` (when the obs stream also carries 1019/…; usually a separate
  ``--eph-mount`` thread does this) and push completed epochs onto ``obs_queue``.
- :func:`tow_to_gps_datetime` maps a GNSS time-of-week to a GPS-time datetime
  using the wall clock for the week number (rollover-safe).

GPS-first; GLONASS FDMA is a deliberate follow-on.  All I/O (message iterator,
wall clock) is injected so the assembly + timestamping are unit-testable with
no network.
"""
from __future__ import annotations

import logging
from collections import defaultdict
from datetime import datetime, timedelta, timezone

from peppar_fix.rtcm_msm_obs import (
    cells_to_raw_obs, decode_msm_obs, default_sig_lookup,
)

log = logging.getLogger("peppar-fix.msm_source")

_GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
_WEEK_S = 604800.0
_GPS_UTC_LEAP_S = 18            # GPS−UTC as of 2017; MSM tow is GPS time

# Broadcast-ephemeris RTCM message types (GPS/GLO/GAL/QZSS/BDS 1019/1020/1042/
# 1044/1045/1046).  Routed to beph if the obs stream happens to carry them.
_EPH_IDS = frozenset({"1019", "1020", "1042", "1044", "1045", "1046"})
_ARP_IDS = frozenset({"1005", "1006"})


def tow_to_gps_datetime(tow_ms, now_utc, *, leap_s=_GPS_UTC_LEAP_S):
    """GNSS time-of-week (ms) → GPS-time datetime, taking the week number from
    the wall clock ``now_utc``.  Rollover-safe: if the tow is more than half a
    week from the current wall tow, the week is nudged ±1 (so an epoch captured
    just across the Sat/Sun GPS-week boundary still lands on the right week)."""
    gps_now = now_utc + timedelta(seconds=leap_s)
    elapsed = (gps_now - _GPS_EPOCH).total_seconds()
    week = int(elapsed // _WEEK_S)
    tow_now = elapsed - week * _WEEK_S
    tow_s = tow_ms / 1000.0
    if tow_s - tow_now > _WEEK_S / 2:
        week -= 1
    elif tow_now - tow_s > _WEEK_S / 2:
        week += 1
    return _GPS_EPOCH + timedelta(seconds=week * _WEEK_S + tow_s)


class MsmEpochAssembler:
    """Accumulate the MSM messages of one epoch and emit IF observations at the
    epoch boundary.  ``add(msg)`` returns ``(tow_ms, observations)`` for the
    *previous* epoch when a new time-of-week arrives, else None; ``flush()``
    emits the final in-progress epoch.  A single-constellation stream (GPS
    1077 only) emits one epoch per message via the boundary on the next one;
    ``flush()`` releases the last."""

    def __init__(self, sig_lookup, systems, ssr=None):
        self._sig_lookup = sig_lookup
        self._systems = systems
        self._ssr = ssr
        self._tow = None
        self._raw = None

    def add(self, msg):
        dec = decode_msm_obs(msg)
        if dec is None:                       # unsupported constellation
            return None
        _prefix, tow, cells = dec
        completed = None
        if self._tow is not None and tow != self._tow and self._raw:
            completed = self._emit()
        self._tow = tow
        if self._raw is None:
            self._raw = defaultdict(dict)
        cells_to_raw_obs(cells, self._sig_lookup, self._raw)
        return completed

    def flush(self):
        return self._emit() if self._raw else None

    def _emit(self):
        from realtime_ppp import raw_obs_to_if_observations
        obs, _raw, _noff, _nsingle = raw_obs_to_if_observations(
            self._raw, self._systems, self._ssr)
        out = (self._tow, obs)
        self._raw = None
        return out


def msm_ntrip_reader(messages, obs_queue, stop_event, sig_lookup, *,
                     systems=None, ssr=None, beph=None, now_fn=None,
                     station_cb=None):
    """Reader loop: consume decoded RTCM ``messages`` (an iterator, e.g.
    ``NtripStream.messages()``), assemble MSM epochs, and push
    ``(gps_time, observations)`` onto ``obs_queue`` — the serial_reader
    contract.  Broadcast-eph messages go to ``beph`` (if given); 1005/1006
    station ARP goes to ``station_cb`` (if given).  ``now_fn`` supplies the
    wall clock for week numbering (default: real UTC now).  Returns the count
    of epochs queued.  Stops when ``stop_event`` is set or the iterator ends.
    """
    import time as _time

    from peppar_fix.event_time import ObservationEvent
    now_fn = now_fn or (lambda: datetime.now(timezone.utc))  # wallclock-todo: GPS week from system clock; should derive from ephemeris
    asm = MsmEpochAssembler(sig_lookup, systems, ssr)
    n_epochs = 0

    def _queue(tow_ms, obs):
        nonlocal n_epochs
        if obs:
            now = now_fn()
            obs_queue.put(ObservationEvent(
                gps_time=tow_to_gps_datetime(tow_ms, now),
                observations=obs,
                recv_mono=_time.monotonic(),
                recv_utc=now))
            n_epochs += 1

    for msg in messages:
        if stop_event is not None and stop_event.is_set():
            break
        ident = str(getattr(msg, "identity", getattr(msg, "DF002", "")))
        if ident in _EPH_IDS:
            if beph is not None:
                try:
                    # Stamp recv_mono explicitly — these are bare pyrtcm
                    # messages (no recv_mono attr), so without this the
                    # correction gate's broadcast_ready stays False.
                    beph.update_from_rtcm(msg, recv_mono=_time.monotonic())
                except Exception as e:       # noqa: BLE001 - bad eph is non-fatal
                    log.debug("beph update failed (%s): %s", ident, e)
            continue
        if ident in _ARP_IDS:
            if station_cb is not None:
                try:
                    station_cb((float(msg.DF025), float(msg.DF026),
                                float(msg.DF027)))
                except Exception as e:       # noqa: BLE001
                    log.debug("station ARP parse failed: %s", e)
            continue
        completed = asm.add(msg)
        if completed is not None:
            _queue(*completed)
    final = asm.flush()
    if final is not None:
        _queue(*final)
    return n_epochs


def run_msm_ntrip_source(args, obs_queue, stop_event, *, ssr=None,
                         sig_lookup=None, systems=None, beph=None):
    """Engine obs-source thread target: stream RTCM MSM from the NTRIP obs mount
    (``args.obs_ntrip_mount``, reusing the ``--ntrip-*`` caster/credentials) and
    fill ``obs_queue`` with ObservationEvents — in place of ``serial_reader``.
    Broadcast ephemeris is fed by the separate ``--eph-mount`` thread, so this
    only produces observations.  Blocks until ``stop_event`` or the stream ends.
    """
    from ntrip_client import NtripStream
    if systems is None:
        systems = (set(args.systems.split(",")) if getattr(args, "systems", None)
                   else {"gps"})
    if sig_lookup is None:
        sig_lookup = default_sig_lookup(systems,
                                        gps_l2=getattr(args, "msm_l2_sig", "GPS-L2W"))
    use_tls = (getattr(args, "ntrip_tls", False)
               or getattr(args, "ntrip_port", 2101) == 443)
    stream = NtripStream(
        caster=args.ntrip_caster, port=args.ntrip_port,
        mountpoint=args.obs_ntrip_mount,
        user=args.ntrip_user, password=args.ntrip_password, tls=use_tls,
        http10=getattr(args, 'ntrip_http10', False))
    try:
        stream.connect()
        log.info("MSM obs source: %s:%s/%s", args.ntrip_caster,
                 args.ntrip_port, args.obs_ntrip_mount)
        n = msm_ntrip_reader(stream.messages(), obs_queue, stop_event,
                             sig_lookup, systems=systems, ssr=ssr, beph=beph)
        log.info("MSM obs source ended (%d epochs queued)", n)
    except Exception as e:                    # noqa: BLE001 - thread must log, not crash
        log.error("MSM obs source failed: %s", e)
    finally:
        try:
            stream.close()
        except Exception:                     # noqa: BLE001
            pass
