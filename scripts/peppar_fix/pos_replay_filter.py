"""pos_replay runner — drive the engine's position filter from a replayed
bundle to regenerate ``[PPP_STATE]`` and score it.

The loop closer (pos-replay-stage2b-plan §4): a ``ReplayDriver(decode_obs=True)``
rebuilds the ``(gps_time, observations, obs_counts)`` epoch timeline from the
captured RAWX (#243); this drives the **real** ``AntPosEstThread._process_epoch``
(#240) over it — inline via the driver's ``epoch_sink``, so the filter's
corrections (``RealtimeCorrections`` over the replay's ``SSRState`` /
``BroadcastEphemeris``) reflect the SSR/eph applied up to each epoch's
``recv_mono`` (the same per-epoch interleave as the obs bias correction, not the
post-run final state).  ``_process_epoch`` emits ``[PPP_STATE]`` via the shared
``format_ppp_state_line``; :class:`PppStateCapture` collects those lines, which
``pos_replay_compare`` then scores with the ``pos_sim`` DivergenceMonitor.

Faithful by reuse: the **engine's own** filter (``AntPosEstThread``), epoch step,
RAWX→obs, and RTCM routing — no reimplementation.  End-to-end ``[PPP_STATE]``
output needs a real bundle (broadcast eph for sat positions); the wiring here is
unit-tested with a synthetic RAWX + the real filter construction.
"""
from __future__ import annotations

import logging
import os
import sys
import threading
from typing import Optional

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix.pos_replay_driver import (ReplayDriver,  # noqa: E402
                                          read_manifest_conventions)

_ENGINE_LOGGER = "peppar-fix"


class PppStateCapture(logging.Handler):
    """Capture ``[PPP_STATE]`` log lines emitted by ``_process_epoch``.

    Attaches to the engine logger; collects each record whose message is a
    ``[PPP_STATE]`` line (the same string ``pos_replay_compare`` parses).  Use as
    a context manager — it detaches on exit."""

    def __init__(self):
        super().__init__()
        self.lines: list = []
        self._saved_level = None

    def emit(self, record):
        try:
            msg = record.getMessage()
        except Exception:
            return
        if msg.startswith("[PPP_STATE]"):
            self.lines.append(msg)

    def __enter__(self):
        logger = logging.getLogger(_ENGINE_LOGGER)
        # [PPP_STATE] is logged at INFO; ensure the logger passes INFO to us even
        # if the ambient config left it at WARNING (root default) — save/restore
        # so we don't permanently change the engine logger's verbosity.
        if logger.getEffectiveLevel() > logging.INFO:
            self._saved_level = logger.level
            logger.setLevel(logging.INFO)
        logger.addHandler(self)
        return self

    def __exit__(self, *exc):
        logger = logging.getLogger(_ENGINE_LOGGER)
        logger.removeHandler(self)
        if self._saved_level is not None:
            logger.setLevel(self._saved_level)
            self._saved_level = None


def build_filter_thread(driver: ReplayDriver, known_ecef, *, systems=None,
                        args=None):
    """Construct the engine's ``AntPosEstThread`` over the replay's correction
    state — ``RealtimeCorrections`` wrapping the SAME ``beph``/``ssr`` the
    ``driver`` populates inline, so the filter and the obs decode share one
    evolving correction state.  The thread is built but NOT started; the runner
    calls ``_process_epoch`` directly per replayed epoch."""
    from types import SimpleNamespace
    import peppar_fix_engine as eng
    from ssr_corrections import RealtimeCorrections
    if args is None:
        args = SimpleNamespace(wl_only=False, ppp_state_log=True)
    corrections = RealtimeCorrections(driver.stores["beph"], driver.stores["ssr"])
    ape_sm = eng.AntPosEst(wl_only=bool(getattr(args, "wl_only", False)))
    return eng.AntPosEstThread(
        tuple(known_ecef), corrections, threading.Event(), ape_sm,
        systems=list(systems) if systems else [], args=args)


def run_pos_replay(bundle_dir: str, known_ecef, *, systems=None, args=None,
                   truth=None) -> dict:
    """Replay a bundle through the real position filter; return the regenerated
    ``[PPP_STATE]`` lines (and an optional position score vs ``truth``).

    ``systems`` defaults to the manifest's; ``truth`` (a ``StaticTruth``) enables
    the divergence-monitor position score."""
    if systems is None:
        systems = read_manifest_conventions(bundle_dir).get("systems") or None
    driver = ReplayDriver(bundle_dir, decode_obs=True)
    thread = build_filter_thread(driver, known_ecef, systems=systems, args=args)
    driver.epoch_sink = thread._process_epoch
    with PppStateCapture() as cap:
        driver.run()
    result = {
        "ppp_state_lines": cap.lines,
        "n_epochs_decoded": len(driver.epochs),
        "thread": thread,
        "driver": driver,
    }
    if truth is not None:
        from peppar_fix.pos_replay_compare import parse_ppp_state, compare_position
        result["position"] = compare_position(parse_ppp_state(cap.lines), truth)
    return result
