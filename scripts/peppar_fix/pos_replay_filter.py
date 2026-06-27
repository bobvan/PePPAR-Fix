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
                        args=None, corrections=None):
    """Construct the engine's ``AntPosEstThread``.  By default the filter's
    corrections are ``RealtimeCorrections`` wrapping the SAME ``beph``/``ssr`` the
    ``driver`` populates inline (filter + obs decode share one evolving state).

    Pass ``corrections`` to swap the filter's orbit/clock source — the
    **precise-orbit** product-swap (Charlie #245-1): a corrections OBJECT (e.g.
    :class:`PreciseCorrections` over SP3+CLK), not a `beph` fill.  The thread is
    built but NOT started; the runner calls ``_process_epoch`` per replayed
    epoch."""
    from types import SimpleNamespace
    import peppar_fix_engine as eng
    from ssr_corrections import RealtimeCorrections
    if args is None:
        args = SimpleNamespace(wl_only=False, ppp_state_log=True)
    if corrections is None:
        corrections = RealtimeCorrections(driver.stores["beph"],
                                          driver.stores["ssr"])
    ape_sm = eng.AntPosEst(wl_only=bool(getattr(args, "wl_only", False)))
    return eng.AntPosEstThread(
        tuple(known_ecef), corrections, threading.Event(), ape_sm,
        systems=list(systems) if systems else [], args=args)


class PreciseCorrections:
    """Precise (final-products) orbit/clock corrections as a corrections OBJECT,
    for the precise-orbit product-swap (Charlie #245-1): SP3 orbits + CLKFile
    clocks, bridged into the engine's ``sat_position(sv,t) → (pos, clk)`` /
    ``sat_clock(prn,t)`` interface.

    Note the shape bridge: ``SP3.sat_position`` returns ``(x,y,z)`` (or
    ``(None,None)`` outside its safe window), while the engine's
    ``_process_epoch`` unpacks ``(pos, clk)`` — so this pairs the SP3 position
    with the CLKFile clock.  Biases are NOT here (SP3/CLK carry none); the obs
    leg reads ``stores['ssr']``, so pair this with a Bias-SINEX ssr-fill loader
    (the second half of a precise swap)."""

    def __init__(self, sp3, clk_file=None):
        self.sp3 = sp3
        self.clk_file = clk_file

    def sat_position(self, sv, t):
        import numpy as np
        p = self.sp3.sat_position(sv, t)
        if p is None or p[0] is None:
            return None, None              # outside SP3 window → skip SV cleanly
        clk = self.clk_file.sat_clock(sv, t) if self.clk_file is not None else 0.0
        return np.asarray(p, dtype=float), clk

    def sat_clock(self, prn, t):
        return (self.clk_file.sat_clock(prn, t)
                if self.clk_file is not None else 0.0)


def make_precise_corrections(sp3_path, clk_path=None):
    """Build :class:`PreciseCorrections` from final-product files — ``SP3``
    (solve_pseudorange) orbits + optional ``CLKFile`` (ppp_corrections) clocks.
    Deterministic (fixed local files).  The filter's orbits/clocks become
    precise; pair with ``make_ssr_records_loader``/a Bias-SINEX loader for the
    obs-leg biases and ``swap_streams`` to drop the captured eph/ssr."""
    from solve_pseudorange import SP3
    sp3 = SP3(sp3_path)
    clk_file = None
    if clk_path:
        from ppp_corrections import CLKFile
        clk_file = CLKFile(clk_path)
    return PreciseCorrections(sp3, clk_file)


def run_pos_replay(bundle_dir: str, known_ecef, *, systems=None, args=None,
                   truth=None, corrections_loader=None, swap_streams=None,
                   corrections_override=None) -> dict:
    """Replay a bundle through the real position filter; return the regenerated
    ``[PPP_STATE]`` lines (and an optional position score vs ``truth``).

    ``systems`` defaults to the manifest's; ``truth`` (a ``StaticTruth``) enables
    the divergence-monitor position score.

    **Product-swap** — pass ``corrections_loader``, a ``callable(stores)`` that
    populates ``stores['ssr']`` / ``stores['beph']`` from an ALTERNATE source.
    ``swap_streams`` controls which captured correction streams are swapped OUT
    (default: all three — for a precise-orbit swap).  For an **ssr-source** swap
    (different SSR/biases over the SAME broadcast orbits, e.g.
    :func:`make_ssr_records_loader`), pass ``swap_streams={'ssr','ssr_bias'}`` so
    the captured broadcast ``eph`` — the orbit base those corrections ride on —
    is KEPT (Charlie #245-1/#246).  This is the knob that separates *our filter*
    from *our corrections*: if a ZTD drift disappears under the swapped
    corrections, the gap was products; if it persists, it's the
    filter/observability (manifest §6).

    ``swap_streams`` *with* a loader = **swap-and-replace** (the loader supplies
    the substitute corrections).  ``swap_streams`` *without* a loader = a
    deliberate **ablation** — those streams are dropped with nothing replacing
    them (e.g. "how does the filter do with no SSR, on broadcast orbits?").
    Legitimate as an experiment, just not an accident (Charlie #247).

    **Precise-orbit swap** — pass ``corrections_override`` (a corrections OBJECT,
    e.g. :class:`PreciseCorrections` over SP3+CLK) to replace the filter's
    orbit/clock source entirely.  Captured eph/ssr are swapped out (the override
    supplies orbits+clocks); pair with an ssr-bias ``corrections_loader`` +
    ``swap_streams`` for the obs-leg biases (SP3/CLK carry none)."""
    if systems is None:
        systems = read_manifest_conventions(bundle_dir).get("systems") or None
    swap = corrections_loader is not None or corrections_override is not None
    # precise-orbit override swaps all captured corrections by default (it
    # supplies orbits+clocks); an ssr-source loader narrows that via swap_streams
    if corrections_override is not None and swap_streams is None:
        swap_streams = {"ssr", "ssr_bias", "eph"}
    driver = ReplayDriver(bundle_dir, decode_obs=True,
                          apply_captured_corrections=not swap,
                          swap_streams=swap_streams)
    if corrections_loader is not None:
        # load the alternate corrections BEFORE replay so they're in place for
        # the first epoch (final products are slowly-varying; a snapshot over a
        # replay window is the right first model)
        corrections_loader(driver.stores)
    thread = build_filter_thread(driver, known_ecef, systems=systems, args=args,
                                 corrections=corrections_override)
    driver.epoch_sink = thread._process_epoch
    with PppStateCapture() as cap:
        driver.run()
    result = {
        "ppp_state_lines": cap.lines,
        "n_epochs_decoded": len(driver.epochs),
        "product_swapped": swap,
        "thread": thread,
        "driver": driver,
    }
    if truth is not None:
        from peppar_fix.pos_replay_compare import parse_ppp_state, compare_position
        result["position"] = compare_position(parse_ppp_state(cap.lines), truth)
    return result


def make_ssr_records_loader(path):
    """An **ssr-source** product-swap loader (the first concrete one): populate
    the replay's ``SSRState`` from an external SSR-records JSON — the HAS-bridge
    format (``load_ssr_records`` / ``SSRState.update_from_records``: per-SV
    orbit/clock/code-bias corrections to broadcast).  Pair with
    ``swap_streams={'ssr','ssr_bias'}`` so the captured broadcast ``eph`` (the
    orbit base these corrections ride on) is kept.

    Deterministic by construction — loads from a fixed local file, not a timed
    network fetch, so ``--verify-deterministic`` holds under swap (Charlie
    #245-2).  Raises if the file has no records (a loud coverage check beats
    silently feeding empty corrections — Charlie #245-2)."""
    def _load(stores):
        from realtime_ppp import load_ssr_records
        epoch_s, records = load_ssr_records(path)
        if not records:
            # load_ssr_records returns (None, None) for missing/garbled/empty —
            # all of which would silently feed empty corrections; fail loud.
            raise ValueError(
                f"ssr-records product file missing/garbled/empty (no records): "
                f"{path} (coverage check — would silently feed no corrections)")
        stores["ssr"].update_from_records(records, epoch_s)
    return _load
