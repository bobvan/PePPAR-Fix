"""Shared zenith-tropo target for both filters' ZTD ties.

Per I-132038 (ZTD-target unification): the engine's position filter
(PPPFilter, in AntPosEstThread) and time filter (FixedPosFilter, in
run_steady_state's main loop) both have a soft ZTD-residual prior.
Without unification, PPPFilter pulls toward 0 (default) while
FixedPosFilter pulls toward a METAR-derived residual.  Those targets
disagree by the METAR residual itself — tens of mm in fair weather,
100+ mm in low-pressure systems — and the model inconsistency means
each filter implicitly produces a different clock estimate from the
same observations.

`TropoState` is one shared (target_m, sigma_m) value that both filters
read every epoch.  Periodic METAR fetcher (engine-side) refreshes it;
both filter loops consume it without coordinating with each other.
Lock-protected because PPPFilter runs in `AntPosEstThread` while
FixedPosFilter runs in the engine's main thread.

Cold-start safety: `target_m=0`, `sigma_m=ztd_init_sigma`, `valid=False`
until the first successful METAR fetch updates the cache.  Filter
ties read from the cache regardless — `target_m=0` is the same prior
behaviour we used pre-unification, and the MAIN motivation for METAR-
seeded *initialization* (I-024942) is independent of the periodic tie.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field


@dataclass
class TropoTarget:
    """Snapshot of the current ZTD-tie target.

    target_m       — residual ZTD (m), positive when actual atmosphere
                     wetter / higher-pressure than the engine's 2.3 m
                     hydrostatic apriori
    sigma_m        — soft-prior 1-σ (m); larger → less constraint
    valid          — False before first successful METAR refresh; the
                     engine still ties (with target_m=0) to constrain
                     the null mode
    last_metar_age_s — for diagnostic logging at the call site
    last_refresh_epoch — engine epoch counter at last successful
                          refresh; for skip-rate diagnostics
    """
    target_m: float = 0.0
    sigma_m: float = 0.05
    valid: bool = False
    last_metar_age_s: float | None = None
    last_refresh_epoch: int = 0


class TropoState:
    """Cross-thread shared zenith-tropo state.

    Engine creates one instance in run_steady_state.  Periodic METAR
    fetcher calls `set(...)` to refresh.  PPPFilter (AntPosEst thread)
    and FixedPosFilter (main thread) call `get()` every epoch.

    Reads + writes are lock-protected because the AntPosEst thread
    is concurrent with the main loop.  Both code paths take the lock
    briefly (one tuple read / write); contention is negligible at 1 Hz
    cadence per filter.
    """

    def __init__(self, target_m: float = 0.0, sigma_m: float = 0.05):
        self._target = TropoTarget(target_m=float(target_m),
                                    sigma_m=float(sigma_m),
                                    valid=False)
        self._lock = threading.Lock()

    def get(self) -> TropoTarget:
        """Return a snapshot of the current target.

        Returns a fresh `TropoTarget` instance so callers can hold the
        snapshot across an epoch without lock contention.
        """
        with self._lock:
            return TropoTarget(
                target_m=self._target.target_m,
                sigma_m=self._target.sigma_m,
                valid=self._target.valid,
                last_metar_age_s=self._target.last_metar_age_s,
                last_refresh_epoch=self._target.last_refresh_epoch,
            )

    def set(self, *, target_m: float, sigma_m: float | None = None,
            valid: bool = True, metar_age_s: float | None = None,
            epoch: int = 0) -> None:
        """Update the target (typically called by the periodic METAR
        refresher).  sigma_m=None preserves the previous σ.

        valid=True marks the cache as METAR-backed; False (the default
        cold-start) means filter ties to target_m=0 with the wider
        sigma — same behaviour as pre-unification.
        """
        with self._lock:
            self._target.target_m = float(target_m)
            if sigma_m is not None and sigma_m > 0:
                self._target.sigma_m = float(sigma_m)
            self._target.valid = bool(valid)
            self._target.last_metar_age_s = metar_age_s
            self._target.last_refresh_epoch = int(epoch)
