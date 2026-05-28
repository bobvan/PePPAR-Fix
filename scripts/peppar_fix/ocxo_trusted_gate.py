"""OCXO-trusted observation rejection gate.

Reject EKF arm observations whose innovation magnitude implies physically
impossible DO frequency movement over the elapsed integration interval.

Anchored on each DO's *characterized freerun short-τ noise floor*
(from `state/dos/<label>.json`), independent of the filter's own R/Q
tuning.  This complements (does not replace) the existing chi² gate,
which keys on `√S` — that's the filter's *predicted* noise, which
loosens whenever Q or P bloom.  The OCXO-trusted gate keys on physics:
the OCXO itself cannot move faster than its characterized noise floor.

## Use case

PiFace today: TICC innov std = 10 ns/epoch, engine's √S = 1.1 ns.
The chi² gate fires 5.8% of epochs (10σ); the rest pass and inject
measurement noise into x3, which gets committed to the DAC and the
OCXO inherits.  Disc TDEV(1s) = 702 ps vs freerun 54 ps (13× worse).

The OCXO-trusted gate at K=10 with σ_DO=54 ps gives:

    threshold_1s = 10 × 54 ps × √1.0 = 540 ps

Most of PiFace's 10 ns-std measurement noise exceeds this and gets
rejected.  The few clean observations get through to maintain
long-term lock.

## Two regimes (implemented — `regime_aware=True`)

The v1 single-threshold gate engages by AGE alone (`min_age_s`) and
stays tight forever.  That is correct for a host that LOCKS under the
gate (PiFace: locks fast, then the 99% reject of measurement noise is
the flywheel win), but pathological for a host that does NOT lock by
`min_age_s` (clkPoC3: still acquiring at 60 s; the tight gate then
rejects the catch-up corrections the loop needs → it never locks →
phase wanders ±600 ns → keeps rejecting).  Confirmed by the day0528
A/B: clkPoC3 with the v1 gate never locked (TDEV(1s)=1.74 ns, 93%
reject); without it, locked to 104 ps.

v2 adds two regimes selected at runtime:
  - *Acquiring*: gate is loose (pass-through by default; defers to the
    filter's chi²/√S gate which widens with P) so the loop can catch
    up.  This is also the post-bootstrap / post-holdover / post-
    recovery state.
  - *Locked*: the tight v1 gate (`K·σ_DO·√dt`) — reject measurement
    noise, coast on the flywheel.

### Lock detection — innovation bias-to-noise ratio (NOT P, NOT |innov|)

The regime is driven by the **innovation bias-to-noise ratio**
`|EMA(innov)| / √EMA(innov²)`, NOT by filter-internal state and NOT by
innovation magnitude:

  - Filter-internal estimates (x2, P) are untrustworthy here: when the
    gate over-rejects, the EKF goes OVERCONFIDENT — its x2 stays small
    (~3 ns) while the DO actually wanders ±600 ns.  P is also tuning-
    dependent (Q), so absolute P sentinels don't port across hosts.
  - Innovation *magnitude* is not the signal either: a locked loop has
    LARGE zero-mean innovations (measurement noise — 57 ns RMS on
    clkPoC3 locked), while a stuck loop has SMALLER but BIASED ones
    (13 ns RMS).
  - The bias-to-noise *ratio* is the portable, dimensionless, ground-
    truth discriminator (the gate only ever sees the innovation):
    locked ≈ 0.22, stuck ≈ 0.95 (day0528 clkPoC3).  Zero-mean residuals
    ⇒ the loop is tracking; a persistent bias ⇒ it's systematically
    behind.

Transitions use hysteresis + a dwell: acquiring→locked when the ratio
stays below `lock_bias_ratio` for `lock_dwell_s`; locked→acquiring
immediately when it exceeds `unlock_bias_ratio` (so a holdover/recovery
disturbance reopens the loose gate to re-acquire).

`regime_aware=False` (default) preserves exact v1 behavior for existing
callers/tests.

## What the gate does NOT do

- It does NOT modify the EKF.  An update that is rejected leaves x
  and P unchanged — the engine treats it like a chi² rejection.
- It does NOT replace the chi² gate.  Both run in sequence; either
  one rejecting the update is enough to skip.
- It does NOT learn σ_DO at runtime.  The floor is read once at
  construction from the state JSON.  Hosts without a characterized
  state JSON skip the gate entirely (returns "accept, no_floor").
"""
from __future__ import annotations

import json
import logging
import math
from pathlib import Path
from typing import Optional, Tuple

log = logging.getLogger(__name__)


class OcxoTrustedGate:
    """Per-arm observation gate anchored on DO's freerun short-τ noise."""

    def __init__(self, *,
                 sigma_short_tau_ns: float,
                 k_sigma: float = 10.0,
                 min_age_s: float = 60.0,
                 do_label: str = "",
                 regime_aware: bool = False,
                 k_sigma_acquire: Optional[float] = None,
                 lock_bias_ratio: float = 0.4,
                 unlock_bias_ratio: float = 0.7,
                 ema_alpha: float = 0.05,
                 lock_dwell_s: float = 30.0,
                 min_lock_samples: int = 30):
        """Construct a gate.

        Args:
            sigma_short_tau_ns: DO's characterized freerun TDEV at τ=1 s,
                in nanoseconds.  This is the physical floor — the DO
                cannot move faster than this on average.
            k_sigma: confidence multiplier for the LOCKED (tight) regime.
                Threshold = k × σ × √τ.  10 ≈ "an event 100× more
                powerful than typical OCXO noise".
            min_age_s: gate is disabled for the first `min_age_s`
                seconds of the EKF's life, so bootstrap convergence
                isn't impeded.
            do_label: optional human-readable name for log messages.
            regime_aware: enable the v2 acquiring/locked state machine.
                False (default) = exact v1 behavior (age-gated tight).
            k_sigma_acquire: K applied during the ACQUIRING regime.
                None (default) = pass-through (accept all; defer to the
                filter's chi²/√S gate).  A finite value applies a loose
                gate during acquisition.
            lock_bias_ratio: acquiring→locked when the innovation bias-
                to-noise ratio stays below this for `lock_dwell_s`.
            unlock_bias_ratio: locked→acquiring when the ratio exceeds
                this (hysteresis; > lock_bias_ratio).
            ema_alpha: EMA smoothing for the innovation mean/mean-square.
            lock_dwell_s: dwell below `lock_bias_ratio` before locking.
            min_lock_samples: minimum innovations seen before locking.
        """
        if sigma_short_tau_ns <= 0:
            raise ValueError(f"sigma_short_tau_ns must be > 0, got {sigma_short_tau_ns}")
        if k_sigma <= 0:
            raise ValueError(f"k_sigma must be > 0, got {k_sigma}")
        if not 0.0 < lock_bias_ratio <= unlock_bias_ratio:
            raise ValueError("require 0 < lock_bias_ratio <= unlock_bias_ratio")
        self.sigma_short_tau_ns = float(sigma_short_tau_ns)
        self.k_sigma = float(k_sigma)
        self.min_age_s = float(min_age_s)
        self.do_label = do_label
        self.regime_aware = bool(regime_aware)
        self.k_sigma_acquire = (None if k_sigma_acquire is None
                                else float(k_sigma_acquire))
        self.lock_bias_ratio = float(lock_bias_ratio)
        self.unlock_bias_ratio = float(unlock_bias_ratio)
        self.ema_alpha = float(ema_alpha)
        self.lock_dwell_s = float(lock_dwell_s)
        self.min_lock_samples = int(min_lock_samples)

        self.n_rejected = 0
        self.n_accepted = 0
        self.n_skipped_pre_age = 0
        self.n_acquiring = 0

        # Regime state machine.  Starts ACQUIRING; only `regime_aware`
        # ever leaves it.
        self.regime = "acquiring"
        self._ema_innov = 0.0
        self._ema_innov2 = 0.0
        self._n_seen = 0
        self._lock_cond_since: Optional[float] = None

    @property
    def bias_ratio(self) -> float:
        """|EMA(innov)| / √EMA(innov²) — the lock discriminator."""
        rms = math.sqrt(self._ema_innov2) if self._ema_innov2 > 0 else 0.0
        return abs(self._ema_innov) / rms if rms > 0 else 0.0

    def _tight_check(self, innov_ns: float, dt_s: float,
                     k: float, label: str) -> Tuple[bool, str]:
        # Expected OCXO phase movement over dt_s, in ns.  For τ ≥ 1,
        # τ scales by √τ for white-FM; we use √dt as a reasonable bound
        # that's tight at τ=1 s and loosens slowly for longer gaps.
        sigma_oc_dt = self.sigma_short_tau_ns * math.sqrt(max(1.0, dt_s))
        threshold_ns = k * sigma_oc_dt
        if abs(innov_ns) > threshold_ns:
            self.n_rejected += 1
            return False, (f"ocxo_gate[{self.regime}]: |innov|="
                           f"{abs(innov_ns):.2f} > {threshold_ns:.2f} ns")
        self.n_accepted += 1
        return True, label

    def _update_innov_stats(self, innov_ns: float) -> None:
        a = self.ema_alpha
        if self._n_seen == 0:
            self._ema_innov = innov_ns
            self._ema_innov2 = innov_ns * innov_ns
        else:
            self._ema_innov = (1 - a) * self._ema_innov + a * innov_ns
            self._ema_innov2 = (1 - a) * self._ema_innov2 + a * innov_ns * innov_ns
        self._n_seen += 1

    def _update_regime(self, age_s: float) -> None:
        br = self.bias_ratio
        if self.regime == "acquiring":
            if self._n_seen >= self.min_lock_samples and br < self.lock_bias_ratio:
                if self._lock_cond_since is None:
                    self._lock_cond_since = age_s
                elif age_s - self._lock_cond_since >= self.lock_dwell_s:
                    self.regime = "locked"
                    self._lock_cond_since = None
                    log.info("OCXO gate %s: ACQUIRING→LOCKED "
                             "(bias_ratio=%.2f < %.2f for %.0fs)",
                             self.do_label, br, self.lock_bias_ratio,
                             self.lock_dwell_s)
            else:
                self._lock_cond_since = None
        else:  # locked
            if br > self.unlock_bias_ratio:
                self.regime = "acquiring"
                self._lock_cond_since = None
                log.warning("OCXO gate %s: LOCKED→ACQUIRING "
                            "(bias_ratio=%.2f > %.2f — re-acquiring)",
                            self.do_label, br, self.unlock_bias_ratio)

    def evaluate(self, *,
                 innov_ns: float,
                 dt_s: float,
                 age_s: float) -> Tuple[bool, str]:
        """Return (accept, reason).

        Args:
            innov_ns: the measurement innovation, in ns.
            dt_s: time since last successful update (for OCXO drift integration).
            age_s: time since DOFreqEst init / bootstrap complete.
        """
        if not self.regime_aware:
            # v1: age-gated tight threshold, single regime.
            if age_s < self.min_age_s:
                self.n_skipped_pre_age += 1
                return True, "pre_min_age"
            return self._tight_check(innov_ns, dt_s, self.k_sigma, "accept")

        # v2: feed the innovation statistics on every evaluation, even
        # pre-age and while acquiring, so the lock detector warms up.
        self._update_innov_stats(innov_ns)
        if age_s < self.min_age_s:
            self.n_skipped_pre_age += 1
            self.regime = "acquiring"
            return True, "pre_min_age"

        self._update_regime(age_s)
        if self.regime == "acquiring":
            self.n_acquiring += 1
            if self.k_sigma_acquire is None:
                return True, "acquiring"  # pass-through; chi²/√S carries it
            return self._tight_check(innov_ns, dt_s,
                                     self.k_sigma_acquire, "acquiring_loose")
        return self._tight_check(innov_ns, dt_s, self.k_sigma, "locked")

    @property
    def stats(self) -> dict:
        return dict(
            n_rejected=self.n_rejected,
            n_accepted=self.n_accepted,
            n_skipped_pre_age=self.n_skipped_pre_age,
            n_acquiring=self.n_acquiring,
            sigma_short_tau_ns=self.sigma_short_tau_ns,
            k_sigma=self.k_sigma,
            min_age_s=self.min_age_s,
            do_label=self.do_label,
            regime_aware=self.regime_aware,
            regime=self.regime,
            bias_ratio=self.bias_ratio,
        )


def load_sigma_short_tau_from_state(state_json_path: Path,
                                    target_tau_s: int = 1) -> Optional[float]:
    """Read σ_DO at τ=1 s from a state/dos/<label>.json file.

    Supports two schemas:
      - do_freerun_char.py output: characterization.sources["DO PPS (chA-chB)"]
        or ["DO PPS (chA vs TICC Rb)"] → tdev_ns_by_tau_s["1.0"] or ["1"]
      - timehat offline analyzer (analyze_timehat_freerun_2026_05_27.py):
        characterization.tdev_ns["1"]

    Returns the TDEV at τ=1 s in ns, or None if the file or value isn't
    findable.
    """
    if not state_json_path.exists():
        log.warning("OCXO gate: state JSON not found: %s", state_json_path)
        return None
    try:
        j = json.loads(state_json_path.read_text())
    except (OSError, json.JSONDecodeError) as e:
        log.warning("OCXO gate: could not read %s: %s", state_json_path, e)
        return None
    c = j.get('characterization') or {}
    # Schema 1: do_freerun_char.py
    src = c.get('sources') or {}
    for source_key in ('DO PPS (chA vs TICC Rb)', 'DO PPS (chA-chB)'):
        s = src.get(source_key) or {}
        m = s.get('tdev_ns_by_tau_s') or {}
        for key in (str(target_tau_s), f"{target_tau_s}.0",
                    f"{float(target_tau_s):.1f}"):
            if key in m:
                v = m[key]
                if isinstance(v, (int, float)) and v > 0:
                    return float(v)
    # Schema 2: TimeHat analyzer
    m = c.get('tdev_ns') or {}
    for key in (str(target_tau_s), f"{target_tau_s}.0",
                f"{float(target_tau_s):.1f}"):
        if key in m:
            v = m[key]
            if isinstance(v, (int, float)) and v > 0:
                return float(v)
    log.warning("OCXO gate: no TDEV at τ=%d s in %s", target_tau_s, state_json_path)
    return None
