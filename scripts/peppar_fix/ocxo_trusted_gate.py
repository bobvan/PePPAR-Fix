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

## Two regimes (future)

The current v1 has a single threshold.  A future v2 will add:
  - *Transient*: loose gate (e.g., K=100) during bootstrap, post-
    holdover, post-recovery — let convergence proceed.
  - *Converged*: tight gate after the EKF's P22+P33 have settled
    below sentinel values for N seconds.

For v1 we use a fixed `min_age_s` after which the gate engages, with
the gate disabled before that.  Sufficient to validate the concept
against the disc-study captures.

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
                 is_sole_observer: bool = False):
        """Construct a gate.

        Args:
            sigma_short_tau_ns: DO's characterized freerun TDEV at τ=1 s,
                in nanoseconds.  This is the physical floor — the DO
                cannot move faster than this on average.
            k_sigma: confidence multiplier.  Threshold = k × σ × √τ.
                10 is the v1 default — corresponds to "an event 100×
                more powerful than typical OCXO noise".  Lower values
                reject more; higher values let more through.
            min_age_s: legacy age-based engagement; gate is disabled
                for the first `min_age_s` seconds.  Used only when
                evaluate() is called WITHOUT distance_to_lock (the
                continuous path supersedes it — disciplineModeFsm #3).
            do_label: optional human-readable name for log messages.
            is_sole_observer: when True, this arm is the only carrier
                of its filter state on this host; the gate ALWAYS
                admits (Main's observability refinement on #89, and
                the sole-observer memory — soft-weighting / rejecting
                a sole carrier starves the filter).  Determined
                config-side by the engine (TICC is sole observer of
                x[2] when neither EXTINT nor TDCP-holdover is active).
                Set ONCE at construction — both predicates are
                config-bound (EXTINT presence comes from the host's
                extint_store wiring; TDCP-holdover from
                --servo-input).  Runtime degradation (e.g. EXTINT
                loses lock mid-run, leaving TICC truly alone) is a
                follow-up; today the runtime answer matches the
                construction-time answer for the lifetime of the run.
        """
        if sigma_short_tau_ns <= 0:
            raise ValueError(f"sigma_short_tau_ns must be > 0, got {sigma_short_tau_ns}")
        if k_sigma <= 0:
            raise ValueError(f"k_sigma must be > 0, got {k_sigma}")
        self.sigma_short_tau_ns = float(sigma_short_tau_ns)
        self.k_sigma = float(k_sigma)
        self.min_age_s = float(min_age_s)
        self.do_label = do_label
        self.is_sole_observer = bool(is_sole_observer)
        self.n_rejected = 0
        self.n_accepted = 0
        self.n_skipped_pre_age = 0
        self.n_admitted_sole_observer = 0
        self.n_admitted_continuous_off = 0

    def evaluate(self, *,
                 innov_ns: float,
                 dt_s: float,
                 age_s: Optional[float] = None,
                 distance_to_lock: Optional[float] = None,
                 ) -> Tuple[bool, str]:
        """Return (accept, reason).

        Args:
            innov_ns: the measurement innovation, in ns.
            dt_s: time since last successful update.
            age_s: legacy engagement signal (seconds since bootstrap).
                Used only when ``distance_to_lock`` is None.
            distance_to_lock: derived continuous convergence signal in
                [0, 1] (0=locked, 1=far) — supersedes age_s when set.
                Engagement = max(0, 1 − m): at m=0 the threshold is
                today's k·σ·√dt; at m=0.5 it doubles; the
                ``1/engagement`` scaling grows rapidly near m=1
                (e.g. ~10× at m=0.9, ~100× at m=0.99) by design — the
                "gate mostly off pre-lock" intent is "asymptotically
                off," not a sharp transition.  At m=1 exactly the
                early-return below admits unconditionally — the
                "converging = no rejection, tracking = gate on"
                continuum, done continuously (SatPulse 0.2, our own
                clkpoc3GateOverRejectsBeforeLock finding).
        """
        # Observability override: a sole-carrier arm cannot be
        # soft-weighted or rejected without starving its state.
        if self.is_sole_observer:
            self.n_admitted_sole_observer += 1
            return True, "sole_observer"

        # Engagement: continuous (m) takes precedence over legacy
        # (age_s).  Backward-compat: callers passing only age_s get
        # today's behavior.
        if distance_to_lock is not None:
            engagement = max(0.0, 1.0 - float(distance_to_lock))
            if engagement <= 0.0:
                self.n_admitted_continuous_off += 1
                return True, "off_far_from_lock"
            threshold_scale = 1.0 / engagement
        elif age_s is not None and age_s < self.min_age_s:
            self.n_skipped_pre_age += 1
            return True, "pre_min_age"
        else:
            threshold_scale = 1.0

        # Expected OCXO phase movement over dt_s.  τ scales by √τ for
        # white-FM; √dt is a tight-at-τ=1-s bound that loosens slowly.
        sigma_oc_dt = self.sigma_short_tau_ns * math.sqrt(max(1.0, dt_s))
        threshold_ns = self.k_sigma * sigma_oc_dt * threshold_scale
        if abs(innov_ns) > threshold_ns:
            self.n_rejected += 1
            return False, f"ocxo_gate: |innov|={abs(innov_ns):.2f} > {threshold_ns:.2f} ns"
        self.n_accepted += 1
        return True, "accept"

    @property
    def stats(self) -> dict:
        return dict(
            n_rejected=self.n_rejected,
            n_accepted=self.n_accepted,
            n_skipped_pre_age=self.n_skipped_pre_age,
            n_admitted_sole_observer=self.n_admitted_sole_observer,
            n_admitted_continuous_off=self.n_admitted_continuous_off,
            sigma_short_tau_ns=self.sigma_short_tau_ns,
            k_sigma=self.k_sigma,
            min_age_s=self.min_age_s,
            do_label=self.do_label,
            is_sole_observer=self.is_sole_observer,
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
