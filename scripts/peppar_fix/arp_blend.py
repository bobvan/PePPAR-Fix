"""σ-weighted Bayesian ARP blend used by FixedPosFilter under --ar-mode wl.

Given a current pinned ARP (known_ecef) with prior uncertainty σ_pin
and an incoming refined position estimate (source_ecef) with
uncertainty σ_source, produce the posterior ARP and σ via a Kalman
scalar update on each ECEF component:

    α_eff = σ_pin² / (σ_pin² + σ_source²)
    ARP_new = ARP + α_eff × (source_ecef − ARP)
    σ_pin'  = σ_pin × σ_source / sqrt(σ_pin² + σ_source²)

A Mahalanobis-style 3σ outlier gate rejects implausible jumps:
    accept if ‖source_ecef − ARP‖ ≤ N × (σ_pin + σ_source)

The σ_pin update is then floored at σ_source to prevent the
over-confidence trap that otherwise emerges when successive
publications of the source are correlated (e.g. successive
AntPosEst publications share filter state, successive NAV2 fixes
share atmosphere/multipath).  The floor expresses "the time
filter can never be more confident in the ARP than the source it
learns from."  Discovered 2026-05-09 cold-start smoke on MadHat
(see dayplan I-125649 Stage 2 fix).

Designed to be source-agnostic.  Stage 4 sources from NAV2; future
restoration to AntPosEst (once systematic biases close) is a
caller-side flag flip with the math here unchanged.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


# Mahalanobis-style outlier gate width, in σ-units.  3σ rejects
# obviously broken updates while passing legitimate corrections
# during convergence (NAV2 → first AntPosEst publication can be
# 1-3 m delta legitimately when σ_pin is loose).
_DEFAULT_MAHALANOBIS_N_SIGMA = 3.0


@dataclass
class ArpBlendResult:
    """Outcome of one Bayesian ARP blend step."""

    arp_new: np.ndarray            # posterior ARP (ECEF, m)
    sigma_pin_new_m: float          # posterior σ_pin (m)
    action: str                     # "applied" or "rejected_outlier"
    alpha_eff: float                # Kalman scalar weight, in [0, 1]
    delta_3d_m: float               # ‖source_ecef − ARP‖ (m)
    gate_3sigma_m: float            # outlier gate threshold (m)
    step_3d_m: float                # ‖α_eff × delta‖ (m); 0 if rejected


def bayesian_arp_blend(
    known_ecef: np.ndarray,
    sigma_pin_m: float,
    source_ecef: np.ndarray,
    sigma_source_m: float,
    *,
    n_sigma_gate: float = _DEFAULT_MAHALANOBIS_N_SIGMA,
    floor_pin_at_source: bool = True,
) -> ArpBlendResult:
    """One σ-weighted Bayesian blend step.

    Args:
        known_ecef: current pinned ARP, shape (3,), ECEF metres.
        sigma_pin_m: prior σ_pin in metres.  Must be ≥ 0.
        source_ecef: incoming refined position, shape (3,), ECEF metres.
        sigma_source_m: σ of the source observation in metres.  Must be ≥ 0.
        n_sigma_gate: Mahalanobis outlier-gate width.  3σ default.
            Set to math.inf to disable the gate.
        floor_pin_at_source: when True (the default since 2026-05-09),
            the posterior σ_pin is floored at σ_source.  When False,
            the raw Kalman update is returned (useful for tests that
            verify the unfloored math, or in regimes where the source
            publications are genuinely independent and √N aggregation
            is justified).

    Returns:
        ArpBlendResult.  When action == "rejected_outlier" the
        arp_new and sigma_pin_new_m fields equal the prior values
        unchanged.

    Raises:
        ValueError on malformed inputs (wrong shape, negative σ).
    """
    known = np.asarray(known_ecef, dtype=float)
    src = np.asarray(source_ecef, dtype=float)
    if known.shape != (3,) or src.shape != (3,):
        raise ValueError(
            f"ECEF inputs must be shape (3,); got "
            f"known={known.shape} source={src.shape}"
        )
    if sigma_pin_m < 0 or sigma_source_m < 0:
        raise ValueError(
            f"σ values must be ≥ 0; got σ_pin={sigma_pin_m} "
            f"σ_source={sigma_source_m}"
        )

    delta = src - known
    delta_3d = float(np.linalg.norm(delta))

    # Mahalanobis-style 3σ outlier gate.  Combined σ is the test
    # scale; we use σ_pin + σ_source (sum, not RMS) as the
    # conservative bound — keeps the gate slightly looser than
    # strict Mahalanobis with diagonal Σ, defending against
    # bursts at the cost of a small false-accept rate.
    gate_m = n_sigma_gate * (sigma_pin_m + sigma_source_m)
    if math.isfinite(n_sigma_gate) and delta_3d > gate_m:
        return ArpBlendResult(
            arp_new=known,
            sigma_pin_new_m=sigma_pin_m,
            action="rejected_outlier",
            alpha_eff=0.0,
            delta_3d_m=delta_3d,
            gate_3sigma_m=gate_m,
            step_3d_m=0.0,
        )

    # Kalman scalar update.  Guard against the σ_source = 0 +
    # σ_pin = 0 degenerate case (both zero would divide-by-zero);
    # treat it as "perfect agreement, no movement, σ stays 0."
    s_pin_sq = sigma_pin_m ** 2
    s_src_sq = sigma_source_m ** 2
    denom = s_pin_sq + s_src_sq
    if denom <= 0:
        return ArpBlendResult(
            arp_new=known,
            sigma_pin_new_m=0.0,
            action="applied",
            alpha_eff=0.0,
            delta_3d_m=delta_3d,
            gate_3sigma_m=gate_m,
            step_3d_m=0.0,
        )

    alpha_eff = s_pin_sq / denom
    step = alpha_eff * delta
    arp_new = known + step
    step_3d = float(np.linalg.norm(step))

    sigma_pin_kalman = math.sqrt(s_pin_sq * s_src_sq / denom)
    if floor_pin_at_source:
        sigma_pin_new = max(sigma_pin_kalman, sigma_source_m)
    else:
        sigma_pin_new = sigma_pin_kalman

    return ArpBlendResult(
        arp_new=arp_new,
        sigma_pin_new_m=sigma_pin_new,
        action="applied",
        alpha_eff=alpha_eff,
        delta_3d_m=delta_3d,
        gate_3sigma_m=gate_m,
        step_3d_m=step_3d,
    )
