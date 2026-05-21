"""R-matrix calibration for FixedPosFilter — data-driven observation noise.

Background:  The default observation-noise values in solve_ppp.py are:

    SIGMA_P_IF = 3.0           # PR-IF rows, scaled by 1/w (cno + elev)
    sigma_td   = 0.3 / max(0.2, elev_factor)   # TD-CP rows

Both are global constants without per-system or per-station tuning.  The
filterStateLog finding (2026-05-21 PiFace) showed the filter is 9.1x
overconfident at the clock state: P[CLK,CLK]_post = (45 mm)^2 but actual
TDEV(1s) of x[CLK] = 1.39 ns ≈ 415 mm.  Empirical post-fit residual std
shows the TD-CP rows are particularly mis-sized (~6.6 mm at zenith vs
the 350 mm sigma_td model — 53x oversize).

This module loads a per-host calibration TOML and exposes elev-binned
sigma_pr(sys, elev) and sigma_td(sys, elev) functions.  When no
calibration is loaded, defaults match the existing behavior.

TOML format (`state/receivers/<uid>.r_calibration.toml`):

    [gps]
    elev_bins_deg = [10, 20, 30, 40, 50, 60, 70, 90]
    sigma_pr_m    = [2.50, 2.00, 1.60, 1.20, 0.80, 0.71, 0.71]
    sigma_td_m    = [0.019, 0.047, 0.024, 0.025, 0.015, 0.007]
    [gal]
    elev_bins_deg = [10, 20, 30, 40, 50, 60, 70, 90]
    sigma_pr_m    = [6.50, 7.30, 2.00, 1.30, 1.20, 0.85, 0.84]
    sigma_td_m    = [0.019, 0.030, 0.032, 0.036, 0.007, 0.010, 0.010]

The lists are bin-aligned: sigma_pr_m[i] applies to elevs in [bins[i], bins[i+1]).
Defaults are matched to the existing SIGMA_P_IF/w and 0.3/elev_factor formulas.
"""
from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# Defaults derived from the current FixedPosFilter constants at elev_factor=1
# and cno=45 dBHz (w = cno_factor * elev_factor = 3.16 * 1.0 = 3.16).
# Effective default sigma_pr = SIGMA_P_IF / w = 3.0 / 3.16 ~= 0.95 m at
# zenith for cno=45.  We bake in a conservative fallback per elev bin.
_DEFAULT_BINS = [10, 20, 30, 40, 50, 60, 70, 90]


def _default_sigma_pr_m(elev_deg: float) -> float:
    """Default matches the existing SIGMA_P_IF/elev_factor model, EXCLUDING
    the cno_factor (that gets applied separately per-SV by the caller).

    σ_pr(elev) = SIGMA_P_IF / sin(elev) = 3.0 / sin(elev)
    Floored at sin(5°) ≈ 0.087 → max value ≈ 34.4 m at elev <= 5°.
    """
    s = max(math.sin(math.radians(max(5.0, elev_deg))), 0.087)
    return 3.0 / s


def _default_sigma_td_m(elev_deg: float) -> float:
    """Default matches the existing in-filter formula sigma_td = 0.3 / elev_factor.

    σ_td(elev) = 0.3 / max(0.2, sin(elev))
    """
    return 0.3 / max(0.2, math.sin(math.radians(max(5.0, elev_deg))))


@dataclass(frozen=True)
class _SystemModel:
    """Elev-binned sigma model for one constellation.

    Bins are right-open: bin i applies to [edges[i], edges[i+1]).
    """
    edges: tuple[float, ...]
    sigma_pr_m: tuple[float, ...]
    sigma_td_m: tuple[float, ...]

    def lookup_pr(self, elev_deg: float) -> Optional[float]:
        return self._lookup(elev_deg, self.sigma_pr_m)

    def lookup_td(self, elev_deg: float) -> Optional[float]:
        return self._lookup(elev_deg, self.sigma_td_m)

    def _lookup(self, elev_deg: float,
                table: tuple[float, ...]) -> Optional[float]:
        if not table:
            return None
        for i in range(len(self.edges) - 1):
            if self.edges[i] <= elev_deg < self.edges[i + 1]:
                if i < len(table):
                    v = table[i]
                    return float(v) if v > 0 else None
                return None
        # Outside top edge — use last bin
        if elev_deg >= self.edges[-1] and len(table) > 0:
            v = table[-1]
            return float(v) if v > 0 else None
        return None


class RCalibration:
    """Per-host R-matrix model.

    Use sigma_pr_m(sys, elev_deg) and sigma_td_m(sys, elev_deg) to read
    observation noise σ in meters.  Returns calibrated values when a
    TOML was loaded; otherwise falls back to the legacy default formulas.

    Floor values prevent runaway numerical issues when a calibration
    bin happens to sit on a near-zero std (e.g. a single-SV bin with
    little spread):

        σ_pr ≥ pr_floor_m  (default 0.05 m)
        σ_td ≥ td_floor_m  (default 0.003 m, 1 cm of carrier-phase noise)
    """

    def __init__(self, models: dict[str, _SystemModel] | None = None,
                 pr_floor_m: float = 0.05,
                 td_floor_m: float = 0.003,
                 path: Optional[Path] = None):
        self._models = models or {}
        self.pr_floor_m = float(pr_floor_m)
        self.td_floor_m = float(td_floor_m)
        self.path = path

    @classmethod
    def from_toml(cls, path: Path) -> "RCalibration":
        """Load calibration from a TOML file.

        Missing entries fall back to defaults.  Use this for per-host
        deployment.
        """
        with open(path, "rb") as f:
            data = tomllib.load(f)
        models: dict[str, _SystemModel] = {}
        for sys_key in ('gps', 'gal', 'bds'):
            if sys_key not in data:
                continue
            sec = data[sys_key]
            edges = tuple(float(x) for x in sec.get('elev_bins_deg', ()))
            sigma_pr = tuple(float(x) for x in sec.get('sigma_pr_m', ()))
            sigma_td = tuple(float(x) for x in sec.get('sigma_td_m', ()))
            if len(edges) < 2:
                continue
            models[sys_key] = _SystemModel(edges, sigma_pr, sigma_td)
        pr_floor = float(data.get('pr_floor_m', 0.05))
        td_floor = float(data.get('td_floor_m', 0.003))
        return cls(models=models, pr_floor_m=pr_floor,
                   td_floor_m=td_floor, path=path)

    def sigma_pr_m(self, sys_name: str, elev_deg: float) -> float:
        m = self._models.get(sys_name)
        val: Optional[float] = None
        if m is not None:
            val = m.lookup_pr(elev_deg)
        if val is None:
            val = _default_sigma_pr_m(elev_deg)
        return max(val, self.pr_floor_m)

    def sigma_td_m(self, sys_name: str, elev_deg: float) -> float:
        m = self._models.get(sys_name)
        val: Optional[float] = None
        if m is not None:
            val = m.lookup_td(elev_deg)
        if val is None:
            val = _default_sigma_td_m(elev_deg)
        return max(val, self.td_floor_m)
