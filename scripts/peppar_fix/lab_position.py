"""Load a reference antenna position for lab analysis tools.

Production / steady-state code does NOT use this — it reads the
last-known position from `state/receivers/<uid>.json` written by the
PPP bootstrap or the `--known-pos` CLI flag.

The helper here is only for diagnostic / analysis tools that need a
quick reference ARP to compare residuals against.  Coordinates never
live in the committed repo; they come from one of:

  1. The `ANTENNA_POS_LLA` environment variable, formatted
     "<lat>,<lon>,<alt_m>".
  2. `~/git/timelab/antennas.json` (gitignored in the timelab repo),
     with a keyed structure:
         {
           "ufo1": {"lat": ..., "lon": ..., "alt_m": ...},
           ...
         }

If neither source yields a position, the caller is raised out with
a clear diagnostic.  Analysis tools should call this and let the
SystemExit propagate — it's better to surface missing-config loudly
than to silently run against a stale hardcoded value.
"""

from __future__ import annotations

import json
import os
import sys
from typing import Optional, Tuple

_DEFAULT_KEY = 'ufo1'
_DEFAULT_PATHS = (
    os.path.expanduser('~/git/timelab/antennas.json'),
    '/home/bob/git/timelab/antennas.json',
    os.path.expanduser('~/git/timelab/antPos.json'),
    '/home/bob/git/timelab/antPos.json',
)


def load_lab_position(key: str = _DEFAULT_KEY) -> Tuple[float, float, float]:
    """Return (lat_deg, lon_deg, alt_m) for the named antenna.

    Raises SystemExit with an actionable message if no source is
    available.
    """
    env = os.environ.get('ANTENNA_POS_LLA')
    if env:
        parts = env.split(',')
        if len(parts) != 3:
            raise SystemExit(
                f"ANTENNA_POS_LLA must be 'lat,lon,alt' (got {env!r})")
        return tuple(float(p) for p in parts)  # type: ignore[return-value]

    for path in _DEFAULT_PATHS:
        if os.path.exists(path):
            return _from_json(path, key)

    raise SystemExit(
        f"No antenna position available.  Either set "
        f"ANTENNA_POS_LLA='lat,lon,alt' in the environment, or create "
        f"{_DEFAULT_PATHS[0]} with a '{key}' entry (see docstring).")


def _from_json(path: str, key: str) -> Tuple[float, float, float]:
    with open(path) as f:
        data = json.load(f)
    entry = data.get(key)
    if entry is None:
        raise SystemExit(
            f"{path} has no '{key}' entry.  Keys present: "
            f"{sorted(data.keys())}")
    try:
        return float(entry['lat']), float(entry['lon']), float(entry['alt_m'])
    except (KeyError, TypeError, ValueError) as e:
        raise SystemExit(
            f"{path}['{key}'] malformed ({e}); expected "
            f"{{'lat': .., 'lon': .., 'alt_m': ..}}")


def try_load_lab_position(key: str = _DEFAULT_KEY
                          ) -> Optional[Tuple[float, float, float]]:
    """Like load_lab_position but returns None instead of raising."""
    try:
        return load_lab_position(key)
    except SystemExit:
        return None
