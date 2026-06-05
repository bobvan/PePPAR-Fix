"""Resolve PCV defaults at engine startup.

The engine has supported PCV correction since obs-model Phase 2, but
only when ``--antex-path`` and ``--receiver-antenna`` were both passed
on the command line.  Lab launch lines never passed them, so the entire
fleet ran with PCV silently off and accumulated meter-class position
bias (see I-053035-main, I-121024-main).

This module supplies the auto-discovery:

1. **Receiver antenna**: from ``timelab/antennas.json[arp_label].antenna``
   when not given explicitly.  Canonicalized to the ANTEX 20-char TYPE
   format.
2. **ANTEX file**: from a small fixed search list under
   ``support/antex/`` when not given explicitly.

Explicit CLI overrides still win.  Both successful resolution and
silent-disable cases log one line at engine startup so operators can
spot a misconfiguration without reading meters of position drift.
"""

from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass
from typing import Optional, Sequence

log = logging.getLogger(__name__)


# Status codes for the resolution outcome.  Operators see these in the
# single PCV log line emitted at startup.
ENABLED_FROM_DEFAULTS = "ENABLED_FROM_DEFAULTS"
ENABLED_FROM_CLI = "ENABLED_FROM_CLI"
DISABLED_NO_ARP = "DISABLED_NO_ARP"
DISABLED_NO_ANTENNA_FIELD = "DISABLED_NO_ANTENNA_FIELD"
DISABLED_PARSE_FAILED = "DISABLED_PARSE_FAILED"
DISABLED_NO_ATX_FILE = "DISABLED_NO_ATX_FILE"
DISABLED_NO_ANTENNAS_JSON = "DISABLED_NO_ANTENNAS_JSON"
DISABLED_BY_FLAG = "DISABLED_BY_FLAG"  # --no-pcv


# Search list for the ANTEX file.  Order matters: the repo-tracked
# copy (under ``support/antex/`` resolved from this module's location)
# wins first so resolution is independent of launch CWD — an engine
# launched from ``~/peppar-fix/data/``, a CI runner, a debug session,
# anywhere, still finds the tracked file.  This is the
# pcvSearchRootsRepoRootFallback fix (I-141542-main, follow-up to PR
# #135) — without it, a non-``~/peppar-fix/`` launch silently fell
# through to PCV-off, exactly the I-121024 failure mode.
#
# Then the CWD-relative entry (preserves the lab-convention launch
# from the repo root), the dedicated lab support dir under
# ``~/peppar-fix/``, and finally the transitional bare
# ``~/peppar-fix/`` entry — vestige of the pre-#135 era when
# ngs20.atx was deployed manually at the home-dir root.  Leave the
# vestigial entry until all lab hosts have been confirmed on the
# repo-tracked ``support/antex/`` files (then drop it).
ATX_SEARCH_NAMES = ("ngs20.atx", "igs20.atx", "igs14.atx")

# Repo root derived from this module's filesystem location:
# ``scripts/peppar_fix/antex_resolve.py`` → up 3 levels → repo root.
# Used so the search resolves the same way regardless of the engine's
# launch CWD.
_REPO_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.pardir, os.pardir))

ATX_SEARCH_ROOTS = (
    os.path.join(_REPO_ROOT, "support", "antex"),  # repo-tracked, CWD-independent
    "./support/antex",                              # lab-convention CWD-relative
    "~/peppar-fix/support/antex",                   # lab-host explicit
    "~/peppar-fix",                                 # transitional (pre-#135)
    ".",
)


@dataclass
class ResolvedPCV:
    """Result of resolve_pcv_defaults()."""
    antex_path: Optional[str]
    receiver_antenna: Optional[str]
    status: str
    detail: str
    # Provenance for the log line — "antennas.json[ufo1]" or "--receiver-antenna" etc.
    antenna_source: Optional[str] = None
    antex_source: Optional[str] = None

    @property
    def enabled(self) -> bool:
        return self.status.startswith("ENABLED_")


def _antenna_sanitized_filename(canonical: str) -> str:
    """Convert ``SFESPK6618H     NONE`` to ``SFESPK6618H_NONE.atx``."""
    parts = canonical.split()
    return "_".join(parts) + ".atx"


def _find_atx_file(
    canonical_antenna: Optional[str],
    search_roots: Sequence[str],
) -> tuple[Optional[str], list[str]]:
    """Return (path, tried) where path is the first existing ANTEX
    file in any of the search roots, or None if none found.

    Combined catalogs (ngs20.atx / igs20.atx / igs14.atx) are preferred
    over per-antenna extracts because compute_pcv_correction() needs
    BOTH the receiver-side pattern AND the satellite-side pattern for
    every observation it applies — a per-antenna file like
    ``support/antex/SFESPK6618H_NONE.atx`` is just the receiver block
    extracted from the composite catalog and has no satellite entries,
    so every per-SV lookup misses and pcv=0/N at runtime.  Per-antenna
    files remain in the search list as a last-resort fallback so PCV
    is at least receiver-side-aware when no combined catalog is
    deployed; that's still strictly better than silently-off.
    """
    tried: list[str] = []
    candidates: list[str] = []
    # Combined antex catalogs first (ngs20.atx / igs20.atx / igs14.atx).
    # These cover both the receiver-side and the satellite-side patterns
    # the engine needs at runtime.
    for root in search_roots:
        for name in ATX_SEARCH_NAMES:
            candidates.append(os.path.join(root, name))
    # Per-antenna files (e.g. ``SFESPK6618H_NONE.atx``) as last-resort
    # fallback.  No satellite entries → pcv=0/N for IF-combined corrections;
    # see docstring above.
    if canonical_antenna:
        per_antenna = _antenna_sanitized_filename(canonical_antenna)
        for root in search_roots:
            candidates.append(os.path.join(root, per_antenna))
    for c in candidates:
        full = os.path.expanduser(c)
        tried.append(full)
        if os.path.isfile(full):
            return full, tried
    return None, tried


def resolve_pcv_defaults(
    *,
    arp_label: Optional[str],
    antex_path_cli: Optional[str],
    receiver_antenna_cli: Optional[str],
    pcv_flag: bool = True,
    antennas_path: Optional[str] = None,
    search_roots: Optional[Sequence[str]] = None,
) -> ResolvedPCV:
    """Compute the effective PCV configuration.

    Parameters mirror what ``main()`` has on hand: the parsed
    ``--antex-path`` / ``--receiver-antenna`` CLI values (None if not
    passed), the ``arp_label`` from the per-host config TOML, and the
    ``--no-pcv`` master flag.

    Returns a :class:`ResolvedPCV` describing the resolved paths plus
    a status code suitable for one structured log line.
    """
    # Import lazily so unit tests can monkeypatch find_antennas_json
    # without dragging in the engine's position_state import chain.
    from peppar_fix import position_state as _pstate

    if not pcv_flag:
        return ResolvedPCV(None, None, DISABLED_BY_FLAG,
                           detail="--no-pcv passed on the command line")

    search = list(search_roots) if search_roots is not None else list(ATX_SEARCH_ROOTS)

    # ── Step 1: receiver antenna ────────────────────────────────────
    canonical_antenna: Optional[str] = None
    antenna_source: Optional[str] = None
    antenna_failure_detail: Optional[str] = None

    if receiver_antenna_cli:
        from antex import canonicalize_antex_type
        canonical_antenna = canonicalize_antex_type(receiver_antenna_cli)
        if canonical_antenna is None:
            return ResolvedPCV(
                None, None, DISABLED_PARSE_FAILED,
                detail=("--receiver-antenna value not parseable as ANTEX TYPE: "
                        f"{receiver_antenna_cli!r}"),
            )
        antenna_source = "--receiver-antenna"
    else:
        if not arp_label:
            return ResolvedPCV(
                None, None, DISABLED_NO_ARP,
                detail=("no --receiver-antenna and no arp_label configured. "
                        "Either pass --receiver-antenna or set arp_label in "
                        "the host config so antennas.json can be consulted."),
            )
        path = _pstate.find_antennas_json(antennas_path)
        if path is None:
            return ResolvedPCV(
                None, None, DISABLED_NO_ANTENNAS_JSON,
                detail=("no antennas.json found in any standard path; "
                        "either deploy timelab/antennas.json or pass "
                        "--receiver-antenna explicitly."),
            )
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            return ResolvedPCV(
                None, None, DISABLED_NO_ANTENNAS_JSON,
                detail=f"failed to read {path}: {e}",
            )
        entry = data.get(arp_label) or {}
        raw_antenna = entry.get("antenna")
        if not raw_antenna:
            return ResolvedPCV(
                None, None, DISABLED_NO_ANTENNA_FIELD,
                detail=(f"antennas.json[{arp_label}] has no 'antenna' field. "
                        "Add it (compact 'MODEL RADOME' form, e.g. "
                        "'SFESPK6618H NONE') or pass --receiver-antenna."),
            )
        from antex import canonicalize_antex_type
        canonical_antenna = canonicalize_antex_type(raw_antenna)
        if canonical_antenna is None:
            return ResolvedPCV(
                None, None, DISABLED_PARSE_FAILED,
                detail=(f"antennas.json[{arp_label}].antenna = {raw_antenna!r} "
                        "is free-text, not a canonical ANTEX TYPE. "
                        "Pass --receiver-antenna explicitly for this antenna."),
            )
        antenna_source = f"antennas.json[{arp_label}]"

    # ── Step 2: ANTEX file ──────────────────────────────────────────
    if antex_path_cli:
        if not os.path.isfile(os.path.expanduser(antex_path_cli)):
            return ResolvedPCV(
                None, None, DISABLED_NO_ATX_FILE,
                detail=f"--antex-path={antex_path_cli} does not exist",
            )
        antex_path = os.path.expanduser(antex_path_cli)
        antex_source = "--antex-path"
    else:
        antex_path, tried = _find_atx_file(canonical_antenna, search)
        if antex_path is None:
            return ResolvedPCV(
                None, canonical_antenna, DISABLED_NO_ATX_FILE,
                detail=("no ANTEX file found in any of: "
                        + ", ".join(tried) + ". Drop ngs20.atx (or a per-"
                        "antenna file) into support/antex/ or pass "
                        "--antex-path explicitly."),
            )
        antex_source = "auto-discovered"

    # ── Both resolved ───────────────────────────────────────────────
    status = (ENABLED_FROM_CLI
              if (antex_path_cli or receiver_antenna_cli)
              else ENABLED_FROM_DEFAULTS)
    return ResolvedPCV(
        antex_path=antex_path,
        receiver_antenna=canonical_antenna,
        status=status,
        detail="",
        antenna_source=antenna_source,
        antex_source=antex_source,
    )


def log_pcv_status(pcv: ResolvedPCV, logger: logging.Logger = log) -> None:
    """Emit one structured line summarizing the PCV resolution.

    INFO when enabled, WARNING when disabled.  Operators should be able
    to grep ``"PCV: "`` and see the active configuration at a glance.
    """
    if pcv.enabled:
        logger.info(
            "PCV: enabled — antenna=%r (%s), antex=%s (%s)",
            pcv.receiver_antenna, pcv.antenna_source,
            pcv.antex_path, pcv.antex_source,
        )
    else:
        logger.warning(
            "PCV: DISABLED [%s] — %s",
            pcv.status, pcv.detail,
        )
