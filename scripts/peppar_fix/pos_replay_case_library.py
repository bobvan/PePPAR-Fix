"""pos_replay case-library batch runner (manifest §7).

Iterates a curated set of reference-capture replays — calm / active /
GPS+GAL-fails regimes, each with truth — scoring each through the **real**
position filter (``run_pos_replay``).  This is the orchestration layer where the
arc's diagnostics are exercised at scale: same bundle captured vs product-swapped
(corrections vs filter), DivergenceMonitor verdicts per case.

Per-bundle guard (Charlie #244 note 2): ``_process_epoch`` and the inline
``epoch_sink`` are deliberately *unwrapped* — they propagate, faithfully
matching the live ``_run_inner``.  The guard belongs HERE, at the batch layer:
one malformed bundle/epoch is caught, recorded, and the sweep continues, so a
bad case can't sink the whole library run.

Logging note (Charlie #244 note 1): ``PppStateCapture`` raises the shared
``peppar-fix`` logger to INFO for each replay's capture window — under a richer
ambient logging config that transiently bumps any other handlers on that logger.
Inherent (the logger level gates before handlers); benign for a standalone
library run.
"""
from __future__ import annotations

import os
import sys
from typing import Optional

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix.pos_replay_filter import run_pos_replay  # noqa: E402


def run_case(case: dict, *, default_args=None) -> dict:
    """Run one case through the filter, guarded.  ``case`` keys:
    ``name``, ``bundle_dir``, ``known_ecef`` (required); ``truth`` (StaticTruth,
    enables the position score); ``corrections_loader`` (→ product-swap);
    ``args``.  Returns a result dict with ``status`` ``ok``/``failed``."""
    name = case.get("name") or case.get("bundle_dir", "?")
    try:
        res = run_pos_replay(
            case["bundle_dir"], case["known_ecef"],
            systems=case.get("systems"),
            truth=case.get("truth"),
            corrections_loader=case.get("corrections_loader"),
            args=case.get("args") or default_args)
    except Exception as e:                       # noqa: BLE001 — batch guard
        return {"name": name, "status": "failed",
                "error": f"{type(e).__name__}: {e}"}
    entry = {
        "name": name,
        "status": "ok",
        "n_epochs": res["n_epochs_decoded"],
        "n_ppp_state": len(res["ppp_state_lines"]),
        "product_swapped": res["product_swapped"],
    }
    if "position" in res:
        v = res["position"].get("verdict") or {}
        entry["position_diverged"] = bool(v.get("fired"))
        entry["position_inconclusive"] = (
            res["position"]["n"] < res["position"]["window"])
    return entry


def run_case_library(cases, *, default_args=None) -> dict:
    """Run a list of cases (see :func:`run_case`), one bad case not sinking the
    rest.  Returns ``{"results": [...], "n_ok", "n_failed", "n_diverged"}``."""
    results = [run_case(c, default_args=default_args) for c in cases]
    return {
        "results": results,
        "n_ok": sum(1 for r in results if r["status"] == "ok"),
        "n_failed": sum(1 for r in results if r["status"] == "failed"),
        "n_diverged": sum(1 for r in results if r.get("position_diverged")),
    }


def format_summary(summary: dict) -> str:
    """A one-line-per-case human summary of a :func:`run_case_library` result."""
    lines = [f"case-library: {summary['n_ok']} ok, {summary['n_failed']} failed, "
             f"{summary['n_diverged']} diverged"]
    for r in summary["results"]:
        if r["status"] == "failed":
            lines.append(f"  ✗ {r['name']}: {r['error']}")
        else:
            verdict = ("DIVERGED" if r.get("position_diverged")
                       else "inconclusive" if r.get("position_inconclusive")
                       else "in-corridor" if "position_diverged" in r
                       else "no-score")
            swap = " [swapped]" if r["product_swapped"] else ""
            lines.append(f"  ✓ {r['name']}{swap}: {r['n_epochs']} epochs, "
                         f"{r['n_ppp_state']} [PPP_STATE], {verdict}")
    return "\n".join(lines)
