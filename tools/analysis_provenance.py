#!/usr/bin/env python3
"""Shared analysis-provenance for the PPS-comparison campaign.

Bob is running a multi-clock PPS-comparison campaign (our clocks vs each
other + external clocks otcBob1 / M600 / GNSSDO+) graded on the moonshot
excursion bounds (|Δ| ≤ 1 ns @95% shared-antenna, 2 ns separate-antenna;
see docs/two-site-sync-budget.md).  Results must be comparable ACROSS
capture and analysis runs.  Capture and analysis tooling get DIFFERENT
treatment because of an asymmetry (see the feedback memo
``feedback_capture_recapture_analysis_version_stamp``):

  * CAPTURE tools define the DATA.  A capture-tool change invalidates
    cross-run comparability — you must RE-CAPTURE.  ``ticc_capture.py``
    carries its own ``CAPTURE_VERSION`` stamped into each CSV header.

  * ANALYSIS tools transform data → results.  Analysis can be re-run on
    the SAME captures, so the mitigation is provenance, not recapture:
    a deliberate version number STAMPED onto every output graphic + into
    stderr/result files, so a stale or mismatched figure is obvious.

``ANALYSIS_VERSION`` below is that deliberate integer.

    *** BUMP ``ANALYSIS_VERSION`` BY HAND WHENEVER THE ANALYSIS MATH
        CHANGES. ***

This is the single source of truth and Bob's "second thoughts" marker.
It is NOT auto-derived from the git SHA: the SHA changes on every commit
(docstrings, unrelated tools), but the *analysis math* changes rarely and
deliberately.  Examples of changes that REQUIRE a bump: the excursion
metric definition, the detrend method, the clean-window selection, the
CDF/TDEV/ADEV math, the pairing tolerance.  Examples that do NOT: a
docstring fix, a new CLI flag that doesn't touch the math, a plot colour,
a change to how the provenance STRING itself is computed (e.g. the
dirty-marker probe) — that's provenance plumbing, not analysis math.

The git short-SHA (+ ``-dirty`` marker) rides ALONGSIDE the version for
exactness — it pins the figure to a specific tree state.  The ``-dirty``
marker is load-bearing: a figure must never claim a clean version while
it was produced from uncommitted analysis edits.

Never compare two figures carrying different ``ANALYSIS_VERSION`` — re-run
the older captures through the CURRENT analysis version instead.
"""
from __future__ import annotations

import subprocess
from datetime import datetime, timezone
from pathlib import Path

# Deliberate analysis-math version.  BUMP BY HAND on any analysis-logic
# change.  See module docstring.
#
# Bump log:
#   v1 — initial campaign metrics (excursion CDF, per-clock TDEV/ADEV
#        stability stack, detrend + clean-window selection).
#   v2 — added the three-cornered-hat (TCH) decomposition (tools/plot_tch.py,
#        compare_clocks.py --tch): a new gradeable analysis method that
#        recovers each of {chA, chB, Rb}'s INDIVIDUAL stability past the
#        Rb-reference floor.  Pre-existing metrics are numerically
#        unchanged; the bump marks that a new analysis-math path now ships
#        under this provenance stamp.
ANALYSIS_VERSION = 2

# Repo root = parent of this module's directory (tools/ -> repo root).
_REPO_ROOT = Path(__file__).resolve().parent.parent


def _git(*args: str) -> str | None:
    """Run a git command rooted at the repo, return stripped stdout or
    None on any failure (git missing, not a repo, command error).
    Never raises."""
    try:
        out = subprocess.run(
            ["git", "-C", str(_REPO_ROOT), *args],
            capture_output=True, text=True, timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode != 0:
        return None
    return out.stdout


def _git_descriptor() -> str:
    """Return ``<shortsha>``, ``<shortsha>-dirty``, or ``nogit``.

    Degrades gracefully: any git problem (no git binary, not a working
    tree, detached/empty repo) yields ``nogit`` rather than raising.
    """
    sha_out = _git("rev-parse", "--short", "HEAD")
    if sha_out is None:
        return "nogit"
    sha = sha_out.strip()
    if not sha:
        return "nogit"
    # --untracked-files=no: count only TRACKED modifications (staged +
    # unstaged) toward dirtiness.  Untracked files (venv/, *.egg-info/,
    # state/, timelab/antennas.json) are ALWAYS present on lab hosts and
    # have nothing to do with whether the analysis CODE is uncommitted —
    # counting them made every figure stamp "-dirty" on every host (e.g.
    # PiPuss stamped "609e505-dirty" with pristine tracked code), defeating
    # the marker's purpose.
    status = _git("status", "--porcelain", "--untracked-files=no")
    if status is None:
        # We have a SHA but couldn't read status — report the SHA without
        # claiming clean (can't prove clean, so be honest).
        return f"{sha}-dirty"
    dirty = bool(status.strip())
    return f"{sha}-dirty" if dirty else sha


def analysis_provenance() -> str:
    """Return e.g. ``"v1 (a1b2c3d)"``, ``"v1 (a1b2c3d-dirty)"`` if the
    working tree has uncommitted changes, or ``"v1 (nogit)"`` when not in
    a git tree / git is unavailable."""
    return f"v{ANALYSIS_VERSION} ({_git_descriptor()})"


def provenance_line(toolname: str) -> str:
    """One-line provenance string for stderr / result-file echo.

    Same text the figure footer carries, so a result .txt and its PNG
    always agree."""
    now = datetime.now(timezone.utc).isoformat(timespec="seconds")
    return f"{toolname} · analysis {analysis_provenance()} · {now}"


def skip_comment_lines(fileobj):
    """Yield lines from ``fileobj`` with leading ``#`` comment lines
    dropped.

    ``ticc_capture.py`` (CAPTURE_VERSION ≥ 1) writes a ``# capture_tool=…``
    provenance line before the column header.  ``np.genfromtxt`` skips
    ``#`` lines by default, but ``csv.DictReader`` / ``csv.reader`` do
    NOT — so every csv-based TICC reader wraps its file handle in this
    generator before constructing the reader.  Only lines whose first
    non-whitespace character is ``#`` are dropped, so a real data row
    is never lost."""
    for line in fileobj:
        if line.lstrip().startswith('#'):
            continue
        yield line


def stamp(fig, toolname: str) -> None:
    """Add a small gray provenance footer to a matplotlib Figure.

    Placed on the bottom edge in small gray text so it doesn't overlap
    data.  Call immediately before ``fig.savefig``."""
    fig.text(
        0.005, 0.002, provenance_line(toolname),
        fontsize=6, color="0.5", ha="left", va="bottom",
    )
