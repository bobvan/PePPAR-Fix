"""NL attempt diagnostic — per-SV float-ambiguity quality logger.

When attached to `NarrowLaneResolver`, emits one `[NL_DIAG]` line per
SV per NL attempt (LAMBDA and rounding paths both) plus a per-batch
`[NL_DIAG_BATCH]` line for each LAMBDA batch resolution.  Purpose: see
what the float solution looks like *at the moment NL tries to fix it*,
which is the only place the three ptpmon hypotheses
(LAMBDA-too-strict / biased-float / geometry-weak) can be told apart.

Log format is deliberately grep-friendly and flat — one record per
line, key=value pairs.  Fields are optional; only those known at the
decision point are included.  Example output::

    [NL_DIAG] epoch=1234 sv=E23 az=178 elev=72 frac=0.083 sigma=0.124 wl_fixed=6 result=CAND
    [NL_DIAG] epoch=1234 sv=E06 elev=18 wl_fixed=6 result=SKIP_ELEV (below 20° AR mask)
    [NL_DIAG] epoch=1234 sv=E13 az=294 elev=41 frac=0.287 sigma=0.310 wl_fixed=6 result=SKIP_PRESCREEN
    [NL_DIAG_BATCH] epoch=1234 n=5 ratio=1.8 p_bootstrap=0.887 result=REJECT_LAMBDA_RATIO

Off by default.  Enable with `--nl-diag` on the engine CLI, or
`NlDiagLogger(enabled=True)` directly.  When disabled the resolver's
calls into this module are cheap no-ops.

Design notes:
- The logger accumulates records over a single `attempt()` invocation
  and emits them all in one go at the end.  This lets LAMBDA fill in
  the ratio / P_bootstrap / outcome on each candidate's record after
  the batch resolves, without a second pass.
- Stateless between attempts.  No thresholds to tune — it reports,
  doesn't judge.  Downstream analysis (probably a small awk
  aggregator) picks the hypothesis.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional

log = logging.getLogger(__name__)


# Canonical result strings.  Kept as plain strings (not an enum) so
# downstream grep/awk can match them exactly.  Adding new results is
# additive — no exhaustive-match consumers.
RESULT_CAND            = "CAND"            # candidate listed, batch not yet resolved
RESULT_FIXED_LAMBDA    = "FIXED_LAMBDA"
RESULT_FIXED_ROUNDING  = "FIXED_ROUNDING"
RESULT_SKIP_ELEV       = "SKIP_ELEV"
RESULT_SKIP_BLACKLIST  = "SKIP_BLACKLIST"
RESULT_SKIP_NO_WL      = "SKIP_NO_WL"
RESULT_SKIP_NO_FREQS   = "SKIP_NO_FREQS"
RESULT_SKIP_PRESCREEN  = "SKIP_PRESCREEN"
RESULT_REJ_LAMBDA_RATIO       = "REJECT_LAMBDA_RATIO"
RESULT_REJ_LAMBDA_BOOTSTRAP   = "REJECT_LAMBDA_BOOTSTRAP"
RESULT_REJ_LAMBDA_DISPLACEMENT = "REJECT_LAMBDA_DISPLACEMENT"
RESULT_REJ_CORNER      = "REJECT_CORNER"
RESULT_REJ_RECT        = "REJECT_RECT"


@dataclass
class _Record:
    """Single per-SV NL diagnostic line.  All fields optional; only
    those known at the decision point are set.  A few convenience
    setters below update specific subsets (e.g. on LAMBDA resolution)."""
    epoch: int = 0
    sv: str = "?"
    result: str = "?"
    reason: str = ""                            # optional free-text extension
    az_deg: Optional[float] = None
    elev_deg: Optional[float] = None
    wl_fixed_count: Optional[int] = None
    n1_frac: Optional[float] = None             # |fractional part| of float N1
    sigma_n1_cyc: Optional[float] = None        # ambiguity σ in cycles
    lambda_ratio: Optional[float] = None
    lambda_p_bootstrap: Optional[float] = None
    corner_margin_sum: Optional[float] = None   # rounding-path sum (0..2.0)
    blacklist_remaining: Optional[int] = None


class NlDiagLogger:
    """Accumulates NL diagnostic records over one NarrowLaneResolver.attempt()
    invocation and emits them together.

    Thread-model: callers run serially in AntPosEstThread — no lock.

    Lifecycle per attempt():
        - begin()            at start of attempt
        - record(sv=, ...)   called many times as candidates are evaluated
        - update(sv=, ...)   merge fields into the record for sv
        - set_lambda_batch_result(svs, ratio=, p=, result=)
                             mass-apply LAMBDA batch outcome to listed SVs
        - set_lambda_batch_summary(n=, ratio=, p=, result=)
                             emit a separate [NL_DIAG_BATCH] line
        - emit()             flush all accumulated records; called at end
    """

    # Public flag — flipped by CLI or programmatic toggle.  When False
    # every call below is an O(1) early-return; no records are built.
    def __init__(self, enabled: bool = False) -> None:
        self.enabled = bool(enabled)
        self._records: dict[str, _Record] = {}
        self._batch_summary: Optional[dict] = None
        self._epoch: int = 0

    # ── Lifecycle ────────────────────────────────────────────────── #

    def begin(self, epoch: int) -> None:
        if not self.enabled:
            return
        self._records.clear()
        self._batch_summary = None
        self._epoch = int(epoch)

    def emit(self) -> None:
        """Flush records + optional batch summary as single-line INFO logs."""
        if not self.enabled:
            return
        for rec in self._records.values():
            log.info(self._format(rec))
        if self._batch_summary is not None:
            bs = self._batch_summary
            frags = [f"epoch={self._epoch}"]
            for k in ("n", "ratio", "p_bootstrap", "result"):
                if bs.get(k) is not None:
                    if isinstance(bs[k], float):
                        frags.append(f"{k}={bs[k]:.4f}" if k == "p_bootstrap"
                                     else f"{k}={bs[k]:.3f}")
                    else:
                        frags.append(f"{k}={bs[k]}")
            log.info("[NL_DIAG_BATCH] " + " ".join(frags))
        self._records.clear()
        self._batch_summary = None

    # ── Per-record API ──────────────────────────────────────────── #

    def record(self, **kwargs) -> None:
        """Upsert a record for the given sv (required kwarg).

        If the sv already has a record this epoch, kwargs are merged
        (None values do not overwrite).  Typical usage: a first call
        sets elev + result=SKIP_*, or a later candidate-loop call sets
        frac/sigma + result=CAND.
        """
        if not self.enabled:
            return
        sv = kwargs.pop("sv", None)
        if sv is None:
            return
        rec = self._records.get(sv)
        if rec is None:
            rec = _Record(epoch=self._epoch, sv=sv)
            self._records[sv] = rec
        for k, v in kwargs.items():
            if v is None:
                continue
            setattr(rec, k, v)

    def update(self, sv: str, **kwargs) -> None:
        """Alias for record(sv=sv, ...) — intent-revealing for merges."""
        self.record(sv=sv, **kwargs)

    # ── LAMBDA-specific helpers ─────────────────────────────────── #

    def set_lambda_batch_result(
        self,
        svs: list[str],
        *,
        ratio: Optional[float] = None,
        p_bootstrap: Optional[float] = None,
        result: str,
    ) -> None:
        """Apply a LAMBDA batch outcome to every listed SV's record."""
        if not self.enabled:
            return
        for sv in svs:
            rec = self._records.get(sv)
            if rec is None:
                continue
            if ratio is not None:
                rec.lambda_ratio = float(ratio)
            if p_bootstrap is not None:
                rec.lambda_p_bootstrap = float(p_bootstrap)
            rec.result = result

    def set_lambda_batch_summary(
        self,
        *,
        n: Optional[int] = None,
        ratio: Optional[float] = None,
        p_bootstrap: Optional[float] = None,
        result: str,
    ) -> None:
        """Queue a [NL_DIAG_BATCH] line to be emitted alongside records."""
        if not self.enabled:
            return
        self._batch_summary = {
            "n": n, "ratio": ratio, "p_bootstrap": p_bootstrap, "result": result,
        }

    # ── Formatting ──────────────────────────────────────────────── #

    @staticmethod
    def _format(rec: _Record) -> str:
        """Render one record as a single grep-friendly line."""
        parts = [f"epoch={rec.epoch}", f"sv={rec.sv}"]
        if rec.az_deg is not None:
            parts.append(f"az={rec.az_deg:.0f}")
        if rec.elev_deg is not None:
            parts.append(f"elev={rec.elev_deg:.0f}")
        if rec.n1_frac is not None:
            parts.append(f"frac={rec.n1_frac:.3f}")
        if rec.sigma_n1_cyc is not None:
            parts.append(f"sigma={rec.sigma_n1_cyc:.3f}")
        if rec.lambda_ratio is not None:
            parts.append(f"ratio={rec.lambda_ratio:.3f}")
        if rec.lambda_p_bootstrap is not None:
            parts.append(f"p_bootstrap={rec.lambda_p_bootstrap:.4f}")
        if rec.corner_margin_sum is not None:
            parts.append(f"corner={rec.corner_margin_sum:.3f}")
        if rec.wl_fixed_count is not None:
            parts.append(f"wl_fixed={rec.wl_fixed_count}")
        if rec.blacklist_remaining is not None:
            parts.append(f"bl_rem={rec.blacklist_remaining}")
        parts.append(f"result={rec.result}")
        if rec.reason:
            parts.append(f"reason={rec.reason!r}")
        return "[NL_DIAG] " + " ".join(parts)
