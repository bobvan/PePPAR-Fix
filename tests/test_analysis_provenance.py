"""Unit tests for tools/analysis_provenance.py (PPS-comparison provenance)."""
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_TOOLS = Path(__file__).resolve().parent.parent / "tools"
sys.path.insert(0, str(_TOOLS))

import analysis_provenance as ap  # noqa: E402


def test_analysis_version_present_and_int():
    assert isinstance(ap.ANALYSIS_VERSION, int)
    assert ap.ANALYSIS_VERSION >= 1


def test_analysis_provenance_format():
    s = ap.analysis_provenance()
    # v<N> (<descriptor>) — descriptor is a 7-ish hex sha, sha-dirty, or nogit.
    m = re.fullmatch(r"v(\d+) \((.+)\)", s)
    assert m, f"unexpected provenance format: {s!r}"
    assert int(m.group(1)) == ap.ANALYSIS_VERSION
    descriptor = m.group(2)
    assert (descriptor == "nogit"
            or re.fullmatch(r"[0-9a-f]+(-dirty)?", descriptor)), descriptor


def test_git_descriptor_sha_form(monkeypatch):
    """Clean tree → bare sha (no -dirty)."""
    def fake_git(*args):
        if args[0] == "rev-parse":
            return "abc1234\n"
        if args[0] == "status":
            return ""   # clean
        return None
    monkeypatch.setattr(ap, "_git", fake_git)
    assert ap._git_descriptor() == "abc1234"
    assert ap.analysis_provenance() == f"v{ap.ANALYSIS_VERSION} (abc1234)"


def test_git_descriptor_dirty_form(monkeypatch):
    """Uncommitted changes → -dirty marker (load-bearing)."""
    def fake_git(*args):
        if args[0] == "rev-parse":
            return "abc1234\n"
        if args[0] == "status":
            return " M tools/foo.py\n"   # dirty
        return None
    monkeypatch.setattr(ap, "_git", fake_git)
    assert ap._git_descriptor() == "abc1234-dirty"


def test_git_descriptor_nogit_form(monkeypatch):
    """No git / not a repo → nogit, never raises."""
    monkeypatch.setattr(ap, "_git", lambda *a: None)
    assert ap._git_descriptor() == "nogit"
    assert ap.analysis_provenance() == f"v{ap.ANALYSIS_VERSION} (nogit)"


def test_git_descriptor_status_unavailable_is_not_clean(monkeypatch):
    """SHA available but status unreadable → honest -dirty, never a false
    clean claim."""
    def fake_git(*args):
        if args[0] == "rev-parse":
            return "abc1234\n"
        if args[0] == "status":
            return None   # couldn't read status
        return None
    monkeypatch.setattr(ap, "_git", fake_git)
    assert ap._git_descriptor() == "abc1234-dirty"


def test_provenance_line_contains_tool_and_version():
    line = ap.provenance_line("mytool.py")
    assert line.startswith("mytool.py · analysis v")
    assert f"v{ap.ANALYSIS_VERSION} (" in line


def test_stamp_runs_without_error(tmp_path):
    fig, axis = plt.subplots()
    axis.plot([0, 1], [0, 1])
    ap.stamp(fig, "tool.py")
    out = tmp_path / "stamped.png"
    fig.savefig(out)
    plt.close(fig)
    assert out.exists() and out.stat().st_size > 0
    # The footer text exists among the figure's text artists.
    texts = [t.get_text() for t in fig.texts]
    assert any("analysis v" in t for t in texts)


def test_skip_comment_lines_drops_hash():
    src = ["# capture_tool=ticc_capture.py capture_version=1\n",
           "ts_iso,channel,ref_sec,ref_ps,recv_mono\n",
           "2026-06-20T00:00:00.0Z,chA,1,0,0.0\n"]
    kept = list(ap.skip_comment_lines(iter(src)))
    assert kept[0].startswith("ts_iso")
    assert len(kept) == 2
