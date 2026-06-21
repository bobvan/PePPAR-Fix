"""Unit tests for tools/analysis_provenance.py (PPS-comparison provenance)."""
import re
import subprocess
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


# --------------------------------------------------------------------------
# Deterministic tracked-vs-untracked tests against a real temporary git repo.
#
# These drive _git_descriptor() through the REAL git invocation (no _git
# monkeypatch) by repointing _REPO_ROOT at a freshly-initialised temp repo.
# This genuinely exercises the `git status --untracked-files=no` logic and
# the tracked-vs-untracked distinction that is the whole point of the fix.
# --------------------------------------------------------------------------

def _init_temp_repo(path: Path) -> None:
    """git init a repo at *path* with a local identity and one commit."""
    def g(*args):
        subprocess.run(["git", "-C", str(path), *args],
                       check=True, capture_output=True, text=True)
    g("init")
    g("config", "user.email", "test@example.com")
    g("config", "user.name", "Provenance Test")
    g("config", "commit.gpgsign", "false")
    (path / "tracked.py").write_text("x = 1\n")
    g("add", "tracked.py")
    g("commit", "-m", "initial")


def test_temp_repo_clean_has_no_dirty(tmp_path, monkeypatch):
    """A clean committed repo → bare sha, no -dirty."""
    _init_temp_repo(tmp_path)
    monkeypatch.setattr(ap, "_REPO_ROOT", tmp_path)
    desc = ap._git_descriptor()
    assert desc != "nogit"
    assert not desc.endswith("-dirty"), desc
    assert re.fullmatch(r"[0-9a-f]+", desc), desc


def test_temp_repo_untracked_file_is_not_dirty(tmp_path, monkeypatch):
    """Adding an UNTRACKED file must NOT mark the tree dirty.

    This is the bug being fixed: lab hosts always carry untracked files
    (venv/, *.egg-info/, state/, timelab/antennas.json), and the old
    `git status --porcelain` counted them, stamping every figure -dirty
    even with pristine tracked analysis code."""
    _init_temp_repo(tmp_path)
    (tmp_path / "venv").mkdir()
    (tmp_path / "venv" / "junk").write_text("ignore me\n")
    (tmp_path / "antennas.json").write_text("{}\n")
    monkeypatch.setattr(ap, "_REPO_ROOT", tmp_path)
    desc = ap._git_descriptor()
    assert not desc.endswith("-dirty"), (
        f"untracked files falsely marked tree dirty: {desc!r}")


def test_temp_repo_modified_tracked_file_is_dirty(tmp_path, monkeypatch):
    """Modifying a TRACKED file → -dirty (the marker still fires for real
    uncommitted analysis-code edits)."""
    _init_temp_repo(tmp_path)
    (tmp_path / "tracked.py").write_text("x = 2  # edited\n")
    monkeypatch.setattr(ap, "_REPO_ROOT", tmp_path)
    desc = ap._git_descriptor()
    assert desc.endswith("-dirty"), (
        f"modified tracked file should mark tree dirty: {desc!r}")


def test_temp_repo_staged_tracked_change_is_dirty(tmp_path, monkeypatch):
    """A STAGED tracked change is also dirty (staged + unstaged both count)."""
    _init_temp_repo(tmp_path)
    (tmp_path / "tracked.py").write_text("x = 3  # staged\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "tracked.py"],
                   check=True, capture_output=True, text=True)
    monkeypatch.setattr(ap, "_REPO_ROOT", tmp_path)
    desc = ap._git_descriptor()
    assert desc.endswith("-dirty"), desc


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
