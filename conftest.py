"""Suite-wide test hygiene guards.

Lives at the repo ROOT so it covers both testpaths — ``scripts/peppar_fix/``
and ``tests/`` (see ``[tool.pytest.ini_options]`` in ``pyproject.toml``).

What it guards: ``logging.disable()``.  That call writes
``logging.Logger.manager.disable``, a single process-wide global with no
owner and no scope.  Any module that calls it and does not restore it
suppresses ``self.assertLogs(...)`` and ``caplog`` for **every** test in
the session — and because pytest imports all test modules during
*collection*, a module-scope call takes effect before the first test runs,
so no amount of reordering avoids it.

That is exactly what fullSuiteTestPollution (I-153714) was: three sim test
modules disabled logging at import scope, producing 21 failures spread
across 7 unrelated files, each of which passed when run alone.

This guard turns that silent, spooky-action-at-a-distance failure mode into
a loud one that names the culprit.  It does not forbid ``logging.disable()``
— it requires that whoever calls it puts it back.
"""
import logging
import traceback

import pytest

# Record where any global disable came from, so the report can name the
# culprit instead of just its victims.  Wrapping (not forbidding) keeps
# legitimate save/restore uses working.
_ORIG_DISABLE = logging.disable
_DISABLE_CALLS: list[str] = []


def _recording_disable(level=logging.CRITICAL):
    if level:
        # limit=2 -> [caller_of_disable, _recording_disable]; [0] is the
        # call site we want to name, not our own wrapper.
        frame = traceback.format_stack(limit=2)[0].strip().replace("\n", " | ")
        _DISABLE_CALLS.append(f"logging.disable({level}) at {frame}")
    return _ORIG_DISABLE(level)


logging.disable = _recording_disable


def _poisoned() -> str | None:
    """Return a diagnostic if the global log-disable is set, else None."""
    level = logging.Logger.manager.disable
    if not level:
        return None
    callers = "\n  ".join(_DISABLE_CALLS[-5:]) or "(no call recorded)"
    return (
        f"logging.Logger.manager.disable == {level} — logging is globally "
        f"suppressed, so assertLogs()/caplog cannot see any record at or "
        f"below that level.\n"
        f"Recorded logging.disable() calls:\n  {callers}\n"
        f"Fix the caller: silence the specific logger instead, and restore "
        f"it (setUpModule/tearDownModule or a fixture).  See the "
        f"fullSuiteTestPollution note in this file's docstring."
    )


def pytest_collection_finish(session):
    """Catch import-scope polluters — they fire before any test runs."""
    diag = _poisoned()
    if diag:
        _ORIG_DISABLE(logging.NOTSET)  # unpoison so the run stays meaningful
        raise pytest.UsageError(
            "test collection left logging globally disabled.\n" + diag
        )


@pytest.fixture(autouse=True)
def _no_global_log_disable():
    """Catch run-time polluters, and pin the failure on the test that did it."""
    yield
    diag = _poisoned()
    if diag:
        _ORIG_DISABLE(logging.NOTSET)  # unpoison so one failure stays one
        pytest.fail("this test left logging globally disabled.\n" + diag)
