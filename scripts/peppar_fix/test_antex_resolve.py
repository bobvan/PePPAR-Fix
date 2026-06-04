"""Unit tests for antex_resolve.resolve_pcv_defaults().

These cover the PCV default-on behavior introduced 2026-06-04 to close
I-121024-main: the entire lab fleet had been running with PCV silently
off because the launch scripts didn't pass --antex-path /
--receiver-antenna.  The resolver auto-discovers both from
timelab/antennas.json[arp_label] and a small ANTEX search path.
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from unittest import mock

_SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from antex import canonicalize_antex_type                  # noqa: E402
from peppar_fix import antex_resolve                       # noqa: E402
from peppar_fix.antex_resolve import (                     # noqa: E402
    DISABLED_BY_FLAG,
    DISABLED_NO_ANTENNAS_JSON,
    DISABLED_NO_ANTENNA_FIELD,
    DISABLED_NO_ARP,
    DISABLED_NO_ATX_FILE,
    DISABLED_PARSE_FAILED,
    ENABLED_FROM_CLI,
    ENABLED_FROM_DEFAULTS,
    resolve_pcv_defaults,
)


class CanonicalizeTest(unittest.TestCase):
    """canonicalize_antex_type() — sole caller in production is the
    resolver, but the helper is general."""

    def test_compact_form(self):
        # SFESPK6618H (11) + 5 spaces + NONE (4) = 20 chars.
        self.assertEqual(
            canonicalize_antex_type("SFESPK6618H NONE"),
            "SFESPK6618H     NONE",
        )

    def test_already_canonical(self):
        # 20-char canonical form returns itself.
        canonical = "SFESPK6618H     NONE"
        self.assertEqual(canonicalize_antex_type(canonical), canonical)

    def test_already_canonical_with_trailing_whitespace(self):
        self.assertEqual(
            canonicalize_antex_type("SFESPK6618H     NONE   "),
            "SFESPK6618H     NONE",
        )

    def test_trm_example_from_help_text(self):
        # The engine's help text uses TRM57971.00 NONE as an example.
        self.assertEqual(
            canonicalize_antex_type("TRM57971.00 NONE"),
            "TRM57971.00     NONE",
        )

    def test_rejects_free_text_with_parentheses(self):
        # choke1's antennas.json field: free-text description, not an
        # ANTEX TYPE.  Resolver must fall back to requiring an explicit
        # --receiver-antenna for this antenna.
        free_text = ("CHOKE1 (un-calibrated 3D stepped choke ring) "
                     "— OPUS used SFESPK6618H NONE antex")
        self.assertIsNone(canonicalize_antex_type(free_text))

    def test_rejects_three_tokens(self):
        self.assertIsNone(canonicalize_antex_type("TRM 57971 NONE"))

    def test_rejects_em_dash(self):
        self.assertIsNone(canonicalize_antex_type("SFESPK6618H — NONE"))

    def test_rejects_empty(self):
        self.assertIsNone(canonicalize_antex_type(""))
        self.assertIsNone(canonicalize_antex_type("   "))
        self.assertIsNone(canonicalize_antex_type(None))

    def test_rejects_punctuation(self):
        self.assertIsNone(canonicalize_antex_type("TRM/57971 NONE"))
        self.assertIsNone(canonicalize_antex_type("TRM,57971 NONE"))


def _write_antennas_json(dir_path: str, arp_label: str,
                         antenna_field: object = "SFESPK6618H NONE") -> str:
    """Write a minimal timelab/antennas.json into dir_path/timelab/.

    Including ecef_m + sigma_m so load_arp_from_antennas would also
    accept it (we test only the antenna-field path here, but a
    realistic fixture helps when tests get extended).  Returns the
    absolute path written.
    """
    timelab = os.path.join(dir_path, "timelab")
    os.makedirs(timelab, exist_ok=True)
    entry = {
        "ecef_m": {"x": 0.0, "y": 0.0, "z": 0.0},
        "sigma_m": 0.01,
    }
    if antenna_field is not None:
        entry["antenna"] = antenna_field
    path = os.path.join(timelab, "antennas.json")
    with open(path, "w") as f:
        json.dump({arp_label: entry}, f)
    return path


def _touch(path: str) -> None:
    """Create an empty file at path (and any needed parent dirs)."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "wb").close()


class ResolvePCVDefaultsTest(unittest.TestCase):

    def setUp(self):
        # Each test gets a tmpdir with its own antennas.json + support/.
        self.tmp = tempfile.mkdtemp()
        # Patch the position_state.find_antennas_json so it only sees
        # files we create — the real implementation walks several
        # absolute paths that we must not depend on in unit tests.
        self._find_patcher = mock.patch(
            "peppar_fix.position_state.find_antennas_json",
            side_effect=self._fake_find,
        )
        self._find_patcher.start()
        self._antennas_json_path: str | None = None

    def tearDown(self):
        self._find_patcher.stop()
        # Best-effort cleanup; tmpdir lives in /tmp.
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _fake_find(self, explicit_path=None):
        if explicit_path:
            return explicit_path if os.path.isfile(explicit_path) else None
        return self._antennas_json_path

    def _search_roots(self) -> list[str]:
        """A search root list rooted inside self.tmp so the tests don't
        pick up ./support/antex from the repo."""
        return [
            os.path.join(self.tmp, "support", "antex"),
            os.path.join(self.tmp, "elsewhere"),
        ]

    # --- success paths ------------------------------------------------

    def test_resolve_both_from_antennas_and_search_path(self):
        # Realistic happy path: antennas.json has the antenna field;
        # ngs20.atx lives under support/antex/ in the tmp dir.
        self._antennas_json_path = _write_antennas_json(self.tmp, "ufo1")
        _touch(os.path.join(self.tmp, "support", "antex", "ngs20.atx"))
        pcv = resolve_pcv_defaults(
            arp_label="ufo1",
            antex_path_cli=None,
            receiver_antenna_cli=None,
            search_roots=self._search_roots(),
        )
        self.assertEqual(pcv.status, ENABLED_FROM_DEFAULTS)
        self.assertEqual(pcv.receiver_antenna, "SFESPK6618H     NONE")
        self.assertTrue(pcv.antex_path.endswith("ngs20.atx"))
        self.assertEqual(pcv.antenna_source, "antennas.json[ufo1]")
        self.assertEqual(pcv.antex_source, "auto-discovered")

    def test_resolve_prefers_combined_catalog_over_per_antenna(self):
        # When both ngs20.atx and SFESPK6618H_NONE.atx exist, the
        # combined catalog is preferred — per-antenna extracts contain
        # only the receiver block, so the engine's satellite-side
        # lookups would miss and pcv=0/N at runtime.  (Lab-confirmed
        # 2026-06-04 in the PR #135 validation pass.)
        self._antennas_json_path = _write_antennas_json(self.tmp, "ufo1")
        atx_dir = os.path.join(self.tmp, "support", "antex")
        _touch(os.path.join(atx_dir, "ngs20.atx"))
        _touch(os.path.join(atx_dir, "SFESPK6618H_NONE.atx"))
        pcv = resolve_pcv_defaults(
            arp_label="ufo1",
            antex_path_cli=None,
            receiver_antenna_cli=None,
            search_roots=self._search_roots(),
        )
        self.assertEqual(pcv.status, ENABLED_FROM_DEFAULTS)
        self.assertTrue(pcv.antex_path.endswith("ngs20.atx"),
                        f"expected combined catalog preferred, got {pcv.antex_path}")

    def test_resolve_falls_back_to_per_antenna_when_no_combined(self):
        # Last-resort fallback: per-antenna file is better than nothing
        # (still enables receiver-side PCV awareness).
        self._antennas_json_path = _write_antennas_json(self.tmp, "ufo1")
        atx_dir = os.path.join(self.tmp, "support", "antex")
        _touch(os.path.join(atx_dir, "SFESPK6618H_NONE.atx"))
        pcv = resolve_pcv_defaults(
            arp_label="ufo1",
            antex_path_cli=None,
            receiver_antenna_cli=None,
            search_roots=self._search_roots(),
        )
        self.assertEqual(pcv.status, ENABLED_FROM_DEFAULTS)
        self.assertTrue(pcv.antex_path.endswith("SFESPK6618H_NONE.atx"))

    def test_resolve_cli_override(self):
        # Both CLI args present: status is ENABLED_FROM_CLI regardless of
        # whether antennas.json exists.
        explicit_atx = os.path.join(self.tmp, "explicit.atx")
        _touch(explicit_atx)
        pcv = resolve_pcv_defaults(
            arp_label="ufo1",
            antex_path_cli=explicit_atx,
            receiver_antenna_cli="TRM57971.00 NONE",
            search_roots=self._search_roots(),
        )
        self.assertEqual(pcv.status, ENABLED_FROM_CLI)
        self.assertEqual(pcv.receiver_antenna, "TRM57971.00     NONE")
        self.assertEqual(pcv.antex_path, explicit_atx)

    def test_resolve_cli_partial_uses_antennas_json_for_other(self):
        # --antex-path only; receiver antenna still pulled from
        # antennas.json.  Status is FROM_CLI because at least one
        # explicit override is in play.
        self._antennas_json_path = _write_antennas_json(self.tmp, "ufo1")
        explicit_atx = os.path.join(self.tmp, "explicit.atx")
        _touch(explicit_atx)
        pcv = resolve_pcv_defaults(
            arp_label="ufo1",
            antex_path_cli=explicit_atx,
            receiver_antenna_cli=None,
            search_roots=self._search_roots(),
        )
        self.assertEqual(pcv.status, ENABLED_FROM_CLI)
        self.assertEqual(pcv.receiver_antenna, "SFESPK6618H     NONE")
        self.assertEqual(pcv.antex_path, explicit_atx)

    # --- failure paths ------------------------------------------------

    def test_disabled_by_flag(self):
        # --no-pcv: short-circuit, never look at antennas.json.
        pcv = resolve_pcv_defaults(
            arp_label="ufo1",
            antex_path_cli=None,
            receiver_antenna_cli=None,
            pcv_flag=False,
        )
        self.assertEqual(pcv.status, DISABLED_BY_FLAG)
        self.assertIsNone(pcv.antex_path)
        self.assertIsNone(pcv.receiver_antenna)

    def test_disabled_no_arp(self):
        # No arp_label, no --receiver-antenna: can't even start.
        pcv = resolve_pcv_defaults(
            arp_label=None,
            antex_path_cli=None,
            receiver_antenna_cli=None,
        )
        self.assertEqual(pcv.status, DISABLED_NO_ARP)
        self.assertIn("arp_label", pcv.detail)

    def test_disabled_no_antennas_json(self):
        # arp_label set but no antennas.json found anywhere.
        pcv = resolve_pcv_defaults(
            arp_label="ufo1",
            antex_path_cli=None,
            receiver_antenna_cli=None,
        )
        self.assertEqual(pcv.status, DISABLED_NO_ANTENNAS_JSON)

    def test_disabled_no_antenna_field(self):
        # antennas.json found but the entry lacks an "antenna" field.
        self._antennas_json_path = _write_antennas_json(
            self.tmp, "ufo1", antenna_field=None)
        pcv = resolve_pcv_defaults(
            arp_label="ufo1",
            antex_path_cli=None,
            receiver_antenna_cli=None,
            search_roots=self._search_roots(),
        )
        self.assertEqual(pcv.status, DISABLED_NO_ANTENNA_FIELD)
        self.assertIn("ufo1", pcv.detail)

    def test_disabled_parse_failed_free_text(self):
        # antennas.json has the field but it's free text (choke1 case).
        free_text = ("CHOKE1 (un-calibrated 3D stepped choke ring) "
                     "— OPUS used SFESPK6618H NONE antex")
        self._antennas_json_path = _write_antennas_json(
            self.tmp, "choke1", antenna_field=free_text)
        pcv = resolve_pcv_defaults(
            arp_label="choke1",
            antex_path_cli=None,
            receiver_antenna_cli=None,
            search_roots=self._search_roots(),
        )
        self.assertEqual(pcv.status, DISABLED_PARSE_FAILED)
        self.assertIn("free-text", pcv.detail)

    def test_disabled_parse_failed_cli(self):
        # --receiver-antenna passed but not parseable.
        pcv = resolve_pcv_defaults(
            arp_label="ufo1",
            antex_path_cli=None,
            receiver_antenna_cli="garbage (not antex)",
        )
        self.assertEqual(pcv.status, DISABLED_PARSE_FAILED)

    def test_disabled_no_atx_file(self):
        # antennas.json + antenna field parse fine, but no .atx anywhere.
        self._antennas_json_path = _write_antennas_json(self.tmp, "ufo1")
        pcv = resolve_pcv_defaults(
            arp_label="ufo1",
            antex_path_cli=None,
            receiver_antenna_cli=None,
            search_roots=self._search_roots(),
        )
        self.assertEqual(pcv.status, DISABLED_NO_ATX_FILE)
        # detail should list the canonical antenna so the operator
        # knows what file to drop in.
        self.assertIn("ANTEX", pcv.detail)
        # We still surface the receiver antenna so the operator can
        # see *what* we'd have looked up.
        self.assertEqual(pcv.receiver_antenna, "SFESPK6618H     NONE")

    def test_disabled_cli_atx_missing(self):
        # --antex-path points to a nonexistent file: fail loud, don't
        # silently fall back to auto-discovery.
        pcv = resolve_pcv_defaults(
            arp_label="ufo1",
            antex_path_cli="/no/such/file.atx",
            receiver_antenna_cli="TRM57971.00 NONE",
        )
        self.assertEqual(pcv.status, DISABLED_NO_ATX_FILE)


class LogPCVStatusTest(unittest.TestCase):
    """log_pcv_status() smoke test — INFO when enabled, WARN otherwise."""

    def test_enabled_emits_info(self):
        from peppar_fix.antex_resolve import (
            ResolvedPCV, log_pcv_status, ENABLED_FROM_DEFAULTS,
        )
        import logging
        pcv = ResolvedPCV(
            antex_path="/x.atx", receiver_antenna="TRM57971.00     NONE",
            status=ENABLED_FROM_DEFAULTS, detail="",
            antenna_source="antennas.json[ufo1]", antex_source="auto-discovered",
        )
        logger = logging.getLogger("test_pcv_info")
        with self.assertLogs(logger, level="INFO") as cm:
            log_pcv_status(pcv, logger=logger)
        self.assertTrue(any("PCV: enabled" in m for m in cm.output))

    def test_disabled_emits_warning(self):
        from peppar_fix.antex_resolve import (
            ResolvedPCV, log_pcv_status, DISABLED_NO_ATX_FILE,
        )
        import logging
        pcv = ResolvedPCV(
            antex_path=None, receiver_antenna=None,
            status=DISABLED_NO_ATX_FILE, detail="none found",
        )
        logger = logging.getLogger("test_pcv_warn")
        with self.assertLogs(logger, level="WARNING") as cm:
            log_pcv_status(pcv, logger=logger)
        self.assertTrue(any("PCV: DISABLED" in m for m in cm.output))


if __name__ == "__main__":
    unittest.main()
