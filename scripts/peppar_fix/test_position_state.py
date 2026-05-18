"""Unit tests for position_state.py."""

import os
import tempfile
import unittest

from peppar_fix.position_state import (
    PositionState,
    SURVEY_TIE_BREAK_RATIO,
    _format_toml,
    filter_current_mount,
    load_ppp_state,
    load_survey_state,
    pick_most_confident,
    save_ppp_state,
    utc_now_iso,
)


def _make_state(kind="ppp", mount_id=0, sigma_m=0.05, source="test"):
    return PositionState(
        mount_id=mount_id,
        ecef_m=(157469.3814, -4756189.0729, 4232768.5274),
        sigma_m=sigma_m,
        updated=utc_now_iso(),
        source=source,
        kind=kind,
    )


class TestSaveLoadRoundTrip(unittest.TestCase):

    def test_ppp_round_trip(self):
        with tempfile.TemporaryDirectory() as d:
            s = _make_state(kind="ppp", mount_id=3, sigma_m=0.087,
                            source="peppar_fix_engine PPP-AR")
            save_ppp_state(s, "12345", positions_dir=d)
            loaded = load_ppp_state("12345", positions_dir=d)
            self.assertIsNotNone(loaded)
            self.assertEqual(loaded.mount_id, 3)
            self.assertEqual(loaded.kind, "ppp")
            self.assertAlmostEqual(loaded.sigma_m, 0.087, places=4)
            self.assertAlmostEqual(loaded.ecef_m[0], 157469.3814, places=3)
            self.assertEqual(loaded.source, "peppar_fix_engine PPP-AR")

    def test_extra_round_trip(self):
        with tempfile.TemporaryDirectory() as d:
            s = PositionState(
                mount_id=1,
                ecef_m=(1.0, 2.0, 3.0),
                sigma_m=0.01,
                updated=utc_now_iso(),
                source="test",
                kind="ppp",
                extra={"n_epochs": 14437, "filter": "PPP-AR"},
            )
            save_ppp_state(s, "1", positions_dir=d)
            loaded = load_ppp_state("1", positions_dir=d)
            self.assertEqual(loaded.extra.get("n_epochs"), 14437)
            self.assertEqual(loaded.extra.get("filter"), "PPP-AR")

    def test_atomic_write(self):
        """save_ppp_state must use temp+rename — no .tmp left after success."""
        with tempfile.TemporaryDirectory() as d:
            save_ppp_state(_make_state(), "9", positions_dir=d)
            entries = os.listdir(d)
            self.assertIn("9.ppp.toml", entries)
            self.assertNotIn("9.ppp.toml.tmp", entries)

    def test_save_survey_kind_rejected(self):
        """save_ppp_state refuses kind='survey' to prevent mis-tagging."""
        with tempfile.TemporaryDirectory() as d:
            s = _make_state(kind="survey")
            with self.assertRaises(ValueError):
                save_ppp_state(s, "1", positions_dir=d)


class TestLoadMissingOrMalformed(unittest.TestCase):

    def test_missing_returns_none(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(load_ppp_state("nope", positions_dir=d))
            self.assertIsNone(load_survey_state("nope", positions_dir=d))

    def test_malformed_returns_none(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "1.ppp.toml")
            with open(p, "w") as f:
                f.write("not valid = toml = at all = [")
            self.assertIsNone(load_ppp_state("1", positions_dir=d))

    def test_missing_required_field_returns_none(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "1.ppp.toml")
            with open(p, "w") as f:
                # Missing sigma_m
                f.write('mount_id = 1\necef_m = [1.0, 2.0, 3.0]\n'
                        'updated = "2026-05-18T00:00:00Z"\n'
                        'source = "x"\n')
            self.assertIsNone(load_ppp_state("1", positions_dir=d))

    def test_wrong_ecef_length_returns_none(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "1.ppp.toml")
            with open(p, "w") as f:
                f.write('mount_id = 1\necef_m = [1.0, 2.0]\n'
                        'sigma_m = 0.05\nupdated = "x"\nsource = "x"\n')
            self.assertIsNone(load_ppp_state("1", positions_dir=d))

    def test_survey_load_distinguished_by_filename(self):
        """A .survey.toml file is not picked up by load_ppp_state and
        vice versa — the loaders are filename-discriminated."""
        with tempfile.TemporaryDirectory() as d:
            survey = _make_state(kind="ppp", source="ppp-snap")  # kind set on save
            save_ppp_state(survey, "1", positions_dir=d)
            # Re-shape the file as a survey one and verify load_survey
            # picks it up but load_ppp doesn't pick up a survey file.
            os.rename(os.path.join(d, "1.ppp.toml"),
                      os.path.join(d, "1.survey.toml"))
            self.assertIsNone(load_ppp_state("1", positions_dir=d))
            sv = load_survey_state("1", positions_dir=d)
            self.assertIsNotNone(sv)
            self.assertEqual(sv.kind, "survey")


class TestFilterCurrentMount(unittest.TestCase):

    def test_drops_none(self):
        out = filter_current_mount([None, None], current_mount_id=0)
        self.assertEqual(out, [])

    def test_keeps_matching_mount(self):
        s1 = _make_state(mount_id=3)
        s2 = _make_state(mount_id=3, kind="survey")
        out = filter_current_mount([s1, s2], current_mount_id=3)
        self.assertEqual(len(out), 2)

    def test_drops_stale_mount(self):
        s_stale = _make_state(mount_id=2)
        s_curr = _make_state(mount_id=3, kind="survey")
        out = filter_current_mount([s_stale, s_curr, None],
                                   current_mount_id=3)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].kind, "survey")


class TestPickMostConfident(unittest.TestCase):

    def test_empty_returns_none(self):
        self.assertIsNone(pick_most_confident([]))

    def test_single_state_returned(self):
        s = _make_state(sigma_m=0.05)
        self.assertEqual(pick_most_confident([s]), s)

    def test_smaller_sigma_wins(self):
        ppp_loose = _make_state(kind="ppp", sigma_m=0.10)
        ppp_tight = _make_state(kind="ppp", sigma_m=0.02)
        out = pick_most_confident([ppp_loose, ppp_tight])
        self.assertEqual(out.sigma_m, 0.02)

    def test_survey_wins_when_comparable(self):
        """Survey ties with PPP when its sigma is within 2x — survey wins."""
        ppp = _make_state(kind="ppp", sigma_m=0.020)
        survey = _make_state(kind="survey", sigma_m=0.025)  # 1.25x
        out = pick_most_confident([ppp, survey])
        self.assertEqual(out.kind, "survey")

    def test_survey_loses_when_much_looser(self):
        """Survey > 2x looser than PPP: PPP wins."""
        ppp = _make_state(kind="ppp", sigma_m=0.020)
        survey = _make_state(kind="survey", sigma_m=0.10)  # 5x
        out = pick_most_confident([ppp, survey])
        self.assertEqual(out.kind, "ppp")

    def test_survey_at_exactly_2x_still_wins(self):
        """Boundary: sigma_survey == 2 * sigma_ppp — survey wins."""
        ppp = _make_state(kind="ppp", sigma_m=0.020)
        survey = _make_state(kind="survey",
                             sigma_m=ppp.sigma_m * SURVEY_TIE_BREAK_RATIO)
        out = pick_most_confident([ppp, survey])
        self.assertEqual(out.kind, "survey")

    def test_only_survey_present(self):
        survey = _make_state(kind="survey", sigma_m=0.05)
        out = pick_most_confident([survey])
        self.assertEqual(out, survey)

    def test_only_ppp_present(self):
        ppp = _make_state(kind="ppp", sigma_m=0.05)
        out = pick_most_confident([ppp])
        self.assertEqual(out, ppp)

    def test_two_surveys_smaller_wins(self):
        """If somehow two survey states present, smallest wins (not common)."""
        a = _make_state(kind="survey", sigma_m=0.08)
        b = _make_state(kind="survey", sigma_m=0.04)
        out = pick_most_confident([a, b])
        self.assertEqual(out.sigma_m, 0.04)


class TestFormatToml(unittest.TestCase):

    def test_format_minimal(self):
        s = PositionState(
            mount_id=3,
            ecef_m=(1.0, 2.0, 3.0),
            sigma_m=0.05,
            updated="2026-05-18T00:00:00Z",
            source="test",
        )
        out = _format_toml(s)
        self.assertIn("mount_id = 3", out)
        self.assertIn("ecef_m = [1.0000, 2.0000, 3.0000]", out)
        self.assertIn("sigma_m = 0.050000", out)
        self.assertIn('updated = "2026-05-18T00:00:00Z"', out)
        self.assertIn('source = "test"', out)
        self.assertTrue(out.endswith("\n"))

    def test_format_with_extras(self):
        s = PositionState(
            mount_id=0,
            ecef_m=(1.0, 2.0, 3.0),
            sigma_m=0.01,
            updated="x",
            source="y",
            extra={"n_epochs": 12, "active": True,
                   "method": "PPP-AR", "ratio": 1.7},
        )
        out = _format_toml(s)
        self.assertIn("n_epochs = 12", out)
        self.assertIn("active = true", out)
        self.assertIn('method = "PPP-AR"', out)
        self.assertIn("ratio = 1.7", out)


class TestUtcNowIso(unittest.TestCase):

    def test_format(self):
        s = utc_now_iso()
        # Must be 20 chars, end in 'Z', and parseable per ISO 8601 shape
        self.assertEqual(len(s), 20)
        self.assertTrue(s.endswith("Z"))
        # YYYY-MM-DDTHH:MM:SSZ
        self.assertEqual(s[4], "-")
        self.assertEqual(s[7], "-")
        self.assertEqual(s[10], "T")
        self.assertEqual(s[13], ":")
        self.assertEqual(s[16], ":")


if __name__ == "__main__":
    unittest.main()
