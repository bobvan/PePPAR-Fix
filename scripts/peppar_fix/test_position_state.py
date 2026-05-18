"""Unit tests for position_state.py."""

import os
import tempfile
import unittest

from peppar_fix.position_state import (
    PositionState,
    PppStateWriter,
    SURVEY_TIE_BREAK_RATIO,
    _format_toml,
    filter_current_mount,
    load_current_mount_sn,
    load_ppp_state,
    load_survey_state,
    pick_most_confident,
    save_ppp_state,
    seed_from_state_files,
    utc_now_iso,
)


def _make_state(kind="ppp", mount_sn=0, sigma_m=0.05, source="test"):
    return PositionState(
        mount_sn=mount_sn,
        ecef_m=(157469.3814, -4756189.0729, 4232768.5274),
        sigma_m=sigma_m,
        updated=utc_now_iso(),
        source=source,
        kind=kind,
    )


class TestSaveLoadRoundTrip(unittest.TestCase):

    def test_ppp_round_trip(self):
        with tempfile.TemporaryDirectory() as d:
            s = _make_state(kind="ppp", mount_sn=3, sigma_m=0.087,
                            source="peppar_fix_engine PPP-AR")
            save_ppp_state(s, "12345", positions_dir=d)
            loaded = load_ppp_state("12345", positions_dir=d)
            self.assertIsNotNone(loaded)
            self.assertEqual(loaded.mount_sn, 3)
            self.assertEqual(loaded.kind, "ppp")
            self.assertAlmostEqual(loaded.sigma_m, 0.087, places=4)
            self.assertAlmostEqual(loaded.ecef_m[0], 157469.3814, places=3)
            self.assertEqual(loaded.source, "peppar_fix_engine PPP-AR")

    def test_extra_round_trip(self):
        with tempfile.TemporaryDirectory() as d:
            s = PositionState(
                mount_sn=1,
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
                f.write('mount_sn = 1\necef_m = [1.0, 2.0, 3.0]\n'
                        'updated = "2026-05-18T00:00:00Z"\n'
                        'source = "x"\n')
            self.assertIsNone(load_ppp_state("1", positions_dir=d))

    def test_wrong_ecef_length_returns_none(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "1.ppp.toml")
            with open(p, "w") as f:
                f.write('mount_sn = 1\necef_m = [1.0, 2.0]\n'
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
        out = filter_current_mount([None, None], current_mount_sn=0)
        self.assertEqual(out, [])

    def test_keeps_matching_mount(self):
        s1 = _make_state(mount_sn=3)
        s2 = _make_state(mount_sn=3, kind="survey")
        out = filter_current_mount([s1, s2], current_mount_sn=3)
        self.assertEqual(len(out), 2)

    def test_drops_stale_mount(self):
        s_stale = _make_state(mount_sn=2)
        s_curr = _make_state(mount_sn=3, kind="survey")
        out = filter_current_mount([s_stale, s_curr, None],
                                   current_mount_sn=3)
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
            mount_sn=3,
            ecef_m=(1.0, 2.0, 3.0),
            sigma_m=0.05,
            updated="2026-05-18T00:00:00Z",
            source="test",
        )
        out = _format_toml(s)
        self.assertIn("mount_sn = 3", out)
        self.assertIn("ecef_m = [1.0000, 2.0000, 3.0000]", out)
        self.assertIn("sigma_m = 0.050000", out)
        self.assertIn('updated = "2026-05-18T00:00:00Z"', out)
        self.assertIn('source = "test"', out)
        self.assertTrue(out.endswith("\n"))

    def test_format_with_extras(self):
        s = PositionState(
            mount_sn=0,
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


class TestLoadCurrentMountId(unittest.TestCase):

    def test_uid_none_returns_zero(self):
        self.assertEqual(load_current_mount_sn(None), 0)

    def test_no_receiver_file_returns_zero(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(
                load_current_mount_sn("nope", receivers_dir=d), 0)

    def test_reads_mount_sn_from_receiver_state(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "12345.json"), "w") as f:
                json.dump({"unique_id": 12345, "mount_sn": 7}, f)
            self.assertEqual(
                load_current_mount_sn("12345", receivers_dir=d), 7)

    def test_missing_mount_sn_field_returns_zero(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "12345.json"), "w") as f:
                json.dump({"unique_id": 12345}, f)
            self.assertEqual(
                load_current_mount_sn("12345", receivers_dir=d), 0)

    def test_non_int_mount_sn_returns_zero(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "12345.json"), "w") as f:
                json.dump({"unique_id": 12345, "mount_sn": "garbage"}, f)
            self.assertEqual(
                load_current_mount_sn("12345", receivers_dir=d), 0)


class TestSeedFromStateFiles(unittest.TestCase):

    def setUp(self):
        import json
        self.recv_dir = tempfile.mkdtemp()
        self.pos_dir = tempfile.mkdtemp()
        self.uid = "55555"
        # Default: mount_sn=3
        with open(os.path.join(self.recv_dir, f"{self.uid}.json"), "w") as f:
            json.dump({"unique_id": int(self.uid), "mount_sn": 3}, f)

    def tearDown(self):
        import shutil
        shutil.rmtree(self.recv_dir)
        shutil.rmtree(self.pos_dir)

    def _write_state(self, name, mount_sn, sigma_m, kind):
        s = PositionState(
            mount_sn=mount_sn,
            ecef_m=(1.0, 2.0, 3.0),
            sigma_m=sigma_m,
            updated=utc_now_iso(),
            source=f"test-{kind}",
            kind="ppp" if kind == "ppp" else "ppp",  # save_ppp_state requires ppp
        )
        if kind == "ppp":
            save_ppp_state(s, self.uid, positions_dir=self.pos_dir)
        else:
            # Write the survey file directly (peppar-survey is hypothetical)
            survey_path = os.path.join(self.pos_dir,
                                       f"{self.uid}.survey.toml")
            with open(survey_path, "w") as f:
                f.write(
                    f'mount_sn = {mount_sn}\n'
                    f'ecef_m = [1.0, 2.0, 3.0]\n'
                    f'sigma_m = {sigma_m}\n'
                    f'updated = "{s.updated}"\n'
                    f'source = "test-survey"\n'
                )

    def test_no_files_returns_none(self):
        out = seed_from_state_files(self.uid,
                                    positions_dir=self.pos_dir,
                                    receivers_dir=self.recv_dir)
        self.assertIsNone(out)

    def test_picks_ppp_when_only_ppp(self):
        self._write_state("ppp", mount_sn=3, sigma_m=0.05, kind="ppp")
        out = seed_from_state_files(self.uid,
                                    positions_dir=self.pos_dir,
                                    receivers_dir=self.recv_dir)
        self.assertIsNotNone(out)
        self.assertEqual(out.kind, "ppp")

    def test_picks_survey_when_only_survey(self):
        self._write_state("survey", mount_sn=3, sigma_m=0.05, kind="survey")
        out = seed_from_state_files(self.uid,
                                    positions_dir=self.pos_dir,
                                    receivers_dir=self.recv_dir)
        self.assertIsNotNone(out)
        self.assertEqual(out.kind, "survey")

    def test_picks_survey_when_both_comparable(self):
        self._write_state("ppp", mount_sn=3, sigma_m=0.020, kind="ppp")
        self._write_state("survey", mount_sn=3, sigma_m=0.025, kind="survey")
        out = seed_from_state_files(self.uid,
                                    positions_dir=self.pos_dir,
                                    receivers_dir=self.recv_dir)
        self.assertEqual(out.kind, "survey")

    def test_stale_mount_filtered(self):
        # Engine state says mount_sn=3, files tagged with mount_sn=2
        self._write_state("ppp", mount_sn=2, sigma_m=0.01, kind="ppp")
        self._write_state("survey", mount_sn=2, sigma_m=0.005, kind="survey")
        out = seed_from_state_files(self.uid,
                                    positions_dir=self.pos_dir,
                                    receivers_dir=self.recv_dir)
        self.assertIsNone(out)

    def test_ignore_ppp_skips_ppp_file(self):
        self._write_state("ppp", mount_sn=3, sigma_m=0.01, kind="ppp")
        self._write_state("survey", mount_sn=3, sigma_m=0.05, kind="survey")
        out = seed_from_state_files(self.uid,
                                    ignore_ppp=True,
                                    positions_dir=self.pos_dir,
                                    receivers_dir=self.recv_dir)
        self.assertEqual(out.kind, "survey")

    def test_ignore_survey_skips_survey_file(self):
        self._write_state("ppp", mount_sn=3, sigma_m=0.05, kind="ppp")
        self._write_state("survey", mount_sn=3, sigma_m=0.01, kind="survey")
        out = seed_from_state_files(self.uid,
                                    ignore_survey=True,
                                    positions_dir=self.pos_dir,
                                    receivers_dir=self.recv_dir)
        self.assertEqual(out.kind, "ppp")

    def test_ignore_both_returns_none(self):
        self._write_state("ppp", mount_sn=3, sigma_m=0.05, kind="ppp")
        self._write_state("survey", mount_sn=3, sigma_m=0.05, kind="survey")
        out = seed_from_state_files(self.uid,
                                    ignore_ppp=True, ignore_survey=True,
                                    positions_dir=self.pos_dir,
                                    receivers_dir=self.recv_dir)
        self.assertIsNone(out)

    def test_uid_none_returns_none(self):
        # Doesn't even try to read.
        self.assertIsNone(seed_from_state_files(None,
                                                positions_dir=self.pos_dir,
                                                receivers_dir=self.recv_dir))


class TestPppStateWriter(unittest.TestCase):
    """The throttled + sigma-gated periodic writer used inside
    AntPosEstThread.  Tested in isolation via injected time_fn +
    save_fn."""

    def setUp(self):
        # Injectable clock that the test advances manually.
        self.now = 1000.0
        # Captured save_ppp_state calls.
        self.saved = []

    def _time_fn(self):
        return self.now

    def _save_fn(self, state, uid, positions_dir=None):
        self.saved.append((state, uid))

    def _make_writer(self, *, uid="9999", mount_sn=2,
                     period_s=600.0, max_sigma_m=1.0):
        return PppStateWriter(
            receiver_uid=uid,
            mount_sn=mount_sn,
            period_s=period_s,
            max_sigma_m=max_sigma_m,
            time_fn=self._time_fn,
            save_fn=self._save_fn,
        )

    def test_first_write_succeeds(self):
        w = self._make_writer()
        wrote = w.maybe_write((1.0, 2.0, 3.0), sigma_3d=0.05, n_epochs=10)
        self.assertTrue(wrote)
        self.assertEqual(len(self.saved), 1)
        state, uid = self.saved[0]
        self.assertEqual(uid, "9999")
        self.assertEqual(state.mount_sn, 2)
        self.assertEqual(state.ecef_m, (1.0, 2.0, 3.0))
        self.assertAlmostEqual(state.sigma_m, 0.05)
        self.assertEqual(state.kind, "ppp")
        self.assertEqual(state.extra["n_epochs"], 10)
        self.assertEqual(w.n_writes, 1)

    def test_throttle_blocks_second_write_in_window(self):
        w = self._make_writer(period_s=600.0)
        w.maybe_write((1, 2, 3), 0.05, 10)
        self.now += 100  # well under 600s
        wrote = w.maybe_write((1, 2, 3), 0.05, 20)
        self.assertFalse(wrote)
        self.assertEqual(len(self.saved), 1)
        self.assertEqual(w.n_skipped_throttle, 1)

    def test_throttle_allows_write_after_period(self):
        w = self._make_writer(period_s=600.0)
        w.maybe_write((1, 2, 3), 0.05, 10)
        self.now += 601.0
        wrote = w.maybe_write((1, 2, 3), 0.05, 20)
        self.assertTrue(wrote)
        self.assertEqual(len(self.saved), 2)
        self.assertEqual(self.saved[1][0].extra["n_epochs"], 20)

    def test_sigma_gate_blocks_noisy_writes(self):
        w = self._make_writer(max_sigma_m=1.0)
        wrote = w.maybe_write((1, 2, 3), sigma_3d=2.5, n_epochs=10)
        self.assertFalse(wrote)
        self.assertEqual(w.n_skipped_sigma, 1)
        self.assertEqual(len(self.saved), 0)
        # And throttle was NOT armed — the next acceptable σ should
        # fire immediately, not have to wait 600s.
        wrote = w.maybe_write((1, 2, 3), sigma_3d=0.05, n_epochs=11)
        self.assertTrue(wrote)

    def test_sigma_at_boundary_writes(self):
        w = self._make_writer(max_sigma_m=1.0)
        self.assertTrue(
            w.maybe_write((1, 2, 3), sigma_3d=1.0, n_epochs=10))

    def test_uid_none_is_noop(self):
        w = self._make_writer(uid=None)
        wrote = w.maybe_write((1, 2, 3), sigma_3d=0.05, n_epochs=10)
        self.assertFalse(wrote)
        self.assertEqual(len(self.saved), 0)
        # No counters bumped — it's a clean no-op.
        self.assertEqual(w.n_writes, 0)
        self.assertEqual(w.n_skipped_sigma, 0)
        self.assertEqual(w.n_skipped_throttle, 0)

    def test_save_oserror_logs_and_throttles(self):
        """An OSError from save_fn is logged but doesn't crash; the
        throttle still arms so we don't retry every epoch on a
        persistent filesystem failure."""
        def failing_save(state, uid, positions_dir=None):
            raise OSError("disk full")

        w = PppStateWriter(
            receiver_uid="1",
            mount_sn=0,
            period_s=600.0,
            max_sigma_m=1.0,
            time_fn=self._time_fn,
            save_fn=failing_save,
        )
        wrote = w.maybe_write((1, 2, 3), 0.05, 10)
        self.assertFalse(wrote)
        self.assertEqual(w.n_writes, 0)
        # Next attempt in-window should be throttle-blocked.
        self.now += 100
        wrote2 = w.maybe_write((1, 2, 3), 0.05, 11)
        self.assertFalse(wrote2)
        self.assertEqual(w.n_skipped_throttle, 1)

    def test_end_to_end_with_real_save(self):
        """Integration: real save_fn writes a file readable by
        load_ppp_state."""
        with tempfile.TemporaryDirectory() as d:
            w = PppStateWriter(
                receiver_uid="42",
                mount_sn=5,
                period_s=600.0,
                max_sigma_m=1.0,
                positions_dir=d,
                time_fn=self._time_fn,
            )
            wrote = w.maybe_write((100.0, 200.0, 300.0), 0.04, 999)
            self.assertTrue(wrote)
            loaded = load_ppp_state("42", positions_dir=d)
            self.assertIsNotNone(loaded)
            self.assertEqual(loaded.mount_sn, 5)
            self.assertAlmostEqual(loaded.ecef_m[0], 100.0)
            self.assertAlmostEqual(loaded.sigma_m, 0.04)
            self.assertEqual(loaded.extra.get("n_epochs"), 999)


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
