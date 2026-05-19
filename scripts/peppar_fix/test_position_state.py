"""Unit tests for position_state.py."""

import os
import tempfile
import unittest

from peppar_fix.position_state import (
    AntPosEstWatchdog,
    PositionState,
    PppStateWriter,
    SURVEY_TIE_BREAK_RATIO,
    WatchdogActor,
    _format_toml,
    bump_mount_sn,
    compute_horizontal_displacement,
    filter_current_mount,
    invalidate_ppp_state,
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

    def test_antennas_json_candidate_picked_when_only_one(self):
        """arp_label + antennas_path resolves to a survey-class
        candidate even when no .ppp.toml or .survey.toml exists."""
        import json
        ants = os.path.join(self.pos_dir, "antennas.json")
        with open(ants, "w") as f:
            json.dump({"choke1": {
                "ecef_m": [9.0, 8.0, 7.0],
                "sigma_m": 0.012,
                "method": "OPUS-Static",
            }}, f)
        out = seed_from_state_files(
            self.uid,
            arp_label="choke1",
            antennas_path=ants,
            positions_dir=self.pos_dir,
            receivers_dir=self.recv_dir)
        self.assertIsNotNone(out)
        self.assertEqual(out.kind, "survey")
        self.assertEqual(out.mount_sn, 3)  # inherited from current
        self.assertAlmostEqual(out.ecef_m[0], 9.0)
        self.assertAlmostEqual(out.sigma_m, 0.012)

    def test_antennas_json_loses_to_tighter_ppp(self):
        """If .ppp.toml has a much tighter sigma than antennas.json,
        the picker keeps ppp (no 2x survey tie-break can save a 10x
        worse survey σ)."""
        import json
        self._write_state("ppp", mount_sn=3, sigma_m=0.001, kind="ppp")
        ants = os.path.join(self.pos_dir, "antennas.json")
        with open(ants, "w") as f:
            json.dump({"choke1": {
                "ecef_m": [9.0, 8.0, 7.0],
                "sigma_m": 0.020,
                "method": "OPUS-Static",
            }}, f)
        out = seed_from_state_files(
            self.uid,
            arp_label="choke1",
            antennas_path=ants,
            positions_dir=self.pos_dir,
            receivers_dir=self.recv_dir)
        self.assertEqual(out.kind, "ppp")

    def test_ignore_survey_also_blocks_antennas_json(self):
        """--ignore-survey suppresses both .survey.toml AND
        antennas.json — both are survey-class data."""
        import json
        ants = os.path.join(self.pos_dir, "antennas.json")
        with open(ants, "w") as f:
            json.dump({"choke1": {
                "ecef_m": [9.0, 8.0, 7.0],
                "sigma_m": 0.001,
                "method": "OPUS-Static",
            }}, f)
        out = seed_from_state_files(
            self.uid,
            ignore_survey=True,
            arp_label="choke1",
            antennas_path=ants,
            positions_dir=self.pos_dir,
            receivers_dir=self.recv_dir)
        self.assertIsNone(out)


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

    def test_nav2_better_gate_preferred_over_absolute(self):
        """When nav2_h_acc_m is provided, the preferred relative
        gate (σ ≤ 0.5 × nav2_h_acc) replaces the absolute max."""
        # NAV2 hAcc 0.78 m → relative threshold 0.78 / 2 = 0.39 m.
        # σ=0.5 m would pass the absolute max_sigma_m=1.0 gate but
        # FAILS the relative gate (0.5 > 0.39).  This is exactly the
        # TimeHat 2026-05-18 incident: σ_ppp=0.95 m beat the
        # absolute gate but wasn't actually tighter than NAV2's
        # hAcc=0.78 m.
        w = self._make_writer(max_sigma_m=1.0)  # nav2_better=0.5
        wrote = w.maybe_write(
            (1, 2, 3), sigma_3d=0.5, n_epochs=10, nav2_h_acc_m=0.78)
        self.assertFalse(wrote)
        self.assertEqual(w.n_skipped_sigma, 1)
        # Same σ but NAV2 hAcc widens (e.g. multipath storm): now
        # PPP is meaningfully better than NAV2 (0.5 < 1.2 / 2 = 0.6)
        # so the write fires.
        wrote2 = w.maybe_write(
            (1, 2, 3), sigma_3d=0.5, n_epochs=11, nav2_h_acc_m=1.2)
        self.assertTrue(wrote2)

    def test_nav2_unavailable_falls_back_to_absolute(self):
        """When nav2_h_acc_m is None or non-positive, use the absolute
        max_sigma_m gate so a host with NAV2 loss can still warm-save."""
        w = self._make_writer(max_sigma_m=1.0)
        # No NAV2 hAcc → absolute gate applies.  σ=0.8 passes 1.0.
        wrote = w.maybe_write(
            (1, 2, 3), sigma_3d=0.8, n_epochs=10, nav2_h_acc_m=None)
        self.assertTrue(wrote)

    def test_nav2_zero_h_acc_falls_back_to_absolute(self):
        """Defensive: nav2_h_acc_m=0 (sentinel/garbage from store)
        shouldn't divide-by-zero the threshold."""
        w = self._make_writer(max_sigma_m=1.0)
        # Should fall back to absolute, not crash.
        wrote = w.maybe_write(
            (1, 2, 3), sigma_3d=0.8, n_epochs=10, nav2_h_acc_m=0.0)
        self.assertTrue(wrote)  # 0.8 ≤ 1.0 absolute

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


class TestComputeHorizontalDisplacement(unittest.TestCase):
    """Pure geometry: decompose ECEF Δ into horizontal + vertical
    against geocentric-up at point_a."""

    # A lab-class reference point (Chicago-ish at sea level — ECEF
    # values from real CHOKE1 antPos for plausibility).
    POINT = (157469.3814, -4756189.0729, 4232768.5274)

    def test_zero_displacement(self):
        d3, h, v = compute_horizontal_displacement(self.POINT, self.POINT)
        self.assertAlmostEqual(d3, 0.0, places=9)
        self.assertAlmostEqual(h, 0.0, places=9)
        self.assertAlmostEqual(v, 0.0, places=9)

    def test_pure_radial_displacement_is_all_vertical(self):
        """A radial-outward move at point_a should decompose to
        v ≈ delta, h ≈ 0."""
        import math
        ax, ay, az = self.POINT
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        scale = 1.0 + 1.0 / norm  # move 1 m radially outward
        b = (ax * scale, ay * scale, az * scale)
        # b - point = +1m radial from origin (so point - b is -1m radial)
        d3, h, v = compute_horizontal_displacement(self.POINT, b)
        # Vertical magnitude ≈ 1 m, horizontal ≈ 0
        self.assertAlmostEqual(v, 1.0, places=3)
        self.assertLess(h, 1e-6)
        self.assertAlmostEqual(d3, 1.0, places=3)

    def test_pure_horizontal_displacement_is_all_horizontal(self):
        """A move perpendicular to geocentric-up should decompose
        to h ≈ delta, v ≈ 0."""
        import math
        ax, ay, az = self.POINT
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        # Tangent vector at point_a in the X-Y plane around the polar
        # axis — perpendicular to "up" by construction.
        # u_hat = (ax, ay, az) / norm.  A vector orthogonal to u_hat
        # that lies in the X-Y plane: (-ay, ax, 0) normalised.
        tx, ty, tz = -ay, ax, 0.0
        tnorm = math.sqrt(tx * tx + ty * ty)
        tx, ty = tx / tnorm, ty / tnorm
        # Move 1m along the tangent.
        b = (ax + tx, ay + ty, az + tz)
        d3, h, v = compute_horizontal_displacement(self.POINT, b)
        self.assertAlmostEqual(h, 1.0, places=3)
        self.assertLess(v, 1e-6)
        self.assertAlmostEqual(d3, 1.0, places=3)

    def test_3d_magnitude_independent_of_decomposition(self):
        b = (self.POINT[0] + 1.0,
             self.POINT[1] - 2.0,
             self.POINT[2] + 0.5)
        d3, h, v = compute_horizontal_displacement(self.POINT, b)
        import math
        expected_d3 = math.sqrt(1.0 ** 2 + 2.0 ** 2 + 0.5 ** 2)
        self.assertAlmostEqual(d3, expected_d3, places=6)
        # Pythagoras: d3² ≈ h² + v²
        self.assertAlmostEqual(d3 * d3, h * h + v * v, places=4)

    def test_origin_point_a_falls_back_safely(self):
        """Pathological point_a at Earth's centre — function shouldn't
        crash and should return finite values."""
        d3, h, v = compute_horizontal_displacement(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(d3, 1.0)
        # In the degenerate case h = d3, v = 0 (documented fallback).
        self.assertAlmostEqual(h, 1.0)
        self.assertAlmostEqual(v, 0.0)


class TestAntPosEstWatchdog(unittest.TestCase):
    """Running-mean AntPosEst-vs-pin watchdog.  Pure-state, no
    threading; consumers read the dict snapshot via .state."""

    PIN = (100.0, 200.0, 300.0)

    def test_empty_state_before_set_pin(self):
        w = AntPosEstWatchdog()
        self.assertFalse(w.state["active"])
        self.assertEqual(w.state["n_in_mean"], 0)
        self.assertIsNone(w.state["sigma_3d_m"])

    def test_update_before_set_pin_is_noop(self):
        w = AntPosEstWatchdog()
        w.update((1.0, 2.0, 3.0), 0.01)
        # State unchanged.
        self.assertFalse(w.state["active"])
        self.assertEqual(w.state["n_in_mean"], 0)
        self.assertEqual(w.n_updates, 0)

    def test_set_pin_then_update_activates(self):
        w = AntPosEstWatchdog()
        w.set_pin(self.PIN)
        w.update(self.PIN, sigma_3d_m=0.01)
        self.assertTrue(w.state["active"])
        self.assertEqual(w.state["n_in_mean"], 1)
        self.assertEqual(w.state["sigma_3d_m"], 0.01)

    def test_zero_displacement_when_at_pin(self):
        w = AntPosEstWatchdog()
        w.set_pin(self.PIN)
        w.update(self.PIN, sigma_3d_m=0.01)
        self.assertAlmostEqual(w.state["displ_3d_m"], 0.0, places=6)
        self.assertAlmostEqual(w.state["displ_h_m"], 0.0, places=6)

    def test_high_sigma_skips_buffer(self):
        w = AntPosEstWatchdog(activation_sigma_m=0.05)
        w.set_pin(self.PIN)
        w.update((self.PIN[0] + 1.0, self.PIN[1], self.PIN[2]),
                 sigma_3d_m=10.0)  # cold-start σ
        self.assertFalse(w.state["active"])
        # σ field updated for visibility even on disarm.
        self.assertEqual(w.state["sigma_3d_m"], 10.0)
        # Buffer empty.
        self.assertEqual(w.state["n_in_mean"], 0)
        self.assertEqual(w.n_skipped_sigma, 1)
        self.assertEqual(w.n_updates, 0)

    def test_running_mean_tightens_with_n(self):
        """σ_mean = σ_per_epoch / √N; after 60 epochs with σ=0.012
        per epoch, σ_mean should be ~0.00155."""
        import math
        w = AntPosEstWatchdog(window_epochs=60)
        w.set_pin(self.PIN)
        for _ in range(60):
            w.update(self.PIN, sigma_3d_m=0.012)
        self.assertEqual(w.state["n_in_mean"], 60)
        expected_sigma_mean = 0.012 / math.sqrt(60)
        self.assertAlmostEqual(w.state["sigma_mean_m"],
                               expected_sigma_mean, places=6)

    def test_running_mean_is_actual_mean_of_buffer(self):
        """displ should reflect the mean of the buffered positions,
        not the latest reading."""
        w = AntPosEstWatchdog(window_epochs=10)
        w.set_pin(self.PIN)
        # Half at pin, half displaced 0.01 m along X.
        for _ in range(5):
            w.update(self.PIN, sigma_3d_m=0.01)
        for _ in range(5):
            w.update((self.PIN[0] + 0.01, self.PIN[1], self.PIN[2]),
                     sigma_3d_m=0.01)
        # Mean position is 0.005 m off in X — but X at our PIN is
        # purely along the geocentric-up direction (since pin is
        # along the +X axis when at (100, 200, 300)... actually it's
        # not perfectly radial).  Just check the 3D displacement.
        self.assertAlmostEqual(w.state["displ_3d_m"], 0.005, places=4)

    def test_buffer_is_ring_window_epochs(self):
        w = AntPosEstWatchdog(window_epochs=5)
        w.set_pin(self.PIN)
        for _ in range(20):
            w.update(self.PIN, sigma_3d_m=0.01)
        # Buffer capped at 5.
        self.assertEqual(w.state["n_in_mean"], 5)

    def test_set_pin_replace_for_slew(self):
        """Slice 7 will slew by re-calling set_pin with the
        refined ARP.  Verify the watchdog accepts that."""
        w = AntPosEstWatchdog()
        w.set_pin(self.PIN)
        w.update(self.PIN, sigma_3d_m=0.01)
        # Slew: new pin 0.1 m off.
        new_pin = (self.PIN[0] + 0.1, self.PIN[1], self.PIN[2])
        w.set_pin(new_pin)
        w.update(new_pin, sigma_3d_m=0.01)
        # We still have BOTH samples in the buffer.  Running mean
        # is halfway between PIN and new_pin.  Displacement vs
        # NEW pin is 0.05 m.  Slice 7 design accepts this — the
        # watchdog naturally re-tightens as the buffer ages out.
        self.assertEqual(w.state["n_in_mean"], 2)


class TestBumpMountSn(unittest.TestCase):

    def test_uid_none_returns_minus_one(self):
        self.assertEqual(bump_mount_sn(None), -1)

    def test_no_file_creates_with_one(self):
        with tempfile.TemporaryDirectory() as d:
            new = bump_mount_sn("12345", receivers_dir=d)
            self.assertEqual(new, 1)
            # Re-read to confirm persistence.
            self.assertEqual(load_current_mount_sn("12345",
                                                   receivers_dir=d), 1)

    def test_existing_file_increments(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "12345.json"), "w") as f:
                json.dump({"unique_id": 12345, "mount_sn": 7}, f)
            new = bump_mount_sn("12345", receivers_dir=d)
            self.assertEqual(new, 8)
            self.assertEqual(load_current_mount_sn("12345",
                                                   receivers_dir=d), 8)


class TestInvalidatePppState(unittest.TestCase):

    def test_uid_none_returns_false(self):
        self.assertFalse(invalidate_ppp_state(None))

    def test_no_file_returns_false(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertFalse(invalidate_ppp_state("nope",
                                                  positions_dir=d))

    def test_removes_existing(self):
        with tempfile.TemporaryDirectory() as d:
            s = PositionState(
                mount_sn=0,
                ecef_m=(1.0, 2.0, 3.0),
                sigma_m=0.01,
                updated=utc_now_iso(),
                source="test",
            )
            save_ppp_state(s, "1", positions_dir=d)
            self.assertTrue(os.path.exists(os.path.join(d, "1.ppp.toml")))
            removed = invalidate_ppp_state("1", positions_dir=d)
            self.assertTrue(removed)
            self.assertFalse(os.path.exists(
                os.path.join(d, "1.ppp.toml")))


class TestWatchdogActor(unittest.TestCase):
    """Slice 7 action decision logic.  Pure state, no side effects."""

    PIN = (100.0, 200.0, 300.0)
    SMALL_OFFSET_M = 0.05  # > antpos_threshold 0.03, < slew_step 1.0
    BIG_OFFSET_M = 2.5     # > slew_step 1.0

    def _antpos_state(self, displ_3d_m, n=60):
        return {
            "active": True,
            "n_in_mean": n,
            "sigma_3d_m": 0.01,
            "sigma_mean_m": 0.01 / max(n, 1) ** 0.5,
            "displ_3d_m": displ_3d_m,
            "displ_h_m": displ_3d_m,
            "displ_v_m": 0.0,
            "mean_ecef_m": (self.PIN[0] + displ_3d_m,
                            self.PIN[1], self.PIN[2]),
        }

    def _quiet_antpos_state(self):
        return self._antpos_state(displ_3d_m=0.0, n=60)

    def _inactive_antpos_state(self):
        st = self._antpos_state(displ_3d_m=0.0, n=0)
        st["active"] = False
        return st

    def test_no_bark_returns_none(self):
        a = WatchdogActor()
        out = a.evaluate(t_now=100.0,
                         nav2_disp_3d_m=0.1,
                         nav2_ecef_m=self.PIN,
                         antpos_state=self._quiet_antpos_state())
        self.assertEqual(out["action"], "none")

    def test_antpos_bark_before_sustain_no_action(self):
        a = WatchdogActor(antpos_sustain_s=60.0)
        # First bark — sustain hasn't accumulated yet.
        out = a.evaluate(t_now=100.0,
                         nav2_disp_3d_m=0.1,
                         nav2_ecef_m=self.PIN,
                         antpos_state=self._antpos_state(
                             self.SMALL_OFFSET_M))
        self.assertEqual(out["action"], "none")

    def test_antpos_slew_after_sustain(self):
        a = WatchdogActor(antpos_sustain_s=60.0,
                          slew_step_threshold_m=1.0)
        s = self._antpos_state(self.SMALL_OFFSET_M)
        # Bark starts at t=100.
        a.evaluate(100.0, 0.1, self.PIN, s)
        # Re-bark at t=160 (60 s elapsed) — slew fires.
        out = a.evaluate(160.0, 0.1, self.PIN, s)
        self.assertEqual(out["action"], "slew")
        self.assertEqual(out["source"], "antpos")
        self.assertAlmostEqual(out["displ_m"], self.SMALL_OFFSET_M)
        self.assertEqual(out["new_pin_ecef"], s["mean_ecef_m"])
        self.assertEqual(a.n_slew, 1)

    def test_antpos_slew_latch_blocks_repeat(self):
        a = WatchdogActor(antpos_sustain_s=60.0)
        s = self._antpos_state(self.SMALL_OFFSET_M)
        a.evaluate(100.0, 0.0, None, s)
        a.evaluate(160.0, 0.0, None, s)  # slew #1
        out = a.evaluate(220.0, 0.0, None, s)  # latched
        self.assertEqual(out["action"], "none")
        self.assertEqual(a.n_slew, 1)

    def test_antpos_latch_resets_when_quiet(self):
        a = WatchdogActor(antpos_sustain_s=60.0)
        s_bark = self._antpos_state(self.SMALL_OFFSET_M)
        s_quiet = self._quiet_antpos_state()
        a.evaluate(100.0, 0.0, None, s_bark)
        a.evaluate(160.0, 0.0, None, s_bark)  # slew #1
        # Caller actually slewed; next mean is at the new pin (zero
        # displacement) → bark goes quiet.
        a.evaluate(200.0, 0.0, None, s_quiet)
        # New bark with fresh start time.
        a.evaluate(260.0, 0.0, None, s_bark)
        out = a.evaluate(330.0, 0.0, None, s_bark)
        self.assertEqual(out["action"], "slew")
        self.assertEqual(a.n_slew, 2)

    def test_step_pending_before_auto_move_threshold(self):
        a = WatchdogActor(antpos_sustain_s=60.0,
                          auto_move_threshold_s=3600.0)
        s = self._antpos_state(self.BIG_OFFSET_M)
        a.evaluate(100.0, 0.0, None, s)
        out = a.evaluate(200.0, 0.0, None, s)
        self.assertEqual(out["action"], "step_pending")
        self.assertAlmostEqual(out["remaining_s"], 3500.0)

    def test_step_after_auto_move_threshold(self):
        a = WatchdogActor(antpos_sustain_s=60.0,
                          auto_move_threshold_s=3600.0)
        s = self._antpos_state(self.BIG_OFFSET_M)
        a.evaluate(100.0, 0.0, None, s)
        out = a.evaluate(100.0 + 3601.0, 0.0, None, s)
        self.assertEqual(out["action"], "step")
        self.assertEqual(out["source"], "antpos")
        self.assertEqual(a.n_step, 1)

    def test_auto_move_disabled_yields_persistent_pending(self):
        a = WatchdogActor(antpos_sustain_s=60.0,
                          auto_move_threshold_s=0.0)
        s = self._antpos_state(self.BIG_OFFSET_M)
        a.evaluate(100.0, 0.0, None, s)
        out = a.evaluate(100.0 + 10000.0, 0.0, None, s)
        self.assertEqual(out["action"], "step_pending")
        self.assertEqual(out["remaining_s"], float("inf"))

    def test_nav2_bark_drives_slew(self):
        # Explicit nav2_threshold_m=0.5 — tests the actor's logic
        # in isolation from the default 10 m threshold (which exists
        # to absorb the known 1.5-4 m NAV2 bias; see CLAUDE.md
        # "NAV2 bias" section).
        a = WatchdogActor(nav2_sustain_s=60.0, nav2_threshold_m=0.5)
        new_pin = (self.PIN[0] + 0.4, self.PIN[1], self.PIN[2])
        a.evaluate(100.0, 0.6, new_pin, self._inactive_antpos_state())
        out = a.evaluate(170.0, 0.6, new_pin,
                         self._inactive_antpos_state())
        self.assertEqual(out["action"], "slew")
        self.assertEqual(out["source"], "nav2")
        self.assertEqual(out["new_pin_ecef"], new_pin)

    def test_larger_displacement_wins(self):
        """When both nav2 and antpos are barking, the source with
        larger displacement gets the action.  Explicit
        nav2_threshold_m so the test value of 0.7 m barks."""
        a = WatchdogActor(antpos_sustain_s=60.0, nav2_sustain_s=60.0,
                          nav2_threshold_m=0.5)
        antpos = self._antpos_state(self.SMALL_OFFSET_M)  # 0.05 m
        nav2_disp = 0.7  # 0.7 m
        nav2_pin = (self.PIN[0] + 0.7, self.PIN[1], self.PIN[2])
        a.evaluate(100.0, nav2_disp, nav2_pin, antpos)
        out = a.evaluate(170.0, nav2_disp, nav2_pin, antpos)
        self.assertEqual(out["action"], "slew")
        # NAV2 has larger displacement → its source/pin used.
        self.assertEqual(out["source"], "nav2")

    def test_nav2_default_threshold_absorbs_known_bias(self):
        """Documented NAV2 bias of 1.5-4 m must not trigger the
        default WatchdogActor.  This pins the regression we just
        fixed: 0.5 m threshold caused continuous false positives
        on the lab hosts."""
        a = WatchdogActor()  # all defaults — nav2_threshold_m=10.0
        # MadHat's observed 1.6 m NAV2 disagreement (CONFIDENCE
        # cycles on 2026-05-18).  Must NOT bark.
        a.evaluate(100.0, 1.6, self.PIN, self._inactive_antpos_state())
        out = a.evaluate(200.0, 1.6, self.PIN,
                         self._inactive_antpos_state())
        self.assertEqual(out["action"], "none")
        # 11 m DOES bark (real antenna-fell-off-mast event).
        a2 = WatchdogActor()
        a2.evaluate(100.0, 11.0, self.PIN,
                    self._inactive_antpos_state())
        out2 = a2.evaluate(200.0, 11.0, self.PIN,
                           self._inactive_antpos_state())
        # 11 m > slew_step_threshold (1 m default) → step path,
        # but auto_move_threshold_s (3600 s default) not yet
        # elapsed → step_pending.
        self.assertEqual(out2["action"], "step_pending")

    def test_inactive_antpos_ignored(self):
        """When AntPosEst is unarmed (cold start), NAV2 alone can
        still drive action."""
        a = WatchdogActor(antpos_sustain_s=60.0, nav2_sustain_s=60.0)
        antpos = self._inactive_antpos_state()
        # AntPosEst is unarmed so its displ shouldn't enter the
        # candidates — even if displ field happens to be nonzero.
        antpos["displ_3d_m"] = 5.0  # would-be huge if armed
        a.evaluate(100.0, 0.0, None, antpos)
        out = a.evaluate(200.0, 0.0, None, antpos)
        self.assertEqual(out["action"], "none")


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


class TestSaveSurveyState(unittest.TestCase):

    def test_round_trip(self):
        from peppar_fix.position_state import save_survey_state
        with tempfile.TemporaryDirectory() as d:
            s = PositionState(
                mount_sn=3,
                ecef_m=(157469.3707, -4756188.9618, 4232768.4597),
                sigma_m=0.0125,
                updated=utc_now_iso(),
                source="OPUS 6-day mean",
                kind="survey",
                extra={"method": "OPUS-Static"},
            )
            save_survey_state(s, "12345", positions_dir=d)
            loaded = load_survey_state("12345", positions_dir=d)
            self.assertIsNotNone(loaded)
            self.assertEqual(loaded.kind, "survey")
            self.assertEqual(loaded.mount_sn, 3)
            self.assertAlmostEqual(loaded.sigma_m, 0.0125, places=6)
            self.assertEqual(loaded.extra.get("method"), "OPUS-Static")

    def test_rejects_non_survey_kind(self):
        """Defensive: save_survey_state must refuse kind='ppp' to
        prevent mis-tagging (counterpart to save_ppp_state's guard)."""
        from peppar_fix.position_state import save_survey_state
        with tempfile.TemporaryDirectory() as d:
            s = PositionState(
                mount_sn=0,
                ecef_m=(1.0, 2.0, 3.0),
                sigma_m=0.01,
                updated=utc_now_iso(),
                source="test",
                kind="ppp",
            )
            with self.assertRaises(ValueError):
                save_survey_state(s, "1", positions_dir=d)

    def test_atomic_write(self):
        from peppar_fix.position_state import save_survey_state
        with tempfile.TemporaryDirectory() as d:
            s = PositionState(
                mount_sn=0, ecef_m=(1.0, 2.0, 3.0), sigma_m=0.01,
                updated=utc_now_iso(), source="test", kind="survey")
            save_survey_state(s, "9", positions_dir=d)
            entries = os.listdir(d)
            self.assertIn("9.survey.toml", entries)
            self.assertNotIn("9.survey.toml.tmp", entries)


class TestLoadArpFromAntennas(unittest.TestCase):
    """Read timelab/antennas.json → in-memory PositionState (read-only;
    no file write).  See position_state.load_arp_from_antennas."""

    def setUp(self):
        import json
        # Synthetic antennas.json matching the real schema, but with
        # placeholder coords (the hooks/pre-commit hook rejects
        # literal lab coordinates).  We're testing schema parsing,
        # not coordinate values.
        self.tmp = tempfile.mkdtemp()
        self.antennas_path = os.path.join(self.tmp, "antennas.json")
        with open(self.antennas_path, "w") as f:
            json.dump({
                "test_dict_ecef": {
                    "lat": 12.345678,
                    "lon": 45.678901,
                    "alt_m": 100.0,
                    "sigma_m": 0.0125,
                    "ecef_m": {
                        "x": 1000.111,
                        "y": -2000.222,
                        "z": 3000.333,
                    },
                    "method": "OPUS-Static, 6-day mean",
                    "updated": "2026-05-12",
                },
                "test_list_ecef": {
                    "ecef_m": [1111.222, -2222.333, 3333.444],
                    "sigma_m": 0.009,
                    "method": "OPUS-Static (legacy ecef format)",
                },
            }, f)

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmp)

    def test_happy_path_dict_ecef(self):
        from peppar_fix.position_state import load_arp_from_antennas
        s = load_arp_from_antennas(
            "test_dict_ecef", mount_sn=2,
            antennas_path=self.antennas_path)
        self.assertIsNotNone(s)
        self.assertEqual(s.kind, "survey")
        self.assertEqual(s.mount_sn, 2)
        self.assertAlmostEqual(s.ecef_m[0], 1000.111, places=4)
        self.assertAlmostEqual(s.sigma_m, 0.0125, places=6)
        self.assertIn("test_dict_ecef", s.source)
        self.assertIn("OPUS", s.source)
        self.assertEqual(s.extra.get("arp_label"), "test_dict_ecef")
        self.assertEqual(s.extra.get("antennas_method"),
                         "OPUS-Static, 6-day mean")

    def test_list_ecef_format_also_supported(self):
        """antennas.json's legacy format uses ecef_m as a list, not
        {x,y,z}.  Reader handles both."""
        from peppar_fix.position_state import load_arp_from_antennas
        s = load_arp_from_antennas(
            "test_list_ecef", mount_sn=0,
            antennas_path=self.antennas_path)
        self.assertIsNotNone(s)
        self.assertAlmostEqual(s.ecef_m[0], 1111.222, places=3)

    def test_unknown_arp_label_returns_none(self):
        from peppar_fix.position_state import load_arp_from_antennas
        s = load_arp_from_antennas(
            "no-such-label", mount_sn=0,
            antennas_path=self.antennas_path)
        self.assertIsNone(s)

    def test_empty_arp_label_returns_none(self):
        from peppar_fix.position_state import load_arp_from_antennas
        for empty in (None, ""):
            self.assertIsNone(load_arp_from_antennas(
                empty, mount_sn=0, antennas_path=self.antennas_path))

    def test_missing_file_returns_none(self):
        from peppar_fix.position_state import load_arp_from_antennas
        s = load_arp_from_antennas(
            "test_dict_ecef", mount_sn=0,
            antennas_path="/nope/nope/antennas.json")
        self.assertIsNone(s)

    def test_malformed_json_returns_none(self):
        from peppar_fix.position_state import load_arp_from_antennas
        p = os.path.join(self.tmp, "broken.json")
        with open(p, "w") as f:
            f.write("not valid json {{{")
        self.assertIsNone(load_arp_from_antennas(
            "test_dict_ecef", mount_sn=0, antennas_path=p))

    def test_entry_missing_required_field_returns_none(self):
        import json
        from peppar_fix.position_state import load_arp_from_antennas
        p = os.path.join(self.tmp, "missing.json")
        with open(p, "w") as f:
            json.dump({"x": {"ecef_m": [1, 2, 3]}}, f)  # no sigma_m
        self.assertIsNone(load_arp_from_antennas(
            "x", mount_sn=0, antennas_path=p))


class TestFindAntennasJson(unittest.TestCase):
    """Path discovery: explicit > $PEPPAR_ANTENNAS_JSON > standard list."""

    def test_explicit_path_wins(self):
        from peppar_fix.position_state import find_antennas_json
        with tempfile.NamedTemporaryFile(mode="w",
                                          suffix=".json",
                                          delete=False) as f:
            f.write("{}")
            path = f.name
        try:
            self.assertEqual(find_antennas_json(path), path)
        finally:
            os.unlink(path)

    def test_nonexistent_explicit_falls_through(self):
        # When the explicit path doesn't exist AND no env var AND no
        # standard path, return None.  We can't reliably remove the
        # standard candidates from this test process so we just verify
        # explicit-doesn't-exist doesn't raise.
        from peppar_fix.position_state import find_antennas_json
        # If the dev box has ~/timelab/antennas.json this will be that
        # path; either way the call must not raise.
        result = find_antennas_json("/definitely/not/a/path")
        self.assertTrue(result is None or os.path.isfile(result))


if __name__ == "__main__":
    unittest.main()
