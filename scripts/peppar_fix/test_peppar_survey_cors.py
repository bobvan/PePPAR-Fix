"""Tests for the peppar-survey --cors-ntrip-* live-NTRIP base capture
helpers.

Both subprocesses (str2str + convbin) are dependency-injected so the
test suite doesn't depend on RTKLIB being installed.  Where we need
to assert process-control behavior (terminate on duration, kill on
SIGTERM-ignore) we use small fake Popen objects.
"""
from __future__ import annotations

import os
import subprocess
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import mock

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.peppar_survey_cors import (
    CorsNtripConfig, DEFAULT_CORS_NTRIP_DURATION_S,
    _redact_cmd, capture_cors_base_via_ntrip, ntrip_url,
    rtcm3_to_rinex, stream_rtcm_from_ntrip,
)


# ── CorsNtripConfig validation ─────────────────────────────────── #


class CorsNtripConfigTest(unittest.TestCase):

    def test_minimal_valid(self):
        cfg = CorsNtripConfig(host="peer.lab", port=2102, mount="PEPPAR")
        self.assertEqual(cfg.host, "peer.lab")
        self.assertEqual(cfg.port, 2102)
        self.assertEqual(cfg.mount, "PEPPAR")
        self.assertEqual(cfg.duration_s, DEFAULT_CORS_NTRIP_DURATION_S)

    def test_empty_host_rejected(self):
        with self.assertRaises(ValueError):
            CorsNtripConfig(host="", port=2102, mount="PEPPAR")

    def test_port_out_of_range_rejected(self):
        with self.assertRaises(ValueError):
            CorsNtripConfig(host="peer", port=0, mount="PEPPAR")
        with self.assertRaises(ValueError):
            CorsNtripConfig(host="peer", port=65536, mount="PEPPAR")

    def test_empty_mount_rejected(self):
        with self.assertRaises(ValueError):
            CorsNtripConfig(host="peer", port=2102, mount="")

    def test_nonpositive_duration_rejected(self):
        with self.assertRaises(ValueError):
            CorsNtripConfig(host="peer", port=2102, mount="PEPPAR",
                            duration_s=0)


class NtripUrlTest(unittest.TestCase):
    """str2str's NTRIP client URL scheme is ``ntrip://`` per its -h
    table.  An earlier draft used ``ntripcli://`` and silently
    captured zero bytes; caught by branch-main 2026-05-20 against
    NAPERVILLE-RTCM3.1-MSM5 (0 B vs 5.7 KB in 8 s)."""

    def test_no_auth(self):
        cfg = CorsNtripConfig(host="peer.lab", port=2102, mount="PEPPAR")
        self.assertEqual(
            ntrip_url(cfg),
            "ntrip://peer.lab:2102/PEPPAR")

    def test_user_only(self):
        cfg = CorsNtripConfig(host="peer", port=2102, mount="PEPPAR",
                              user="alice")
        self.assertEqual(
            ntrip_url(cfg),
            "ntrip://alice@peer:2102/PEPPAR")

    def test_user_and_password(self):
        cfg = CorsNtripConfig(host="peer", port=2102, mount="PEPPAR",
                              user="alice", password="secret")
        self.assertEqual(
            ntrip_url(cfg),
            "ntrip://alice:secret@peer:2102/PEPPAR")

    def test_scheme_is_not_ntripcli(self):
        """Regression guard for the bug branch-main caught: any URL
        we hand to str2str must start with 'ntrip://', NOT
        'ntripcli://'.  str2str silently captures 0 bytes from the
        latter."""
        cfg = CorsNtripConfig(host="peer", port=2102, mount="PEPPAR")
        url = ntrip_url(cfg)
        self.assertTrue(url.startswith("ntrip://"),
                        f"unexpected scheme: {url!r}")
        self.assertFalse(url.startswith("ntripcli://"),
                         f"ntripcli scheme regressed: {url!r}")


# ── stream_rtcm_from_ntrip subprocess behavior ─────────────────── #


class FakeProc:
    """Stand-in for subprocess.Popen capturing the canonical lifecycle
    methods (wait, terminate, kill, poll, stderr.read) that
    stream_rtcm_from_ntrip uses.

    Each instance is parameterized by what happens when wait() is
    called with a timeout: ``wait_behavior`` ∈ {
      'finish_promptly': returns 0 immediately
      'timeout_then_exit_on_terminate': raises TimeoutExpired the first
          time; subsequent waits succeed; terminate() succeeds
      'ignores_terminate': raises TimeoutExpired on every wait until
          kill() is called
    }.
    """

    def __init__(self, output_path: Path, bytes_to_write: int,
                 wait_behavior: str = "timeout_then_exit_on_terminate"):
        self.output_path = Path(output_path)
        self.bytes_to_write = bytes_to_write
        self.wait_behavior = wait_behavior
        self._waits = 0
        self._terminated = False
        self._killed = False
        self._exited = False
        self._wrote_output = False
        self.stderr = mock.Mock()
        self.stderr.read = mock.Mock(return_value="")

    def _ensure_output_written(self):
        # Mimic str2str's behavior: bytes accumulate during the run,
        # so the file exists once wait() has been entered (the caller
        # has clear-then-Popen-then-wait ordered, so bytes appear after
        # the clear).  Idempotent.
        if self._wrote_output or self.bytes_to_write <= 0:
            return
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_bytes(b"\xd3\x00\x01" * self.bytes_to_write)
        self._wrote_output = True

    def wait(self, timeout=None):
        self._ensure_output_written()
        self._waits += 1
        if self.wait_behavior == "finish_promptly":
            self._exited = True
            return 0
        if self.wait_behavior == "timeout_then_exit_on_terminate":
            if not self._terminated:
                raise subprocess.TimeoutExpired(cmd="str2str", timeout=timeout)
            self._exited = True
            return 0
        if self.wait_behavior == "ignores_terminate":
            if not self._killed:
                raise subprocess.TimeoutExpired(cmd="str2str", timeout=timeout)
            self._exited = True
            return 0
        raise AssertionError(f"unhandled wait_behavior: {self.wait_behavior}")

    def terminate(self):
        self._terminated = True

    def kill(self):
        self._killed = True

    def poll(self):
        return 0 if self._exited else None


class NmeaPosValidationTest(unittest.TestCase):

    def test_accepts_valid_position(self):
        cfg = CorsNtripConfig(host="h", port=2101, mount="M",
                              nmea_pos=(43.98, -87.78, 200.0))
        self.assertEqual(cfg.nmea_pos, (43.98, -87.78, 200.0))

    def test_rejects_wrong_arity(self):
        with self.assertRaises(ValueError):
            CorsNtripConfig(host="h", port=2101, mount="M",
                            nmea_pos=(43.98, -87.78))

    def test_rejects_out_of_range(self):
        for bad in ((91.0, 0.0, 0.0), (0.0, 181.0, 0.0)):
            with self.assertRaises(ValueError):
                CorsNtripConfig(host="h", port=2101, mount="M", nmea_pos=bad)

    def test_none_is_allowed(self):
        self.assertIsNone(
            CorsNtripConfig(host="h", port=2101, mount="M").nmea_pos)


class RedactCmdTest(unittest.TestCase):
    """The str2str command carries credentials; logs must not."""

    def test_masks_password(self):
        cmd = ["str2str", "-in", "ntrip://ind/bobvv:s3cret@h:2101/M"]
        self.assertEqual(_redact_cmd(cmd)[2],
                         "ntrip://ind/bobvv:***@h:2101/M")
        self.assertNotIn("s3cret", " ".join(_redact_cmd(cmd)))

    def test_leaves_unauthenticated_url_alone(self):
        cmd = ["str2str", "-in", "ntrip://peer:2102/PEPPAR"]
        self.assertEqual(_redact_cmd(cmd), cmd)


class StreamRtcmFromNtripTest(unittest.TestCase):

    def _cfg(self, duration_s=1, **kw):
        return CorsNtripConfig(host="peer", port=2102, mount="PEPPAR",
                               duration_s=duration_s, **kw)

    def _captured_cmd(self, cfg):
        """Run stream_rtcm_from_ntrip and return the argv str2str got."""
        with TemporaryDirectory() as td:
            out = Path(td) / "stream.rtcm3"
            fake = FakeProc(out, bytes_to_write=10)
            with mock.patch(
                "peppar_fix.peppar_survey_cors.subprocess.Popen",
                return_value=fake,
            ) as popen, mock.patch(
                "peppar_fix.peppar_survey_cors.time.sleep",
            ):
                stream_rtcm_from_ntrip(cfg, out, str2str_bin="/x/str2str")
            return popen.call_args[0][0]

    def test_no_gga_flags_without_nmea_pos(self):
        cmd = self._captured_cmd(self._cfg())
        self.assertNotIn("-p", cmd)
        self.assertNotIn("-n", cmd)

    def test_sends_gga_position_when_set(self):
        """Network / nearest-base mounts stream nothing without a GGA."""
        cmd = self._captured_cmd(
            self._cfg(nmea_pos=(43.98, -87.78, 200.0)))
        i = cmd.index("-p")
        self.assertEqual(cmd[i + 1:i + 4],
                         ["43.980000000", "-87.780000000", "200.0000"])
        j = cmd.index("-n")
        self.assertEqual(cmd[j + 1], "10000")

    def test_nmea_cycle_zero_omits_n_flag(self):
        cmd = self._captured_cmd(
            self._cfg(nmea_pos=(43.98, -87.78, 0.0), nmea_cycle_ms=0))
        self.assertIn("-p", cmd)
        self.assertNotIn("-n", cmd)

    def test_success_after_duration(self):
        with TemporaryDirectory() as td:
            out = Path(td) / "stream.rtcm3"
            cfg = self._cfg(duration_s=1)
            fake = FakeProc(out, bytes_to_write=10)
            with mock.patch(
                "peppar_fix.peppar_survey_cors.subprocess.Popen",
                return_value=fake,
            ), mock.patch(
                "peppar_fix.peppar_survey_cors.time.sleep",
            ):
                got = stream_rtcm_from_ntrip(cfg, out, str2str_bin="/x/str2str")
            self.assertEqual(got, out)
            self.assertTrue(fake._terminated)
            self.assertFalse(fake._killed)

    def test_empty_output_returns_none(self):
        with TemporaryDirectory() as td:
            out = Path(td) / "stream.rtcm3"
            cfg = self._cfg(duration_s=1)
            fake = FakeProc(out, bytes_to_write=0)  # nothing captured
            with mock.patch(
                "peppar_fix.peppar_survey_cors.subprocess.Popen",
                return_value=fake,
            ), mock.patch(
                "peppar_fix.peppar_survey_cors.time.sleep",
            ):
                got = stream_rtcm_from_ntrip(cfg, out, str2str_bin="/x/str2str")
            self.assertIsNone(got)

    def test_str2str_missing_returns_none(self):
        with TemporaryDirectory() as td:
            out = Path(td) / "stream.rtcm3"
            cfg = self._cfg(duration_s=1)
            with mock.patch(
                "peppar_fix.peppar_survey_cors.subprocess.Popen",
                side_effect=FileNotFoundError("no str2str"),
            ):
                got = stream_rtcm_from_ntrip(cfg, out, str2str_bin="/nope")
            self.assertIsNone(got)

    def test_terminate_ignored_escalates_to_kill(self):
        with TemporaryDirectory() as td:
            out = Path(td) / "stream.rtcm3"
            cfg = self._cfg(duration_s=1)
            fake = FakeProc(out, bytes_to_write=10,
                            wait_behavior="ignores_terminate")
            with mock.patch(
                "peppar_fix.peppar_survey_cors.subprocess.Popen",
                return_value=fake,
            ), mock.patch(
                "peppar_fix.peppar_survey_cors.time.sleep",
            ):
                got = stream_rtcm_from_ntrip(
                    cfg, out, str2str_bin="/x/str2str",
                    terminate_grace_s=1,
                )
            self.assertEqual(got, out)
            self.assertTrue(fake._terminated)
            self.assertTrue(fake._killed)

    def test_clears_prior_file_before_run(self):
        with TemporaryDirectory() as td:
            out = Path(td) / "stream.rtcm3"
            out.write_bytes(b"STALE")
            cfg = self._cfg(duration_s=1)
            fake = FakeProc(out, bytes_to_write=10)
            with mock.patch(
                "peppar_fix.peppar_survey_cors.subprocess.Popen",
                return_value=fake,
            ), mock.patch(
                "peppar_fix.peppar_survey_cors.time.sleep",
            ):
                got = stream_rtcm_from_ntrip(cfg, out, str2str_bin="/x/str2str")
            self.assertEqual(got, out)
            # FakeProc rewrites the file with non-STALE bytes; verify
            # the new content rather than the prior STALE bytes.
            self.assertNotEqual(out.read_bytes(), b"STALE")
            self.assertEqual(out.stat().st_size, 30)  # 3 bytes * 10


# ── rtcm3_to_rinex subprocess behavior ─────────────────────────── #


class Rtcm3ToRinexTest(unittest.TestCase):

    def _make_rtcm(self, td: Path, size: int = 100) -> Path:
        p = td / "input.rtcm3"
        p.write_bytes(b"\xd3" * size)
        return p

    def test_missing_input_returns_none(self):
        with TemporaryDirectory() as td:
            got = rtcm3_to_rinex(
                Path(td) / "nope.rtcm3", Path(td) / "work",
                convbin_bin="/x/convbin")
            self.assertIsNone(got)

    def test_empty_input_returns_none(self):
        with TemporaryDirectory() as td:
            rtcm = Path(td) / "empty.rtcm3"
            rtcm.write_bytes(b"")
            got = rtcm3_to_rinex(
                rtcm, Path(td) / "work",
                convbin_bin="/x/convbin")
            self.assertIsNone(got)

    def test_success_emits_obs(self):
        with TemporaryDirectory() as td:
            rtcm = self._make_rtcm(Path(td))
            work = Path(td) / "work"

            def fake_run(cmd, **kwargs):
                # Pretend convbin wrote the .obs file before returning.
                out_obs = work / "cors-ntrip.obs"
                out_obs.write_text(
                    "     3.04           OBSERVATION DATA    M (MIXED)\n")
                return mock.Mock(returncode=0, stdout="", stderr="")

            with mock.patch(
                "peppar_fix.peppar_survey_cors.subprocess.run",
                side_effect=fake_run,
            ):
                got = rtcm3_to_rinex(
                    rtcm, work,
                    convbin_bin="/x/convbin",
                )
            self.assertEqual(got, work / "cors-ntrip.obs")
            self.assertGreater(got.stat().st_size, 0)

    def test_convbin_failure_returns_none(self):
        with TemporaryDirectory() as td:
            rtcm = self._make_rtcm(Path(td))
            work = Path(td) / "work"
            with mock.patch(
                "peppar_fix.peppar_survey_cors.subprocess.run",
                return_value=mock.Mock(
                    returncode=1, stdout="", stderr="convbin: bad input"),
            ):
                got = rtcm3_to_rinex(
                    rtcm, work, convbin_bin="/x/convbin")
            self.assertIsNone(got)

    def test_convbin_missing_returns_none(self):
        with TemporaryDirectory() as td:
            rtcm = self._make_rtcm(Path(td))
            with mock.patch(
                "peppar_fix.peppar_survey_cors.subprocess.run",
                side_effect=FileNotFoundError("no convbin"),
            ):
                got = rtcm3_to_rinex(
                    rtcm, Path(td) / "work", convbin_bin="/nope")
            self.assertIsNone(got)

    def test_convbin_timeout_returns_none(self):
        with TemporaryDirectory() as td:
            rtcm = self._make_rtcm(Path(td))
            with mock.patch(
                "peppar_fix.peppar_survey_cors.subprocess.run",
                side_effect=subprocess.TimeoutExpired(
                    cmd="convbin", timeout=10),
            ):
                got = rtcm3_to_rinex(
                    rtcm, Path(td) / "work",
                    convbin_bin="/x/convbin", timeout_s=10)
            self.assertIsNone(got)

    def test_emits_obs_but_empty_returns_none(self):
        with TemporaryDirectory() as td:
            rtcm = self._make_rtcm(Path(td))
            work = Path(td) / "work"

            def fake_run(cmd, **kwargs):
                out_obs = work / "cors-ntrip.obs"
                out_obs.touch()  # empty
                return mock.Mock(returncode=0, stdout="", stderr="")

            with mock.patch(
                "peppar_fix.peppar_survey_cors.subprocess.run",
                side_effect=fake_run,
            ):
                got = rtcm3_to_rinex(
                    rtcm, work, convbin_bin="/x/convbin")
            self.assertIsNone(got)


# ── capture_cors_base_via_ntrip end-to-end (with injected mocks) ── #


class CaptureCorsBaseTest(unittest.TestCase):

    def test_streamer_failure_short_circuits(self):
        cfg = CorsNtripConfig(host="peer", port=2102, mount="PEPPAR")
        with TemporaryDirectory() as td:
            work = Path(td) / "work"
            stream_calls = []
            convert_calls = []

            def fake_streamer(c, p, *, str2str_bin):
                stream_calls.append((c, p))
                return None  # simulate stream failure

            def fake_converter(p, w, *, output_obs_name, convbin_bin):
                convert_calls.append((p, w))
                return w / output_obs_name

            got = capture_cors_base_via_ntrip(
                cfg, work,
                ntrip_streamer=fake_streamer,
                rtcm_converter=fake_converter,
            )
            self.assertIsNone(got)
            self.assertEqual(len(stream_calls), 1)
            self.assertEqual(len(convert_calls), 0)

    def test_converter_failure_returns_none(self):
        cfg = CorsNtripConfig(host="peer", port=2102, mount="PEPPAR")
        with TemporaryDirectory() as td:
            work = Path(td) / "work"

            def fake_streamer(c, p, *, str2str_bin):
                # Honor the streamer's contract: write something
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_bytes(b"\xd3" * 50)
                return p

            def fake_converter(p, w, *, output_obs_name, convbin_bin):
                return None  # convbin failure

            got = capture_cors_base_via_ntrip(
                cfg, work,
                ntrip_streamer=fake_streamer,
                rtcm_converter=fake_converter,
            )
            self.assertIsNone(got)

    def test_happy_path(self):
        cfg = CorsNtripConfig(host="peer", port=2102, mount="PEPPAR")
        with TemporaryDirectory() as td:
            work = Path(td) / "work"

            def fake_streamer(c, p, *, str2str_bin):
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_bytes(b"\xd3" * 50)
                return p

            def fake_converter(p, w, *, output_obs_name, convbin_bin):
                out = w / output_obs_name
                out.parent.mkdir(parents=True, exist_ok=True)
                out.write_text("dummy obs\n")
                return out

            got = capture_cors_base_via_ntrip(
                cfg, work,
                output_obs_name="my-base.obs",
                ntrip_streamer=fake_streamer,
                rtcm_converter=fake_converter,
            )
            self.assertEqual(got, work / "my-base.obs")
            self.assertTrue(got.is_file())


if __name__ == "__main__":
    unittest.main()
