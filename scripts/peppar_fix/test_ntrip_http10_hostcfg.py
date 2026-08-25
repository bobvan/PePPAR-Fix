"""ntrip_http10 must be settable from the per-host TOML.

Pulling from our own str2str `ntripc` re-casters on gt (mounts SSR/PTBB/BCEP)
requires NTRIP v1 framing: str2str answers "ICY 200 OK\r\n" and then streams
RTCM immediately, with NO blank line.  An HTTP/1.1 client reads RTCM bytes into
the header and mis-frames the stream.  --ntrip-http10 selects v1 framing, and it
has to be expressible per host, because whether a host talks to a v1 re-caster
or a v2 public caster is a property of the host's config, not of the invocation.

The trap this guards is the same one caught on MadHat 2026-05-22 for
--perout-period-ns: _apply_host_config skips any dest whose value is not None,
so an `action="store_true"` flag (argparse default False) can never be set from
the TOML -- `ntrip_http10 = true` would be silently inert.
"""

import os
import tempfile
import unittest


class TestNtripHttp10HostConfig(unittest.TestCase):

    def _apply(self, toml_text):
        """Run _apply_host_config against a temp TOML, return the args."""
        import peppar_fix_engine
        from types import SimpleNamespace

        with tempfile.NamedTemporaryFile("w", suffix=".toml",
                                         delete=False) as f:
            f.write(toml_text)
            path = f.name
        self.addCleanup(os.unlink, path)

        # Only the dests the mapping touches need to exist; everything else
        # is looked up with getattr(..., None) and skipped.
        args = SimpleNamespace(host_config=path, ntrip_http10=None,
                               ntrip_conf=None)
        peppar_fix_engine._apply_host_config(args)
        return args

    def test_toml_true_reaches_args(self):
        args = self._apply("[peppar]\nntrip_http10 = true\n")
        self.assertTrue(
            args.ntrip_http10,
            "ntrip_http10 = true in the host TOML must reach args; if this "
            "fails the engine will speak HTTP/1.1 to a v1-only re-caster and "
            "mis-frame the RTCM stream")

    def test_toml_false_reaches_args(self):
        args = self._apply("[peppar]\nntrip_http10 = false\n")
        self.assertFalse(args.ntrip_http10)

    def test_absent_from_toml_leaves_none(self):
        args = self._apply("[peppar]\nntrip_conf = 'ntrip.conf'\n")
        self.assertIsNone(args.ntrip_http10)

    def test_cli_wins_over_toml(self):
        """CLI already set it -> _apply_host_config must not clobber it."""
        import peppar_fix_engine
        from types import SimpleNamespace

        with tempfile.NamedTemporaryFile("w", suffix=".toml",
                                         delete=False) as f:
            f.write("[peppar]\nntrip_http10 = false\n")
            path = f.name
        self.addCleanup(os.unlink, path)

        args = SimpleNamespace(host_config=path, ntrip_http10=True,
                               ntrip_conf=None)
        peppar_fix_engine._apply_host_config(args)
        self.assertTrue(args.ntrip_http10,
                        "an explicit --ntrip-http10 must survive the host TOML")

    def test_argparse_default_is_none(self):
        """Source guard: store_true's default of False would make the TOML
        key inert, because _apply_host_config skips non-None dests."""
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        engine = os.path.join(here, "peppar_fix_engine.py")
        with open(engine) as f:
            src = f.read()
        idx = src.find('"--ntrip-http10"')
        self.assertGreater(idx, 0, "--ntrip-http10 flag not found")
        snippet = src[idx:idx + 300]
        self.assertIn(
            "default=None", snippet,
            "--ntrip-http10 must use default=None so _apply_host_config can "
            "set it from the per-host TOML; with store_true's False default "
            "the TOML key is silently ignored")


if __name__ == "__main__":
    unittest.main()
