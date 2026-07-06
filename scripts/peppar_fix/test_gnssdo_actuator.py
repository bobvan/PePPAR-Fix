"""Unit tests for GnssdoActuator (SXT-D $W console actuator).

Uses a fake console link that simulates the firmware's $R/$W/$E/$T
replies, so the actuator's protocol + ppb↔control-word math are tested
without hardware.
"""

import re

import pytest

from peppar_fix.gnssdo_actuator import (
    GnssdoActuator, GnssdoConsoleError, _STP3593LF_PPB_PER_CODE)


class FakeConsole:
    """Simulates the GNSSDO+ external-control console."""

    def __init__(self, start_word=489533, word_max=(1 << 20) - 1):
        self.word = start_word
        self.word_max = word_max
        self.ext = 0
        self.watchdog = 15
        self.state = "WARMUP"
        self.opened = False
        self.sent = []          # every command line
        # These mirror _ConsoleLink attributes the actuator peeks at for logs
        self._tcp = None
        self._serial_port = "/dev/fake"

    def open(self):
        self.opened = True

    def close(self):
        self.opened = False

    def send(self, line):
        self.sent.append(line)

    def request(self, line, want, timeout=None):
        self.sent.append(line)
        reply = self._reply(line)
        m = want.match(reply)
        if not m:
            raise GnssdoConsoleError("fake: %r did not match for %r"
                                     % (reply, line))
        return m

    def _reply(self, line):
        if line == "$R":
            return "$R,%d,%d,%.6e,%s,%d,%d" % (
                self.ext, self.word, -1.2e-11, self.state, self.watchdog, 0)
        if line.startswith("$W,"):
            self.word = max(0, min(self.word_max, int(line.split(",")[1])))
            self.ext = 1
            self.state = "EXTERNAL_CONTROL"
            return "$W,OK,%d" % self.word
        if line == "$E,1":
            self.ext = 1
            self.state = "EXTERNAL_CONTROL"
            return "$E,OK,1"
        if line == "$E,0":
            self.ext = 0
            self.state = "FINETIME"
            return "$E,OK,0"
        if line.startswith("$T,"):
            self.watchdog = int(line.split(",")[1])
            return "$T,OK,%d" % self.watchdog
        return "$,ERR,UNKNOWN"


def make(**kw):
    fake = FakeConsole(**{k: kw.pop(k) for k in list(kw)
                          if k in ("start_word", "word_max")})
    act = GnssdoActuator(_link=fake, **kw)
    return act, fake


def test_setup_takes_control_and_anchors_at_live_word():
    act, fake = make(start_word=489533)
    act.setup()
    assert fake.opened
    # anchor defaults to the word read at setup
    assert act._center_word == 489533
    assert act.read_frequency_ppb() == pytest.approx(0.0)
    # took external control and set a watchdog
    assert "$E,1" in fake.sent
    assert any(s.startswith("$T,") for s in fake.sent)
    assert fake.ext == 1


def test_adjust_sign_and_resolution():
    act, fake = make(start_word=500000)
    act.setup()
    # +1 LSB is +ppb_per_code; default STP3593LF sign is positive
    applied = act.adjust_frequency_ppb(_STP3593LF_PPB_PER_CODE)
    assert fake.word == 500001
    assert applied == pytest.approx(_STP3593LF_PPB_PER_CODE)
    # a round number of ppb maps to the nearest word
    act.adjust_frequency_ppb(1.0)   # 1 ppb / 8e-4 = 1250 codes
    assert fake.word == 500000 + 1250


def test_adjust_returns_readback_word():
    act, fake = make(start_word=400000)
    act.setup()
    act.adjust_frequency_ppb(-0.08)   # -0.08 ppb / 8e-4 = -100 codes
    assert fake.word == 399900
    assert act.current_word == 399900
    assert act.read_frequency_ppb() == pytest.approx(-0.08)


def test_clamp_at_word_max_warns_once(caplog):
    act, fake = make(start_word=1_048_000)   # near the 2^20-1 ceiling
    act.setup()
    # request way past the top; should saturate at word_max
    with caplog.at_level("WARNING"):
        act.adjust_frequency_ppb(1000.0)     # huge
    assert fake.word == (1 << 20) - 1
    warns = [r for r in caplog.records if "saturating" in r.message]
    assert len(warns) == 1
    # a second saturating command does not re-warn
    act.adjust_frequency_ppb(1000.0)
    warns = [r for r in caplog.records if "saturating" in r.message]
    assert len(warns) == 1


def test_teardown_hands_back():
    act, fake = make()
    act.setup()
    act.teardown()
    assert "$E,0" in fake.sent
    assert fake.ext == 0
    assert not fake.opened


def test_max_adj_ppb_is_smaller_reachable_side():
    # anchor near the top → up-side is small
    act, fake = make(start_word=1_000_000)
    act.setup()
    up = ((1 << 20) - 1 - 1_000_000) * _STP3593LF_PPB_PER_CODE
    assert act.max_adj_ppb == pytest.approx(up)


def test_explicit_center_word_reports_offset_at_start():
    act, fake = make(start_word=500000, center_word=499000)
    act.setup()
    # started 1000 codes above the (explicit) anchor
    assert act.read_frequency_ppb() == pytest.approx(1000 * _STP3593LF_PPB_PER_CODE)


def test_zero_ppb_per_code_rejected():
    with pytest.raises(ValueError):
        GnssdoActuator(_link=FakeConsole(), ppb_per_code=0)
