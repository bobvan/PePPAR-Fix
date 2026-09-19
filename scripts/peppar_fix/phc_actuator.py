"""PHC adjfine frequency actuator — wraps PtpDevice.adjfine()."""

from peppar_fix.interfaces import FrequencyActuator


class PhcAdjfineActuator(FrequencyActuator):
    """Frequency steering via Linux PHC clock_adjtime(ADJ_FREQUENCY).

    This is the default actuator for i226 and E810 platforms.
    Resolution ~1 ppb, range **±62,500 ppm** (i226; `max_adj = 62499999`
    and that field is ppb — measured 2026-09-19).  E810 was recorded as
    "±250 ppm"; that figure is UNVERIFIED and likely the same 1000x
    ppb/ppm misreading, but our E810 host is decommissioned so it has
    not been re-measured.
    """

    def __init__(self, ptp_device):
        self._ptp = ptp_device
        self._max_ppb = 62_500_000.0  # updated in setup()

    def setup(self) -> None:
        caps = self._ptp.get_caps()
        self._max_ppb = float(caps.get('max_adj', 62_500_000))

    def teardown(self) -> None:
        pass  # Don't zero frequency — bootstrap preserves drift file

    def adjust_frequency_ppb(self, ppb: float) -> float:
        return self._ptp.adjfine(ppb)

    def read_frequency_ppb(self) -> float:
        return self._ptp.read_adjfine()

    @property
    def max_adj_ppb(self) -> float:
        return self._max_ppb

    @property
    def resolution_ppb(self) -> float:
        return 1.0 / 65.536  # adjfine internal scaling: 1 count ≈ 0.015 ppb
