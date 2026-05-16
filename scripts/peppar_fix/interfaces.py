"""Abstract interfaces for phase measurement and frequency actuation.

These ABCs decouple the servo loop from specific hardware.  Current
implementations of ``FrequencyActuator``:

  - ``DacActuator`` (AD5693R/MCP4725 → VCOCXO varactor) — see
    ``dac_actuator.py``.  Used by PiFace + clkPoC3.
  - ``PhcAdjfineActuator`` (Linux PHC ``clock_adjtime``) — see
    ``phc_actuator.py``.  Used by TimeHat + MadHat (i226 NIC TCXO).
  - ``ClockmatrixActuator`` (Renesas 8A34002 I²C DPLL FCW) — see
    ``clockmatrix_actuator.py``.  Used by Timebeat OTC hosts
    (otcBob1, ptBoat).

Future: White Rabbit, other timing chips.  Adding a new DO architecture
means implementing this interface; servo, characterization
(``tools/calibrate_do.py``), and engine startup discovery work
unchanged.
"""

from abc import ABC, abstractmethod


class FrequencyActuator(ABC):
    """Adjusts the output frequency of a clock source."""

    @abstractmethod
    def setup(self) -> None:
        """Prepare hardware for frequency control (e.g., DPLL mode switch)."""

    @abstractmethod
    def teardown(self) -> None:
        """Restore hardware to default state."""

    @abstractmethod
    def adjust_frequency_ppb(self, ppb: float) -> float:
        """Apply absolute frequency offset in ppb. Returns actual applied ppb."""

    @abstractmethod
    def read_frequency_ppb(self) -> float:
        """Read current frequency offset in ppb."""

    @property
    @abstractmethod
    def max_adj_ppb(self) -> float:
        """Maximum frequency adjustment magnitude."""

    @property
    @abstractmethod
    def resolution_ppb(self) -> float:
        """Smallest distinguishable frequency step in ppb."""


class PhaseSource(ABC):
    """Reads phase error between a reference and a steered clock."""

    @abstractmethod
    def setup(self) -> None:
        """Prepare hardware for phase measurement."""

    @abstractmethod
    def teardown(self) -> None:
        """Release hardware resources."""

    @abstractmethod
    def read_phase_ns(self) -> float | None:
        """Read current phase error in nanoseconds.

        Returns None if measurement is unavailable.
        Sign: positive = steered clock is late (needs to speed up).
        """

    @property
    @abstractmethod
    def resolution_ns(self) -> float:
        """Nominal single-shot resolution in nanoseconds."""
