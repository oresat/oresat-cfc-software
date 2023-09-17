"""The RC6-2.5 thermoelectric cooler (TEC) driver"""

from olaf import Gpio


class Rc625:
    """RC6-2.5 thermoelectric cooler (TEC)"""

    def __init__(self, gpio_num: int, mock: bool = False):
        self._gpio = Gpio(gpio_num, mock=mock)

    def enable(self):
        """Enable the TEC."""
        self._gpio.high()

    def disable(self):
        """Disable the TEC."""
        self._gpio.low()

    @property
    def is_enabled(self) -> bool:
        """bool: Is the TEC enabled"""

        return self._gpio.is_high
