"""The RC6-2.5 thermoelectric cooler (TEC) driver"""

import gpiod
from gpiod.line import Direction, Value


class Rc625:
    """RC6-2.5 thermoelectric cooler (TEC)"""

    def __init__(self, gpio: tuple[int], mock: bool = False):
        self._num = gpio[1]
        self._mock = mock
        if not mock:
            self.gpio_lines = gpiod.request_lines(
                f"/dev/gpiochip{gpio[0]}",
                consumer="oresat-cfc",
                config={
                    self._num: gpiod.LineSettings(
                        direction=Direction.OUTPUT,
                        output_value=Value.INACTIVE,
                    ),
                },
            )

    def enable(self):
        """Enable the TEC."""
        if not self._mock:
            self.gpio_lines.set_value(self._num, Value.ACTIVE)
        self._mock_value = True

    def disable(self):
        """Disable the TEC."""
        if not self._mock:
            self.gpio_lines.set_value(self._num, Value.INACTIVE)
        self._mock_value = False

    @property
    def is_enabled(self) -> bool:
        """bool: Is the TEC enabled"""
        if not self._mock:
            return self.gpio_lines.get_value(self._num) == Value.ACTIVE
        return self._mock_value
