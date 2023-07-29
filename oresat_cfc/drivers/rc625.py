from olaf import Gpio


class Rc625:
    '''RC6-2.5 thermalelctric cooler'''

    def __init__(self, gpio_num: int, mock: bool = False):

        self._gpio = Gpio(gpio_num, mock=mock)

    def enable(self):

        self._gpio.high()

    def disable(self):

        self._gpio.low()

    @property
    def is_enabled(self) -> bool:
        '''bool: Is the TEC enabled'''

        return self._gpio.is_high
