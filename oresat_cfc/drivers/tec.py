import math as m
import random

import gpio


def res_to_temp_celcius(res: int) -> float:
    '''
    Per the steinhart/hart equation where A-D are the steinhart coefficients,
    R25 is the NTC resistance at 25C(10k), and RT is the NTC resistance
    T=1/(A1+B1*LN(RT/R25)+C1*LN(RT/R25)^2+D1*LN(RT/R25)^3)-273.15
    '''

    r25c = 10000  # resistance of the NTC at 25C

    # steinhart coefficients per Vishay website for part NTCS0805E3103FMT
    # NOTE: don't take the equations off the site it self, export/download the
    # data for the specific part as there are more specifics there.
    # https://www.vishay.com/thermistors/ntc-rt-calculator/
    sh_a = 0.003354016434680530000
    sh_b = 0.000286451700000000000
    sh_c = 0.000003252255000000000
    sh_d = 0.000000045945010000000

    tmp = m.log(res / r25c)
    temp_k = 1 / (sh_a + (sh_b * tmp) + (sh_c * tmp ** 2) + (sh_d * m.pow(res / r25c, 3)))
    temp_c = temp_k - 273.15

    return temp_c


class TEC:

    R1 = 10000
    '''resistance of the upper resistor in the divider'''

    ADC_STEPS = 4096  # TODO should this be 4095?
    V_IN = 1.8

    GPIO_PIN = 88

    ADC_PATH = '/sys/bus/iio/devices/iio:device0/in_voltage0_raw'

    def __init__(self, mock: bool = False):

        self.mock = mock

        self._tec_on = False  # is the TEC is currently on/off

        # setup GPIO and set to default state
        if not self.mock:
            gpio.setup(self.GPIO_PIN, gpio.OUT)
        self.disable()

    def disable(self):
        '''Disable the TEC'''

        if not self.mock:
            gpio.set(self.GPIO_PIN, 0)
        self._tec_on = False

    def enable(self):
        '''Enable the TEC'''

        if not self.mock:
            gpio.set(self.GPIO_PIN, 1)
        self._tec_on = True

    def _get_temp(self) -> float:
        '''Get the raw temperature of the TEC'''

        if self.mock:
            return random.uniform(-10, 0)

        with open(self.ADC_PATH, 'r') as f:
            val = f.read()
            val = int(val)
            # convert the adc value 0 - 4096 to volts
            vout = (val / self.ADC_STEPS) * self.V_IN

            # The 10k NTC is part of a voltage divider with a 10k resistor between
            # 1.8v and Vout and the NTC between Vout and ground. Thus, the equation is:
            #
            # Vout = 1.8v * (NTC / (10k + NTC))
            #
            # We know volts and need to solve for NTC resistance, which is:
            #
            # NTC = (10k * Vout) / (1.8v - Vout)

            res = (self.R1 * vout) / (self.V_IN - vout)

            temp = res_to_temp_celcius(res)

            return temp

    @property
    def temperature(self) -> float:
        '''float: The temperature of the TEC'''

        samples = 10
        samples_sum = 0

        for i in range(samples):
            samples_sum += self._get_temp()

        return samples_sum / samples
