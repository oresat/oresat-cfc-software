from datetime import datetime

from olaf import logger
from simple_pid import PID

from .drivers.tec import Tec


class TecController:

    SATURATION_DIFF = 6
    '''
    the difference between the current temp and the setpoint at which the TEC is
    considered saturated and therefore should be disabled. Note that it would
    only be considered saturated after it first crossed below the saturation
    and then came above it as the TEC because saturated with heat.
    '''

    def __init__(self, tec: Tec):

        self._tec = tec

        self.pid = PID(0.5, 0.0, 0.1)
        self.pid.setpoint = 10

        # flags whether the TEC temperature is saturated. This is defined by the
        # TEC first going below zero after being enabled, and then going above zero
        # again when the TEC cannot maintain the temperature differential
        self._saturated = False

        self._saturation_time = None
        self._past_saturation_pt_since_enable = False
        self._samples = []
        self._num_samples = 20

    def __del__(self):

        self.disable_controller()

    def _get_moving_average(self, temp):
        '''
        Calculate the moving average of the TEC temperature, using the newly provided temperature
        sample.
        '''

        # pop the oldest sample if we have the max number of samples
        if len(self._samples) >= self._num_samples:
            self._samples.pop(0)

        # add the latest sample to the list
        self._samples.append(temp)

        # return the average
        return sum(self._samples) / len(self._samples)

    def loop(self):

        # sample the temp and get the PID correction
        current_temp = self._tec.temperature
        diff = self.pid(current_temp)

        # get the moving average
        avg = self._get_moving_average(current_temp)

        # calculate the saturation point based on the setpoint
        saturation_pt = self.pid.setpoint + self.SATURATION_DIFF

        # if the average goes below the saturation point since enabled, flag it
        if avg <= saturation_pt:
            self._past_saturation_pt_since_enable = True

        # if the average goes above the saturation point, after going below it,
        # since enabled, then the TEC is probably saturated so disable it
        if avg > saturation_pt and self._past_saturation_pt_since_enable:
            logger.warning('TEC is saturated, disabling TEC')
            self._saturated = True
            self._saturation_time = datetime.utcnow()

        # drive the TEC power based on the PID output
        if diff < 0:
            logger.debug('enabling TEC')
            self._tec.enable()
        else:
            logger.debug('disabling TEC')
            self._tec.disable()

    @property
    def is_saturated(self):
        '''bool: Check if the TEC is saturated.'''

        return self._saturated
