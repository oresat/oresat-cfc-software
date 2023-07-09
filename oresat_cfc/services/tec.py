from simple_pid import PID
from olaf import Service, logger

from ..drivers.pirt1280 import Pirt1280
from ..drivers.tec import Tec


class TecService(Service):
    '''Service for controlling and monitoring the TEC (thermalelectic cooler).'''

    SATURATION_DIFF = 6
    '''
    the difference between the current temp and the setpoint at which the TEC is
    considered saturated and therefore should be disabled. Note that it would
    only be considered saturated after it first crossed below the saturation
    and then came above it as the TEC because saturated with heat.
    '''

    def __init__(self, pirt1280: Pirt1280, tec: Tec):
        super().__init__()

        self._pirt1280 = pirt1280
        self._tec = tec
        self._tec.disable()

        self._pid = PID(0.5, 0.0, 0.1)
        self._pid.setpoint = 10  # aka the target temp

        # flags whether the TEC temperature is saturated. This is defined by the
        # TEC first going below zero after being enabled, and then going above zero
        # again when the TEC cannot maintain the temperature differential
        self._saturated = False

        self._past_saturation_pt_since_enable = False
        self._samples = []
        self._num_samples = 20

        self.tec_index = 0x6002

    def on_start(self):

        self.node.add_sdo_read_callback(self.tec_index, self.on_tec_read)
        self.node.add_sdo_write_callback(self.tec_index, self.on_tec_write)

    def on_stop(self):

        self._tec.disable()

    def _get_moving_average(self, temp: float) -> float:
        '''
        Calculate the moving average of the temperature, using the newly provided temperature
        sample.
        '''

        # pop the oldest sample if we have the max number of samples
        if len(self._samples) >= self._num_samples:
            self._samples.pop(0)

        # add the latest sample to the list
        self._samples.append(temp)

        # return the average
        return sum(self._samples) / len(self._samples)

    def on_loop(self) -> bool:

        # sample the temp and get the PID correction
        current_temp = self._pirt1280.temperature
        diff = self._pid(current_temp)

        # get the moving average
        avg = self._get_moving_average(current_temp)

        # calculate the saturation point based on the setpoint
        saturation_pt = self._pid.setpoint + self.SATURATION_DIFF

        # if the average goes below the saturation point since enabled, flag it
        if avg <= saturation_pt:
            self._past_saturation_pt_since_enable = True

        # if the average goes above the saturation point, after going below it,
        # since enabled, then the TEC is probably saturated so disable it
        if avg > saturation_pt and self._past_saturation_pt_since_enable:
            self._saturated = True

        # drive the TEC power based on the PID output
        if not self._saturated and diff >= 0:
            self._tec.enable()
        else:
            self._tec.disable()

        self.sleep(0.5)

    def on_loop_error(self, error: Exception):
        '''on_loop() raised an unexpected error'''

        logger.critical('disabling TEC due to unexpected error with tec loop')
        logger.error(error)

        try:
            self._tec.disable()
        except Exception:
            logger.critical('failed to disable tec')

    def on_tec_read(self, index: int, subindex: int):
        '''SDO read on the camera obj'''

        if index != self.tec_index:
            return

        ret = None

        if subindex == 0x1:
            ret = self._tec.is_enabled
        elif subindex == 0x2:
            ret = self._saturated
        elif subindex == 0x3:
            ret = self._pid.setpoint

        return ret

    def on_tec_write(self, index: int, subindex: int, value):
        '''SDO read on the camera obj'''

        if index != self.tec_index:
            return

        if subindex == 0x1:
            if value:
                self._tec.enable()
            else:
                self._tec.disable()
        elif subindex == 0x3:
            self._pid.setpoint = value
