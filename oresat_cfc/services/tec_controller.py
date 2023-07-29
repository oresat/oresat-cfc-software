'''
The TEC (termalelectric cooler) controller service.

Seperate from the camera service as the camera can be used regaurdless if the TEC is enabled or
not.
'''

from simple_pid import PID
from olaf import Service, logger

from ..drivers.pirt1280 import Pirt1280
from ..drivers.rc625 import Rc625


class TecControllerService(Service):
    '''Service for controlling and monitoring the TEC (thermalelectic cooler).'''

    def __init__(self, pirt1280: Pirt1280, rc6_25: Rc625):
        super().__init__()

        self._camera = pirt1280
        self._tec = rc6_25
        self._tec_ctrl_enable = False
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

        self._saturation_diff_obj = None

        self.tec_index = 0x6002

    def on_start(self):

        self.node.add_sdo_read_callback(self.tec_index, self.on_tec_read)
        self.node.add_sdo_write_callback(self.tec_index, self.on_tec_write)
        self._saturation_diff_obj = self.node.od[0x6002][0x5]

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

        if self._camera.is_enabled and self._tec_ctrl_enable:

            # sample the temp and get the PID correction
            current_temp = self._camera.temperature
            diff = self._pid(current_temp)

            # get the moving average
            avg = self._get_moving_average(current_temp)

            # calculate the saturation point based on the setpoint
            saturation_pt = self._pid.setpoint + self._saturation_diff_obj.value

            # if the average goes below the saturation point since enabled, flag it
            if avg <= saturation_pt:
                self._past_saturation_pt_since_enable = True

            # if the average goes above the saturation point, after going below it,
            # since enabled, then the TEC is probably saturated so disable it
            if avg > saturation_pt and self._past_saturation_pt_since_enable:
                logger.info('TEC saturated')
                self._saturated = True

            # drive the TEC power based on the PID output
            if not self._saturated and diff < 0:
                self._tec.enable()
            else:
                self._tec.disable()
        else:
            self._tec.disable()

        self.sleep(0.5)

    def on_loop_error(self, error: Exception):

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
            ret = self._tec_ctrl_enable
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
            self._tec_ctrl_enable = value
            if value:
                logger.info('enabling TEC controller')
            else:
                logger.info('disabling TEC controller')
                self._tec.disable()
        elif subindex == 0x3:
            self._pid.setpoint = value
