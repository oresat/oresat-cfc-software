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
    '''
    Service for controlling and monitoring the TEC (thermalelectic cooler).

    Uses a PID (Proportional–Integral–Derivative) controller for the TEC.
    '''

    def __init__(self, pirt1280: Pirt1280, rc6_25: Rc625):
        super().__init__()

        self._camera = pirt1280
        self._tec = rc6_25
        self._tec_ctrl_enable = False
        self._tec.disable()  # make sure this is disable by default

        self._pid = None

        self._past_saturation_pt_since_enable = False
        self._samples = []
        self._lowest_temp = 100  # something high by default

        self.tec_index = 0x6002

    def on_start(self):

        self.node.add_sdo_read_callback(self.tec_index, self.on_tec_read)
        self.node.add_sdo_write_callback(self.tec_index, self.on_tec_write)

        tec_controller_rec = self.node.od[0x6002]
        self._controller_enable_obj = tec_controller_rec[0x1]
        self._controller_enable_obj.value = False  # make sure this is disable by default
        self._saturated_obj = tec_controller_rec[0x2]
        self._saturated_obj.value = False  # make sure this is False by default
        self._saturation_diff_obj = tec_controller_rec[0x5]
        self._pid_delay_obj = tec_controller_rec[0x9]
        self._cooldown_temp_obj = tec_controller_rec[0xA]
        self._mv_avg_samples_obj = tec_controller_rec[0xB]

        self._pid = PID(
            tec_controller_rec[0x6].value,
            tec_controller_rec[0x7].value,
            tec_controller_rec[0x8].value
        )
        self._pid.setpoint = tec_controller_rec[0x3].value

    def on_stop(self):

        self._tec.disable()

    def _get_moving_average(self, temp: float) -> float:
        '''
        Calculate the moving average of the temperature, using the newly provided temperature
        sample.
        '''

        # pop the oldest sample if we have the max number of samples
        if len(self._samples) >= self._mv_avg_samples_obj.value:
            self._samples.pop(0)

        # add the latest sample to the list
        self._samples.append(temp)

        # return the average
        return sum(self._samples) / len(self._samples)

    def on_loop(self) -> bool:

        self.sleep(self._pid_delay_obj.value / 1000)

        if not self._camera.is_enabled or not self._controller_enable_obj.value:
            self._tec.disable()
            return

        current_temp = self._camera.temperature
        diff = self._pid(current_temp)
        mv_avg = self._get_moving_average(current_temp)

        if current_temp < self._lowest_temp:
            self._lowest_temp = current_temp

        logger.debug(f'target: {self._pid.setpoint} / current: {current_temp} / '
                     f'lowest: {self._lowest_temp} / mv avg: {mv_avg} / PID diff: {diff}')

        if current_temp >= self._cooldown_temp_obj.value:
            self._tec.disable()
            return  # don't even try to control the TEC

        saturation_pt = self._lowest_temp + self._saturation_diff_obj.value

        # if the average goes below the saturation point since enabled, flag it
        if mv_avg <= saturation_pt and not self._past_saturation_pt_since_enable:
            logger.info('TEC has reached target temperature')
            self._past_saturation_pt_since_enable = True

        # if the average goes above the saturation point, after going below it,
        # since enabled, then the TEC is probably saturated so disable it
        if mv_avg > saturation_pt and self._past_saturation_pt_since_enable:
            logger.info('TEC saturated')
            self._saturated_obj.value = True

        # drive the TEC power based on the PID output
        if not self._saturated_obj.value and diff < 0:
            self._tec.enable()
        else:
            self._tec.disable()

    def on_loop_error(self, error: Exception):

        logger.critical('disabling TEC due to unexpected error with tec loop')
        logger.error(error)

        try:
            self._tec.disable()
        except Exception:
            logger.critical('failed to disable tec')

    def on_tec_read(self, index: int, subindex: int):
        '''SDO read on the camera obj'''

        if index == self.tec_index and subindex == 0x3:
            return self._pid.setpoint

    def on_tec_write(self, index: int, subindex: int, value):
        '''SDO read on the camera obj'''

        if index != self.tec_index:
            return

        if subindex == 0x1:
            if value and self._controller_enable_obj.value is False:
                # reset these on an enable, if currently disabled
                self._past_saturation_pt_since_enable = False
                self._saturated_obj.value = False
                self._lowest_temp = 100

            self._controller_enable_obj.value = value
            if value:
                logger.info('enabling TEC controller')
            else:
                logger.info('disabling TEC controller')
                self._tec.disable()
        elif subindex == 0x3:
            self._pid.setpoint = value
