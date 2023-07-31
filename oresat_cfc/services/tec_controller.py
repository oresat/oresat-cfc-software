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
        self._tec.disable()  # make sure this is disable by default

        self._pid = None

        self._past_saturation_pt_since_enable = False
        self._samples = []
        self._lowest_temp = 100  # something high by default
        self._controller_enable = False

        self.tec_index = 0x6002

    def on_start(self):

        self.node.add_sdo_read_callback(self.tec_index, self.on_tec_read)
        self.node.add_sdo_write_callback(self.tec_index, self.on_tec_write)

        tec_controller_rec = self.node.od[0x6002]
        self._saturated_obj = tec_controller_rec[0x2]
        self._saturated_obj.value = False  # make sure this is False by default
        self._saturation_diff_obj = tec_controller_rec[0x5]
        self._pid_delay_ms_obj = tec_controller_rec[0x9]
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

        self.sleep(self._pid_delay_ms_obj.value / 1000)

        # only run tec controller alg when the camera and TEC controller are both enabled
        if not self._camera.is_enabled or not self._controller_enable:
            self._tec.disable()
            return

        current_temp = self._camera.temperature
        diff = self._pid(current_temp)
        mv_avg = self._get_moving_average(current_temp)

        # update the lowest temperature
        if current_temp < self._lowest_temp:
            self._lowest_temp = current_temp

        logger.debug(f'target: {self._pid.setpoint} / current: {current_temp} / '
                     f'lowest: {self._lowest_temp} / mv avg: {mv_avg} / PID diff: {diff}')

        # don't even try to control the TEC if above the cooldown temperature
        if current_temp >= self._cooldown_temp_obj.value:
            logger.info('current temperature is above cooldown temperature, disabling TEC '
                        'controller')
            self._controller_enable = False
            return

        saturation_pt = self._lowest_temp + self._saturation_diff_obj.value

        # if the average goes below the saturation point since enabled, flag it
        if mv_avg <= saturation_pt and not self._past_saturation_pt_since_enable:
            logger.info('TEC has past saturation point toward target temperature')
            self._past_saturation_pt_since_enable = True

        # if the average goes above the saturation point, after going below it,
        # since enabled, then the TEC is probably saturated so disable it
        if mv_avg > saturation_pt and self._past_saturation_pt_since_enable:
            logger.info('TEC is saturated')
            self._saturated_obj.value = True
            # handles case shere user moves the setpoint around a lot
            self._past_saturation_pt_since_enable = True

        # drive the TEC power based on the PID output
        if not self._saturated_obj.value and diff < 0:
            self._tec.enable()
        else:
            self._tec.disable()

    def on_loop_error(self, error: Exception):

        logger.critical('disabling TEC due to unexpected error with TEC controller loop')
        logger.exception(error)

        try:
            self._tec.disable()
        except Exception:
            logger.critical('failed to disable the TEC')

        self._controller_enable = False

    def on_tec_read(self, index: int, subindex: int):
        '''SDO read on the camera obj'''

        ret = None

        if index == self.tec_index:
            return ret

        if subindex == 0x1:
            ret = self._controller_enable
        elif subindex == 0x3:
            ret = self._pid.setpoint

        return ret

    def on_tec_write(self, index: int, subindex: int, value):
        '''SDO read on the camera obj'''

        if index != self.tec_index:
            return

        if subindex == 0x1:
            if value and not self._controller_enable:
                # reset these on an enable, if currently disabled
                logger.info('enabling TEC controller')
                self._past_saturation_pt_since_enable = False
                self._saturated_obj.value = False
                self._lowest_temp = 100
            elif not value and self._controller_enable:
                logger.info('disabling TEC controller')
            self._controller_enable = value
        elif subindex == 0x3:
            self._pid.setpoint = value
