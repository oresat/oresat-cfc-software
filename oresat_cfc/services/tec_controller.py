"""
The TEC (termalelectric cooler) controller service.

Seperate from the camera service as the camera can be used regaurdless if the TEC is enabled or
not.
"""

import logging
from threading import Event, Thread
from time import monotonic

from oresat_libcanopend import NodeClient
from simple_pid import PID

from ..drivers.pirt1280 import Pirt1280
from ..drivers.rc625 import Rc625
from ..gen.od import CfcEntry


class TecControllerService:
    """
    Used to control and monitoring the TEC (thermalelectic cooler).

    Uses a PID (Proportional–Integral–Derivative) controller for the TEC.
    """

    def __init__(self, node: NodeClient, pirt1280: Pirt1280, rc6_25: Rc625):
        super().__init__()

        self._node = node
        self._camera = pirt1280
        self._tec = rc6_25
        self._tec.disable()  # make sure this is disabled by default

        self._node.request_ownership

        self._node.request_ownership(CfcEntry.TEC_STATUS, None, self.enable)
        self._node.request_ownership(
            CfcEntry.TEC_PID_SETPOINT, self._get_pid_setpoint, self._set_pid_setpoint
        )
        self._node.request_ownership(CfcEntry.TEC_PID_KP, self._get_pid_kp, self._set_pid_kp)
        self._node.request_ownership(CfcEntry.TEC_PID_KI, self._get_pid_ki, self._set_pid_ki)
        self._node.request_ownership(CfcEntry.TEC_PID_KD, self._get_pid_kd, self._set_pid_kd)

        self._pid = PID(
            Kp=self._node.od_read(CfcEntry.TEC_PID_KP),
            Ki=self._node.od_read(CfcEntry.TEC_PID_KI),
            Kd=self._node.od_read(CfcEntry.TEC_PID_KD),
        )
        self._pid.setpoint = self._node.od_read(CfcEntry.TEC_PID_SETPOINT)

        self._controller_enabled = False
        self._saturated = False
        self._samples = []
        self._lowest_temp = 100

        self._thread = Thread(target=self._thread_run, daemon=True)
        self._event = Event()

    def _get_moving_average(self, temp: float) -> float:
        """
        Calculate the moving average of the temperature, using the newly provided temperature
        sample.
        """

        # pop the oldest sample if we have the max number of samples
        if len(self._samples) >= 4:
            self._samples.pop(0)

        # add the latest sample to the list
        self._samples.append(temp)

        # return the average
        return sum(self._samples) / len(self._samples)

    def run(self, thread: bool):
        if thread:
            self._thread.start()
        else:
            self._thread_run()

    def _thread_run(self):
        while True:
            ts = monotonic()
            self._loop()
            delay_ms = self._node.od_read(CfcEntry.TEC_PID_DELAY)
            self._event.wait((monotonic() - ts) - (delay_ms / 1000))

    def _loop(self):
        current_temp = self._camera.temperature
        diff = self._pid(current_temp)
        mv_avg = self._get_moving_average(current_temp)

        sat_diff = self._node.od_read(CfcEntry.TEC_SATURATION_DIFF)
        cooldown_temp_c = self._node.od_read(CfcEntry.TEC_COOLDOWN_TEMPERATURE)

        # update the lowest temperature
        self._lowest_temp = min(self._lowest_temp, current_temp)

        if not self._camera.is_enabled or not self._controller_enabled:
            self._tec.disable()
        elif current_temp >= cooldown_temp_c:
            logging.info(
                "current temperature is above cooldown temperature, disabling TEC controller"
            )
            self._controller_enabled = False
        else:
            saturation_pt = self._lowest_temp + sat_diff

            # if the average goes below the saturation point since enabled, flag it
            if mv_avg <= saturation_pt and not self._past_saturation_pt_since_enable:
                logging.info("TEC has past saturation point toward target temperature")
                self._past_saturation_pt_since_enable = True

            # if the average goes above the saturation point, after going below it,
            # since enabled, then the TEC is probably saturated so disable it
            if mv_avg > saturation_pt and self._past_saturation_pt_since_enable:
                logging.info("TEC is saturated")
                self._controller_enabled = False
                self._saturated = True
                # handles case shere user moves the setpoint around a lot
                self._past_saturation_pt_since_enable = True

            # drive the TEC power based on the PID output
            if not self._saturated and diff < 0:
                self._tec.enable()
            else:
                self._tec.disable()

        self._node.od_write_multi(
            {
                CfcEntry.TEC_STATUS: self._controller_enabled,
                CfcEntry.TEC_SATURATED: self._saturated,
            }
        )

    def enable(self, value: bool):
        if value and not self._controller_enabled:
            # reset these on an enable, if currently disabled
            logging.info("enabling TEC controller")
            self._past_saturation_pt_since_enable = False
            self._saturated = False
            self._lowest_temp = 100
        elif not value and self._controller_enabled:
            logging.info("disabling TEC controller")
        self._controller_enabled = value

    def _get_pid_setpoint(self) -> int:
        return self._pid.setpoint

    def _get_pid_kp(self) -> float:
        return self._pid.Kp

    def _get_pid_ki(self) -> float:
        return self._pid.Ki

    def _get_pid_kd(self) -> float:
        return self._pid.Kd

    def _set_pid_setpoint(self, setpoint: int):
        self._pid.setpoint = setpoint

    def _set_pid_kp(self, kp: float):
        self._pid.Kp = kp

    def _set_pid_ki(self, ki: float):
        self._pid.Ki = ki

    def _set_pid_kd(self, kd: float):
        self._pid.Kd = kd
