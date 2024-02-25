"""
The TEC (termalelectric cooler) controller service.

Seperate from the camera service as the camera can be used regaurdless if the TEC is enabled or
not.
"""

import time

import canopen
import matplotlib
import matplotlib.pyplot as plt
from olaf import Service, logger
from simple_pid import PID

from ..drivers.pirt1280 import Pirt1280
from ..drivers.rc625 import Rc625

matplotlib.use("Agg")


class TecControllerService(Service):
    """
    Service for controlling and monitoring the TEC (thermalelectic cooler).

    Uses a PID (Proportional–Integral–Derivative) controller for the TEC.
    """

    def __init__(self, pirt1280: Pirt1280, rc6_25: Rc625):
        super().__init__()

        self._camera = pirt1280
        self._tec = rc6_25
        self._tec.disable()  # make sure this is disabled by default

        self._pid: PID = None

        self._past_saturation_pt_since_enable = False
        self._samples: list = []
        self._lowest_temp = 100  # something high by default
        self._controller_enabled = False

        self._saturated_obj: canopen.objectdictionary.Variable = None
        self._saturation_diff_obj: canopen.objectdictionary.Variable = None
        self._pid_delay_obj: canopen.objectdictionary.Variable = None
        self._cooldown_temp_obj: canopen.objectdictionary.Variable = None
        self._mv_avg_samples_obj: canopen.objectdictionary.Variable = None

        self._start_time: int = 0
        self._target_temperatures: list = []
        self._current_temperatures: list = []
        self._pid_outputs: list = []
        self._cooldown_temps: list = []
        self._graph_unix_times: list = []
        self._maximum_graph_size: int = 10_000
        self._graph_filename_save: str = "temperature_graph"

    def on_start(self):
        self.node.add_sdo_callbacks("tec", "status", self._on_read_status, self._on_write_status)
        self.node.add_sdo_callbacks(
            "tec",
            "pid_setpoint",
            self._on_read_pid_setpoint,
            self._on_write_pid_setpoint,
        )
        self.node.add_sdo_callbacks("tec", "pid_kp", None, self._on_write_pid_p)
        self.node.add_sdo_callbacks("tec", "pid_ki", None, self._on_write_pid_i)
        self.node.add_sdo_callbacks("tec", "pid_kd", None, self._on_write_pid_d)
        self.node.add_sdo_callbacks("tec", "pid_graph", self._on_read_pid_graph, None)

        rec = self.node.od["tec"]
        self._saturated_obj = rec["saturated"]
        self._saturated_obj.value = False  # make sure this is False by default
        self._saturation_diff_obj = rec["saturation_diff"]
        self._pid_delay_obj = rec["pid_delay"]
        self._cooldown_temp_obj = rec["cooldown_temperature"]
        self._mv_avg_samples_obj = rec["moving_avg_samples"]

        self._pid = PID(
            Kp=rec["pid_kp"].value,
            Ki=rec["pid_ki"].value,
            Kd=rec["pid_kd"].value,
        )
        self._pid.setpoint = rec["pid_setpoint"].value

    def on_stop(self):
        self._tec.disable()

    def _get_moving_average(self, temp: float) -> float:
        """
        Calculate the moving average of the temperature, using the newly provided temperature
        sample.
        """

        # pop the oldest sample if we have the max number of samples
        if len(self._samples) >= self._mv_avg_samples_obj.value:
            self._samples.pop(0)

        # add the latest sample to the list
        self._samples.append(temp)

        # return the average
        return sum(self._samples) / len(self._samples)

    def _update_graph_data(self, current_temp: float, diff: float, mv_avg: float):
        """Update the local data for the graph."""

        self._current_temperatures.append(current_temp)
        self._pid_outputs.append(0 if diff < 0 else (100 if diff > 100 else diff))
        self._target_temperatures.append(self._pid.setpoint)
        self._cooldown_temps.append(self._cooldown_temp_obj.value)
        self._graph_unix_times.append((time.time_ns() // 1000000) - self._start_time)
        if len(self._current_temperatures) > self._maximum_graph_size:
            self._current_temperatures.pop(0)
        if len(self._pid_outputs) > self._maximum_graph_size:
            self._pid_outputs.pop(0)
        if len(self._target_temperatures) > self._maximum_graph_size:
            self._target_temperatures.pop(0)
        if len(self._cooldown_temps) > self._maximum_graph_size:
            self._cooldown_temps.pop(0)
        if len(self._graph_unix_times) > self._maximum_graph_size:
            self._graph_unix_times.pop(0)

    def on_loop(self):
        self.sleep(self._pid_delay_obj.value / 1000)

        current_temp = self._camera.temperature
        diff = self._pid(current_temp)
        mv_avg = self._get_moving_average(current_temp)

        self._update_graph_data(current_temp, diff, mv_avg)

        # update the lowest temperature
        self._lowest_temp = min(self._lowest_temp, current_temp)

        logger.debug(
            f"target: {self._pid.setpoint} / current: {current_temp} / "
            f"lowest: {self._lowest_temp} / mv avg: {mv_avg} / PID diff: {diff}"
        )

        # only run tec controller alg when the camera and TEC controller are both enabled
        if not self._camera.is_enabled or not self._controller_enabled:
            self._tec.disable()
            return

        # don"t even try to control the TEC if above the cooldown temperature
        if current_temp >= self._cooldown_temp_obj.value:
            logger.info(
                "current temperature is above cooldown temperature, disabling TEC controller"
            )
            self._controller_enabled = False
            return

        saturation_pt = self._lowest_temp + self._saturation_diff_obj.value

        # if the average goes below the saturation point since enabled, flag it
        if mv_avg <= saturation_pt and not self._past_saturation_pt_since_enable:
            logger.info("TEC has past saturation point toward target temperature")
            self._past_saturation_pt_since_enable = True

        # if the average goes above the saturation point, after going below it,
        # since enabled, then the TEC is probably saturated so disable it
        if mv_avg > saturation_pt and self._past_saturation_pt_since_enable:
            logger.info("TEC is saturated")
            self._controller_enabled = False
            self._saturated_obj.value = True
            # handles case shere user moves the setpoint around a lot
            self._past_saturation_pt_since_enable = True

        # drive the TEC power based on the PID output
        if not self._saturated_obj.value and diff < 0:
            self._tec.enable()
        else:
            self._tec.disable()

    def on_loop_error(self, error: Exception):
        logger.critical("disabling TEC due to unexpected error with TEC controller loop")
        logger.exception(error)

        self._tec.disable()
        self._controller_enabled = False

    def _on_read_status(self) -> bool:
        """SDO read callback for TEC status."""

        return self._controller_enabled

    def _on_write_status(self, value: bool):
        """SDO write callback for TEC status."""

        if value and not self._controller_enabled:
            # reset these on an enable, if currently disabled
            logger.info("enabling TEC controller")
            self._past_saturation_pt_since_enable = False
            self._saturated_obj.value = False
            self._lowest_temp = 100

            self._start_time = time.time_ns() // 1000000
            self._target_temperatures.clear()
            self._current_temperatures.clear()
            self._pid_outputs.clear()
            self._cooldown_temps.clear()
        elif not value and self._controller_enabled:
            logger.info("disabling TEC controller")

        self._controller_enabled = value

    def _on_read_pid_setpoint(self) -> int:
        """SDO read callback for TEC"s PID setpoint."""

        return int(self._pid.setpoint)

    def _on_write_pid_setpoint(self, value: int):
        """SDO read callback for TEC"s PID setpoint."""

        self._pid.setpoint = value

    def _on_write_pid_p(self, value: float):
        """SDO write callback for the TEC"s PID Kp."""

        self._pid.Kp = value

    def _on_write_pid_i(self, value: float):
        """SDO write callback for the TEC"s PID Ki."""

        self._pid.Ki = value

    def _on_write_pid_d(self, value: float):
        """SDO write callback for the TEC"s PID Kd."""

        self._pid.Kd = value

    def _on_read_pid_graph(self):
        """SDO read collback for TEC PID Temp over Time graph"""

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

        # Top subplot
        ax1.plot(self._graph_unix_times, self._cooldown_temps, label="Cooldown Temp")
        ax1.plot(self._graph_unix_times, self._target_temperatures, label="Target Temp")
        ax1.plot(self._graph_unix_times, self._current_temperatures, label="Current Temp")
        ax1.set_title("Temperatures")
        ax1.set_xlabel("Time (ms)")
        ax1.set_ylabel("Temperature (C)")
        ax1.legend()

        # Bottom subplot
        ax2.plot(self._graph_unix_times, self._pid_outputs, label="PID Output", color="red")
        ax2.set_title("PID")
        ax2.set_xlabel("Time (ms)")
        ax2.set_ylabel("Amplitude")
        ax2.legend()

        # Adjust spacing between subplots
        plt.tight_layout()
        plt.savefig(f"/tmp/{self._graph_filename_save}.jpg")
        plt.close(fig)
        with open(f"/tmp/{self._graph_filename_save}.jpg", "rb") as image_file:
            image_binary = image_file.read()
        return image_binary
