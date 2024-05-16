"""
Main camera service.

Seperate from the TEC controller service as the camera can be used regaurdless if the TEC is
enabled or not.
"""

from enum import IntEnum
from time import monotonic, time

import canopen
import cv2
import numpy as np
import tifffile
from olaf import Service, logger, new_oresat_file

from .. import __version__
from ..drivers.pirt1280 import Pirt1280, Pirt1280Error, pirt1280_raw_to_numpy


class CameraState(IntEnum):
    """All the states for CFC camera."""

    OFF = 0x1
    """Camera is off"""
    STANDBY = 0x2
    """Camera is on, but no doing anything"""
    CAPTURE = 0x3
    """Camera is capturing image and saving them to freac cache"""
    BOOT_LOCKOUT = 0x4
    """Camera is locked out until system is done booting and PRUs are available."""
    ERROR = 0xFF
    """Error with camera hardware"""


STATE_TRANSMISSIONS = {
    CameraState.OFF: [CameraState.OFF, CameraState.STANDBY],
    CameraState.STANDBY: [CameraState.OFF, CameraState.STANDBY, CameraState.CAPTURE],
    CameraState.CAPTURE: [
        CameraState.OFF,
        CameraState.STANDBY,
        CameraState.CAPTURE,
        CameraState.ERROR,
    ],
    CameraState.BOOT_LOCKOUT: [CameraState.OFF],
    CameraState.ERROR: [CameraState.OFF, CameraState.ERROR],
}
"""Valid state transistions."""


class CameraService(Service):
    """Service for camera and capture state machine"""

    _BOOT_LOCKOUT_S = 70

    def __init__(self, pirt1280: Pirt1280):
        super().__init__()

        self._state = CameraState.BOOT_LOCKOUT
        self._next_state_internal = -1
        self._next_state_user = -1

        self._pirt1280 = pirt1280

        self._capture_delay_obj: canopen.objectdictionary.Variable = None
        self._capture_count_obj: canopen.objectdictionary.Variable = None
        self._capture_save_obj: canopen.objectdictionary.Variable = None
        self._last_capture_obj: canopen.objectdictionary.Variable = None
        self._last_capture_time_obj: canopen.objectdictionary.Variable = None

        self._count = 0

    def on_start(self):
        rec = self.node.od["camera"]
        self._capture_delay_obj = rec["capture_delay"]
        self._capture_count_obj = rec["number_to_capture"]
        self._capture_save_obj = rec["save_captures"]
        self._capture_save_obj.value = True  # make sure this is True by default
        self._last_capture_obj = rec["last_capture"]
        self._last_capture_time_obj = rec["last_capture_time"]
        self._last_capture_time_obj.value = 0

        self.node.add_sdo_callbacks("camera", "status", self._on_read_status, self._on_write_status)
        self.node.add_sdo_callbacks(
            "camera",
            "integration_time",
            self._on_read_cam_integration,
            self._on_write_cam_integration,
        )
        self.node.add_sdo_callbacks("camera", "temperature", self._on_read_cam_temp, None)
        self.node.add_sdo_callbacks(
            "camera", "last_display_image", self._on_read_last_display_capture, None
        )
        self.node.add_sdo_callbacks("camera", "enabled", self._on_read_cam_enabled, None)

    def on_stop(self):
        self._pirt1280.disable()

    def _state_machine_transition(self, new_state: [CameraState, int]):
        """Transition from one state to another."""

        if new_state not in list(CameraState):
            logger.error(f"invalid new state {new_state}")
            return

        if isinstance(new_state, int):
            new_state = CameraState(new_state)

        if new_state not in STATE_TRANSMISSIONS[self._state]:
            logger.error(f"invalid state transistion {self._state.name} -> {new_state.name}")
            return

        try:
            if new_state in [CameraState.OFF, CameraState.ERROR]:
                self._pirt1280.disable()
            elif new_state == CameraState.STANDBY:
                self._pirt1280.enable()
            elif new_state == CameraState.CAPTURE:
                self._count = 0
        except Pirt1280Error as e:
            logger.exception(e)
            new_state = CameraState.ERROR

        logger.info(f"state transistion {self._state.name} -> {new_state.name}")

        self._state = new_state

    def on_loop(self):

        if self._state == CameraState.BOOT_LOCKOUT and monotonic() > self._BOOT_LOCKOUT_S:
            self._next_state_internal = CameraState.OFF.value

        if self._next_state_internal != -1:
            self._state_machine_transition(self._next_state_internal)
            self._next_state_internal = -1
        elif self._next_state_user != -1:
            self._state_machine_transition(self._next_state_user)
            self._next_state_user = -1

        if self._state in [
            CameraState.OFF,
            CameraState.STANDBY,
            CameraState.ERROR,
            CameraState.BOOT_LOCKOUT,
        ]:
            self.sleep(0.1)
        elif self._state == CameraState.CAPTURE:
            self._count += 1

            try:
                self._capture()
            except Pirt1280Error:
                self._next_state_internal = CameraState.ERROR.value
                return

            if self._capture_count_obj.value != 0 and self._count >= self._capture_count_obj.value:
                # that was the last capture in a sequence requested
                self._next_state_internal = CameraState.STANDBY.value
            else:  # no limit
                self.sleep(self._capture_delay_obj.value / 1000)
        else:
            logger.error(f"was in unknown state {self._state}, resetting to OFF")
            self._next_state_internal = CameraState.OFF.value

    def on_loop_error(self, error: Exception):
        logger.exception(error)
        self._state_machine_transition(CameraState.ERROR)

    def _capture(self, count: int = 1):
        """Capture x raw images in a row with no delay and save them to fread cache"""

        logger.info("capture")
        ts = time()
        self._last_capture_obj.value = self._pirt1280.capture()
        self._last_capture_time_obj.value = int(ts * 1000)

        if self._capture_save_obj.value:  # save captures
            metadata = {
                "sw_version": __version__,
                "time": ts,
                "temperature": self._pirt1280.temperature,
                "integration_time": self._pirt1280.integration_time,
            }

            file_name = "/tmp/" + new_oresat_file("capture", date=ts, ext=".tiff")
            data = pirt1280_raw_to_numpy(self._last_capture_obj.value)

            tifffile.imwrite(
                file_name,
                data,
                dtype=data.dtype,
                metadata=metadata,
                photometric="miniswhite",
            )

            self.node.fread_cache.add(file_name, consume=True)

    def _on_read_status(self) -> int:
        """SDO read callback for status"""

        return self._state.value

    def _on_write_status(self, value: int):
        """SDO write callback for status"""

        self._next_state_user = value

    def _on_read_cam_temp(self) -> int:
        """SDO read callback for camera temperature."""

        return int(self._pirt1280.temperature)

    def _on_read_cam_integration(self) -> int:
        """SDO read callback for camera integration time."""

        return self._pirt1280.integration_time

    def _on_write_cam_integration(self, value: int):
        """SDO write callback for camera integration time."""

        try:
            self._pirt1280.integration_time = value
        except ValueError as e:
            logger.error(e)

    def _on_read_last_display_capture(self) -> bytes:
        """SDO read callback for display image."""

        if not self._last_capture_obj.value:
            return b""
        return make_display_image(self._last_capture_obj.value, sat_percent=95, downscale_factor=2)

    def _on_read_cam_enabled(self) -> bool:
        """SDO read callback for camera enabled."""

        return self._pirt1280.is_enabled


def make_display_image(
    raw: bytes, ext: str = ".jpg", sat_percent: int = 0, downscale_factor: int = 1
) -> bytes:
    """
    Generate an image to send to UI (heavy manipulated image for UI display).

    Parameters
    ----------
    raw: bytes
        The raw PIRT1280 data.
    ext: str
        The extension of the file to generate.
    sat_percent: int
        Option to color pixel above a saturation percentage red. Set to 0 to disable.
    downscale_factor: int
        Downscale factor size in both row and columns. Set to 1 or less to not downscale.
    """

    data = pirt1280_raw_to_numpy(raw)

    # convert single pixel value int 3 values for BGR format (BGR values are all the same)
    tmp = np.zeros((data.shape[0], data.shape[1], 3), dtype=data.dtype)
    for i in range(3):
        tmp[:, :, i] = data[:, :]
    data = tmp

    # manipulate image for displaying
    data //= 64  # scale 14-bits to 8-bits
    data = data.astype(np.uint8)  # imencode wants uint8 or uint64
    data = np.invert(data)  # invert black/white values for displaying

    # downscale image
    if downscale_factor > 1:
        data = np.copy(data[::downscale_factor, ::downscale_factor])

    # color saturate pixel red
    if sat_percent > 0:
        sat_value = (255 * sat_percent) // 100
        sat_pixels = np.where(data[:, :] >= [sat_value, sat_value, sat_value])
        data[sat_pixels[0], sat_pixels[1]] = [0, 0, 255]  # red

    ok, encoded = cv2.imencode(ext, data)  # pylint: disable=E1101
    if not ok:
        raise ValueError(f"{ext} encode error")

    return bytes(encoded)
