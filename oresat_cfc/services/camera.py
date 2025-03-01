"""
Main camera service.

Seperate from the TEC controller service as the camera can be used regaurdless if the TEC is
enabled or not.
"""

import logging
import os
from threading import Event, Thread
from time import monotonic, time

import tifffile
import zmq
from oresat_libcanopend import NodeClient

from .. import __version__
from ..drivers.pirt1280 import Pirt1280, Pirt1280Error, pirt1280_raw_to_numpy
from ..gen.od import CameraStatus, CfcEntry

STATE_TRANSMISSIONS = {
    CameraStatus.OFF: [CameraStatus.OFF, CameraStatus.STANDBY],
    CameraStatus.STANDBY: [CameraStatus.OFF, CameraStatus.STANDBY, CameraStatus.CAPTURE],
    CameraStatus.CAPTURE: [
        CameraStatus.OFF,
        CameraStatus.STANDBY,
        CameraStatus.CAPTURE,
        CameraStatus.ERROR,
    ],
    CameraStatus.BOOT_LOCKOUT: [CameraStatus.OFF],
    CameraStatus.ERROR: [CameraStatus.OFF, CameraStatus.ERROR],
}


class CameraService:
    _BOOT_LOCKOUT_S = 70

    def __init__(self, node: NodeClient, pirt1280: Pirt1280):
        super().__init__()

        self._node = node
        self._pirt1280 = pirt1280

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.bind("tcp://*:7000")

        self._node.add_write_callback(CfcEntry.CAMERA_STATUS, self._set_state)

        self._state = CameraStatus.BOOT_LOCKOUT
        self._next_state_internal = -1
        self._next_state_user = -1
        self._count = 0

        self.last_capture = b""
        self.last_capture_time = 0

        self._thread = Thread(target=self._thread_run, daemon=True)
        self._event = Event()

    def _state_machine_transition(self, new_state: [CameraStatus, int]):
        if new_state not in list(CameraStatus) and new_state not in [
            s.value for s in list(CameraStatus)
        ]:
            logging.error(f"invalid new state {new_state}")
            return

        if isinstance(new_state, int):
            new_state = CameraStatus(new_state)

        if new_state not in STATE_TRANSMISSIONS[self._state]:
            logging.error(f"invalid state transistion {self._state.name} -> {new_state.name}")
            return

        try:
            if new_state in [CameraStatus.OFF, CameraStatus.ERROR]:
                self._pirt1280.disable()
            elif new_state == CameraStatus.STANDBY:
                self._pirt1280.enable()
            elif new_state == CameraStatus.CAPTURE:
                self._count = 0
        except Pirt1280Error as e:
            logging.exception(e)
            new_state = CameraStatus.ERROR

        if self._state != new_state:
            logging.info(f"state transistion {self._state.name} -> {new_state.name}")

        self._state = new_state

    def run(self, thread: bool):
        if thread:
            self._thread.start()
        else:
            self._thread_run()

    def _thread_run(self):
        while True:
            self._loop()

    def _loop(self):
        ts = monotonic()

        capture_count = self._node.od_read(CfcEntry.CAMERA_NUMBER_TO_CAPTURE)
        integration_time_us = self._node.od_read(CfcEntry.CAMERA_INTEGRATION_TIME)
        save_captures = self._node.od_read(CfcEntry.CAMERA_SAVE_CAPTURES)
        capture_delay_ms = self._node.od_read(CfcEntry.CAMERA_CAPTURE_DELAY)

        if self._state == CameraStatus.BOOT_LOCKOUT and monotonic() > self._BOOT_LOCKOUT_S:
            self._next_state_internal = CameraStatus.OFF.value

        if self._next_state_internal != -1:
            self._state_machine_transition(self._next_state_internal)
            self._next_state_internal = -1
        elif self._next_state_user != -1:
            self._state_machine_transition(self._next_state_user)
            self._next_state_user = -1

        delay = 0.1
        if self._state == CameraStatus.CAPTURE:
            self._count += 1

            try:
                self._pirt1280.integration_time = integration_time_us
                self.capture(save_captures)
            except Pirt1280Error:
                self._next_state_internal = CameraStatus.ERROR.value
                delay = 0
        elif self._state not in list(CameraStatus):
            logging.error(f"was in unknown state {self._state}, resetting to OFF")
            self._next_state_internal = CameraStatus.OFF.value
            delay = 0

        self._node.od_write_multi(
            {
                CfcEntry.CAMERA_STATUS: self._state,
                CfcEntry.CAMERA_TEMPERATURE: int(self._pirt1280.temperature),
            }
        )

        if self._state == CameraStatus.CAPTURE:
            if 0 < capture_count < self._count:
                # that was the last capture in a sequence requested
                self._next_state_internal = CameraStatus.STANDBY.value
            else:  # no limit
                delay = max((capture_delay_ms / 1000) - (monotonic() - ts), 0)

        if delay > 0:
            self._event.wait(delay)

    def capture(self, save: bool = True):
        logging.info("capture")
        ts = time()
        self.last_capture = self._pirt1280.capture()
        self.last_capture_time = int(ts * 1000)

        if save:
            metadata = {
                "sw_version": __version__,
                "time": ts,
                "temperature": self._pirt1280.temperature,
                "integration_time": self._pirt1280.integration_time,
            }

            file_name = f"/tmp/oresat-cfc_capture_{self.last_capture_time}.tiff"
            data = pirt1280_raw_to_numpy(self.last_capture)

            tifffile.imwrite(
                file_name,
                data,
                dtype=data.dtype,
                metadata=metadata,
                photometric="miniswhite",
            )

            try:
                self._node.add_file(file_name)
                os.remove(file_name)
            except Exception:
                logging.error("failed to add capture to cache")

        try:
            self._socket.send(self.last_capture)
        except Exception:
            logging.error("failed to stream capture")

    def _set_state(self, value: int):
        self._next_state_user = value
