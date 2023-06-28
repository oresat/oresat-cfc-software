from enum import IntEnum

import cv2
import numpy as np
from olaf import Resource, logger, TimerLoop, new_oresat_file

from .drivers.pirt1280 import Pirt1280, Pirt1280Error, pirt1280_raw_to_numpy
from .drivers.tec import Tec
from .tec_controller import TecController


class State(IntEnum):
    OFF = 0x0
    STANDBY = 0x1
    CAPTURE = 0x2
    SATURATED = 0x3
    ERROR = 0x4


class CFCResource(Resource):

    def __init__(self, camera: Pirt1280, tec: Tec):
        super().__init__()

        self._state = State.STANDBY

        self._cam = camera
        self._cam.integration_time = 0.05

        self._tec = tec
        self._tec_ctrl = TecController(self._tec)

        self.timer_loop = TimerLoop('cfc resource', self._state_loop, 1000)
        self.tec_timer_loop = TimerLoop('tec', self._tec_loop, 500,
                                        exc_func=self.on_tec_loop_error)
        self.test_camera_index = 0x7000

    def on_start(self):

        logger.info('enabling camera')
        self._cam.enable()

        self.node.add_sdo_read_callback(self.test_camera_index, self.on_test_camera_read)
        self.node.add_sdo_write_callback(self.test_camera_index, self.on_test_camera_write)

    def on_end(self):

        logger.info('disabling camera')
        self._cam.disable()

        self.tec_timer_loop.stop()

        logger.info('disabling TEC')
        self._tec.disable()

    def on_tec_loop_error(self, error: Exception):
        '''_tec_loop() raised an unexpected error'''

        logger.critical(f'disabling TEC due to unexpected error: {error}')
        self._tec.disable()

    def _state_loop(self) -> bool:
        '''Run the state machine loop'''

        if self._state == State.CAPTURE:
            self._capture_and_save()

        return True

    def _capture_and_save(self) -> bytes:
        '''Capture an raw image and save it'''

        data = self._cam.capture()

        # save capture
        name = '/tmp/' + new_oresat_file('capture', ext='.raw')
        with open(name, 'wb') as f:
            f.write(data)

        # add capture to fread cache
        self.fread_cache.add(name, consume=True)
        logger.info(f'saved new capture {name}')

        return data

    def _capture(self, ext: str = '.bmp', sat_percent: int = 90) -> bytes:
        '''Capture an image to send to UI (heavy manipulated image for UI display)'''

        raw = self._cam.capture()
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

        if sat_percent > 0:
            sat_value = (255 * sat_percent) // 100
            sat_pixels = np.where(data[:, :] >= [sat_value, sat_value, sat_value])
            data[sat_pixels[0], sat_pixels[1]] = [0, 0, 255]  # red

        ok, encoded = cv2.imencode(ext, data)
        if not ok:
            raise ValueError(f'{ext} encode error')

        return bytes(encoded)

    def on_test_camera_read(self, index: int, subindex: int):
        '''SDO read on the test camera obj'''

        if index == self.test_camera_index:
            if subindex == 0x1:
                return self._capture(ext='.jpg')
            elif subindex == 0x2:
                return self._cam.integration_time

    def on_test_camera_write(self, index: int, subindex: int, value):
        '''SDO write on the test camera obj'''

        if index == self.test_camera_index and subindex == 0x2:
            self._cam.integration_time = value

    def _tec_loop(self) -> True:
        '''Run the TEC loop'''

        self._tec_ctrl.loop()

        return True
