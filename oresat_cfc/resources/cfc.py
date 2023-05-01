from enum import IntEnum

import cv2
import numpy as np
from olaf import Resource, logger, TimerLoop, new_oresat_file

from ..drivers.pirt1280 import PIRT1280


class State(IntEnum):
    OFF = 0x0
    STANDBY = 0x1
    CAPTURE = 0x2
    SATURATED = 0x3
    ERROR = 0x4


class CFCResource(Resource):

    def __init__(self, camera: PIRT1280):
        super().__init__()

        self._state = State.STANDBY

        self._cam = camera
        self._cam.set_integration(0.05)

        self.timer_loop = TimerLoop('cfc resource', self._loop, 1000)
        self.test_camera_index = 0x7000

    def on_start(self):

        logger.info('enabling camera')
        self._cam.enable()

        self.node.add_sdo_read_callback(self.test_camera_index, self.on_test_camera_read)

    def on_end(self):

        logger.info('disabling camera')
        self._cam.disable()

    def _loop(self) -> bool:

        if self._state == State.CAPTURE:
            self._capture(save=True)

        return True

    def _capture(self, ext: str = '.bmp', save: bool = False) -> bytes:

        data = self._cam.capture_as_np_array()

        # convert single pixel value int 3 values for bgr format (bgr value are all the same)
        tmp = np.zeros((data.shape[0], data.shape[1], 3), dtype=data.dtype)
        for i in range(3):
            tmp[:, :, i] = data[:, :]
        data = tmp

        # down scale data to uint8
        data //= 255
        data = data.astype(np.uint8)

        ok, encoded = cv2.imencode(ext, data)
        if not ok:
            raise ValueError(f'{ext} encode error')

        if save:
            # save capture
            name = '/tmp/' + new_oresat_file('capture', ext=ext)
            with open(name, 'wb') as f:
                f.write(encoded)

            # add capture to fread cache
            self.fread_cache.add(name, consume=True)
            logger.info(f'saved new capture {name}')

        return bytes(encoded)

    def on_test_camera_read(self, index: int, subindex: int):

        if index == self.test_camera_index and subindex == 0x1:
            return self._capture(ext='.jpg')
