from enum import IntEnum

import cv2
import numpy as np
from olaf import Resource, logger, TimerLoop, new_oresat_file

from .drivers.pirt1280 import PIRT1280
from .drivers.tec import TEC
from .tec_controller import TecController


class State(IntEnum):
    OFF = 0x0
    STANDBY = 0x1
    CAPTURE = 0x2
    SATURATED = 0x3
    ERROR = 0x4


class CFCResource(Resource):

    def __init__(self, camera: PIRT1280, tec: TEC):
        super().__init__()

        self._state = State.STANDBY

        self._cam = camera
        self._cam.set_integration(0.05)

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
            self._capture(save=True)

        return True

    def _capture(self, ext: str = '.bmp', save: bool = False) -> bytes:
        '''Capture an image to send to UI (heavy manipulated image for UI display)'''

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
        '''SDO read on the test camera obj'''

        if index == self.test_camera_index and subindex == 0x1:
            return self._capture(ext='.jpg')

    def _tec_loop(self) -> True:
        '''Run the TEC loop'''

        self._tec_ctrl.loop()

        return True
