'''
Main camera service.

Seperate from the TEC controller service as the camera can be used regaurdless if the TEC is
enabled or not.
'''

from enum import IntEnum
from time import time

import cv2
import tifffile
import numpy as np
from olaf import Service, logger, new_oresat_file

from .. import __version__
from ..drivers.pirt1280 import Pirt1280, Pirt1280Error, pirt1280_raw_to_numpy


class CameraState(IntEnum):
    OFF = 0x1
    '''Camera is off'''
    STANDBY = 0x2
    '''Camera is on, but no doing anything'''
    CAPTURE = 0x3
    '''Camera is capturing image and saving them to freac cache'''
    ERROR = 0xFF
    '''Error with camera hardware'''


STATE_TRANSMISSIONS = {
    CameraState.OFF: [CameraState.OFF, CameraState.STANDBY],
    CameraState.STANDBY: [CameraState.OFF, CameraState.STANDBY, CameraState.CAPTURE],
    CameraState.CAPTURE: [CameraState.OFF, CameraState.STANDBY, CameraState.CAPTURE,
                          CameraState.ERROR],
    CameraState.ERROR: [CameraState.OFF, CameraState.ERROR],
}


class CameraService(Service):
    '''Service for camera and capture state machine'''

    def __init__(self, pirt1280: Pirt1280):
        super().__init__()

        self._state = CameraState.OFF
        self._last_capture = None
        self._next_state_internal = None
        self._next_state_user = None

        self._pirt1280 = pirt1280

        self.state_index = 0x6000
        self.camera_index = 0x6001
        self.test_camera_index = 0x7000

    def on_start(self):

        self._capture_delay_obj = self.node.od[self.state_index][2]
        self._capture_count_obj = self.node.od[self.state_index][3]

        self.node.add_sdo_read_callback(self.state_index, self.on_state_read)
        self.node.add_sdo_write_callback(self.state_index, self.on_state_write)

        self.node.add_sdo_read_callback(self.camera_index, self.on_camera_read)
        self.node.add_sdo_write_callback(self.camera_index, self.on_camera_write)

        self.node.add_sdo_read_callback(self.test_camera_index, self.on_test_camera_read)
        self.node.add_sdo_write_callback(self.test_camera_index, self.on_test_camera_write)

    def on_stop(self):

        self._pirt1280.disable()

    def _state_machine_transition(self, new_state: CameraState or int):
        '''Transition from one state to another.'''

        if new_state not in list(CameraState):
            logger.error(f'invalid new state {new_state}')
            return

        if isinstance(new_state, int):
            new_state = CameraState(new_state)

        if new_state not in STATE_TRANSMISSIONS[self._state]:
            logger.error(f'invalid state transistion {self._state.name} -> {new_state.name}')
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

        logger.info(f'state transistion {self._state.name} -> {new_state.name}')

        self._state = new_state

    def on_loop(self):

        if self._next_state_internal is not None:
            self._state_machine_transition(self._next_state_internal)
            self._next_state_internal = None
        elif self._next_state_user is not None:
            self._state_machine_transition(self._next_state_user)
            self._next_state_user = None

        if self._state in [CameraState.OFF, CameraState.STANDBY, CameraState.ERROR]:
            self.sleep(1)
        elif self._state == CameraState.CAPTURE:
            self._count += 1

            try:
                self._capture()
            except Pirt1280Error:
                self._next_state_internal = CameraState.ERROR
                return

            if self._capture_count_obj.value != 0 and \
                    self._count >= self._capture_count_obj.value:
                # that was the last capture in a sequence requested
                self._next_state_internal = CameraState.STANDBY
            else:  # no limit
                self.sleep(self._capture_delay_obj.value / 1000)
        else:
            logger.error(f'was in unknown state {self._state}, resetting to OFF')
            self._next_state_internal = CameraState.OFF

    def on_loop_error(self, error: Exception):

        logger.exception(error)
        self._state_machine_transition(CameraState.ERROR)

    def _capture(self, count: int = 1):
        '''Capture x raw images in a row with no delay and save them to fread cache'''

        logger.info('capture')
        dt = time()
        self._last_capture = self._pirt1280.capture()
        metadata = {
            'sw_version': __version__,
            'time': dt,
            'temperature': self._pirt1280.temperature,
            'integration_time': self._pirt1280.integration_time,
        }

        file_name = '/tmp/' + new_oresat_file('capture', date=dt, ext='.tiff')
        data = pirt1280_raw_to_numpy(self._last_capture)

        tifffile.imwrite(
            file_name,
            data,
            dtype=data.dtype,
            metadata=metadata,
            photometric='miniswhite',
        )

        self.node.fread_cache.add(file_name, consume=True)

    def on_state_read(self, index: int, subindex: int):
        '''SDO read on the state machine obj'''

        if index == self.state_index and subindex == 0x1:
            return self._state.value

    def on_state_write(self, index: int, subindex: int, value):
        '''SDO read on the state machine obj'''

        if index == self.state_index and subindex == 0x1:
            self._next_state_user = value

    def on_camera_read(self, index: int, subindex: int):
        '''SDO read on the camera obj'''

        if index != self.camera_index:
            return

        ret = None

        if subindex == 0x4:
            ret = self._pirt1280.is_enabled
        elif subindex == 0x5:
            ret = int(self._pirt1280.temperature)
        elif subindex == 0x7:
            ret = self._pirt1280.integration_time

        return ret

    def on_camera_write(self, index: int, subindex: int, value):
        '''SDO read on the camera obj'''

        if index == self.camera_index and subindex == 0x7:
            self._pirt1280.integration_time = value

    def on_test_camera_read(self, index: int, subindex: int):
        '''SDO read on the test camera obj'''

        if index != self.test_camera_index:
            return

        ret = None

        if subindex == 0x1:
            ret = self._last_capture
        elif subindex == 0x2 and self._last_capture is not None:
            ret = make_display_image(self._last_capture, sat_percent=95, downscale_factor=2)
        elif subindex == 0x3:
            ret = self._pirt1280.is_enabled

        return ret

    def on_test_camera_write(self, index: int, subindex: int, value):
        '''SDO write on the test camera obj'''

        if index != self.test_camera_index and subindex != 0x3:
            return

        if value:
            logger.info('enabling pirt1280')
            self.camera.enable()
        else:
            logger.info('disabling pirt1280')
            self.camera.disable()


def make_display_image(raw: bytes, ext: str = '.jpg', sat_percent: int = 0,
                       downscale_factor: int = 1) -> bytes:
    '''
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
    '''

    data = pirt1280_raw_to_numpy(raw)

    # convert single pixel value int 3 values for BGR format (BGR values are all the same)
    tmp = np.zeros((data.shape[0], data.shape[1], 3), dtype=data.dtype)
    for i in range(3):
        tmp[:, :, i] = data[:, :]
    data = tmp

    '''
    # flip the image because it is read upside down
    data = cv2.flip(data, 0)
    data = cv2.flip(data, 1)
    '''

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

    ok, encoded = cv2.imencode(ext, data)
    if not ok:
        raise ValueError(f'{ext} encode error')

    return bytes(encoded)
